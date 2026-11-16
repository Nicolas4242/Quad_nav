"""Passerelle unique entre les commandes ROS, l'IMU et le port Arduino."""

import math
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
from std_msgs.msg import String


def read_from_arduino(ser, node, stop_event):
    """Lire les lignes dans un thread, sans bloquer les commandes ROS."""
    pending = bytearray()
    discard_line = False
    while not stop_event.is_set() and rclpy.ok():
        try:
            # Une lecture peut finir sur timeout avant le saut de ligne.
            chunk = ser.read_until(b'\n', size=4096)
            if stop_event.is_set() or not rclpy.ok():
                break
            if not chunk:
                continue
            if not discard_line:
                pending.extend(chunk)
                if len(pending) > 4096:
                    pending.clear()
                    discard_line = True
                    node.get_logger().warning('Ligne série trop longue : ignorée.')
            if not chunk.endswith(b'\n'):
                continue
            if not discard_line:
                try:
                    line = pending.decode('utf-8').strip()
                except UnicodeDecodeError:
                    node.get_logger().warning('Ligne série non UTF-8 : ignorée.')
                else:
                    node.handle_serial_line(line)
            pending.clear()
            discard_line = False
        except Exception as error:
            if not stop_event.is_set() and rclpy.ok():
                node.get_logger().error(f'Lecture série interrompue : {error}')
            node.stop_requested = True
            break


class SerialBridge(Node):
    """Envoyer les commandes et publier l'IMU depuis la même liaison."""

    VALID_COMMANDS = {'start', 'stand', 'stop', 'status'}

    def __init__(self):
        super().__init__('serial_bridge')
        self.stop_requested = False
        self.ser = None
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('imu_frame_id', 'imu_link')
        # getVector(VECTOR_GYROSCOPE) du BNO055 renvoie des degrés/s.
        self.declare_parameter('gyro_in_degrees', True)
        self.imu_frame_id = self.get_parameter('imu_frame_id').value
        self.gyro_in_degrees = self.get_parameter('gyro_in_degrees').value
        self.imu_publisher = self.create_publisher(Imu, 'imu/data', 10)
        self.subscription = self.create_subscription(
            String, 'command', self.listener_callback, 10)

    def listener_callback(self, msg):
        """Conserver les commandes et le comportement de quit existants."""
        command = msg.data.strip().lower()
        if command != 'quit' and command not in self.VALID_COMMANDS:
            self.get_logger().warning(
                f'Commande refusée : "{command}". '
                'Commandes autorisées : start, stand, stop, status, quit.')
            return

        try:
            # quit reste local : l'Arduino reçoit stand, jamais quit.
            outgoing = 'stand' if command == 'quit' else command
            self.ser.write((outgoing + '\n').encode('utf-8'))
            if command == 'quit':
                self.ser.flush()
        except serial.SerialException as error:
            self.get_logger().error(
                f'Envoi de {outgoing} impossible : {error}. '
                'Fermeture non demandée par cette commande.')
            return

        if command == 'quit':
            self.get_logger().info('Commande stand envoyée : fermeture du bridge.')
            self.stop_requested = True
        else:
            self.get_logger().info(f'Commande envoyée : "{command}"')

    def handle_serial_line(self, line):
        """Diriger les mesures vers ROS et les autres lignes vers le journal."""
        if not line:
            return
        parts = line.split(',')
        if parts[0] != 'IMU':
            self.get_logger().info(f'Arduino says: {line}')
            return

        try:
            if len(parts) != 7:
                raise ValueError('six mesures attendues')
            values = [float(value) for value in parts[1:]]
            if not all(math.isfinite(value) for value in values):
                raise ValueError('mesure non finie')
        except ValueError as error:
            self.get_logger().warning(f'Trame IMU ignorée : {error}')
            return

        msg = Imu()
        # Horodatage à réception sur le Pi ; la trame n'a pas de temps Arduino.
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.imu_frame_id
        msg.linear_acceleration.x = values[0]
        msg.linear_acceleration.y = values[1]
        msg.linear_acceleration.z = values[2]
        scale = math.pi / 180.0 if self.gyro_in_degrees else 1.0
        msg.angular_velocity.x = values[3] * scale
        msg.angular_velocity.y = values[4] * scale
        msg.angular_velocity.z = values[5] * scale
        # Aucune orientation transmise ; covariances des mesures inconnues (0).
        msg.orientation_covariance[0] = -1.0
        self.imu_publisher.publish(msg)


def main(args=None):
    """Ouvrir une seule fois le port et arrêter le lecteur avant de fermer ROS."""
    rclpy.init(args=args)
    node = None
    ser = None
    reader_thread = None
    stop_event = threading.Event()

    try:
        node = SerialBridge()
        port = node.get_parameter('serial_port').value
        baud_rate = node.get_parameter('baud_rate').value
        try:
            # Le verrou exclusif évite deux instances du bridge sur le même port.
            ser = serial.Serial(
                port, baud_rate, timeout=1, write_timeout=1, exclusive=True)
        except serial.SerialException as error:
            node.get_logger().error(f'Ouverture de {port} impossible : {error}')
            return

        time.sleep(2)
        node.ser = ser
        reader_thread = threading.Thread(
            target=read_from_arduino, args=(ser, node, stop_event), daemon=True)
        reader_thread.start()
        node.get_logger().info(
            f'Arduino connecté sur {port}. Commandes : start, stand, stop, status, '
            'quit. IMU publiée sur imu/data.')
        while rclpy.ok() and not node.stop_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        stop_event.set()
        # timeout=1 borne l'attente de lecture ; plus de publication après join.
        if reader_thread is not None:
            reader_thread.join()
        try:
            if ser is not None:
                ser.close()
        finally:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
