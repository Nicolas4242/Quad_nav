import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import serial
import threading
import time

SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200

# Lit et affiche les réponses Arduino dans un thread séparé.
def read_from_arduino(ser):
    while True:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print(f'\nArduino says: {line}')
        except Exception as e:
            print(f'\nRead error: {e}')
            break


class CommandReceiver(Node):
    VALID_COMMANDS = {'start', 'stand', 'stop', 'status'}

    def __init__(self):
        super().__init__('command_receiver')
        self.stop_requested = False
        # Chaque message reçu sur command déclenche listener_callback.
        self.subscription = self.create_subscription(
            String, 'command', self.listener_callback, 10)

    def listener_callback(self, msg):
        command = msg.data.strip().lower()

        # Demande la position debout avant de fermer ; ne transmet pas quit.
        if command == 'quit':
            try:
                self.ser.write(b'stand\n')
                # Attend la fin de l'écriture série avant de fermer le port.
                self.ser.flush()
            except serial.SerialException as e:
                self.get_logger().error(
                    f'Envoi de stand impossible : {e}. Récepteur maintenu ouvert.')
                return
            self.get_logger().info('Commande stand envoyée : fermeture du récepteur.')
            self.stop_requested = True
            return

        # Vérifie la commande avant de la transmettre à l'Arduino.
        if command not in self.VALID_COMMANDS:
            self.get_logger().warning(
                f'Commande refusée : "{command}". '
                'Commandes autorisées : start, stand, stop, status, quit.')
            return

        self.get_logger().info(f'Commande acceptée : "{command}"')

        # Envoie le texte en série ; le saut de ligne termine la commande.
        self.ser.write((command + '\n').encode('utf-8'))


def main(args=None):
    rclpy.init(args=args)
    node = None

    ser = None

    try:
        node = CommandReceiver()

        try:
            # Ouvre le port ; une lecture attend au maximum une seconde.
            ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        except Exception as e:
            print('Could not open serial port.')
            print(e)
            print('Check if CP2102 is connected and if the port is /dev/ttyUSB0.')
            return
        
        time.sleep(2)
        # Partage le même port entre la callback et le thread de lecture.
        node.ser = ser
        print('Connected to Arduino through CP2102.')
        
        # La lecture série avance indépendamment de la réception ROS.
        reader_thread = threading.Thread(
            target=read_from_arduino, args=(ser,), daemon=True)
        reader_thread.start()
        
        print('Récepteur prêt : start, stand, stop, status, quit. Ctrl+C pour quitter.')
        # Traite les messages jusqu'à quit, puis passe au nettoyage du finally.
        while rclpy.ok() and not node.stop_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        # Ferme le port et libère ROS à la sortie du programme.
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
