import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial

class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        
        # Port série de l'Arduino Mega
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1.0)
        
        # Timer à 50 Hz pour scruter le port série
        self.timer = self.create_timer(0.02, self.read_serial_and_publish)
        self.get_logger().info('Nœud IMU démarré sur /dev/ttyUSB0')

    def read_serial_and_publish(self):
        if not self.serial_port.in_waiting:
            return

        try:
            line = self.serial_port.readline().decode('utf-8').strip()
            parts = line.split(',')

            if parts[0] == 'IMU' and len(parts) == 7:
                msg = Imu()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'imu_link'

                # Accélérations linéaires (m/s²)
                msg.linear_acceleration.x = float(parts[1])
                msg.linear_acceleration.y = float(parts[2])
                msg.linear_acceleration.z = float(parts[3])

                # Vitesses angulaires (rad/s)
                msg.angular_velocity.x = float(parts[4])
                msg.angular_velocity.y = float(parts[5])
                msg.angular_velocity.z = float(parts[6])

                # Orientation : désactivée ici (-1 dans la covariance si non fournie)
                msg.orientation_covariance[0] = -1.0

                self.publisher_.publish(msg)

        except Exception as e:
            self.get_logger().warn(f'Erreur de décodage série : {e}')

    def destroy_node(self):
        self.serial_port.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()