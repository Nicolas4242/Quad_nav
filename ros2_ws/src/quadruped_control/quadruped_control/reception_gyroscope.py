import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuSubscriber(Node):
    def __init__(self):
        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(
            Imu,
            'imu/data',
            self.listener_callback,
            10
        )
        self.get_logger().info('Nœud récepteur démarré, en attente de données...')

    def listener_callback(self, msg):
        acc = msg.linear_acceleration
        gyro = msg.angular_velocity
        self.get_logger().info(
            f"Acc: [{acc.x:.2f}, {acc.y:.2f}, {acc.z:.2f}] m/s² | "
            f"Gyro: [{gyro.x:.2f}, {gyro.y:.2f}, {gyro.z:.2f}] rad/s"
        )

def main(args=None):
    rclpy.init(args=args)
    node = ImuSubscriber()
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