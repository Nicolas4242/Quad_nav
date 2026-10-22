import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )
        self.get_logger().info('Nœud récepteur LiDAR démarré, écoute de /scan...')

    def scan_callback(self, msg: LaserScan):
        # Filtrer les mesures valides (ignorer les inf, nan et distances hors limites)
        valid_ranges = [
            (i, r) for i, r in enumerate(msg.ranges)
            if msg.range_min <= r <= msg.range_max and not math.isinf(r) and not math.isnan(r)
        ]

        if not valid_ranges:
            self.get_logger().warn('Aucun obstacle détecté dans la portée utile.')
            return

        # Obstacle le plus proche sur 360°
        closest_idx, min_dist = min(valid_ranges, key=lambda item: item[1])
        closest_angle_deg = math.degrees(msg.angle_min + closest_idx * msg.angle_increment)

        # Extraction d'un cône frontal de ±15° devant le robot
        front_distances = []
        for i, r in valid_ranges:
            angle_rad = msg.angle_min + i * msg.angle_increment
            if abs(angle_rad) <= math.radians(15):
                front_distances.append(r)

        front_dist_str = f"{min(front_distances):.2f} m" if front_distances else "dégagé"

        self.get_logger().info(
            f"Obstacle le plus proche : {min_dist:.2f} m à {closest_angle_deg:.1f}° | "
            f"Face avant : {front_dist_str}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = LidarSubscriber()
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