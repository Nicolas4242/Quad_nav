"""Afficher le dernier scan LiDAR dans le navigateur, sans bureau sur le Pi."""

import math

import matplotlib
import matplotlib.pyplot as plt
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tornado.ioloop import IOLoop, PeriodicCallback


WEB_PORT = 8081
VIEW_RANGE = 5.0  # Vue de -5 à +5 mètres sur chaque axe.
REFRESH_MS = 100  # 100 ms = au maximum 10 affichages par seconde.


class LidarSubscriber(Node):
    """Recevoir /scan et remplacer les points du graphique à chaque relevé."""

    def __init__(self):
        super().__init__('lidar_subscriber')
        self.figure, self.axes = plt.subplots(figsize=(7, 7))
        self.points, = self.axes.plot([], [], '.', markersize=3)
        self.axes.plot(0, 0, 'r+', markersize=12, label='LiDAR')
        self.axes.set(
            xlim=(-VIEW_RANGE, VIEW_RANGE), ylim=(-VIEW_RANGE, VIEW_RANGE),
            xlabel='x du LiDAR (m)', ylabel='y du LiDAR (m)',
            title='En attente de /scan', aspect='equal',
        )
        self.axes.grid(True)
        self.axes.legend(loc='upper right')
        self.figure.tight_layout()

        # Une seule mesure en attente : les anciens scans ne s'accumulent pas.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, qos)

    def scan_callback(self, msg):
        """Convertir les distances et angles en coordonnées dans le repère LiDAR."""
        x, y = [], []
        for i, distance in enumerate(msg.ranges):
            if math.isfinite(distance) and msg.range_min <= distance <= msg.range_max:
                angle = msg.angle_min + i * msg.angle_increment
                x.append(distance * math.cos(angle))
                y.append(distance * math.sin(angle))

        # set_data remplace le relevé précédent, même si aucun point n'est valide.
        self.points.set_data(x, y)
        self.axes.set_title(
            f'Dernier relevé : {len(x)} points valides — {msg.header.frame_id}')
        self.figure.canvas.draw_idle()


def main(args=None):
    """Laisser WebAgg gérer le navigateur et traiter ROS sans attente bloquante."""
    matplotlib.use('WebAgg')
    matplotlib.rcParams.update({
        'webagg.address': '0.0.0.0',
        'webagg.port': WEB_PORT,
        'webagg.port_retries': 1,
        'webagg.open_in_browser': False,
    })
    rclpy.init(args=args)
    node = None
    timer = None
    try:
        node = LidarSubscriber()

        def update():
            if not rclpy.ok():
                IOLoop.current().stop()
                return
            try:
                rclpy.spin_once(node, timeout_sec=0.0)
            except ExternalShutdownException:
                IOLoop.current().stop()

        # La boucle web consulte ROS toutes les 100 ms, sans thread supplémentaire.
        timer = PeriodicCallback(update, REFRESH_MS)
        timer.start()
        node.get_logger().info(
            f'Graphique : http://IP_DU_PI:{WEB_PORT} — Ctrl+C pour arrêter.')
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        if timer is not None:
            timer.stop()
        plt.close('all')
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
