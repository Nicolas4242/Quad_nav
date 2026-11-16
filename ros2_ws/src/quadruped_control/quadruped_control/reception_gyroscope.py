"""Lire /imu/data et servir des courbes dessinées par le navigateur."""
import math
from pathlib import Path
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from tornado.ioloop import IOLoop, PeriodicCallback
from tornado.web import Application, RequestHandler

WEB_PORT = 8082
REFRESH_MS = 100


class ImuSubscriber(Node):
    def __init__(self):
        super().__init__('imu_subscriber')
        self.latest = None
        self.received_at = 0.0
        self.sequence = 0
        self.subscription = self.create_subscription(
            Imu, 'imu/data', self.listener_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

    def listener_callback(self, msg):
        acc, gyro = msg.linear_acceleration, msg.angular_velocity
        values = [acc.x, acc.y, acc.z, gyro.x, gyro.y, gyro.z]
        if not all(math.isfinite(value) for value in values):
            return
        self.sequence += 1
        self.received_at = time.monotonic()
        self.latest = {'sequence': self.sequence, 'acc': values[:3],
                       'gyro': values[3:], 'frame': msg.header.frame_id}


class Page(RequestHandler):
    def get(self):
        self.set_header('Content-Type', 'text/html; charset=utf-8')
        self.write(Path(__file__).with_name('imu.html').read_text(encoding='utf-8'))


class Data(RequestHandler):
    def initialize(self, node):
        self.node = node

    def get(self):
        self.set_header('Cache-Control', 'no-store')
        age = time.monotonic() - self.node.received_at
        self.write({'sample': self.node.latest, 'fresh': self.node.latest is not None and age < 1.0})


def main(args=None):
    rclpy.init(args=args)
    node = ImuSubscriber()
    loop = IOLoop.current()
    server = None

    def update():
        if not rclpy.ok():
            loop.stop()
            return
        try:
            rclpy.spin_once(node, timeout_sec=0.0)
        except ExternalShutdownException:
            loop.stop()

    timer = PeriodicCallback(update, REFRESH_MS)
    try:
        server = Application([(r'/', Page), (r'/data', Data, {'node': node})]).listen(WEB_PORT)
        timer.start()
        node.get_logger().info(f'IMU : http://IP_DU_PI:{WEB_PORT} - Ctrl+C pour arrêter.')
        loop.start()
    except KeyboardInterrupt:
        pass
    finally:
        timer.stop()
        if server is not None:
            server.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
