"""Vérifier la passerelle sans matériel ni installation ROS sur le poste."""

import importlib.util
import math
from pathlib import Path
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch


class FakeNode:
    """Fournir uniquement l'API ROS nécessaire aux tests de logique."""

    def __init__(self, name):
        self.name = name
        self.parameters = {}
        self.logger = Mock()

    def declare_parameter(self, name, value):
        self.parameters[name] = value

    def get_parameter(self, name):
        return SimpleNamespace(value=self.parameters[name])

    def create_publisher(self, message_type, topic, depth):
        return Mock(topic=topic)

    def create_subscription(self, message_type, topic, callback, depth):
        return SimpleNamespace(topic=topic, callback=callback)

    def get_logger(self):
        return self.logger

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: 'stamp'))

    def destroy_node(self):
        pass


def make_imu():
    return SimpleNamespace(
        header=SimpleNamespace(stamp=None, frame_id=''),
        linear_acceleration=SimpleNamespace(x=0.0, y=0.0, z=0.0),
        angular_velocity=SimpleNamespace(x=0.0, y=0.0, z=0.0),
        orientation_covariance=[0.0] * 9)


def load_bridge():
    """Isoler les doubles de test pour ne pas remplacer ROS dans les autres tests."""
    modules = {
        'rclpy': SimpleNamespace(ok=Mock(return_value=True)),
        'rclpy.node': SimpleNamespace(Node=FakeNode),
        'rclpy.executors': SimpleNamespace(ExternalShutdownException=RuntimeError),
        'sensor_msgs': SimpleNamespace(),
        'sensor_msgs.msg': SimpleNamespace(Imu=make_imu),
        'std_msgs': SimpleNamespace(),
        'std_msgs.msg': SimpleNamespace(String=SimpleNamespace),
        'serial': SimpleNamespace(SerialException=OSError),
    }
    path = Path(__file__).resolve().parents[1] / 'quadruped_control/serial_bridge.py'
    spec = importlib.util.spec_from_file_location('_bridge_under_test', path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict('sys.modules', modules):
        spec.loader.exec_module(module)
    return module


bridge = load_bridge()


class SerialBridgeTests(unittest.TestCase):
    """Couvrir les trames mixtes, les commandes et la fermeture du lecteur."""

    def setUp(self):
        self.node = bridge.SerialBridge()
        self.node.ser = Mock()

    def feed(self, chunks, fail=False):
        stop_event = threading.Event()
        values = iter(chunks)

        def read(*args, **kwargs):
            try:
                return next(values)
            except StopIteration:
                if fail:
                    raise OSError('USB débranché')
                stop_event.set()
                return b''

        self.node.ser.read_until.side_effect = read
        bridge.read_from_arduino(self.node.ser, self.node, stop_event)

    def test_interleaved_status_and_fragmented_imu(self):
        self.feed([
            b'STATE,STANDING\n', b'IMU,1,2,', b'',
            b'9.81,180,-90,0\r\n', b'STATUS,ready=1\n'])
        self.node.imu_publisher.publish.assert_called_once()
        msg = self.node.imu_publisher.publish.call_args.args[0]
        self.assertEqual(msg.header.frame_id, 'imu_link')
        self.assertEqual(msg.header.stamp, 'stamp')
        self.assertEqual(msg.linear_acceleration.z, 9.81)
        self.assertAlmostEqual(msg.angular_velocity.x, math.pi)
        self.assertAlmostEqual(msg.angular_velocity.y, -math.pi / 2)
        self.assertEqual(msg.orientation_covariance[0], -1.0)
        self.assertEqual(self.node.logger.info.call_count, 2)

    def test_radians_can_be_received_without_conversion(self):
        self.node.gyro_in_degrees = False
        self.node.handle_serial_line('IMU,0,0,9.81,1.5,-2,0')
        msg = self.node.imu_publisher.publish.call_args.args[0]
        self.assertEqual(msg.angular_velocity.x, 1.5)
        self.assertEqual(msg.angular_velocity.y, -2.0)

    def test_invalid_frames_do_not_prevent_next_measurement(self):
        self.feed([
            b'IMU,1,2\n', b'IMU,no,0,0,0,0,0\n',
            b'IMU,nan,0,0,0,0,0\n', b'IMU,0,0,0,inf,0,0\n',
            b'IMU,0,0,0,0,0,\xff\n', b'IMU,0,0,9.81,0,0,0\n'])
        self.assertEqual(self.node.logger.warning.call_count, 5)
        self.node.imu_publisher.publish.assert_called_once()

    def test_oversized_line_is_discarded_until_newline(self):
        self.feed([
            b'x' * 4096, b'x', b'IMU,0,0,1,0,0,0\n',
            b'IMU,0,0,2,0,0,0\n'])
        self.node.imu_publisher.publish.assert_called_once()
        msg = self.node.imu_publisher.publish.call_args.args[0]
        self.assertEqual(msg.linear_acceleration.z, 2.0)

    def test_commands_and_invalid_command(self):
        for command in [' START ', 'stand', 'stop', 'status']:
            self.node.listener_callback(SimpleNamespace(data=command))
        self.node.listener_callback(SimpleNamespace(data='start\nstop'))
        self.assertEqual(
            [call.args[0] for call in self.node.ser.write.call_args_list],
            [b'start\n', b'stand\n', b'stop\n', b'status\n'])
        self.assertFalse(self.node.stop_requested)
        self.node.ser.flush.assert_not_called()

    def test_quit_sends_stand_then_flush_before_requesting_exit(self):
        self.node.ser.flush.side_effect = lambda: self.assertFalse(
            self.node.stop_requested)
        self.node.listener_callback(SimpleNamespace(data=' QUIT '))
        self.node.ser.write.assert_called_once_with(b'stand\n')
        self.node.ser.flush.assert_called_once()
        self.assertTrue(self.node.stop_requested)

    def test_quit_write_or_flush_failure_does_not_request_exit(self):
        for operation in ['write', 'flush']:
            with self.subTest(operation=operation):
                self.node.ser = Mock()
                getattr(self.node.ser, operation).side_effect = OSError('failed')
                self.node.listener_callback(SimpleNamespace(data='quit'))
                self.assertFalse(self.node.stop_requested)

    def test_disconnect_requests_exit(self):
        self.feed([], fail=True)
        self.assertTrue(self.node.stop_requested)
        self.node.logger.error.assert_called_once()

    def test_shutdown_waits_for_reader_before_close_and_destroy(self):
        events = []
        node = self.node
        port = node.ser
        port.close.side_effect = lambda: events.append('close')
        node.destroy_node = lambda: events.append('destroy')
        reader = Mock()
        reader.join.side_effect = lambda: events.append('join')
        fake_ros = SimpleNamespace(
            init=Mock(), ok=Mock(return_value=True),
            spin_once=Mock(side_effect=lambda *a, **k: node.listener_callback(
                SimpleNamespace(data='quit'))),
            shutdown=lambda: events.append('shutdown'))
        with patch.object(bridge, 'rclpy', fake_ros), \
                patch.object(bridge, 'SerialBridge', return_value=node), \
                patch.object(bridge.serial, 'Serial', create=True,
                             return_value=port) as open_port, \
                patch.object(bridge.time, 'sleep'), \
                patch.object(bridge.threading, 'Thread', return_value=reader):
            bridge.main()
        open_port.assert_called_once_with(
            '/dev/ttyUSB0', 115200, timeout=1, write_timeout=1, exclusive=True)
        self.assertEqual(events, ['join', 'close', 'destroy', 'shutdown'])


if __name__ == '__main__':
    unittest.main()
