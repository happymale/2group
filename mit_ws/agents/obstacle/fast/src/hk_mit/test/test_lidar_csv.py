"""CSV 데이터 보존과 실제 ROS LaserScan 수신을 임시 디렉터리에서 검증합니다."""
import csv
import math
from pathlib import Path
import tempfile
import time
import unittest

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from hk_mit.lidar_perception import CSV_FIELDS, LidarPerception


class LidarCsvTests(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.path = Path(self.temp_dir.name) / 'nested' / 'scan.csv'
        self.nodes = []

    def tearDown(self):
        for node in reversed(self.nodes):
            node.destroy_node()
        rclpy.try_shutdown()
        self.temp_dir.cleanup()

    def start_recorder(self, record=True):
        rclpy.init(args=[
            '--ros-args', '-p', f'csv_path:={self.path}',
            '-p', f'record_csv:={str(record).lower()}',
            '-r', '/hk/sensors/lidar/scan:=/hk_mit_csv_test/scan',
            '-r', '/tf:=/hk_mit_csv_test/tf', '-r', '/tf_static:=/hk_mit_csv_test/tf_static',
            '-r', '/sim/events:=/hk_mit_csv_test/events',
            '-r', '/hk/perception/obstacles:=/hk_mit_csv_test/obstacles',
            '-r', '/hk/perception/markers:=/hk_mit_csv_test/markers',
        ])
        node = LidarPerception()
        self.nodes.append(node)
        return node

    def scan(self):
        m = LaserScan()
        m.header.stamp.sec, m.header.stamp.nanosec = 12, 345
        m.header.frame_id = 'lidar_link'
        m.angle_min, m.angle_increment = 0.0, math.pi / 2
        m.range_min, m.range_max = 0.5, 10.0
        m.time_increment, m.scan_time = 0.001, 0.1
        m.ranges = [1.0, 2.0, float('inf'), float('-inf'), float('nan'), 0.1, 11.0, 0.5, 10.0]
        return m

    def read_rows(self):
        with self.path.open(newline='', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            self.assertEqual(reader.fieldnames, list(CSV_FIELDS))
            return list(reader)

    def test_raw_values_coordinates_and_scan_grouping(self):
        node = self.start_recorder()
        m = self.scan()
        node.on_scan(m)
        m.intensities = [42.0] * len(m.ranges)
        node.on_scan(m)
        rows = self.read_rows()  # 파일이 열려 있어도 스캔별 flush로 읽을 수 있어야 합니다.
        self.assertEqual(len(rows), 18)
        self.assertEqual({r['scan_id'] for r in rows[:9]}, {'0'})
        self.assertEqual({r['scan_id'] for r in rows[9:]}, {'1'})
        self.assertEqual(rows[0]['stamp_sec'], '12')
        self.assertEqual(rows[0]['stamp_nanosec'], '345')
        self.assertEqual(rows[0]['frame_id'], 'lidar_link')
        self.assertAlmostEqual(float(rows[0]['x_lidar_m']), 1.0)
        self.assertAlmostEqual(float(rows[1]['y_lidar_m']), 2.0)
        self.assertEqual([r['valid'] for r in rows[:9]], ['1', '1', '0', '0', '0', '0', '0', '1', '1'])
        for row in rows[2:7]:
            self.assertEqual((row['x_lidar_m'], row['y_lidar_m']), ('', ''))
        self.assertEqual(float(rows[2]['range_m']), float('inf'))
        self.assertEqual(float(rows[3]['range_m']), float('-inf'))
        self.assertTrue(math.isnan(float(rows[4]['range_m'])))
        self.assertEqual(rows[0]['intensity'], '')
        self.assertEqual(float(rows[9]['intensity']), 42.0)
        self.assertAlmostEqual(float(rows[1]['beam_time_offset_s']), 0.001)

    def test_existing_file_is_preserved(self):
        self.path.parent.mkdir()
        self.path.write_text('existing data\n')
        with self.assertRaises(FileExistsError):
            self.start_recorder()
        self.assertEqual(self.path.read_text(), 'existing data\n')

    def test_disabled_recording_still_keeps_latest_scan(self):
        node = self.start_recorder(record=False)
        m = self.scan()
        node.on_scan(m)
        self.assertIs(node.scan, m)
        self.assertFalse(self.path.exists())

    def test_scan_is_received_over_ros_and_file_is_closed(self):
        node = self.start_recorder()
        source = rclpy.create_node('hk_mit_csv_test_source')
        self.nodes.append(source)
        pub = source.create_publisher(LaserScan, '/hk/sensors/lidar/scan', qos_profile_sensor_data)
        deadline = time.monotonic() + 5.0
        while node.scan is None and time.monotonic() < deadline:
            pub.publish(self.scan())
            rclpy.spin_once(node, timeout_sec=0.1)
        self.assertIsNotNone(node.scan, 'LaserScan이 ROS 구독 콜백에 도착하지 않았습니다.')
        self.assertGreaterEqual(len(self.read_rows()), 9)
        node.destroy_node()
        self.nodes.remove(node)
        self.assertTrue(node.csv_file.closed)


if __name__ == '__main__':
    unittest.main()
