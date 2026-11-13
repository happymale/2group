"""ROS interfaces: exact-time transforms, fresh observations and braking."""
import json
import math
import time
import unittest
from types import SimpleNamespace

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32, Header, String
from hk_interfaces.msg import CtrlCmd

from hk_mit.avoidance import ReferencePath
from hk_mit.control import Control
from hk_mit.lidar_perception import LidarPerception
from hk_mit.planner import Planner


class AvoidanceNodeTests(unittest.TestCase):
    def setUp(self):
        remaps = []
        for topic in ('/hk/sensors/lidar/scan', '/hk/perception/obstacles', '/hk/perception/markers',
                      '/hk/local/odom', '/hk/plan/local_path', '/hk/plan/speed_limit', '/hk/plan/status',
                      '/hk/plan/avoidance_markers', '/hk/control/command', '/tf', '/tf_static', '/sim/events'):
            remaps.extend(['-r', f'{topic}:=/hk_mit_test{topic}'])
        rclpy.init(args=['--ros-args', '-p', 'record_csv:=false'] + remaps)
        self.nodes = []

    def tearDown(self):
        for node in reversed(self.nodes):
            node.destroy_node()
        rclpy.try_shutdown()

    def make(self, cls):
        node = cls()
        self.nodes.append(node)
        return node

    def scan(self, node, ranges=(2.0,)):
        m = LaserScan()
        m.header = Header(stamp=node.get_clock().now().to_msg(), frame_id='lidar_link')
        m.angle_increment = .01
        m.range_min, m.range_max = .5, 100.
        m.ranges = list(ranges)
        return m

    def add_transform(self, node):
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = 'map', 'lidar_link'
        t.transform.translation.x, t.transform.translation.y = 10., 20.
        t.transform.rotation.z, t.transform.rotation.w = math.sin(math.pi/4), math.cos(math.pi/4)
        node.tf_buffer.set_transform_static(t, 'test')

    def receive_cloud(self, node, scan):
        observer = rclpy.create_node('hk_mit_cloud_test_observer')
        self.nodes.append(observer)
        clouds = []
        observer.create_subscription(PointCloud2, '/hk/perception/obstacles', clouds.append, 1)
        deadline = time.monotonic() + 3
        while not clouds and time.monotonic() < deadline:
            scan.header.stamp = node.get_clock().now().to_msg()
            node.on_scan(scan)
            rclpy.spin_once(observer, timeout_sec=.05)
        self.assertTrue(clouds, 'No obstacle cloud was published')
        return clouds[-1]

    def test_perception_runs_with_csv_disabled_and_uses_map_transform(self):
        node = self.make(LidarPerception)
        self.add_transform(node)
        cloud = self.receive_cloud(node, self.scan(node))
        self.assertIsNone(node.csv_writer)
        self.assertEqual(cloud.header.frame_id, 'map')
        points = point_cloud2.read_points(cloud)
        self.assertAlmostEqual(float(points[0]['x']), 10., places=5)
        self.assertAlmostEqual(float(points[0]['y']), 22., places=5)
        self.assertIn('cluster_id', points.dtype.names)

    def test_no_return_is_fresh_empty_observation(self):
        node = self.make(LidarPerception)
        self.add_transform(node)
        cloud = self.receive_cloud(node, self.scan(node, (float('inf'),) * 10))
        self.assertEqual(cloud.width, 0)
        planner = self.make(Planner)
        planner.on_obstacles(cloud)
        self.assertEqual(planner.obstacles, [])

    def test_missing_transform_never_publishes_mislabeled_coordinates(self):
        node = self.make(LidarPerception)
        m = self.scan(node)
        node.on_scan(m)
        self.assertIs(node.pending_scan, m)
        self.add_transform(node)
        node.publish_pending_scan()
        self.assertIsNone(node.pending_scan)

    def test_delayed_dynamic_tf_does_not_lose_earlier_scan(self):
        node = self.make(LidarPerception)
        clouds = []
        node.obstacle_pub = SimpleNamespace(publish=clouds.append)
        now = node.get_clock().now().nanoseconds
        old = self.scan(node)
        new = self.scan(node)
        from rclpy.time import Time
        old.header.stamp = Time(nanoseconds=now - 150_000_000).to_msg()
        new.header.stamp = Time(nanoseconds=now - 50_000_000).to_msg()
        node.on_scan(old)
        node.on_scan(new)
        self.assertIs(node.pending_scan, old)
        for stamp in (now - 160_000_000, now - 140_000_000):
            transform = TransformStamped()
            transform.header = Header(stamp=Time(nanoseconds=stamp).to_msg(), frame_id='map')
            transform.child_frame_id = 'lidar_link'
            transform.transform.rotation.w = 1.
            node.tf_buffer.set_transform(transform, 'test')
        node.publish_pending_scan()
        self.assertEqual(len(clouds), 1)
        self.assertEqual(clouds[0].header.stamp, old.header.stamp)
        self.assertIs(node.pending_scan, new)
        node.publish_pending_scan()  # New scan still requires a future TF.
        self.assertEqual(len(clouds), 1)

    def test_waiting_scan_queue_discards_expired_data(self):
        node = self.make(LidarPerception)
        from rclpy.time import Time
        old = self.scan(node)
        old.header.stamp = Time(nanoseconds=node.get_clock().now().nanoseconds - 1_000_000_000).to_msg()
        node.on_scan(old)
        self.assertIsNone(node.pending_scan)
        self.add_transform(node)
        new = self.scan(node)
        node.on_scan(new)
        self.assertIsNone(node.pending_scan)

    def prepared_planner(self):
        node = self.make(Planner)
        node.route.points = [(float(x), 0.) for x in np.arange(-10, 81, .5)]
        node.reference = ReferencePath(node.route.points)
        node.reset_time = node.now() - 1
        node.active = True
        odom = Odometry()
        odom.header = Header(stamp=node.get_clock().now().to_msg(), frame_id='map')
        odom.pose.pose.orientation.w = 1.
        node.on_odom(odom)
        return node

    def cloud(self, node, points=()):
        fields = [PointField(name=name, offset=offset, datatype=kind, count=1) for name, offset, kind in (
            ('x', 0, PointField.FLOAT32), ('y', 4, PointField.FLOAT32),
            ('z', 8, PointField.FLOAT32), ('cluster_id', 12, PointField.UINT32))]
        return point_cloud2.create_cloud(Header(stamp=node.get_clock().now().to_msg(), frame_id='map'), fields, points)

    def test_planner_distinguishes_unavailable_from_empty_and_stops_when_stale(self):
        node = self.prepared_planner()
        node.plan()
        self.assertEqual(node.last_status, 'sensor_unavailable')
        node.on_obstacles(self.cloud(node))
        node.plan()
        self.assertEqual(node.last_status, 'following')
        self.assertIsNotNone(node.previous)
        node.observation_stamp = node.now() - 1
        node.plan()
        self.assertEqual(node.last_status, 'sensor_unavailable')

    def test_blocked_road_and_reset_clear_previous_state(self):
        node = self.prepared_planner()
        node.on_obstacles(self.cloud(node, [(15., -6., 0., 0), (15., 6., 0., 0)]))
        node.plan()
        self.assertEqual(node.last_status, 'no_safe_path')
        node.on_reset(0, False)
        self.assertIsNone(node.obstacles)
        self.assertIsNone(node.previous)
        self.assertIsNone(node.odom)

    def test_empty_path_and_zero_limit_brake_despite_positive_integral(self):
        node = self.make(Control)
        node.reset_control()
        node.speed_error_sum = 5.
        node.path = []
        self.assertEqual(node.drive(), (0., 1., 0.))
        node.path = [(0., 0., 0.), (10., 0., 0.)]
        node.on_speed_limit(Float32(data=0.))
        self.assertEqual(node.drive(), (0., 1., 0.))
        node.on_speed_limit(Float32(data=float('nan')))
        self.assertEqual(node.drive(), (0., 1., 0.))

    def test_control_brakes_if_plan_heartbeat_is_lost(self):
        node = self.make(Control)
        node.active, node.reset_time = True, node.now() - 1
        m = Odometry()
        m.header = Header(stamp=node.get_clock().now().to_msg(), frame_id='map')
        m.pose.pose.orientation.w = 1.
        node.on_odom(m)
        path = Path(header=m.header)
        node.on_path(path)
        node.on_speed_limit(Float32(data=3.))
        node.speed_limit_stamp = node.now() - 1
        sent = []
        node.send = lambda *values: sent.append(values)
        node.tick()
        self.assertEqual(sent[-1], (0., 1., 0.))

    def test_small_clock_delivery_skew_does_not_stop_planner(self):
        node = self.prepared_planner()
        from rclpy.time import Time
        future = Time(nanoseconds=node.get_clock().now().nanoseconds + 30_000_000).to_msg()
        node.odom.header.stamp = future
        cloud = self.cloud(node)
        cloud.header.stamp = future
        node.on_obstacles(cloud)
        node.plan()
        self.assertEqual(node.last_status, 'following')
        node.odom.header.stamp = Time(nanoseconds=node.get_clock().now().nanoseconds + 1_000_000_000).to_msg()
        node.plan()
        self.assertEqual(node.last_status, 'stale_odom')

    def test_control_allows_small_clock_skew_but_rejects_large_future_stamp(self):
        node = self.make(Control)
        node.active, node.reset_time = True, node.now() - 1
        node.reset_control()
        from rclpy.time import Time
        stamp = Time(nanoseconds=node.get_clock().now().nanoseconds + 30_000_000).to_msg()
        odom = Odometry(header=Header(stamp=stamp, frame_id='map'))
        odom.pose.pose.orientation.w = 1.
        node.on_odom(odom)
        pose = PoseStamped()
        pose.pose.position.x, pose.pose.orientation.w = 10., 1.
        node.on_path(Path(header=odom.header, poses=[pose]))
        node.on_speed_limit(Float32(data=3.))
        sent = []
        node.send = lambda *values: sent.append(values)
        node.tick()
        self.assertGreater(sent[-1][0], 0)
        self.assertEqual(sent[-1][1], 0)
        odom.header.stamp = Time(nanoseconds=node.get_clock().now().nanoseconds + 1_000_000_000).to_msg()
        node.tick()
        self.assertEqual(sent[-1], (0., 1., 0.))

    def test_published_scan_drives_avoidance_and_sensor_loss_brakes(self):
        perception = self.make(LidarPerception)
        planner = self.prepared_planner()
        control = self.make(Control)
        control.active, control.reset_time = True, control.now() - 1
        control.reset_control()
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = 'map', 'lidar_link'
        t.transform.translation.x, t.transform.rotation.w = 1.15, 1.
        perception.tf_buffer.set_transform_static(t, 'test')
        driver = rclpy.create_node('hk_mit_pipeline_test_driver')
        self.nodes.append(driver)
        from rclpy.qos import qos_profile_sensor_data
        scan_pub = driver.create_publisher(LaserScan, '/hk/sensors/lidar/scan', qos_profile_sensor_data)
        odom_pub = driver.create_publisher(Odometry, '/hk/local/odom', 10)
        states, commands, paths, limits = [], [], [], []
        driver.create_subscription(String, '/hk/plan/status', lambda m: states.append(json.loads(m.data)['state']), 1)
        driver.create_subscription(CtrlCmd, '/hk/control/command', commands.append, 1)
        driver.create_subscription(Path, '/hk/plan/local_path', paths.append, 1)
        driver.create_subscription(Float32, '/hk/plan/speed_limit', limits.append, 1)
        scan = self.scan(driver)
        scan.angle_min, scan.angle_increment = -math.pi, math.pi / 360
        angles = scan.angle_min + np.arange(720) * scan.angle_increment
        distances = np.full(720, np.inf)
        front = np.cos(angles) > .1
        distances[front] = (25. - 1.15) / np.cos(angles[front])
        hits = front & (np.abs(distances * np.sin(angles)) <= .5)
        distances[~hits] = np.inf
        scan.ranges = distances.tolist()

        def pump(duration, publish_scan):
            deadline, next_publish = time.monotonic() + duration, 0.
            while time.monotonic() < deadline:
                if time.monotonic() >= next_publish:
                    odom = Odometry()
                    odom.header = Header(stamp=driver.get_clock().now().to_msg(), frame_id='map')
                    odom.pose.pose.orientation.w = 1.
                    odom_pub.publish(odom)
                    if publish_scan:
                        scan.header.stamp = driver.get_clock().now().to_msg()
                        scan_pub.publish(scan)
                    next_publish = time.monotonic() + .08
                for node in (perception, planner, control, driver):
                    rclpy.spin_once(node, timeout_sec=.001)

        pump(1.2, True)
        self.assertIn('avoiding', states)
        self.assertTrue(any(p.poses for p in paths))
        self.assertTrue(any(0 < m.data <= planner.cfg.avoidance_speed for m in limits))
        self.assertTrue(any(m.accel > 0 and m.brake == 0 for m in commands))
        states.clear()
        pump(.9, False)
        self.assertIn('sensor_unavailable', states)
        self.assertEqual(len(paths[-1].poses), 0)
        self.assertEqual(limits[-1].data, 0.)
        self.assertEqual((commands[-1].accel, commands[-1].brake), (0., 1.))


if __name__ == '__main__':
    unittest.main()
