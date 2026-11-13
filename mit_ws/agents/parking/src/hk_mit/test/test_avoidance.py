"""Geometry scenarios for static-obstacle planning; no simulator required."""
import math
import unittest

import numpy as np

from hk_mit.avoidance import (
    AvoidanceConfig, Ego, ReferencePath, collision_and_clearance,
    footprint_corners, jump_distance_clusters, obstacle_geometry, plan_avoidance,
)


class AvoidanceTests(unittest.TestCase):
    def setUp(self):
        self.reference = ReferencePath([(float(x), 0.0) for x in np.arange(-10, 101, 0.5)])
        self.cfg = AvoidanceConfig()
        self.ego = Ego(0.0, 0.0, 0.0, 5.0)
        self.box = np.array([[25., -.5], [26., -.5], [26., .5], [25., .5]])

    def test_empty_scene_follows_reference(self):
        result = plan_avoidance(self.reference, self.ego, [], self.cfg)
        self.assertIsNotNone(result.selected)
        np.testing.assert_allclose(result.selected.d, 0, atol=1e-8)
        self.assertEqual(result.speed, self.cfg.cruise_speed)

    def test_box_is_avoided_with_full_vehicle_inside_corridor(self):
        result = plan_avoidance(self.reference, self.ego, [self.box], self.cfg)
        self.assertIsNotNone(result.selected)
        chosen = result.selected
        self.assertGreater(abs(chosen.offset), 1.0)
        np.testing.assert_allclose(chosen.xy[0], [self.ego.x, self.ego.y], atol=1e-6)
        self.assertAlmostEqual(chosen.yaw[0], self.ego.yaw)
        body = footprint_corners(chosen.xy, chosen.yaw, self.cfg, self.cfg.margin)
        self.assertLessEqual(float(np.max(abs(body[..., 1]))), self.cfg.road_half_width)
        collision, _ = collision_and_clearance(chosen.xy, chosen.yaw, obstacle_geometry([self.box]), self.cfg)
        self.assertFalse(collision)
        self.assertLessEqual(result.speed, self.cfg.avoidance_speed)
        self.assertTrue(any(c.reason == 'collision' for c in result.candidates))

    def test_wall_across_road_stops(self):
        wall = np.array([[15., -6.], [15., 6.]])
        result = plan_avoidance(self.reference, self.ego, [wall], self.cfg)
        self.assertIsNone(result.selected)
        self.assertEqual(result.speed, 0)

    def test_replanning_keeps_side_and_returns_after_passing(self):
        ego, previous, chosen_side = self.ego, None, None
        for _ in range(24):
            result = plan_avoidance(self.reference, ego, [self.box], self.cfg, previous)
            self.assertIsNotNone(result.selected, f'Lost feasible maneuver at x={ego.x}')
            selected = result.selected
            if ego.x < 26:
                side = math.copysign(1, selected.offset) if selected.offset else chosen_side
                chosen_side = side if chosen_side is None else chosen_side
                self.assertEqual(side, chosen_side)
            previous = selected
            index = min(10, len(selected.xy) - 1)  # advance approximately 2 m
            x, y = selected.xy[index]
            ego = Ego(float(x), float(y), float(selected.yaw[index]), result.speed)
        self.assertGreater(ego.x, 45)
        self.assertLess(abs(ego.y), 0.15)

    def test_pose_heading_and_curve_are_respected(self):
        points = [(35 * math.sin(a), 35 * (1 - math.cos(a))) for a in np.linspace(-.2, 1.3, 180)]
        reference = ReferencePath(points)
        result = plan_avoidance(reference, Ego(0, 0.4, .08, 3), [], self.cfg)
        self.assertIsNotNone(result.selected)
        np.testing.assert_allclose(result.selected.xy[0], [0, .4], atol=1e-4)
        self.assertAlmostEqual(result.selected.yaw[0], .08, places=6)

    def test_rotated_vehicle_and_contour_between_hits_collide(self):
        cfg = self.cfg
        # A long edge crosses the front of the body; neither endpoint is inside.
        geometry = obstacle_geometry([np.array([[2., -10.], [2., 10.]])])
        collision, _ = collision_and_clearance(np.array([[0., 0.], [.2, 0.]]), np.zeros(2), geometry, cfg)
        self.assertTrue(collision)
        geometry = obstacle_geometry([np.array([[-10., 2.], [10., 2.]])])
        collision, _ = collision_and_clearance(np.array([[0., 0.], [0., .2]]), np.full(2, math.pi/2), geometry, cfg)
        self.assertTrue(collision)

    def test_vehicle_inside_obstacle_hull_collides(self):
        geometry = obstacle_geometry([np.array([[-3., -3.], [4., -3.], [4., 3.], [-3., 3.]])])
        collision, _ = collision_and_clearance(np.array([[0., 0.], [.2, 0.]]), np.zeros(2), geometry, self.cfg)
        self.assertTrue(collision)

    def test_invalid_ranges_break_clusters_and_walls_are_kept(self):
        clusters = jump_distance_clusters([2., 2., float('nan'), 2., float('inf'), float('-inf')],
                                         0, .01, .5, 100)
        self.assertEqual([len(c) for c in clusters], [2, 1, 1])
        self.assertAlmostEqual(np.linalg.norm(clusters[-1][0]), .5)
        self.assertEqual(jump_distance_clusters([float('inf')] * 720, -math.pi, math.pi/360, .5, 100), [])

    def test_full_circle_seam_merges_only_adjacent_hits(self):
        ranges = [float('inf')] * 720
        ranges[0] = ranges[-1] = 3.0
        clusters = jump_distance_clusters(ranges, -math.pi, math.pi/360, .5, 100)
        self.assertEqual(len(clusters), 1)
        self.assertEqual(len(clusters[0]), 2)

    def test_long_curved_wall_does_not_fill_its_interior(self):
        wall = np.column_stack((12 * np.cos(np.linspace(0, math.pi, 60)),
                                12 * np.sin(np.linspace(0, math.pi, 60))))
        geometry = obstacle_geometry([wall])
        self.assertEqual(len(geometry[2]), 0)
        collision, _ = collision_and_clearance(np.array([[0., 0.], [.2, 0.]]), np.zeros(2), geometry, self.cfg)
        self.assertFalse(collision)


if __name__ == '__main__':
    unittest.main()
