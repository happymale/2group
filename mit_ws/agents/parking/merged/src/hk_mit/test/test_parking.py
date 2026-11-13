"""주차 계산(parking.py) 시험: 합성 벽 점으로 칸 검출, 후보 동작의 칸 안 최종 자세·자유 공간, 경로 끼우기."""
import math
import unittest

import numpy as np

from hk_mit.parking import (FORWARD, REVERSE, Mission, ParkingConfig, SlotFrame, detect_slot, free_space_ok,
                            in_slot, parallel_exit, plan_maneuver, splice_route)

ROAD_X = -13.1647
PPARK = {'id': 'm_ppark', 'type': 'parallel_parking',
         'slot': {'center': [-18.1147, 120.7754], 'yaw': -math.pi / 2, 'length': 7.4, 'width': 3.6},
         'zone': [[-15.6647, 113.2754], [-15.6647, 128.2754], [-20.6647, 128.2754], [-20.6647, 113.2754]],
         'slide': {'axis': [0.0, 1.0], 'range': 3.0}}
RPARK = {'id': 'm_rpark', 'type': 'rear_parking',
         'slot': {'center': [0.3353, 82.9754], 'yaw': -math.pi / 2, 'length': 5.4, 'width': 3.6},
         'zone': [[3.3353, 63.2754], [3.3353, 89.2754], [-10.6647, 89.2754], [-10.6647, 63.2754]],
         'slide': {'axis': [0.0, 1.0], 'range': 3.0}}
PARAMS = {
    'parallel_parking': dict(radii=[4.43], final_lateral=[0.6, 0.5], final_back=[1.1, 1.0], start_lateral=[0.0],
                             shift_radius=12.0, straight_before_stop=0.0, exit_radius=4.43, hold_s=3.2,
                             cost_per_radius=0.0),
    'rear_parking': dict(entry_radii=[5.0], turn_radii=[4.7], final_back=[0.05], passage_offsets=[-0.5, -0.75],
                         final_lateral=-0.2, straight_before=0.0, straight_after_turn=0.0, exit_turn_radius=4.43,
                         exit_radius=4.43, exit_passage_offset=-0.25, hold_s=3.2),
}
FREE = {'parallel_parking': [(-4.0, 4.0, -2.05, 2.5)],
        'rear_parking': [(4.2, 9.2, -11.0, -2.0), (-2.8, 16.2, -2.475, 2.525)]}


def wall_points(side_x, y0, y1, gap, face_depth):
    """도로 옆 벽 선(x = side_x)을 y0~y1에 0.2 m 간격으로, gap=(남쪽 끝, 북쪽 끝) 구간은 비우고, 남쪽 끝에는
    입구 안쪽 면(도로에서 멀어지는 방향으로 face_depth) 점을 둠."""
    ys = np.arange(y0, y1, 0.2)
    ys = ys[(ys < gap[0]) | (ys > gap[1])]
    pts = [np.column_stack((np.full_like(ys, side_x), ys))]
    xs = side_x + np.sign(side_x - ROAD_X) * np.arange(0.0, face_depth, 0.2)
    pts.append(np.column_stack((xs, np.full_like(xs, gap[0]))))
    return np.vstack(pts)


def route():
    ys = np.arange(160.0, 30.0, -0.5)
    return np.column_stack((np.full_like(ys, ROAD_X), ys))


class ParkingTests(unittest.TestCase):
    def setUp(self):
        self.cfg = ParkingConfig(radius_margin=1.05, clearance=0.3, slot_margin=0.15)

    def test_detects_slide_from_wall_opening(self):
        mission = Mission.from_json(PPARK)
        for slide in (-2.5, 0.0, 1.8):
            c = 120.7754 + slide
            pts = wall_points(-16.165, 95.0, 150.0, (c - 4.0, c + 4.0), 4.0)
            det = detect_slot(mission, pts, 4.95, self.cfg, 8.0)
            self.assertIsNotNone(det)
            self.assertAlmostEqual(det['slide'], slide, delta=0.15)

    def test_rear_detection_uses_passage_and_zone_offset(self):
        mission = Mission.from_json(RPARK)
        slide = -1.7
        c = 76.2754 + slide                               # 통로 중심 = 존 중심 + slide
        pts = wall_points(-10.165, 50.0, 100.0, (c - 2.5, c + 2.5), 8.0)
        det = detect_slot(mission, pts, -13.5, self.cfg, 5.0)
        self.assertIsNotNone(det)
        self.assertAlmostEqual(det['slide'], slide, delta=0.15)
        self.assertAlmostEqual(det['slot_center'][1], 82.9754 + slide, delta=0.15)

    def test_maneuvers_end_in_slot_and_stay_in_free_space(self):
        for raw, y_road in ((PPARK, 4.95), (RPARK, -13.5)):
            mission = Mission.from_json(raw)
            det = {'slide': 0.0, 'slot_center': mission.slot_center, 'wall': 3.0}
            maneuver = plan_maneuver(mission, det, y_road, self.cfg, PARAMS[mission.type], FREE[mission.type])
            self.assertIsNotNone(maneuver, mission.type)
            self.assertEqual([s.gear for s in maneuver.segments], [FORWARD, REVERSE, FORWARD])
            frame = SlotFrame(mission.slot_center, mission.slot_yaw)
            final = maneuver.segments[1].poses[-1]
            local = np.r_[frame.to_local(final[:2]), final[2] - mission.slot_yaw]
            self.assertTrue(in_slot(local, mission.slot_length, mission.slot_width, 0.05, self.cfg))
            # 곡률 ≤ 1/R_min (원호 사이 정지 지점 제외)
            for s in maneuver.segments:
                if len(s.poses) < 2:
                    continue
                d = np.linalg.norm(np.diff(s.poses[:, :2], axis=0), axis=1)
                k = np.abs(np.diff(np.unwrap(s.poses[:, 2]))) / np.maximum(d, 1e-6)
                self.assertLessEqual(float(np.max(k)), 1.0 / self.cfg.min_radius + 1e-3)

    def test_splice_keeps_prefix_and_segments_do_not_overlap(self):
        mission = Mission.from_json(PPARK)
        det = {'slide': 1.0, 'slot_center': mission.slot_center + [0.0, 1.0], 'wall': 3.0}
        maneuver = plan_maneuver(mission, det, 4.95, self.cfg, PARAMS[mission.type], FREE[mission.type])
        r = route()
        sp = splice_route(r, maneuver, mission, 3.0)
        self.assertIsNotNone(sp)
        np.testing.assert_allclose(sp.points[:sp.first], r[:sp.first])
        ends = [s[1] for s in sp.segments]
        starts = [s[0] for s in sp.segments]
        self.assertEqual(starts[1:], [e + 1 for e in ends[:-1]])
        self.assertTrue(np.all(sp.gears[starts[1]:ends[1] + 1] == REVERSE))
        self.assertGreater(float(np.min(np.linalg.norm(np.diff(sp.points, axis=0), axis=1))), 1e-3)

    def test_parallel_exit_from_rotated_pose(self):
        rects = [(-80, 80, 1.95, 7.95)] + FREE['parallel_parking']
        poses = parallel_exit((-2.6, 0.6, math.radians(5.0)), 4.95, 4.43)
        self.assertIsNotNone(poses)
        self.assertAlmostEqual(poses[-1][1], 4.95, delta=0.05)
        self.assertAlmostEqual(poses[-1][2], 0.0, delta=0.02)
        self.assertTrue(free_space_ok(poses, rects, ParkingConfig(clearance=0.2))[0])


if __name__ == '__main__':
    unittest.main()
