"""코스·경로 파일과 경로 도우미 (MIT, 보통 고칠 필요가 없습니다).

기본은 이 패키지의 course/ 폴더(설치되면 share/hk_mit/course/)에 있는 route.csv(전역 경로, 0.5 m 간격)와
course.json(출발·체크포인트 복귀 자세, 지도 원점)입니다. 파라미터 course_json, route_csv로 바꿀 수 있습니다.
"""
import csv
import json
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

COURSE_DIR = Path(get_package_share_directory('hk_mit')) / 'course'


def declare_file_params(node):
    """노드에 route_csv, course_json 파라미터를 만듭니다 (비우면 패키지의 course/ 파일)."""
    node.declare_parameter('route_csv', '')
    node.declare_parameter('course_json', '')


def files_signature(node, *extra):
    """경로·코스 파일 경로와 수정 시각 (+ extra). 시작(START)마다 파일을 다시 읽어 경로·속도 프로파일을 새로 만들면
    planner가 0.3~1 s 멈춰 출발이 0.5~2 s 늦어지므로, 이 값이 같으면 지난번 것을 그대로 씁니다."""
    sig = list(extra)
    for name, default in (('route_csv', 'route.csv'), ('course_json', 'course.json')):
        path = node.get_parameter(name).value or str(COURSE_DIR / default)
        try:
            sig.append((path, Path(path).stat().st_mtime_ns))
        except OSError:
            sig.append((path, None))
    return tuple(sig)


def load_course(node):
    return Course(node.get_parameter('course_json').value or str(COURSE_DIR / 'course.json'))


def load_route(node):
    return Route(node.get_parameter('route_csv').value or str(COURSE_DIR / 'route.csv'))


class Course:
    """출발·체크포인트 복귀 자세와 지도 원점 (course.json: 공개 코스 + 이번 구성의 복귀 자세)."""

    def __init__(self, path):
        with open(path, encoding='utf-8') as f:
            d = json.load(f)
        c = d['course']
        self.origin = c['origin_utm_52n']
        self.start = (c['start']['x'], c['start']['y'], c['start']['yaw'])
        poses = d['respawn_poses']
        self.cps = [None if poses.get(cp['id']) is None else
                    (poses[cp['id']]['x'], poses[cp['id']]['y'], poses[cp['id']]['yaw']) for cp in c['checkpoints']]

    def pose(self, n):
        """n번째 체크포인트의 복귀 자세 (0 = 출발 자세)."""
        if 0 < n <= len(self.cps) and self.cps[n - 1] is not None:
            return self.cps[n - 1]
        return self.start


class Route:
    """전역 경로(0.5 m 간격 x, y)와 그 위의 지금 위치."""

    def __init__(self, path=None):
        self.points = []
        if path:
            with open(path, encoding='utf-8') as f:
                self.points = [(float(r['x']), float(r['y'])) for r in csv.DictReader(f)]
        self.reset()

    def reset(self):
        """시작·복귀: 다음 nearest()는 경로 전체에서 찾습니다."""
        self.idx = None

    def nearest(self, x, y):
        """(x, y)에서 가장 가까운 경로 점의 번호. 시작·복귀 직후에는 경로 전체에서, 그 뒤에는 지난번 점부터
        앞쪽 40 m에서만 찾습니다(되돌아가지 않게)."""
        pts = self.points
        start = 0 if self.idx is None else self.idx
        end = len(pts) if self.idx is None else min(self.idx + 80, len(pts))
        self.idx = min(range(start, end), key=lambda i: math.hypot(pts[i][0] - x, pts[i][1] - y))
        return self.idx

    def yaw_at(self, i):
        """i번째 점에서의 경로 방향 (rad)."""
        j = max(0, min(i, len(self.points) - 2))
        (ax, ay), (bx, by) = self.points[j], self.points[j + 1]
        return math.atan2(by - ay, bx - ax)

    def past_end(self, x, y):
        """경로 마지막 점을 5 m 넘게 지났으면 True."""
        ex, ey = self.points[-1]
        return self.idx is not None and self.idx >= len(self.points) - 2 and math.hypot(ex - x, ey - y) > 5.0
