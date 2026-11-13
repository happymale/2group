# heven-hackathon-2026

HEVEN 2026 Docker 배포본 기반 참가자 실행 환경·스켈레톤·사용 문서.

## 이 레포에서 수정할 주행 코드

### MIT

실제 참가자 패키지는 [`mit_ws/src/hk_mit`](mit_ws/src/hk_mit)에 있습니다.

- [`localization.py`](mit_ws/src/hk_mit/hk_mit/localization.py): 위치 추정
- [`planner.py`](mit_ws/src/hk_mit/hk_mit/planner.py): 경로 계획
- [`control.py`](mit_ws/src/hk_mit/hk_mit/control.py): 조향·속도 제어
- [`lidar_perception.py`](mit_ws/src/hk_mit/hk_mit/lidar_perception.py): LiDAR 입력
- `config/`, `launch/`, `course/`, `rviz/`: 설정·실행·코스·시각화 자료

기존 로컬 MIT 작업 폴더에서 소스와 패키지 자료를 가져왔으며, 빌드 결과물은 포함하지 않았습니다.
기본 실행 스크립트는 사용자 홈의 작업 폴더를 연결합니다. 이 레포의 코드를 직접 연결하려면
저장소 루트에서 `HEVEN_HOME`을 설정하고 컨테이너를 다시 생성합니다.
아래 `stop` 명령은 실행 중인 시뮬레이터를 종료하므로 주행을 끝낸 뒤 실행하세요.

```powershell
# Windows PowerShell, 저장소 루트
$env:HEVEN_HOME = (Get-Location).Path
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\run.ps1 stop
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\run.ps1
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\run.ps1 shell
```

컨테이너 안에서:

```bash
cd /mit_ws
colcon build --packages-select hk_mit
source install/setup.bash
ros2 launch hk_mit bringup.launch.py
```

### MORAI

실제 참가자 패키지는 [`morai_ws/src/hk_morai`](morai_ws/src/hk_morai)에 있습니다.

- [`localization.py`](morai_ws/src/hk_morai/hk_morai/localization.py): 위치 추정
- [`planner.py`](morai_ws/src/hk_morai/hk_morai/planner.py): 경로 계획
- [`control.py`](morai_ws/src/hk_morai/hk_morai/control.py): 조향·속도 제어
- [`lidar_perception.py`](morai_ws/src/hk_morai/hk_morai/lidar_perception.py): LiDAR 입력
- [`camera_perception.py`](morai_ws/src/hk_morai/hk_morai/camera_perception.py): 카메라 입력 스켈레톤
- `config/`, `launch/`, `course/`, `rviz/`: 설정·실행·MORAI 코스·시각화 자료

MIT와 동일하게 기존 로컬 작업 폴더에서 패키지 자료와 소스를 변경 없이 가져왔습니다.
위의 `HEVEN_HOME` 설정으로 레포를 연결하면 Docker 안에서 `/morai_ws`로 보입니다.
참가자 Docker에서는 다음 명령으로 빌드할 수 있습니다. MORAI 시뮬레이터 자체는 운영진 PC에서 실행되며,
이 레포에는 포함되지 않습니다. 실제 연결·주행 검증은 아직 수행하지 않았습니다.

```bash
cd /morai_ws
colcon build --packages-select hk_morai
source install/setup.bash
```

> **대회 안내(정본): [2026 성균관대 자율주행 해커톤 Notion](https://app.notion.com/p/2026-416a48b57a16808e8bc0c0f4e688aed3)** — 시간표, 실행 방법, 기본 코드, 규정이 모두 여기에 있습니다.

## 빠른 시작 (MIT 종목)

1. Docker를 설치하고 켭니다(Windows·macOS: Docker Desktop, Ubuntu: Docker Engine). `docker info`에 Server 정보가 나오면 준비된 것입니다.
2. 이 저장소를 받고 그 폴더에서 실행합니다. 처음 한 번은 시뮬레이터 이미지(약 3.5 GB, `ghcr.io/skku-heven/heven-hackathon-2026:2026`)를 내려받습니다.

```bash
git clone https://github.com/skku-heven/heven-hackathon-2026.git
cd heven-hackathon-2026
./run.sh            # macOS·Ubuntu  (Windows PowerShell: .\run.ps1)
```

3. 브라우저에 시뮬레이터 화면 **http://localhost:8080** 이 열리고, 작업 폴더 `~/heven_hackathon_2026_ws`에 기본 코드가 들어갑니다.

```
~/heven_hackathon_2026_ws/
├── mit_ws/      MIT 코드 (컨테이너 /mit_ws), 기본 코드 hk_mit
├── morai_ws/    MORAI 코드 (컨테이너 /morai_ws, 빌드 확인용), 기본 코드 hk_morai
└── data/        결과 기록 (컨테이너 /data)
```

| 명령 | 하는 일 |
|---|---|
| `./run.sh shell` | 컨테이너 셸(ROS 2 Humble). `cd /mit_ws && colcon build && source install/setup.bash && ros2 launch hk_mit bringup.launch.py` |
| `./run.sh desktop` | 브라우저 데스크톱 **http://localhost:6080** (RViz·rqt·터미널, 켤 때만 돔) |
| `./run.sh update` | 운영진이 새로 올린 이미지 받기(작업 폴더는 그대로) |
| `./run.sh stop` | 끄기 |

Windows에서 스크립트 실행이 막히면 `powershell -ExecutionPolicy Bypass -File .\run.ps1`로 실행합니다.

MORAI는 운영진 PC에서 돕니다. 자기 차례에 `morai_ws/src`를 USB로 가져오세요. 자세한 방법은 Notion 안내의 MORAI 절에 있습니다.

## 배포 경계

개발 저장소 [heven-hackathon-2026-dev](https://github.com/skku-heven/heven-hackathon-2026-dev)가 검증한 Docker 이미지(GitHub 패키지)를 사용한다. `run.sh`·`run.ps1`은 개발 저장소의 `deployment/`에서 복사한다. 참가자 안내의 정본은 Notion 페이지다. 기본 코드(`hk_mit`, `hk_morai`, MORAI 코스 파일 포함)는 이미지 안에 있고 `run.sh`가 작업 폴더로 복사한다. StrictDoc은 OFF이며 요구사항 정본을 중복 관리하지 않는다.

## 협업 설정

저장소 전용 Notion·Discord 알림/멘션·Codex 자동 리뷰는 OFF다. Project 상태 전환은 사용한다.

- 이 repository의 Notion hub: 사용 안 함 (NOTION_MODE=disabled)
- [공통 Notion 가이드](https://app.notion.com/p/39c3bf068300807c9e5bcf6b469f94ee)
