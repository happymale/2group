#!/usr/bin/env bash
# HEVEN 2026 MIT simulator launcher (Linux / macOS).
#   ./run.sh          start (downloads the image the first time) and open the simulator screen in the browser
#   ./run.sh shell    open a terminal inside the container (build and run your code here)
#   ./run.sh desktop  open a desktop in the browser (noVNC) for RViz, rqt and terminals
#   ./run.sh update   download the newest image and restart (your workspace stays)
#   ./run.sh stop     stop the simulator (your workspace stays on this computer)
#   ./run.sh logs     show simulator logs
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
NAME=heven
IMAGE="${HEVEN_IMAGE:-ghcr.io/skku-heven/heven-hackathon-2026:2026}"
HOME_DIR="${HEVEN_HOME:-$HOME/heven_hackathon_2026}"   # mit_ws (/mit_ws), morai_ws (/morai_ws), data (/data)
PORT="${HEVEN_PORT:-8080}"
DESKTOP_PORT="${HEVEN_DESKTOP_PORT:-6080}"

arch() { case "$(uname -m)" in x86_64|amd64) echo amd64 ;; arm64|aarch64) echo arm64 ;; *) uname -m ;; esac; }

open_url() {
  if command -v open >/dev/null 2>&1; then open "$1"; elif command -v xdg-open >/dev/null 2>&1; then xdg-open "$1" >/dev/null 2>&1 || true; fi
}

need_docker() {
  if ! command -v docker >/dev/null 2>&1; then
    echo "Docker가 없습니다. 안내서대로 Docker를 먼저 설치하세요." >&2; exit 1
  fi
  if ! docker info >/dev/null 2>&1; then
    echo "Docker가 실행 중이 아닙니다. Docker Desktop을 켜거나(Linux: sudo systemctl start docker) 다시 시도하세요." >&2; exit 1
  fi
}

get_image() {
  if docker image inspect "$IMAGE" >/dev/null 2>&1; then return; fi
  local f="$HERE/images/heven-$(arch).tar.gz"
  if [ -f "$f" ]; then                         # offline copy (organizer USB)
    echo "이미지를 불러오는 중(몇 분 걸립니다): $f"
    docker load -i "$f"
  else
    echo "이미지를 내려받는 중(약 3 GB, 처음 한 번만): $IMAGE"
    docker pull "$IMAGE"
  fi
}

shell_user() {  # Linux: run as the host user so files in the workspace stay editable on the host
  if [ "$(uname -s)" = Linux ]; then echo "-u $(id -u):$(id -g) -e HOME=/mit_ws/.home -e ROS_LOG_DIR=/mit_ws/.home/ros_log"; fi
}

prepare_dirs() {  # the base code into each empty workspace: hk_mit -> mit_ws, hk_morai -> morai_ws
  mkdir -p "$HOME_DIR/mit_ws/src" "$HOME_DIR/morai_ws/src" "$HOME_DIR/data"
  local ws pkg
  for ws in mit morai; do
    pkg="hk_$ws"
    if [ -z "$(ls -A "$HOME_DIR/${ws}_ws/src" 2>/dev/null)" ]; then
      if [ -d "$HERE/participant/$pkg" ]; then
        cp -R "$HERE/participant/$pkg" "$HOME_DIR/${ws}_ws/src/"
      else                                      # the base code inside the image
        # shellcheck disable=SC2046
        docker run --rm $(shell_user) -v "$HOME_DIR/${ws}_ws:/w" --entrypoint cp "$IMAGE" -R "/opt/heven/src/$pkg" /w/src/
      fi
      echo "기본 코드 $pkg 를 $HOME_DIR/${ws}_ws/src 에 복사했습니다."
    fi
  done
}

running() { docker ps --format '{{.Names}}' | grep -qx "$NAME"; }

start() {
  need_docker; get_image; prepare_dirs
  if running; then echo "이미 실행 중입니다."; else
    docker rm -f "$NAME" >/dev/null 2>&1 || true
    local ids=""
    if [ "$(uname -s)" = Linux ]; then ids="-e HEVEN_UID=$(id -u) -e HEVEN_GID=$(id -g)"; fi
    # shellcheck disable=SC2086
    docker run -d --name "$NAME" -p "127.0.0.1:$PORT:8080" -p "127.0.0.1:$DESKTOP_PORT:6080" $ids \
      -v "$HOME_DIR/mit_ws:/mit_ws" -v "$HOME_DIR/morai_ws:/morai_ws" -v "$HOME_DIR/data:/data" "$IMAGE" >/dev/null
  fi
  local url="http://localhost:$PORT"
  echo "시뮬레이터 화면: $url"
  echo "터미널: $0 shell   데스크톱(RViz 등): $0 desktop"
  echo "작업 폴더: $HOME_DIR/mit_ws (컨테이너의 /mit_ws, MIT)  $HOME_DIR/morai_ws (/morai_ws, MORAI는 빌드만 확인)"
  sleep 3
  open_url "$url"
}

case "${1:-start}" in
  start) start ;;
  shell)
    need_docker
    # shellcheck disable=SC2046
    docker exec -it $(shell_user) "$NAME" bash -c 'mkdir -p "$HOME"; exec bash' ;;
  desktop)
    need_docker
    running || start
    # shellcheck disable=SC2046
    docker exec $(shell_user) "$NAME" bash -c 'mkdir -p "$HOME"; heven-desktop'
    sleep 2
    open_url "http://localhost:$DESKTOP_PORT/vnc.html?autoconnect=1&resize=scale&view_only=0" ;;
  update)
    need_docker
    docker pull "$IMAGE"
    if running; then docker rm -f "$NAME" >/dev/null; start; fi ;;
  stop) need_docker; docker rm -f "$NAME" >/dev/null && echo "멈췄습니다. 작업 폴더는 $HOME_DIR 에 남아 있습니다." ;;
  logs) need_docker; docker logs --tail 200 -f "$NAME" ;;
  *) sed -n '2,8p' "$0"; exit 2 ;;
esac
