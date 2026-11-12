# HEVEN 2026 MIT simulator launcher (Windows PowerShell).
#   .\run.ps1          start (downloads the image the first time) and open the simulator screen in the browser
#   .\run.ps1 shell    open a terminal inside the container (build and run your code here)
#   .\run.ps1 desktop  open a desktop in the browser (noVNC) for RViz, rqt and terminals
#   .\run.ps1 update   download the newest image and restart (your workspace stays)
#   .\run.ps1 stop     stop the simulator (your workspace stays on this computer)
#   .\run.ps1 logs     show simulator logs
param([string]$Action = "start")
$ErrorActionPreference = "Stop"
$Here = Split-Path -Parent $MyInvocation.MyCommand.Path
$Name = "heven"
$Image = if ($env:HEVEN_IMAGE) { $env:HEVEN_IMAGE } else { "ghcr.io/skku-heven/heven-hackathon-2026:2026" }
$HomeDir = if ($env:HEVEN_HOME) { $env:HEVEN_HOME } else { Join-Path $env:USERPROFILE "heven_hackathon_2026" }   # mit_ws (/mit_ws), morai_ws (/morai_ws), data (/data)
$Port = if ($env:HEVEN_PORT) { $env:HEVEN_PORT } else { "8080" }
$DesktopPort = if ($env:HEVEN_DESKTOP_PORT) { $env:HEVEN_DESKTOP_PORT } else { "6080" }

function Need-Docker {
  if (-not (Get-Command docker -ErrorAction SilentlyContinue)) { throw "Docker가 없습니다. 안내서대로 Docker Desktop을 먼저 설치하세요." }
  docker info *> $null
  if ($LASTEXITCODE -ne 0) { throw "Docker Desktop이 실행 중이 아닙니다. 켠 뒤 다시 실행하세요." }
}

function Get-Image {
  docker image inspect $Image *> $null
  if ($LASTEXITCODE -eq 0) { return }
  $arch = if ($env:PROCESSOR_ARCHITECTURE -eq "ARM64") { "arm64" } else { "amd64" }
  $f = Join-Path $Here "images\heven-$arch.tar.gz"
  if (Test-Path $f) {                      # offline copy (organizer USB)
    Write-Host "이미지를 불러오는 중(몇 분 걸립니다): $f"
    docker load -i $f
  } else {
    Write-Host "이미지를 내려받는 중(약 3 GB, 처음 한 번만): $Image"
    docker pull $Image
    if ($LASTEXITCODE -ne 0) { throw "이미지를 내려받지 못했습니다: $Image" }
  }
}

function Prepare-Dirs {  # the base code into each empty workspace: hk_mit -> mit_ws, hk_morai -> morai_ws
  New-Item -ItemType Directory -Force -Path (Join-Path $HomeDir "mit_ws\src"), (Join-Path $HomeDir "morai_ws\src"), (Join-Path $HomeDir "data") | Out-Null
  foreach ($ws in @("mit", "morai")) {
    $pkg = "hk_$ws"
    $wsDir = Join-Path $HomeDir "${ws}_ws"
    $src = Join-Path $wsDir "src"
    if (-not (Get-ChildItem $src -Force | Select-Object -First 1)) {
      $usb = Join-Path $Here "participant\$pkg"
      if (Test-Path $usb) {
        Copy-Item -Recurse -Force $usb $src
      } else {                             # the base code inside the image
        docker run --rm -v "${wsDir}:/w" --entrypoint cp $Image -R "/opt/heven/src/$pkg" /w/src/
      }
      Write-Host "기본 코드 $pkg 를 $src 에 복사했습니다."
    }
  }
}

function Is-Running { [bool](docker ps --format "{{.Names}}" | Where-Object { $_ -eq $Name }) }

function Start-Sim {
  Need-Docker; Get-Image; Prepare-Dirs
  if (Is-Running) { Write-Host "이미 실행 중입니다." } else {
    docker rm -f $Name *> $null
    docker run -d --name $Name -p "127.0.0.1:${Port}:8080" -p "127.0.0.1:${DesktopPort}:6080" `
      -v "${HomeDir}\mit_ws:/mit_ws" -v "${HomeDir}\morai_ws:/morai_ws" -v "${HomeDir}\data:/data" $Image | Out-Null
  }
  $url = "http://localhost:$Port"
  Write-Host "시뮬레이터 화면: $url"
  Write-Host "터미널: .\run.ps1 shell   데스크톱(RViz 등): .\run.ps1 desktop"
  Write-Host "작업 폴더: $HomeDir\mit_ws (컨테이너의 /mit_ws, MIT)  $HomeDir\morai_ws (/morai_ws, MORAI는 빌드만 확인)"
  Start-Sleep 3
  Start-Process $url
}

switch ($Action) {
  "start"   { Start-Sim }
  "shell"   { Need-Docker; docker exec -it $Name bash }
  "desktop" {
    Need-Docker
    if (-not (Is-Running)) { Start-Sim }
    docker exec $Name heven-desktop
    Start-Sleep 2
    Start-Process "http://localhost:${DesktopPort}/vnc.html?autoconnect=1&resize=scale&view_only=0"
  }
  "update"  {
    Need-Docker
    docker pull $Image
    if (Is-Running) { docker rm -f $Name | Out-Null; Start-Sim }
  }
  "stop"    { Need-Docker; docker rm -f $Name | Out-Null; Write-Host "멈췄습니다. 작업 폴더는 $HomeDir 에 남아 있습니다." }
  "logs"    { Need-Docker; docker logs --tail 200 -f $Name }
  default   { Get-Content $MyInvocation.MyCommand.Path -TotalCount 7 }
}
