# 대회 공식 저장소 가져오기

- 원본: https://github.com/skku-heven/heven-hackathon-2026
- 대상: https://github.com/happymale/2group
- 원본 브랜치: `dev` (가져온 시점의 기본 브랜치)
- 원본 커밋: `2aa0f9d0c537d490c4626af23aedb75986f813c5`
- 가져온 날짜: 2026-11-13 (Asia/Seoul)

공식 저장소의 파일과 Git 이력을 보존했습니다. 이후 README에 MIT 소스 위치와 실행 방법을 추가했습니다.
원본 remote는 `upstream`, 개인 저장소 remote는 `origin`으로 구분합니다.

설치와 실행은 [README.md](README.md)를 따릅니다.
이 저장소에는 참가자 실행 스크립트와 문서가 포함돼 있으며,
주행 기본 코드 `hk_mit`, `hk_morai`는 실행 시 Docker 이미지에서 작업 폴더로 복사됩니다.
추가로 기존 로컬 MIT 작업 폴더의 `mit_ws/src/hk_mit` 패키지 전체를 같은 상대 경로에 복사했습니다.
Python 노드, 설정, launch, 코스 자료, RViz 설정, 패키지 메타데이터를 포함하며 소스 내용은 변경하지 않았습니다.
Docker 이미지와 빌드 결과물, 로그, 로컬 환경 설정은 포함하지 않았습니다.
