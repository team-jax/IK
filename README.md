# 역운(逆運) — 휴머노이드 상체 로봇 프로젝트

> **Team JAX** · 물건을 집어서 목표 위치에 놓는 휴머노이드 상체 로봇

웹캠으로 사람의 손을 추적해서, MuJoCo 시뮬레이션 속 7자유도(7DOF) 로봇 팔 + 3손가락
그리퍼를 실시간으로 따라 움직이게 하는 프로젝트입니다. 손목 위치는 역운동학(IK)으로
팔 관절각이 되고, 검지/중지/약지의 굽힘은 그리퍼 손가락 3개를 하나씩 움직입니다.
접촉 물리(contact physics)가 켜져 있어서 받침대 위의 큐브를 실제로 집을 수 있습니다.

```
웹캠 → MediaPipe (손목 위치 + 손가락 굽힘) → UDP → IK → MuJoCo 시뮬레이션
                                          └→ 웹 대시보드 (브라우저 시각화)
```

최종 목표는 시뮬레이션 검증을 마친 제어를 실제 로봇(Jetson Nano + STM32F407 + AK45/서보모터)에
옮기고, AMR(자율이동로봇) 위에 탑재하는 것입니다. 하드웨어 스펙과 전체 설계는
[CLAUDE.md](CLAUDE.md)에 정리되어 있습니다.

---

## 두 동작

같은 로봇(팀의 최종 URDF `URDF_1006/`의 7DOF 팔 + 3손가락 손)을 쓰는 두 개의 독립된 시뮬레이션 작업입니다. 공용 가상환경 `venv/`는 루트에 있습니다.

| | 동작 1 | 동작 2 |
|---|---|---|
| 폴더 | [`motion1/`](motion1/README.md) (별도 git 저장소) | [`motion2/`](motion2/README.md) |
| 하는 일 | 스크립트 컨트롤러로 50 mm 큐브를 집어 목표 지점에 놓기(배달), 2단 쌓기 | 웹캠으로 사람의 손을 추적해 팔과 손가락을 실시간으로 조종하고 큐브 집기 |
| 상태 | 배달 100/100, 2단 쌓기 46/50 (2026-10-12) | 모델 생성, IK 0.00 mm, 파지 9/9, UDP 파이프라인 3/3. 실제 사람 추적의 육안 확인이 남음 |
| 문서 | [`motion1/README.md`](motion1/README.md), [`STATUS.md`](motion1/STATUS.md) | [`motion2/README.md`](motion2/README.md), [`STATUS.md`](motion2/STATUS.md) |

## 빠른 시작 (저장소 루트에서)

```bash
# 가상환경 (처음 한 번)
python3 -m venv venv
venv/bin/python -m pip install mujoco gymnasium numpy scipy glfw ikpy mediapipe opencv-python imageio imageio-ffmpeg pillow

# 동작 2 데모: 터미널 2~3개, 1번을 먼저. 웹캠과 화면이 필요합니다. 조작법은 motion2/README.md
motion2/scripts/run_receiver.sh     # 1: MuJoCo 시뮬레이션 (팔 + 그리퍼 + 큐브)
motion2/scripts/run_sender.sh       # 2: 웹캠 손 추적 → UDP 전송
motion2/scripts/run_web.sh          # 3 (선택): 웹 대시보드 http://127.0.0.1:8000

# 동작 1: 화면·웹캠 불필요
venv/bin/python motion1/scripts/smoke_test.py
```

---

## 리포지토리 구조

저장소 루트:

| 경로 | 내용 |
|---|---|
| `URDF_1006/` | 팀의 최종 URDF(CAD export). 동작 1·2가 공용으로 읽는 원본 |
| `motion1/` | 동작 1: 최종 URDF 한 팔 + 3손가락 손의 큐브 배달·2단 쌓기 (별도 git 저장소, 자체 README) |
| `motion2/` | 동작 2: 웹캠 손 추적 텔레오퍼레이션 + 팔 모델 생성기 (자체 README, 아래 표) |
| `venv/` | 두 동작이 공용하는 가상환경 |
| `CLAUDE.md` | 프로젝트 전체 설계: 동작 1/2 구분, 하드웨어, 아키텍처, 통신 프로토콜, 로드맵 |
| `STATUS.md` | 전체 요약과 저장소 공통 기록 (동작별 기록은 각 폴더의 `STATUS.md`) |
| `run_tests.sh` | 전체 테스트 한 번에 (`--full`, `--viewers` 옵션) |
| `check_docs.py` | 문서에 적힌 기호·경로·링크가 코드에 실제로 있는지 대조 |
| `AGENTS.md` | AI 에이전트용 문서의 최상위 (하위 폴더마다 `AGENTS.md`가 있음) |
| `haness.md` | AI 작업 규칙 (동작 1은 `motion1/HARNESS.md`가 우선) |

`motion2/` 안 (이 아래 경로는 `motion2/` 기준):

| 경로 | 내용 |
|---|---|
| `README.md`, `STATUS.md` | 실행·조작·구조·UDP 패킷·테스트·한계, 진행 기록 |
| `src/` | 송신기 `vision_sender.py`(MediaPipe로 손목·손가락 추출, UDP 전송), 수신기 `mujoco_receiver.py`(IK + 접촉 물리), 대시보드 `web_dashboard.py` |
| `scripts/` | 실행 래퍼 `run_*.sh`와 화면·웹캠 없는 검사 `check_*.py`, 뷰어 확인 `check_viewer.py` |
| `models/` | MediaPipe 모델 `pose_landmarker_full.task`, `hand_landmarker.task` |
| `arm/` | 최종 URDF에서 팔 모델(`arm.urdf`, `arm.xml`, `meshes/`)을 만드는 생성기 `make_arm_urdf.py`와 IK `ik.py` |
| `AGENTS.md`, `src/AGENTS.md`, `scripts/AGENTS.md`, `arm/AGENTS.md` | AI 에이전트용 상세 문서 (구현 패턴, 주의사항) |

### 주의: Python 실행 방법

- 항상 루트 `venv/bin/python`을 사용하세요 (전역 python 금지).
- **macOS에서 MuJoCo 뷰어를 띄우는 스크립트는 `venv/bin/mjpython`으로 실행해야 합니다**
  (`motion2/src/mujoco_receiver.py`, `motion2/arm/ik.py --view`, `motion2/scripts/check_viewer.py`, 동작 1의
  `watch_grasp.py`). 일반 python으로 실행하면 `launch_passive requires ... mjpython` 에러가 납니다.
  `motion2/scripts/run_*.sh`는 이를 자동 분기합니다.

---

## 테스트 (웹캠·디스플레이 불필요)

저장소 루트에서 한 번에 (끝에 통과/실패 개수를 출력하고, 하나라도 실패하면 종료 코드 1):

```bash
./run_tests.sh             # 빠른 모드, 약 1분: 문법·문서 대조, 동작 2 전체, 동작 1 스모크
./run_tests.sh --full      # + 배달 100회·쌓기 50회 성공률, 도달 맵, 영상 2개 (약 3분 추가)
./run_tests.sh --viewers   # + MuJoCo 뷰어 4개를 9초씩 열어 둠 (창이 뜨고, 화면이 켜져 있어야 함)
venv/bin/python check_docs.py   # 문서 대조만 따로
```

개별 검사와 측정 도구는 동작 2는 [`motion2/README.md`](motion2/README.md), 동작 1은 [`motion1/README.md`](motion1/README.md)에 있습니다.

---

## 개발 로드맵

- [x] **Phase 1 (일부)** — STM32 FreeRTOS + PWM 설계, UART 패킷 프로토콜 설계
- [x] **Phase 2** — MuJoCo 7DOF 팔 모델 + ikpy IK + 손 추적 텔레오퍼레이션 + 접촉 물리 파지 (동작 2), 스크립트 컨트롤러로 큐브 배달·2단 쌓기 (동작 1)
- [ ] **Phase 3** — 실제 로봇 통합 (IK → 실모터 연동, 강화학습 적용)
- [ ] **Phase 4** — AMR 탑재 및 전체 시스템 통합

## 팀 (Team JAX)

| 이름 | 역할 |
|---|---|
| 이재준 | 통신 + 모터 제어 |
| 강준영 | 기구 설계 |
| 김규민 | 하드웨어 |
| 정태양 (PRDX) | 팀장 / 전체 방향 |
