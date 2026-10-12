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

> 프로젝트는 두 동작으로 나뉩니다. **동작 1** = `motion1/` (최종 URDF의 팔 + 3손가락 손으로 50 mm 큐브를 집어 배달·2단 쌓기하는 스크립트 컨트롤러, 자체 README 참고),
> **동작 2** = `motion2/` (자체 제작 7DOF 팔 + 웹캠 손 추적, 아래 내용). 공용 가상환경 `venv/`는 루트에 있습니다.

## 동작 2 데모: 손으로 시뮬레이션 팔 조종하기

아래 경로는 별도 언급이 없으면 `motion2/` 기준이고, 실행 명령은 `motion2/`에서 합니다 (`cd motion2`). 가상환경 만들기와 패키지 설치만 저장소 루트에서 합니다.

### 1. 준비물

- **macOS 또는 Linux** + 로컬 디스플레이 (MuJoCo 뷰어가 GUI 창을 띄우므로 headless 불가)
- **웹캠**
- **Python 가상환경** — 저장소 루트의 `venv/`를 동작 1·2가 함께 씁니다 (루트에서 실행):

```bash
python3 -m venv venv
venv/bin/python -m pip install mujoco gymnasium numpy scipy glfw ikpy mediapipe opencv-python imageio imageio-ffmpeg pillow
```

- **MediaPipe 모델 파일 2개** — `motion2/`에 이미 들어 있습니다. 없을 때만 `motion2/`에서 내려받으세요:

```bash
curl -o pose_landmarker_full.task \
  https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_full/float16/latest/pose_landmarker_full.task
curl -o hand_landmarker.task \
  https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task
```

### 2. 실행 (터미널 2~3개)

| 터미널 | 명령 | 역할 |
|---|---|---|
| 1 | `./run_receiver.sh` | MuJoCo 시뮬레이션 (팔 + 그리퍼 + 큐브) |
| 2 | `./run_sender.sh` | 웹캠 손 추적 → UDP 전송 |
| 3 (선택) | `./run_web.sh` | 웹 대시보드 → http://127.0.0.1:8000 |

터미널 1을 먼저 실행하고, 2·3은 순서 무관합니다. 자세한 내용은 [실행방법.txt](motion2/실행방법.txt) 참고.

### 3. 조작법

- **팔** — 오른손 손목을 카메라 앞에서 움직이면 팔이 따라옵니다.
- **손가락** — 검지/중지/약지를 각각 굽히면 그리퍼 손가락 1/2/3번이 따로 움직입니다.
  (그리퍼가 3손가락이라 엄지/새끼는 의도적으로 무시)
- **집기** — 좁은 받침 기둥 위 빨간 큐브 양옆으로 손을 내리고 손가락을 오므리면 접촉·마찰로 집힙니다.
- MuJoCo 뷰어 키: **T** = 그리퍼 추적 카메라 켜기/끄기, **C** = 좌우 중심 재캘리브레이션
- 종료: 웹캠 창에서 **q**, MuJoCo 뷰어는 창 닫기

팔 움직임 범위가 이상하면 `mujoco_receiver.py` 상단의 `LATERAL_SCALE`/`HEIGHT_SCALE`/
`DEPTH_SCALE`/`BASE_Z`를 조정하세요. 모델을 바꾸려면 `../URDF_1006`을 교체하고 `../venv/bin/python arm/make_arm_urdf.py`를 실행합니다.

---

## 리포지토리 구조

저장소 루트:

| 경로 | 내용 |
|---|---|
| `URDF_1006/` | 팀의 최종 URDF(CAD export). 동작 1·2가 공용으로 읽는 원본 |
| `motion1/` | 동작 1: 최종 URDF 한 팔 + 3손가락 손의 큐브 배달·2단 쌓기 (별도 git 저장소, 자체 README) |
| `motion2/` | 동작 2: 웹캠 손 추적 텔레오퍼레이션 + 팔 모델 생성기 (아래 표) |
| `venv/` | 두 동작이 공용하는 가상환경 |
| `CLAUDE.md` | 프로젝트 전체 설계: 동작 1/2 구분, 하드웨어, 아키텍처, 통신 프로토콜, 로드맵 |
| `STATUS.md` | 진행 상황과 문제·해결 기록 (동작 2 중심, 동작 1은 `motion1/STATUS.md`) |
| `AGENTS.md` | AI 에이전트용 문서의 최상위 (하위 폴더마다 `AGENTS.md`가 있음) |
| `haness.md` | AI 작업 규칙 (동작 1은 `motion1/HARNESS.md`가 우선) |

`motion2/` 안 (이 아래 경로는 `motion2/` 기준):

| 경로 | 내용 |
|---|---|
| `vision_sender.py` | MediaPipe Tasks(Pose + Hand)로 손목 (x, y, z)와 손가락 3개 curl(0=펴짐~1=굽힘)을 추출, UDP 5005/5006으로 전송 |
| `mujoco_receiver.py` | UDP 수신 → palm-down 제약 IK → position actuator로 팔·그리퍼 구동, `mj_step` 접촉 물리 |
| `web_dashboard.py` | UDP 수신 값을 브라우저에 실시간 표시 (stdlib만 사용, 외부 패키지 불필요) |
| `arm/make_arm_urdf.py` | 최종 URDF(`../URDF_1006`)에서 `arm.urdf`/`arm.xml`/`meshes/`를 생성하는 스크립트 |
| `arm/arm.urdf` | ikpy용 7관절 IK 체인 (생성 파일, 손 미포함) |
| `arm/arm.xml` | MuJoCo 모델: 팔 7관절 + 3손가락 손 + 기둥/큐브 씬 + actuator 13개 (생성 파일) |
| `arm/ik.py` | IK 단독 풀이/검증 스크립트 (`--target x y z`, `--view`, `--interactive`) |
| `check_grasp.py` | 화면 없는 파지 검증: 큐브가 ±8 mm 어긋난 9가지 위치에서 닫고 들어올리기 (기대값 9/9) |
| `run_*.sh` | 위 스크립트들의 실행 래퍼 (venv 체크, macOS/Linux 분기) |
| `test.py` | MuJoCo + gymnasium 설치 확인용 스크래치 |
| `실행방법.txt` | 3터미널 실행 순서와 스케일 조정법 |
| `AGENTS.md`, `arm/AGENTS.md` | AI 에이전트용 상세 문서 (구현 패턴, 주의사항) |

### 주의: Python 실행 방법

- 항상 루트 `venv/bin/python`(motion2에서는 `../venv/bin/python`)을 사용하세요 (전역 python 금지).
- **macOS에서 MuJoCo 뷰어를 띄우는 스크립트는 `venv/bin/mjpython`으로 실행해야 합니다**
  (`mujoco_receiver.py`, `arm/ik.py --view`, `test.py`, 동작 1의 `watch_grasp.py`). 일반 python으로 실행하면
  `launch_passive requires ... mjpython` 에러가 납니다. `run_*.sh`는 이를 자동 분기합니다.

### UDP 패킷 포맷

```
"x y z curl_index curl_middle curl_ring"                    (float 6개)
"x y z curl_index curl_middle curl_ring ex ey ez sx sy sz"   (12개: 팔꿈치·어깨가 둘 다 보일 때만)
```

- `x, y`: 프레임 정규화 좌표(0~1), `z`: MediaPipe 깊이 추정치
- 6필드면 손목만, 12필드면 팔꿈치 스윙각까지 따라갑니다.
- 구형 4필드 `"x y z grip"`도 수신 측에서 하위 호환 지원 (grip을 3손가락에 동일 적용)
- 필드 수를 바꾸면 `vision_sender.py`, `mujoco_receiver.py`, `web_dashboard.py` 셋 다 맞춰야 함

---

## 테스트 (웹캠·디스플레이 불필요)

```bash
../venv/bin/python arm/ik.py                  # 기본 타깃 풀고 오차(mm) 출력
../venv/bin/python arm/ik.py --target 0.3 0.2 0.5 # world 좌표 타깃 지정
../venv/bin/mjpython arm/ik.py --interactive      # 뷰어 + 터미널에 "x y z" 입력으로 실시간 IK
../venv/bin/python check_grasp.py                 # 큐브 파지·들어올리기 9/9 확인
```

동작 1은 저장소 루트에서 `venv/bin/python motion1/scripts/smoke_test.py` (뷰어는 `venv/bin/mjpython motion1/scripts/watch_grasp.py`, 자세한 측정 도구는 `motion1/README.md`).

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
