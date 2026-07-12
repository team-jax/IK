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

## 데모: 손으로 시뮬레이션 팔 조종하기

### 1. 준비물

- **macOS 또는 Linux** + 로컬 디스플레이 (MuJoCo 뷰어가 GUI 창을 띄우므로 headless 불가)
- **웹캠**
- **Python 가상환경** — `venv/`에 만들어 사용합니다:

```bash
python3 -m venv venv
venv/bin/pip install mujoco gymnasium numpy glfw ikpy mediapipe opencv-python
```

- **MediaPipe 모델 파일 2개** — 리포지토리 루트에 내려받아 둡니다:

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

터미널 1을 먼저 실행하고, 2·3은 순서 무관합니다. 자세한 내용은 [실행방법.txt](실행방법.txt) 참고.

### 3. 조작법

- **팔** — 오른손 손목을 카메라 앞에서 움직이면 팔이 따라옵니다.
- **손가락** — 검지/중지/약지를 각각 굽히면 그리퍼 손가락 1/2/3번이 따로 움직입니다.
  (그리퍼가 3손가락이라 엄지/새끼는 의도적으로 무시)
- **집기** — 받침대 위 빨간 큐브 주위로 그리퍼를 내리고 손가락을 오므리면 접촉·마찰로 집힙니다.
- MuJoCo 뷰어 키: **T** = 그리퍼 추적 카메라 켜기/끄기, **C** = 좌우 중심 재캘리브레이션
- 종료: 웹캠 창에서 **q**, MuJoCo 뷰어는 창 닫기

팔 움직임 범위가 이상하면 `mujoco_receiver.py` 상단의 `LATERAL_SCALE`/`HEIGHT_SCALE`/
`DEPTH_SCALE`/`BASE_Z`를 조정하세요.

---

## 리포지토리 구조

| 경로 | 내용 |
|---|---|
| `vision_sender.py` | MediaPipe Tasks(Pose + Hand)로 손목 (x, y, z)와 손가락 3개 curl(0=펴짐~1=굽힘)을 추출, UDP 5005/5006으로 전송 |
| `mujoco_receiver.py` | UDP 수신 → palm-down 제약 IK → position actuator로 팔·그리퍼 구동, `mj_step` 접촉 물리 |
| `web_dashboard.py` | UDP 수신 값을 브라우저에 실시간 표시 (stdlib만 사용, 외부 패키지 불필요) |
| `arm/arm.urdf` | ikpy용 7관절 IK 체인 (그리퍼 미포함) |
| `arm/arm.xml` | MuJoCo 모델: 팔 7관절 + 3손가락 그리퍼 + 받침대/큐브 씬 + actuator 13개 |
| `arm/ik.py` | IK 단독 풀이/검증 스크립트 (`--target x y z`, `--view`, `--interactive`) |
| `run_*.sh` | 위 스크립트들의 실행 래퍼 (venv 체크, macOS/Linux 분기) |
| `test.py` | MuJoCo + gymnasium 설치 확인용 스크래치 |
| `CLAUDE.md` | 프로젝트 전체 설계: 하드웨어, 아키텍처, 통신 프로토콜, 로드맵 |
| `AGENTS.md`, `arm/AGENTS.md` | AI 에이전트용 상세 문서 (구현 패턴, 주의사항) |

### 주의: Python 실행 방법

- 항상 `venv/bin/python`을 사용하세요 (전역 python 금지).
- **macOS에서 MuJoCo 뷰어를 띄우는 스크립트는 `venv/bin/mjpython`으로 실행해야 합니다**
  (`mujoco_receiver.py`, `arm/ik.py --view`, `test.py`). 일반 python으로 실행하면
  `launch_passive requires ... mjpython` 에러가 납니다. `run_*.sh`는 이를 자동 분기합니다.

### UDP 패킷 포맷

```
"x y z curl_index curl_middle curl_ring"   (공백 구분 float 6개 문자열)
```

- `x, y`: 프레임 정규화 좌표(0~1), `z`: MediaPipe 깊이 추정치
- 구형 4필드 `"x y z grip"`도 수신 측에서 하위 호환 지원 (grip을 3손가락에 동일 적용)
- 필드 수를 바꾸면 `vision_sender.py`, `mujoco_receiver.py`, `web_dashboard.py` 셋 다 맞춰야 함

---

## IK 단독 테스트 (웹캠 불필요)

```bash
venv/bin/python arm/ik.py                      # 기본 타깃 풀고 오차(mm) 출력
venv/bin/python arm/ik.py --target 0.3 0.2 0.5 # world 좌표 타깃 지정
venv/bin/mjpython arm/ik.py --interactive      # 뷰어 + 터미널에 "x y z" 입력으로 실시간 IK
```

---

## 개발 로드맵

- [x] **Phase 1 (일부)** — STM32 FreeRTOS + PWM 설계, UART 패킷 프로토콜 설계
- [x] **Phase 2** — MuJoCo 7DOF 팔 모델 + ikpy IK + 손 추적 텔레오퍼레이션 + 접촉 물리 파지
- [ ] **Phase 3** — 실제 로봇 통합 (IK → 실모터 연동, 강화학습 적용)
- [ ] **Phase 4** — AMR 탑재 및 전체 시스템 통합

## 팀 (Team JAX)

| 이름 | 역할 |
|---|---|
| 이재준 | 통신 + 모터 제어 |
| 강준영 | 기구 설계 |
| 김규민 | 하드웨어 |
| 정태양 (PRDX) | 팀장 / 전체 방향 |
