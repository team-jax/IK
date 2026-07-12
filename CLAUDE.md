# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

# 🤖 Team JAX - 휴머노이드 상체 로봇 프로젝트

> 물건을 집어서 목표 위치에 놓는 휴머노이드 상체 로봇 개발

---

## 0. 현재 리포지토리 상태

STM32 펌웨어, Jetson/ROS 코드, 강화학습 구현 등은 아래 문서에 설계된 계획이며 아직 이
리포지토리에 커밋되어 있지 않다. 새 코드를 추가할 때 아래 아키텍처/프로토콜 설계를 기준으로
삼을 것. Phase 2(시뮬레이션 + IK + 그리퍼 파지)는 `arm/`에 구현되어 있고 (§0.2), 웹캠
손 추적으로 시뮬레이션 팔을 실시간 조종하는 파이프라인이 리포 루트에 있다 (§0.3).

### 0.1 Python 환경 (MuJoCo 시뮬레이션)

- 가상환경은 `venv/`에 있다 (Python 3.14). 전역 `python`/`pip` 대신 venv를 직접 사용:
  - `venv/bin/python <script>.py`
  - `venv/bin/pip install <package>`
- 설치된 주요 패키지: `mujoco`, `gymnasium`, `numpy`, `glfw`, `ikpy`(+ `scipy`, `sympy`),
  `mediapipe`, `opencv-python`.
- 패키지 매니페스트(`requirements.txt`/`pyproject.toml`)는 없음 — 의존성은 venv에 직접 설치됨.

`test.py`는 gymnasium이 함께 제공하는 MuJoCo 모델(`reacher.xml`)을 불러와 인터랙티브 passive
viewer를 띄우는 스크래치 스크립트:

```
venv/bin/mjpython test.py   # macOS
venv/bin/python test.py     # Linux
```

이 모델은 `gymnasium.envs.mujoco` 패키지가 설치된 위치(`.../assets/reacher.xml`)에서
동적으로 경로를 찾아 불러온다. `mujoco.viewer.launch_passive`는 GUI 창을 열고 창이 닫힐
때까지 블로킹한다 — 로컬 디스플레이가 필요하며 headless로는 실행할 수 없다. macOS에서는
일반 `python`으로 실행하면 `RuntimeError: launch_passive requires that the Python script
be run under mjpython on macOS`가 발생하므로 venv에 함께 설치된 `mjpython`을 사용해야 한다.

### 0.2 `arm/` — 7DOF 팔 모델 + IK (Phase 2)

CLAUDE.md §2/§4의 하드웨어 스펙(어깨 2DOF + 팔꿈치 1DOF + 상완회전 1DOF + 손목 3DOF = 7DOF,
한 팔 기준)을 그대로 따르는 한 팔 키네마틱 모델과 IK 데모.

- `arm/arm.urdf` — ikpy가 읽는 7-revolute-joint 체인 정의. 관절 순서:
  `shoulder_pitch → shoulder_roll → upper_arm_roll → elbow → wrist_yaw → wrist_pitch → wrist_roll`.
  링크 길이(상완 0.30m / 전완 0.25m / 손 0.12m)는 기구 설계 확정 전까지의 placeholder.
- `arm/arm.xml` — 팔 7관절은 위 URDF와 동일한 관절 순서/축/링크 길이를 갖는 MuJoCo MJCF.
  `arm.urdf`를 수정하면 `arm.xml`의 팔 부분도 같이 맞춰야 두 모델의 forward kinematics가
  일치한다. 엔드 이펙터 위치는 `end_effector` site로 노출됨. URDF에는 없는 요소가 추가되어
  있다: 3손가락 그리퍼(손가락당 hinge 2개 = 관절 6개, `wrist_roll_link` 끝에 부착),
  받침대(pedestal) 위의 파지용 큐브(freejoint), 바닥 plane, 그리고 팔 7관절 + 손가락 6관절을
  구동하는 `<position>` actuator 13개. 그리퍼는 ikpy 체인(`arm.urdf`)에 포함되지 않으므로 팔
  IK에는 영향이 없고, qpos 배치는 팔 `qpos[0:7]` → 손가락 `qpos[7:13]` → 큐브 freejoint
  `qpos[13:20]` 순서로 고정되어 있다 (`arm/ik.py`의 `apply_pose()`가 `qpos[:7]`만 쓰는 것과
  `mujoco_receiver.py`가 이 배치에 의존). 충돌은 손가락↔큐브/받침대/바닥만 활성화되어 있고
  팔 링크와 손가락↔손가락 충돌은 비트마스크로 제외됨 (파일 상단 주석 참고).
- `arm/ik.py` — ikpy로 목표 위치(x, y, z, world frame)에 대한 IK를 풀고, 그 관절각을 MJCF에
  적용해 `end_effector` site가 실제로 목표에 도달하는지 검증:
  ```
  venv/bin/python arm/ik.py                          # 기본 타깃으로 풀고 오차(mm) 출력
  venv/bin/python arm/ik.py --target 0.3 0.2 0.5     # world-space 타깃 직접 지정
  venv/bin/python arm/ik.py --view                   # MuJoCo passive viewer로 결과 확인 (로컬 디스플레이 필요)
  venv/bin/python arm/ik.py --interactive            # 뷰어를 띄운 채 터미널에 "x y z" 입력 시 실시간 IK
  ```
  `--view`/`--interactive` 둘 다 `mj_step`을 호출하지 않는다 — 이 뷰어는 동역학이 아니라
  IK 결과를 정적으로 확인하는 용도라, 스텝을 돌리면 actuator 없이 중력만 적분되어 팔이
  처지고 흔들린다. `--interactive`는 입력마다 새 자세로 즉시 스냅(보간 없음)하며,
  `launch_passive`가 렌더링을 자체 스레드에서 돌리는 것을 이용해 메인 스레드의 `input()`
  블로킹과 `viewer.lock()`만으로 별도 threading 없이 구현됨.
  ikpy의 기본 시드(all-zero pose)는 팔이 일직선으로 펴진 특이점이라 SLSQP가 수렴하지 못하는
  타깃이 많았다 — `solve_ik`의 `SEED_POSE`(팔꿈치를 살짝 굽힌 비특이 자세)로 시작하도록 고정.
  새 타깃에서 IK가 수렴하지 않으면 가장 먼저 의심할 지점.
- 그리퍼/파지는 `arm/arm.xml` + `mujoco_receiver.py` 조합으로 구현됨 (§0.3). 단 `arm/ik.py`
  단독 실행은 여전히 엔드 이펙터의 위치 도달만 검증한다 (손가락 관절은 IK 대상이 아님).

### 0.3 손 추적 텔레오퍼레이션 파이프라인 (리포 루트)

웹캠으로 오른손을 추적해 MuJoCo 팔+그리퍼를 실시간 조종하고, 받침대 위의 큐브를 집는 데모.
UDP(127.0.0.1)로 연결된 3개 프로세스로 구성되며, 실행 순서/키 조작은 `실행방법.txt` 참고:

- `vision_sender.py` — OpenCV + MediaPipe Tasks API(PoseLandmarker/HandLandmarker)로 오른쪽
  손목 (x, y, z)와 검지/중지/약지의 curl 값(0=폄, 1=완전히 굽힘)을 추출해
  `"x y z curl_index curl_middle curl_ring"` 패킷을 UDP 5005(수신기)와 5006(대시보드) 양쪽에
  전송. 모델 파일 `pose_landmarker_full.task`/`hand_landmarker.task`가 리포 루트에 필요
  (docstring에 다운로드 URL 있음). mediapipe>=0.10.30은 레거시 `mp.solutions` API가 없어져서
  Tasks API를 사용한다.
- `mujoco_receiver.py` — UDP 5005 수신. 손목 좌표를 팔 워크스페이스로 매핑해 ikpy IK를 풀고,
  손가락 curl은 검지→finger1, 중지→finger2, 약지→finger3으로 매핑. 관절각을 qpos에 직접
  쓰지 않고 전부 `<position>` actuator ctrl로 보낸 뒤 `mj_step`으로 접촉 물리를 적분한다 —
  qpos 직접 쓰기는 손가락이 큐브를 뚫고 지나가므로 금지. 첫 패킷 수신 시 좌우 중심과 깊이
  기준을 자동 캘리브레이션 (뷰어에서 C 키로 재캘리브레이션, T 키로 그리퍼 추적 카메라 토글).
  macOS에서는 `mjpython` 필요. 좌표 매핑 스케일(`LATERAL_SCALE`/`HEIGHT_SCALE`/`BASE_Z` 등)과
  그리퍼 닫힘 각도(`MAX_CURL`)는 파일 상단 상수로 튜닝한다 — BASE_Z/큐브 높이의 기하 관계는
  해당 주석 참고.
- `web_dashboard.py` — UDP 5006 수신, `http://127.0.0.1:8000`에서 손목 좌표/curl 값을
  브라우저로 보여주는 선택사항 대시보드 (stdlib만 사용, Flask 없음).
- `run_receiver.sh` / `run_sender.sh` — venv 존재 확인 + macOS에서 mjpython 선택까지 처리하는
  실행 래퍼. 레거시 4필드 패킷(`"x y z grip"`)도 수신기가 계속 지원한다 (grip 값이 세 손가락에
  공통 적용).

---

## 1. 프로젝트 개요

### 목표
- 양팔을 가진 휴머노이드 상체 로봇 제작
- 역운동학(IK) + 강화학습(RL) 기반 물체 파지 및 배치
- AMR(자율이동로봇) 위에 탑재 예정

### 팀 구성

| 이름 | 역할 |
|---|---|
| 이재준 | 통신 + 모터 제어 |
| 강준영 | 기구 설계 |
| 김규민 | 하드웨어 |
| 정태양 (PRDX) | 팀장 / 전체 방향 |

---

## 2. 하드웨어 구성

### 연산/제어

| 부품 | 역할 |
|---|---|
| Jetson Nano P3450 | 메인 컴퓨터, micro-ROS, IK 연산 |
| STM32F407 × 2 | 오른팔/왼팔 서보 PWM 제어 |
| USB-CAN 분석기 | Jetson → AK45 CAN 통신 |
| SN65HVD230 | STM32 CAN 트랜시버 |

### 모터 구성 (한 팔 기준)

| 부위 | 모터 | 통신 | 수량 |
|---|---|---|---|
| 어깨 앞뒤/펼침 | AK45 | CAN | 2개 |
| 팔꿈치 | AK45 | CAN | 1개 |
| 상완 회전 | MG996R | PWM | 1개 |
| 손목 3축 | MG996R | PWM | 3개 |
| 손가락 (3개 × 2) | MG90S | PWM | 6개 |
| 손가락 회전 | MG90S | PWM | 1개 |

**한 팔 PWM 모터 합계: 11개 / AK45: 3개**

### 센서

| 센서 | 용도 |
|---|---|
| 앱코 APC930U 웹캠 | 물체 인식 / 위치 파악 |
| 접촉 센서 (미정) | 물체 파지 감지 |

### 전원부

| 부품 | 전압 | 용도 |
|---|---|---|
| 배터리 (미정) | 24V | 메인 전원 |
| DC-DC 컨버터 | 24V → 6V | MG996R, MG90S |
| DC-DC 컨버터 | 24V → 5V | Jetson, STM32 |

---

## 3. 소프트웨어 아키텍처

### 전체 구조

```
Jetson Nano (micro-ROS)
├── 카메라 → 물체 인식 (OpenCV / YOLO)
├── IK 연산 → 관절 각도 계산
├── USB-CAN → AK45 모터 제어 (CAN)
└── UART → STM32F407 × 2 → PWM → 서보모터
```

### STM32 펌웨어

- **RTOS**: FreeRTOS CMSIS_V2
- **통신**: UART 인터럽트 수신 (HAL_UART_Receive_IT)
- **구조**: Queue → PWM_Control_Task
- **패킷 프로토콜**: `[0xAA][motor_id][angle_H][angle_L][checksum]` 5바이트
- **PWM 설정**: 50Hz (PSC=15, ARR=19999, HSI 16MHz)
- **CCR 범위**: 500 (0°) ~ 2500 (180°)

### 핀 배정 (STM32F407)

| 모터 | 타이머 | 채널 | 핀 |
|---|---|---|---|
| 상완 회전 (MG996R) | TIM2 | CH1 | PA0 |
| 손목 축1 (MG996R) | TIM2 | CH2 | PA1 |
| 손목 축2 (MG996R) | TIM2 | CH3 | PA2 |
| 손목 축3 (MG996R) | TIM2 | CH4 | PA3 |
| 손가락1A (MG90S) | TIM3 | CH1 | PA6 |
| 손가락1B (MG90S) | TIM3 | CH2 | PA7 |
| 손가락2A (MG90S) | TIM3 | CH3 | PB0 |
| 손가락2B (MG90S) | TIM3 | CH4 | PB1 |
| 손가락3A (MG90S) | TIM4 | CH1 | PB6 |
| 손가락3B (MG90S) | TIM4 | CH2 | PB7 |
| 손가락 회전 (MG90S) | TIM4 | CH3 | PB8 |
| UART RX (Jetson) | USART1 | - | PA10 |
| UART TX (Jetson) | USART1 | - | PA9 |

---

## 4. 제어 알고리즘

### 역운동학 (IK)

- **시뮬레이션 환경**: MuJoCo 3.10.0
- **IK 라이브러리**: ikpy (예정)
- **자유도**: 7DOF (한 팔 기준)
- **목표**: 목표 위치 (x, y, z) → 관절 각도 계산 → 모터 제어

### 강화학습 (RL)

- IK로 기본 동작 안정화 후 RL 적용 예정
- 환경: MuJoCo + Gymnasium
- 목표 동작: 물체 파지 → 목표 위치 배치

### 제어 흐름

```
카메라 → 물체 위치 감지 (x, y, z)
→ IK 연산 → 관절 각도 (θ1 ~ θ7)
→ Jetson UART → STM32 PWM → 서보모터
→ Jetson USB-CAN → AK45 모터
```

---

## 5. 개발 환경

| 항목 | 내용 |
|---|---|
| IDE | VSCode + PlatformIO |
| MCU 설정 | STM32CubeMX |
| 시뮬레이션 | MuJoCo 3.10.0 |
| ROS | micro-ROS (Jetson) |
| 언어 | C (STM32), Python (Jetson) |
| OS | Ubuntu (Jetson), macOS (개발) |

---

## 6. 개발 로드맵

### Phase 1 - 통신 및 모터 제어 (현재)
- [x] STM32 FreeRTOS + PWM 코드 작성
- [x] UART 패킷 프로토콜 설계
- [ ] STM32 빌드 + 업로드 + 서보 테스트
- [ ] AK45 CAN 통신 테스트
- [ ] Jetson ↔ STM32 UART 연동

### Phase 2 - 시뮬레이션
- [x] MuJoCo 팔 모델 URDF 제작 (`arm/arm.urdf`, `arm/arm.xml`)
- [x] ikpy로 IK 구현 (`arm/ik.py`, 목표 위치 도달 오차 0에 가깝게 수렴)
- [x] 시뮬레이션에서 파지 동작 검증 (3손가락 그리퍼 + 큐브 접촉 물리, `arm/arm.xml` + `mujoco_receiver.py`)
- [x] 웹캠 손 추적 텔레오퍼레이션 (`vision_sender.py` → `mujoco_receiver.py`, §0.3)

### Phase 3 - 실제 로봇 통합
- [ ] 설계 완료 후 실제 팔에 적용
- [ ] IK → 실제 모터 연동
- [ ] 강화학습 적용

### Phase 4 - AMR 탑재
- [ ] AMR 위에 상체 탑재
- [ ] 전체 시스템 통합 테스트

---

## 7. 통신 프로토콜

### UART 패킷 (Jetson → STM32)

```
[0xAA][motor_id][angle_H][angle_L][checksum]
  1byte   1byte    1byte    1byte     1byte
```

- 헤더: `0xAA`
- motor_id: 0~10 (모터 번호)
- angle: 각도 × 10 (0~1800)
- checksum: motor_id + angle_H + angle_L 하위 1바이트

### CAN 통신 (Jetson → AK45)

- Bit Rate: 1Mbps
- 인터페이스: USB-CAN 분석기 → SocketCAN (can0)
- 라이브러리: python-can

---

## 8. 참고 자료

- [CubeMars AK Series Manual](https://www.cubemars.com)
- [MuJoCo Documentation](https://mujoco.readthedocs.io)
- [STM32F407 Reference Manual](https://www.st.com)
- [micro-ROS Documentation](https://micro.ros.org)
