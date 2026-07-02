<!-- Generated: 2026-07-02 | Updated: 2026-07-02 -->

# 역운 — 휴머노이드 상체 로봇 프로젝트

## Purpose
양팔 휴머노이드 상체 로봇의 소프트웨어 레포지토리. MuJoCo 시뮬레이션 기반 IK 검증,
웹캠 손목 추적 → UDP → MuJoCo 실시간 동기화 데모, STM32/Jetson 통합 설계 문서를 포함한다.
Phase 2(7DOF 팔 IK)까지 구현됨; Phase 3(실제 모터 연동)은 미착수.

## Key Files

| File | Description |
|------|-------------|
| `CLAUDE.md` | 프로젝트 전체 설명, 하드웨어 스펙, 소프트웨어 아키텍처, 개발 로드맵, 통신 프로토콜 |
| `haness.md` | AI 작업 가이드라인 — 해야 할 것/하지 말아야 할 것, 진행 플로우, 코드 작성 규칙 |
| `vision_sender.py` | OpenCV + MediaPipe Pose로 오른손 손목(landmark 16) X/Y/Z를 추출해 UDP 5005로 전송 |
| `mujoco_receiver.py` | UDP 5005 논블로킹 수신 → MuJoCo 인라인 MJCF 내 `target` body(sphere)의 qpos 동기화 |
| `run_sender.sh` | `vision_sender.py` 실행 스크립트 (venv 체크, 안내 메시지 포함) |
| `run_receiver.sh` | `mujoco_receiver.py` 실행 스크립트 (macOS→mjpython / Linux→python 자동 분기) |
| `test.py` | gymnasium `reacher.xml`을 로드해 MuJoCo passive viewer를 띄우는 환경 검증 스크립트 |

## Subdirectories

| Directory | Purpose |
|-----------|---------|
| `arm/` | 7DOF 팔 URDF + MJCF 모델, ikpy IK 구현 및 검증 (see `arm/AGENTS.md`) |

## For AI Agents

### Working In This Directory
- Python은 반드시 `venv/bin/python` 사용 (전역 python 금지). macOS에서 MuJoCo viewer가 필요한 스크립트는 `venv/bin/mjpython` 사용.
- 새 패키지 설치 시 `venv/bin/pip install <pkg>`. `requirements.txt` 없음 — venv에 직접 설치.
- 코드 수정 시 `CLAUDE.md`, `haness.md`의 규칙 준수 필수. `haness.md`는 AI가 지켜야 할 작업 프로토콜.
- 파라미터(UDP 포트, 좌표 스케일 등) 변경 시 `vision_sender.py`와 `mujoco_receiver.py` 양쪽 모두 확인.

### Testing Requirements
- `vision_sender.py`: `venv/bin/python vision_sender.py` — 웹캠 필요, 'q' 키로 종료.
- `mujoco_receiver.py`: `venv/bin/mjpython mujoco_receiver.py` (macOS) — 로컬 디스플레이 필요.
- 두 스크립트는 별도 터미널에서 동시 실행해 구(sphere)가 손목을 따라 움직이는지 확인.
- `test.py`: `venv/bin/mjpython test.py` — gymnasium 설치 확인용.

### Common Patterns
- UDP 통신: `socket.SOCK_DGRAM`, `sock.setblocking(False)`, 패킷 포맷 `"x y z\n"` (공백 구분 float 문자열).
- MuJoCo qpos 쓰기 시 반드시 `viewer.lock()` 컨텍스트 내에서 수행 후 `mujoco.mj_forward()` 호출.
- MuJoCo passive viewer: `mj_step` 호출 없이 `viewer.sync()` + `time.sleep(0.01)` 루프.

## Dependencies

### Internal
- `arm/` — `ik.py`가 `arm.urdf`, `arm.xml` 참조

### External
| Package | Version | 용도 |
|---------|---------|------|
| `mujoco` | 3.10.0 | 물리 시뮬레이션 및 passive viewer |
| `mediapipe` | 0.10.35 | Pose landmark 추출 |
| `opencv-python` | 4.13.0 | 웹캠 프레임 캡처 및 화면 표시 |
| `ikpy` | — | URDF 체인 로드 및 IK 풀이 |
| `gymnasium` | — | reacher.xml 에셋 경로 (test.py) |
| `numpy` | — | 좌표 변환 및 오차 계산 |

<!-- MANUAL: -->
