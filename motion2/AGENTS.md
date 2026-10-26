<!-- Parent: ../AGENTS.md -->
<!-- Generated: 2026-07-02 | Updated: 2026-10-13 -->

# motion2/ — 웹캠 손 추적 텔레오퍼레이션 (동작 2)

> 이 문서의 경로는 `motion2/` 기준이다. 명령은 저장소 루트 기준(`venv/bin/python motion2/...`)이고, 스크립트가 `__file__` 기준 경로를 쓰므로 어느 위치에서 실행해도 된다.

## Purpose
웹캠 손목 추적 → UDP → MuJoCo 실시간 동기화 데모(7DOF 팔 + 3손가락 그리퍼, 손 모양으로
쥐기/펴기 제어) + 웹 대시보드 시각화를 포함한다. 로봇 모델은 최종 URDF(`../URDF_1006`)에서 생성한다.
Phase 2(7DOF 팔 IK + 그리퍼 kinematics + contact physics 파지: 받침대 위 큐브를 실제
접촉/마찰로 집기)까지 구현됨; Phase 3(실제 모터 연동)은 미착수.

## Key Files
| File | Description |
|------|-------------|
| `README.md` | 사람용 기준 문서: 현재 상태·한계, 설치, 실행, 조작법, 구조, UDP 패킷, 튜닝, 테스트, 알아낸 것 |
| `STATUS.md` | 동작 2의 진행 기록과 문제·해결 |

## Subdirectories
| Directory | Purpose |
|-----------|---------|
| `src/` | 프로그램 세 개: 송신기 `vision_sender.py`, 수신기 `mujoco_receiver.py`, 대시보드 `web_dashboard.py` (see `src/AGENTS.md`) |
| `scripts/` | 실행 래퍼 `run_*.sh`와 화면·웹캠 없는 검사 `check_*.py`, 뷰어 확인 `check_viewer.py` (see `scripts/AGENTS.md`) |
| `models/` | MediaPipe 모델 `pose_landmarker_full.task`, `hand_landmarker.task` (커밋됨, 합쳐 약 17 MB). 다운로드 URL은 `README.md` |
| `arm/` | 최종 URDF에서 생성한 7DOF 팔 URDF + MJCF 모델, 생성기, ikpy IK (see `arm/AGENTS.md`) |

## For AI Agents

### Working In This Directory
- Python은 반드시 `venv/bin/python` 사용 (전역 python 금지). macOS에서 MuJoCo viewer가 필요한 스크립트는 `venv/bin/mjpython` 사용.
- 새 패키지 설치 시 `venv/bin/python -m pip install <pkg>` (폴더를 옮기면 `venv/bin/pip`의 셔뱅이 깨지므로 `python -m pip`가 안전). `requirements.txt` 없음 — venv에 직접 설치.
- 코드 수정 시 루트 `CLAUDE.md`, `haness.md`의 규칙 준수 필수. `haness.md`는 AI가 지켜야 할 작업 프로토콜.
- `arm/arm.xml`, `arm/arm.urdf`, `arm/meshes/`는 생성 파일이다. 직접 고치지 말고 `arm/make_arm_urdf.py` 상수를 바꿔 다시 생성한다.
- `arm/`의 위치는 바꾸지 않는다. 동작 1이 `motion2/arm/make_arm_urdf.py`를 불러다 쓴다(`motion1/src/humanoid_grasp_rl/assets.py`, 위치 지정은 환경변수 `JAX_ARM_DIR`).
- 송신기·수신기·대시보드의 포트와 패킷 필드는 `src/AGENTS.md`의 주의를 따른다.

### Testing Requirements
- 화면·웹캠 없이: 루트의 `./run_tests.sh`가 동작 2 검사를 모두 돌린다. 개별 검사와 "무엇을 바꾸면 무엇을 돌리나"는 `src/AGENTS.md`, `scripts/AGENTS.md`. IK는 `arm/ik.py`(오차 0.00 mm).
- 웹캠은 켜지 않는다(실제 카메라 시험은 2026-10-13 `STATUS.md` 참고).
- 전체 파이프라인 확인: `scripts/run_receiver.sh` → `scripts/run_sender.sh`(+ 선택적으로 `scripts/run_web.sh`)를 각각 별도 터미널에서 실행한다 (순서와 조작법은 `README.md`).

## Dependencies

### Internal
- `../URDF_1006/` — `arm/make_arm_urdf.py`가 읽는 최종 URDF 원본 (URDF 관절 → 프로젝트 관절 이름 대응표는 `../URDF_1006/JAXV2_Assembly_description/AGENTS.md`)
- `../motion1/` — `arm/make_arm_urdf.py`의 `robot_parts()`를 불러다 쓴다(동작 1이 이 폴더에 의존, 반대는 없음). 생성기 인터페이스를 바꾸면 `motion1/src/humanoid_grasp_rl/assets.py`도 확인
- `arm/` — `src/mujoco_receiver.py`와 `arm/ik.py` 모두 `arm.urdf`, `arm.xml` 참조 (see `arm/AGENTS.md`). `arm.xml`의 그리퍼·받침대·큐브·actuator는 `arm.urdf`/ikpy 체인에는 없음 — IK와 무관한 순수 MuJoCo 전용 구조.
- `models/` — `src/vision_sender.py`가 로드하는 MediaPipe 모델 2개(손목 위치용 `pose_landmarker_full.task`, 손가락 curl용 `hand_landmarker.task`), 레포에 커밋됨 (다운로드 URL은 `README.md`)

### External
| Package | Version | 용도 |
|---------|---------|------|
| `mujoco` | 3.10.0 | 물리 시뮬레이션 및 passive viewer |
| `mediapipe` | 0.10.35 | Pose landmark 추출 (Tasks API) |
| `opencv-python` | 4.13.0 | 웹캠 프레임 캡처 및 화면 표시 |
| `ikpy` | 3.4.2 | URDF 체인 로드 및 IK 풀이 |
| `gymnasium` | 1.3.0 | reacher.xml 에셋 경로 (`scripts/check_viewer.py`) |
| `numpy`, `scipy` | — / 1.18.0 | 좌표 변환 및 오차 계산, 수신기의 팔 전체 IK(`least_squares`)와 생성기의 쿼터니언 변환 |
| (stdlib only) | — | `web_dashboard.py`는 외부 패키지 없이 `http.server` + `socket`만 사용 |

<!-- MANUAL: -->
