<!-- Generated: 2026-07-02 | Updated: 2026-10-13 -->

# 역운(逆運) — 휴머노이드 상체 로봇 프로젝트 (Team JAX)

## Purpose
물건을 집어 목표 위치에 놓는 휴머노이드 상체 로봇의 소프트웨어 레포지토리. 팀의 최종 URDF(`URDF_1006`)를 원본으로, 두 개의 독립된 시뮬레이션 작업(동작 1, 동작 2)이 같은 로봇(7DOF 팔 + 3손가락 손)을 쓴다. 실제 하드웨어(STM32/Jetson/AK45)는 설계만 `CLAUDE.md`에 있고 코드는 아직 없다.

## Key Files
| File | Description |
|------|-------------|
| `CLAUDE.md` | 프로젝트 전체 설계: 동작 1/2 구분, 하드웨어, 프로토콜, 로드맵. 작업의 1순위 기준 |
| `README.md` | 사람용 소개와 동작 2 실행법 |
| `STATUS.md` | 동작 2 중심의 진행 상황·문제와 해결 기록 |
| `run_tests.sh` | 전체 테스트 한 번에 (`--full`, `--viewers`). 단계는 종료 코드와 출력 내용을 둘 다 본다 |
| `check_docs.py` | 문서의 기호·경로·링크를 코드와 대조 |
| `haness.md` | AI 작업 규칙(근거 없는 단정 금지, 최소 복잡도, 변경 이유 한 줄). 큰 범위 변경은 사전 승인 |
| `venv/` | 두 동작이 공용하는 Python 3.14 가상환경 (git 추적 안 함) |

## Subdirectories
| Directory | Purpose |
|-----------|---------|
| `URDF_1006/` | 팀의 최종 URDF(CAD export). 두 동작이 공용하는 원본 (see `URDF_1006/AGENTS.md`) |
| `motion1/` | 동작 1: 스크립트 상태기계로 50 mm 큐브를 집어 배달·2단 쌓기. **별도 git 저장소** (see `motion1/AGENTS.md`) |
| `motion2/` | 동작 2: 웹캠 손 추적 텔레오퍼레이션 + 팔 모델 생성기 (see `motion2/AGENTS.md`) |

`AGENTS_files/`, `motion1/README_files/`는 Quarto 렌더 부산물이라 문서화 대상이 아니다. `.omc/`, `.omo/`는 도구 상태 폴더다.

## For AI Agents

### Working In This Directory
- 의존 방향은 `motion1 → motion2/arm/make_arm_urdf.py → URDF_1006` 하나뿐이다. motion2는 motion1을 import하지 않는다.
- Python은 항상 루트 `venv/bin/python`, macOS에서 MuJoCo 뷰어를 띄우는 스크립트는 `venv/bin/mjpython`. 패키지는 `venv/bin/python -m pip install`. 폴더를 옮기거나 이름을 바꾸면 `venv/bin/pip`·`mjpython` 등의 셔뱅이 깨진다(`STATUS.md` 참고).
- `motion1/`은 자체 `.git`이 있다. 루트 저장소에서는 untracked로 보이며, 동작 1 변경은 그 안에서 커밋한다.
- 문서와 코드가 어긋나면 코드와 실제 실행 결과가 기준이다. 숫자(성공률 등)는 측정한 날짜와 시드 범위를 함께 적는다.

### Testing Requirements
저장소 루트에서, 화면·웹캠 없이: `./run_tests.sh` (빠른 모드, 약 3분). 성공률과 영상까지 보려면 `--full`, 뷰어를 열어 보려면 `--viewers`. 코드나 문서를 고친 뒤에는 반드시 돌린다. 개별 검사:
- `venv/bin/python motion1/scripts/smoke_test.py`
- `venv/bin/python motion2/arm/ik.py` (IK 오차 0.00 mm)
- `venv/bin/python motion2/check_grasp.py` (`lifted 9 /9`), `check_teleop.py` (`3/3`), `check_sender.py`, `check_dashboard.py`
- `venv/bin/python check_docs.py` (문서 대조)

### Common Patterns
- 로봇 모델 파일(`motion2/arm/arm.xml`, `arm.urdf`, `meshes/`)은 생성물이다. 고칠 때는 `motion2/arm/make_arm_urdf.py`의 상수를 바꾸고 다시 생성한다.

## Dependencies
- `mujoco`, `numpy`, `gymnasium`, `ikpy`, `scipy`, `mediapipe`, `opencv-python`, `imageio` (모두 `venv/`에 직접 설치, `requirements.txt` 없음. `motion1/pyproject.toml`에 동작 1 의존성 고정값이 있다)

<!-- MANUAL: -->
