<!-- Generated: 2026-07-02 | Updated: 2026-10-26 -->

# 역운(逆運) — 휴머노이드 상체 로봇 프로젝트 (Team JAX)

## Purpose
물건을 집어 목표 위치에 놓는 휴머노이드 상체 로봇의 소프트웨어 레포지토리. 팀의 최종 URDF(`URDF_1006`)를 원본으로, 두 개의 독립된 시뮬레이션 작업(동작 1, 동작 2)이 같은 로봇(7DOF 팔 + 3손가락 손)을 쓴다. 실제 하드웨어(STM32/Jetson/AK45)는 설계만 `CLAUDE.md`에 있고 코드는 아직 없다.

## Key Files
| File | Description |
|------|-------------|
| `CLAUDE.md` | 프로젝트 전체 설계: 동작 1/2 구분, 하드웨어, 프로토콜, 로드맵. 작업의 1순위 기준 |
| `README.md` | 사람용 소개: 두 동작 개요, 구조, 테스트, 로드맵. 동작별 상세는 `motion1/README.md`, `motion2/README.md` |
| `STATUS.md` | 전체 요약과 저장소 공통 기록(테스트 도구, 환경 문제). 동작별 기록은 각 폴더의 `STATUS.md` |
| `run_tests.sh` | 전체 테스트 한 번에 (`--full`, `--viewers`). 단계는 종료 코드와 출력 내용을 둘 다 본다 |
| `check_docs.py` | 문서의 기호·경로·링크를 코드와 대조. `motion1/`이 없으면(team-jax/IK 클론) 그쪽 전용 참조는 건너뛴다 |
| `haness.md` | AI 작업 규칙(근거 없는 단정 금지, 최소 복잡도, 변경 이유 한 줄). 큰 범위 변경은 사전 승인. 상단 메모대로 `motion1/HARNESS.md`의 구판이라 다른 프로젝트 예시가 남아 있다 |
| `.gitignore` | `__pycache__/`, `*.pyc`, `.DS_Store` 제외 |
| `AGENTS.html` | 이 문서의 Quarto 렌더본(git 추적, 부산물 `AGENTS_files/`와 함께) |
| `venv/` | 두 동작이 공용하는 Python 3.14 가상환경 (git 추적 안 함) |

## Subdirectories
| Directory | Purpose |
|-----------|---------|
| `URDF_1006/` | 팀의 최종 URDF(CAD export). 두 동작이 공용하는 원본 (see `URDF_1006/AGENTS.md`) |
| `motion1/` | 동작 1: 스크립트 상태기계로 50 mm 큐브를 집어 배달·2단 쌓기. **별도 git 저장소이며 team-jax/IK에는 올라가지 않았다** (로컬에 있을 때만 see `motion1/AGENTS.md`) |
| `motion2/` | 동작 2: 웹캠 손 추적 텔레오퍼레이션(`src/`, `scripts/`, `models/`) + 팔 모델 생성기(`arm/`), 자체 README·STATUS (see `motion2/AGENTS.md`) |

문서 계층(`Parent:` 태그로 이어지고 `check_docs.py`가 검사한다):

```
AGENTS.md
├── URDF_1006/AGENTS.md
│   └── JAXV2_Assembly_description/AGENTS.md
├── motion1/AGENTS.md          (별도 저장소, team-jax/IK에는 없음)
│   ├── scripts/AGENTS.md
│   └── src/humanoid_grasp_rl/AGENTS.md
└── motion2/AGENTS.md
    ├── src/AGENTS.md
    ├── scripts/AGENTS.md
    └── arm/AGENTS.md
```

문서를 두지 않는 폴더: `AGENTS_files/`, `motion1/README_files/`(Quarto 렌더 부산물), `.omc/`·`.omo/`·`.claude/`(도구 상태·설정), `.vscode/`(편집기 설정), 생성물·데이터·산출물 폴더(`motion2/arm/meshes/`, `motion2/models/`, `motion1/artifacts/`, `URDF_1006/JAXV2_Assembly_description/` 아래의 `config/ docs/ images/ launch/ meshes/ rviz/ urdf/`).

## For AI Agents

### Working In This Directory
- 의존 방향은 `motion1 → motion2/arm/make_arm_urdf.py → URDF_1006` 하나뿐이다. motion2는 motion1을 import하지 않는다.
- Python은 항상 루트 `venv/bin/python`, macOS에서 MuJoCo 뷰어를 띄우는 스크립트는 `venv/bin/mjpython`. 패키지는 `venv/bin/python -m pip install`. 폴더를 옮기거나 이름을 바꾸면 `venv/bin/pip`·`mjpython` 등의 셔뱅이 깨진다(`STATUS.md` 참고).
- 원격 저장소는 `origin` = <https://github.com/team-jax/IK>(`main`). 이전 `team-jax/CV_TEST2`는 `upstream`으로 남아 있다.
- `motion1/`은 자체 `.git`(원격 `gyumingim/Humanoid_Grasp_RL`)이 있다. 루트 저장소에서는 untracked로 보이고 team-jax/IK에는 올리지 않으며, 동작 1 변경은 그 안에서 커밋한다. `motion1/`이 없는 클론에서도 `motion2/`는 그대로 동작한다.
- 문서와 코드가 어긋나면 코드와 실제 실행 결과가 기준이다. 숫자(성공률 등)는 직접 재현한 것만, 측정한 날짜와 시드 범위를 함께 적는다.
- 새 `AGENTS.md`를 만들면 첫 줄에 부모를 가리키는 `Parent` 주석 태그(다른 `AGENTS.md`와 같은 형식)를 달고 `check_docs.py`의 `DOCS` 목록에도 넣는다. 부모 경로는 모든 `AGENTS.md`에서 검사하지만 기호·경로 대조는 목록의 문서만 한다. 검사기는 문서에서 처음 나오는 "Parent" + 콜론 + 공백 뒤의 단어를 부모 경로로 읽으므로 본문에 그 형식의 예시를 적지 않는다. 이 파일을 고쳤으면 `quarto render AGENTS.md`로 `AGENTS.html`도 다시 만든다.

### Testing Requirements
저장소 루트에서, 화면·웹캠 없이: `./run_tests.sh` (빠른 모드, 약 1분). 성공률과 영상까지 보려면 `--full`, 뷰어를 열어 보려면 `--viewers`. 코드나 문서를 고친 뒤에는 반드시 돌린다. 개별 검사:
- `venv/bin/python motion1/scripts/smoke_test.py`
- `venv/bin/python motion2/arm/ik.py` (IK 오차 0.00 mm)
- `venv/bin/python motion2/scripts/check_grasp.py` (`lifted 9 /9`), `check_teleop.py` (`3/3`), `check_sender.py`, `check_dashboard.py` (같은 폴더)
- `venv/bin/python check_docs.py` (문서 대조)

### Common Patterns
- 로봇 모델 파일(`motion2/arm/arm.xml`, `arm.urdf`, `meshes/`)은 생성물이다. 고칠 때는 `motion2/arm/make_arm_urdf.py`의 상수를 바꾸고 다시 생성한다.

## Dependencies
- `mujoco`, `numpy`, `gymnasium`, `ikpy`, `scipy`, `mediapipe`, `opencv-python`, `imageio` (모두 `venv/`에 직접 설치, `requirements.txt` 없음. `motion1/pyproject.toml`에 동작 1 의존성 하한이 있다)

<!-- MANUAL: -->
