<!-- Parent: ../AGENTS.md -->
<!-- Generated: 2026-10-13 | Updated: 2026-10-13 -->

# scripts/ — 실행 래퍼와 검사

> 이 문서의 경로는 `motion2/` 기준이다. 명령은 저장소 루트 기준이고, 어느 폴더에서 실행해도 된다.

## Purpose
`src/`의 프로그램 세 개를 띄우는 래퍼(`run_*.sh`)와, 화면·웹캠 없이 파이프라인을 확인하는 검사(`check_*.py`)다. 검사는 첫 줄에서 `_paths`를 import해 `src/`를 경로에 넣고, 래퍼는 자기 위치에서 저장소 루트를 찾아 절대 경로로 실행한다. 둘 다 현재 폴더에 의존하지 않는다.

## Key Files
| File | Description |
|------|-------------|
| `_paths.py` | `src/`를 `sys.path`에 넣고 `ROOT`(`motion2/`)를 제공. 새 검사는 맨 위에서 `import _paths` |
| `run_receiver.sh` | `src/mujoco_receiver.py` 실행 (venv 확인, macOS→`mjpython` / Linux→`python` 자동 분기) |
| `run_sender.sh` | `src/vision_sender.py` 실행 (venv 확인, 안내 메시지) |
| `run_web.sh` | `src/web_dashboard.py` 실행 (venv 확인만, 추가 패키지 불필요) |
| `check_teleop.py` | 화면·웹캠 없이 UDP 파이프라인 전체 검사: 가짜 송신 스레드가 60 Hz로 패킷을 보내고(깨진 패킷·NaN 위치 포함) 수신기 `main()`이 그대로 돌며 큐브를 집어 손을 85 mm 올린다(큐브 중심 z 0.265 → 0.347~0.349). 시나리오 3개(`wrist`, `arm`, `swing-jump`), `teleop: 3/3 scenarios passed`가 기대값. 수신기의 IK·스윙·서보·접촉을 바꿨으면 반드시 실행 |
| `check_sender.py` | 웹캠 없이 송신기 검사: 굽힘 계산(펴짐 0, 90° 0.833, 접힘 1), 두 `.task` 모델이 빈 프레임에서 로드·실행됨, 가짜 카메라·랜드마커로 `main()`을 돌려 패킷이 두 포트에 같은 내용으로 12/6/12필드로 나가는지, 가시성 낮은 프레임을 버리는지, 손이 사라지면 굽힘을 유지하는지, 수신기 파서가 받아들이는지 확인 |
| `check_dashboard.py` | `web_dashboard.py`를 서브프로세스로 띄워 6/12/4필드 패킷 반영, 잘못된 패킷 무시, `/wrist.json`·페이지 응답 확인 |
| `check_grasp.py` | 화면 없는 파지 검증: 큐브 ±8 mm 어긋난 9곳에서 닫고 들어올리기, `lifted 9 /9`가 기대값 (손가락 기하·마찰·`FINGER_*`/`DISTAL_SCALE`을 바꿨으면 반드시 실행) |
| `check_viewer.py` | gymnasium `reacher.xml`을 로드해 MuJoCo passive viewer를 띄우는 환경 확인(예전 이름 test.py). 화면이 필요하고 macOS는 `mjpython` |

## For AI Agents

### Working In This Directory
- 새 검사: 첫 줄에 `import _paths  # noqa: F401`, 성공하면 기대 출력 한 줄(`... ok`)을 찍고 실패하면 0이 아닌 종료 코드를 내며, 루트 `run_tests.sh`에 `step`을 추가한다(종료 코드와 출력 내용을 둘 다 본다).
- 새 래퍼: 위치에서 저장소 루트를 찾아(`ROOT`) 절대 경로로 실행한다. 상대 경로(`../venv`)나 `cd`에 기대지 않는다.
- 검사는 웹캠을 켜지 않는다. UDP 5005/5006과 HTTP 8000을 쓰므로 한 번에 하나씩 돌린다(`run_tests.sh`가 순서대로 돌린다).
- 뷰어를 여는 것(`run_receiver.sh`, `check_viewer.py`)은 화면이 켜져 있어야 한다. 꺼져 있으면 GLFW가 모니터를 못 찾아 `Segmentation fault`로 죽는다(`README.md`의 "주의").

### Testing Requirements
- `venv/bin/python motion2/scripts/check_<이름>.py` 각각, 또는 루트 `./run_tests.sh`. 기대 출력은 `src/AGENTS.md`의 표.
- 래퍼는 `./run_tests.sh --viewers`(수신기 뷰어가 9초 이상 살아 있는지)와 문법 검사 `bash -n`(빠른 모드가 돌린다)으로 확인한다.

## Dependencies
- Internal: `src/`, `models/`(`check_sender.py`가 모델 로딩을 확인), `arm/`
- External: `mujoco`, `numpy`, `mediapipe`, `opencv-python`(`check_sender.py`), `gymnasium`(`check_viewer.py`)

<!-- MANUAL: -->
