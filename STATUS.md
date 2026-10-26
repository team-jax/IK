# STATUS

최종 갱신: 2026-10-26

> 저장소 전체의 요약과 공통 사항이다. 동작별 기록은 동작 1 `motion1/STATUS.md`, 동작 2 `motion2/STATUS.md`에 있다. 원격은 <https://github.com/team-jax/IK>(`origin`)이고, 동작 1(`motion1/`)은 별도 저장소라 거기에는 없다.

## 전체 목표

웹캠 손목·손가락 추적으로 최종 URDF(`URDF_1006/JAXV2_Assembly_description`) 기반 7DOF 팔과 3손가락 손을 MuJoCo에서 제어하고 큐브를 파지한다 (동작 2, `motion2/`). 동작 1(`motion1/`)도 같은 URDF의 팔·손으로 스크립트 컨트롤러를 만들어 큐브를 배달·쌓는다 (2026-10-12 전환, 자세한 내용은 `motion1/README.md`·`STATUS.md`).

## 동작별 현황

- 동작 1 (`motion1/`): 같은 URDF로 교체 완료. 배달 100/100(시드 0~99), 2단 쌓기 46/50(시드 0~49, 24개는 고정 배치로 대체). 한계는 `motion1/README.md`
- 동작 2 (`motion2/`): 모델 생성, IK 오차 0.00 mm, 파지 9/9(±8 mm), UDP 파이프라인 전체 3/3. 실제 웹캠은 열리고 송신 루프가 초당 27.8프레임으로 돌지만 사람 추적은 아직 못 봄. 자세한 내용은 `motion2/README.md`·`STATUS.md`

## 저장소 공통

전체 리포 재검증(2026-10-13): 이제 `./run_tests.sh` 한 번으로 돈다 (빠른 모드 20개 항목, `--full`은 성공률·영상 포함, `--viewers`는 뷰어 포함). 항목은 종료 코드와 출력 내용을 함께 본다. 이 검사들이 실제로 실패를 잡는지도 확인했다(`DISTAL_SCALE`을 0.3으로 되돌리면 `lifted 2 /9`, 스윙 고정을 끄면 `swing-jump` 실패).

- 동작 1 `smoke_test`, `build_reachmap`(저장된 맵과 바이트 단위로 같음), `tune_grasp`, `layouts`, `render_frame`, `record_delivery`(배달·쌓기 영상), `classify_failures`(배달 100/100, 쌓기 46/50 재현)
- 동작 2 모델 생성, `ik.py`, 검사 4개(파지 9/9, 송신기, UDP 파이프라인 3/3, 대시보드)
- 모든 `.py` 컴파일, `.sh` 문법, 문서의 기호·경로·링크를 코드와 대조
- 뷰어를 띄우는 `watch_grasp.py`와 `motion2/scripts/run_receiver.sh`가 macOS에서 실행되어 10초 이상 살아 있음을 확인 (창을 눈으로 본 것은 아님)

## 다음 할 일

- `.omc/`·`.omo/`(도구 상태·세션 기록)가 이미 git에 추적되어 team-jax/IK에 올라가 있다. `.gitignore`에 넣고 추적을 풀지 정해야 한다 (풀어도 이미 올라간 커밋 기록에는 남는다)
- 동작 2: `motion2/STATUS.md`의 "다음 할 일" (사람 추적 육안 확인, 작업 영역 가장자리의 IK 해 점프)
- 동작 1의 `watch_grasp.py` 뷰어 재생을 육안 확인
- 동작 1의 한계(탑 2단까지, 도달 띠가 y -0.28~+0.03으로 치우침)는 `motion1/README.md` 참고

## 문제점과 해결 (공통·동작 1)

동작 2의 문제와 해결(폴더 정리, 팔꿈치 스윙, 파지 창, 문서 수치)은 `motion2/STATUS.md`에 있다.

2026-10-26:

- 문서는 `motion2/` 폴더 정리(`src/`·`scripts/`·`models/`)가 끝났다고 적었지만 git에는 이름 바꾸기만 스테이지되고 `motion2/README.md`·`STATUS.md` 등은 untracked였음 → 커밋 `340d292`로 함께 묶어 team-jax/IK(빈 저장소)의 `main`에 푸시. `origin`을 `team-jax/CV_TEST2`에서 `team-jax/IK`로 바꾸고 CV_TEST2는 `upstream`으로 남겼다. `.omc/` 세션 파일, `.vscode/`, `motion1/`은 올리지 않았다
- 올린 저장소를 새로 클론해 `./run_tests.sh`(빠른 모드)를 돌려 보니 `motion1/`이 없어서 문서 대조(`check_docs.py`)만 실패(35건, 전부 `motion1/` 파일과 거기서만 정의되는 이름을 가리킴)하고 동작 2 검사는 모두 통과 → `motion1/`이 없으면 `motion1/` 전용 참조를 대조에서 빼고 몇 건을 건너뛰었는지 출력하도록 `check_docs.py`를 고침(`motion1/`이 있으면 이전과 같이 전부 검사). README·AGENTS·CLAUDE의 동작 1 설명에는 "별도 저장소, team-jax/IK에는 없음"을 적고 README의 `motion1/` 링크(GitHub에서는 열리지 않음)는 경로 표기로 바꿈
- `motion2/README.md`가 URDF 폴더를 `JAXV2_Assembly`로 적고 있었음(실제는 `JAXV2_Assembly_description`) → 정정

2026-10-13:

- `run_tests.sh --viewers` 중 수신기 뷰어가 `Segmentation fault: 11`로 한 번 죽음(같은 검사를 6번 다시 돌리면 모두 정상) → macOS 충돌 보고서의 스택이 `glfwGetVideoMode`(MuJoCo 뷰어 생성 중, 모니터 포인터가 NULL)였고 전원 로그에서 그 시각에 디스플레이가 꺼져 있었음(08:41:57 꺼짐, 08:47:16 켜짐, 충돌 08:46:26) → 저장소 버그가 아니라 화면이 꺼지거나 잠긴 동안 GLFW가 모니터를 못 찾는 환경 문제. 뷰어를 쓰는 실행·검사는 화면을 켠 채로 한다. `run_tests.sh`는 이 경우(종료 코드 139)를 알아보고 안내한다
- 빠른 모드의 `venv_tools` 단계가 `mjpython -c 'print(1)'`을 실행했는데, `mjpython`은 Cocoa 호스트를 띄워서 화면이 잠기면(`IOConsoleLocked`) 10분이 넘게 멈춤(일반 파이썬은 0.0초) → 빠른 모드가 화면 없이 돈다는 약속과 어긋났음. `venv/bin` 스크립트의 셔뱅이 가리키는 인터프리터가 있는지 직접 확인하도록 바꿈. 폴더 이름을 바꾼 뒤 셔뱅이 깨지는 것을 잡는 원래 목적은 그대로이고, 깨진 셔뱅을 넣으면 실패하는 것도 확인했다. 화면이 잠긴 채 빠른 모드 20/20 통과(오프스크린 렌더 포함). 실제 뷰어는 화면을 켠 채 `--viewers`로 확인한다
- `venv/bin/pip`, `mjpython` 등 13개 실행 파일이 옛 폴더("역운")를 셔뱅으로 가리켜 실행 불가 → 현재 경로로 셔뱅과 `activate*`를 다시 씀. 폴더를 옮기거나 이름을 바꾸면 다시 깨진다(그 경우 `python -m venv --upgrade` 또는 venv 재생성)
- 동작 1 스크립트가 `PYTHONPATH=src`와 실행 위치에 의존, `motion1/.venv`는 없는데 문서가 안내 → `scripts/_paths.py`로 어디서나 실행 가능하게 하고 문서를 루트 `venv/` 기준으로 정정
- 동작 1 `tune_grasp.py`가 박스 충돌 geom(`dataid=-1`)을 메시로 읽어 엉뚱한 정점으로 간격을 잼(g=0에서 17.6 mm, 접촉 g=-1.22) → 박스는 모서리, 메시는 정점을 쓰도록 수정. 이제 g=0에서 89.7 mm, `CUBE_GRIP`=0.86에서 50 mm로 `env.py` 주석과 일치
- 동작 1 `classify_failures.py`가 큐브가 50 mm 올라야 "집음"으로 판정(환경은 30 mm) → 시드 97(49 mm 들림)이 성공인데 `picked=False`로 표시 → 환경 기준(`LIFT_REQUIRED`)을 사용
