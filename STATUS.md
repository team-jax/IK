# STATUS

최종 갱신: 2026-11-09

> 저장소 전체의 요약과 공통 사항이다. 동작별 기록은 동작 1 `motion1/STATUS.md`, 동작 2 `motion2/STATUS.md`에 있다. 원격은 <https://github.com/team-jax/IK>(`origin`)이고, 동작 1(`motion1/`)도 2026-11-01부터 여기에 들어 있다(원래 `gyumingim/Humanoid_Grasp_RL`에서 개발).

## 전체 목표

웹캠 손목·손가락 추적으로 최종 URDF(`URDF_1108/JAXV3_Assembly_description`, 2026-11-09부터. 질량과 손가락 충돌은 이전 `URDF_1006`) 기반 7DOF 팔과 3손가락 손을 MuJoCo에서 제어하고 큐브를 파지한다 (동작 2, `motion2/`). 동작 1(`motion1/`)도 같은 URDF의 팔·손으로 스크립트 컨트롤러를 만들어 큐브를 배달·쌓는다 (2026-10-12 전환, 자세한 내용은 `motion1/README.md`·`STATUS.md`).

## 동작별 현황

- 동작 1 (`motion1/`): 최종 URDF V3(2026-11-09)에서 배달 73/100(시드 0~99), 2단 쌓기 10/50(시드 0~49, 24개는 고정 배치로 대체되고 그 배치가 실패). V2에서는 68/100, 39/50이었고(패드–큐브 마찰 0.4 실측, 2026-11-04), 마찰 2.0이던 때는 100/100, 46/50. 2단 하락은 고정 배치 하나의 성패가 작은 변화로 뒤집히는 탓이 크다(V2에서도 HOME만 0.02 rad 미만 바꾸면 12/50). `run_tests.sh`의 `m1_smoke`(2단), `m1_run_real` 예제와 `--full`의 2단 기대값 39가 지금 실패한다: 기준을 바꿀지는 사용자 결정(`motion1/STATUS.md`). 실물 배치(`JAX_SETUP=real`: 바닥에서 어깨 회전축 1212.5 mm, 책상 735 mm, 책상은 거치대 FRONT 쪽 앞, 어깨 축에서 22.5 cm)는 지금 손으로 배달 2/100, 2단 0/50이라 그리퍼 변경 대기(`motion1/STATUS.md`). 한계는 `motion1/README.md`
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
- 동작 1의 한계(탑 2단까지, 도달 띠가 y -0.28~+0.05로 치우침)는 `motion1/README.md` 참고
- 최종 URDF V3 전환 뒤 동작 1의 2단 기대값(`run_tests.sh`의 39, `m1_smoke` 2단, `m1_run_real` 예제)을 새 성적에 맞출지, 동작 1을 V3에 맞게 다시 조정할지 결정(`motion1/STATUS.md`)
- 팔 실제 질량 재기(지금은 V2 질량). 실물 배치는 JaxIsaacLab deploy와 같게 맞췄다(2026-11-10: 어깨 1212.5 mm, 책상 FRONT 쪽 22.5 cm)

## 문제점과 해결 (공통·동작 1)

동작 2의 문제와 해결(폴더 정리, 팔꿈치 스윙, 파지 창, 문서 수치)은 `motion2/STATUS.md`에 있다.

2026-11-10 (실물 배치를 JaxIsaacLab `deploy_2026-11-05`와 같게, 사용자 지시: 책상은 거치대 FRONT 쪽 앞이 맞고 어깨 높이는 1212.5 mm):

- JaxIsaacLab의 실물 배포 사양과 비교: 로봇 모델(V3 + V2 부호·충돌)은 같았고, 다른 것은 책상 방향(그쪽은 FRONT 쪽, 우리는 어깨가 튀어나온 쪽), 어깨 높이(1212.5 / 1207.5 mm), 어깨→책상 앞 모서리(22.5 / 약 −1 cm)였다. 셋 다 그쪽에 맞춤(`motion1/STATUS.md`). 시뮬 배치(`sim`)는 장면 XML까지 그대로
- 바꾸는 중 찾은 것: 가장 넓은 도달 띠를 주는 HOME 후보(`find_home.py`의 1등)가 실제 컨트롤러로는 큐브에 가지 못했다(어깨 롤 한계에 붙은 자세에서 팔꿈치가 0°로 펴지며 멈춤). 도달 맵은 자세마다 한 번에 풀어서 이것을 못 본다 → 후보를 30번씩 실제로 돌려 큐브를 집는 후보를 골랐다

2026-11-09 (최종 URDF V3 `URDF_1108`로 전환, 사용자 지시: 아래팔 롤 축 방향과 팔 질량은 V2 그대로, 나머지는 V3대로):

- 생성기가 V3를 읽게 바꾸고(`motion2/STATUS.md`의 변경 이력), 바꾸기 전 기준선(`run_tests.sh --full` 31/31, 배달 68/100, 2단 39/50 등)과 같은 명령으로 다시 쟀다(`motion1/STATUS.md`).
- 찾아서 고친 것: ① `motion1/scripts/build_reachmap.py`가 11-05 커밋 `298106d` 뒤로 회전 없는 도달 지도를 저장하지 않아 `--full`의 "다시 만든 지도 == 레포 지도" 검사가 늘 통과했다 → 저장을 되살림 ② 실물 배치 HOME의 팔꿈치가 V2 한계 105°에 붙어 있어 V3(±100°) 범위 밖 → `find_home.py`로 다시 찾음 ③ `map_signature()`에 로봇 모델이 들어 있지 않아 모델이 바뀌어도 낡은 지도 경고가 나지 않는다 → 문서에 적고 세 지도를 다시 만듦(시그니처 자체는 그대로) ④ `run_tests.sh`의 `delivered()`가 마지막 줄이 "N delivered"로 시작할 때만 숫자를 읽어, 집계가 "39 placed short,  10 delivered" 순서로 나오자 2단을 0/50으로 보고했다 → 줄 어디에 있든 읽게 고침(기준값은 그대로)
- V3와 다시 대조(사용자 지시: 수직 높이·질량·부호만 V2, 나머지는 모두 V3): 관절 원점·축·범위, 외형 메시, 손가락 위치, 파지 중심은 이미 V3였다. 손가락 충돌은 V2 STL을 쓰지만 V3의 실제 손가락 모양(외형 메시)과 같고, V3 자체의 충돌 STL은 내보내기 도구가 만든 원기둥이다(`robot_data.yaml`의 `collision_source: primitive`). 남아 있던 V2 값은 책상 위치를 정하는 거치대 면(`stand_surface_x`, V2 거치대에서 읽음) 하나였다 → V3 기둥 면 x 0.45로 바꿈. 성적 출력은 바이트까지 같다(`motion1/STATUS.md`)
- SimToReal과 team-jax/jax_ros2의 보정 파일(calibration.json의 sim_range_deg와 AK45 limit_deg)도 `shoulder_yaw` 0~50°, `elbow` ±100°로 맞춰야 그쪽 시험(check_protocol.py가 이 레포의 `arm.xml`과 대조)이 통과한다.

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
