# STATUS

최종 갱신: 2026-10-13

> **구조 안내:** 이 문서의 파일 경로(`arm/`, `vision_sender.py`, `run_*.sh` 등)는 모두 `motion2/`(동작 2) 기준이다. 동작 1은 `motion1/`(자체 `STATUS.md`). 루트 `venv/`를 공용으로 쓴다.

## 전체 목표

웹캠 손목·손가락 추적으로 최종 URDF(`URDF_1006/JAXV2_Assembly_description`) 기반 7DOF 팔과 3손가락 손을 MuJoCo에서 제어하고 큐브를 파지한다 (동작 2, `motion2/`). 동작 1(`motion1/`)도 같은 URDF의 팔·손으로 스크립트 컨트롤러를 만들어 큐브를 배달·쌓는다 (2026-10-12 전환, 자세한 내용은 `motion1/README.md`·`STATUS.md`).

## 완료된 일

동작 2:

- `motion2/arm/make_arm_urdf.py`가 최종 URDF에서 `arm.urdf`(ikpy 체인), `arm.xml`(MuJoCo), `meshes/`를 생성
- ikpy FK ↔ MuJoCo `end_effector` site 일치(1e-9), IK 오차 0.00 mm
- 수신기(`mujoco_receiver.py`)를 새 관절 구성(7번째 관절이 손 회전, 전완 회전 없음)·손가락 기하에 맞춰 수정
- 화면 없는 시뮬레이션: 큐브가 ±8 mm 어긋나도 접근→하강→닫기→들어올리기 9/9 성공(`motion2/check_grasp.py`), 수신기 `main()` 루프 UDP E2E 통과, 대시보드 HTTP 응답 확인

동작 1:

- 같은 URDF로 교체 완료: 배달 100/100(시드 0~99), 2단 쌓기 46/50(시드 0~49, 24개는 고정 배치로 대체)

전체 리포 재검증(2026-10-13):

- 동작 1 `smoke_test`, `build_reachmap`(저장된 맵과 바이트 단위로 같음), `tune_grasp`, `layouts`, `render_frame`, `record_delivery`(배달·쌓기 영상), `classify_failures`(배달 100/100, 쌓기 46/50 재현)
- 동작 2 모델 생성, `ik.py`, `check_grasp.py`(9/9), 수신기 E2E, 대시보드
- 모든 `.py` 컴파일, `.sh` 문법, 문서의 기호·경로·링크를 코드와 대조
- 뷰어를 띄우는 `watch_grasp.py`와 `run_receiver.sh`가 macOS에서 실행되어 10초 이상 살아 있음을 확인 (창을 눈으로 본 것은 아님)

## 현재 상태

- 모델: `nq=20`, `nu=13`, MJCF 메시 16개(`meshes/`에는 OBJ 16개 + 충돌 STL 6개, 박스 충돌이라 STL은 `JAX_FINGER_COLLISION=mesh`로 생성할 때만 로드)
- 손가락 판은 손 x 방향으로 얇고, 손가락 1·2와 손가락 3(`ROTATION_BASE_ANGLE=-π`)이 마주 본다. 판 안쪽 면 사이 간격은 근위 q=0에서 30 mm, 열림 q=-0.25에서 큐브 높이 기준 약 75~83 mm (2026-10-13 측정)
- 손가락 닫힘은 `FINGER_CLOSED=0.15`, `DISTAL_SCALE=0.1`. 0.3이면 판이 기울어 큐브 모서리만 물고 들어올릴 때 미끄러진다
- 큐브 30 mm는 윗면 z=0.25인 좁은 기둥 위. 판 끝은 파지 중심(`end_effector`)보다 17.5 mm 아래라, `BASE_Z=0.255`(큐브 바닥보다 5 mm 위)에서 판 끝이 큐브 바닥보다 약 12.5 mm 아래로 내려가 평평한 패드면으로 집는다
- palm-down 도달 영역은 z=0.25~0.34에서 넓고, 0.38 이상 중앙은 불가 → `WORKSPACE_Z=(0.255, 0.34)`

## 다음 할 일

- 실제 디스플레이를 보며 `run_receiver.sh` / `run_sender.sh`로 팔 움직임과 파지를 육안 확인 (웹캠 필요)
- 동작 1의 `watch_grasp.py` 뷰어 재생을 육안 확인
- 동작 1의 한계(탑 2단까지, 도달 띠가 y -0.28~+0.03으로 치우침)는 `motion1/README.md` 참고

## 문제점과 해결

2026-10-13:

- 들어올리면 큐브가 손가락 사이에서 빠짐(`check_grasp.py` 0/9) → 접촉 시 `DISTAL_SCALE=0.3`이 판을 기울여 모서리 접촉만 남음 → `FINGER_CLOSED=0.15`, `DISTAL_SCALE=0.1`로 조정해 9/9
- `venv/bin/pip`, `mjpython` 등 13개 실행 파일이 옛 폴더("역운")를 셔뱅으로 가리켜 실행 불가 → 현재 경로로 셔뱅과 `activate*`를 다시 씀. 폴더를 옮기거나 이름을 바꾸면 다시 깨진다(그 경우 `python -m venv --upgrade` 또는 venv 재생성)
- 동작 1 스크립트가 `PYTHONPATH=src`와 실행 위치에 의존, `motion1/.venv`는 없는데 문서가 안내 → `scripts/_paths.py`로 어디서나 실행 가능하게 하고 문서를 루트 `venv/` 기준으로 정정
- 동작 1 `tune_grasp.py`가 박스 충돌 geom(`dataid=-1`)을 메시로 읽어 엉뚱한 정점으로 간격을 잼(g=0에서 17.6 mm, 접촉 g=-1.22) → 박스는 모서리, 메시는 정점을 쓰도록 수정. 이제 g=0에서 89.7 mm, `CUBE_GRIP`=0.86에서 50 mm로 `env.py` 주석과 일치
- 동작 1 `classify_failures.py`가 큐브가 50 mm 올라야 "집음"으로 판정(환경은 30 mm) → 시드 97(49 mm 들림)이 성공인데 `picked=False`로 표시 → 환경 기준(`LIFT_REQUIRED`)을 사용
- 문서의 수치가 측정과 어긋남(열림 간격 "59 mm" → 실제 75~83 mm, "판 끝이 큐브 바닥보다 7 mm 아래" → 실제 12.5 mm) → 문서와 `mujoco_receiver.py` 주석을 측정값으로 정정

이전:

- 큐브가 시작 직후 튕겨 나감 → q=0에서 패드 간격이 큐브 폭과 같아 초기 접촉 → 손가락 qpos를 열림 자세로 시작
- 큐브를 10~19 mm 파고듦 → 손가락 링크가 0.01~0.07 kg라 기본 접촉이 물렁함 → 손가락/큐브 geom `solref="0.006 1"`
- 큐브가 한쪽으로 밀림 → 손가락 1·2(두 개) 대 손가락 3(한 개) 힘 불균형 → 손가락3 서보 kp 2배
- 판 끝 모서리로만 큐브를 집어 미끄러짐/튕김 → 넓은 받침대에 막혀 손가락이 못 내려감 → 큐브 폭 이하의 좁은 기둥으로 변경
- 이전 CAD 기반 기록(DAE→OBJ 변환기, 1.623156 rad 배치축, `MAX_CURL` 튜닝)은 최종 URDF 전환으로 폐기
