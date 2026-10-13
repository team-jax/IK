<!-- Parent: ../AGENTS.md -->
<!-- Generated: 2026-07-02 | Updated: 2026-10-13 -->

# motion2/ — 웹캠 손 추적 텔레오퍼레이션 (동작 2)

> 이 문서의 경로는 `motion2/` 기준이다. 아래 명령은 저장소 루트 기준(`venv/bin/python motion2/<script>`)이다. 스크립트가 `__file__` 기준 경로를 쓰므로 어느 위치에서 실행해도 되고, `motion2/`에서는 `../venv/bin/python <script>`로 쓴다.

## Purpose
웹캠 손목 추적 → UDP → MuJoCo 실시간 동기화 데모(7DOF 팔 + 3손가락 그리퍼, 손 모양으로
쥐기/펴기 제어) + 웹 대시보드 시각화를 포함한다. 로봇 모델은 최종 URDF(`../URDF_1006`)에서 생성한다.
Phase 2(7DOF 팔 IK + 그리퍼 kinematics + contact physics 파지: 받침대 위 큐브를 실제
접촉/마찰로 집기)까지 구현됨; Phase 3(실제 모터 연동)은 미착수.

## Key Files

| File | Description |
|------|-------------|
| `실행방법.txt` | 3터미널(수신기/센더/대시보드) 실행 순서, 스케일 조정법, IK 단독 테스트 방법 요약 |
| `vision_sender.py` | OpenCV + MediaPipe Tasks PoseLandmarker/HandLandmarker로 오른손 손목(landmark 16) X/Y/Z와 검지/중지/약지 각각의 굽힘(curl: 0=펴짐/1=완전히 굽힘)을 추출해 UDP 5005(mujoco_receiver)와 5006(web_dashboard) 양쪽으로 전송. visibility 필터(`MIN_VISIBILITY=0.5`) + EMA 스무딩(`EMA_ALPHA`/`GRIP_EMA_ALPHA=0.3`) 적용. 엄지/새끼는 의도적으로 무시(그리퍼가 3손가락) |
| `mujoco_receiver.py` | UDP 5005 논블로킹 수신 → palm-down 제약 IK(`solve_ik_palm_down`, 12필드 패킷이면 팔꿈치 스윙까지 함께 푸는 `solve_ik_arm`)를 풀어 `arm/arm.xml`의 7DOF 팔을 **position actuator ctrl 타깃**으로 구동(직접 qpos 쓰기 아님 — `mj_step` contact physics와 공존), 손가락별 curl 값으로 3손가락 그리퍼 개별 구동(검지→finger1, 중지→finger2, 약지→finger3). 물리 루프: 렌더 프레임당 `mj_step` 5회(500Hz 물리 / 100Hz 렌더·UDP). 첫 패킷 기준 좌우 자동 캘리브레이션('C' 키로 재캘리브레이션), 시작 시 카메라를 작업공간(받침대+큐브)으로 조준하고 'T' 키로 그리퍼 추적 카메라 토글 |
| `web_dashboard.py` | UDP 5006 수신 → stdlib만으로 손목 (x,y,z)와 손가락 3개 curl 값을 `http://127.0.0.1:8000`에 실시간 시각화 (Flask 등 의존성 없음) |
| `pose_landmarker_full.task` | MediaPipe Tasks PoseLandmarker 모델 파일 (`vision_sender.py`가 로드, 손목 위치용) |
| `hand_landmarker.task` | MediaPipe Tasks HandLandmarker 모델 파일 (`vision_sender.py`가 로드, 손 모양/grip용) |
| `run_sender.sh` | `vision_sender.py` 실행 스크립트 (venv 체크, 안내 메시지 포함) |
| `run_receiver.sh` | `mujoco_receiver.py` 실행 스크립트 (macOS→mjpython / Linux→python 자동 분기) |
| `run_web.sh` | `web_dashboard.py` 실행 스크립트 (venv 체크만, 추가 패키지 불필요) |
| `check_teleop.py` | 화면·웹캠 없이 UDP 파이프라인 전체 검사: 가짜 송신 스레드가 60 Hz로 패킷을 보내고(깨진 패킷·NaN 위치 포함) 수신기 `main()`이 그대로 돌며 큐브를 집어 85 mm 들어올린다. 시나리오 3개(`wrist`, `arm`, `swing-jump`), `teleop: 3/3 scenarios passed`가 기대값. 수신기의 IK·스윙·서보·접촉을 바꿨으면 반드시 실행 |
| `check_sender.py` | 웹캠 없이 송신기 검사: 굽힘 계산(펴짐 0, 90° 0.833, 접힘 1), 두 `.task` 모델이 빈 프레임에서 로드·실행됨, 가짜 카메라·랜드마커로 `main()`을 돌려 패킷이 두 포트에 같은 내용으로 12/6/12필드로 나가는지, 가시성 낮은 프레임을 버리는지, 손이 사라지면 굽힘을 유지하는지, 수신기 파서가 받아들이는지 확인 |
| `check_dashboard.py` | `web_dashboard.py`를 서브프로세스로 띄워 6/12/4필드 패킷 반영, 잘못된 패킷 무시, `/wrist.json`·페이지 응답 확인 |
| `check_grasp.py` | 화면 없는 파지 검증: 큐브 ±8 mm 어긋난 9곳에서 닫고 들어올리기, `lifted 9 /9`가 기대값 (손가락 기하·마찰·`FINGER_*`/`DISTAL_SCALE`을 바꿨으면 반드시 실행) |
| `test.py` | gymnasium `reacher.xml`을 로드해 MuJoCo passive viewer를 띄우는 환경 검증 스크립트 |

## Subdirectories

| Directory | Purpose |
|-----------|---------|
| `arm/` | 최종 URDF에서 생성한 7DOF 팔 URDF + MJCF 모델, 생성기, ikpy IK (see `arm/AGENTS.md`) |

## For AI Agents

### Working In This Directory
- Python은 반드시 `venv/bin/python` 사용 (전역 python 금지). macOS에서 MuJoCo viewer가 필요한 스크립트는 `venv/bin/mjpython` 사용.
- 새 패키지 설치 시 `venv/bin/python -m pip install <pkg>` (폴더를 옮기면 `venv/bin/pip`의 셔뱅이 깨지므로 `python -m pip`가 안전). `requirements.txt` 없음 — venv에 직접 설치.
- 코드 수정 시 루트 `CLAUDE.md`, `haness.md`의 규칙 준수 필수. `haness.md`는 AI가 지켜야 할 작업 프로토콜.
- `arm/arm.xml`, `arm/arm.urdf`, `arm/meshes/`는 생성 파일이다. 직접 고치지 말고 `arm/make_arm_urdf.py` 상수를 바꿔 다시 생성한다.
- 파라미터(UDP 포트, 좌표 스케일 등) 변경 시 `vision_sender.py`, `mujoco_receiver.py`, `web_dashboard.py` 모두 확인 — 포트 번호가 세 파일에 하드코딩되어 있어 한 곳만 바꾸면 불일치 발생.
- `vision_sender.py`는 mediapipe의 레거시 `mp.solutions` API가 아닌 Tasks API(`PoseLandmarker`)를 사용 — mediapipe>=0.10.30에서 레거시 API가 제거됨.

### Testing Requirements
- `vision_sender.py`: `venv/bin/python motion2/vision_sender.py` — 웹캠 + `pose_landmarker_full.task` 필요, 'q' 키로 종료.
- `mujoco_receiver.py`: `venv/bin/mjpython motion2/mujoco_receiver.py` (macOS) — 로컬 디스플레이 필요, `arm/arm.xml`의 7DOF 팔을 실시간 IK로 구동.
- `web_dashboard.py`: `venv/bin/python motion2/web_dashboard.py` — 브라우저에서 `http://127.0.0.1:8000` 접속.
- 화면·웹캠 없이: 루트의 `./run_tests.sh`가 아래를 모두 돌린다. 개별로는 `check_grasp.py`(파지 9/9), `check_teleop.py`(UDP 전체 경로 3/3), `check_sender.py`, `check_dashboard.py`, `arm/ik.py`(IK 오차 0.00 mm). 웹캠은 켜지 않는다(실제 카메라 시험은 2026-10-13 `STATUS.md` 참고).
- 전체 파이프라인 확인: `run_receiver.sh` → `run_sender.sh`(+ 선택적으로 `run_web.sh`) 순서로 각각 별도 터미널에서 실행 (자세한 순서는 `실행방법.txt` 참고).
- `test.py`: `venv/bin/mjpython motion2/test.py` — gymnasium 설치 확인용.

### Common Patterns
- UDP 통신: `socket.SOCK_DGRAM`, 패킷 포맷 `"x y z curl_index curl_middle curl_ring"` (공백 구분 float 6개 문자열). 팔꿈치·어깨가 둘 다 보이면 `ex ey ez sx sy sz`를 덧붙여 12필드가 되고, 수신기는 이때 팔꿈치 스윙각까지 푼다(`solve_ik_arm`). 수신 측(`mujoco_receiver.parse_packet`, `web_dashboard.udp_listener`)은 구형 4필드 `"x y z grip"` 패킷도 받아 grip을 3손가락에 동일 적용(하위 호환). `vision_sender.py`는 동일 패킷을 5005(mujoco_receiver, 논블로킹 수신)와 5006(web_dashboard) 양쪽에 fan-out. **패킷 필드 개수를 바꾸면 세 파일 모두 파싱 로직을 같이 맞춰야 함** — 안 그러면 `ValueError`가 조용히 삼켜져(catch 후 continue) 수신 측이 멈춘 것처럼 보임.
- 손목 좌표 안정화: visibility 필터(`MIN_VISIBILITY`) → 통과 프레임만 EMA 스무딩(`EMA_ALPHA`) → 전송. 필터 없이 저신뢰 프레임을 그대로 보내면 팔이 "flapping"함.
- 손가락 curl 추출: HandLandmarker에서 손가락별 PIP 관절의 3D 굽힘각(MCP–PIP–TIP 사이 각, 편 손 ~180° / 주먹 ~60–90°)을 `CURL_EXTENDED_DEG`(165°)→0 / `CURL_CLOSED_DEG`(75°)→1로 선형 매핑 + clip — 검지(5,6,8)/중지(9,10,12)/약지(13,14,16). 관절각 기반이라 손 방향·카메라 거리와 무관하고 손가락별 독립. **이전의 2D tip-to-wrist 거리 비율 방식은 손바닥이 카메라를 정면으로 안 볼 때 값이 0.4~0.65 중간 대역에 압축되어 "손가락이 안 움직이는" 증상을 냈으므로(실측) 되돌리지 말 것.** 손이 감지 안 되면 마지막 curl 값들을 유지(hold), 0으로 스냅하지 않음. 웹캠 창에 추적 중인 3손가락 스켈레톤 + "HAND: OK / NOT DETECTED" 상태 표시(`draw_hand`), 손 감지 신뢰도 문턱은 0.4(상체 전체가 프레임에 들어와 손이 작게 잡히므로 0.5는 감지 드랍이 잦았음).
- 좌우/깊이 캘리브레이션: `mujoco_receiver.py`는 첫 UDP 패킷의 x좌표(좌우 중심)와 z값(중립 깊이)을 자동 캘리브레이션, 뷰어 창에서 'C' 키로 언제든 재캘리브레이션 가능. **깊이 캘리브레이션은 필수** — MediaPipe pose z는 hip 기준 상대값이라 사람/자세마다 baseline이 -0.8~-1.9까지 달라(실측), 빼지 않고 `DEPTH_SCALE`로 매핑하면 목표점이 팔 베이스 뒤쪽(x≈-0.1~-0.46)으로 가서 팔이 받침대 반대편으로 뻗음("안 움직이는" 증상). 계산된 목표점은 추가로 `WORKSPACE_X/Y/Z` 박스로 클램프됨.
- 팔꿈치 스윙(12필드 패킷): 사람의 스윙 변화를 로봇에 그대로 주지 않는다. ① `SWIVEL_LIMIT`=±40° (이 밖에서는 큐브 위치에서 들어올릴 때 IK 해가 한 번에 0.2~0.5 rad 점프), ② `SWIVEL_MAX_RATE`=90°/s로 램프 (`swivel_angle()`이 팔이 수직에 가까우면 기준을 바꿔 측정값이 한 프레임에 최대 90° 뜀), ③ 손가락을 쥐는 동안(`SWIVEL_HOLD_CURL`=0.3) 고정 (스윙이 움직이면 파지 안에서 큐브가 미끄러지거나 튕김). ①이나 ③을 빼면 `swing-jump`가 실패하고, ②를 빼면 통과하지만 스윙이 급변하는 구간에서 손 이탈(7.9→2.5 mm)·서보 지연(0.45→0.10 rad)이 3~4배 커진다 (수치는 `mujoco_receiver.py` 주석). 작업 영역 가장자리(x≥0.19, |y|≥0.15)에서는 스윙과 무관하게 IK 해가 점프하는 구간이 있다(STATUS.md).
- IK 실패 시 폴백: `solve_ik` 결과가 NaN이거나 예외 발생 시 마지막으로 성공한 `last_good_qpos`를 그대로 사용해 팔이 튀지 않도록 함.
- 그리퍼 구동: `mujoco_receiver.py`의 `apply_grip()`이 curl 3개를 손가락 6개 position actuator에 쓴다. curl 0→1이 근위 관절을 `FINGER_OPEN=-0.25`(큐브 높이에서 패드 간격 약 75~83 mm) → `FINGER_CLOSED=0.15` rad로 보간하고, 먼쪽 관절은 `(근위-FINGER_OPEN)*DISTAL_SCALE(0.1)`. DISTAL_SCALE이 크면 접촉 시 판이 기울어 큐브 모서리만 물고 들어올릴 때 미끄러진다. `check_grasp.py` 결과(2026-10-13, `FINGER_CLOSED=0.15`): DISTAL_SCALE 0~0.10은 9/9, 0.15는 7/9, 0.20은 0/9, 0.30은 2/9이고, `DISTAL_SCALE=0.1`에서 FINGER_CLOSED 0.10·0.15·0.20은 모두 9/9다. 손가락 기하·마찰을 바꾸면 `check_grasp.py`로 다시 확인할 것. 손가락3의 배치축(`ROTATION_BASE_ANGLE=-π`)은 `make_arm_urdf.py`에서 고정. 시작할 때 손가락 qpos도 열림 자세로 맞춘다(q=0이면 패드 간격이 큐브 폭과 같아 초기 접촉이 생김). qpos 직접 쓰기가 아니므로 contact 중 서보의 잔여 힘이 파지력이 된다.
- 뷰어 카메라: `mujoco_receiver.py`가 시작 시 free camera를 `CAM_LOOKAT`(팔 작업공간 중심)으로 조준 — MuJoCo 기본 프레이밍은 바닥 평면까지 포함해 팔이 화면 위쪽에 치우침. 'T' 키로 `mjCAMERA_TRACKING`(trackbody=`MG996R`, 손) 토글. key callback은 뷰어 스레드에서 도는 만큼 플래그만 바꾸고, 실제 `viewer.cam` 변경은 메인 루프에서 `viewer.lock()` 안에서 적용.
- IK orientation 제약: `mujoco_receiver.solve_ik_palm_down()`이 z≤`FULL_ORIENT_MAX_Z`(0.34)에서 ikpy `orientation_mode="all"`로 end frame을 월드 축에 정렬(손가락 아래, 손가락 간격 방향이 월드 x)해 먼저 푼다. 요(yaw)까지 고정하는 이유: 요가 임의면 손가락 판이 큐브 모서리를 쳐서 큐브가 튕겨 나간다. ikpy "all"은 위치+방향 합산 최소화라 위치만 맞고 요가 틀어질 수 있어 `_orient_err`(`ORIENT_ERR_TOL` 0.1rad)로 방향도 검사하고, 실패하면 `SEED_POSE`에서 재시도 → 그래도 안 되거나 z가 `FULL_ORIENT_MAX_Z`보다 높으면 position-only 폴백. `orientation_mode="Z"`는 느리고 수렴도 나빠 제거. `arm/ik.py`의 `solve_ik`는 건드리지 않고 receiver 쪽 래퍼로 구현.
- MuJoCo 상태 변경(qpos 쓰기, `mj_step`)은 반드시 `viewer.lock()` 컨텍스트 내에서 수행.
- passive viewer 두 가지 용법: `mujoco_receiver.py`는 매 프레임 `mj_step`으로 contact physics를 적분(servo가 팔을 끌고 감 — 약간의 서보 지연은 정상). 반면 `arm/ik.py`의 `--view`/`--interactive`는 `mj_step` 호출 없이 `viewer.sync()`만 돌림(actuator 타깃 없이 스텝하면 중력에 팔이 처지므로 정적 IK 확인 용도).
- 좌표 매핑 지오메트리 예산: 명령점은 `end_effector`(손가락 사이 파지 중심)이고 높이 범위는 `[BASE_Z, BASE_Z+HEIGHT_SCALE]`(0.255~0.34). 큐브(30 mm)는 윗면 z=0.25인 좁은 기둥 위(중심 z=0.265)에 있고, 손가락 끝이 파지 중심보다 약 17.5 mm 아래라 `BASE_Z=0.255`(큐브 바닥보다 5 mm 위)에서 판 끝이 큐브 바닥보다 약 12.5 mm 아래(기둥 옆)로 내려가 평평한 패드면이 큐브 옆면에 닿는다. 0.34 위에서는 손가락 아래 자세를 유지하지 못해(IK 스윕) 상한을 둔다.

## Dependencies

### Internal
- `../URDF_1006/` — `arm/make_arm_urdf.py`가 읽는 최종 URDF 원본 (URDF 관절 → 프로젝트 관절 이름 대응표는 `../URDF_1006/JAXV2_Assembly_description/AGENTS.md`)
- `../motion1/` — `arm/make_arm_urdf.py`의 `robot_parts()`를 불러다 쓴다(동작 1이 이 폴더에 의존, 반대는 없음). 생성기 인터페이스를 바꾸면 `motion1/src/humanoid_grasp_rl/assets.py`도 확인
- `arm/` — `mujoco_receiver.py`와 `ik.py` 모두 `arm.urdf`, `arm.xml` 참조 (see `arm/AGENTS.md`). `arm.xml`의 그리퍼·받침대·큐브·actuator는 `arm.urdf`/ikpy 체인에는 없음 — IK와 무관한 순수 MuJoCo 전용 구조.
- `pose_landmarker_full.task` — `vision_sender.py`가 로드하는 모델 파일(손목 위치), 레포에 커밋됨
- `hand_landmarker.task` — `vision_sender.py`가 로드하는 모델 파일(손가락 curl), 레포에 커밋됨(다운로드 URL은 루트 `README.md`)

### External
| Package | Version | 용도 |
|---------|---------|------|
| `mujoco` | 3.10.0 | 물리 시뮬레이션 및 passive viewer |
| `mediapipe` | 0.10.35 | Pose landmark 추출 (Tasks API) |
| `opencv-python` | 4.13.0 | 웹캠 프레임 캡처 및 화면 표시 |
| `ikpy` | 3.4.2 | URDF 체인 로드 및 IK 풀이 |
| `gymnasium` | 1.3.0 | reacher.xml 에셋 경로 (test.py) |
| `numpy`, `scipy` | — / 1.18.0 | 좌표 변환 및 오차 계산, 수신기의 팔 전체 IK(`least_squares`)와 생성기의 쿼터니언 변환 |
| (stdlib only) | — | `web_dashboard.py`는 외부 패키지 없이 `http.server` + `socket`만 사용 |

<!-- MANUAL: -->
