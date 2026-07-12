<!-- Generated: 2026-07-02 | Updated: 2026-07-09 -->

# 역운 — 휴머노이드 상체 로봇 프로젝트

## Purpose
양팔 휴머노이드 상체 로봇의 소프트웨어 레포지토리. MuJoCo 시뮬레이션 기반 IK 검증,
웹캠 손목 추적 → UDP → MuJoCo 실시간 동기화 데모(7DOF 팔 + 3손가락 그리퍼, 손 모양으로
쥐기/펴기 제어) + 웹 대시보드 시각화, STM32/Jetson 통합 설계 문서를 포함한다.
Phase 2(7DOF 팔 IK + 그리퍼 kinematics + contact physics 파지: 받침대 위 큐브를 실제
접촉/마찰로 집기)까지 구현됨; Phase 3(실제 모터 연동)은 미착수.

## Key Files

| File | Description |
|------|-------------|
| `CLAUDE.md` | 프로젝트 전체 설명, 하드웨어 스펙, 소프트웨어 아키텍처, 개발 로드맵, 통신 프로토콜 |
| `haness.md` | AI 작업 가이드라인 — 해야 할 것/하지 말아야 할 것, 진행 플로우, 코드 작성 규칙 |
| `실행방법.txt` | 3터미널(수신기/센더/대시보드) 실행 순서, 스케일 조정법, IK 단독 테스트 방법 요약 |
| `vision_sender.py` | OpenCV + MediaPipe Tasks PoseLandmarker/HandLandmarker로 오른손 손목(landmark 16) X/Y/Z와 검지/중지/약지 각각의 굽힘(curl: 0=펴짐/1=완전히 굽힘)을 추출해 UDP 5005(mujoco_receiver)와 5006(web_dashboard) 양쪽으로 전송. visibility 필터(`MIN_VISIBILITY=0.5`) + EMA 스무딩(`EMA_ALPHA`/`GRIP_EMA_ALPHA=0.3`) 적용. 엄지/새끼는 의도적으로 무시(그리퍼가 3손가락) |
| `mujoco_receiver.py` | UDP 5005 논블로킹 수신 → palm-down 제약 IK(`solve_ik_palm_down`)를 풀어 `arm/arm.xml`의 7DOF 팔을 **position actuator ctrl 타깃**으로 구동(직접 qpos 쓰기 아님 — `mj_step` contact physics와 공존), 손가락별 curl 값으로 3손가락 그리퍼 개별 구동(검지→finger1, 중지→finger2, 약지→finger3). 물리 루프: 렌더 프레임당 `mj_step` 5회(500Hz 물리 / 100Hz 렌더·UDP). 첫 패킷 기준 좌우 자동 캘리브레이션('C' 키로 재캘리브레이션), 시작 시 카메라를 작업공간(받침대+큐브)으로 조준하고 'T' 키로 그리퍼 추적 카메라 토글 |
| `web_dashboard.py` | UDP 5006 수신 → stdlib만으로 손목 (x,y,z)와 손가락 3개 curl 값을 `http://127.0.0.1:8000`에 실시간 시각화 (Flask 등 의존성 없음) |
| `pose_landmarker_full.task` | MediaPipe Tasks PoseLandmarker 모델 파일 (`vision_sender.py`가 로드, 손목 위치용) |
| `hand_landmarker.task` | MediaPipe Tasks HandLandmarker 모델 파일 (`vision_sender.py`가 로드, 손 모양/grip용) |
| `run_sender.sh` | `vision_sender.py` 실행 스크립트 (venv 체크, 안내 메시지 포함) |
| `run_receiver.sh` | `mujoco_receiver.py` 실행 스크립트 (macOS→mjpython / Linux→python 자동 분기) |
| `run_web.sh` | `web_dashboard.py` 실행 스크립트 (venv 체크만, 추가 패키지 불필요) |
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
- 파라미터(UDP 포트, 좌표 스케일 등) 변경 시 `vision_sender.py`, `mujoco_receiver.py`, `web_dashboard.py` 모두 확인 — 포트 번호가 세 파일에 하드코딩되어 있어 한 곳만 바꾸면 불일치 발생.
- `vision_sender.py`는 mediapipe의 레거시 `mp.solutions` API가 아닌 Tasks API(`PoseLandmarker`)를 사용 — mediapipe>=0.10.30에서 레거시 API가 제거됨.

### Testing Requirements
- `vision_sender.py`: `venv/bin/python vision_sender.py` — 웹캠 + `pose_landmarker_full.task` 필요, 'q' 키로 종료.
- `mujoco_receiver.py`: `venv/bin/mjpython mujoco_receiver.py` (macOS) — 로컬 디스플레이 필요, `arm/arm.xml`의 7DOF 팔을 실시간 IK로 구동.
- `web_dashboard.py`: `venv/bin/python web_dashboard.py` — 브라우저에서 `http://127.0.0.1:8000` 접속.
- 전체 파이프라인 확인: `run_receiver.sh` → `run_sender.sh`(+ 선택적으로 `run_web.sh`) 순서로 각각 별도 터미널에서 실행 (자세한 순서는 `실행방법.txt` 참고).
- `test.py`: `venv/bin/mjpython test.py` — gymnasium 설치 확인용.

### Common Patterns
- UDP 통신: `socket.SOCK_DGRAM`, 패킷 포맷 `"x y z curl_index curl_middle curl_ring"` (공백 구분 float 6개 문자열). 수신 측(`mujoco_receiver.parse_packet`, `web_dashboard.udp_listener`)은 구형 4필드 `"x y z grip"` 패킷도 받아 grip을 3손가락에 동일 적용(하위 호환). `vision_sender.py`는 동일 패킷을 5005(mujoco_receiver, 논블로킹 수신)와 5006(web_dashboard) 양쪽에 fan-out. **패킷 필드 개수를 바꾸면 세 파일 모두 파싱 로직을 같이 맞춰야 함** — 안 그러면 `ValueError`가 조용히 삼켜져(catch 후 continue) 수신 측이 멈춘 것처럼 보임.
- 손목 좌표 안정화: visibility 필터(`MIN_VISIBILITY`) → 통과 프레임만 EMA 스무딩(`EMA_ALPHA`) → 전송. 필터 없이 저신뢰 프레임을 그대로 보내면 팔이 "flapping"함.
- 손가락 curl 추출: HandLandmarker에서 손가락별 PIP 관절의 3D 굽힘각(MCP–PIP–TIP 사이 각, 편 손 ~180° / 주먹 ~60–90°)을 `CURL_EXTENDED_DEG`(165°)→0 / `CURL_CLOSED_DEG`(75°)→1로 선형 매핑 + clip — 검지(5,6,8)/중지(9,10,12)/약지(13,14,16). 관절각 기반이라 손 방향·카메라 거리와 무관하고 손가락별 독립. **이전의 2D tip-to-wrist 거리 비율 방식은 손바닥이 카메라를 정면으로 안 볼 때 값이 0.4~0.65 중간 대역에 압축되어 "손가락이 안 움직이는" 증상을 냈으므로(실측) 되돌리지 말 것.** 손이 감지 안 되면 마지막 curl 값들을 유지(hold), 0으로 스냅하지 않음. 웹캠 창에 추적 중인 3손가락 스켈레톤 + "HAND: OK / NOT DETECTED" 상태 표시(`draw_hand`), 손 감지 신뢰도 문턱은 0.4(상체 전체가 프레임에 들어와 손이 작게 잡히므로 0.5는 감지 드랍이 잦았음).
- 좌우/깊이 캘리브레이션: `mujoco_receiver.py`는 첫 UDP 패킷의 x좌표(좌우 중심)와 z값(중립 깊이)을 자동 캘리브레이션, 뷰어 창에서 'C' 키로 언제든 재캘리브레이션 가능. **깊이 캘리브레이션은 필수** — MediaPipe pose z는 hip 기준 상대값이라 사람/자세마다 baseline이 -0.8~-1.9까지 달라(실측), 빼지 않고 `DEPTH_SCALE`로 매핑하면 목표점이 팔 베이스 뒤쪽(x≈-0.1~-0.46)으로 가서 팔이 받침대 반대편으로 뻗음("안 움직이는" 증상). 계산된 목표점은 추가로 `WORKSPACE_X/Y/Z` 박스로 클램프됨.
- IK 실패 시 폴백: `solve_ik` 결과가 NaN이거나 예외 발생 시 마지막으로 성공한 `last_good_qpos`를 그대로 사용해 팔이 튀지 않도록 함.
- 그리퍼 구동: `mujoco_receiver.py`의 `apply_grip()`이 curl 3개(검지/중지/약지 순, `FINGER_ORDER` = finger1/2/3)를 받아 `arm/arm.xml`의 6개 finger position actuator(`fingerN_joint{1,2}_act`)의 ctrl에 씀 — qpos 직접 쓰기가 아니므로 contact 중인 손가락은 타깃에 못 미치고 멈추며, 서보의 잔여 힘이 큐브를 쥐는 힘이 됨. distal 관절은 proximal의 `DISTAL_SCALE`(0.6)배만 굽혀 자연스러운 호를 그림. `MAX_CURL`(0.78 rad)은 실측(그립 0→1 스윕하며 fingertip이 중심축으로 단조 수렴하는지 확인)으로 튜닝된 값이라, 그리퍼 지오메트리(finger 세그먼트 길이/반경)를 바꾸면 재튜닝 필요 — 너무 크면 손가락이 중심축을 넘어가 다시 벌어짐.
- 뷰어 카메라: `mujoco_receiver.py`가 시작 시 free camera를 `CAM_LOOKAT`(팔 작업공간 중심)으로 조준 — MuJoCo 기본 프레이밍은 바닥 평면까지 포함해 팔이 화면 위쪽에 치우침. 'T' 키로 `mjCAMERA_TRACKING`(trackbody=`gripper_base`) 토글. key callback은 뷰어 스레드에서 도는 만큼 플래그만 바꾸고, 실제 `viewer.cam` 변경은 메인 루프에서 `viewer.lock()` 안에서 적용.
- IK orientation 제약 (하이브리드): position-only IK는 손목 방향이 임의로 남아 파지 자세에서 손바닥이 옆을 향했음 — `mujoco_receiver.solve_ik_palm_down()`이 ikpy `orientation_mode="Z"`로 end link의 local z축을 world +z에 고정해 손가락이 아래를 향하게 함. 단 **palm-down은 작업공간 전체에서 만족 불가** — y≈0 근거리/높은 목표는 손을 수직으로 유지하려면 팔꿈치가 관절 한계(2.4435 rad)를 넘어야 해서 위치 오차 54~153mm가 남음(`max_iter` 증가로 안 고쳐짐 — 수렴 문제가 아니라 관절 한계 벽, 실측). 그래서 palm-down을 먼저 풀고 위치 오차가 `PALM_DOWN_ERR_TOL`(20mm) 초과면 position-only로 재풀이해 더 나은 쪽 사용: 파지 구역(큐브 주변 낮은 목표)에서는 palm-down 유지, 그 외에는 방향을 양보하고 위치 추종. `arm/ik.py`의 `solve_ik`는 건드리지 않고 receiver 쪽 래퍼로 구현.
- MuJoCo 상태 변경(qpos 쓰기, `mj_step`)은 반드시 `viewer.lock()` 컨텍스트 내에서 수행.
- passive viewer 두 가지 용법: `mujoco_receiver.py`는 매 프레임 `mj_step`으로 contact physics를 적분(servo가 팔을 끌고 감 — 약간의 서보 지연은 정상). 반면 `arm/ik.py`의 `--view`/`--interactive`는 `mj_step` 호출 없이 `viewer.sync()`만 돌림(actuator 타깃 없이 스텝하면 중력에 팔이 처지므로 정적 IK 확인 용도).
- 좌표 매핑 지오메트리 예산: 그리퍼 높이 명령 범위는 `[BASE_Z, BASE_Z+HEIGHT_SCALE]`(0.40~0.90), 받침대 윗면 z=0.45, 큐브 중심 z=0.465 — `BASE_Z`를 0.465 위로 올리면 그리퍼가 큐브를 감쌀 만큼 못 내려가므로 상수 조정 시 이 관계 유지.

## Dependencies

### Internal
- `arm/` — `mujoco_receiver.py`와 `ik.py` 모두 `arm.urdf`, `arm.xml` 참조 (see `arm/AGENTS.md`). `arm.xml`의 그리퍼·받침대·큐브·actuator는 `arm.urdf`/ikpy 체인에는 없음 — IK와 무관한 순수 MuJoCo 전용 구조.
- `pose_landmarker_full.task` — `vision_sender.py`가 로드하는 모델 파일(손목 위치), 레포에 커밋됨
- `hand_landmarker.task` — `vision_sender.py`가 로드하는 모델 파일(손가락 curl), 로컬에만 존재(아직 미커밋 — 커밋하거나 README에 다운로드 URL 안내 필요)

### External
| Package | Version | 용도 |
|---------|---------|------|
| `mujoco` | 3.10.0 | 물리 시뮬레이션 및 passive viewer |
| `mediapipe` | 0.10.35 | Pose landmark 추출 (Tasks API) |
| `opencv-python` | 4.13.0 | 웹캠 프레임 캡처 및 화면 표시 |
| `ikpy` | — | URDF 체인 로드 및 IK 풀이 |
| `gymnasium` | — | reacher.xml 에셋 경로 (test.py) |
| `numpy` | — | 좌표 변환 및 오차 계산 |
| (stdlib only) | — | `web_dashboard.py`는 외부 패키지 없이 `http.server` + `socket`만 사용 |

<!-- MANUAL: -->
