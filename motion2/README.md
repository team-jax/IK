# Hand Teleop (동작 2)

웹캠으로 사람의 오른손을 추적해, MuJoCo 속 팀의 최종 URDF(`../URDF_1006/JAXV2_Assembly_description`)의 7자유도 팔 + 3손가락 손을 실시간으로 따라 움직이게 하고, 받침 기둥 위의 30 mm 큐브를 집어 올린다. 손목 위치는 IK로 팔 관절각이 되고, 검지·중지·약지의 굽힘은 손가락 3개를 하나씩 움직인다. 접촉 물리가 켜져 있어 큐브를 실제 마찰로 집는다.

```
웹캠 → MediaPipe (손목 위치 + 손가락 굽힘) → UDP → IK → MuJoCo 시뮬레이션
                                          └→ 웹 대시보드 (브라우저 시각화)
```

로봇 모델(`arm/`)은 `../URDF_1006`에서 `arm/make_arm_urdf.py`가 생성한다. 동작 1(`../motion1/`, 별도 저장소로 team-jax/IK에는 없음)이 같은 생성기와 모델을 쓰므로 `arm/` 위치는 바꾸지 않는다. 동작 2는 동작 1 없이 이 저장소만으로 돌아간다.

## 현재 상태 (2026-10-13)

| 항목 | 결과 | 확인 |
| --- | --- | --- |
| 모델 생성 | 최종 URDF → `arm.urdf`, `arm.xml`, `meshes/`. 생성기 출력이 저장소 파일과 같다 | `./run_tests.sh` |
| IK | ikpy FK ↔ MuJoCo `end_effector` site 일치 1e-9, 목표 도달 오차 0.00 mm | `arm/ik.py` |
| 파지 | 큐브가 ±8 mm 어긋난 9곳에서 닫고 손을 84 mm 올려 들어올리기 9/9 | `scripts/check_grasp.py` |
| UDP 파이프라인 전체 | 가짜 송신 → 수신기 → IK → 서보 → 접촉 → 손을 85 mm 올려 큐브를 들어올림(큐브 중심 z 0.265 → 0.347~0.349). 시나리오 3개(6필드, 12필드, 팔꿈치 스윙 급점프) 3/3, 깨진 패킷·NaN이 섞여도 견딤 | `scripts/check_teleop.py` |
| 송신기 | 굽힘 계산, 두 모델 로딩, 패킷 형식·가시성 필터·스무딩 | `scripts/check_sender.py` |
| 대시보드 | 6/12/4필드 패킷 반영, 잘못된 패킷 무시, HTTP 응답 | `scripts/check_dashboard.py` |
| 실제 웹캠 | 열림(1920×1080, 30 fps). 송신기가 14초 동안 오류 없이 돌고 초당 27.8프레임을 처리(읽기 11 ms + 포즈 13 ms + 손 11 ms). **화면에 사람이 없어 추적과 패킷은 못 봄** | 수동 |

### 한계 (측정으로 확인한 것)

- **사람을 실제로 추적한 육안 확인은 아직 없다.** 카메라 앞에 선 채 `scripts/run_receiver.sh`와 `scripts/run_sender.sh`를 같이 돌려 팔 움직임과 파지를 눈으로 봐야 한다.
- **작업 영역 가장자리(x≥0.19 또는 |y|≥0.15)에서 손을 수직으로 움직이면 IK 해가 점프한다.** 팔 전체 솔버가 최대 0.26 rad, 손목만 쓰는 palm-down 솔버가 최대 1.8 rad(스윙이 -85° 흘러내림)다. 큐브 위치 (0.12, 0)에서는 0.01 rad 이하라 데모에는 문제없지만, 가장자리에서 큐브를 집어 올리면 튕길 수 있다. **원인은 팔이 못 닿는 것이 아니라 풀이다.** 작업 영역 상자 안 75개 점(x 5 × y 5 × z 3) 전부에서 손가락이 아래를 향한 자세가 존재하는데(시작점 16개로 풀면 위치 2 cm·기울기 6° 이내로 모두 성공), 이전 자세에서 출발하는 한 번짜리 풀이(ikpy SLSQP)는 다른 관절 가지로 가서 해가 뛰거나 손이 기운다(z=0.34, x≥0.12, y≥+0.1을 훑을 때 기울기 최대 114° 관측). 충돌은 보지 않은 운동학 기준이고 2026-10-13에 측정했다.
- **높은 곳에서는 손가락을 아래로 유지하지 못한다.** 몸통에 가까운 앞쪽(x≥0.17, y≈0)부터 닿지 않는다. 시작점 16개 중 최선의 위치 오차는 (0.17, 0)의 z=0.37에서 34 mm, 큐브 위치 (0.12, 0)의 z=0.37에서 15 mm, z=0.40에서 41 mm다. 그래서 `WORKSPACE_Z=(0.255, 0.34)`로 막아 뒀다.
- **팔꿈치 스윙은 일부만 따른다.** ±40°(`SWIVEL_LIMIT`), 초당 90°(`SWIVEL_MAX_RATE`), 손가락을 쥐는 동안(평균 curl > `SWIVEL_HOLD_CURL`=0.3) 고정. 이유는 아래 "알아낸 것".

## 설치 및 확인

저장소 루트의 `venv/`를 동작 1·2가 함께 쓴다. 아래 명령은 모두 저장소 루트 `IK/` 기준이고, 스크립트가 `__file__` 기준으로 경로를 찾으므로 어느 폴더에서 실행해도 된다.

```bash
# 가상환경 (처음 한 번)
python3 -m venv venv
venv/bin/python -m pip install mujoco gymnasium numpy scipy glfw ikpy mediapipe opencv-python imageio imageio-ffmpeg pillow

# 화면·웹캠 없이 동작 확인
venv/bin/python motion2/scripts/check_grasp.py     # lifted 9 /9
```

MediaPipe 모델 파일 2개(`models/`)는 저장소에 들어 있다. 없을 때만 내려받는다:

```bash
curl -o motion2/models/pose_landmarker_full.task https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_full/float16/latest/pose_landmarker_full.task
curl -o motion2/models/hand_landmarker.task https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task
```

필요한 것: macOS 또는 Linux, 로컬 디스플레이(MuJoCo 뷰어가 창을 띄운다), 웹캠.

## 실행 (터미널 2~3개)

| 터미널 | 명령 | 역할 |
| --- | --- | --- |
| 1 | `motion2/scripts/run_receiver.sh` | MuJoCo 시뮬레이션 (팔 + 그리퍼 + 큐브) |
| 2 | `motion2/scripts/run_sender.sh` | 웹캠 손 추적 → UDP 전송 |
| 3 (선택) | `motion2/scripts/run_web.sh` | 웹 대시보드 → http://127.0.0.1:8000 |

터미널 1을 먼저 실행하고, 2·3은 순서 무관하다. 5005·5006은 UDP 소켓이라 브라우저로 열 수 없고, 브라우저는 8000번 포트로 접속한다.

종료: 2번은 웹캠 창에서 `q`, 1번은 MuJoCo 뷰어 창 닫기, 3번은 `Ctrl+C`.

## 조작법

- **팔** — 오른손 손목을 카메라 앞에서 움직이면 팔이 따라온다. 팔꿈치와 어깨도 보이면(12필드 패킷) 팔꿈치를 옆으로 벌리거나 모을 때 로봇 팔꿈치도 같이 벌어진다(7자유도 중 남는 1자유도). 시작할 때와 재캘리브레이션 때의 팔꿈치 자세가 기준(0°)이니 편한 자세에서 시작한다. 어깨·팔꿈치가 안 보이면 손목만 따라간다.
- **손가락** — 검지/중지/약지를 각각 굽히면 그리퍼 손가락 1/2/3번이 따로 움직인다(그리퍼가 3손가락이라 엄지·새끼는 무시). 손이 감지되지 않으면 마지막 굽힘을 유지한다.
- **집기** — 좁은 받침 기둥 위 빨간 큐브 양옆으로 손을 내리고 손가락을 오므리면 접촉·마찰로 집힌다. 큐브를 쥐고 있는 동안은 팔꿈치가 고정되고, 손을 펴면 다시 따라온다.
- 뷰어 키: **T** = 그리퍼 추적 카메라 켜기/끄기, **C** = 좌우 중심·깊이·팔꿈치 기준각 재캘리브레이션. 첫 패킷이 오면 자동으로 한 번 캘리브레이션한다.

## 구조

| 경로 | 역할 |
| --- | --- |
| `src/vision_sender.py` | MediaPipe Tasks(Pose + Hand)로 오른손 손목 (x, y, z)와 검지·중지·약지 curl(0=펴짐~1=굽힘), 보이면 팔꿈치·어깨까지 추출해 UDP 5005(수신기)와 5006(대시보드)로 전송 |
| `src/mujoco_receiver.py` | UDP 수신 → 좌표를 팔 작업 영역으로 매핑 → IK(손목 위치 + 팔꿈치 스윙) → position actuator로 팔·그리퍼 구동, `mj_step`으로 접촉 물리 적분 |
| `src/web_dashboard.py` | UDP 5006 값을 브라우저에 실시간 표시 (stdlib만 사용) |
| `scripts/run_receiver.sh`, `run_sender.sh`, `run_web.sh` | 위 세 프로그램의 실행 래퍼 (venv 확인, macOS는 `mjpython` 선택) |
| `scripts/check_*.py` | 화면·웹캠 없는 검사 4개와 뷰어 환경 확인(`check_viewer.py`). `_paths.py`가 `src/`를 경로에 넣는다 |
| `models/` | MediaPipe 모델 `pose_landmarker_full.task`, `hand_landmarker.task` |
| `arm/make_arm_urdf.py` | 최종 URDF에서 `arm.urdf`, `arm.xml`, `meshes/`를 만드는 생성기. 파지 중심·손가락 배치·기둥/큐브 크기·서보 게인·접촉 파라미터는 이 파일 상단 상수로만 바꾼다 |
| `arm/arm.urdf`, `arm/arm.xml`, `arm/meshes/` | 생성 파일(커밋됨). ikpy용 7관절 체인, MuJoCo 모델(팔 7관절 + 3손가락 손 + 기둥·큐브 + actuator 13개), 시각 OBJ·충돌 STL |
| `arm/ik.py` | IK 단독 풀이·검증 (`--target x y z`, `--view`, `--interactive`) |

모델은 `nq=20`(팔 7 + 손가락 6 + 큐브 7), `nu=13`이다. 큐브는 30 mm, 윗면 z=0.25인 좁은 기둥 위에 있다. 자세한 구현 패턴은 `src/AGENTS.md`, 모델은 `arm/AGENTS.md`.

### UDP 패킷

```
"x y z curl_index curl_middle curl_ring"                    (6필드: 손목만)
"x y z curl_index curl_middle curl_ring ex ey ez sx sy sz"   (12필드: 팔꿈치·어깨가 둘 다 보일 때만 추가)
```

`x, y`는 프레임 정규화 좌표(0~1), `z`는 MediaPipe 깊이 추정치다. 옛 4필드 `"x y z grip"`도 수신 측이 받아 grip을 세 손가락에 똑같이 적용한다. 필드 수를 바꾸면 `src/` 세 파일의 파싱을 모두 맞춰야 한다.

## 튜닝

팔 움직임 범위가 이상하면 `src/mujoco_receiver.py` 상단의 숫자를 바꾸고 다시 실행한다.

| 상수 | 값 | 뜻 |
| --- | --- | --- |
| `LATERAL_SCALE` | 0.6 | 좌우 범위 |
| `HEIGHT_SCALE` | 0.085 | 상하 범위 (손가락을 아래로 유지할 수 있는 높이까지만) |
| `DEPTH_SCALE` | 0.3 | 앞뒤 범위 |
| `BASE_Z` | 0.255 | 손가락 패드가 큐브 옆면에 오는 파지 중심 높이 |
| `FINGER_OPEN`, `FINGER_CLOSED`, `DISTAL_SCALE` | -0.25, 0.15, 0.1 | 그리퍼 열림·닫힘. 바꾼 뒤에는 `check_grasp.py`가 9/9인지 확인한다 |

모델을 바꾸려면 `../URDF_1006`을 교체하고 `venv/bin/python motion2/arm/make_arm_urdf.py`를 실행한다. IK만 따로 보려면:

```bash
venv/bin/python motion2/arm/ik.py                       # 기본 타깃 풀고 오차(mm) 출력
venv/bin/python motion2/arm/ik.py --target 0.3 0.2 0.5  # world 좌표 타깃
venv/bin/mjpython motion2/arm/ik.py --interactive       # 뷰어 + 터미널에 "x y z" 입력으로 실시간 IK (종료: 창 닫기)
```

## 테스트 (웹캠·디스플레이 불필요)

```bash
./run_tests.sh             # 저장소 전체, 빠른 모드 약 1분 (동작 2는 아래 검사 전부 포함)
./run_tests.sh --full      # + 동작 1 성공률·영상 (약 3분 추가)
./run_tests.sh --viewers   # + 뷰어 창 열기 (화면이 켜져 있어야 함)

venv/bin/python motion2/scripts/check_grasp.py       # lifted 9 /9
venv/bin/python motion2/scripts/check_teleop.py      # teleop: 3/3 scenarios passed
venv/bin/python motion2/scripts/check_sender.py      # sender: ok
venv/bin/python motion2/scripts/check_dashboard.py   # dashboard: ok
venv/bin/mjpython motion2/scripts/check_viewer.py    # 뷰어 환경 확인 (macOS는 mjpython, 화면 필요)
```

## 알아낸 것

- **손가락 판의 기울기가 파지를 좌우한다.** 먼쪽 관절 비율 `DISTAL_SCALE`이 크면 접촉 때 판이 기울어 큐브 모서리만 물고 들어올릴 때 미끄러진다. `FINGER_CLOSED=0.15`에서 `DISTAL_SCALE` 0~0.10은 9/9, 0.15는 7/9, 0.20은 0/9, 0.30은 2/9이고, `DISTAL_SCALE=0.1`에서 `FINGER_CLOSED` 0.10~0.20은 모두 9/9다 (2026-10-13, `check_grasp.py` 절차).
- **판 끝은 파지 중심보다 17.5 mm 아래까지 내려온다.** 그래서 `BASE_Z=0.255`(큐브 바닥보다 5 mm 위)에서 판 끝이 큐브 바닥보다 약 12.5 mm 아래(기둥 옆)에 오고 평평한 패드 면이 큐브 옆면에 닿는다. 열린 손의 패드 간격은 큐브 높이에서 약 75~83 mm다.
- **qpos를 직접 쓰지 않는다.** 모든 목표를 `<position>` actuator ctrl로 보내고 `mj_step`이 접촉을 적분한다. qpos를 쓰면 손가락이 큐브를 뚫는다. 손가락 qpos도 열림 자세로 시작해야 한다(q=0이면 패드 간격이 큐브 폭과 같아 초기 접촉이 생긴다).
- **깊이 캘리브레이션이 필수다.** MediaPipe pose z는 사람·자세마다 기준이 -0.8~-1.9까지 달라서, 빼지 않으면 목표점이 팔 뒤쪽으로 가 팔이 안 움직인다.
- **팔꿈치 스윙을 그대로 따르면 큐브가 튕긴다** (`scripts/check_teleop.py`의 `swing-jump`). 원인 셋을 따로 측정해 각각 막았다. 스윙 한계 70°는 큐브 위치에서 들어올릴 때 IK 해가 0.2~0.5 rad 점프하는 범위라 ±40°로 줄였고(±40° 안쪽은 0.02 rad 이하), 팔이 수직에 가까우면 측정 스윙이 한 프레임에 90° 뛰어 변화 속도를 90°/s로 제한했고, 큐브를 쥔 채 스윙이 움직이면 파지가 흔들려(20°/s에서 15 mm 미끄러짐, 45°/s 이상이면 튕김) 쥐는 동안은 고정했다. 한계와 고정은 빼면 즉시 실패하고, 속도 제한은 빼도 통과하지만 급변 구간의 손 이탈이 2.5 → 7.9 mm, 서보 지연이 0.10 → 0.45 rad로 커진다.

## 주의

- 항상 `venv/bin/python`을 쓴다(전역 python 금지). **macOS에서 MuJoCo 뷰어를 띄우는 스크립트는 `venv/bin/mjpython`으로 실행해야 한다**(`src/mujoco_receiver.py`, `arm/ik.py --view`, `scripts/check_viewer.py`, 동작 1의 `watch_grasp.py`). 일반 python은 `launch_passive requires ... mjpython` 오류가 난다. `scripts/run_receiver.sh`는 이를 자동으로 고른다.
- 뷰어가 `Segmentation fault: 11`로 바로 죽으면 화면이 꺼져 있거나 잠겨 있는지 확인한다. MuJoCo/GLFW는 연결된 모니터가 없으면 뷰어를 만들다가 죽는다. 화면이 잠겨 있을 때는 죽는 대신 `mjpython` 자체가 멈추기도 한다(잠긴 콘솔에서 `mjpython -c 'print(1)'`이 응답하지 않는 것을 확인). 화면을 켜고 잠금을 푼 뒤 다시 실행하면 된다.
- 폴더를 옮기거나 이름을 바꾸면 `venv/bin/pip`·`mjpython`의 셔뱅이 깨진다. `python -m pip`가 안전하고, 깨졌으면 `python -m venv --upgrade`나 venv를 다시 만든다.
- `arm/arm.urdf`, `arm/arm.xml`, `arm/meshes/`는 생성 파일이지만 커밋되어 있다. 생성기나 URDF를 바꾸면 다시 생성해 함께 커밋한다(`./run_tests.sh`가 생성기 출력과 저장소 파일이 같은지 확인한다). 모델 파일 `models/*.task`(합쳐 약 17 MB)도 커밋되어 있다.
