<!-- Parent: ../AGENTS.md -->
<!-- Generated: 2026-10-13 | Updated: 2026-10-13 -->

# JAXV2_Assembly_description/ — 최종 URDF 패키지 (Fusion 360 export)

## Purpose
팀 로봇 전체(`JAXV2_Assembly`)를 Fusion URDF/XACRO Exporter v3.1.1이 내보낸 ROS 2 설명 패키지다. 링크 16, 관절 15(움직이는 것 14), 총 23.917 kg, 루트 `base_link`. 이 레포의 코드가 읽는 것은 **평평한 URDF와 메시뿐**이다(`motion2/arm/make_arm_urdf.py`가 읽고, 동작 1은 그 생성기를 거쳐 쓴다). 나머지는 ROS 2에서 RViz로 보거나 컨트롤러를 붙일 때 쓰는 파일이다. 읽기 전용 원본이다.

## Key Files
| File | Description |
|------|-------------|
| `urdf/JAXV2_Assembly.urdf` | **생성기의 입력.** 평평한 URDF. 링크·관절 이름과 한계의 기준 |
| `meshes/JAXV2_Assembly/` | 링크마다 시각용 `.dae` 16개 + 충돌용 `_collision.stl` 16개. 생성기가 DAE를 OBJ로 바꿔 `motion2/arm/meshes/`에 두고, 손가락 6개의 STL은 충돌 박스를 맞추는 기준으로 쓴다 |
| `README.md` | 링크 질량·충돌 형태 표, 관절 표, 운동학 트리(익스포터 생성). 파일 표에 시각 메시가 OBJ라고 적혀 있지만 실제로는 DAE다 |
| `urdf/JAXV2_Assembly.urdf.xacro`, `urdf/assemblies/` | xacro 버전(진입점과 어셈블리 매크로). 이 레포는 읽지 않는다 |
| `robot_data.yaml` | URDF 밖의 보조 데이터(메타데이터 등). 읽는 코드 없음 |
| `config/` | 익스포터의 프레임 규칙(`config/FRAME_OVERRIDES.md`, `config/frame_overrides.csv`), `joint_state.yaml`, `ros2_controllers.yaml` |
| `docs/transforms.md` | 변환 행렬 문서 |
| `launch/display.launch.py`, `rviz/display.rviz` | RViz2로 보는 ROS 2 실행 파일. 이 레포에서는 쓰지 않는다 |
| `package.xml`, `CMakeLists.txt` | ROS 2 패키지 매니페스트(`ament_cmake`) |
| `images/robot.png` | 미리보기 |

`.omc/`는 도구 상태 폴더라 무시한다. 하위 폴더(`config/`, `docs/`, `images/`, `launch/`, `meshes/`, `rviz/`, `urdf/`)는 모두 익스포터 산출물이라 따로 문서를 두지 않는다.

## For AI Agents

### Working In This Directory
- 수정하지 않는다. 새 CAD export로 바꿀 때는 폴더째 교체하고 이 순서로 확인한다: ① `venv/bin/python motion2/arm/make_arm_urdf.py` (관절 이름이 바뀌면 생성기의 `ARM_JOINTS`·`FINGER_JOINTS` 대응이 어긋난다) ② 생성기 상수(`GRASP_CENTER_HAND`, `BASE_LINK_WORLD`, `ROTATION_BASE_ANGLE` 등)가 새 기하와 맞는지 ③ `./run_tests.sh`와 `motion1/scripts/build_reachmap.py`. 재생성한 `arm.xml`·`arm.urdf`·`meshes/`를 커밋하지 않으면 `run_tests.sh`의 "생성기 출력 == 레포 파일" 검사가 실패한다.
- 관절 이름 `finger_1_2_to_finger_2_2`는 export의 이름 착오처럼 보이지만 실제로 `Finger_2_1`→`Finger_2_2`를 잇는다(URDF를 파싱해 확인). 이름을 고치지 말고 생성기의 대응표를 따른다.

### URDF 관절 → 프로젝트 이름 (생성기의 `ARM_JOINTS`, `FINGER_JOINTS`)
| URDF 관절 | 프로젝트 이름 | URDF 한계 |
|-----------|---------------|-----------|
| `shoulder_pitch_to_shoulder_roll` | `shoulder_roll` | −45° ~ 45° |
| `shoulder_roll_to_shoulder_yaw` | `shoulder_yaw` | 0° ~ 45° |
| `shoulder_yaw_to_upper_arm` | `upper_arm_roll` | −90° ~ 240° |
| `upper_arm_to_under_arm` | `elbow` | −105° ~ 105° |
| `under_arm_to_wrist_pitch` | `wrist_pitch` | −90° ~ 90° |
| `wrist_pitch_to_wrist_yaw` | `wrist_yaw` | −90° ~ 90° |
| `wrist_yaw_to_hand_base` | `hand_roll` | −120° ~ 120° |
| `hand_base_to_finger_1_1` / `finger_1_1_to_finger_1_2` | `finger1_joint1` / `finger1_joint2` | −90° ~ 90° / 0° ~ 120° |
| `hand_base_to_finger_2_1` / `finger_1_2_to_finger_2_2` | `finger2_joint1` / `finger2_joint2` | 〃 |
| `rotation_base_to_finger_3_1` / `finger_3_1_to_finger_3_2` | `finger3_joint1` / `finger3_joint2` | 〃 |
| `hand_base_to_rotation_base` | (모델에서 −π로 고정) | −180° ~ 0° |

`base_link_to_shoulder_pitch`는 고정 관절이다. 한계는 URDF를 직접 파싱해 이 표와 같음을 확인했다(2026-10-13).

### Testing Requirements
이 폴더 자체의 테스트는 없다. 루트 `./run_tests.sh`의 `m2_generate`(생성기 실행 + 결과가 레포 파일과 같은지)와 `check_docs.py`가 이름 변경을 잡는다.

## Dependencies

### Internal
- 읽는 쪽: `../../motion2/arm/make_arm_urdf.py` (그리고 그 생성기를 쓰는 `../../motion1/src/humanoid_grasp_rl/assets.py`)

### External
- 없음(이 레포에서는). ROS 2(`robot_state_publisher`, `rviz2`, `xacro`, `ros2_control` 등)는 `launch/`를 실행할 때만 필요하다.

<!-- MANUAL: -->
