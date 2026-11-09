<!-- Parent: ../AGENTS.md -->
<!-- Generated: 2026-11-09 | Updated: 2026-11-09 -->

# JAXV3_Assembly_description/ — 최종 URDF 패키지 V3 (Fusion 360 export)

## Purpose
팀 로봇 전체(`JAXV3_Assembly`)를 Fusion URDF/XACRO Exporter v3.1.1이 내보낸 ROS 2 설명 패키지다. 링크 17, 관절 16(움직이는 것 14), 총 143.370 kg(받침대 `base_link`만 137.266 kg), 루트 `base_link`. 이 레포의 코드가 읽는 것은 **평평한 URDF와 시각 메시뿐**이다(`motion2/arm/make_arm_urdf.py`의 `load()`가 읽고, 동작 1은 그 생성기를 거쳐 쓴다). 나머지는 ROS 2에서 RViz로 보거나 컨트롤러를 붙일 때 쓰는 파일이다. 읽기 전용 원본이다.

## Key Files
| File | Description |
|------|-------------|
| `urdf/JAXV3_Assembly.urdf` | **생성기의 입력.** 평평한 URDF. 관절 위치·한계와 시각 메시 배치의 기준 |
| `meshes/JAXV3_Assembly/` | 링크마다 시각용 `.dae` 17개 + 충돌용 `_collision.stl` 17개. 생성기가 DAE를 OBJ로 바꿔 `motion2/arm/meshes/`에 둔다. 충돌 STL은 쓰지 않는다(아래) |
| `README.md` | 링크 질량·충돌 형태 표, 관절 표, 운동학 트리(익스포터 생성) |
| `urdf/JAXV3_Assembly.urdf.xacro`, `urdf/assemblies/` | xacro 버전. 이 레포는 읽지 않는다 |
| `robot_data.yaml` | URDF 밖의 보조 데이터(메타데이터 등). 읽는 코드 없음 |
| `config/` | 익스포터의 프레임 규칙, `joint_state.yaml`, `ros2_controllers.yaml` |
| `docs/transforms.md` | 변환 행렬 문서 |
| `launch/display.launch.py`, `rviz/display.rviz` | RViz2로 보는 ROS 2 실행 파일. 이 레포에서는 쓰지 않는다 |
| `package.xml`, `CMakeLists.txt` | ROS 2 패키지 매니페스트(`ament_cmake`) |
| `images/robot.png` | 미리보기 |

하위 폴더는 모두 익스포터 산출물이라 따로 문서를 두지 않는다.

## V2(`URDF_1006`)와 다른 점과 생성기가 하는 일
URDF 두 개를 직접 파싱해 영점 자세에서 비교한 값이다(2026-11-09).

| 항목 | V2 | V3 | 생성기(`load()`)의 모델 |
|------|----|----|------------------------|
| 링크 이름 | `Shoulder_Pitch`, `Finger_1_1` … | 익스포터가 `_1`을 붙임(`Shoulder_Pitch_1`, `Finger_1_1_1`), `base_link`만 그대로 | `_1`을 떼어 V2 이름 그대로(`LINK_SUFFIX`) |
| `Holder_1` | 없음 | `base_link`에 고정된 새 링크(3.331 kg) | 받침대에 붙은 외형만(`Holder` body, 충돌·관성 없음) |
| `base_link_to_shoulder_pitch` | (−0.16, 0, 0.85) | (−0.15, 0, 1.2125): `base_link`가 1.2 m 받침대 전체 | `SHOULDER_OFFSET`. 어깨 월드 위치는 `BASE_LINK_WORLD`로 그대로 둔다 |
| `shoulder_roll_to_shoulder_yaw` | 0° ~ 45° | 자식 프레임이 x축으로 180° 돌아 축이 반대, −50° ~ 0° | V2 부호로 되돌림(`FLIPPED_JOINTS`): **0° ~ 50°** |
| `upper_arm_to_under_arm` | −105° ~ 105° | −100° ~ 100° | −100° ~ 100° |
| `under_arm_to_wrist_pitch` | −90° ~ 90° | 축이 반대(한계는 같음), 관절 위치 0.8 mm 이동 | V2 부호로 되돌림, −90° ~ 90° |
| 손(손가락 관절, `RotationBody`) | | 손목 쪽으로 3.72 mm(손 프레임 z +3.5 mm) | V3 그대로 |
| 손가락 2 | 손 프레임 y 0.003665 | y 0.006665: 손가락 1–2 간격 37 → 40 mm | V3 그대로 |
| 질량·관성 | 팔(어깨 피치 링크 제외) 4.068 kg | 1.827 kg(CAD 재질 값) | **V2 그대로**(팀이 실제 팔을 재기 전까지) |
| 충돌 형상 | 링크별 STL | 익스포터의 primitive(원기둥·상자) | 손가락 6개만 쓰고, **V2 STL** |

축이 뒤집힌 두 관절은 원래 V3 URDF(그리고 그것을 그대로 쓰는 ROS 쪽)에서 모델 각도의 **음수**로 읽힌다. 모델이 V2 부호를 지키는 이유: IK 시드, 동작 1의 자세·도달 지도, 팀원의 Isaac 정책(team-jax/JaxIsaacLab의 V3 URDF도 같은 두 관절을 뒤집었다)이 모두 V2 부호로 만들어졌다. 생성된 모델은 원래 V3 URDF와 21개 무작위 자세에서 링크 프레임이 4e-6 mm, 3e-6° 안에서 같다(두 관절 부호만 반대).

## For AI Agents

### Working In This Directory
- 수정하지 않는다. 새 CAD export로 바꿀 때는 폴더째 교체하고 이 순서로 확인한다: ① `venv/bin/python motion2/arm/make_arm_urdf.py` (관절 이름이 바뀌면 생성기의 `ARM_JOINTS`·`FINGER_JOINTS` 대응이, 링크 이름 꼬리가 바뀌면 `LINK_SUFFIX`가, 축 방향이 바뀌면 `FLIPPED_JOINTS`가 어긋난다) ② 생성기 상수(`GRASP_CENTER_HAND`, `SHOULDER_OFFSET`, `BASE_LINK_WORLD`, `ROTATION_BASE_ANGLE` 등)가 새 기하와 맞는지 ③ `./run_tests.sh`와 `motion1/scripts/build_reachmap.py`(도달 지도 세 개). 재생성한 `arm.xml`·`arm.urdf`·`meshes/`를 커밋하지 않으면 `run_tests.sh`의 "생성기 출력 == 레포 파일" 검사가 실패한다.
- 관절 이름 `finger_1_2_to_finger_2_2`는 V2와 같이 `Finger_2_1`→`Finger_2_2`를 잇는다. 이름을 고치지 말고 생성기의 대응표를 따른다.

### URDF 관절 → 프로젝트 이름 (생성기의 `ARM_JOINTS`, `FINGER_JOINTS`)
| URDF 관절 | 프로젝트 이름 | URDF 한계 | 모델 한계(V2 부호) |
|-----------|---------------|-----------|--------------------|
| `shoulder_pitch_to_shoulder_roll` | `shoulder_roll` | −45° ~ 45° | 같음 |
| `shoulder_roll_to_shoulder_yaw` | `shoulder_yaw` | −50° ~ 0° (축 반대) | 0° ~ 50° |
| `shoulder_yaw_to_upper_arm` | `upper_arm_roll` | −90° ~ 240° | 같음 |
| `upper_arm_to_under_arm` | `elbow` | −100° ~ 100° | 같음 |
| `under_arm_to_wrist_pitch` | `wrist_pitch` | −90° ~ 90° (축 반대) | −90° ~ 90° |
| `wrist_pitch_to_wrist_yaw` | `wrist_yaw` | −90° ~ 90° | 같음 |
| `wrist_yaw_to_hand_base` | `hand_roll` | −120° ~ 120° | 같음 |
| `hand_base_to_finger_1_1` / `finger_1_1_to_finger_1_2` | `finger1_joint1` / `finger1_joint2` | −90° ~ 90° / 0° ~ 120° | 같음 |
| `hand_base_to_finger_2_1` / `finger_1_2_to_finger_2_2` | `finger2_joint1` / `finger2_joint2` | 〃 | 같음 |
| `rotation_base_to_finger_3_1` / `finger_3_1_to_finger_3_2` | `finger3_joint1` / `finger3_joint2` | 〃 | 같음 |
| `hand_base_to_rotation_base` | (모델에서 −π로 고정) | −180° ~ 0° | 고정 |

`base_link_to_shoulder_pitch`, `base_link_to_holder`는 고정 관절이다.

### Testing Requirements
이 폴더 자체의 테스트는 없다. 루트 `./run_tests.sh`의 `m2_generate`(생성기 실행 + 결과가 레포 파일과 같은지)와 `check_docs.py`가 이름 변경을 잡는다.

## Dependencies

### Internal
- 읽는 쪽: `../../motion2/arm/make_arm_urdf.py` (그리고 그 생성기를 쓰는 `../../motion1/src/humanoid_grasp_rl/assets.py`)

### External
- 없음(이 레포에서는). ROS 2(`robot_state_publisher`, `rviz2`, `xacro`, `ros2_control` 등)는 `launch/`를 실행할 때만 필요하다.

<!-- MANUAL: -->
