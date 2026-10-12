<!-- Parent: ../AGENTS.md -->
<!-- Generated: 2026-07-02 | Updated: 2026-10-13 -->

# arm/ — 7DOF 팔 IK 모델 (최종 URDF `JAXV2_Assembly` 기반)

## Purpose
팀의 최종 URDF(`../../URDF_1006/JAXV2_Assembly_description`)에서 생성한 단일 팔(7 revolute joint) 모델.
ikpy용 URDF, MuJoCo MJCF, 생성기(`make_arm_urdf.py`), IK 풀이 + 검증 스크립트로 구성된다.
`arm.xml`에는 3손가락 손(6 hinge joint, `../mujoco_receiver.py`가 검지/중지/약지 curl 값으로
손가락별 구동), 파지 대상 씬(좁은 받침 기둥 + freejoint 큐브, contact/마찰 활성), 13개 position
actuator가 있다 — `arm.urdf`/ikpy 체인에는 손이 포함되지 않으므로 IK와 독립적이다.

## Key Files

| File | Description |
|------|-------------|
| `make_arm_urdf.py` | **생성기.** 최종 URDF를 읽어 `arm.urdf`, `arm.xml`, `meshes/`를 만든다. 튜닝 상수(파지 중심 `GRASP_CENTER_HAND`, 손가락3 회전각 `ROTATION_BASE_ANGLE`, 기둥/큐브 크기, 서보 게인, 접촉 `solref`)는 이 파일 상단에만 있다. 생성 파일은 직접 수정하지 말 것. |
| `arm.urdf` | (생성) ikpy용 7-revolute 체인. 관절 순서: `shoulder_roll → shoulder_yaw → upper_arm_roll → elbow → wrist_pitch → wrist_yaw → hand_roll`. 모든 축은 로컬 z, 영점 자세는 팔이 아래로 늘어지며 손가락이 -z. `base_link`는 고정된 Shoulder_Pitch 프레임(기둥 오프셋은 `BASE_POS_WORLD`). `end_effector_link`는 손가락 사이 파지 중심이고 z축이 손가락 반대 방향 → 월드 +z와 정렬하면 손가락이 아래(palm-down). |
| `arm.xml` | (생성) 같은 7관절 + 3손가락 손 + 바닥/기둥/큐브 + 13개 position actuator. 로봇 지지대 `column`은 `(0.50, 0, 0)`에 서 있고 어깨 프레임은 월드 `(0.34, 0, 0.85)`, 팔은 로컬 -x(월드 -x) 쪽으로 뻗는다. 큐브를 올려 둔 좁은 받침 기둥은 `pedestal`(`(0.12, 0, 0.125)`, 윗면 z=0.25)로 `column`과 다르다. 시각 메시는 OBJ(`meshes/`), 손가락 6개만 충돌(collision STL을 감싸는 박스, 환경변수 `JAX_FINGER_COLLISION=mesh`면 STL 그대로; 그룹 3, 뷰어에서 숨김). `nq == 20`(팔 7 + 손가락 6 + 큐브 7), `nu == 13`. |
| `meshes/` | (생성) 최종 URDF의 DAE를 OBJ로 변환한 시각 메시 + 손가락 충돌 STL 6개. |
| `ik.py` | ikpy로 world-space 목표(x,y,z)에 대한 IK를 풀고, 결과를 `arm.xml`에 적용해 `end_effector` site 오차를 검증. 손가락 관절은 건드리지 않음 — `apply_pose()`는 `qpos[:7]`만 씀. |
| `One_Arm_JAX_URDF_description/` | 이전 CAD(`One_Arm_JAX_URDF`) 잔재. **현재 사용하지 않음**(참고용으로만 남김). |
| `역운.code-workspace` | 저장소 루트를 여는 VS Code 워크스페이스 |

## For AI Agents

### Working In This Directory
- URDF가 바뀌면 `venv/bin/python motion2/arm/make_arm_urdf.py`로 세 산출물을 한 번에 재생성한다. 손으로 고치지 말 것 — `arm.urdf`와 `arm.xml`의 FK 일치는 생성기가 보장한다.
- 생성 후 `venv/bin/python motion2/arm/ik.py`로 오차를 재확인하고, ikpy FK ↔ MuJoCo `end_effector` site 일치(1e-9 수준)를 확인한다.
- ikpy는 chain에서 base link(index 0)와 end-effector link(index -1)를 fixed dummy로 포함 — `active_links_mask`로 7개 실제 관절만 활성화.
- IK 수렴 실패 시 가장 먼저 `SEED_POSE` 확인 (all-zero는 특이점).

### Testing Requirements
```bash
venv/bin/python motion2/arm/ik.py                       # 기본 타깃 오차(mm)
venv/bin/python motion2/arm/ik.py --target 0.12 0 0.28  # 커스텀 타깃
venv/bin/mjpython motion2/arm/ik.py --view              # 정적 자세 확인 (macOS, 디스플레이 필요)
venv/bin/mjpython motion2/arm/ik.py --interactive       # 뷰어 + "x y z" 입력으로 실시간 IK
```

### Common Patterns
- **좌표계**: ikpy chain은 `base_link`(어깨) 기준 local frame. `ik.py`의 `BASE_POS_WORLD = [0.34, 0, 0.85]`를 빼서 변환.
- **SEED_POSE**: 큐브 위 palm-down 자세를 multi-start IK로 구해 관절 한계 여유가 가장 큰 것을 고른 값.
- **qpos 인덱싱**: 팔 7개 hinge = qpos[0..6], 손가락 6개 = qpos[7..12](finger1 j1,j2 → finger2 → finger3), 큐브 freejoint = qpos[13..19].
- **손가락 기하**: 손가락 판은 손 프레임 x 방향으로 얇다. 손가락 1·2가 한쪽, 손가락 3이 `ROTATION_BASE_ANGLE=-π`로 반대쪽에 서서 마주 본다. 근위 관절 q=0에서 패드 간격이 30 mm(=큐브 폭)이므로 열림은 q=-0.25(큐브 높이에서 약 75~83 mm)이고, 먼쪽 관절이 간격을 크게 좌우한다. 판 끝이 `end_effector`보다 약 17 mm 아래까지 내려온다.
- **접촉**: 손가락 링크가 0.01–0.07 kg로 가벼워 기본 접촉(solref 0.02)은 너무 물렁해 큐브를 10–19 mm 파고든다 → 손가락/큐브 geom에 `solref="0.006 1"`, 손가락3 서보는 반대편 힘 균형을 위해 kp 2배.
- **뷰어 모드** (`ik.py --view`/`--interactive` 한정): `mj_step` 호출 없음. 물리 구동은 `../mujoco_receiver.py`.

## Dependencies

### Internal
- `../../URDF_1006/JAXV2_Assembly_description/` — 생성기의 입력(최종 URDF + 메시). 이 폴더를 바꾸면 생성기를 다시 돌린다.
- `../../CLAUDE.md` §2/§4 — 하드웨어 관절 순서 및 스펙 기준

### External
| Package | 용도 |
|---------|------|
| `ikpy` | URDF 체인 파싱, SLSQP 기반 IK 풀이 |
| `mujoco` | MJCF 로드, forward kinematics, passive viewer |
| `scipy` | 생성기의 쿼터니언 변환, 수신기의 팔 전체 IK |
| `numpy` | 좌표 변환, 오차 계산 |

<!-- MANUAL: -->
