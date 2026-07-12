<!-- Parent: ../AGENTS.md -->
<!-- Generated: 2026-07-02 | Updated: 2026-07-09 -->

# arm/ — 7DOF 팔 IK 모델

## Purpose
CLAUDE.md §2/§4 하드웨어 스펙을 그대로 따르는 단일 팔(7 revolute joint) 키네마틱 모델.
ikpy용 URDF, MuJoCo 검증용 MJCF, IK 풀이 + 검증 스크립트로 구성된다. `arm.xml`에는 3손가락
그리퍼(6 hinge joint, `../mujoco_receiver.py`가 검지/중지/약지 curl 값으로 손가락별 구동),
파지 대상 씬(받침대 + freejoint 큐브, contact/마찰 활성), 13개 position actuator가 순수
MuJoCo 전용 구조로 추가되어 있음 — `arm.urdf`/ikpy 체인에는 포함되지 않으므로 IK와 완전히
독립적. 손가락↔큐브 접촉 물리로 실제 파지가 동작한다(`../mujoco_receiver.py`가 `mj_step`으로 구동).

## Key Files

| File | Description |
|------|-------------|
| `arm.urdf` | ikpy가 읽는 7-revolute-joint URDF 체인. 관절 순서: `shoulder_pitch → shoulder_roll → upper_arm_roll → elbow → wrist_yaw → wrist_pitch → wrist_roll`. 링크 길이(상완 0.30m / 전완 0.25m / 손 0.12m)는 기구 설계 확정 전 placeholder. 그리퍼는 포함하지 않음(순수 IK 체인). |
| `arm.xml` | `arm.urdf`와 동일한 관절 순서/축/링크 길이를 갖는 MuJoCo MJCF + `wrist_roll_link` 아래 3손가락 그리퍼(`gripper_base` → `fingerN_base`/`fingerN_distal`, N=1..3, 각 2 hinge joint) + 파지 씬(바닥/받침대/freejoint 큐브) + 13개 `<position>` actuator(팔 kp=100, 손가락 kp=5). `end_effector` site로 엔드 이펙터 위치 노출(그리퍼 추가와 무관하게 위치 고정). `arm.urdf` 수정 시 7개 팔 관절 부분만 같이 맞추면 됨 — 그리퍼/씬은 `arm.urdf`에 없으므로 동기화 대상 아님. `model.nq == 20`(팔 7 + 손가락 6 + 큐브 freejoint 7, 이 순서 고정 — 큐브 body가 팔 서브트리 뒤에 정의되어야 유지됨), `model.nu == 13`. 충돌은 contype/conaffinity 비트마스크로 제어: 손가락(2/1) ↔ 물체·받침대·바닥(1/3)만 활성, 손가락끼리와 팔 링크는 충돌 없음(기본 0/0). integrator는 `implicitfast` — 서보 강성(kp)을 timestep 안정성과 분리. |
| `ik.py` | ikpy로 world-space 목표(x,y,z)에 대한 IK를 풀고, 결과 관절각을 `arm.xml`에 적용해 `end_effector` site 위치로 오차를 검증. 그리퍼 관절은 건드리지 않음 — `apply_pose()`는 `qpos[:len(joint_angles)]`(=7)만 씀. |

## For AI Agents

### Working In This Directory
- `arm.urdf`와 `arm.xml`은 항상 동기화 유지. 한쪽만 수정하면 forward kinematics 불일치 발생.
- 링크 길이/관절 축 변경 시 두 파일 모두 수정하고 `ik.py`로 오차 재검증 필수.
- ikpy는 chain에서 base link(index 0)와 end-effector link(index -1)를 fixed dummy로 포함 — `active_links_mask`로 7개 실제 관절만 활성화.
- IK 수렴 실패 시 가장 먼저 `SEED_POSE` 확인 (all-zero는 특이점).

### Testing Requirements
```bash
# 기본 타깃으로 IK 풀고 오차 출력 (mm 단위, 수렴 시 거의 0)
venv/bin/python arm/ik.py

# 커스텀 타깃
venv/bin/python arm/ik.py --target 0.3 0.2 0.5

# MuJoCo passive viewer로 자세 확인 (로컬 디스플레이 필요)
venv/bin/mjpython arm/ik.py --view      # macOS

# 실시간 IK (뷰어 + 터미널 입력)
venv/bin/mjpython arm/ik.py --interactive
```

### Common Patterns
- **좌표계**: ikpy chain은 `base_link` 기준 local frame. `ik.py`의 `BASE_POS_WORLD = [0, 0, 0.8]`을 빼서 변환.
- **SEED_POSE**: `[0.0, 0.0, 0.3, 0.0, 0.5, 0.0, 0.3, 0.0, 0.0]` — 팔꿈치 살짝 굽힌 비특이 자세.
- **qpos 인덱싱**: `arm.xml`에서 팔 7개 hinge joint가 qpos[0..6], 손가락 6개가 qpos[7..12], 큐브 freejoint가 qpos[13..19]. `ik.py`의 `apply_pose()`는 qpos[:7]만 씀.
- **뷰어 모드** (`ik.py --view`/`--interactive` 한정): `mj_step` 호출 없음 — actuator ctrl을 세팅하지 않은 채 스텝하면 중력만 작용해 팔이 처짐. 물리 구동은 `../mujoco_receiver.py`가 담당(ctrl 타깃 + `mj_step`).
- **interactive 모드**: `launch_passive`의 렌더링 스레드 분리를 이용, main thread에서 `input()` 블로킹 + `viewer.lock()`으로 threading 없이 구현.

## Dependencies

### Internal
- `../CLAUDE.md` §2/§4 — 하드웨어 관절 순서 및 스펙 기준

### External
| Package | 용도 |
|---------|------|
| `ikpy` | URDF 체인 파싱, SLSQP 기반 IK 풀이 |
| `mujoco` 3.10.0 | MJCF 로드, forward kinematics, passive viewer |
| `numpy` | 좌표 변환, 오차 계산 |

<!-- MANUAL: -->
