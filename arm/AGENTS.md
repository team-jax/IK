<!-- Parent: ../AGENTS.md -->
<!-- Generated: 2026-07-02 | Updated: 2026-07-02 -->

# arm/ — 7DOF 팔 IK 모델

## Purpose
CLAUDE.md §2/§4 하드웨어 스펙을 그대로 따르는 단일 팔(7 revolute joint) 키네마틱 모델.
ikpy용 URDF, MuJoCo 검증용 MJCF, IK 풀이 + 검증 스크립트로 구성된다.
그리퍼/충돌/파지는 미구현 — 현재는 엔드 이펙터 위치 도달만 검증.

## Key Files

| File | Description |
|------|-------------|
| `arm.urdf` | ikpy가 읽는 7-revolute-joint URDF 체인. 관절 순서: `shoulder_pitch → shoulder_roll → upper_arm_roll → elbow → wrist_yaw → wrist_pitch → wrist_roll`. 링크 길이(상완 0.30m / 전완 0.25m / 손 0.12m)는 기구 설계 확정 전 placeholder. |
| `arm.xml` | `arm.urdf`와 동일한 관절 순서/축/링크 길이를 갖는 MuJoCo MJCF. `end_effector` site로 엔드 이펙터 위치 노출. `arm.urdf` 수정 시 반드시 같이 수정. |
| `ik.py` | ikpy로 world-space 목표(x,y,z)에 대한 IK를 풀고, 결과 관절각을 `arm.xml`에 적용해 `end_effector` site 위치로 오차를 검증. |

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
- **qpos 인덱싱**: `arm.xml`에서 7개 hinge joint가 순서대로 qpos[0..6]에 매핑됨.
- **뷰어 모드**: `mj_step` 호출 없음 — 동역학 적분 시 actuator 없이 중력만 작용해 팔이 처짐.
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
