# Deep Interview Spec: 손목 추적 → MuJoCo 팔 실시간 제어

## Metadata
- Interview ID: di-역운-001
- Rounds: 5 (+ Round 0 토폴로지)
- Final Ambiguity Score: 19%
- Type: brownfield
- Generated: 2026-07-02
- Threshold: 0.20
- Threshold Source: default
- Initial Context Summarized: no
- Status: PASSED

## Clarity Breakdown
| Dimension       | Score | Weight | Weighted |
|-----------------|-------|--------|----------|
| Goal Clarity    | 0.85  | 35%    | 0.298    |
| Constraint      | 0.75  | 25%    | 0.188    |
| Criteria        | 0.75  | 25%    | 0.188    |
| Context Clarity | 0.90  | 15%    | 0.135    |
| **Total Clarity**|      |        | **0.809**|
| **Ambiguity**   |       |        | **19%**  |

## Topology
| Component            | Status | Description                                      | Coverage                                    |
|----------------------|--------|--------------------------------------------------|---------------------------------------------|
| 좌표 변환 레이어     | active | MediaPipe 정규화 좌표 → arm world-space (meters) | 상수 기반, 런타임 튜닝 허용                 |
| 실시간 IK 통합       | active | 변환 좌표 → ikpy SLSQP → 7관절각 매 프레임       | IK 실패 시 last-good 유지                   |
| MuJoCo 팔 렌더링     | active | arm/arm.xml 모델 + passive viewer + qpos 적용    | mujoco_receiver.py 수정으로 구현            |

## Goal
`mujoco_receiver.py`를 수정하여, UDP로 수신한 손목 좌표(MediaPipe)를 `arm/arm.xml` 7DOF 팔 모델의 IK 입력으로 변환하고, 결과 관절각을 매 프레임 적용해 MuJoCo passive viewer에서 팔이 손목을 시각적으로 따라오게 한다.

## Constraints
- **수정 대상 파일**: `mujoco_receiver.py` 단일 파일만 수정 (새 파일 불필요)
- **IK 실패 처리**: `last-good` — 마지막 성공한 관절각을 보존, 실패 시 유지
- **좌표 스케일**: 파일 상단 상수로 선언, 실행 중 조정 가능하도록 분리
- **arm 모델**: `arm/arm.xml` 직접 로드 (`MjModel.from_xml_path`)
- **IK 재사용**: `arm/ik.py`의 `build_chain()`, `solve_ik()`, `apply_pose()` 직접 import
- **viewer**: macOS `mjpython` 필요 (기존 `run_receiver.sh` 유지)
- **UDP 포트**: 5005 유지, 논블로킹 수신 유지

## Non-Goals
- `vision_sender.py` 수정 없음 — 송신 쪽은 그대로
- `arm/ik.py` 수정 없음 — import해서 사용
- 새 파일 생성 없음
- 좌표 스케일 자동 캘리브레이션 없음 (수동 튜닝)
- 손가락/그리퍼 제어 없음
- IK 실패 시 보간/스무딩 없음

## Acceptance Criteria
- [ ] `venv/bin/mjpython mujoco_receiver.py` 실행 시 MuJoCo viewer에 7DOF 팔이 표시됨
- [ ] `vision_sender.py`를 동시에 실행하면 팔이 손목 방향으로 시각적으로 따라옴
- [ ] IK 수렴 실패 시 팔이 마지막 자세에서 멈추고 튀지 않음
- [ ] UDP 패킷이 없을 때 viewer가 멈추지 않음 (논블로킹 유지)
- [ ] 파일 상단의 스케일 상수를 바꾸면 팔 움직임 범위가 변함

## Assumptions Exposed & Resolved
| Assumption                         | Challenge                                 | Resolution                                       |
|------------------------------------|-------------------------------------------|--------------------------------------------------|
| 정확한 1:1 스케일 매핑 필요         | "스케일은 지금 결정 안 해도 된다"          | 상수로 먼저 넣고 실행하며 튜닝                   |
| IK는 항상 실시간으로 수렴           | "실패하면 어떻게 할까?"                    | last-good 자세 유지                              |
| arm workspace 전체 커버 필요       | "어느 범위를 커버하길 원하는가?"           | 팔 도달 가능 범위 내면 충분, 클리핑 OK           |
| 지연 0.1초 이하 필요               | "0.5초 지연도 허용 가능한가?"             | 시각적으로 따라오면 OK, 엄격한 레이턴시 요건 없음|
| 새 파일 분리 필요                  | "mujoco_receiver.py 수정으로 충분한가?"   | 단일 파일 수정으로 구현                          |

## Technical Context

### 기존 코드 (brownfield)
- `mujoco_receiver.py:29-42` — 인라인 MJCF에 sphere body 정의 → **arm/arm.xml로 교체**
- `mujoco_receiver.py:49-50` — `target_body_id`, `qpos_start` sphere 기준 → **7관절 qpos[0:7]로 교체**
- `mujoco_receiver.py:61-73` — 좌표 변환 + qpos 쓰기 → **IK 호출로 교체**
- `mujoco_receiver.py:25-27` — `X_SCALE`, `Y_SCALE`, `Z_SCALE`, `Z_OFFSET` → **arm workspace용 상수로 교체**
- `arm/ik.py:38-55` — `build_chain()`, `solve_ik()`, `apply_pose()` — **import해서 재사용**
- `arm/arm.xml` — base at `[0,0,0.8]`, 7 hinge joints, `end_effector` site
- `arm/ik.py:29` — `BASE_POS_WORLD = [0.0, 0.0, 0.8]` — IK world→local 변환 기준

### 좌표 변환 방향 (초기값 — 튜닝 대상)
```
MediaPipe x ∈ [0,1] (left→right)  →  sim y축 (lateral):  (x_norm - 0.5) * LATERAL_SCALE
MediaPipe y ∈ [0,1] (top→bottom)  →  sim z축 (height):   (1 - y_norm) * HEIGHT_SCALE + BASE_Z
MediaPipe z (depth, relative)      →  sim x축 (forward):  z_m * DEPTH_SCALE
```
결과 `[sim_x, sim_y, sim_z]`를 `solve_ik(chain, target_world)`에 world-space 좌표로 전달.

## Ontology (Key Entities)
| Entity         | Type          | Fields                                | Relationships                              |
|----------------|---------------|---------------------------------------|--------------------------------------------|
| WristCoord     | core domain   | x_norm, y_norm, z_m (MediaPipe)       | → (변환) → ArmTarget                      |
| ArmTarget      | core domain   | x, y, z (world meters)               | → solve_ik → JointAngles                  |
| IKSolver       | external tool | chain (ikpy), SEED_POSE, BASE_POS_WORLD | solve_ik(chain, target) → JointAngles   |
| JointAngles    | core domain   | q[0..6] (rad, 7DOF)                  | applied to MjData.qpos                    |
| LastGoodPose   | supporting    | q[0..6] (last successful)            | fallback when IK fails                    |

## Ontology Convergence
| Round | Entity Count | New | Changed | Stable | Stability Ratio |
|-------|-------------|-----|---------|--------|----------------|
| 1     | 3           | 3   | -       | -      | N/A            |
| 2     | 3           | 0   | 0       | 3      | 100%           |
| 3     | 4           | 1   | 0       | 3      | 75%            |
| 4     | 4           | 0   | 0       | 4      | 100%           |
| 5     | 4           | 0   | 0       | 4      | 100%           |

## Interview Transcript
<details>
<summary>Full Q&A (5 rounds + Round 0)</summary>

### Round 0 (토폴로지)
**Q:** 3개 컴포넌트(좌표변환 / IK통합 / 팔렌더링) 토폴로지가 맞나요?
**A:** 맞음, 3개 모두

### Round 1 (coord-transform / Goal)
**Q:** 손목으로 팔 어디를 커버하기를 원하시나요?
**A:** 암 바구니 한 곳에 놓을 수 있는 범위 (팔 도달 가능 범위)

### Round 2 (arm-rendering / Goal)
**Q:** 최종 결과물이 어떤 형태이길 원하시나요?
**A:** mujoco_receiver.py 수정

### Round 3 (realtime-ik / Constraints)
**Q:** IK가 실패하거나 에러가 나면 어떻게 할까요?
**A:** 지난 자세 유지 (last-good를 꿀림)

### Round 4 (realtime-ik / Criteria — Contrarian)
**Q:** 팔이 손목을 시각적으로 따라오면 성공인가요?
**A:** 네, 팔이 손목을 시각적으로 따라오면 나의 (목표)

### Round 5 (coord-transform / Constraints — Simplifier)
**Q:** 좌표 스케일을 어떻게 정하기를 원하시나요?
**A:** 상수로 일단 넣고 나중에 튜닝

</details>
