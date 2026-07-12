# Deep Interview Spec: 파지(그리퍼) 시뮬레이션 — 손 추적 텔레오퍼레이션으로 물체 집기

## Metadata
- Interview ID: di-yeokun-002-grasp-sim
- Rounds: 5 (+ Round 0 토폴로지)
- Final Ambiguity Score: 15%
- Type: brownfield
- Generated: 2026-07-07
- Threshold: 0.20
- Threshold Source: default
- Initial Context Summarized: no
- Status: PASSED

## Clarity Breakdown
| Dimension        | Score | Weight | Weighted |
|------------------|-------|--------|----------|
| Goal Clarity     | 0.90  | 35%    | 0.315    |
| Constraint       | 0.80  | 25%    | 0.200    |
| Criteria         | 0.80  | 25%    | 0.200    |
| Context Clarity  | 0.90  | 15%    | 0.135    |
| **Total Clarity**|       |        | **0.850**|
| **Ambiguity**    |       |        | **15%**  |

## Topology
| Component        | Status | Description                                            | Coverage                                        |
|------------------|--------|--------------------------------------------------------|--------------------------------------------------|
| 물리 씬 전환     | active | kinematic(qpos+mj_forward) → 접촉 물리(actuator+mj_step) | 기존 파일 직접 전환, 손 추적 텔레오퍼레이션 유지 |
| 파지 대상 물체   | active | 작은 공/큐브 1개 + 작업공간 높이 받침대                | 크기/질량 상수 튜닝, 받침대 z≈0.5               |
| 파지 실행·검증   | active | 손으로 조작해 물체를 집어 들어올림                     | 판정 코드 없음 — 눈으로 확인                    |

## Goal
`mujoco_receiver.py`와 `arm/arm.xml`을 접촉 물리 기반으로 전환하여, 사용자가 카메라 앞에서
자기 손을 움직여(기존 vision_sender.py 파이프라인 그대로) MuJoCo의 7DOF 팔 + 3손가락
그리퍼로 받침대 위의 작은 물체(공 또는 큐브 1개)를 실제 접촉 물리로 집어 들어올릴 수 있게
한다. 성공은 눈으로 확인한다 — 물체가 손가락 사이에 물려 함께 올라오면 성공.

## Constraints
- **수정 대상**: `mujoco_receiver.py` + `arm/arm.xml` 직접 전환 (새 파일 없음, 기존
  kinematic 동작은 git 이력에만 남음)
- **무수정**: `vision_sender.py`, `arm/ik.py`, `arm/arm.urdf` — IK 체인은 충돌 지오메트리와
  무관하므로 영향 없음
- **제어 전환**: qpos 직접 쓰기 → 관절 액추에이터(position servo)가 IK 결과/curl 목표를
  추종 + `mj_step` 물리 적분. IK 호출·last-good 폴백·UDP 논블로킹 수신(포트 5005)·
  캘리브레이션('C')·카메라 토글('T') 등 기존 동작 유지
- **충돌**: 그리퍼 손가락, 물체, 받침대는 충돌 활성화. (팔 링크 자체는 단순화를 위해
  비충돌 유지 가능 — 구현 재량)
- **물체**: 그리퍼 개방폭(~5cm)보다 작은 공/큐브 1개. 크기·질량·마찰은 파일 상단 상수로
  선언해 튜닝 가능하게
- **받침대**: 팔 작업공간 높이(z≈0.5)에 맞춘 고정 받침대/테이블을 씬에 추가, 그 위에 물체 배치
- **viewer**: macOS `mjpython` 필요 (기존 `run_receiver.sh` 유지)

## Non-Goals
- 파지 성공 자동 판정/로깅 없음 (눈으로 확인)
- 자동(스크립트) pick-and-place 시퀀스 없음 — 손 추적 텔레오퍼레이션만
- 바구니/놓기 목표 위치 없음 — 집어 들어올리면 끝
- 물체 여러 개, 모양 다양화 없음
- vision_sender.py 쪽 변경 없음
- 물리/kinematic 모드 전환 플래그 없음

## Acceptance Criteria
- [ ] `venv/bin/mjpython mujoco_receiver.py` 실행 시 받침대 + 물체가 씬에 표시됨
- [ ] 물체가 받침대 위에 물리적으로 안착함 (뚫고 떨어지거나 튀지 않음)
- [ ] `vision_sender.py` 실행 시 팔이 손목을 따라 물체에 접근 가능
- [ ] 손가락을 오므리면 손가락이 물체를 통과하지 않고 접촉함
- [ ] 물체를 감싸고 손목을 올리면 물체가 손가락 사이에 물려 함께 올라옴
- [ ] 손가락을 펴면 물체가 낙하함
- [ ] IK 실패/UDP 무패킷 시에도 viewer가 멈추거나 팔이 튀지 않음 (기존 동작 유지)
- [ ] 파일 상단 상수(물체 크기·질량, 받침대 위치 등)를 바꾸면 씬이 그에 맞게 변함

## Assumptions Exposed & Resolved
| Assumption                             | Challenge                                      | Resolution                                  |
|----------------------------------------|------------------------------------------------|---------------------------------------------|
| 파지 데모 = 자동 pick-and-place        | "무엇이 구동하길 원하는가?"                    | 손 추적 텔레오퍼레이션 (내 손으로 집기)     |
| 여러 물체/바구니 필요                  | "어떤 물체를 놓고 싶은가?"                     | 작은 공/큐브 1개, 들어올리면 성공           |
| 기존 kinematic 데모 보존 필요          | "지금 잘 되는 데모를 어떻게 처리할까?"         | 기존 파일 직접 전환, git 이력으로 충분      |
| 파지 성공 판정 코드 필요               | (Contrarian) "판정이 정말 필요한가?"           | 눈으로 확인만 — 판정 로직 불필요            |
| 물체는 바닥에 두면 됨                  | "팔 작업공간(z≈0.4~0.8) 대비 바닥은 z=0"       | 작업공간 높이 받침대 위에 배치              |

## Technical Context

### 기존 코드 (brownfield)
- `arm/arm.xml` — 7DOF 팔 + 3손가락 그리퍼(각 손가락 proximal+distal 힌지, qpos[7:13]).
  **모든 geom이 `contype=0 conaffinity=0`(default 블록)** — 충돌 전면 비활성 상태
- `mujoco_receiver.py` — UDP(5005)로 손목좌표+per-finger curl 수신 → ikpy IK →
  `apply_pose()`(qpos[:7] 직접 쓰기) + `apply_grip()`(qpos[7:13] 직접 쓰기) → `mj_forward`.
  **`mj_step` 없음, actuator 없음** — 순수 kinematic 퍼펫
- `arm/ik.py` — `build_chain()`, `solve_ik()`(SEED_POSE 워름스타트), `apply_pose()`;
  `BASE_POS_WORLD=[0,0,0.8]`. 충돌과 무관하므로 무수정
- `vision_sender.py` — MediaPipe pose+hand, "x y z curl_i curl_m curl_r" 전송. 무수정
- 이전 스펙: `.omc/specs/deep-interview-wrist-to-arm.md` (구현 완료 — 본 스펙의 기반)

### 핵심 전환 포인트
1. **arm.xml**: default `contype=0 conaffinity=0` 해제(최소한 손가락·물체·받침대에서),
   받침대(고정 geom, z≈0.5)와 자유 물체(`freejoint` body, 공/큐브) 추가, 손가락·물체에
   마찰·질량 등 접촉 파라미터 부여, 관절 액추에이터(position) 정의
2. **mujoco_receiver.py**: `apply_pose`/`apply_grip`의 qpos 직접 쓰기 →
   `data.ctrl`에 목표각 쓰기로 교체, 루프에서 `mj_forward` → `mj_step`(실시간 페이싱)으로 교체
3. **주의**: freejoint 물체가 추가되면 qpos 레이아웃이 바뀜(물체 7 DOF가 qpos에 추가) —
   조인트 이름 기반 접근(`data.joint(...)`, `data.actuator(...)`)으로 인덱스 하드코딩 회피
4. **주의**: actuator 추종은 kinematic 스냅과 달리 지연·출렁임이 있음 — gain 상수 튜닝 필요

## Ontology (Key Entities)
| Entity         | Type          | Fields                                   | Relationships                                  |
|----------------|---------------|------------------------------------------|-------------------------------------------------|
| HandInput      | core domain   | wrist(x,y,z), curls[3] (MediaPipe/UDP)   | → IK/ctrl 목표 → JointActuator                 |
| Gripper        | core domain   | finger1..3 (proximal+distal joints)      | JointActuator가 구동, TargetObject와 접촉      |
| TargetObject   | core domain   | 공/큐브, size, mass, friction, freejoint | SupportSurface 위에 안착, Gripper에 파지됨     |
| SupportSurface | supporting    | 받침대/테이블, 고정, z≈0.5               | TargetObject를 지지                            |
| JointActuator  | core domain   | position servo, gain, ctrl target        | IK 결과/curl → 관절 토크                       |
| PhysicsSim     | external tool | mj_step, contacts, gravity               | 모든 entity의 접촉·동역학 적분                 |

## Ontology Convergence
| Round | Entity Count | New | Changed | Stable | Stability Ratio |
|-------|-------------|-----|---------|--------|----------------|
| 1     | 5           | 5   | -       | -      | N/A            |
| 2     | 6           | 1   | 0       | 5      | 83%            |
| 3     | 6           | 0   | 0       | 6      | 100%           |
| 4     | 6           | 0   | 0       | 6      | 100%           |
| 5     | 6           | 0   | 0       | 6      | 100%           |

## Interview Transcript
<details>
<summary>Full Q&A (5 rounds + Round 0)</summary>

### Round 0 (토폴로지)
**Q:** 3개 컴포넌트(물리 씬 전환 / 파지 대상 물체 / 파지 실행·검증)가 맞나요?
**A:** 맞음, 3개 모두

### Round 1 (물리-씬-전환 / Goal)
**Q:** 파지 동작을 무엇이 구동하길 원하나요? (손 추적 텔레오퍼레이션 vs 스크립트 자동 파지)
**A:** 손 추적 텔레오퍼레이션
**Ambiguity:** 62% (Goal 0.40, Constraints 0.20, Criteria 0.20, Context 0.90)

### Round 2 (파지-대상-물체 / Goal)
**Q:** 씬에 어떤 물체를 놓고 싶나요?
**A:** 작은 공/큐브 1개 (가장 단순)
**Ambiguity:** 45% (Goal 0.70, Constraints 0.30, Criteria 0.40, Context 0.90)

### Round 3 (물리-씬-전환 / Constraints)
**Q:** 지금 잘 동작하는 kinematic 데모를 어떻게 처리할까요?
**A:** 기존 파일 직접 전환 (mujoco_receiver.py + arm.xml, 새 파일 없음)
**Ambiguity:** 35% (Goal 0.75, Constraints 0.60, Criteria 0.40, Context 0.90)

### Round 4 (파지-실행·검증 / Criteria — Contrarian)
**Q:** 성공 '판정'이 정말 필요한가요?
**A:** 눈으로 확인만
**Ambiguity:** 22% (Goal 0.85, Constraints 0.70, Criteria 0.70, Context 0.90)

### Round 5 (파지-대상-물체 / Constraints)
**Q:** 물체를 어디에 놓을까요? (팔 작업공간 z≈0.4~0.8 vs 바닥 z=0)
**A:** 받침대/테이블 위 (작업공간 높이 z≈0.5)
**Ambiguity:** 15% (Goal 0.90, Constraints 0.80, Criteria 0.80, Context 0.90) — PASSED

</details>
