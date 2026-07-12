# Autopilot Implementation Plan: 파지(그리퍼) 시뮬레이션 (rev 2 — critic 반영)

Spec: `.omc/specs/deep-interview-grasp-sim.md` (deep-interview PASSED, ambiguity 15%)
Critic verdict rev1: REJECT → 본 rev2에서 4개 필수 수정 반영.

## Scope
- **수정**: `arm/arm.xml`, `mujoco_receiver.py` (2개 파일만)
- **무수정**: `vision_sender.py`, `arm/ik.py`, `arm/arm.urdf`, `run_receiver.sh`
- **신규 파일 없음** (repo 내). QA용 headless 테스트 스크립트는 scratchpad에만 생성.

## 작업공간 기하 (critic Finding #1 해결 — 근거 방정식)

기하 상수: end_effector/gripper_base는 wrist_roll_link에서 −0.12, 손가락은 gripper_base
아래로 최대 ~0.06 (proximal 0.035 + distal 0.025, 펼침 기준).

- 명령 가능한 IK 타깃(= gripper_base) z 범위: `[BASE_Z, BASE_Z + HEIGHT_SCALE]`
- 파지하려면 gripper_base가 큐브 중심까지 내려가야 함:
  **`BASE_Z ≤ cube_center_z − margin`**, margin ≈ 0.05
- 확정값: `BASE_Z = 0.40`, 받침대 상단 z = 0.45, 큐브 half 0.015 → 큐브 중심 z = 0.465
  - 검증: 최저 gripper_base 0.40 < 0.465 (margin 0.065 ✓),
    최저 fingertip 0.34 < 큐브 하단 0.45 ✓
  - 도달성: base(0,0,0.8) → 파지점(0.12, 0, 0.465) 거리 ≈ 0.356 m < 팔 길이 0.67 m ✓
- **전방(x) 정렬**: 기존 `x = z_m * DEPTH_SCALE`은 중심이 0이라 받침대(x=0.12)와 어긋남.
  수신부에 `FORWARD_BASE = 0.12` 상수 추가: `x = FORWARD_BASE + z_m * DEPTH_SCALE`
  → 손 중립 깊이에서 받침대 정면에 위치. (스펙상 매핑 상수는 튜닝 가능 항목)

## Task 1 — `arm/arm.xml`: 물리 씬 구성

1. **옵션 고정** (critic Finding #3): `<option timestep="0.002" integrator="implicitfast"
   gravity="0 0 -9.81"/>` — implicitfast로 servo 강성-timestep 결합 제거.
   질량 예산(기본 밀도 1000 kg/m³ 유도값): 상완 실린더 ≈ 0.59 kg, 전완 ≈ 0.31 kg,
   손가락 캡슐 ≈ 4-5 g → 팔 kp=100/kv=10, 손가락 kp=5/kv=0.5 조합의 근거.
2. **충돌 비트마스크** (critic Finding #4 — 손가락끼리 비충돌):
   - 손가락 캡슐 6개: `contype="2" conaffinity="1"`
   - 물체·받침대·바닥: `contype="1" conaffinity="3"`
   - 검증: 손가락↔손가락 (2&1)|(2&1)=0 비충돌 ✓, 손가락↔물체/받침대 2&3=2 충돌 ✓,
     물체↔받침대/바닥 1&3=1 충돌 ✓. 팔 링크는 default contype=0 conaffinity=0 유지.
3. **받침대**: 고정 box, `pos="0.12 0 0.225"` `size="0.04 0.04 0.225"` (상단 z=0.45).
4. **물체**: freejoint body, 큐브 half 0.015, `mass="0.05"`,
   `friction="1.0 0.02 0.001"` `condim="4"`. **스폰 z = 0.45 + 0.015 = 0.465 정확히**
   (첫 스텝 관통 방지). 손가락 캡슐에도 동일 friction/condim.
5. **액추에이터**: `{joint_name}_act` 이름 규칙.
   - 팔 7관절: `<position joint="..." kp="100" kv="10" ctrlrange="관절 range"/>`
   - 손가락 6관절: `<position joint="..." kp="5" kv="0.5" ctrlrange="관절 range"/>`
6. **XML 순서**: 팔 body가 물체 body보다 앞 → 팔 관절 qpos[0:13], 물체 qpos[13:20].
   `arm/ik.py`의 `apply_pose`(qpos[:7]) 단독 실행 시 여전히 유효 (critic 확인 완료).

## Task 2 — `mujoco_receiver.py`: kinematic → 물리 제어

1. **ctrl 기반 제어**: qpos 직접 쓰기 제거.
   - 팔: IK 결과(last-good 폴백 유지) → `data.actuator("{joint}_act").ctrl`
   - 손가락: proximal `curl*MAX_CURL`, distal `curl*MAX_CURL*DISTAL_SCALE` → ctrl
   - `arm.ik` import에서 `apply_pose` 제거 (`build_chain`, `solve_ik`, `SEED_POSE` 유지)
2. **매핑 상수**: `BASE_Z = 0.40`, `FORWARD_BASE = 0.12` 추가,
   `x = FORWARD_BASE + z_m*DEPTH_SCALE`. 나머지 상수 유지.
3. **mj_step 루프** (critic minor #1 — 500Hz 루프 회피): 프레임당 substep 방식.
   ```
   while viewer.is_running():
       frame_start = monotonic()
       (UDP 논블로킹 수신 → ctrl 목표 갱신, 프레임당 1회)
       with viewer.lock():
           for _ in range(5): mj_step(model, data)   # 5 × 0.002s = 10ms/frame
       viewer.sync()
       sleep(max(0, 0.01 - 경과))
   ```
4. **초기화**: qpos[:7] = SEED 팔 자세 + 팔 ctrl 동일 목표, **손가락 ctrl = 0 (펼침)**,
   `mj_forward` 1회 후 루프 진입 — 0-pose 특이점 서보 튐 방지.
5. **유지**: UDP 5005 논블로킹, 캘리브레이션('C'), 카메라 토글('T'), last-good IK 폴백,
   4/6필드 패킷 파싱, 카메라 프레이밍 상수(CAM_LOOKAT z는 새 작업공간에 맞게 조정 가능).

## Task 3 — QA (headless, scratchpad 스크립트)

`venv/bin/python`으로 viewer 없이 검증:
- **envelope** (critic Finding #2 해결 — IK 오차 대신 매핑 포락선 검증):
  최저 명령값(y_norm=1, 받침대 정면 x/y)에 대해 IK→FK로 실제 fingertip z 계산,
  **`min_fingertip_z < 큐브 하단(0.45)`** 및 gripper_base가 큐브 중심(0.465) 이하 도달 단언
- **settle**: 2초 스텝 후 물체가 받침대 위 유지 (z ≈ 0.465 ± tol), qacc 유계·NaN 없음
  (critic Finding #3의 안정성 단언 포함)
- **grasp**: 스크립트 텔레오퍼레이션 — 접근→하강(gripper_base ≈ 큐브 중심)→손가락 닫기→
  상승 ctrl 시퀀스 후 **물체 z 상승 ≥ 0.08 m** (접촉이 아니라 '들림'을 측정 — critic 지적)
- **release**: 손가락 열면 물체 낙하 (z 하강 확인)
- **empty-close**: 물체 없이(치워두고) 손가락 완전 닫기 → 발산/NaN 없음 (형제 손가락
  비충돌 확인)
- **parse_packet**: 4/6필드 파싱 회귀
- 실패 시 kp/kv/friction/받침대·BASE_Z 튜닝 후 재실행 (최대 5 사이클)

뷰어 표시·실제 손 추적 연동은 사용자가 눈으로 확인 (스펙: 시각 확인).

## Risks (잔여)
- MediaPipe z_m 실측 범위가 FORWARD_BASE 중심 정렬과 어긋날 수 있음 → 상수 튜닝으로 흡수,
  실기 확인은 사용자 시각 검증 단계
- kp=5 손가락의 파지 수직력이 50g 큐브에 경계선일 수 있음 → grasp QA가 '들림'을 직접
  측정하므로 미달 시 kp/마찰 튜닝으로 검출·해결
