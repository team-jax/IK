<!-- Parent: ../../AGENTS.md -->
<!-- Generated: 2026-10-13 | Updated: 2026-10-13 -->

# humanoid_grasp_rl/ — 환경·IK·상태기계 패키지

## Purpose
`from humanoid_grasp_rl import JaxDeliveryEnv, JaxGraspEnv, JaxStackEnv`로 쓰는 gymnasium 환경과 스크립트 컨트롤러.

## Key Files
| File | Description |
|------|-------------|
| `env.py` | `JaxGraspEnv`: 장면(책상·큐브·목표 원·JAX 팔) 조립, 팔 IK, 손 개폐, 레이아웃 샘플링(도달 맵 + `_servable` IK 검증), 성공 판정. 파지 상수와 근거가 상단에 있다. 행동 4차원(파지 중심 xyz 증분 + 닫힘 속도), 보상은 성공 래치 스텝에 1.0 |
| `single_env.py` | `JaxDeliveryEnv`: 큐브 하나를 목표 지점에 놓기 |
| `stack_env.py` | `JaxStackEnv`: 2단 탑. 큐브 간격은 열린 손 발자국(`CLEAR_DX/DY`)으로 잰다. 못 찾은 배치는 `layout_fallbacks`로 센다 |
| `demonstration.py` | `scripted_palm_place(env, step)`: reach → down → grip → up → cross → lower → settle → let go → back 상태기계와 복구 분기. `settle`은 명령 자세를 위로 끌지 않는다(끌면 조임이 남은 손이 큐브를 들어 올린다) |
| `ik.py` | 파지 중심 site에 대한 damped least-squares IK (`solve_site_pose`) |
| `assets.py` | `motion2/arm/make_arm_urdf.py`를 불러와 로봇 MJCF 조각(`RobotParts`)을 만든다. `JAX_ARM_DIR`로 위치 변경 |
| `reachmap.npz` | 실제로 풀어 본 도달 가능 격자 + 상수 시그니처. `scripts/build_reachmap.py`로만 만든다 |
| `__init__.py` | 세 환경 export |

## For AI Agents

### Working In This Directory
- `env.py`의 상수를 바꾸면 `reachmap.npz`를 다시 만들어야 한다(`map_signature()`가 어긋나면 경고).
- 세 번째 큐브(3단)는 운반 높이(파지 중심 z≈0.39)에서 팔이 손가락을 아래로 유지하지 못해 불가능하다. `NOMINAL_SPOTS`를 늘려도 풀리지 않는다.
- 손가락 1·2는 같은 각도로 묶여 있다(`COUPLE_FINGERS`). 판 기울기가 파지를 좌우하므로 `FINGER_OPEN/CLOSED`는 `scripts/tune_grasp.py`로 측정한 뒤 바꾼다.

### Testing Requirements
- `venv/bin/python motion1/scripts/smoke_test.py`, 이어서 `classify_failures.py`.

## Dependencies
- Internal: `../../../motion2/arm/make_arm_urdf.py`
- External: `mujoco`, `gymnasium`, `numpy`, `scipy`

<!-- MANUAL: -->
