# Humanoid Grasp (동작 1)

팀의 최종 URDF(`../URDF_1006/JAXV2_Assembly`)의 한 팔 + 3손가락 손으로, 책상 위 50 mm 정육면체를 집어 무작위 목표 지점에 놓거나 다른 큐브 위에 쌓는 MuJoCo 환경과 스크립트 컨트롤러다. 강화학습은 접었다. 컨트롤러는 타이머가 아니라 장면의 조건으로 전이하는 상태기계다.

> 이 폴더는 원래 Unitree G1 + Dex3-1 버전이었다. 2026-10-12에 JAX 로봇으로 교체했고, G1 버전은 이 저장소의 `g1-final` 태그에 그대로 있다 (`git checkout g1-final`).

## 현재 성적 (완전 무작위 배치)

| 과제 (2026-10-16, 시드 0부터 연속) | 성공 | 최종 오차 중앙 / 90% / 최대 |
| --- | --- | --- |
| 배달 (큐브 → 책상 위 지점), 시드 0~99 | 100/100 (재시도가 있었던 에피소드 9) | 2.5 / 7.8 / 16.8 mm |
| 2단 쌓기, 시드 0~49 | 46/50 (재시도 3) | 4.6 / 6.6 / 7.9 mm (성공한 것만) |

오차의 "90%"는 에피소드의 90%가 그 값 이하라는 뜻이다(nearest-rank). 2단 쌓기 50개 중 24개는 샘플러가 검증을 통과하는 배치를 못 찾아 고정 배치로 되돌아갔다(`layout_fallbacks`로 센다; 도달 맵 가장자리를 깎아 띠가 좁아졌다). 실패 4개: 위 큐브를 내리다 밑 큐브에 닿기 전에 손이 바닥 한계에 닿은 것 3개, 놓은 뒤 4.0 mm 움직여 성공 조건(정지)을 못 채운 것 1개(시드 8, 15, 38, 47). 2026-10-16에 놓는 동작을 고친 뒤 다시 쟀고, 고치기 전의 값을 같은 방법으로 다시 재면 배달 3.7 / 7.4 / 19.4 mm(재시도 8), 쌓기 3.6 / 5.2 / 7.7 mm(재시도 3)였다. 배달 오차는 줄고 쌓기 오차는 약 1 mm 늘었다. 쌓기에서는 큐브가 내려오는 동안 이미 5 mm쯤 밀린 채 닿는데, 옛 동작은 큐브를 떠올렸다 떨어뜨리며 그 오차를 우연히 1~2 mm 줄여 주고 있었다. 검증하지 않은 것: 다른 시드 범위, G1 시절 같은 200개 이상 규모.

배치는 사용 전에 컨트롤러가 명령할 자세(집기·접근·들어올리기·놓기·운반 높이)를 IK로 검증한다. 성공은 "한 번 됐음"이 아니라 **에피소드 끝까지 유지**(큐브가 면에 평평, 실제 지지, 손 접촉 없음, 손이 목표에서 `HAND_CLEAR` 이상 떨어짐)로 센다. 큐브는 50 mm, 0.05 kg이다.

### 한계 (측정으로 확인한 것)

- **탑은 2단까지.** 세 번째 큐브를 집으려면 2단 탑 위로 운반해야 하는데, 그 높이(파지 중심 z≈0.39)에서는 팔이 손가락을 아래로 유지하지 못한다. 3단 배치를 찾아 봤지만 IK 검증을 통과한 것이 없었다.
- **작업 영역이 한쪽으로 치우쳐 있다.** palm-down은 어깨 롤을 한계 근처로 쓰고, 팔꿈치를 어느 쪽으로 꺾느냐에 따라 풀이가 둘로 갈린다. 홈 자세가 한쪽을 고르므로(`env.py`의 `HOME_ARM_QPOS`) 도달 가능한 띠는 x -0.04~0.24, y -0.28~+0.03이다.
- **30 mm 큐브는 책상에서 못 집는다.** 손가락 판이 파지 중심 아래 17 mm까지만 내려와 책상 위에서는 큐브 윗부분만 잡는다 (30 mm는 미끄러지거나 튕김, 40 mm는 5번 중 4번, 50 mm는 5번 중 5번).
- **내려놓을 때 큐브가 손가락 사이에서 한 번 미끄러진다.** 운반하는 동안 큐브가 판 사이에서 수직축으로 4.5~9° 비틀리고 닿는 손가락이 3개에서 1~2개로 줄어든다. 내려가는 첫 움직임에서 그 비틀림이 한꺼번에 풀려(317°/s) 3 스텝 동안 접촉이 끊기고, 큐브가 자유낙하한 뒤 세 손가락으로 다시 잡힌다. 배달 시드 0~63에서 8 mm 넘게 미끄러진 것이 34%(평균 7.2 mm, 90% 17.1 mm, 최대 26.5 mm), 0.3 m/s 넘는 속도로 책상에 닿은 것이 47%다. 손이 제자리에서 버틸 때는 생기지 않고 내려가는 속도와도 무관하다(5 mm/s로 내려도 같다). 성공률에는 영향이 없지만 영상에서 큐브가 툭 떨어지듯 놓인다. 고쳐 보려 한 것은 모두 효과가 없었다(각각 시드 16~64개): 내리는 속도 0.1~3 mm/step와 서서히 가속, 내릴 때의 조임 세기(풀수록 악화), 널스페이스 힘 0~0.10, 솔버 `noslip`(최댓값은 줄지만 재시도 0 → 3), 더 깊게 잡기 `GRASP_DZ` 0 / -0.002 / -0.004(튀는 비율 34% / 33% / 34%), 큐브 토션 마찰 0.02~0.3(34% → 45%). 내리기 전에 조임을 살짝 풀어 큐브를 다시 앉히면 튀는 비율이 35% → 19~23%로 줄지만 배달 성공이 48/48 → 42~45/48로 떨어졌다. 고치려면 손가락 판의 형상을 바꿔야 할 것으로 보이며, 이는 동작 2와 공유하는 모델 생성기(`../motion2/arm/make_arm_urdf.py`)를 건드린다(검증하지 않음).

## 설치 및 확인

저장소 루트의 `venv/`를 그대로 쓴다. 스크립트가 `src/`를 스스로 찾으므로 설치나 `PYTHONPATH`가 필요 없고, 어느 폴더에서 실행해도 된다 (아래 명령은 저장소 루트 `IK/` 기준).

```bash
venv/bin/python motion1/scripts/smoke_test.py
```

별도 환경을 원하면 `motion1/`에서 `python -m venv .venv && .venv/bin/python -m pip install -e .` 후 `.venv/bin/python scripts/smoke_test.py`. `motion1/.venv`는 기본으로 만들어져 있지 않다.

로봇 모델은 `../motion2/arm/make_arm_urdf.py`가 최종 URDF에서 생성한다(동작 2와 같은 로봇). `src/humanoid_grasp_rl/assets.py`가 그 생성기를 불러와 메시(`../motion2/arm/meshes/`)가 없으면 만든다. 위치를 바꿀 때는 `JAX_ARM_DIR`로 지정한다.

## 구조

| 파일 | 역할 |
| --- | --- |
| `src/humanoid_grasp_rl/env.py` | MuJoCo 장면(책상·큐브·목표 원·JAX 팔), 팔 IK, 손 개폐, 레이아웃 샘플링(도달 맵 + 레이아웃별 IK 검증 `_servable`), 과제 판정(`_update_latches`·`_resting_on_support`·`_success`). 파지 상수(`FINGER_OPEN/CLOSED`, `CUBE_GRIP`, `GRASP_DZ`)와 그 근거가 여기 있다. 보상은 성공 래치 스텝에 1.0, 그 외 0. |
| `src/humanoid_grasp_rl/demonstration.py` | `scripted_palm_place` — reach → down → grip → up → cross → lower → settle → let go → back 상태기계. 단계 마감시간은 실패로 기록되고(`env._palm_failures`) 복구(재조준·재파지·park)로 이어진다. 탑에서는 큐브를 놓고 손이 충분히 떨어지면 다음 큐브로 넘어간다. |
| `src/humanoid_grasp_rl/single_env.py` | 큐브 하나를 목표 지점에 놓는 배달 환경 (`JaxDeliveryEnv`) |
| `src/humanoid_grasp_rl/stack_env.py` | 2단 탑 환경 (`JaxStackEnv`). 밑 큐브 위에 위 큐브를 얹는다. 큐브 간격은 열린 손의 발자국(x ±58 mm, y ±37 mm)으로 잰다. 배치는 도달 셀에서 백트래킹으로 뽑고 못 찾으면 `layout_fallbacks`에 센다. |
| `src/humanoid_grasp_rl/ik.py` | 파지 중심 site에 대한 damped least-squares IK + 홈 자세로의 널스페이스 |
| `src/humanoid_grasp_rl/assets.py` | `motion2/arm/make_arm_urdf.py`에서 로봇 MJCF 조각을 가져온다 |
| `src/humanoid_grasp_rl/reachmap.npz` | 집기·놓기 자세를 실제로 풀어 본 도달 가능 격자(가장자리 한 칸 제외) |

작업은 관절이 7개인 팔의 파지 중심(`end_effector` site, 손가락 사이)을 손가락이 아래를 향한 채로 움직이는 것이다. 행동은 4차원: 파지 중심의 위치 증분(x, y, z)과 닫힘 속도.

## 파지에서 알아낸 것

- **판의 기울기가 파지를 좌우한다.** 각 손가락 판은 근위+원위 각도의 합만큼 기운다. 끝이 큐브에서 멀어지는 쪽(Λ)으로 기울면 조임이 큐브를 아래로 미는 힘이 되어 들어올리는 순간 빠진다. 닫힘 경로(`FINGER_OPEN` → `FINGER_CLOSED`)는 큐브에 닿는 순간 판이 +2°로 살짝 안쪽으로 기울도록 정했다.
- **손가락 1·2를 같은 각도로 묶는다.** 같은 쪽에 나란히 선 두 손가락이 독립적이면 접촉 하중으로 서로 다르게 휘어, 패드가 서로 반대 방향의 마찰로 큐브를 굴린다(큐브가 패드 사이 축을 중심으로 서서히 돌며 미끄러지는 것을 관측). 묶으면 재시도가 19회 → 8회로 줄었다 (`COUPLE_FINGERS`).
- **들어올림·운반은 느리게.** 5 mm/step(0.25 m/s)은 50 g 큐브를 빼낸다. 2 mm/step에서 20/20, 3 mm/step에서 17/20이 배달됐다.
- **놓을 때는 조임을 먼저 풀고, 눌린 하중은 그다음에 푼다.** 착지하면 팔이 책상 쪽으로 4~5 mm 눌려 있다(패드가 큐브를 약 10 N으로 누르고 책상이 받친다). 명령 자세를 착지를 감지한 높이로 되돌리면 손이 7 mm 올라오면서 아직 닫힌 패드가 큐브를 4~9 mm 끌어올렸다가, 조임이 풀리면 떨어뜨렸다(모든 시드). `settle` 동안 명령 자세를 그대로 두고(`demonstration.py`) 하중은 아무것도 닿지 않는 `let go`에서 푼다. 큐브가 오르는 높이가 평균 6.4 → 0.7 mm(배달 48개), 6.5 → 0.1 mm(쌓기 40개)로 줄었다. `scripts/smoke_test.py`가 놓는 동안 큐브가 2 mm 넘게 오르면 실패한다.
- **복귀는 탑에서 멀어질 때까지 수평으로.** 홈으로 대각선으로 내려오면 열린 손의 판(중심에서 x ±58 mm)이 쌓은 탑을 쳤다.

## 측정 도구

```bash
# 도달 맵 재생성 - 파지 상수, 홈 자세, 작업 영역을 바꿨으면 반드시
venv/bin/python motion1/scripts/build_reachmap.py

# 패드 간격과 판 기울기를 닫힘 정도 g별로 측정 (FINGER_OPEN/CLOSED, CUBE_GRIP 정할 때)
venv/bin/python motion1/scripts/tune_grasp.py

# 레이아웃 고정 - 맵이 바뀌어도 같은 장면을 재생하도록 (watch_grasp --layouts 쓰기 전에 먼저 만들어야 함)
venv/bin/python motion1/scripts/layouts.py --scale 1.0 --out motion1/artifacts/layouts_10.json

# 실패 분류: 집지 못함 / 짧게 놓임 / 방출에서 잃음  (--stack 이면 2단 쌓기)
# 기본은 배율마다 시드 12개. 위 성적표의 숫자는 아래 두 명령으로 얻는다 (각각 수 분 걸림).
venv/bin/python motion1/scripts/classify_failures.py --scales 1.0 --seeds 100          # 배달, 시드 0~99
venv/bin/python motion1/scripts/classify_failures.py --stack --scales 1.0 --seeds 50   # 2단 쌓기, 시드 0~49
venv/bin/python motion1/scripts/classify_failures.py --scales 0.0 0.3 0.6 1.0          # 무작위 배율 비교 (시드 12개씩)
```

## 보기와 녹화

```bash
venv/bin/mjpython motion1/scripts/watch_grasp.py --layouts motion1/artifacts/layouts_10.json --seed 3   # 뷰어: macOS는 반드시 mjpython
venv/bin/python motion1/scripts/record_delivery.py --seed 3 --randomization-scale 1.0 --output motion1/artifacts/video/seed3.mp4
venv/bin/python motion1/scripts/record_delivery.py --stack --randomization-scale 1.0 --seed 3 --output motion1/artifacts/video/stack3.mp4
venv/bin/python motion1/scripts/render_frame.py        # artifacts/scene.png
```

`motion1/artifacts/`는 Git에서 제외된다(그래서 처음엔 비어 있다). 위 성적표의 숫자는 레이아웃 파일이 아니라 시드(`classify_failures.py --seeds`)로 얻은 것이다. 고정 레이아웃 파일(`artifacts/layouts_*.json`)은 맵이나 컨트롤러를 바꾼 뒤에도 같은 장면으로 비교하려고 만드는 로컬 파일이다. 비교하는 동안 장면이 바뀌면 안 되니 덮어쓰지 말고 새 이름으로 만든다. G1 시절의 `teleop.py`, `audit_contacts.py`, `trace_carry.py`는 이 로봇에 맞게 옮기지 않았다(`g1-final` 태그에 있다).
