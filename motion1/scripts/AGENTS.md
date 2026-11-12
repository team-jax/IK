<!-- Parent: ../AGENTS.md -->
<!-- Generated: 2026-10-13 | Updated: 2026-10-13 -->

# scripts/ — 실행·측정·녹화

## Purpose
동작 1 환경을 확인하고, 파지 상수를 측정하고, 성공률을 재고, 영상을 만든다. 모든 스크립트는 첫 줄에서 `_paths`를 import해 `src/`를 경로에 넣으므로 어느 폴더에서 실행해도 된다. 기본 출력 경로는 `motion1/` 기준(`_paths.ROOT`).

## Key Files
| File | Description |
|------|-------------|
| `_paths.py` | `src/`를 `sys.path`에 넣고 `ROOT`(motion1/) 제공. 새 스크립트는 맨 위에서 `import _paths` |
| `smoke_test.py` | gymnasium env checker + 배달·쌓기 한 번씩 실행(에피소드 길이는 env의 `max_steps`). 놓는 동안 큐브가 2 mm 넘게 오르면 실패(놓는 동작의 회귀 검사). 실물 배치(`JAX_SETUP=real`)에서는 고정 배치 2단이 아직 실패한다(`STATUS.md` 2026-11-10 (8)) |
| `build_reachmap.py` | 도달 가능 격자를 풀어 `reachmap.npz` 재생성(`JAX_SETUP=real`이면 `reachmap_real.npz`). 파지 상수·홈 자세·작업 영역을 바꾸면 필수. `--yaws`(실물 배치만: 이 맵을 읽는 것은 `realcoords.py`뿐)는 큐브 회전각 × 손 추가 회전(0, ±90°)별 맵을 `*_yaw.npz`로 따로 만든다(회전 0 맵은 건드리지 않음). 이 맵은 큐브 면에 맞는 파지만 담는다: 실물 배치의 반 바퀴 돌린 파지와 어긋난 파지(`env._grasp_turns`)는 env가 직접 풀어 판정한다 |
| `find_home.py` | 홈 자세(`HOME_ARM_QPOS`)를 고른 기준을 재현: 홈 지점에 파지 중심을 두는 다중 시작 IK 후보마다 `build_reachmap.py`와 같은 도달 띠를 풀어 가장 넓은 것을 고른다. env.py는 고치지 않고 결과만 출력 |
| `tune_grasp.py` | 닫힘 정도별 패드 간격과 판 기울기 측정. 박스 충돌 geom은 모서리, 메시 geom은 정점으로 잰다. 기대값: g=0에서 89.7 mm, `CUBE_GRIP`(0.86)에서 50 mm 큐브 폭 |
| `layouts.py` | 큐브·목표 배치를 JSON으로 고정, `replay()`로 재생. 파일에 배치 종류(`setup`)를 적고, 다른 배치(`JAX_SETUP`)에서 재생하면 거부한다(그 전 파일은 sim). 재생할 때도 파지는 env(`_servable`)가 고른다 |
| `classify_failures.py` | 실패를 집지 못함/짧게 놓임/방출에서 잃음/제자리에 있지만 미완료(시간 안에 손을 떼고 빠지지 못함)로 분류하고 성공률·오차 통계 출력 (`--stack`). 고정 배치로 대체된 시드는 배달·2단 모두 `[fixed-layout fallback]`, 합계는 마지막 줄 끝. `--layouts`는 파일의 시드를 모두 돈다(`--first-seed`/`--seeds`로 좁힘). "집음"은 큐브가 환경의 기준(`LIFT_REQUIRED`, 30 mm) 이상 올라간 것으로 센다. `--yaw 45`는 큐브 회전을 ±45°에서 뽑는다(`env.cube_yaw_max`) |
| `run_real.py` | 실물 좌표의 큐브(x, y, 회전각)와 놓을 곳으로 시뮬 배달을 돌려 관절 궤적 JSON 저장(`JAX_SETUP=real` 필수. 재시도 없는 배달만 궤적을 쓴다: 종료 0 저장 / 1 실패 / 4 재시도 뒤 성공 / 3 닿지 않음 / 2 인자 오류·`JAX_SETUP`이 real이 아님, `--keep`은 `playable: false`로 저장). team-jax/SimToReal의 `play_trajectory`가 재생 |
| `record_delivery.py` | 1920x1080 mp4 녹화 (`--stack` 가능) |
| `render_frame.py` | 장면 PNG 한 장 (`--out`, 기본 `artifacts/scene.png`) |
| `watch_grasp.py` | 대화형 뷰어 재생. macOS는 `mjpython` 필수. `--layouts`를 쓰려면 먼저 `layouts.py`로 JSON을 만든다 |

## For AI Agents
- 확인: `venv/bin/python motion1/scripts/smoke_test.py`.
- `artifacts/layouts_*.json`은 같은 장면으로 비교하려고 만드는 로컬 파일이다(Git에 없음). 비교하는 동안 바뀌면 안 되니 덮어쓰지 말고 새 이름으로 만든다. 성적표의 숫자는 이 파일이 아니라 시드로 얻은 것이다.
- G1 시절의 `teleop`, `audit_contacts`, `trace_carry`는 옮기지 않았다(`g1-final` 태그).

## Dependencies
- Internal: `humanoid_grasp_rl` 패키지
- External: `mujoco`, `numpy`, `imageio`, `pillow`

<!-- MANUAL: -->
