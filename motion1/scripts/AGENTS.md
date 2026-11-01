<!-- Parent: ../AGENTS.md -->
<!-- Generated: 2026-10-13 | Updated: 2026-10-13 -->

# scripts/ — 실행·측정·녹화

## Purpose
동작 1 환경을 확인하고, 파지 상수를 측정하고, 성공률을 재고, 영상을 만든다. 모든 스크립트는 첫 줄에서 `_paths`를 import해 `src/`를 경로에 넣으므로 어느 폴더에서 실행해도 된다. 기본 출력 경로는 `motion1/` 기준(`_paths.ROOT`).

## Key Files
| File | Description |
|------|-------------|
| `_paths.py` | `src/`를 `sys.path`에 넣고 `ROOT`(motion1/) 제공. 새 스크립트는 맨 위에서 `import _paths` |
| `smoke_test.py` | gymnasium env checker + 배달·쌓기 한 번씩 실행. 놓는 동안 큐브가 2 mm 넘게 오르면 실패(놓는 동작의 회귀 검사) |
| `build_reachmap.py` | 도달 가능 격자를 풀어 `reachmap.npz` 재생성. 파지 상수·홈 자세·작업 영역을 바꾸면 필수 |
| `tune_grasp.py` | 닫힘 정도별 패드 간격과 판 기울기 측정. 박스 충돌 geom은 모서리, 메시 geom은 정점으로 잰다. 기대값: g=0에서 89.7 mm, `CUBE_GRIP`(0.86)에서 50 mm 큐브 폭 |
| `layouts.py` | 큐브·목표 배치를 JSON으로 고정, `replay()`로 재생 |
| `classify_failures.py` | 실패를 집지 못함/짧게 놓임/방출에서 잃음으로 분류하고 성공률·오차 통계 출력 (`--stack`). "집음"은 큐브가 환경의 기준(`LIFT_REQUIRED`, 30 mm) 이상 올라간 것으로 센다 |
| `record_delivery.py` | 1920x1080 mp4 녹화 (`--stack` 가능) |
| `render_frame.py` | 장면 PNG 한 장 |
| `watch_grasp.py` | 대화형 뷰어 재생. macOS는 `mjpython` 필수. `--layouts`를 쓰려면 먼저 `layouts.py`로 JSON을 만든다 |

## For AI Agents
- 확인: `venv/bin/python motion1/scripts/smoke_test.py`.
- `artifacts/layouts_*.json`은 같은 장면으로 비교하려고 만드는 로컬 파일이다(Git에 없음). 비교하는 동안 바뀌면 안 되니 덮어쓰지 말고 새 이름으로 만든다. 성적표의 숫자는 이 파일이 아니라 시드로 얻은 것이다.
- G1 시절의 `teleop`, `audit_contacts`, `trace_carry`는 옮기지 않았다(`g1-final` 태그).

## Dependencies
- Internal: `humanoid_grasp_rl` 패키지
- External: `mujoco`, `numpy`, `imageio`, `pillow`

<!-- MANUAL: -->
