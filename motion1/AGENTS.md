<!-- Parent: ../AGENTS.md -->
<!-- Generated: 2026-10-13 | Updated: 2026-10-13 -->

# motion1/ — 스크립트 파지·배치 (동작 1)

## Purpose
최종 URDF의 한 팔 + 3손가락 손이 책상 위 50 mm 큐브를 집어 무작위 목표 지점에 놓거나(배달) 다른 큐브 위에 쌓는(2단) MuJoCo 환경과 상태기계 컨트롤러. 강화학습은 접었다. 원래 Unitree G1 + Dex3-1이었고 2026-10-12에 교체했다(G1 버전은 원래 저장소 `gyumingim/Humanoid_Grasp_RL`의 `g1-final` 태그).

2026-11-01부터 team-jax/IK가 이 폴더를 직접 추적하므로 변경은 저장소 루트에서 커밋한다. 안에 남은 `.git`은 원래 저장소(`gyumingim/Humanoid_Grasp_RL`)의 기록이다.

## Key Files
| File | Description |
|------|-------------|
| `README.md` | 현재 성적, 한계, 구조, 파지에서 알아낸 것, 측정·녹화 명령 (작업의 기준) |
| `STATUS.md` | 진행 기록. 맨 위 세 항목(2026-10-12 이후)이 지금 로봇, 그 아래는 G1 시절 기록 |
| `HARNESS.md` | 이 폴더의 AI 작업 규칙(큰 범위 변경은 사전 승인, 최소 복잡도) |
| `CODE_REVIEW.md` | G1 코드 검토 기록. 현재 코드에는 적용되지 않음(폐기) |
| `pyproject.toml` | 패키지 `humanoid-grasp-rl`과 의존성 하한(`mujoco>=3.3` 등). 실제 실행은 루트 `venv/`(mujoco 3.10.0, gymnasium 1.3.0)로 하고 그 조합에서 검증했다 |
| `.gitignore` | `.venv/`, `__pycache__/`, `*.egg-info/`, `models/`, `artifacts/`, `logs/` 제외 |
| `MUJOCO_LOG.TXT`, `README.html`, `README_files/` | 생성 부산물 |

## Subdirectories
| Directory | Purpose |
|-----------|---------|
| `src/humanoid_grasp_rl/` | 환경·IK·상태기계 패키지 (see `src/humanoid_grasp_rl/AGENTS.md`) |
| `scripts/` | 실행·측정·녹화 스크립트 (see `scripts/AGENTS.md`) |
| `artifacts/` | 생성물(레이아웃 JSON, 영상, 이미지). git 제외라 처음엔 비어 있다 |
| `.claude/agents/` | 에이전트 정의. `.claude/agents/fable-max.md`는 실험 결과를 외부에서 검토하는 리뷰어(RL이 멈춘 뒤로는 스크립트 컨트롤러의 측정 결과를 검토) |

## For AI Agents

### Working In This Directory
- 실행은 저장소 루트에서 `venv/bin/python motion1/scripts/<script>.py` (뷰어는 `venv/bin/mjpython`). 스크립트가 `src/`를 스스로 찾는다.
- 로봇 모델은 `../motion2/arm/make_arm_urdf.py`의 `robot_parts()`로 만든다(`assets.py`). 모델 상수는 그쪽 파일에서만 바꾼다.
- 파지 상수(`FINGER_*`, `CUBE_GRIP`, `GRASP_DZ`, `HOME_ARM_QPOS`, 작업 영역)를 바꾸면 도달 맵을 다시 만든다: `scripts/build_reachmap.py`. 시그니처가 다르면 `env.py`가 경고한다.
- 배치는 `env.py`의 `SETUPS` 두 가지다: 기본 `sim`(어깨 0.85 m, 책상 0.25 m, 아래 성적의 기준)과 `JAX_SETUP=real`(실물: 어깨 회전축 1.2075 m, 책상 0.735 m, 손 40° 기울임, 도달 맵 `reachmap_real.npz`). real의 상수를 바꾸면 `JAX_SETUP=real`로 도달 맵을 다시 만든다.
- 증상을 가리는 상수 조정보다 원인 구조를 고친다(`HARNESS.md`).

### Testing Requirements
- `smoke_test.py`가 통과해야 한다. 컨트롤러 변경은 `classify_failures.py`로 배달 시드 0~99, 쌓기 `--stack` 시드 0~49를 다시 재고, 고정 레이아웃(`layouts.py`)으로 비교한다.
- 현재 기준: 배달 68/100, 2단 39/50 (2026-11-04, 패드–큐브 마찰 0.4 실측, 운반 1 mm/step). 마찰 2.0이던 때는 100/100, 46/50 (2026-10-12, 2026-10-16에 다시 재서 같음). 실물 높이 배치(`JAX_SETUP=real`)는 지금 손으로 배달 2/100, 2단 0/50(큐브가 운반 중 빠짐, 손 교체 대기, `STATUS.md`).
- 저장소 루트의 `./run_tests.sh`가 이 폴더에서 `smoke_test.py`, `tune_grasp.py`(g=0에서 패드 간격 89.7 mm), `layouts.py`, 고정 레이아웃 재생(`classify_failures.py --layouts`, 3개 중 2개 이상 배달), `render_frame.py`, 실물 높이 배치 확인(`m1_real_setup`: 어깨·책상 높이와 간격, 도달 맵이 최신인지)을 돌린다. `--full`이면 배달 시드 0~99(68개 이상), 쌓기 시드 0~49(39개 이상), `build_reachmap.py`가 저장된 맵과 같은 파일을 만드는지, 영상 2개(1920x1080)까지 확인한다.

### Common Patterns
- 성공은 에피소드 끝까지 유지되는 상태로 센다(래치가 아니라 최종 판정).
- 단계 마감시간 초과는 실패로 기록되고 복구 분기로 이어진다.

## Dependencies
- Internal: `../motion2/arm/` (모델 생성기와 메시), `../URDF_1006/`
- External: `mujoco`, `gymnasium`, `numpy`, `scipy`, `imageio`(+ffmpeg), `pillow`

<!-- MANUAL: -->
