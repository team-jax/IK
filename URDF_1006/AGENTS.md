<!-- Parent: ../AGENTS.md -->
<!-- Generated: 2026-10-13 | Updated: 2026-10-13 -->

# URDF_1006/ — 팀의 최종 URDF

## Purpose
CAD에서 export한 로봇 전체(`JAXV2_Assembly`, 링크 16, 관절 15)의 설명 패키지. 동작 1·2 어느 쪽에도 속하지 않는 **읽기 전용 원본**이다. `motion2/arm/make_arm_urdf.py`가 이 폴더를 읽어 한 팔 모델을 만든다.

## Key Files
| File | Description |
|------|-------------|
| `JAXV2_Assembly_description/urdf/JAXV2_Assembly.urdf` | 생성기가 파싱하는 URDF (xacro 버전과 `assemblies/`도 있음) |
| `JAXV2_Assembly_description/meshes/` | 시각용 DAE와 충돌용 STL |
| `JAXV2_Assembly_description/README.md` | 링크·관절 표, 운동학 트리 |
| `JAXV2_Assembly_description/config/`, `docs/transforms.md` | 프레임 오버라이드, ROS2 컨트롤러 설정, 변환 문서 |
| `debug/export_log.md`, `debug/extraction_report.md` | CAD export 기록 |

## For AI Agents
- 이 폴더를 수정하지 않는다. URDF가 바뀌면(새 CAD export) `venv/bin/python motion2/arm/make_arm_urdf.py`를 다시 실행하고, 생성기 상수(파지 중심, 손가락 배치, 큐브/기둥 크기)가 새 기하와 맞는지 확인한 뒤 `motion2/check_grasp.py`와 `motion1/scripts/build_reachmap.py`를 다시 돌린다.
- `.omc/`는 도구 상태 폴더라 무시한다.

<!-- MANUAL: -->
