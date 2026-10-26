<!-- Parent: ../AGENTS.md -->
<!-- Generated: 2026-10-13 | Updated: 2026-10-13 -->

# URDF_1006/ — 팀의 최종 URDF

## Purpose
Fusion 360 디자인 `JAXV2-Assembly v4`를 2026-10-06에 export한 로봇 전체(`JAXV2_Assembly`, 링크 16, 관절 15)의 설명 패키지와 export 기록. 동작 1·2 어느 쪽에도 속하지 않는 **읽기 전용 원본**이다. `motion2/arm/make_arm_urdf.py`가 이 폴더를 읽어 한 팔 모델을 만든다.

## Key Files
| File | Description |
|------|-------------|
| `debug/export_log.md`, `debug/extraction_report.md` | 익스포터(fusion2URDF v3.1.1)의 실행 로그와 추출 보고서 |
| `debug/validation.md` | URDF 검증 보고서(PASS, 오류 0·경고 0) |
| `debug/frame_model.json`, `debug/fusion_transforms.json`, `debug/snapshot.json` | 익스포터 설정·프레임 캐시, Fusion 변환(cm 단위), Fusion 디자인 스냅샷(mm 단위). 읽는 코드는 없다 |

## Subdirectories
| Directory | Purpose |
|-----------|---------|
| `JAXV2_Assembly_description/` | ROS 2 설명 패키지: 생성기가 읽는 `urdf/JAXV2_Assembly.urdf`와 `meshes/JAXV2_Assembly/`, URDF 관절 → 프로젝트 이름 대응표 (see `JAXV2_Assembly_description/AGENTS.md`) |
| `debug/` | 익스포터의 기록과 중간 산출물 (위 표) |

## For AI Agents
- 이 폴더를 수정하지 않는다. URDF가 바뀌면(새 CAD export) `venv/bin/python motion2/arm/make_arm_urdf.py`를 다시 실행하고, 생성기 상수(파지 중심, 손가락 배치, 큐브/기둥 크기)가 새 기하와 맞는지 확인한 뒤 `motion2/scripts/check_grasp.py`와 `motion1/scripts/build_reachmap.py`를 다시 돌린다. 절차 전체는 `JAXV2_Assembly_description/AGENTS.md`에 있다.
- `.omc/`는 도구 상태 폴더라 무시한다.

<!-- MANUAL: -->
