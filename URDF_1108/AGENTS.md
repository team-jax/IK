<!-- Parent: ../AGENTS.md -->
<!-- Generated: 2026-11-09 | Updated: 2026-11-09 -->

# URDF_1108/ — 팀의 최종 URDF (V3)

## Purpose
Fusion 360 디자인 `JAXV3-Assembly v5`를 2026-11-08에 export한 로봇 전체(`JAXV3_Assembly`, 링크 17, 관절 16)의 설명 패키지와 export 기록. 동작 1·2 어느 쪽에도 속하지 않는 **읽기 전용 원본**이다. 2026-11-09부터 `motion2/arm/make_arm_urdf.py`가 이 폴더에서 관절 위치·한계와 외형 메시를 읽는다. 질량·관성과 손가락 충돌 STL은 아직 `../URDF_1006`(V2)에서 읽는다(이유는 `JAXV3_Assembly_description/AGENTS.md`).

## Key Files
| File | Description |
|------|-------------|
| `debug/export_log.md`, `debug/extraction_report.md` | 익스포터(fusion2URDF v3.1.1)의 실행 로그와 추출 보고서 |
| `debug/validation.md` | URDF 검증 보고서(PASS, 오류 0·경고 0). 링크마다 충돌 형상이 `[primitive]`(익스포터가 만든 원기둥·상자)라고 적혀 있다 |
| `debug/frame_model.json`, `debug/fusion_transforms.json`, `debug/snapshot.json` | 익스포터 설정·프레임 캐시, Fusion 변환, Fusion 디자인 스냅샷. 읽는 코드는 없다 |

## Subdirectories
| Directory | Purpose |
|-----------|---------|
| `JAXV3_Assembly_description/` | ROS 2 설명 패키지: 생성기가 읽는 `urdf/JAXV3_Assembly.urdf`와 `meshes/JAXV3_Assembly/`, V2와 다른 점, URDF 관절 → 프로젝트 이름 대응표 (see `JAXV3_Assembly_description/AGENTS.md`) |
| `debug/` | 익스포터의 기록과 중간 산출물 (위 표) |

## For AI Agents
- 이 폴더를 수정하지 않는다. 생성기가 이 export에 맞추는 것(링크 이름의 `_1` 꼬리, 축이 뒤집힌 두 관절, V2 질량·손가락 충돌)은 모두 `motion2/arm/make_arm_urdf.py`의 `load()`에서 한다. 새 export로 바꿀 때의 확인 순서는 `JAXV3_Assembly_description/AGENTS.md`에 있다.
- `.omc/`는 도구 상태 폴더라 무시한다.

<!-- MANUAL: -->
