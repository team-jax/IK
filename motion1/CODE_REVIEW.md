> **폐기된 문서 (2026-10-12).** 아래는 Unitree G1 + Dex3-1 코드(`g1-final` 태그)에 대한 검토이며, 동작 1이 JAX 최종 URDF 기반 단일 팔 환경으로 교체되어 더는 현재 코드에 해당하지 않는다.
> 새 코드의 검증 내용은 `README.md`와 `STATUS.md`를 볼 것. 이력 보존을 위해 내용은 그대로 둔다.

# 현재 코드 수정 검토 — 2026-09-26

## 범위와 결론

- 목적: 고정 몸통 G1 + Dex3-1의 **스크립트 배달·탑 쌓기와 성공 영상**을 안정화한다. RL 재개는 범위 밖이다.
- 검토 대상: 현재 working tree의 환경, 상태기계, IK, 샘플러, 도달 맵 생성, 평가·배치 재생·녹화 스크립트.
- 기준 HEAD: `5e44363eb154bda1ff4bc303c3dcaa3bd0e7a07f`. 미커밋 변경이 있으므로 HEAD만으로 검토 대상을 재현할 수 없다. 주요 파일 해시는 아래에 기록했다.
- 실행한 검증: MuJoCo 3.3.7에서 작은 IK 재현, 복구 함수 분기 확인, 샘플러 100 seed, 4큐브 reset. 실행 코드는 변경하지 않았다.
- 실행하지 않은 검증: seed 81 물리 재현, 전체 배달·쌓기 100회, 새 MP4 생성·디코딩, 영상 육안 판정. 사용자가 보고한 성공률은 이번 검토에서 독립 확인하지 않았다.

**우선 수정할 것은 실패 시 조건을 우회하는 복구, 샘플러와 평가 지표다.** 그다음 IK와 경로 검증을 정리한다. 파지 자유도 추가는 원인 확인 후 결정한다.

P1은 결과 신뢰성 또는 동작 실패를 먼저 바로잡아야 하는 항목, P2는 확장·진단·산출물의 안정성을 위한 항목이다. 우선순위는 모든 항목이 현재 성공 장면의 실패 원인이라는 뜻이 아니다.

## 3차 반영 현황 — 2026-09-26 오후 (작업 세션이 기록)

리뷰 항목별로 무엇을 어떻게 반영했는지. 수치는 `scripts/classify_failures.py`(성공 = 래치 AND 종료 시 유지) 기준.

| 항목 | 상태 | 반영 내용 |
| --- | --- | --- |
| R01 복구가 조건 우회 | 반영 | `_recover`가 장면 상태로 분기: 손에 있으면 제자리 상승 후 재조준, 지점 위에 지지돼 있으면 놓기/후퇴(실패 기록), 책상 위면 재파지, 탑 위면 park. park는 grip 유지·제자리 정지. lower는 실제 지지(`_resting_on_support`) 없이는 settle로 못 감 |
| R02 전제조건 | 반영 | cross→lower에 `carrying` 필요; 단계 안의 복구는 그 tick을 정지 액션으로 끝냄; 다음 큐브·재파지는 현재 위치·높이 사용; reset에서 `_carry_aim` 제거 |
| R03 지지·성공 의미 | 반영 | `_placed_object_mask`가 `_resting_on_support` 사용; `_success`에 전 큐브 4° 이내 평평 조건 추가; `info["success_now"]`(현재) vs `is_success`(래치) 분리, 평가는 둘 다 요구 |
| R04 샘플러 검증 누락 | 반영 | 원본 셀·지터 모두 `_far_enough`·`_clear_of_hands`·`_servable` 통과 시에만 반환; 리셋별 `layout_fell_back`; 후보 24개 |
| R05 평가 지표 | 반영 | 큐브별·실제 접촉 해제 시점 기록, 최종/최대 변위, 종료 플래그 준수, `--layouts`, 아래 큐브 오차 보고 |
| R06 IK 180° | 반영 | `ik.rotation_error`가 쿼터니언 차(진짜 축각). 0/2/30/90/179/180° 검증: 옛 0/2/28.6/57.3/1.0/0 → 새 0/2/30/90/179/180 |
| R07 IK 최종 오차 | 반영 | 마지막 갱신 후 기구학 갱신하고 오차 계산. `_update_arm_targets`의 실패 전파는 미반영(컨트롤러 마감·복구가 처리) |
| R08 도달 맵 | 부분 | 맵에 생성 상수 서명 저장·불일치 시 경고. 레이아웃별 IK 검증(`_servable`: 집기·접근·놓기·운반 높이)이 배달·탑 모두에 적용. 팔별 맵·경로·충돌 검사는 미반영 |
| R09 4큐브 | 반영 | `cubes`는 2~3만 허용, 명시적 오류 |
| R10 손 형상 충돌 | 부분 | 샘플러가 집는 손목을 다른 큐브에서 90 mm 이상 띄움; geom 단위 경로 충돌 검사는 미반영 |
| R11 영상 | 반영 | 기본 1080p, 큐브별 phase 기록, 종료 시 성공 여부 출력, MP4 재읽기(해상도·fps·프레임) |
| R12 첫 reset | 반영 확인 | `_servable`이 홈 자세에서 직접 회전 계산 |
| R13 튜닝 지표 | 반영 | 완전 성공 기준, 굴림 = 착지→정지 변위, 손 안 오프셋은 접촉 유지·정지 프레임 평균(분산 출력), 배치 파일 seed 사용 |
| R14 운반 로그 | 반영 | `mj_objectVelocity` 원점 속도, 벡터 가속도, 요청→클램프량 vs 추종 지연 분리, 이동 큐브 추적 |
| R15 질량 | 반영 | 0.05 kg으로 README에 명시(0.2 kg 언급은 오류였음) |
| R16 패키징 | 반영 | `package-data`에 reachmap.npz |
| fable-max Q4 단순화 | 반영 | env.py의 RL 보상 기계(단계 사다리·퍼텐셜·비용항·stage 정보·`solve_body_position`) 제거, 래치·성공 판정만 `_update_latches`로. 고정 배치 성공 시점 불변, 세 과제 12/12 유지 |
| fable-max Q1.1 탑 위 재파지 | 반영 | 책상 위(z가 OBJECT_Z+20 mm 이내)일 때만 재파지, 아니면 park |
| fable-max Q3 배달 샘플러 검증 | 반영 | `_servable`을 base env로 올려 배달·탑 공용(집기·접근·놓기·운반 높이). 새 시드 200개 비교: 검증 있음 200/200, 없음 199/200 |

측정(고정 배치, 완전 무작위): 배달 100/100(중앙 5.3 / 최대 20.2 mm), 2단 100/100(7.4 / 11.2), 3단 100/100(8.0 / 14.1, 재시도 0). 밑 큐브는 위 큐브가 눕는 동안 최대 ~10 mm 흔들렸다 원위치. 배달 샘플러 검증 유/무 비교(새 시드 100~299, 각 200개, 고정 파일 `artifacts/layouts_10_s100-299_{validated,unvalidated}.json`): 검증 있음 200/200(중앙 4.9 / p90 14.2 / 최대 21.4 mm), 검증 없음 199/200(seed 211이 down 3회 타임아웃 후 park — 도달 불가 파지 자세). 리뷰어의 중단 기준(검증본에서 down 타임아웃 1건 이상)에 해당하지 않아 5 mm 문턱 유지. 탑 v2 배치(`artifacts/layouts_tower_10_100_v2.json`) 100/100, 위 큐브 7.9/10.5/12.0 mm, 가운데 7.5/9.5/12.2 mm, 재시도 0.

## 2차 검토 요약 — 2026-09-26 10:10 KST

코드가 다른 세션에서 계속 바뀌므로 10:06:19의 임시 복사본으로 재현하고, 이후 변경된 상태기계·샘플러는 다시 대조했다. 아래 R12는 재현 후 다른 세션에서 수정됐으며, 나머지 신규 R13~R16은 미수정이다. 현재 파일 지문은 문서 끝에 기록한다.

| 항목 | 2차 확인 결과 | 상태 |
| --- | --- | --- |
| R04 샘플러 | 손목 여유·층별 IK 검사가 추가됨. 그러나 원본 셀 반환의 손 여유 검사는 여전히 빠져 최신 scale 0.6 seed 0~11에서 2회 위반 | 미수정 |
| R08 도달성 | `_servable()`이 실제 층별 pick/drop IK를 추가 검사함. 운반 높이·연속 경로·충돌 검증까지 해결된 것은 아님 | 부분 개선 |
| R12 첫 reset | 새 랜덤 스택의 `_episode_rotations` 미초기화 오류 재현 → 이후 수정본의 첫 reset 성공 확인 | 수정 확인 |
| R13 튜닝 지표 | `delivered`는 전체 성공이 아니라 placed latch, `roll`은 실제 굴림 변위가 아님 | 신규 |
| R14 운반 로그 | 손목 속도의 기준점 오류, 속력 차이를 가속도로 표시, 리쉬 적용량과 잔여 오차 혼동 | 신규 |
| R15 질량 | 현재 큐브는 0.05kg. 이전 질의의 0.2kg와 다름 | 새로 확인한 실험 명세 불일치 |
| R16 패키징 | wheel에 reachmap.npz 미포함, wheel에서 패키지 import 실패 재현 | 신규 |

**2차 권고:** R01~R05와 함께 R13~R14의 측정부터 바로잡는다. 파지 설정을 추가 탐색하기 전에 어떤 성공·속도·굴림을 최적화하는지 정확히 해야 한다. R12의 수정 사실을 누락하고 현재 장애로 재보고하지 않는다.

## 이전 검토 이후 반영된 부분

- 모든 마감시간에 단순히 다음 phase로 가던 코드는 `_recover()`와 제한된 재시도로 교체됐다. 그러나 해제 단계의 조건 우회는 남았다(R01).
- `_resting_on_support()`가 추가되어 lower/settle은 손 이외 접촉 전체 대신 테이블·큐브의 아래쪽 수직 접촉을 검사한다. 평가 판정과 하중 검사는 아직 일치하지 않는다(R03).
- `down`의 마감시간은 거리/속도에 따라 계산된다. `lower`, `cross` 등은 여전히 고정 예산이다.
- 들어 올림 이력에서 법선 대향도 필수 조건이 제거됐다. 반면 RL용 `lift` 지표는 여전히 대향도에 종속된다. 이를 실제 파지 성공 횟수로 해석하지 않아야 한다.
- 고정 좌표 재생 도구 `scripts/layouts.py`가 있다. 실패 분류 스크립트에는 이 도구가 연결되지 않았다.
- 검토 도중 다른 세션이 코드를 수정했다. 최종 문서에는 park의 grip 유지, 낙하 감지, 백트래킹 샘플러, 실제 방출 변위 계산을 반영했다. 구 샘플러 수치는 변경 전 증거로만 보존한다.

## 수정 목록

### R01 · P1 · 복구가 settle·해제 조건을 우회한다

**근거:** [demonstration.py:116](/home/karma/Humanoid_Grasp_RL/src/humanoid_grasp_rl/demonstration.py:116), [park 동작:350](/home/karma/Humanoid_Grasp_RL/src/humanoid_grasp_rl/demonstration.py:350).

- settle 시간초과 → 평평하거나 지지됐는지와 무관하게 `let go`.
- let go 시간초과 → 손 접촉이 남아도 `back`.
- 재시도 소진 → `park`; grip 0 개방은 검토 중 수정되어 현재는 grip을 유지한다. 그러나 현재 위치 유지가 아니라 `retreat` 이동이며, 그 경로의 충돌 검증은 없다.
- lower의 높이 하한 도달 역시 지지 확인 없이 settle 진입을 허용한다.

**변경 이유:** 실패를 기록하는 것만으로는 실패 후 위험한 다음 동작을 막지 못한다.

**수정 방향:** 상태 결과를 진행/완료/실패로 구분한다. 지지 미확인, 접촉 잔류, 재시도 소진마다 유지·재배치·중단을 명시한다. 정상 성공 전이 조건을 복구 경로도 지켜야 한다. `park`는 물체 보유 상태별 정책이 필요하다.

**확인:** settle 불성립·손 접촉 잔류·재시도 소진을 각각 주입해 개방 또는 상승으로 자동 진행하지 않는지 검사한다. 아래 작은 분기 검사는 현재 우회를 재현했다.

```text
recover settle carrying=True -> let go
recover let go carrying=True -> back
recover lower carrying=True -> cross
```

### R02 · P1 · 복구 경로와 정상 전이의 전제조건이 불완전하다

**근거:** [복구:133](/home/karma/Humanoid_Grasp_RL/src/humanoid_grasp_rl/demonstration.py:133), [cross 완료:271](/home/karma/Humanoid_Grasp_RL/src/humanoid_grasp_rl/demonstration.py:271), [다음 큐브:369](/home/karma/Humanoid_Grasp_RL/src/humanoid_grasp_rl/demonstration.py:369).

- lower 실패 후 carrying이면 바로 cross로 간다. 주석의 “운반 높이로 복귀한 뒤 재조준”과 달리, cross는 수평·수직을 동시에 명령할 수 있다. 탑 옆에서 대각선으로 이동할 수 있다.
- cross→lower는 목표 xy 거리와 큐브 속도만 검사한다. 현재 파지·큐브 높이를 요구하지 않아, 목표 아래로 떨어져 정지한 큐브도 조건을 만족할 수 있다.
- 검토 중 cross에 낙하 감지가 추가됐지만, `_recover()` 호출 뒤 별도 `if`로 완료를 검사한다. 같은 tick에서 `reach`/`park` 결정을 `lower`로 덮어쓸 수 있다. 복구 뒤에는 이전 phase의 성공 검사와 action 생성을 계속하지 않아야 한다.
- settle에서 목표 반경을 벗어나면 `_recover(..., "settle", ..., False)`를 부르지만, 이 복구는 재파지가 아니라 `let go`를 선택한다. 새 낙하 감지와 복구 의미가 맞지 않는다.
- 재파지는 큐브의 xy만 갱신한다. 큐브가 다른 물체 위에 있거나 회전한 경우의 실제 높이·새 접근 자세는 재계획하지 않는다.
- 다음 큐브는 현재 위치가 아니라 `_initial_object_positions`를 사용한다. 이전 동작에 밀린 큐브를 옛 위치에서 집으려 할 수 있다.
- 초기화 시 `_carry_aim`을 지우지 않는다. 같은 env를 재사용하고 다음 에피소드에서 파지를 못 하면 이전 목표가 사용될 수 있다.

**변경 이유:** 재시도는 phase 이름 변경이 아니라 현재 장면에서 유효한 행동의 재계획이어야 한다.

**수정 방향:** 충돌 없는 높이까지 상승 완료 → 재조준을 분리한다. 파지 상실은 목표 도착보다 먼저 처리한다. 재파지·다음 큐브 접근 때 현재 pose를 읽는다. controller state를 한 객체로 모아 reset/새 물체/재시도 시 초기화 책임을 통일한다.

**확인:** 목표 아래 낙하, 옆으로 밀린 다음 큐브, 탑 측면에서 lower 실패, 연속 두 reset을 각각 재현한다. 재시도 뒤 큐브-손 상대 자세와 주변 큐브 변위도 기록한다.

### R03 · P1 · 제어기의 지지 판정과 평가의 놓임 판정이 다르다

**근거:** [env.py:547](/home/karma/Humanoid_Grasp_RL/src/humanoid_grasp_rl/env.py:547), [일반 접촉:811](/home/karma/Humanoid_Grasp_RL/src/humanoid_grasp_rl/env.py:811), [놓임 판정:935](/home/karma/Humanoid_Grasp_RL/src/humanoid_grasp_rl/env.py:935).

- 새 helper는 접촉 대상·법선의 수직 성분·접촉점 높이를 검사한다. 실제 지지력 크기와 의도한 지지 큐브의 ID는 검사하지 않는다.
- `_placed_object_mask()`는 여전히 모든 비손 접촉을 `supported`로 사용한다. 탑의 아래층부터 이어지는 지지 관계도 별도로 검사하지 않는다.
- 최종 성공은 평평함·각속도를 직접 검사하지 않는다. 정상 settle은 이를 검사하지만 R01의 우회가 존재한다.
- `_success_latch`는 성공을 한 번 달성한 뒤 유지된다. “한때 성공”과 “최종 프레임에서도 탑 유지”는 다르다.

**변경 이유:** 제어기가 안착으로 보지 않는 상태를 평가기가 성공으로 보거나, 완성 뒤 무너진 탑을 성공 영상으로 보고하지 않아야 한다.

**수정 방향:** 접촉/파지/의도한 지지/놓임의 의미를 분리하고 공통 판정으로 사용한다. 탑은 아래층의 유효 지지까지 연결해 검사한다. 보고값은 `ever_success`와 현재·종료 시점의 안정 성공을 구분한다. 힘 임계값은 질량·중력 및 실측으로 정한다.

**확인:** 측면 접촉만 있는 큐브, 잘못된 큐브 위의 물체, 놓은 뒤 회전하는 큐브, 성공 유지 후 탑 붕괴를 성공으로 유지하지 않는지 검사한다. 현재 실제 성공 영상이 오판정이었다고 확인한 것은 아니다.

### R04 · P1 · 새 샘플러의 원본 셀 반환 경로가 손 초기 여유를 검사하지 않는다

**근거:** [stack_env.py:79](/home/karma/Humanoid_Grasp_RL/src/humanoid_grasp_rl/stack_env.py:79), [jitter 검사:135](/home/karma/Humanoid_Grasp_RL/src/humanoid_grasp_rl/stack_env.py:135).

검토 도중 독립 선형 보간·50회 재시도 방식이 백트래킹으로 교체됐고 fallback 카운터가 추가됐다. 과거의 붕괴를 현재 결함으로 재기술하지 않는다.

**변경 전 측정:** 3큐브, reach, seed 0~99에서 scale 0.3/0.6/1.0의 nominal 반복은 88/91/36회, 고유 배치는 13/10/65개였다.

**1차 검토 당시 변경 후 재측정:** 같은 생성 조건에서 `_sample_layout()`만 실행했다. 물리 rollout 결과가 아니며, 이후 `_servable()` 추가 전 버전의 수치다.

| scale | nominal과 동일 | 고유 배치 | fallback | `_clear_of_hands` 실패 |
| --- | ---: | ---: | ---: | ---: |
| 0.3 | 0/100 | 98 | 0 | 17/100 |
| 0.6 | 0/100 | 96 | 0 | 19/100 |
| 1.0 | 0/100 | 100 | 0 | 11/100 |

- 최신 `search()`는 거리·nominal 범위·손목 여유를 검사하고, 반환 전에 `_servable()`로 pick/drop IK도 검사한다. 하지만 손 초기 여유는 jitter 배치에만 검사한다. jitter 불합격 시 원래 셀은 `_clear_of_hands()` 없이 반환할 수 있다. 2차 검토의 최신 코드에서도 scale 0.6 seed 0~11 중 2개가 이 여유 조건을 위반했다(fallback 0). 실제 geom 충돌률을 뜻하는 것은 아니다.
- fallback 카운터는 env 누적값이다. 분류 스크립트는 값이 0이 아니면 현재 에피소드를 fallback이라고 표시하므로 한 번 발생한 뒤 후속 정상 배치도 잘못 표시할 수 있다.
- 한 단계 후보를 무작위 8개만 시험한다. 탐색 실패와 유효 배치 부재는 다르며, 현재 fallback을 없애려면 둘을 구분해야 한다.

**변경 이유:** 샘플러가 다양성을 회복하면서 기존 유효성 조건을 빠뜨리면 새 실패 원인이 된다.

**수정 방향:** 원본 셀·jitter·fallback이 동일한 전체 배치 검증 함수를 통과하도록 한다. env 누적 fallback 수와 이번 reset의 fallback 여부를 분리한다. scale의 새 의미(허용 반경)를 기록하고 기존 보간 분포와 성능을 직접 비교하지 않는다.

**확인:** 현재 seed 0~99에서 손 여유 실패가 0인지 확인하고, 지점 간격·도달성·고유 배치·fallback을 함께 보고한다. 명시적 fallback 1회 뒤 정상 reset의 표시도 검사한다. 컨트롤러 비교에는 저장된 동일 좌표를 사용한다.

### R05 · P1 · 방출 변위는 수정됐지만 사건 시각·큐브별 평가가 아직 불완전하다

**근거:** [classify_failures.py:40](/home/karma/Humanoid_Grasp_RL/scripts/classify_failures.py:40).

**수정 확인:** 검토 도중 목표 오차의 차이 대신 큐브별 시작 좌표를 저장해 실제 xy 변위를 계산하도록 바뀌었다. 0mm 오차를 결측으로 취급하던 `released or ...`도 제거됐다.

**남은 문제:**

- 저장 시점은 여전히 `let go` 진입이다. 손 접촉이 끝난 시각이 아니므로 지표 이름을 실제 방출 후 이동으로 쓰기 어렵다.
- 재시도 때 같은 큐브의 최초 `release_at`을 갱신하지 않는다. 다음 시도의 운반까지 방출 후 이동에 포함될 수 있다.
- `picked`, `best`, `angle`은 큐브·시도별 분리가 안 돼 탑에서 서로 다른 물체의 사건을 섞는다. 현재 탑은 목표 xy가 같아 목표좌표 자체보다 물체 ID 혼합이 문제다.
- 높이만으로 picked를 판단하므로 튕김도 포함될 수 있다. best carry도 phase만 보고 실제 보유 여부는 검사하지 않는다.
- 종료 플래그를 여전히 무시하고, `--layouts` 입력이 없어 맵 변경 전후 공정한 비교를 직접 지원하지 않는다.
- 마지막 변위만 기록하면 큐브가 움직였다가 돌아온 경로를 놓친다. 종료 시 변위와 이후 최대 변위를 구분해야 한다.

**변경 이유:** 개선된 거리 계산을 정확한 물리 사건 및 물체 ID와 연결해야 튜닝 근거가 된다.

**수정 방향:** 물체 ID·시도 ID별 기록을 만들고, 실제 접촉 해제 event에 위치를 저장한다. episode 종료를 따르고, 고정 레이아웃 입력·누적 성공·최종 안정 성공을 따로 기록한다.

**확인:** 목표 기준 (10,0)mm→(-10,0)mm 이동에서 20mm를 출력하는지, 정확히 0mm 오차를 보존하는지, 동일 큐브 두 번째 시도와 탑의 두 번째 배달이 서로 섞이지 않는지 검사한다.

### R06 · P1 · IK가 180° 반대 자세를 성공으로 판정한다

**근거:** [ik.py:79](/home/karma/Humanoid_Grasp_RL/src/humanoid_grasp_rl/ik.py:79). 열벡터 외적 합의 크기를 회전 오차와 성공 판정에 사용한다.

**재현:** env reset 후 현재 손목 위치를 목표 위치로 두고, 현재 회전에 `diag(1,-1,-1)`을 곱한 180° 회전을 목표로 설정했다. `solve_body_pose(..., max_iterations=1)`이 아래를 반환했다.

```text
IK_180deg_result (True, 0.0, 0.0)
```

**변경 이유:** 이 표현은 180°에서도 0이 되어 실제 각거리와 다르다. 자세 후보·도달 맵의 성공 판정을 신뢰할 수 없게 한다. 현재 제한된 스크립트 자세에서 이 결함이 실제 실패를 일으켰다는 뜻은 아니다.

**수정 방향:** 성공 판정에는 실제 회전 각거리를 사용한다. 반복 갱신도 180° 근처를 처리하는 SO(3) 로그/쿼터니언 오차를 검토한다. 동일 외적 표현을 쓰는 action 생성부도 함께 확인한다.

**확인:** 0°, 작은 각도, 90°, 179°, 180°에서 보고한 각거리와 성공 여부를 검증한다. 도달 불가 반대 자세를 반드시 풀어야 하는 것이 아니라, 성공으로 오판정하지 않아야 한다. 회전 표현 근거는 참고자료 2.

### R07 · P2 · IK 마지막 오차가 이전 상태 값이며 호출부는 실패를 무시한다

**근거:** [ik.py:108](/home/karma/Humanoid_Grasp_RL/src/humanoid_grasp_rl/ik.py:108), [env.py:688](/home/karma/Humanoid_Grasp_RL/src/humanoid_grasp_rl/env.py:688).

- `refresh_dynamics=False`로 반복 한도를 소진하면 마지막 qpos 갱신 후 기구학을 갱신하지 않은 채 xpos/xmat로 최종 오차를 계산한다.
- `_update_arm_targets()`는 IK의 성공 여부·오차 반환값을 받지 않고 관절 목표를 적용한다.

```text
# 현재 손목에서 x +30mm 목표, max_iterations=1, refresh_dynamics=False
reported_position_error_m = 0.030000000000000000
actual_after_kinematics_m  = 0.022854138375181545
```

**변경 이유:** 리쉬 포화, IK 실패, 서보 추종 실패를 구별할 진단값이 현재 신뢰되지 않는다.

**수정 방향:** 최종 오차 계산 전 최소 기구학 패스를 실행한다. 원하는 목표/작업공간 제한/리쉬 제한/IK 잔차/실제 추종 오차를 구분해 전달하고, 지속 실패를 상태기계에 알려야 한다.

**확인:** 반환 관절각으로 다시 FK한 결과와 반환 오차가 일치해야 한다. 잘 푸는 경로의 동작은 유지하면서 불가능한 목표에 명시적 실패가 나오는지 검사한다.

### R08 · P1 · 도달 맵이 현재 작업 전체의 가능성을 보장하지 않는다

**근거:** [build_reachmap.py:55](/home/karma/Humanoid_Grasp_RL/scripts/build_reachmap.py:55), [env.py:486](/home/karma/Humanoid_Grasp_RL/src/humanoid_grasp_rl/env.py:486).

- 맵은 책상 높이 파지·놓기와 고정 여유 높이의 개별 자세를 rest에서 푼다. 탑 높이별 lower와 clear_z 경로, 장애물, 실제 시작 관절각에서의 연속 경로를 검사하지 않는다. 2차 검토 시 stack sampler에 `_servable()`이 추가되어 실제 층별 pick/drop endpoint는 별도 검사한다. 따라서 층별 놓기 자세가 전혀 검사되지 않는다는 과거 해석은 철회한다. 운반 높이·연속 경로·충돌 검사는 여전히 별도다.
- grasp와 place를 각각 양팔 중 어느 팔이든 성공하면 참으로 합친다. 두 단계가 같은 팔에서 가능한지 저장하지 않는다. 실제 잘못 허용한 셀의 존재는 이번에 열거하지 않았다.
- `_reachable()`은 가장 가까운 셀만 읽는다. 범위 밖 좌표를 명시적으로 거부하지 않고, jitter된 점의 실제 자세도 다시 풀지 않는다.
- npz에는 상수·관절 제한·모델·생성 코드 버전의 일치 여부를 확인할 metadata가 없다.

**변경 이유:** “맵에 있음”과 “현재 팔·높이·장애물 조건에서 배달 가능”을 혼동하지 않기 위해서다.

**수정 방향:** 팔별 가능성을 보존하고 맵의 적용 높이·자세·버전을 명시한다. 각 작업 요청은 실제 시작 관절각에서 접근/상승/운반/하강 경로를 웜스타트 IK 및 충돌 검사로 선별한다. 모든 셀의 장시간 물리 실행부터 시작할 필요는 없다.

**확인:** 기존 성공 배치와 경계·다층 배치에서 정적 맵 통과, 경로 검사 통과, 실제 완료를 따로 집계한다. jitter·범위 밖 좌표·상수 변경 후 stale map도 확인한다.

### R09 · P2 · 4큐브 입력과 오른팔 고정 배치의 계약을 명확히 해야 한다

**근거:** [stack_env.py:47](/home/karma/Humanoid_Grasp_RL/src/humanoid_grasp_rl/stack_env.py:47). `cubes`는 제한 없이 받지만 nominal 지점은 3개뿐이며 모두 +y 쪽이다.

```text
cubes4 place     IndexError index 3 is out of bounds for axis 0 with size 3
cubes4 randomize IndexError index 3 is out of bounds for axis 0 with size 3
```

**변경 이유:** README상 현재 지원은 2·3개다. 미지원 입력은 즉시 설명 가능한 오류로 거부하고, 확장 시에는 초기 배치부터 일반화해야 한다.

**수정 방향:** 당장은 지원 개수·팔 조합을 검증한다. 이후 n개 유효 배치 생성 및 팔별 레이아웃을 만들고, 양팔 교대는 물체와 팔의 할당을 명시한다. 오른팔 실패율은 이번에 실행하지 않았다.

**확인:** cubes=1/2/3/4 및 active_arm=0/1의 지원 범위가 명확해야 한다. 4개를 지원할 때는 초기 물체 수·목표 높이 수·모든 reset 배열 길이를 검사한다.

### R10 · P2 · 충돌 여유와 복귀가 손 전체의 형상을 보장하지 않는다

**근거:** [demonstration.py:205](/home/karma/Humanoid_Grasp_RL/src/humanoid_grasp_rl/demonstration.py:205), [복귀:358](/home/karma/Humanoid_Grasp_RL/src/humanoid_grasp_rl/demonstration.py:358).

- clear는 세 손가락 body 원점의 평균 거리다. 가장 가까운 손가락 표면·손목 링크의 거리가 아니다.
- 복귀는 첫 40스텝 후 실제로 탑을 벗어났는지와 관계없이 home 방향으로 전환된다. 초기 상승 목표도 현재 xy가 아니라 over_drop이므로 완전한 수직 이동을 보장하지 않는다.
- 펼친 손 아래 깊이는 홈 자세에서 측정한 상수다. 새 접근 방향·새 자세에도 같은 여유가 유효하다는 검증은 없다.

**변경 이유:** 성공 판정 거리와 충돌 없는 경로는 서로 다른 조건이다.

**수정 방향:** 충돌 회피는 손목·손가락·전완을 포함한 실제 geom 경로로 검사한다. 큐브별 접근 방향/집는 순서를 선택하고, 상승 완료 후 수평 복귀를 허용한다. 90° 큐브 대칭을 로봇 도달성의 대칭으로 취급하지 않는다.

**확인:** 손목 스침이 있었던 배치를 고정해 비대상 큐브 최대 변위, 최소 여유, 접촉 geom, 복귀 중 실제 높이를 비교한다.

### R11 · P2 · 영상 산출물의 성공·시간·메타데이터를 검증해야 한다

**근거:** [record_delivery.py:20](/home/karma/Humanoid_Grasp_RL/scripts/record_delivery.py:20), [녹화 루프:48](/home/karma/Humanoid_Grasp_RL/scripts/record_delivery.py:48).

- 기본 해상도는 1280×720으로, 이전 1920×1080 이상 요청과 다르다.
- fps=50을 고정한다. 현재 제어 주기와 일치하지만 변경 시 자동 검증되지 않는다.
- `--hold`는 물리를 진행하지 않고 마지막 프레임을 반복한다. 이를 추가 안정 유지 시간으로 해석하면 안 된다.
- phase 시각을 문자열 하나로 저장하므로 탑의 이전 배달·재시도 기록을 덮어쓴다.
- 출력 메시지는 과거 성공 latch를 사용하며 파일 작성 후 실제 MP4의 해상도·fps·길이·디코딩을 검사하지 않는다.

**변경 이유:** 사용자의 최종 산출물은 실제 성공 장면 영상이다. 정지 화면 연장은 안정성 증거가 아니다.

**수정 방향:** 1080p 기본값, 제어 주기 기반 fps 검증, 큐브·시도별 event 기록, 실제 안정 관찰과 편집상 정지 구간의 구분, MP4 재읽기 검증을 추가한다. 예외 시 writer/renderer도 닫히도록 정리한다.

**확인:** 생성된 MP4를 읽어 파일 크기·프레임 수·fps·재생 시간·해상도를 출력한다. 물리 안정성은 별도 기록하고 육안 품질 합격은 사용자에게 맡긴다.

### R12 · 수정 확인 · 새 `_servable()`이 첫 reset 이전 episode 자세를 참조했다

**근거:** `stack_env.py::_sample_layout → _servable`, `env.py::reset`, `demonstration.py::_palm_pose`.

10:06:19 복사본에서는 `_sample_layout()` 안에서 `_palm_pose()`를 호출했으나, 해당 함수가 읽는 `_episode_rotations`는 샘플링이 끝난 뒤 `_aim_wrist()`에서 생성됐다. 새 env의 randomize 첫 reset만 깨지고, place reset을 먼저 한 뒤 randomize로 바꾸면 통과해 실행 순서에 의존했다.

```text
fresh_stack_reset place OK
fresh_stack_reset randomize AttributeError 'G1StackEnv' object has no attribute '_episode_rotations'
place_then_randomize_reset OK
```

**현재 상태:** 10:06:31 수정본은 probe를 홈 관절각으로 놓고 회전을 직접 계산한다. 이후 최신 소스에서 `G1StackEnv(cubes=3, stage="randomize", action_mode="reach").reset(seed=0)` 성공을 확인했다. 현재 미해결 항목으로 세지 않는다.

**변경 이유/유지할 검증:** reset 중 샘플러가 이전 에피소드 상태에 의존하면 seed가 같아도 실행 순서에 따라 결과가 달라진다. 새 env 첫 reset, 다른 episode 실행 후 같은 seed reset을 모두 검사하고 파지 pose 계산식 두 벌의 일치도 유지해야 한다. 성공률 100회는 재검증하지 않았다.

### R13 · P1 · 파지 튜닝 도구가 다른 성공 조건과 잘못된 굴림 지표를 최적화한다

**근거:** [tune_grasp.py:76](/home/karma/Humanoid_Grasp_RL/scripts/tune_grasp.py:76), [held offset 측정:45](/home/karma/Humanoid_Grasp_RL/scripts/tune_grasp.py:45).

- `wins += bool(env._placed_latch[env.moving])`를 `delivered`로 출력한다. 손이 빠졌는지, 전체 물체가 성공했는지, 성공 유지 시간을 채웠는지 검사하지 않는다. 최종 산출물의 배달 성공과 다른 수치다.
- `rolls.append(final - min(best, final))`는 목표 오차 증가분이며 실제 settle 중 굴림 변위가 아니다. 목표에서 같은 거리인 두 지점 사이로 20mm 굴러도 0이 될 수 있다.
- held offset 보정은 고정 장면 마지막 프레임의 높이 >50mm만으로 성공을 판정한다. 손에 붙잡혀 안정적으로 정지했는지 확인하지 않는다. 대리 지표를 배제한다는 도구 설명과 맞지 않는다.
- `--layouts`가 돌려주는 실제 seed 목록을 버리고 `range(args.seeds)`로 실행한다. 저장된 seed가 부족하거나 연속 0 시작이 아니면 KeyError가 날 수 있다.
- 파일 끝은 E 모듈의 상수만 복구하고 D 모듈의 상수는 복구하지 않는다. 단독 프로세스 종료에는 영향이 작지만 같은 프로세스 재사용 시 마지막 스윕 설정이 남는다.

**변경 이유:** 잘못된 성공·굴림 지표로 선택한 파지 설정은 전체 배달 성능 향상을 보장하지 않는다.

**수정 방향:** R05와 공통 평가 기록을 사용한다. 완전한 배달, 실제 접촉 해제, settle 시작→종료 변위를 구분한다. offset은 접촉 유지·낮은 상대 속도를 확인한 여러 프레임에서 측정하고 분산도 남긴다. 재생 seed는 파일 목록을 사용한다.

**확인:** placed=True지만 손 접촉이 남은 상태를 배달 성공으로 세지 않기, 반경 일정한 굴림을 0으로 보지 않기, 3개만 저장된 layout 파일을 범위 초과 없이 처리하기. 이 항목은 코드 경로 확인이며 이번에 전체 튜닝 스윕을 돌리지 않았다.

### R14 · P1 · 운반 원인 분석 로그의 속도·가속도·리쉬 정의가 틀리다

**근거:** [trace_carry.py:61](/home/karma/Humanoid_Grasp_RL/scripts/trace_carry.py:61), [리쉬 기록:77](/home/karma/Humanoid_Grasp_RL/scripts/trace_carry.py:77).

- `data.cvel[hand_body, 3:]`는 곧바로 손목 body 원점의 선속도가 아니다. 손목 원점과 같은 기준점의 속도를 써야 한다.
- `abs(norm(v_t)-norm(v_prev))/dt`는 속력의 변화다. 방향만 바뀌는 운동의 가속도를 놓친다.
- `leash = norm(target_after_step - hand_after_step)`는 스텝 후 추종 오차다. 주석의 “이번 스텝 clamp가 깎은 양”도, 리쉬 적용 전 요구량도 아니다. `before_target`은 저장만 하고 사용하지 않는다.
- cube_body와 goal을 루프 밖에서 고정하므로 stack replay로 moving이 바뀌면 첫 큐브를 계속 추적한다. actuator ID도 선언만 하고 포화값을 출력하지 않는다.

**수치 재현:** reset 후 왼팔 wrist_roll 한 관절에 qvel=1rad/s, 나머지 0을 넣고 `mj_forward()`를 실행했다. 실제 rollout의 파지 실패 측정은 아니다.

```text
norm(cvel[hand,3:])                       = 0.3612617324030626 m/s
norm(mj_jacBody(position) @ qvel)         = 0.005286572889368984 m/s
norm(mj_objectVelocity(mjOBJ_XBODY)[3:]) = 0.005286572889368984 m/s
```

또한 v=(1,0,0)→(0,1,0)m/s, dt=0.02s이면 현재 가속도 출력은 0, 벡터 차분의 크기는 70.710678m/s²다.

**변경 이유:** 이 로그로 “팔이 빠르게 흔들려 놓쳤다” 또는 “리쉬가 포화됐다”를 판단하면 다른 현상을 원인으로 지목할 수 있다.

**수정 방향:** 위치 기준점에 맞는 Jacobian×qvel 또는 `mj_objectVelocity`를 사용하고, 월드 속도 벡터를 차분한다. raw 목표→workspace 제한→leash 제한을 실제 명령 시점에서 따로 기록한다. 물체·시도 ID도 기록한다. MuJoCo 문서의 object-centered velocity 정의를 확인했다(참고자료 5).

**확인:** 고정 속력 방향 전환, 알려진 관절속도, 실제 clamp가 없는 서보 지연, 탑의 다음 큐브 전환 사례에서 로그가 기대량과 일치해야 한다. 이 지적을 자유물체 큐브의 모든 cvel 사용이 틀렸다는 주장으로 확대하지 않는다.

### R15 · P2 · 큐브 질량의 설명과 현재 모델이 다르다

**근거:** [env.py:423](/home/karma/Humanoid_Grasp_RL/src/humanoid_grasp_rl/env.py:423). XML은 `mass=".05"`이며 실제 컴파일된 `model.body_mass[object_body_ids]`도 `[0.05]`kg다.

이전 질의의 약 0.2kg와 4배 차이 난다. 현재 g=9.81m/s²에서 큐브 무게는 0.4905N이며 1.962N이 아니다. 어느 질량을 의도했는지는 이번 코드만으로 결정할 수 없다. 0.05kg 자체가 물리 버그라는 뜻도 아니다.

**변경 이유:** 힘/무게 비율과 지지 하중 임계값, 파지 여유 해석의 기준을 일치시켜야 한다.

**수정 방향:** 질량을 명명된 한 설정으로 두고 실험 결과에 컴파일된 질량·중력을 기록한다. 의도가 50g이면 설명을 고치고, 200g이면 물리 변경으로 별도 승인·재검증한다. 이번 검토에서는 질량을 바꾸지 않았다.

**확인:** XML 설정→MuJoCo body mass→보고된 mg가 일치해야 한다. 기존 결과가 어느 질량에서 나온 것인지도 구별한다.

### R16 · P2 · wheel에 필수 reachmap.npz가 포함되지 않아 설치 후 import가 실패한다

**근거:** [pyproject.toml](/home/karma/Humanoid_Grasp_RL/pyproject.toml), [env.py:34](/home/karma/Humanoid_Grasp_RL/src/humanoid_grasp_rl/env.py:34).

`env.py`는 import 때 패키지 옆의 reachmap.npz를 읽지만, 패키징 설정에 이 데이터를 포함하는 선언이 없다. editable 설치는 원본 파일을 읽어 문제를 숨긴다.

**실제 검증:** /tmp에 복사한 소스에서 `pip wheel --no-deps . -w dist`로 wheel을 만들고, 별도 디렉터리에 추출한 패키지를 우선 import했다. 프로젝트 venv에 설치하거나 소스를 변경하지 않았다. 빌드 의존성은 pip의 임시 격리 환경을 사용했다.

```text
wheel: humanoid_grasp_rl-0.1.0-py3-none-any.whl
reachmap_in_wheel: False
wheel_import_exit: 1
FileNotFoundError: .../wheel_extract/humanoid_grasp_rl/reachmap.npz
```

최초 `--no-build-isolation` 시도는 현재 venv의 bdist_wheel 부재로 실패했다. 이는 프로젝트 패키징 결함의 근거로 삼지 않았고, 격리 빌드에 성공한 뒤 누락을 확인했다.

**변경 이유:** 소스 폴더에서만 실행되는 상태로는 다른 환경에서 재현·배포할 수 없다.

**수정 방향:** setuptools package-data에 reachmap.npz를 명시하고, wheel의 실제 파일 목록과 깨끗한 설치에서 import를 검사한다. map 재생성도 맵이 없는 상태에서 env import에 막히는 현재 순환 의존을 별도로 정리한다. 근거는 setuptools 공식 문서(참고자료 6).

**확인:** 새 wheel 안에 npz가 있고 원본 checkout을 보지 않는 실행에서 패키지 import·환경 초기화가 성공해야 한다. 같은 PC의 editable 설치 성공만으로 통과시키지 않는다.

## 미확정 원인: seed 81의 회전 이탈

위 코드 결함이 seed 81의 직접 원인이라고 단정하지 않는다. 대향도 0은 불량 파지의 충분조건이 아니다. 추가 측정은 손-큐브 상대 회전·각속도, 접촉점 이동, 마찰 여유, 손목 가속도, 탑 최초 접촉 시각이다. 회전이 공중에서 먼저 발생하는지, 모서리 충돌 뒤 발생하는지부터 구분한다.

멈춤만 / 부드러운 멈춤 후 추가 조임을 같은 초기 상태에서 비교한다. grip 증가는 힘과 기하를 동시에 바꾸므로 일괄 증가부터 하지 않는다. 독립 관절 조정이 기존 스칼라 보간보다 유효하다는 증거가 생기면 자유도 추가를 검토한다.

## 실행 순서와 완료 조건

1. **R01~R03:** 실패 경로와 지지·성공의 의미를 일치시킨다. 조건 미충족 상태에서 놓거나 후퇴하는 경로를 없앤다.
2. **R04~R05, R13~R15:** 배치·튜닝·진단 지표와 질량 명세를 수정하고 고정 검증 세트를 저장한다. 기존 성공률과 새 분포의 성공률을 섞지 않는다.
3. **R06~R08:** IK 오차와 실패 전파를 고치고 팔·높이·장애물을 포함한 경로 검사를 연결한다.
4. **R09~R10:** 4큐브·양팔 교대는 명시적 입력 계약과 충돌 검증 뒤 확장한다.
5. **R11, R16:** 고정 배치의 실제 최종 안정 상태와 1080p 영상 파일, 재현 가능한 설치 산출물을 함께 확인한다. R12는 첫 reset 회귀 검사로 유지한다.

각 단계는 실패 재현 사례와 기존 성공 사례를 함께 검증한다. 작은 검사가 통과한 뒤에만 별도의 미사용 배치로 100회 평가한다. 실패율뿐 아니라 고유 배치 수·재시도 수·마감 원인·비대상 물체 변위·최종 탑 유지 여부를 보고한다.

## 검증 기록과 제한

- 검증은 `.venv/bin/python`의 일회성 검사와 /tmp 소스 복사본의 격리 wheel 빌드로 실행했다. 작업 트리의 테스트 파일·실행 코드·모델·도달 맵은 수정하지 않았다. 다른 세션의 코드 변경과 이 검토의 문서 변경은 구별한다.
- IK 180° 검사는 위치를 현재 손목에 맞추고 목표 회전만 반전했다. 성공 반환이 잘못됐음을 확인한 것이며, 그 자세의 물리적 도달 가능성을 검사한 것은 아니다.
- IK 마지막 오차 검사는 `position_tolerance=1e-8`, `max_iterations=1`, `refresh_dynamics=False`로 마지막 갱신 경로를 강제로 통과시켰다.
- 샘플러 결과는 위에 명시한 seed와 현재 맵에만 해당한다. `100/100`이라는 사용자의 배달 결과를 이번에 다시 실행하거나 무효라고 판정하지 않았다.
- 기록만으로 충분히 확인한 결함과 실제 rollout이 필요한 가설을 분리했다. 접촉력/파지 품질 개선량, 새로운 복구 성공률은 아직 측정하지 않았다.

## 참고자료

1. [MuJoCo 3.3.7 API](https://mujoco.readthedocs.io/en/3.3.7/APIreference/APIfunctions.html): 기구학/forward 계산과 접촉력 API의 역할 확인.
2. [Modern Robotics: 회전의 exponential coordinates](https://modernrobotics.northwestern.edu/nu-gm-book-resource/3-2-3-exponential-coordinates-of-rotation-part-2-of-2/): 회전행렬과 회전벡터·로그 표현의 근거.
3. [BehaviorTree.CPP 기본 의미](https://behaviortree.dev/docs/learn-the-basics/BT_basics/): 실행 중/성공/실패 및 제한된 재시도 설계 참고. 라이브러리 도입 제안은 아니다.
4. [Modern Robotics: force closure](https://modernrobotics.northwestern.edu/nu-gm-book-resource/12-2-3-force-closure/): 대향도 하나와 실제 접촉 안정성을 구분하는 근거.

5. [MuJoCo 3.3.7 mj_objectVelocity](https://mujoco.readthedocs.io/en/3.3.7/APIreference/APIfunctions.html#mj-objectvelocity): object-centered 속도 정의와 기준점 선택.
6. [setuptools Data Files Support](https://setuptools.pypa.io/en/stable/userguide/datafiles.html): runtime 데이터의 package-data 포함 방식.

## 검토 시점 파일 지문 (SHA-256)

2차 최종 대조: 2026-09-26T10:12:20.629290+09:00

```text
src/humanoid_grasp_rl/demonstration.py 031f0a393d2517676cc7e43ffee6f0cce41796c76a313b6a01ec5877b03fb85d
src/humanoid_grasp_rl/env.py cd055fe9761774211823988ef116a33031b10b564d3fb59f24972885bae314c4
src/humanoid_grasp_rl/stack_env.py 6de9e7210fd3205bc41cd2a8e17333506ca3b8beca7b563ba67adf266a519aa7
src/humanoid_grasp_rl/ik.py 42dc3698b69809e8ba0ddd17c68794ebb6f229fc422227c1b896ee11b21f19d6
scripts/classify_failures.py 4608a183e73dd4b585839771714199f3439fd0cad52cb52e27f01368a6db1766
scripts/build_reachmap.py 764c7b60dba84971cec02cea95f85d0bdfc9a4af57412ef7ea061318c68f0b77
src/humanoid_grasp_rl/reachmap.npz 932f689b53082c81c6e84189ae5a5284066e0a74a932af356f7223a978cb6a4b
scripts/tune_grasp.py 14ad804534cf335f845f500e6bc75cba0ea36f48143ca59a1054822f576b63f4
scripts/trace_carry.py 9ececc2f3576fae5eb3046b1f4736f6d01fe9dcde0cf587997e0516ed930d091
pyproject.toml 0f431d40a2082932228e6984fb888b73b5080ef3b4ea069d2580b9bb279945b3
```
