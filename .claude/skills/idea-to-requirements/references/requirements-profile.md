# HEVEN 요구사항 프로필

ISO/IEC/IEEE 29148을 참고한 간소화이며 표준 준수나 인증을 뜻하지 않는다.

## 계층

`heven.toml`의 `requirements.levels`에서 필요한 계층만 고른다.

| 계층 | 접두사 |
|---|---|
| business | `BR` |
| stakeholder | `STK` |
| system | `SYS` |
| software | `SW` |
| hardware | `HW` |

순서는 `BR → STK → SYS → SW, HW`다. `SW`와 `HW`는 나란히 놓인다. 부모는 선택한 계층 중 가장 가까운 상위 계층에 연결한다. `SW-VEH-001`처럼 세부 분류는 저장소에서 정한다.

## 필드

`UID`, `TITLE`, `STATEMENT`, `STATUS`는 필수다. `RATIONALE`, `COMMENT`, Parent 관계는 선택이다. `SW`는 `CATEGORY`도 필수이며 `Functional`, `Non-functional/Performance Efficiency`, `Non-functional/Other` 중 하나를 쓴다.

수식과 인터페이스처럼 여러 요구사항이 함께 쓰는 내용은 별도 문서에 두고 참조한다. Issue 연결은 `docs/requirements-workflow.md`의 UID 블록으로 관리하고 `COMMENT`에 적지 않는다.

## 상태

| 상태 | 의미 |
|---|---|
| `Draft` | 작성 중. 미정 항목이 있어도 된다 |
| `In Review` | 미정 항목을 풀고 검토를 요청한 상태 |
| `Approved` | 담당자가 명시적으로 승인한 구현 기준 |
| `Retired` | 폐기. `COMMENT`에 이유를 쓴다 |

새 요구사항은 `Draft`로 시작하고, 승인 없이 `Approved`로 바꾸지 않는다.

## 작성 체크리스트

강제 규칙이 아니라 쓰고 검토할 때 참고하는 기준이다. 걸리는 항목은 사용자에게 알린다.

- 한 문장에 의무가 하나인가?
- 수치, 단위, 조건이 있어 시험할 수 있는가? "적절한", "확정된 값" 같은 표현에 미정 사항을 숨기지 않았는가?
- 어떻게 검증할지 정해져 있는가?
- 부모 요구사항과 실제로 관련 있는가?
