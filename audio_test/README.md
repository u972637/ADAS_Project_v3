# 음향 테스트 데이터 시트

스피커 거리·각도·주파수별 음압(dB) 측정 결과를 기록하는 시트. 나중에
MATLAB 등에서 이 CSV를 읽어 그래프를 그릴 예정이다.

## 파일

- `with_reflector.csv` / `without_reflector.csv` — 1차 측정(반사판 유무).
  거리 1/5/10/20/30/50m × 각도 0/15/30/45/60/90/180° × 주파수
  500~1500Hz(100Hz 간격)+2000/3000/4000Hz, 1회 측정.
- `new_with_reflector.csv` / `new_without_reflector.csv` — 추가 측정용
  템플릿. 거리 10/50m × 각도 0/30/90° × 주파수 2000Hz만, **2회 측정 +
  평균**을 기록하도록 열 구성이 다르다(아래 참고). 값은 비어 있으며
  실측해서 채운다.

## 열(column)

### `with_reflector.csv` / `without_reflector.csv`

| 열 | 의미 |
| --- | --- |
| `distance_m` | 측정 거리(미터). 1,5,10,20,30,50 — 측정 중 가장 안 바뀌는 값이라 맨 앞. |
| `angle_deg` | 스피커 조준 각도. 0/15/30/45/60/90/180도. |
| `frequency_hz` | 테스트 주파수. 500~1500Hz 100Hz 간격 + 2000/3000/4000Hz — 측정 중 가장 자주 바뀌는 값이라 맨 뒤. |
| `dB` | 측정한 음압(dB). 1회만 측정 — 반복 평균 없음. |
| `baseline_dB` | 그 조건에서의 배경/기준 소음 레벨(ambient baseline, dB) — 스피커 신호와 별개로 측정한 "기본" 상태값. |

행 순서는 `distance_m` → `angle_deg` → `frequency_hz` 순으로 정렬돼 있다
(거리 고정 → 각도 고정 → 주파수만 빠르게 스윕하는 실제 측정 순서와 맞춤).
각 CSV는 6개 거리 × 7개 각도 × 14개 주파수 = 588행이다.

### `new_with_reflector.csv` / `new_without_reflector.csv`

| 열 | 의미 |
| --- | --- |
| `distance_m` | 측정 거리(미터). 10, 50. |
| `angle_deg` | 스피커 조준 각도. 0, 30, 90도. |
| `frequency_hz` | 테스트 주파수. 2000Hz만. |
| `dB1` | 1차 측정값. |
| `dB2` | 2차 측정값. |
| `avg_dB` | `dB1`, `dB2`의 평균. |
| `baseline_dB` | 그 조건에서의 배경/기준 소음 레벨. |

각 CSV는 2개 거리 × 3개 각도 × 1개 주파수 = 6행이다.
