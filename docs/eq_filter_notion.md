# 퇴치 음원 EQ 필터 (1.5~2kHz 강조)

> **요약** — 멧돼지·고라니용 퇴치 음원 20개에 "1.5~2kHz는 그대로, 나머지 대역은 −12dB" EQ를 씌운 버전을 만들었다. 반사판을 달았을 때 지향성이 가장 뚜렷한 2kHz 부근으로 소리 에너지를 몰아주기 위해서다. 대부분 클립에서 이 대역이 차지하는 비중이 **약 5~13배** 늘었지만, 원래 이 대역 소리가 거의 없는 클립은 효과가 약하다.

작성일: 2026-10-17

---

## 1. EQ란?

**EQ(이퀄라이저) = 주파수별 볼륨 조절.** 소리를 낮은 소리 ~ 높은 소리로 나눠서 구간마다 크기를 따로 키우거나 줄이는 것이다. 음악 앱의 "베이스 부스트"가 저음 구간만 키우는 EQ다.

이번 필터는 **1.5~2kHz는 그대로 두고 나머지를 줄여서**, 결과적으로 그 대역이 상대적으로 강조되게 만든다.

## 2. 왜 1.5~2kHz인가

스피커 각도별 음압 측정(`audio_test/`) 결과, **0°(정면) 대비 옆(45°·90°)에서 소리가 얼마나 줄어드는지**:

| 주파수 | 반사판 O · 45° | 반사판 O · 90° | 반사판 X · 45° | 반사판 X · 90° |
| --- | --- | --- | --- | --- |
| 500 Hz | −7.2 dB | −7.6 dB | −2.6 dB | −3.2 dB |
| 1000 Hz | −12.7 dB | −12.4 dB | −1.8 dB | −1.2 dB |
| 1500 Hz | −9.7 dB | −13.2 dB | −0.5 dB | −2.0 dB |
| **2000 Hz** | **−19.7 dB** | **−22.9 dB** | −5.2 dB | −2.5 dB |
| 3000 Hz | −14.8 dB | −21.4 dB | −6.5 dB | −10.4 dB |

*1·5·10·20·30·50m 측정값 평균. 반사판 O 데이터는 일부 미측정 칸 제외.*

- 반사판을 달면 **2kHz에서 옆으로 새는 소리가 가장 많이 줄어든다** → 정면(동물 쪽)으로만 소리를 보내기 좋다.
- 반사판이 없으면 대역 상관없이 지향성이 약하다 → 이 EQ는 **반사판과 같이 쓸 때** 의미가 있다.
- 참고: 1600~1900Hz는 측정하지 않았고, 3kHz도 지향성이 강한 편이라 대역을 넓히는 것도 검토할 만하다.

## 3. 필터 설계

| 주파수 구간 | 볼륨 |
| --- | --- |
| ~ 1 kHz | −12 dB (소리 세기 약 1/4) |
| 1 ~ 1.5 kHz | 서서히 올라감 |
| **1.5 ~ 2 kHz** | **0 dB (그대로)** |
| 2 ~ 3 kHz | 서서히 내려감 |
| 3 kHz ~ | −12 dB |

- **볼륨만 바뀌고 소리의 시간 모양·길이는 그대로**다 (선형 위상 FIR 필터).
- **전체 평균 크기(RMS)는 원본과 같게 맞췄다.** 그래서 원본과 비교하면 "더 커졌다"가 아니라 "음색이 바뀌었다"로 들린다. 최대값은 −1dBFS 이하로 제한(이번 20개는 제한에 걸린 클립 없음).
- 경계를 급하게 자르지 않고 서서히 바꿔서 울림 같은 잡음이 생기지 않게 했다.

## 4. 결과 요약

**1.5~2kHz 비중** = 클립 전체 소리 에너지 중 1.5~2kHz가 차지하는 비율.

| 클립 | 소리 | 길이 | 1.5~2kHz 비중 (원본 → 필터) |
| --- | --- | --- | --- |
| boar_01 | 늑대 울음 | 6.0초 | 1.3% → 12.4% |
| boar_02 | 개 짖음·으르렁 | 6.0초 | 2.9% → 20.9% |
| boar_03 | 개 짖음·으르렁 | 2.6초 | 0.4% → 2.5% ⚠️ |
| boar_04 | 개 짖음·으르렁 | 4.0초 | 2.9% → 23.5% |
| boar_05 | 개 짖음·으르렁 | 6.0초 | 0.7% → 8.6% |
| boar_06 | 개 짖음·으르렁 | 1.7초 | 0.2% → 2.6% ⚠️ |
| boar_07 | 개 짖음·으르렁 | 1.8초 | 0.1% → 1.3% ⚠️ |
| boar_08 | 개 짖음·으르렁 | 6.0초 | 1.1% → 11.2% |
| boar_09 | 개 짖음·으르렁 | 1.4초 | 3.7% → 19.4% |
| boar_10 | 개 짖음·으르렁 | 4.0초 | **23.6% → 55.2%** |
| water_deer_01 | 개 짖음·으르렁 | 5.4초 | 3.2% → 27.6% |
| water_deer_02 | 개 짖음·으르렁 | 6.0초 | 5.3% → 32.0% |
| water_deer_03 | 개 짖음·으르렁 | 0.9초 | 0.3% → 3.0% ⚠️ |
| water_deer_04 | 개 짖음·으르렁 | 6.0초 | 3.7% → 18.9% |
| water_deer_05 | 개 짖음·으르렁 | 6.0초 | 2.2% → 23.1% |
| water_deer_06 | 개 짖음·으르렁 | 6.0초 | 0.7% → 8.8% |
| water_deer_07 | = boar_03 | | |
| water_deer_08 | = boar_01 | | |
| water_deer_09 | = boar_08 | | |
| water_deer_10 | = boar_02 | | |

- 대부분 클립은 비중이 **약 5~13배** 늘었다 (원래 비중이 높던 boar_10은 2.3배).
- ⚠️ 표시 클립은 원래 1.5~2kHz 소리가 거의 없어서 필터 후에도 3% 이하다. **EQ는 이미 있는 소리의 크기만 바꿀 뿐 없는 소리를 만들지 못하기 때문**이다. 이 클립들은 해당 대역 성분이 많은 다른 음원으로 바꾸는 걸 검토.
- water_deer_07~10은 멧돼지 쪽 클립과 **완전히 같은 파일**이다(아래 비교에서는 생략).

## 5. 원본 vs 필터 후 들어보기

> 이어폰보다 **스피커**로 들어야 차이가 잘 들린다. 필터 후 소리가 더 "얇고 날카롭게" 들리는 게 정상이다.

### 멧돼지용

#### boar_01 · 늑대 울음 · 1.3% → 12.4%
- 원본: 📎 `assets/audio/boar/boar_01.wav`
- 필터 후: 📎 `assets/audio_eq/boar/boar_01.wav`

#### boar_02 · 개 짖음·으르렁 · 2.9% → 20.9%
- 원본: 📎 `assets/audio/boar/boar_02.wav`
- 필터 후: 📎 `assets/audio_eq/boar/boar_02.wav`

#### boar_03 · 개 짖음·으르렁 · 0.4% → 2.5% ⚠️
- 원본: 📎 `assets/audio/boar/boar_03.wav`
- 필터 후: 📎 `assets/audio_eq/boar/boar_03.wav`

#### boar_04 · 개 짖음·으르렁 · 2.9% → 23.5%
- 원본: 📎 `assets/audio/boar/boar_04.wav`
- 필터 후: 📎 `assets/audio_eq/boar/boar_04.wav`

#### boar_05 · 개 짖음·으르렁 · 0.7% → 8.6%
- 원본: 📎 `assets/audio/boar/boar_05.wav`
- 필터 후: 📎 `assets/audio_eq/boar/boar_05.wav`

#### boar_06 · 개 짖음·으르렁 · 0.2% → 2.6% ⚠️
- 원본: 📎 `assets/audio/boar/boar_06.wav`
- 필터 후: 📎 `assets/audio_eq/boar/boar_06.wav`

#### boar_07 · 개 짖음·으르렁 · 0.1% → 1.3% ⚠️
- 원본: 📎 `assets/audio/boar/boar_07.wav`
- 필터 후: 📎 `assets/audio_eq/boar/boar_07.wav`

#### boar_08 · 개 짖음·으르렁 · 1.1% → 11.2%
- 원본: 📎 `assets/audio/boar/boar_08.wav`
- 필터 후: 📎 `assets/audio_eq/boar/boar_08.wav`

#### boar_09 · 개 짖음·으르렁 · 3.7% → 19.4%
- 원본: 📎 `assets/audio/boar/boar_09.wav`
- 필터 후: 📎 `assets/audio_eq/boar/boar_09.wav`

#### boar_10 · 개 짖음·으르렁 · 23.6% → 55.2%
- 원본: 📎 `assets/audio/boar/boar_10.wav`
- 필터 후: 📎 `assets/audio_eq/boar/boar_10.wav`

### 고라니용

#### water_deer_01 · 개 짖음·으르렁 · 3.2% → 27.6%
- 원본: 📎 `assets/audio/water_deer/water_deer_01.wav`
- 필터 후: 📎 `assets/audio_eq/water_deer/water_deer_01.wav`

#### water_deer_02 · 개 짖음·으르렁 · 5.3% → 32.0%
- 원본: 📎 `assets/audio/water_deer/water_deer_02.wav`
- 필터 후: 📎 `assets/audio_eq/water_deer/water_deer_02.wav`

#### water_deer_03 · 개 짖음·으르렁 · 0.3% → 3.0% ⚠️
- 원본: 📎 `assets/audio/water_deer/water_deer_03.wav`
- 필터 후: 📎 `assets/audio_eq/water_deer/water_deer_03.wav`

#### water_deer_04 · 개 짖음·으르렁 · 3.7% → 18.9%
- 원본: 📎 `assets/audio/water_deer/water_deer_04.wav`
- 필터 후: 📎 `assets/audio_eq/water_deer/water_deer_04.wav`

#### water_deer_05 · 개 짖음·으르렁 · 2.2% → 23.1%
- 원본: 📎 `assets/audio/water_deer/water_deer_05.wav`
- 필터 후: 📎 `assets/audio_eq/water_deer/water_deer_05.wav`

#### water_deer_06 · 개 짖음·으르렁 · 0.7% → 8.8%
- 원본: 📎 `assets/audio/water_deer/water_deer_06.wav`
- 필터 후: 📎 `assets/audio_eq/water_deer/water_deer_06.wav`

*water_deer_07~10은 각각 boar_03 · boar_01 · boar_08 · boar_02와 같은 파일이라 생략.*

## 6. 직접 돌려보기

```bash
uv run python tools/eq_filter.py                    # EQ 음원 20개 생성 + 비중 표 출력
uv run python tools/eq_filter.py --compare boar_07  # 원본 → 1초 쉬고 → 필터 후 연속 재생
```

- 실제 퇴치 동작(start.sh 1·2번, `main.py`)에 EQ 음원을 쓰려면 `animal/settings.py`에서 `USE_EQ_AUDIO = True`.
- 대역·감쇠량을 바꾸려면 `tools/eq_filter.py` 맨 위 상수(`EQ_PASS_HZ`, `EQ_CUT_DB` 등)를 고치고 다시 실행.

## 7. 다음 단계

- [ ] 스피커로 원본/필터 후를 직접 들어보고 퇴치음으로서 위협감이 유지되는지 확인
- [ ] 반사판 장착 상태에서 원본/필터 후 음원을 각도별로 재생해 **실제로 옆으로 새는 소리가 줄었는지** 측정
- [ ] ⚠️ 클립(boar_03·06·07, water_deer_03)을 1.5~2kHz 성분이 많은 음원으로 교체 검토
- [ ] 1600~1900Hz 추가 측정 후 대역 범위(필요하면 3kHz까지 확장) 재검토
