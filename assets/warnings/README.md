# 경계음 (warnings) — 현재 상태

`placeholder_track_start2.wav` (boar/water_deer 각 1개)는 **임시 플레이스홀더**다.
`assets/voice/track_start2.mp3`를 `mpg123 -w`로 WAV 변환해 복사한 것으로,
실제 경계음이 아니라 "메뉴 2 테스트 시 `Warning asset missing` 경고 없이
경계음 단계까지 흐름을 확인"하기 위한 용도다.

## 실제 경계음 준비 시 할 일

1. `assets/warnings/boar/`, `assets/warnings/water_deer/`에 들어있는
   `placeholder_track_start2.wav`를 삭제한다.
2. 각 폴더에 실제 경계음 WAV 파일을 넣는다 (최소 1개, 여러 개면 무작위
   비반복 선택됨 — `animal/wildlife.py`가 `AudioCatalog.from_directory()`로
   폴더째 읽는다).
3. EQ 버전을 쓰려면 `assets/warnings_eq/{boar,water_deer}/`에도 동일하게
   넣고 `animal/settings.py`의 `WARNING_AUDIO_VARIANT`를 `"eq"`로 바꾼다.
