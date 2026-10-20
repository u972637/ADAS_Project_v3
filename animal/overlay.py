"""Korean-text overlay helper, shared by the production runtime and the
manual-control experiment menu. cv2.putText() cannot render Hangul (shows as
"???"), so Korean text is drawn with PIL using a CJK font, then composited
back onto the cv2 BGR frame.
"""

import os

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

_KOREAN_FONT_CANDIDATES = (
    "/usr/share/fonts/truetype/nanum/NanumGothic.ttf",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
)
_korean_font_cache: dict[int, "ImageFont.FreeTypeFont | None"] = {}


def _korean_font(size: int):
    if size not in _korean_font_cache:
        font = None
        for path in _KOREAN_FONT_CANDIDATES:
            if os.path.isfile(path):
                font = ImageFont.truetype(path, size)
                break
        if font is None:
            print(
                f"[WARN] 한글 폰트를 못 찾음({_KOREAN_FONT_CANDIDATES}) "
                f"— 한글 오버레이가 깨질 수 있음"
            )
        _korean_font_cache[size] = font
    return _korean_font_cache[size]


def put_text_kr(frame_bgr, text: str, org: tuple[int, int],
                 font_size: int = 20, color_bgr: tuple[int, int, int] = (255, 255, 255)):
    """cv2 BGR 프레임 위에 한글이 섞인 텍스트를 PIL로 그려 합성한다.
    폰트를 못 찾으면(이 머신에 나눔고딕이 없으면) cv2.putText로 폴백한다
    (한글은 깨지지만 최소한 영문/숫자는 보이게)."""
    return put_lines_kr(frame_bgr, [(text, org, font_size, color_bgr)])


def put_lines_kr(frame_bgr, lines):
    """여러 줄을 한 번의 PIL 왕복으로 그린다 -- 한글이 섞인 줄마다 매번
    cv2<->PIL 변환을 반복하면(특히 큰 프레임에서) 불필요하게 느려지므로,
    한 프레임에 여러 줄을 그릴 때는 put_text_kr을 줄 수만큼 호출하지 말고
    이 함수에 (text, org, font_size, color_bgr) 목록을 한 번에 넘길 것.
    cv2.putText는 한글을 "???"로 깨뜨리므로, 한글이 섞인 텍스트는 반드시
    이 경로(또는 put_text_kr)를 거쳐야 한다."""
    if not lines:
        return frame_bgr
    fonts = [_korean_font(size) for _, _, size, _ in lines]
    if all(font is None for font in fonts):
        for (text, org, size, color_bgr), _ in zip(lines, fonts):
            cv2.putText(frame_bgr, text, org, cv2.FONT_HERSHEY_SIMPLEX,
                        size / 30.0, color_bgr, 2)
        return frame_bgr
    image = Image.fromarray(cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB))
    drawer = ImageDraw.Draw(image)
    for (text, org, size, color_bgr), font in zip(lines, fonts):
        if font is None:
            # Falls back per-line onto the still-RGB PIL canvas; Hangul in
            # this particular line will be missing glyphs, but mixed
            # English/numeric lines on the same frame still render.
            font = ImageFont.load_default()
        drawer.text(org, text, font=font, fill=(color_bgr[2], color_bgr[1], color_bgr[0]))
    return cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)
