"""matplotlib 한글 폰트 + 공통 스타일.

그래프를 그리는 모든 스크립트가 이 모듈을 import 하면 된다.
matplotlib 기본 폰트(DejaVu Sans)에는 한글 글리프가 없어서
설정하지 않으면 한글이 전부 두부(□)로 나온다.
"""
import matplotlib
matplotlib.use("Agg")  # 화면 없이 파일로만 저장 (헤드리스 환경 대응)

import matplotlib.pyplot as plt
from matplotlib import font_manager


def use_korean_font() -> str:
    """사용 가능한 한글 폰트를 찾아 matplotlib 기본값으로 설정하고 이름을 반환."""
    available = {f.name for f in font_manager.fontManager.ttflist}
    # macOS 우선순위: 시스템 기본 한글 폰트 -> 나눔 -> 기타
    for cand in ("Apple SD Gothic Neo", "AppleGothic", "Nanum Gothic",
                 "Malgun Gothic", "Noto Sans CJK KR"):
        if cand in available:
            plt.rcParams["font.family"] = cand
            # 한글 폰트는 유니코드 마이너스(−)를 종종 지원하지 않아
            # 음수 부호가 깨진다. ASCII 하이픈을 쓰도록 강제.
            plt.rcParams["axes.unicode_minus"] = False
            return cand
    return plt.rcParams["font.family"][0]


FONT = use_korean_font()
