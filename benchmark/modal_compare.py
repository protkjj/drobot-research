"""3 modal 결과 비교 유틸 — 상위집합 관계 강제와 동률 처리.

왜 이 모듈이 필요한가
---------------------
세 modal 은 완전히 독립적이지 않고 포함 관계가 있다.

    rover_detour  우회만
    rover_climb   우회 + 밟고넘기   ⊇ rover_detour
    hybrid        우회 + 비행       ⊇ rover_detour

'밟고 넘을 수 있는 로봇'은 밟지 않는 경로도 그대로 갈 수 있다.
따라서 rover_climb 의 최적 비용은 rover_detour 보다 클 수 없다.
hybrid 도 마찬가지다.

그런데 실제로 위반이 나왔다. 원인을 단계별로 분리해 확인한 결과:

    맵           계수   A* 격자해                    스무딩 후
    easy_open    5.0   밟고넘기 17.148 <= 우회 17.195   밟고넘기 17.077 > 우회 16.289  ★
    medium_open  12.0  밟고넘기 28.146 <= 우회 28.240   밟고넘기 27.472 > 우회 26.617  ★
    hard_open    8.0   밟고넘기 30.937 <= 우회 31.031   밟고넘기 30.242 > 우회 29.479  ★

A* 격자해는 전부 정상이다 (상위집합이 항상 더 싸다).
역전은 스무딩 단계에서만 생긴다. 우회 경로는 5.0~5.8% 개선되는데
밟고넘기 경로는 0.4~2.4% 밖에 개선되지 않기 때문이다.
밟고넘기 경로는 장애물을 가로질러 이미 짧아서 shortcut 여지가 적고,
우회 경로는 길고 지그재그가 많아 개선 폭이 크다.
shortcut smoothing 은 그리디 휴리스틱이라 최적을 보장하지 않으므로
이런 순위 뒤집힘이 생길 수 있다.

해결
----
상위집합 modal 의 비용을 하위집합 modal 의 비용으로 상한을 씌운다.
이는 결과를 유리하게 조작하는 게 아니라, 그 modal 이 '실제로 실행 가능한
경로' 중 더 싼 것을 택하는 것이다 — 물리적으로 정당하다.

한계
----
근본 원인(스무더가 비최적)은 남아 있다. 이 보정은 modal 간 포함 관계가
있는 경우만 바로잡을 뿐, 같은 modal 안에서 스무딩이 놓친 개선은 못 찾는다.
논문에 쓸 때는 '경로 후처리로 shortcut smoothing 을 쓰며 최적은 아니다'를
명시하는 편이 정직하다.
"""
from __future__ import annotations

MODALS = ["rover_detour", "rover_climb", "hybrid"]

KR = {
    "rover_detour": "우회",
    "rover_climb": "밟고넘기",
    "hybrid": "하이브리드",
}

# 상위집합 -> 그 안에 포함되는 modal 목록.
# 상위집합은 하위집합의 경로를 그대로 실행할 수 있다.
SUPERSET_OF: dict[str, list[str]] = {
    "rover_climb": ["rover_detour"],
    "hybrid": ["rover_detour"],
}

TOL = 1e-9


def enforce_superset(per_modal: dict[str, dict | None]) -> dict[str, dict | None]:
    """상위집합 modal 이 하위집합보다 비싸게 나온 경우를 바로잡는다.

    per_modal: {modal 이름: 결과 dict(최소한 'cost' 키를 가짐) 또는 None}
    반환값은 새 dict 이며, 보정된 항목에는 'corrected_from' 키가 붙는다
    (어느 modal 의 결과를 빌려왔는지 기록 — 나중에 몇 건이 보정됐는지 셀 수 있다).
    """
    out = dict(per_modal)
    for sup, subs in SUPERSET_OF.items():
        cur = out.get(sup)
        for sub in subs:
            other = out.get(sub)
            if other is None:
                continue
            # 상위집합이 해를 못 찾았거나 더 비싸면 하위집합 결과를 쓴다
            if cur is None or other["cost"] < cur["cost"] - TOL:
                borrowed = dict(other)
                borrowed["corrected_from"] = sub
                out[sup] = borrowed
                cur = borrowed
    return out


def pick_winner(per_modal: dict[str, dict | None]) -> tuple[str | None, bool]:
    """(승자 modal, 동률여부) 를 돌려준다.

    동률을 따로 표시하는 이유:
        min() 은 값이 같으면 dict 순서상 앞선 것을 고른다.
        가중치 축퇴 조합(예: wE=0, wT=0 이면 지상 경로가 전부 비용 0)에서
        이게 '우회가 이겼다'로 잘못 집계된다. 실제로 sweep 330행 중
        21행이 동률이었고, 그중 5행은 완전 축퇴였다.
    """
    valid = {k: v for k, v in per_modal.items() if v is not None}
    if not valid:
        return None, False
    best = min(v["cost"] for v in valid.values())
    winners = [k for k, v in valid.items() if v["cost"] <= best + TOL]
    return winners[0], len(winners) > 1
