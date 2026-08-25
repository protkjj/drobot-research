"""2차 연구일지 HTML 생성 — 에너지 비용함수 정규화 전후 비교.

1차(research_log.html)는 "A* vs RRT* 어느 쪽인가"를 다뤘다.
이번 것은 그 위에서 "비용함수 형태를 바꾸면 결과가 어떻게 달라지는가"를
전후 비교로 다룬다.
"""
import base64
from pathlib import Path

HERE = Path(__file__).resolve().parent
RESULTS = HERE / "results"
OUT = HERE / "research_log2.html"


def b64(name: str) -> str:
    return base64.b64encode((RESULTS / name).read_bytes()).decode()


IMG_ABL = b64("fig_ablation.png")

HTML = f"""<title>비용함수 정규화 실험</title>
<style>
:root {{
  --ground:  #f7f7f5;
  --surface: #ffffff;
  --sunken:  #eeeeea;
  --ink:     #1a1a17;
  --muted:   #5f5f57;
  --faint:   #8d8d83;
  --rule:    #e0e0d8;
  --before:  #8a8a80;
  --after:   #2f7a55;
  --astar:   #2b6cb0;
  --rrt:     #c0561f;
  --warn:    #a8741a;

  --sans: "Apple SD Gothic Neo", "Pretendard", "Malgun Gothic",
          -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
  --mono: ui-monospace, SFMono-Regular, "SF Mono", Menlo, monospace;
  --measure: 66ch;
  --wide: 1000px;
}}
@media (prefers-color-scheme: dark) {{
  :root:not([data-theme="light"]) {{
    --ground:  #16161a;
    --surface: #1d1d22;
    --sunken:  #121216;
    --ink:     #e8e8e2;
    --muted:   #9d9d94;
    --faint:   #71716a;
    --rule:    #30302f;
    --before:  #7a7a72;
    --after:   #68d391;
    --astar:   #63b3ed;
    --rrt:     #ed8936;
    --warn:    #d9ad5c;
  }}
}}
:root[data-theme="dark"] {{
  --ground:  #16161a;
  --surface: #1d1d22;
  --sunken:  #121216;
  --ink:     #e8e8e2;
  --muted:   #9d9d94;
  --faint:   #71716a;
  --rule:    #30302f;
  --before:  #7a7a72;
  --after:   #68d391;
  --astar:   #63b3ed;
  --rrt:     #ed8936;
  --warn:    #d9ad5c;
}}

* {{ box-sizing: border-box; }}
body {{
  background: var(--ground);
  color: var(--ink);
  font-family: var(--sans);
  font-size: 16.5px;
  line-height: 1.75;
  margin: 0;
  padding: 0 20px 88px;
  -webkit-font-smoothing: antialiased;
}}
.wrap {{ max-width: var(--wide); margin: 0 auto; }}
.col {{ max-width: var(--measure); }}

header {{ padding: 64px 0 28px; border-bottom: 1px solid var(--rule); margin-bottom: 44px; }}
.eyebrow {{
  font-family: var(--mono); font-size: 11.5px; letter-spacing: 0.13em;
  text-transform: uppercase; color: var(--faint); margin: 0 0 16px;
}}
h1 {{
  font-size: clamp(28px, 4.2vw, 40px); line-height: 1.2; font-weight: 700;
  letter-spacing: -0.02em; margin: 0 0 18px; text-wrap: balance; max-width: 22ch;
}}
.standfirst {{ font-size: 17.5px; line-height: 1.62; color: var(--muted); margin: 0; max-width: 56ch; }}
.meta {{
  display: flex; flex-wrap: wrap; gap: 8px 24px; margin-top: 24px;
  font-family: var(--mono); font-size: 12.5px; color: var(--faint);
}}

/* 전후 대비 카드 */
.ba {{
  display: grid; gap: 1px; background: var(--rule);
  border: 1px solid var(--rule); border-radius: 3px; overflow: hidden;
  margin-bottom: 48px;
}}
@media (min-width: 700px) {{ .ba {{ grid-template-columns: 1fr 1fr; }} }}
.ba > div {{ background: var(--surface); padding: 22px 24px 24px; }}
.ba .tag {{
  font-family: var(--mono); font-size: 11px; letter-spacing: 0.1em;
  text-transform: uppercase; margin: 0 0 12px; font-weight: 600;
}}
.ba .before .tag {{ color: var(--before); }}
.ba .after .tag {{ color: var(--after); }}
.ba .formula {{
  font-family: var(--mono); font-size: 13px; line-height: 1.6;
  background: var(--sunken); padding: 11px 13px; border-radius: 2px;
  margin: 0 0 14px; overflow-x: auto; white-space: nowrap;
}}
.ba ul {{ margin: 0; padding-left: 1.2em; font-size: 14.5px; }}
.ba li {{ margin-bottom: 6px; color: var(--muted); }}
.ba li strong {{ color: var(--ink); }}

section {{ margin-bottom: 56px; }}
h2.sec {{
  font-size: 22px; font-weight: 700; letter-spacing: -0.015em;
  margin: 0 0 6px; display: flex; gap: 13px; align-items: baseline;
}}
h2.sec .num {{ font-family: var(--mono); font-size: 12.5px; color: var(--faint); font-weight: 400; flex: none; }}
.sec-note {{ font-size: 14.5px; color: var(--muted); margin: 0 0 24px 34px; max-width: 58ch; }}
h3 {{ font-size: 16.5px; font-weight: 700; margin: 30px 0 11px; }}
p {{ margin: 0 0 16px; }}
em {{ font-style: normal; background: color-mix(in srgb, var(--warn) 20%, transparent);
      padding: 0 3px; border-radius: 2px; }}

.tbl-wrap {{ overflow-x: auto; margin: 0 0 20px; }}
table {{ border-collapse: collapse; width: 100%; font-size: 14px; font-variant-numeric: tabular-nums; }}
th, td {{ text-align: right; padding: 9px 12px; border-bottom: 1px solid var(--rule); white-space: nowrap; }}
th:first-child, td:first-child {{ text-align: left; }}
thead th {{
  font-family: var(--mono); font-size: 11px; font-weight: 500; letter-spacing: 0.05em;
  text-transform: uppercase; color: var(--faint); border-bottom: 1px solid var(--ink);
}}
tbody tr:last-child td {{ border-bottom: none; }}
td.num {{ font-family: var(--mono); font-size: 13.5px; }}
.c-before {{ color: var(--before); }}
.c-after {{ color: var(--after); font-weight: 600; }}
.dim {{ color: var(--faint); }}

figure {{ margin: 0 0 24px; }}
figure img {{ width: 100%; height: auto; display: block; border: 1px solid var(--rule); border-radius: 3px; background: #fff; }}
figcaption {{ font-size: 13px; color: var(--muted); margin-top: 10px; line-height: 1.55; max-width: 72ch; }}

pre {{
  background: var(--sunken); border: 1px solid var(--rule); border-radius: 3px;
  padding: 14px 16px; overflow-x: auto; font-family: var(--mono);
  font-size: 13px; line-height: 1.6; margin: 0 0 18px; color: var(--ink);
}}
code {{ font-family: var(--mono); font-size: 0.925em; }}
p code, li code, td code {{ background: var(--sunken); padding: 1.5px 5px; border-radius: 2px; }}

.callout {{
  border-left: 2px solid var(--warn);
  background: color-mix(in srgb, var(--warn) 8%, transparent);
  padding: 15px 19px; border-radius: 0 3px 3px 0; margin: 0 0 22px;
}}
.callout p:last-child {{ margin-bottom: 0; }}
.callout .label {{
  font-family: var(--mono); font-size: 10.5px; letter-spacing: 0.1em;
  text-transform: uppercase; color: var(--warn); display: block; margin-bottom: 6px;
}}

ul, ol {{ margin: 0 0 16px; padding-left: 1.3em; }}
li {{ margin-bottom: 7px; }}
li::marker {{ color: var(--faint); }}
footer {{ border-top: 1px solid var(--rule); padding-top: 24px; margin-top: 16px; font-size: 13.5px; color: var(--muted); }}
a {{ color: var(--astar); text-underline-offset: 2px; }}
:focus-visible {{ outline: 2px solid var(--astar); outline-offset: 2px; border-radius: 2px; }}
@media (prefers-reduced-motion: reduce) {{ * {{ animation: none !important; transition: none !important; }} }}
</style>

<div class="wrap">

<header>
  <p class="eyebrow">연구일지 2 · drobot-research</p>
  <h1>에너지 비용함수<br>정규화의 효과</h1>
  <p class="standfirst">
    하이브리드 경로계획에서 최적해가 모드 전환을 선택하지 않는 현상이 관측됨.
    비용함수의 표현 형태를 변경하여 원인을 규명하고 그 효과를 정량 비교함.
  </p>
  <div class="meta">
    <span>2026-08-24</span>
    <span>맵 5종 · 시드 20 · 2.0초 제약</span>
    <span>선행 문서: A* vs RRT* 비교</span>
  </div>
</header>

<div class="ba">
  <div class="before">
    <p class="tag">변경 전 — 가중합 형태</p>
    <p class="formula">C = α·E_motion + β·E_switch + γ·T</p>
    <ul>
      <li>α=1.0, β=1.0, γ=0.5</li>
      <li><strong>에너지(Wh)와 시간(s)을 직접 합산</strong> — 단위 혼재</li>
      <li>가중치가 단위 변환 계수를 겸하여 실제 기여도 판단 불가</li>
      <li>최적해의 모드 전환 <strong>0회</strong></li>
    </ul>
  </div>
  <div class="after">
    <p class="tag">변경 후 — 정규화 형태</p>
    <p class="formula">C = wE·E/E_ref + wS·E_sw/E_sw_ref + wT·T/T_ref</p>
    <ul>
      <li>wE=0.5, wS=0.2, wT=0.3 (합 = 1.0)</li>
      <li><strong>세 항 모두 무차원</strong></li>
      <li>가중치가 곧 비중이므로 직접 해석 가능</li>
      <li>최적해의 모드 전환 <strong>4회</strong> (medium_open·hard_open 각 2회)</li>
    </ul>
  </div>
</div>

<div class="col">
<section>
  <h2 class="sec"><span class="num">01</span>배경 및 문제 제기</h2>
  <p class="sec-note">선행 실험에서 관측된 이상 현상의 원인 규명.</p>
  <p>
    A* 와 RRT* 를 비교하는 과정에서 <strong>최적해가 모드 전환(비행)을
    한 번도 선택하지 않는 현상</strong>이 관측됨. 대상 5개 맵 전체에서 동일하게 나타남.
  </p>
  <p>
    하이브리드 경로계획을 검증하는 실험에서 비행이 선택되지 않을 경우,
    실질적으로 2D 지상 경로계획 문제를 푸는 것과 동일하므로
    실험의 타당성이 성립하지 않음.
  </p>
  <p>원인 규명을 위해 다음을 순차적으로 시도하였음.</p>
  <div class="tbl-wrap">
  <table>
    <thead><tr><th>시도</th><th style="text-align:left">조치</th><th style="text-align:left">결과</th></tr></thead>
    <tbody>
      <tr><td class="num">1</td><td style="text-align:left">맵 재설계 (장애물을 좁고 길게)</td><td style="text-align:left" class="dim">비행 미선택</td></tr>
      <tr><td class="num">2</td><td style="text-align:left">우회거리를 손익분기(7.3~10.3m) 이상으로 확대</td><td style="text-align:left" class="dim">비행 미선택</td></tr>
      <tr><td class="num">3</td><td style="text-align:left">takeoff 에너지 sweep (0.1~5.0 Wh)</td><td style="text-align:left" class="dim">부분적 개선</td></tr>
      <tr><td class="num">4</td><td style="text-align:left"><strong>비용함수 표현 형태 변경</strong></td><td style="text-align:left" class="c-after"><strong>비행 선택됨</strong></td></tr>
    </tbody>
  </table>
  </div>
  <div class="callout">
    <span class="label">원인</span>
    <p>
      변경 전 계수(α=1.0, β=1.0, γ=0.5)를 정규화 형태로 환산한 결과,
      <strong>모드 전환 페널티가 전체 비용의 78.7%</strong>를 차지하고 있었음.
      에너지 항은 4.9%에 불과하여, 에너지 인식 경로계획이라는 설계 목적과
      실제 최적화 대상이 불일치하는 상태였음.
    </p>
  </div>
  <p>
    두 형태는 <code>α = wE/E_ref</code> 로 정확히 동등함. 즉 본 변경은
    <em>동일한 비용함수의 표현 방식을 바꾼 것</em>이며 최적화 대상 자체를
    변경한 것이 아님. 단위가 혼재된 표현에서는 이러한 불균형이
    계수만으로 드러나지 않음.
  </p>
</section>
</div>

<section>
  <div class="col">
    <h2 class="sec"><span class="num">02</span>실험 결과</h2>
    <p class="sec-note">에너지 파라미터는 동일하게 유지하고 가중치 표현만 변경하여 재측정함. 맵 5종, 시드 20개, 2.0초 제약.</p>
  </div>

  <figure>
    <img src="data:image/png;base64,{IMG_ABL}" alt="비용함수 정규화 전후 비교 4분할 그래프">
    <figcaption>
      ① 동일 계수를 정규화로 환산한 결과 전환 페널티가 78.7%임이 확인됨.
      ② 변경 전 모드 전환 0회, 변경 후 medium_open·hard_open 에서 각 2회 발생.
      ③ A* 는 두 조건 모두 5/5 우위이나 평균 마진은 3.09% → 2.08% 로 감소.
      ④ 비용 스케일이 상이하므로 두 조건의 절대값은 직접 비교 불가.
    </figcaption>
  </figure>

  <div class="tbl-wrap">
  <table>
    <thead><tr>
      <th>맵</th>
      <th>전 A*</th><th>전 전환</th><th>전 우위</th>
      <th>후 A*</th><th>후 전환</th><th>후 우위</th>
    </tr></thead>
    <tbody>
      <tr><td>easy_open</td>
        <td class="num c-before">44.506</td><td class="num dim">0</td><td class="num c-before">+0.11%</td>
        <td class="num c-after">16.289</td><td class="num dim">0</td><td class="num c-after">+0.88%</td></tr>
      <tr><td>easy_corridor</td>
        <td class="num c-before">47.744</td><td class="num dim">0</td><td class="num c-before">+2.13%</td>
        <td class="num c-after">17.628</td><td class="num dim">0</td><td class="num c-after">+2.08%</td></tr>
      <tr><td>medium_open</td>
        <td class="num c-before">72.088</td><td class="num dim">0</td><td class="num c-before">+4.74%</td>
        <td class="num c-after">25.846</td><td class="num c-after"><strong>2</strong></td><td class="num c-after">+2.70%</td></tr>
      <tr><td>medium_corridor</td>
        <td class="num c-before">78.381</td><td class="num dim">0</td><td class="num c-before">+3.35%</td>
        <td class="num c-after">29.356</td><td class="num dim">0</td><td class="num c-after">+1.81%</td></tr>
      <tr><td>hard_open</td>
        <td class="num c-before">79.407</td><td class="num dim">0</td><td class="num c-before">+5.14%</td>
        <td class="num c-after">30.012</td><td class="num c-after"><strong>2</strong></td><td class="num c-after">+2.91%</td></tr>
    </tbody>
  </table>
  </div>

  <div class="col">
  <p>
    <strong>A* 는 두 조건 모두에서 5/5 우위를 유지함.</strong>
    알고리즘 선택 결론은 불변임. 다만 평균 마진이
    <code>+3.09%</code> → <code>+2.08%</code> 로 감소함.
  </p>
  <p>
    비행이 선택되면서 탐색 문제의 난이도가 상승하였고, 이에 따라 A* 의
    격자 이산화 페널티가 상대적으로 드러난 것으로 해석됨. 즉
    <em>변경 전 조건의 큰 마진에는 문제 난이도가 낮았던 요인이 일부 포함</em>되어 있었음.
  </p>
  </div>
</section>

<div class="col">
<section>
  <h2 class="sec"><span class="num">03</span>고찰</h2>
  <p class="sec-note">변경으로 확보된 세 가지.</p>

  <h3>3.1 하이브리드 동작의 회복</h3>
  <p>
    변경 전에는 전 맵에서 비행이 선택되지 않았으나, 변경 후
    <code>medium_open</code> 및 <code>hard_open</code> 에서 각 2회
    (이륙 1 + 착륙 1)의 모드 전환이 발생함.
    해당 맵은 3D 상태공간에서 통과 불가능한 높이(0.95 m 초과) 장애물을
    포함하도록 설계된 맵이며, 설계 의도대로 비행이 선택된 것으로 확인됨.
  </p>

  <h3>3.2 가중치 해석 가능성</h3>
  <pre>에너지 우선   wE=0.7  wS=0.2  wT=0.1
균형형        wE=0.5  wS=0.2  wT=0.3   ← 현재
시간 우선     wE=0.3  wS=0.1  wT=0.6</pre>
  <p>
    가중치 합이 1이므로 임무 목적을 직접 표현할 수 있음.
    가중합 형태에서는 <code>β=1.0</code> 이 전체의 78.7%에 해당한다는 사실을
    별도 환산 없이는 확인할 수 없음.
  </p>

  <h3>3.3 실측 파라미터 변경에 대한 안정성</h3>
  <p>
    참조값을 상수로 고정하지 않고 모델 파라미터에서 유도하는 구조로 설계함.
  </p>
  <pre>E_ref        = ground_mode.energy_per_meter      (평지 1m 주행)
E_switch_ref = takeoff_energy + landing_energy   (전환 1회)
T_ref        = 1 / ground_mode.speed             (평지 1m 주행)</pre>
  <p>
    Phase 2 실측을 통해 에너지 파라미터를 교체하더라도 가중치의 의미가 보존됨.
    <code>wE = 0.5</code> 는 파라미터 변경 후에도 "평지 1 m 주행 대비 기준"을 유지함.
  </p>
</section>

<section>
  <h2 class="sec"><span class="num">04</span>한계</h2>
  <p class="sec-note">본 실험이 보장하지 않는 범위.</p>

  <h3>4.1 비용 절대값 비교 불가</h3>
  <p>
    변경 전 44.506 과 변경 후 16.289 는 스케일이 상이하므로 직접 비교할 수 없음.
    비교 가능한 지표는 동일 조건 내의 상대값(A* 우위 %)과
    경로 특성(모드 전환 횟수)에 한정됨.
  </p>

  <h3>4.2 가중치 선정의 근거 부족</h3>
  <p>
    현재 가중치(0.5 / 0.2 / 0.3)는 균형형이라는 명목으로 설정한 값이며,
    실측 또는 최적화를 통해 도출한 값이 아님.
    향후 JMP 를 이용하여 <code>wE, wS, wT</code> 를 sweep 하고
    total_energy / total_time / switch_count 를 desirability 로 평가하면
    임무별 적정 가중치를 근거 있게 결정할 수 있을 것으로 판단됨.
  </p>

  <h3>4.3 에너지 파라미터 미실측</h3>
  <p>
    <code>takeoff_energy = 5.0 Wh</code> 는 동일 config 의
    <code>hover_power = 50 W</code>, <code>takeoff_time = 5 s</code> 로부터
    산출한 값(50 W × 5 s = 0.069 Wh)의 <em>약 72배</em>에 해당함.
    비용함수의 표현 형태를 개선하더라도 파라미터 자체가 부정확할 경우
    결론의 타당성이 확보되지 않음.
    <strong>Phase 2 실측의 최우선 대상임.</strong>
  </p>
</section>

<section>
  <h2 class="sec"><span class="num">05</span>결론</h2>
  <p class="sec-note">네 가지로 정리됨.</p>
  <ol>
    <li>
      비용함수를 정규화 형태로 변경한 결과, 기존 설정이 모드 전환에
      전체 비용의 <strong>78.7%</strong>를 배분하고 있었음이 확인됨.
      이는 최적해가 비행을 선택하지 않은 원인으로 판단됨.
    </li>
    <li>
      가중치를 0.5 / 0.2 / 0.3 으로 재설정한 후 모드 전환이 정상적으로
      선택되었으며, 하이브리드 경로계획 실험의 타당성이 확보됨.
    </li>
    <li>
      A* 채택 결론은 두 조건에서 모두 유지됨. 다만 평균 마진이
      +3.09%에서 +2.08%로 감소하였으며, 이는 A* 채택 근거가 해 품질보다
      최적성 보장·재현성·시간 여유에 있다는 선행 판단을 뒷받침함.
    </li>
    <li>
      단위가 혼재된 비용함수는 가중치의 실제 기여도를 은폐함.
      본 사례에서 이상 결과의 원인 진단이 지연된 주된 요인이었음.
    </li>
  </ol>
</section>

<footer>
  <p>
    재현 절차: <code>python3 benchmark/exp_costfn_ablation.py</code> →
    <code>results/benchmark_costfn_ablation.json</code> 생성,
    <code>python3 benchmark/plot_ablation.py</code> → 그림 생성.
    선행 문서는 <code>benchmark/RESEARCH_LOG.md</code>.
  </p>
</footer>

</div>
"""

OUT.write_text(HTML, encoding="utf-8")
print(f"저장: {OUT}  ({len(HTML)/1024:.0f} KB)")
