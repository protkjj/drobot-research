"""연구일지 HTML 생성.

그래프 PNG를 base64로 인라인해서 자체완결 페이지를 만든다.
(Artifact는 외부 리소스를 CSP로 차단하므로 이미지를 링크할 수 없다)
"""
import base64
from pathlib import Path

HERE = Path(__file__).resolve().parent
RESULTS = HERE / "results"
OUT = HERE / "research_log.html"


def b64(name: str) -> str:
    return base64.b64encode((RESULTS / name).read_bytes()).decode()


IMG_SUMMARY = b64("fig_summary.png")
IMG_MAPS = b64("maps.png")

HTML = f"""<title>Drobot 플래너 벤치마크</title>
<style>
/* ---------- 토큰: 라이트 팔레트를 bare :root 에 전부 정의 ---------- */
:root {{
  --ground:  #f6f7f8;
  --surface: #ffffff;
  --sunken:  #eef1f4;
  --ink:     #171c23;
  --muted:   #5c6773;
  --faint:   #8b95a1;
  --rule:    #dee3e8;
  --astar:   #2b6cb0;
  --rrt:     #c05621;
  --warn:    #b7791f;
  --ok:      #2f7a55;

  --fs-body: 16.5px;
  --lh-body: 1.72;
  --measure: 68ch;
  --wide: 1040px;

  --sans: "Apple SD Gothic Neo", "Pretendard", "Malgun Gothic",
          -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
  --mono: ui-monospace, SFMono-Regular, "SF Mono", Menlo, Consolas, monospace;
}}

/* 시스템 다크 (명시적 라이트 선택이 이기도록 guard) */
@media (prefers-color-scheme: dark) {{
  :root:not([data-theme="light"]) {{
    --ground:  #13171c;
    --surface: #1a1f26;
    --sunken:  #10141a;
    --ink:     #e4e8ed;
    --muted:   #98a3b1;
    --faint:   #6d7885;
    --rule:    #2b323b;
    --astar:   #63b3ed;
    --rrt:     #ed8936;
    --warn:    #e0b355;
    --ok:      #68d391;
  }}
}}
/* 명시적 다크 선택 */
:root[data-theme="dark"] {{
  --ground:  #13171c;
  --surface: #1a1f26;
  --sunken:  #10141a;
  --ink:     #e4e8ed;
  --muted:   #98a3b1;
  --faint:   #6d7885;
  --rule:    #2b323b;
  --astar:   #63b3ed;
  --rrt:     #ed8936;
  --warn:    #e0b355;
  --ok:      #68d391;
}}

* {{ box-sizing: border-box; }}

body {{
  background: var(--ground);
  color: var(--ink);
  font-family: var(--sans);
  font-size: var(--fs-body);
  line-height: var(--lh-body);
  margin: 0;
  padding: 0 20px 96px;
  -webkit-font-smoothing: antialiased;
}}

.wrap {{ max-width: var(--wide); margin: 0 auto; }}
.col  {{ max-width: var(--measure); }}

/* ---------- 헤더 ---------- */
header {{
  padding: 68px 0 30px;
  border-bottom: 1px solid var(--rule);
  margin-bottom: 40px;
}}
.eyebrow {{
  font-family: var(--mono);
  font-size: 11.5px;
  letter-spacing: 0.13em;
  text-transform: uppercase;
  color: var(--faint);
  margin: 0 0 18px;
}}
h1 {{
  font-size: clamp(30px, 4.4vw, 43px);
  line-height: 1.16;
  letter-spacing: -0.02em;
  font-weight: 700;
  margin: 0 0 20px;
  text-wrap: balance;
  max-width: 20ch;
}}
h1 .vs {{ color: var(--faint); font-weight: 400; }}
.standfirst {{
  font-size: 18px;
  line-height: 1.62;
  color: var(--muted);
  margin: 0;
  max-width: 58ch;
}}
.meta {{
  display: flex; flex-wrap: wrap; gap: 8px 26px;
  margin-top: 26px;
  font-family: var(--mono);
  font-size: 12.5px;
  color: var(--faint);
}}

/* ---------- 결론 카드 (맨 위) ---------- */
.verdict {{
  background: var(--surface);
  border: 1px solid var(--rule);
  border-radius: 3px;
  padding: 28px 30px 26px;
  margin-bottom: 52px;
}}
.verdict h2 {{
  font-size: 12px; font-family: var(--mono);
  letter-spacing: 0.13em; text-transform: uppercase;
  color: var(--faint); margin: 0 0 18px; font-weight: 500;
}}
.headline-answer {{
  font-size: 25px; font-weight: 700; line-height: 1.35;
  letter-spacing: -0.015em; margin: 0 0 8px;
}}
.headline-answer .pick {{ color: var(--astar); }}
.qualifier {{ font-size: 15.5px; color: var(--muted); margin: 0 0 26px; }}

.branches {{
  display: grid; gap: 1px;
  background: var(--rule);
  border: 1px solid var(--rule);
  border-radius: 2px;
  overflow: hidden;
}}
@media (min-width: 720px) {{ .branches {{ grid-template-columns: repeat(3, 1fr); }} }}
.branch {{ background: var(--surface); padding: 17px 19px 18px; }}
.branch .cond {{
  font-family: var(--mono); font-size: 11.5px; color: var(--faint);
  letter-spacing: 0.04em; margin: 0 0 9px;
}}
.branch .pickline {{
  font-size: 16.5px; font-weight: 700; margin: 0 0 7px;
  display: flex; align-items: baseline; gap: 8px;
}}
.branch .why {{ font-size: 13.5px; color: var(--muted); line-height: 1.55; margin: 0; }}
.pick-a {{ color: var(--astar); }}
.pick-r {{ color: var(--rrt); }}

/* ---------- 섹션 ---------- */
section {{ margin-bottom: 62px; }}
h2.sec {{
  font-size: 23px; font-weight: 700; letter-spacing: -0.015em;
  margin: 0 0 6px; display: flex; gap: 14px; align-items: baseline;
}}
h2.sec .num {{
  font-family: var(--mono); font-size: 13px; color: var(--faint);
  font-weight: 400; flex: none;
}}
.sec-note {{
  font-size: 14.5px; color: var(--muted); margin: 0 0 26px 36px;
  max-width: 60ch;
}}
h3 {{
  font-size: 16.5px; font-weight: 700; margin: 34px 0 12px;
  letter-spacing: -0.005em;
}}
p {{ margin: 0 0 17px; }}
p.tight {{ margin-bottom: 10px; }}
strong {{ font-weight: 700; }}
em {{ font-style: normal; background: color-mix(in srgb, var(--warn) 17%, transparent);
      padding: 0 3px; border-radius: 2px; }}

/* ---------- 표 ---------- */
.tbl-wrap {{ overflow-x: auto; margin: 0 0 22px; }}
table {{
  border-collapse: collapse; width: 100%; font-size: 14px;
  font-variant-numeric: tabular-nums;
}}
th, td {{
  text-align: right; padding: 9px 13px;
  border-bottom: 1px solid var(--rule); white-space: nowrap;
}}
th:first-child, td:first-child {{ text-align: left; }}
thead th {{
  font-family: var(--mono); font-size: 11.5px; font-weight: 500;
  letter-spacing: 0.05em; text-transform: uppercase;
  color: var(--faint); border-bottom: 1px solid var(--ink);
}}
tbody tr:last-child td {{ border-bottom: none; }}
td.num {{ font-family: var(--mono); font-size: 13.5px; }}
td.win-a {{ color: var(--astar); font-weight: 700; }}
td.win-r {{ color: var(--rrt); font-weight: 700; }}
.dim {{ color: var(--faint); }}

/* ---------- 그림 ---------- */
figure {{ margin: 0 0 26px; }}
figure img {{
  width: 100%; height: auto; display: block;
  border: 1px solid var(--rule); border-radius: 3px;
  background: #fff;
}}
figcaption {{
  font-size: 13px; color: var(--muted); margin-top: 11px;
  line-height: 1.55; max-width: 74ch;
}}

/* ---------- 코드 ---------- */
pre {{
  background: var(--sunken); border: 1px solid var(--rule);
  border-radius: 3px; padding: 15px 17px; overflow-x: auto;
  font-family: var(--mono); font-size: 13px; line-height: 1.62;
  margin: 0 0 20px; color: var(--ink);
}}
code {{ font-family: var(--mono); font-size: 0.925em; }}
p code, li code, td code {{
  background: var(--sunken); padding: 1.5px 5px; border-radius: 2px;
}}

/* ---------- 강조 박스 ---------- */
.callout {{
  border-left: 2px solid var(--warn);
  background: color-mix(in srgb, var(--warn) 7%, transparent);
  padding: 16px 20px; border-radius: 0 3px 3px 0; margin: 0 0 24px;
}}
.callout p:last-child {{ margin-bottom: 0; }}
.callout .label {{
  font-family: var(--mono); font-size: 11px; letter-spacing: 0.1em;
  text-transform: uppercase; color: var(--warn); display: block; margin-bottom: 7px;
}}

/* ---------- 리스트 ---------- */
ul, ol {{ margin: 0 0 18px; padding-left: 1.35em; }}
li {{ margin-bottom: 8px; }}
li::marker {{ color: var(--faint); }}

/* ---------- 푸터 ---------- */
footer {{
  border-top: 1px solid var(--rule); padding-top: 26px; margin-top: 20px;
  font-size: 13.5px; color: var(--muted);
}}

a {{ color: var(--astar); text-underline-offset: 2px; }}
a:focus-visible, :focus-visible {{
  outline: 2px solid var(--astar); outline-offset: 2px; border-radius: 2px;
}}
@media (prefers-reduced-motion: reduce) {{
  * {{ animation: none !important; transition: none !important; }}
}}
</style>

<div class="wrap">

<header>
  <p class="eyebrow">연구일지 · drobot-research</p>
  <h1>A* <span class="vs">vs</span> RRT*,<br>우리 문제에서는 어느 쪽인가</h1>
  <p class="standfirst">
    GPS 없는 실내에서 주행과 비행을 함께 계획하는 하이브리드 플래너.
    패키지 이름은 이미 <code>hybrid_rrt_star_planner</code>로 정해져 있었지만,
    그 선택의 근거는 어디에도 없었다. 구현 전에 실제로 재봤다.
  </p>
  <div class="meta">
    <span>2026-08-23</span>
    <span>ROS 2 Jazzy / Python 벤치마크</span>
    <span>맵 6종 · 시드 20 · 2.0초 제약</span>
  </div>
</header>

<div class="verdict">
  <h2>결론</h2>
  <p class="headline-answer">현재 설계에서는 <span class="pick">A*</span>를 쓴다.</p>
  <p class="qualifier">
    다만 무조건이 아니다. 조건이 셋으로 갈린다.
  </p>
  <div class="branches">
    <div class="branch">
      <p class="cond">3D (x, y, mode) — 현재 설계</p>
      <p class="pickline"><span class="pick-a">A*</span></p>
      <p class="why">0.12~1.00초에 최적해. 결정론적이고 2초 제약에 2~16배 여유.</p>
    </div>
    <div class="branch">
      <p class="cond">비행이 활발한 조건</p>
      <p class="pickline"><span class="pick-r">RRT*</span> <span class="dim" style="font-size:13px;font-weight:400">근소</span></p>
      <p class="why">해 품질 1~3% 우위. A*도 격자를 촘촘히 하면 따라잡지만 시간 제약을 넘긴다.</p>
    </div>
    <div class="branch">
      <p class="cond">4D (x, y, z, mode) 확장 시</p>
      <p class="pickline"><span class="pick-r">RRT*</span></p>
      <p class="why">A*는 가장 거친 해상도에서도 전부 timeout. 선택지가 아니다.</p>
    </div>
  </div>
</div>

<div class="col">

<section>
  <h2 class="sec"><span class="num">01</span>왜 이걸 먼저 했나</h2>
  <p class="sec-note">구현을 시작하면 되돌리기 어려워진다.</p>
  <p>
    경로계획의 표준은 A*다. 심사에서 <strong>"왜 굳이 RRT*냐"</strong>는 질문은
    반드시 나온다. 그런데 저장소에는 답이 없었다. 패키지 이름과 config만 있고
    근거가 없는 상태였다.
  </p>
  <p>
    그래서 헤더 파일 한 줄을 더 쓰기 전에 두 알고리즘을 실제로 비교하기로 했다.
    전제는 하나였다 — <em>결과가 어느 쪽으로 나오든 그대로 쓴다.</em>
    RRT*를 정해놓고 뒷받침 숫자만 찾으면 나중에 더 아프게 돌아온다.
  </p>
</section>

<section>
  <h2 class="sec"><span class="num">02</span>어떻게 쟀나</h2>
  <p class="sec-note">공정성이 이 실험의 전부다.</p>
  <p>
    ROS2/Gazebo 없이 Python 독립 벤치마크로 만들었다. 물리 시뮬레이션은
    알고리즘 비교에 불필요하고, 결정론적이라 재현이 보장된다.
  </p>
  <div class="tbl-wrap">
  <table>
    <thead><tr><th>항목</th><th style="text-align:left">내용</th></tr></thead>
    <tbody>
      <tr><td>상태공간</td><td style="text-align:left">3D <code>(x, y, mode)</code> / 4D <code>(x, y, z, mode)</code></td></tr>
      <tr><td>비용함수</td><td style="text-align:left"><code>C = α·E_motion + β·E_switch + γ·T</code></td></tr>
      <tr><td>맵</td><td style="text-align:left">{{easy, medium, hard}} × {{open, corridor}} · 20×14m ~ 24×16m</td></tr>
      <tr><td>시간 제약</td><td style="text-align:left">2.0초 (<code>hybrid_rrt_params.yaml</code>의 timeout)</td></tr>
      <tr><td>비교 대상</td><td style="text-align:left">Dijkstra(참조 최적해) · A* · Informed RRT*</td></tr>
    </tbody>
  </table>
  </div>
  <p>
    <strong>비용 계산은 <code>cost/energy.py</code> 한 곳에만 두고</strong> 세 플래너가 공유한다.
    플래너마다 비용 함수가 다르면 성능 차이가 알고리즘 때문인지 문제 정의 때문인지
    구분할 수 없게 된다.
  </p>
  <p>
    목적이 중간에 "A* 채택 근거 확보"로 바뀐 뒤에도 이 원칙은 유지했다.
    오히려 RRT*를 <strong>더 강하게</strong> 만들었다 — Informed 샘플링,
    k-nearest rewire, 공간 해싱, 시드 20개 중 최선. 약한 상대를 이기면 아무도 믿지 않는다.
  </p>
</section>
</div>

<section>
  <div class="col">
    <h2 class="sec"><span class="num">03</span>결과</h2>
    <p class="sec-note">네 장의 그림이 전부를 요약한다.</p>
  </div>

  <figure>
    <img src="data:image/png;base64,{IMG_SUMMARY}" alt="벤치마크 종합 결과 4분할 그래프">
    <figcaption>
      ① 현재 config에서는 A*가 5/5 우위. ② 그러나 takeoff 에너지를 낮춰 비행 유인을
      키우면 역전된다. ③ A*의 열세는 이/착륙 지점이 격자에 묶인 탓이라
      해상도를 높이면 따라잡는다. ④ 다만 그 해상도에서는 2초 제약을 위반한다.
    </figcaption>
  </figure>

  <div class="col">
  <h3>3-1. 현재 config에서는 A*가 우위 (5/5)</h3>
  </div>
  <div class="tbl-wrap">
  <table>
    <thead><tr>
      <th>맵</th><th>A*</th><th>A* 시간</th><th>RRT* 평균</th><th>RRT* σ</th><th>A* 우위</th>
    </tr></thead>
    <tbody>
      <tr><td>easy_open</td><td class="num win-a">44.276</td><td class="num">0.12s</td><td class="num">44.278</td><td class="num dim">0.250</td><td class="num">+0.00%</td></tr>
      <tr><td>easy_corridor</td><td class="num win-a">47.744</td><td class="num">0.12s</td><td class="num">48.473</td><td class="num dim">0.305</td><td class="num">+1.53%</td></tr>
      <tr><td>medium_open</td><td class="num win-a">72.088</td><td class="num">0.81s</td><td class="num">74.158</td><td class="num dim">0.552</td><td class="num">+2.87%</td></tr>
      <tr><td>medium_corridor</td><td class="num win-a">79.256</td><td class="num">0.95s</td><td class="num">79.502</td><td class="num dim">0.627</td><td class="num">+0.31%</td></tr>
      <tr><td>hard_open</td><td class="num win-a">79.407</td><td class="num">1.00s</td><td class="num">81.671</td><td class="num dim">0.726</td><td class="num">+2.85%</td></tr>
    </tbody>
  </table>
  </div>

  <div class="col">
  <p>
    A*는 <strong>최적해</strong>를 낸다 — Dijkstra 전수탐색과 비용이 완전히 일치함을
    검증했다. RRT*는 같은 시간에 근사해이고, 실행할 때마다 다르다 (σ = 0.25~0.73).
  </p>

  <h3>3-2. 그런데 비행이 활발해지면 역전된다</h3>
  <p>
    <code>takeoff</code> 에너지를 낮춰 비행 유인을 키우면서 다시 쟀다.
  </p>
  </div>

  <div class="tbl-wrap">
  <table>
    <thead><tr>
      <th>takeoff</th><th>A* 모드전환</th><th>A* 우위 맵수</th><th style="text-align:left">medium_open 기준</th>
    </tr></thead>
    <tbody>
      <tr><td class="num">0.1 Wh</td><td class="num">1.33회</td><td class="num">2/3</td><td style="text-align:left" class="win-r">RRT* +3.06%</td></tr>
      <tr><td class="num">0.3 Wh</td><td class="num">1.33회</td><td class="num win-r">1/3</td><td style="text-align:left" class="win-r">RRT* +3.08%</td></tr>
      <tr><td class="num">0.7 Wh</td><td class="num">1.33회</td><td class="num win-r">1/3</td><td style="text-align:left" class="win-r">RRT* +3.16%</td></tr>
      <tr><td class="num">1.5 Wh</td><td class="num">1.33회</td><td class="num win-r">1/3</td><td style="text-align:left" class="win-r">RRT* +2.95%</td></tr>
      <tr><td class="num">3.0 Wh</td><td class="num">0.67회</td><td class="num">2/3</td><td style="text-align:left" class="win-r">RRT* +3.18%</td></tr>
      <tr><td class="num"><strong>5.0 Wh</strong> <span class="dim">현재</span></td><td class="num"><strong>0회</strong></td><td class="num win-a">3/3</td><td style="text-align:left" class="win-a">A* +2.74%</td></tr>
    </tbody>
  </table>
  </div>

  <div class="col">
  <div class="callout">
    <span class="label">핵심</span>
    <p>
      3-1의 "A* 5/5 승"은 <strong>현재 config가 비행을 구조적으로 배제한 결과였다.</strong>
      비행이 0회면 사실상 2D 지상 경로계획 문제이고, 거기서는 A*가 강하다.
    </p>
  </div>

  <h3>3-3. 4D에서는 A*를 쓸 수 없다</h3>
  <p>
    고도 <code>z</code>를 최적화 변수로 올리면 상태공간이 25만~277만 노드가 된다.
    양쪽 모두 2초로 묶고 재면:
  </p>
  <pre>맵                z해상도 전부      RRT*
easy_open         44.276           <strong>44.050</strong>
medium_open       <strong>timeout</strong>          73.017
medium_corridor   <strong>timeout</strong>          78.102
hard_open         <strong>timeout</strong>          81.228
hard_corridor     <strong>timeout</strong>          해 없음</pre>
  <p>
    가장 거친 z해상도(0.5m)에서도 A*는 3.75~4.97초가 걸린다.
    다만 4D가 실제로 필요한 경우가 있다 — <code>hard_corridor</code>는
    <strong>3D로는 아예 풀리지 않는다.</strong> 3D는 비행 고도가
    <code>장애물높이 + 0.8m</code>로 고정이라, 천장 2.5m 아래에서
    0.95m 넘는 장애물을 넘지 못하기 때문이다. 4D는 1.55m까지 넘는다.
  </p>
  </div>
</section>

<section>
  <div class="col">
    <h2 class="sec"><span class="num">04</span>실험 환경</h2>
    <p class="sec-note">맵은 손익분기 계산에 맞춰 두 번 설계했다.</p>
  </div>
  <figure>
    <img src="data:image/png;base64,{IMG_MAPS}" alt="실험 맵 6종 높이맵">
    <figcaption>
      1차 설계는 폐기했다. 최적해가 6개 맵 전부에서 비행을 0회 선택했기 때문이다.
      원인은 계산으로 규명했다 — 비행 1회 고정비가 지상 주행 6.4m에 해당하는데
      장애물 폭이 1m라 우회거리가 2~3m뿐이었다. 2차는 손익분기 우회거리(7.3~10.3m)에
      맞춰 '좁고 긴 벽' 구조로 다시 만들었다.
    </figcaption>
  </figure>
</section>

<div class="col">
<section>
  <h2 class="sec"><span class="num">05</span>왜 A*인가 — 해 품질이 아니다</h2>
  <p class="sec-note">1~3% 차이로 알고리즘을 고르는 건 설득력이 약하다.</p>
  <p>
    측정된 비용 차이는 1~3%다. 그런데 이 연구의 에너지 모델은 아직 실측 전이고,
    <code>takeoff_energy = 5.0 Wh</code>는 같은 config의 <code>hover_power = 50W</code>로
    계산한 값(50W × 5s = 0.069 Wh)의 <em>72배</em>다.
  </p>
  <p>
    파라미터가 수십 배 불확실한 상태에서 3% 최적화 차이를 근거로
    알고리즘을 고르는 건 무리다. <strong>A*를 택하는 진짜 이유는 이쪽이다.</strong>
  </p>
  <div class="tbl-wrap">
  <table>
    <thead><tr><th>이유</th><th style="text-align:left">근거</th></tr></thead>
    <tbody>
      <tr><td>최적성 보장</td><td style="text-align:left">Dijkstra 전수탐색과 비용 완전 일치 검증</td></tr>
      <tr><td>결정론적 재현성</td><td style="text-align:left">σ=0. RRT*는 σ=0.25~0.73으로 매 실행 다른 경로</td></tr>
      <tr><td>시간 여유</td><td style="text-align:left">0.12~1.00초 vs 제약 2.0초 (2~16배)</td></tr>
      <tr><td>튜닝 부담</td><td style="text-align:left">휴리스틱 하나. RRT*는 goal_bias·gamma_rrt·step_size 등 다수</td></tr>
    </tbody>
  </table>
  </div>
  <p>
    로봇 시스템에서 "같은 상황에 항상 같은 경로"는 디버깅·재현·인증에서 큰 값어치가 있다.
    <em>"어제는 됐는데 오늘은 안 되네"</em>의 원인을 하나 줄인다.
  </p>
</section>

<section>
  <h2 class="sec"><span class="num">06</span>이 결론을 믿어도 되는 이유</h2>
  <p class="sec-note">내 구현의 버그 5개를 고쳤고, 각각이 결론을 뒤집을 수 있었다.</p>
</section>
</div>

<div class="tbl-wrap">
<table>
  <thead><tr>
    <th>#</th><th style="text-align:left">버그</th><th style="text-align:left">증상</th><th style="text-align:left">영향</th>
  </tr></thead>
  <tbody>
    <tr><td class="num">1</td><td style="text-align:left">A* corner-cutting</td>
      <td style="text-align:left">대각 이동이 벽 모서리를 관통. 높이 1.10m 벽을 통과하는 경로를 "최적해"로 냄</td>
      <td style="text-align:left" class="win-a">A*에 부당하게 유리</td></tr>
    <tr><td class="num">2</td><td style="text-align:left">격자 각도 페널티</td>
      <td style="text-align:left">A*만 45° 배수로 제한되고 RRT*는 연속 공간</td>
      <td style="text-align:left"><strong>RRT*가 5.3% 앞서 보임 — 결론이 정반대</strong></td></tr>
    <tr><td class="num">3</td><td style="text-align:left">RRT* O(n²)</td>
      <td style="text-align:left">rewire 반경 하한이 반경 축소 성질을 파괴</td>
      <td style="text-align:left" class="dim">4만 샘플에 34초 → 40배 가속</td></tr>
    <tr><td class="num">4</td><td style="text-align:left">RRT* 착륙 노드 미생성</td>
      <td style="text-align:left">비행 후 지상 복귀 불가 (이륙 1, 착륙 0). 하이브리드 문제 0% 성공</td>
      <td style="text-align:left" class="win-r">RRT*에 부당하게 불리</td></tr>
    <tr><td class="num">5</td><td style="text-align:left">상승각의 z해상도 종속</td>
      <td style="text-align:left">고도 변경을 격자 1칸으로 제한 → z=0.10에서 45°, z=0.05에서 26.6°</td>
      <td style="text-align:left" class="dim">해상도 높일수록 해가 나빠짐</td></tr>
  </tbody>
</table>
</div>

<div class="col">
  <p>
    2번을 안 고쳤으면 "RRT*가 낫다"로 결론냈을 것이고, 4번을 안 고쳤으면
    "RRT*는 하이브리드 문제를 못 푼다"는 틀린 주장을 했을 것이다.
  </p>
  <p>
    <strong>"A*가 이겼다"는 결과는 이 다섯을 다 고쳐 공정한 조건을 만든 뒤에도
    유지된 것</strong>이다. 그게 이 결론을 신뢰할 수 있는 근거다.
  </p>
</div>

<div class="col">
<section>
  <h2 class="sec"><span class="num">07</span>한계와 다음</h2>
  <p class="sec-note">기각된 가설도 함께 기록한다.</p>

  <h3>측정하지 못한 것</h3>
  <ul>
    <li>실제 ROS2/Nav2 환경에서의 성능 — 전부 Python 독립 실행이다</li>
    <li>동적 장애물과 재계획 상황 — 정적 맵 1회 계획만 쟀다</li>
    <li>실측 에너지 파라미터 — 전부 <code>energy_params.yaml</code>의 추정값 기반이다</li>
  </ul>

  <h3>기각된 가설</h3>
  <p>
    <strong>"좁은 통로에서 샘플링 기반이 실패한다"</strong> — 성립하지 않았다.
    통로를 0.4m까지 좁혀도 RRT*가 20/20 성공했다.
    <code>hard_corridor</code>에서 RRT*가 실패한 건 통로가 좁아서가 아니라
    모드 전환 시퀀스가 필요해서였고, 그마저 버그 4를 고친 뒤 대부분 해소됐다.
  </p>

  <h3>다음 단계</h3>
  <ol>
    <li><code>hybrid_rrt_star_planner</code> → <code>hybrid_astar_planner</code> 이름 변경.
        헤더가 아직 6줄뿐이라 지금이 바꾸기 쉽다</li>
    <li>Nav2의 <code>SmacPlanner2D</code> 구조를 참고해 C++ 구현.
        단 <strong>하이브리드(모드 전환)는 직접 넣어야 한다</strong> —
        표준 Nav2 플래너에는 ground/air 상태 개념이 없다</li>
    <li><code>ElevationLayer</code> (2.5D costmap) 구현 — 현재 헤더만 있는 상태</li>
    <li><strong>Phase 2에서 이착륙 에너지를 최우선으로 실측.</strong>
        이 값이 알고리즘 선택보다 결과에 훨씬 큰 영향을 준다</li>
  </ol>
</section>

<footer>
  <p>
    재현: <code>benchmark/</code> 아래 검증·실험 스크립트가 전부 들어 있다.
    <code>verify_step1.py</code>부터 순서대로 실행하면 이 문서의 모든 수치가 재생산된다.
    원자료는 <code>results/benchmark_param_sweep.json</code>.
  </p>
</footer>
</div>

</div>
"""

OUT.write_text(HTML, encoding="utf-8")
print(f"저장: {OUT}  ({len(HTML)/1024:.0f} KB)")
