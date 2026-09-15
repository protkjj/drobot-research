"""벤치마크 높이맵 -> Gazebo SDF 월드 변환.

benchmark/envs/heightmap.py 에서 설계·검증한 맵 6종을 Gazebo 에서 그대로
쓸 수 있는 SDF 로 내보낸다.

왜 이게 필요한가:
    원래 있던 월드 파일들은 Git LFS 포인터만 남고 실제 데이터가
    서버에 없어 복구가 불가능하다. 그리고 커스텀 맵이 있던
    worlds/generated/ 는 .gitignore 대상이라 커밋된 적이 없다.

    벤치마크 맵을 쓰면 오히려 낫다:
      - R1/R2/R3 요구사항 검증을 이미 마쳤다
      - 3D 로는 못 넘는 높이(0.95~1.55m)를 포함해 하이브리드 동작을 시험할 수 있다
      - 벤치마크 수치와 시뮬레이션 결과를 '같은 맵'에서 비교할 수 있다

변환 방식:
    높이맵은 셀 단위 2D 배열이다. 셀 하나당 박스를 만들면 수만 개가 되어
    Gazebo 가 못 버틴다. 그래서 같은 높이의 인접 셀을 직사각형으로 병합해서
    (greedy rectangle merge) 박스 개수를 줄인다.

실행:
    python3 benchmark/export_sdf.py
    -> src/drobot_description/worlds/benchmark/*.sdf
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from envs.heightmap import HeightMap, build_all, ROVER_MAX  # noqa: E402

OUT_DIR = (Path(__file__).resolve().parents[1]
           / "src/drobot_description/worlds/benchmark")


def merge_rectangles(mask: np.ndarray) -> list[tuple[int, int, int, int]]:
    """True 로 표시된 셀들을 직사각형 목록으로 병합한다.

    greedy 방식: 왼쪽 위부터 훑으며, 가능한 한 넓은 직사각형을 잘라낸다.
    최적 분할은 아니지만 축 정렬 블록 구조에서는 충분히 잘 줄어든다.
    반환: (x0, y0, x1, y1) 인덱스 목록 (x1, y1 은 배타적)
    """
    m = mask.copy()
    ny, nx = m.shape
    rects = []

    for y in range(ny):
        x = 0
        while x < nx:
            if not m[y, x]:
                x += 1
                continue
            # 가로로 최대한 늘린다
            x1 = x
            while x1 < nx and m[y, x1]:
                x1 += 1
            # 세로로 최대한 늘린다 (그 가로폭이 전부 True 인 동안)
            y1 = y + 1
            while y1 < ny and m[y1, x:x1].all():
                y1 += 1
            rects.append((x, y, x1, y1))
            m[y:y1, x:x1] = False
            x = x1
    return rects


def height_layers(hm: HeightMap) -> dict[float, np.ndarray]:
    """높이별 마스크. 부동소수 오차를 피하려고 소수 3자리로 반올림한다."""
    g = np.round(hm.grid, 3)
    out = {}
    for h in np.unique(g):
        if h <= ROVER_MAX:      # 로버가 그냥 지나가는 높이는 장애물로 만들지 않는다
            continue
        out[float(h)] = (g == h)
    return out


def to_sdf(hm: HeightMap) -> str:
    res = hm.resolution
    parts = []
    add = parts.append

    add('<?xml version="1.0" ?>')
    add('<!-- 자동 생성 파일 — 직접 수정하지 말 것.')
    add('     생성: python3 benchmark/export_sdf.py')
    add(f'     원본: benchmark/envs/heightmap.py 의 {hm.name}')
    add(f'     {hm.description} -->')
    add('<sdf version="1.9">')
    add(f'  <world name="{hm.name}">')
    add('')
    add('    <physics name="1ms" type="ignored">')
    add('      <max_step_size>0.001</max_step_size>')
    add('      <real_time_factor>1.0</real_time_factor>')
    add('    </physics>')
    add('')
    # gz-sim 필수 시스템 플러그인
    for name, fn in [
        ("gz::sim::systems::Physics", "gz-sim-physics-system"),
        ("gz::sim::systems::UserCommands", "gz-sim-user-commands-system"),
        ("gz::sim::systems::SceneBroadcaster", "gz-sim-scene-broadcaster-system"),
        ("gz::sim::systems::Contact", "gz-sim-contact-system"),
        ("gz::sim::systems::Imu", "gz-sim-imu-system"),
        ("gz::sim::systems::Sensors", "gz-sim-sensors-system"),
    ]:
        extra = ("\n      <render_engine>ogre2</render_engine>"
                 if "Sensors" in name else "")
        add(f'    <plugin filename="{fn}" name="{name}">{extra}')
        add('    </plugin>')
    add('')
    add('    <light type="directional" name="sun">')
    add('      <cast_shadows>true</cast_shadows>')
    add('      <pose>0 0 10 0 0 0</pose>')
    add('      <diffuse>0.8 0.8 0.8 1</diffuse>')
    add('      <specular>0.2 0.2 0.2 1</specular>')
    add('      <direction>-0.5 0.1 -0.9</direction>')
    add('    </light>')
    add('')

    # 바닥 — 맵 전체를 덮는 평면
    add('    <model name="ground_plane">')
    add('      <static>true</static>')
    add('      <link name="link">')
    add('        <collision name="collision">')
    add('          <geometry><plane>')
    add('            <normal>0 0 1</normal>')
    add(f'            <size>{hm.width_m + 10:.1f} {hm.height_m + 10:.1f}</size>')
    add('          </plane></geometry>')
    add('        </collision>')
    add('        <visual name="visual">')
    add('          <geometry><plane>')
    add('            <normal>0 0 1</normal>')
    add(f'            <size>{hm.width_m + 10:.1f} {hm.height_m + 10:.1f}</size>')
    add('          </plane></geometry>')
    add('          <material>')
    add('            <ambient>0.75 0.75 0.72 1</ambient>')
    add('            <diffuse>0.75 0.75 0.72 1</diffuse>')
    add('          </material>')
    add('        </visual>')
    add('      </link>')
    add('    </model>')
    add('')

    # 장애물 — 높이별로 직사각형 병합 후 박스 생성
    n_box = 0
    for h, mask in sorted(height_layers(hm).items()):
        rects = merge_rectangles(mask)
        # 높이에 따라 색을 달리해 RViz/Gazebo 에서 구분되게 한다.
        #   회색   : 로버가 못 넘고 3D 비행으로 넘는 높이
        #   주황   : 3D 로는 못 넘고 4D 로만 넘는 높이 (0.95~1.55m)
        #   진회색 : 통과 불가 (벽)
        if h > 1.55:
            rgba = "0.25 0.25 0.27 1"
        elif h > 0.95:
            rgba = "0.91 0.58 0.23 1"
        else:
            rgba = "0.66 0.81 0.88 1"

        for (x0, y0, x1, y1) in rects:
            n_box += 1
            sx = (x1 - x0) * res
            sy = (y1 - y0) * res
            cx = (x0 + x1) / 2.0 * res
            cy = (y0 + y1) / 2.0 * res
            name = f"obs_{n_box:04d}_h{int(round(h * 100)):03d}"
            add(f'    <model name="{name}">')
            add('      <static>true</static>')
            add(f'      <pose>{cx:.3f} {cy:.3f} {h / 2:.3f} 0 0 0</pose>')
            add('      <link name="link">')
            add('        <collision name="collision">')
            add(f'          <geometry><box><size>{sx:.3f} {sy:.3f} {h:.3f}</size></box></geometry>')
            add('        </collision>')
            add('        <visual name="visual">')
            add(f'          <geometry><box><size>{sx:.3f} {sy:.3f} {h:.3f}</size></box></geometry>')
            add('          <material>')
            add(f'            <ambient>{rgba}</ambient>')
            add(f'            <diffuse>{rgba}</diffuse>')
            add('          </material>')
            add('        </visual>')
            add('      </link>')
            add('    </model>')

    add('')
    add('  </world>')
    add('</sdf>')
    return "\n".join(parts) + "\n", n_box


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    maps = build_all(res=0.1)

    print("벤치마크 맵 -> SDF 변환")
    print("=" * 70)
    print(f"  {'맵':<17} {'크기':>10} {'박스 수':>8} {'start':>12} {'goal':>12}")
    print("  " + "-" * 64)

    spawn = {}
    for name, hm in maps.items():
        sdf, n_box = to_sdf(hm)
        out = OUT_DIR / f"{name}.sdf"
        out.write_text(sdf)
        spawn[name] = (hm.start, hm.goal)
        print(f"  {name:<17} {hm.width_m:.0f}x{hm.height_m:.0f}m {n_box:>8} "
              f"{str(hm.start):>12} {str(hm.goal):>12}")

    print()
    print(f"저장: {OUT_DIR}")
    print()
    print("spawn_positions.yaml 에 추가할 항목:")
    for name, (s, g) in spawn.items():
        print(f"  {name}: {{x: {s[0]}, y: {s[1]}}}   # goal: {g}")


if __name__ == "__main__":
    main()
