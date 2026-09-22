"""RRT* prototyping용 2.5D 테스트 맵 생성기 (pure Python, ROS/Gazebo 의존 없음).

높이 임계값 / 코스트 / 해상도는 elevation_params.yaml에서 로드한다.
박스(직육면체) 장애물 1개로 시작 — 위치/크기/높이 인자화.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import yaml
from matplotlib.colors import BoundaryNorm, ListedColormap

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_SRC = SCRIPT_DIR.parents[1]
PARAMS_YAML = REPO_SRC / "drobot_costmap_2_5d" / "config" / "elevation_params.yaml"
DEFAULT_OUT_DIR = SCRIPT_DIR / "maps"
DEFAULT_SDF_DIR = REPO_SRC / "drobot_description" / "worlds"


@dataclass
class ElevationParams:
    resolution: float
    rover_max: float
    flyover_max: float
    cost_free: int
    cost_rover: int
    cost_flyover: int
    cost_impass: int

    @classmethod
    def load(cls, yaml_path: Path = PARAMS_YAML) -> "ElevationParams":
        with open(yaml_path) as f:
            data = yaml.safe_load(f)
        p = data["elevation_costmap_layer"]["ros__parameters"]
        return cls(
            resolution=p["resolution"],
            rover_max=p["height_thresholds"]["rover_traversable_max"],
            flyover_max=p["height_thresholds"]["fly_over_max"],
            cost_free=p["cost_values"]["free"],
            cost_rover=p["cost_values"]["rover_traversable"],
            cost_flyover=p["cost_values"]["fly_over"],
            cost_impass=p["cost_values"]["impassable"],
        )


@dataclass
class TestMap:
    params: ElevationParams
    size_x_m: float = 6.0
    size_y_m: float = 11.0
    start_m: tuple[float, float] = (2.0, 1.0)
    goal_m: tuple[float, float] = (2.0, 10.0)
    height_map: np.ndarray = field(init=False)
    boxes: list[tuple[float, float, float, float, float]] = field(default_factory=list)

    def __post_init__(self) -> None:
        res = self.params.resolution
        nx = int(round(self.size_x_m / res))
        ny = int(round(self.size_y_m / res))
        self.height_map = np.zeros((ny, nx), dtype=np.float32)

    @property
    def nx(self) -> int:
        return self.height_map.shape[1]

    @property
    def ny(self) -> int:
        return self.height_map.shape[0]

    def add_box(self, x: float, y: float, w: float, d: float, h: float) -> None:
        """축 정렬 직육면체. (x, y) = 중심(m), w/d = x/y 방향 폭(m), h = 높이(m)."""
        res = self.params.resolution
        j0 = max(int(np.floor((x - w / 2) / res)), 0)
        j1 = min(int(np.ceil((x + w / 2) / res)), self.nx)
        i0 = max(int(np.floor((y - d / 2) / res)), 0)
        i1 = min(int(np.ceil((y + d / 2) / res)), self.ny)
        self.height_map[i0:i1, j0:j1] = np.maximum(self.height_map[i0:i1, j0:j1], h)
        self.boxes.append((x, y, w, d, h))

    def classify(self) -> np.ndarray:
        """0=Free, 1=Rover, 2=Flyover, 3=Impass."""
        h = self.height_map
        c = np.zeros_like(h, dtype=np.uint8)
        c[h > 0.0] = 1
        c[h > self.params.rover_max] = 2
        c[h > self.params.flyover_max] = 3
        return c

    def costmap(self) -> np.ndarray:
        lut = np.array(
            [
                self.params.cost_free,
                self.params.cost_rover,
                self.params.cost_flyover,
                self.params.cost_impass,
            ],
            dtype=np.uint8,
        )
        return lut[self.classify()]

    def save(self, path: Path) -> None:
        np.savez(
            path,
            height_map=self.height_map,
            class_map=self.classify(),
            cost_map=self.costmap(),
            start_m=np.array(self.start_m, dtype=np.float32),
            goal_m=np.array(self.goal_m, dtype=np.float32),
            resolution=np.float32(self.params.resolution),
            size_x_m=np.float32(self.size_x_m),
            size_y_m=np.float32(self.size_y_m),
        )

    @classmethod
    def load_npz(cls, path: Path, params: ElevationParams | None = None) -> "TestMap":
        data = np.load(path)
        params = params or ElevationParams.load()
        m = cls(
            params=params,
            size_x_m=float(data["size_x_m"]),
            size_y_m=float(data["size_y_m"]),
            start_m=tuple(data["start_m"].tolist()),
            goal_m=tuple(data["goal_m"].tolist()),
        )
        m.height_map = data["height_map"].astype(np.float32)
        return m

    def to_sdf(self, world_name: str) -> str:
        """Gazebo Sim용 SDF 월드 문자열 (ground plane + sun + 박스 N개 + start/goal 마커)."""
        box_models = "\n".join(_box_model_xml(i, b) for i, b in enumerate(self.boxes))
        markers = "\n".join([
            _marker_model_xml("start_marker", self.start_m, rgba=(0.0, 1.0, 0.0, 1.0)),
            _marker_model_xml("goal_marker", self.goal_m, rgba=(1.0, 0.85, 0.0, 1.0)),
        ])
        return _SDF_TEMPLATE.format(
            world_name=world_name, box_models=box_models, markers=markers
        )

    def visualize(self, save_png: Path | None = None, show: bool = True) -> None:
        aspect = self.size_x_m / self.size_y_m
        fig, axes = plt.subplots(1, 2, figsize=(6 + 6 * aspect, 7))
        extent = (0.0, self.size_x_m, 0.0, self.size_y_m)

        vmax = max(2.5, float(self.height_map.max()) + 0.1)
        im0 = axes[0].imshow(
            self.height_map,
            origin="lower",
            extent=extent,
            cmap="viridis",
            vmin=0.0,
            vmax=vmax,
        )
        axes[0].set_title("Height map (m)")
        plt.colorbar(im0, ax=axes[0], label="height (m)")

        cmap_cls = ListedColormap(["white", "lightgreen", "skyblue", "red"])
        norm = BoundaryNorm([0, 1, 2, 3, 4], cmap_cls.N)
        im1 = axes[1].imshow(
            self.classify(),
            origin="lower",
            extent=extent,
            cmap=cmap_cls,
            norm=norm,
        )
        axes[1].set_title("Class: Free / Rover / Flyover / Impass")
        cbar = plt.colorbar(im1, ax=axes[1], ticks=[0.5, 1.5, 2.5, 3.5])
        cbar.ax.set_yticklabels(["Free", "Rover", "Flyover", "Impass"])

        for ax in axes:
            ax.plot(*self.start_m, "o", color="lime", markeredgecolor="black",
                    markersize=12, label="start")
            ax.plot(*self.goal_m, "*", color="red", markeredgecolor="black",
                    markersize=18, label="goal")
            ax.set_xlim(0, self.size_x_m)
            ax.set_ylim(0, self.size_y_m)
            ax.set_xlabel("x (m)")
            ax.set_ylabel("y (m)")
            ax.set_aspect("equal")
            ax.legend(loc="upper left")

        plt.tight_layout()
        if save_png is not None:
            save_png.parent.mkdir(parents=True, exist_ok=True)
            fig.savefig(save_png, dpi=110, bbox_inches="tight")
        if show:
            plt.show()
        plt.close(fig)


_SDF_TEMPLATE = """<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="{world_name}">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

{box_models}

{markers}
  </world>
</sdf>
"""


def _box_model_xml(idx: int, box: tuple[float, float, float, float, float]) -> str:
    x, y, w, d, h = box
    z = h / 2  # SDF는 박스 중심 좌표 — 지면 위에 얹으려면 h/2
    return (
        f'    <model name="obstacle_{idx}">\n'
        f'      <static>true</static>\n'
        f'      <pose>{x} {y} {z} 0 0 0</pose>\n'
        f'      <link name="link">\n'
        f'        <collision name="collision">\n'
        f'          <geometry><box><size>{w} {d} {h}</size></box></geometry>\n'
        f'        </collision>\n'
        f'        <visual name="visual">\n'
        f'          <geometry><box><size>{w} {d} {h}</size></box></geometry>\n'
        f'          <material>\n'
        f'            <ambient>0.5 0.5 0.8 1</ambient>\n'
        f'            <diffuse>0.5 0.5 0.8 1</diffuse>\n'
        f'          </material>\n'
        f'        </visual>\n'
        f'      </link>\n'
        f'    </model>'
    )


def _marker_model_xml(
    name: str,
    pos: tuple[float, float],
    rgba: tuple[float, float, float, float],
    radius: float = 0.25,
    height: float = 0.04,
) -> str:
    """Visual-only 마커 (collision 없음). 로봇이 통과해도 충돌 없음."""
    x, y = pos
    z = height / 2
    r, g, b, a = rgba
    return (
        f'    <model name="{name}">\n'
        f'      <static>true</static>\n'
        f'      <pose>{x} {y} {z} 0 0 0</pose>\n'
        f'      <link name="link">\n'
        f'        <visual name="visual">\n'
        f'          <geometry><cylinder><radius>{radius}</radius><length>{height}</length></cylinder></geometry>\n'
        f'          <material>\n'
        f'            <ambient>{r} {g} {b} {a}</ambient>\n'
        f'            <diffuse>{r} {g} {b} {a}</diffuse>\n'
        f'            <emissive>{r * 0.3} {g * 0.3} {b * 0.3} 1</emissive>\n'
        f'          </material>\n'
        f'          <transparency>0.2</transparency>\n'
        f'        </visual>\n'
        f'      </link>\n'
        f'    </model>'
    )


def build_base_map(obstacle_height: float = 0.5) -> TestMap:
    """1차 프로토타입: 6x10 m, 4면 벽 + 중앙 박스 1개 (높이 sweep용).

    INA226 실측 전 단계 — geometry는 고정, 박스 높이만 변동시켜 분류/결정 변화 관찰.
    4면 벽: 높이 3m (Impassable) — 알고리즘이 경계 밖 샘플링하는 것 차단.
    """
    m = TestMap(params=ElevationParams.load())
    sx, sy = m.size_x_m, m.size_y_m
    WALL_H, WALL_T = 3.0, 0.1
    m.add_box(x=sx / 2, y=WALL_T / 2, w=sx, d=WALL_T, h=WALL_H)               # south
    m.add_box(x=sx / 2, y=sy - WALL_T / 2, w=sx, d=WALL_T, h=WALL_H)          # north
    m.add_box(x=WALL_T / 2, y=sy / 2, w=WALL_T, d=sy, h=WALL_H)               # west
    m.add_box(x=sx - WALL_T / 2, y=sy / 2, w=WALL_T, d=sy, h=WALL_H)          # east
    m.add_box(x=2.0, y=5.5, w=4.0, d=3.0, h=obstacle_height)                  # 중앙 박스 (start↔box, box↔goal 각각 3m)
    return m


def class_summary(m: TestMap) -> str:
    """현재 맵의 분류 셀 개수 요약 (어떤 regime에 떨어졌는지 확인용)."""
    labels = ["Free", "Rover", "Flyover", "Impass"]
    cls = m.classify()
    counts = [int((cls == i).sum()) for i in range(4)]
    total = cls.size
    parts = [f"{l}={c} ({100 * c / total:.1f}%)" for l, c in zip(labels, counts)]
    return " | ".join(parts)


def main() -> None:
    ap = argparse.ArgumentParser(description="RRT prototyping 테스트 맵 생성기")
    ap.add_argument("--height", type=float, default=0.5,
                    help="중앙 박스 높이 (m). 0.05→Rover, 0.5→Flyover, 2.5→Impass 식으로 sweep")
    ap.add_argument("--name", default=None,
                    help="저장 파일 베이스 이름. 미지정 시 'base_map_h{height}' 자동 생성")
    ap.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR,
                    help=f"npz/png 출력 디렉토리 (기본: {DEFAULT_OUT_DIR})")
    ap.add_argument("--sdf-out-dir", type=Path, default=DEFAULT_SDF_DIR,
                    help=f"SDF 출력 디렉토리 — launch가 찾는 위치 (기본: {DEFAULT_SDF_DIR})")
    ap.add_argument("--no-sdf", action="store_true", help="SDF 생성 건너뛰기")
    ap.add_argument("--show", action="store_true",
                    help="matplotlib GUI 창 띄우기 (기본은 PNG만 저장)")
    args = ap.parse_args()

    m = build_base_map(obstacle_height=args.height)
    name = args.name or f"base_map_h{args.height:g}"
    print(f"grid: {m.nx}x{m.ny} ({m.size_x_m}x{m.size_y_m} m), res: {m.params.resolution} m")
    print(f"start: {m.start_m}, goal: {m.goal_m}")
    print(f"obstacle height: {args.height} m -> {class_summary(m)}")

    args.out_dir.mkdir(parents=True, exist_ok=True)
    npz_path = args.out_dir / f"{name}.npz"
    png_path = args.out_dir / f"{name}.png"
    m.save(npz_path)
    print(f"saved -> {npz_path}")
    m.visualize(save_png=png_path, show=args.show)
    print(f"saved -> {png_path}")

    if not args.no_sdf:
        args.sdf_out_dir.mkdir(parents=True, exist_ok=True)
        sdf_path = args.sdf_out_dir / f"{name}.sdf"
        sdf_path.write_text(m.to_sdf(world_name=name))
        print(f"saved -> {sdf_path}")


if __name__ == "__main__":
    main()
