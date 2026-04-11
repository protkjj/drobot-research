from pathlib import Path
import xml.etree.ElementTree as ET
import random
import math
import re
import os
import shutil
from typing import Tuple
import yaml

def get_base_world_name() -> str:
    """Get base world name passed by UI launch."""
    base_world_name = os.environ.get("DROBOT_BASE_WORLD_NAME")
    if not base_world_name:
        raise RuntimeError(
            "DROBOT_BASE_WORLD_NAME is not set. "
            "Run world_generator.py from ui.launch.py Create World flow."
        )
    return base_world_name

BASE_WORLD_NAME = get_base_world_name()


def get_workspace_root() -> Path:
    """Resolve workspace root, preferring explicit env from UI."""
    ws_env = os.environ.get("DROBOT_WORKSPACE_DIR")
    if ws_env:
        return Path(ws_env).resolve()

    this_file = Path(__file__).resolve()
    for parent in this_file.parents:
        if parent.name == "install":
            return parent.parent

    # Source-layout fallback: <ws>/src/drobot_description/scripts/world_generator.py
    return this_file.parents[3]


def get_drobot_description_root() -> Path:
    """
    Resolve drobot_description package root in source space when available.
    """
    ws_root = get_workspace_root()
    src_pkg = ws_root / "src" / "drobot_description"
    if src_pkg.is_dir():
        return src_pkg
    return Path(__file__).resolve().parent.parent


def get_worlds_root() -> Path:
    """Return worlds directory for this package."""
    return get_drobot_description_root() / "worlds"


def get_original_worlds_dir() -> Path:
    """Return source worlds directory."""
    return get_worlds_root() / "original"


def get_generated_worlds_dir() -> Path:
    """Return generated worlds directory."""
    return get_worlds_root() / "generated"


def resolve_base_world_file(base_world_name: str) -> Path:
    """
    Resolve base world file from absolute path, worlds/, or worlds/original/.
    """
    candidate = Path(base_world_name).expanduser()
    if candidate.is_file():
        return candidate.resolve()

    worlds_root = get_worlds_root()
    direct = worlds_root / base_world_name
    if direct.is_file():
        return direct.resolve()

    original = get_original_worlds_dir() / base_world_name
    if original.is_file():
        return original.resolve()

    raise FileNotFoundError(
        f"Base world file not found: {base_world_name} "
        f"(searched: absolute, {worlds_root}, {get_original_worlds_dir()})"
    )


def get_next_output_world_name(base_world_name: str, search_dir: Path | None = None) -> str:
    base = Path(base_world_name)
    stem = base.stem
    ext = base.suffix if base.suffix else ".sdf"

    root = Path(search_dir) if search_dir else get_generated_worlds_dir()
    if not root.exists():
        return f"{stem}_1{ext}"

    regex = re.compile(rf"^{re.escape(stem)}_(\d+){re.escape(ext)}$")
    max_index = 0

    for world_file in root.rglob(f"{stem}_*{ext}"):
        if not world_file.is_file():
            continue
        match = regex.match(world_file.name)
        if not match:
            continue
        max_index = max(max_index, int(match.group(1)))

    return f"{stem}_{max_index + 1}{ext}"


OUTPUT_WORLD_NAME = get_next_output_world_name(BASE_WORLD_NAME)


def get_base_goals() -> str:
    """Get base goals from base world."""
    base_goals = os.environ.get("DROBOT_BASE_GOALS")
    if not base_goals:
        raise RuntimeError(
            "DROBOT_BASE_GOALS is not set. "
            "Select a goal in ui.launch.py before Create World."
        )
    return base_goals

BASE_GOALS = get_base_goals()

def get_base_options() -> str:
    """Get base options from base world."""
    base_options = os.environ.get("DROBOT_BASE_OPTIONS")
    if not base_options:
        raise RuntimeError(
            "DROBOT_BASE_OPTIONS is not set. "
            "Select an option in ui.launch.py before Create World."
        )
    return base_options


BASE_OPTIONS = get_base_options()


def parse_goal_count(base_goals: str) -> int:
    """Convert BASE_GOALS input to a concrete goal count."""
    value = str(base_goals).strip().lower()
    if value == "random":
        return random.randint(0, 3)
    try:
        count = int(value)
    except ValueError as exc:
        raise ValueError(f"Invalid BASE_GOALS value: {base_goals}") from exc
    return max(0, count)


def get_goal_dir() -> Path:
    """Return goal object directory."""
    return get_drobot_description_root() / "object" / "goal"


def parse_pose_text(pose_text: str | None) -> Tuple[float, float, float]:
    """Parse SDF pose text and return (x, y, yaw)."""
    if not pose_text:
        return 0.0, 0.0, 0.0
    values = [float(v) for v in pose_text.split()]
    while len(values) < 6:
        values.append(0.0)
    return values[0], values[1], values[5]


def compose_pose(parent_pose: Tuple[float, float, float], child_pose: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """Compose two planar poses (x, y, yaw)."""
    px, py, pyaw = parent_pose
    cx, cy, cyaw = child_pose
    x = px + math.cos(pyaw) * cx - math.sin(pyaw) * cy
    y = py + math.sin(pyaw) * cx + math.cos(pyaw) * cy
    return x, y, pyaw + cyaw


def get_spawn_config_path() -> Path:
    """Return bringup spawn_positions.yaml path."""
    ws_root = get_workspace_root()
    src_config = ws_root / "src" / "drobot_bringup" / "config" / "spawn_positions.yaml"
    if src_config.is_file():
        return src_config
    install_config = ws_root / "install" / "drobot_bringup" / "share" / "drobot_bringup" / "config" / "spawn_positions.yaml"
    return install_config


def get_robot_spawn_xy(base_world_name: str) -> Tuple[float, float]:
    """Read robot spawn position used by bringup for this world."""
    config_file = get_spawn_config_path()
    if not config_file.is_file():
        return 0.0, 0.0

    try:
        data = yaml.safe_load(config_file.read_text(encoding="utf-8")) or {}
        worlds = data.get("worlds", {}) or {}
        default = data.get("default", {"x": 0.0, "y": 0.0}) or {"x": 0.0, "y": 0.0}
        key = Path(base_world_name).stem
        pose = worlds.get(key, default) or default
        return float(pose.get("x", 0.0)), float(pose.get("y", 0.0))
    except Exception:
        return 0.0, 0.0


def extract_obstacle_circles(world_file: Path) -> list[Tuple[float, float, float]]:
    """
    Extract obstacles as circles: (x, y, radius).
    Uses geometric collisions when available, with fallback radius for includes/unknowns.
    """
    tree = ET.parse(world_file)
    world_el = tree.find(".//world")
    if world_el is None:
        return []

    obstacles: list[Tuple[float, float, float]] = []
    include_fallback_radius = 0.8
    unknown_fallback_radius = 0.6

    for child in list(world_el):
        if child.tag == "include":
            pose = parse_pose_text(child.findtext("pose"))
            obstacles.append((pose[0], pose[1], include_fallback_radius))
            continue

        if child.tag != "model":
            continue

        model_name = child.get("name", "")
        if "ground_plane" in model_name.lower():
            continue

        model_pose = parse_pose_text(child.findtext("pose"))
        model_obstacle_count_before = len(obstacles)

        for link in child.findall("link"):
            link_pose = compose_pose(model_pose, parse_pose_text(link.findtext("pose")))
            for collision in link.findall("collision"):
                collision_pose = compose_pose(link_pose, parse_pose_text(collision.findtext("pose")))
                geometry = collision.find("geometry")
                if geometry is None:
                    continue

                box = geometry.find("box")
                if box is not None:
                    size_text = box.findtext("size")
                    if size_text:
                        sx, sy, _ = [float(v) for v in size_text.split()]
                        radius = math.hypot(sx / 2.0, sy / 2.0)
                        obstacles.append((collision_pose[0], collision_pose[1], radius))
                        continue

                cylinder = geometry.find("cylinder")
                if cylinder is not None:
                    radius_text = cylinder.findtext("radius")
                    if radius_text:
                        obstacles.append((collision_pose[0], collision_pose[1], float(radius_text)))
                        continue

                cone = geometry.find("cone")
                if cone is not None:
                    radius_text = cone.findtext("radius")
                    if radius_text:
                        obstacles.append((collision_pose[0], collision_pose[1], float(radius_text)))
                        continue

                sphere = geometry.find("sphere")
                if sphere is not None:
                    radius_text = sphere.findtext("radius")
                    if radius_text:
                        obstacles.append((collision_pose[0], collision_pose[1], float(radius_text)))
                        continue

        # If no collision geometry was found for this model, keep a fallback safety circle.
        if len(obstacles) == model_obstacle_count_before:
            obstacles.append((model_pose[0], model_pose[1], unknown_fallback_radius))

    return obstacles


def get_sampling_bounds(obstacles: list[Tuple[float, float, float]]) -> Tuple[float, float, float, float]:
    """Compute map bounds from obstacle extents with safe inward margin."""
    if not obstacles:
        return -5.0, 5.0, -5.0, 5.0

    x_min = min(x - r for x, _, r in obstacles)
    x_max = max(x + r for x, _, r in obstacles)
    y_min = min(y - r for _, y, r in obstacles)
    y_max = max(y + r for _, y, r in obstacles)

    # Keep goals away from outer limits.
    margin = 1.0
    x_min += margin
    x_max -= margin
    y_min += margin
    y_max -= margin

    if x_min >= x_max or y_min >= y_max:
        return -5.0, 5.0, -5.0, 5.0
    return x_min, x_max, y_min, y_max


def sample_goal_positions(goal_count: int, base_world_name: str, template_pose: dict, template_radius: float) -> list[Tuple[float, float]]:
    """
    Sample random goal positions inside map while avoiding obstacles and robot spawn.
    """
    if goal_count <= 0:
        return []

    world_file = resolve_base_world_file(base_world_name)
    obstacles = extract_obstacle_circles(world_file)
    x_min, x_max, y_min, y_max = get_sampling_bounds(obstacles)
    robot_x, robot_y = get_robot_spawn_xy(base_world_name)

    robot_exclusion_radius = 1.2
    obstacle_clearance = 0.6
    goal_spacing = 0.45
    max_attempts = 5000

    positions: list[Tuple[float, float]] = []

    def is_valid(px: float, py: float) -> bool:
        # Keep away from robot spawn.
        if math.hypot(px - robot_x, py - robot_y) < robot_exclusion_radius:
            return False

        # Keep away from existing world obstacles.
        for ox, oy, oradius in obstacles:
            min_dist = oradius + template_radius + obstacle_clearance
            if math.hypot(px - ox, py - oy) < min_dist:
                return False

        # Keep goals from overlapping each other.
        for gx, gy in positions:
            if math.hypot(px - gx, py - gy) < (2.0 * template_radius + goal_spacing):
                return False

        return True

    attempts = 0
    while len(positions) < goal_count and attempts < max_attempts:
        attempts += 1
        px = random.uniform(x_min, x_max)
        py = random.uniform(y_min, y_max)
        if is_valid(px, py):
            positions.append((round(px, 3), round(py, 3)))

    if len(positions) < goal_count:
        raise RuntimeError(
            f"Could not place {goal_count} goals safely in {world_file.name}. "
            f"Placed {len(positions)} after {attempts} attempts."
        )

    return positions


def append_goal_cones_to_world(output_world_file: Path, goal_count: int, base_world_name: str) -> list[str]:
    """
    Append goal cone models directly into one generated world file.
    """
    if goal_count <= 0:
        return []

    goal_dir = get_goal_dir()
    template = goal_dir / "goal.yaml"
    if not template.is_file():
        raise FileNotFoundError(f"Goal template not found: {template}")

    template_data = yaml.safe_load(template.read_text(encoding="utf-8")) or {}
    template_pose = template_data.get("pose", {}) or {}
    template_size = template_data.get("size", {}) or {}
    template_visual = template_data.get("visual", {}) or {}
    template_color = (template_visual.get("color", {}) or {})
    template_physics = template_data.get("physics", {}) or {}

    radius = float(template_size.get("radius", 0.3))
    length = float(template_size.get("length", template_size.get("height", 1.0)))
    z = float(template_pose.get("z", length / 2.0))
    roll = float(template_pose.get("roll", 0.0))
    pitch = float(template_pose.get("pitch", 0.0))
    is_static = bool(template_physics.get("static", False))

    positions = sample_goal_positions(goal_count, base_world_name, template_pose, radius)

    tree = ET.parse(output_world_file)
    world_el = tree.find(".//world")
    if world_el is None:
        raise RuntimeError(f"No <world> element in {output_world_file}")

    existing_names = {m.get("name", "") for m in world_el.findall("model")}
    added_names: list[str] = []

    for idx, (px, py) in enumerate(positions, start=1):
        model_name = f"goal_cone_{idx:02d}"
        while model_name in existing_names:
            idx += 1
            model_name = f"goal_cone_{idx:02d}"
        existing_names.add(model_name)
        added_names.append(model_name)

        yaw = round(random.uniform(-math.pi, math.pi), 4)

        model_el = ET.SubElement(world_el, "model", {"name": model_name})
        ET.SubElement(model_el, "pose").text = f"{px} {py} {z} {roll} {pitch} {yaw}"
        ET.SubElement(model_el, "static").text = "true" if is_static else "false"
        ET.SubElement(model_el, "self_collide").text = "false"

        link_el = ET.SubElement(model_el, "link", {"name": "goal_cone_link"})
        ET.SubElement(link_el, "pose").text = "0 0 0 0 0 0"

        inertial_el = ET.SubElement(link_el, "inertial")
        ET.SubElement(inertial_el, "pose").text = "0 0 0 0 0 0"
        ET.SubElement(inertial_el, "mass").text = "1.0"
        inertia_el = ET.SubElement(inertial_el, "inertia")
        ET.SubElement(inertia_el, "ixx").text = "0.075"
        ET.SubElement(inertia_el, "ixy").text = "0"
        ET.SubElement(inertia_el, "ixz").text = "0"
        ET.SubElement(inertia_el, "iyy").text = "0.075"
        ET.SubElement(inertia_el, "iyz").text = "0"
        ET.SubElement(inertia_el, "izz").text = "0.075"

        collision_el = ET.SubElement(link_el, "collision", {"name": "goal_cone_collision"})
        collision_geom = ET.SubElement(collision_el, "geometry")
        collision_cone = ET.SubElement(collision_geom, "cone")
        ET.SubElement(collision_cone, "radius").text = str(radius)
        ET.SubElement(collision_cone, "length").text = str(length)

        visual_el = ET.SubElement(link_el, "visual", {"name": "goal_cone_visual"})
        visual_geom = ET.SubElement(visual_el, "geometry")
        visual_cone = ET.SubElement(visual_geom, "cone")
        ET.SubElement(visual_cone, "radius").text = str(radius)
        ET.SubElement(visual_cone, "length").text = str(length)

        material_el = ET.SubElement(visual_el, "material")
        r = float(template_color.get("r", 0.7))
        g = float(template_color.get("g", 0.7))
        b = float(template_color.get("b", 0.7))
        a = float(template_color.get("a", 1.0))
        ET.SubElement(material_el, "ambient").text = f"{r} {g} {b} {a}"
        ET.SubElement(material_el, "diffuse").text = f"{r} {g} {b} {a}"
        ET.SubElement(material_el, "specular").text = "1 1 1 1"

    tree.write(output_world_file, encoding="utf-8", xml_declaration=False)
    return added_names


def copy_base_world_to_generated() -> Path:
    """
    Copy selected base world into worlds/generated using OUTPUT_WORLD_NAME.
    """
    source_world = resolve_base_world_file(BASE_WORLD_NAME)
    generated_dir = get_generated_worlds_dir()
    generated_dir.mkdir(parents=True, exist_ok=True)
    output_world = generated_dir / OUTPUT_WORLD_NAME
    shutil.copy2(source_world, output_world)
    return output_world


def main():
    output_world = copy_base_world_to_generated()
    goal_count = parse_goal_count(BASE_GOALS)
    added_models = append_goal_cones_to_world(output_world, goal_count, BASE_WORLD_NAME)
    print(f"BASE_WORLD_NAME={BASE_WORLD_NAME}")
    print(f"OUTPUT_WORLD_NAME={OUTPUT_WORLD_NAME}")
    print(f"BASE_GOALS={BASE_GOALS}")
    print(f"GOAL_COUNT={goal_count}")
    if BASE_OPTIONS:
        print(f"BASE_OPTIONS={BASE_OPTIONS}")
    print(f"Generated world created: {output_world}")
    print(f"Goal cones added to world: {len(added_models)}")


if __name__ == "__main__":
    main()
