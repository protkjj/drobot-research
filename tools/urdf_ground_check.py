#!/usr/bin/env python3
"""base_footprint 가 정말 바닥면에 있는지 URDF 만으로 확인한다.

왜 필요한가
    물리엔진에서 로봇은 '가장 낮은 충돌체'가 바닥에 닿은 채로 놓인다.
    그 최저점이 base_footprint 기준 z=0 이 아니면, TF 가 말하는 센서 높이와
    Gazebo 안의 실제 센서 높이가 그만큼 어긋난다. EKF(two_d_mode) 는
    base_footprint 를 z=0 으로 고정하므로, 센서 점이 통째로 들리거나 꺼진다.

    2026-10-02 에 실제로 base_footprint_joint z=0.25 때문에 빈 바닥이
    0.25 m 로 측정됐다 (gz model z = -0.2518 로 확인).

하는 일
    관절각 0 에서 모든 <collision> 형상(STL 메시 · box · cylinder · sphere)을
    base_footprint 좌표로 옮겨 링크별 최저 z 를 잰다. ROS 없이 맥에서 돈다.

사용 (저장소 루트에서)
    python3 tools/urdf_ground_check.py
    python3 tools/urdf_ground_check.py src/drobot_description/urdf/drobot_primitives.urdf.xacro

    최저점이 |z| <= --tol 이면 종료 코드 0, 아니면 1.

한계
    .xacro 를 그냥 XML 로 읽는다. 매크로·${} 식이 joint/collision 에 쓰이면
    틀린 값이 나온다 (그런 파일이면 경고한다). 그때는 컨테이너에서
    `xacro 파일 > /tmp/out.urdf` 로 펼친 뒤 그 파일을 넘길 것.
    관절각 0 가정이라 팔이 처지면 수 mm~cm 달라질 수 있다.
"""
from __future__ import annotations

import argparse
import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
DEFAULT_URDF = REPO / "src" / "drobot_description" / "urdf" / "drobot.urdf.xacro"


# ---------------------------------------------------------------------------
# 좌표 변환
# ---------------------------------------------------------------------------
def rpy_to_R(roll, pitch, yaw):
    """URDF 규약: R = Rz(yaw) · Ry(pitch) · Rx(roll)."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def origin_to_T(elem) -> np.ndarray:
    """<origin xyz rpy> 를 4x4 동차변환으로. 없으면 단위행렬."""
    T = np.eye(4)
    if elem is None:
        return T
    T[:3, :3] = rpy_to_R(*[float(v) for v in elem.get("rpy", "0 0 0").split()])
    T[:3, 3] = [float(v) for v in elem.get("xyz", "0 0 0").split()]
    return T


# ---------------------------------------------------------------------------
# 형상 -> 꼭짓점 (형상 자신의 좌표계)
# ---------------------------------------------------------------------------
def load_binary_stl(path: Path) -> np.ndarray:
    data = path.read_bytes()
    # 바이너리 STL: 80바이트 헤더 + 삼각형 수(uint32) + 삼각형마다 50바이트
    n = struct.unpack_from("<I", data, 80)[0]
    if 84 + 50 * n != len(data):
        sys.exit(f"{path.name}: 바이너리 STL 이 아니다 (ASCII 이거나 Git LFS 포인터일 수 있다)")
    tri = np.frombuffer(data, offset=84, count=n, dtype=np.dtype(
        [("normal", "<f4", 3), ("v", "<f4", (3, 3)), ("attr", "<u2")]))
    return tri["v"].reshape(-1, 3).astype(float)


def resolve_package_uri(uri: str) -> Path:
    """package://<패키지>/<경로>  ->  저장소의 src/<패키지>/<경로>."""
    pkg, rel = uri.removeprefix("package://").split("/", 1)
    return REPO / "src" / pkg / rel


def shape_vertices(geometry) -> np.ndarray | None:
    g = list(geometry)[0]
    if g.tag == "mesh":
        V = load_binary_stl(resolve_package_uri(g.get("filename")))
        scale = [float(v) for v in g.get("scale", "1 1 1").split()]
        return V * scale
    if g.tag == "box":
        hx, hy, hz = (float(v) / 2 for v in g.get("size").split())
        return np.array([[x, y, z] for x in (-hx, hx) for y in (-hy, hy) for z in (-hz, hz)])
    if g.tag == "cylinder":
        # 축이 로컬 z. 옆면 최저점이 어디든 잡히도록 원둘레를 촘촘히 찍는다.
        r, L = float(g.get("radius")), float(g.get("length"))
        t = np.linspace(0, 2 * np.pi, 720, endpoint=False)
        ring = np.c_[r * np.cos(t), r * np.sin(t)]
        return np.vstack([np.c_[ring, np.full(len(t), z)] for z in (-L / 2, L / 2)])
    if g.tag == "sphere":
        r = float(g.get("radius"))
        return np.array([[0, 0, -r], [0, 0, r]])   # 회전과 무관하게 최저점은 중심 - r
    return None


# ---------------------------------------------------------------------------
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("urdf", nargs="?", default=str(DEFAULT_URDF))
    ap.add_argument("--root", default="base_footprint", help="바닥면이어야 하는 링크")
    ap.add_argument("--camera", default="camera_link")
    ap.add_argument("--tol", type=float, default=0.01, help="허용 오차 [m]")
    a = ap.parse_args()

    path = Path(a.urdf)
    text = path.read_text()
    if "${" in text or "xacro:macro" in text or "xacro:property" in text:
        print("⚠ xacro 식/매크로가 있다. 값이 틀릴 수 있으니 xacro 로 펼친 파일을 넘길 것")

    robot = ET.fromstring(text)
    joint_of_child = {j.find("child").get("link"): j for j in robot.findall("joint")}

    cache = {a.root: np.eye(4)}

    def T_of(link: str) -> np.ndarray:
        """관절각 0 에서 root -> link 변환 (부모를 따라 재귀)."""
        if link not in cache:
            j = joint_of_child[link]
            cache[link] = T_of(j.find("parent").get("link")) @ origin_to_T(j.find("origin"))
        return cache[link]

    rows = []        # (최저 z, 링크, 최저점 좌표)
    for link in robot.findall("link"):
        name = link.get("name")
        for col in link.findall("collision"):     # 주석 처리된 collision 은 XML 파서가 무시한다
            V = shape_vertices(col.find("geometry"))
            if V is None:
                continue
            T = T_of(name) @ origin_to_T(col.find("origin"))
            W = V @ T[:3, :3].T + T[:3, 3]
            k = W[:, 2].argmin()
            rows.append((W[k, 2], name, W[k]))
    if not rows:
        sys.exit("충돌체가 하나도 없다")
    rows.sort(key=lambda r: r[0])

    print(f"{path.name}  (관절각 0, {a.root} 기준)")
    print(f"{'링크':<22}{'최저 z [m]':>12}")
    for z, name, _ in rows:
        print(f"{name:<22}{z:>+12.4f}")

    lowest = rows[0][0]
    print()
    print(f"충돌체 최저점   {lowest:+.4f} m  ({rows[0][1]})   <- 0 이어야 한다")

    # 바퀴 좌우 접지 높이 차 -> 바닥에 놓였을 때의 기울기 (TF 에는 안 들어간다)
    # 바퀴가 최저점일 때만 의미가 있다 (몸체가 먼저 닿으면 바퀴로 서지 않는다)
    wheels = [(p, n) for _, n, p in rows if "wheel" in n]
    left = [p for p, n in wheels if n.startswith("left")]
    right = [p for p, n in wheels if n.startswith("right")]
    if left and right and "wheel" in rows[0][1]:
        zl, zr = np.mean([p[2] for p in left]), np.mean([p[2] for p in right])
        yl, yr = np.mean([p[1] for p in left]), np.mean([p[1] for p in right])
        roll = np.degrees(np.arctan2(zr - zl, yl - yr))
        print(f"좌우 바퀴 접지 높이차 {abs(zr - zl) * 1000:.1f} mm  ->  놓였을 때 roll 약 {roll:+.2f}°")

    if a.camera in joint_of_child:
        cam = T_of(a.camera)[2, 3]
        print(f"카메라 높이     TF {cam:.3f} m  /  바닥에 놓였을 때 실제 {cam - lowest:.3f} m")

    ok = abs(lowest) <= a.tol
    print(f"\n{'OK' if ok else 'FAIL'}: |{lowest:+.4f}| {'<=' if ok else '>'} {a.tol} m")
    if not ok:
        print(f"  TF 를 거친 센서 점이 {lowest:+.3f} m 어긋나 보인다. "
              f"{a.root} 관절의 z 를 확인할 것.")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
