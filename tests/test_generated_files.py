"""Repository-level checks: generated files are up to date and consistent with each other."""
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "tools"))
import generate_warehouse as gw  # noqa: E402

GENERATED = ["amr_gazebo/worlds/warehouse.sdf", "amr_navigation/maps/warehouse.pgm",
             "amr_navigation/maps/warehouse.yaml", "amr_navigation/maps/keepout_mask.pgm",
             "amr_navigation/maps/speed_mask.pgm", "amr_mission/config/stations.yaml",
             "amr_navigation/config/docks.yaml"]


def read_pgm(path):
    data = path.read_bytes()
    parts = data.split(b"\n", 3)
    w, h = map(int, parts[1].split())
    return np.frombuffer(parts[3], dtype=np.uint8).reshape(h, w)


def test_generated_files_are_up_to_date(tmp_path):
    for rel in GENERATED:
        (tmp_path / rel).parent.mkdir(parents=True, exist_ok=True)
    shutil.copytree(REPO / "amr_gazebo" / "config", tmp_path / "amr_gazebo" / "config")
    gw.generate(tmp_path / "amr_gazebo" / "config" / "warehouse_layout.yaml", tmp_path)
    for rel in GENERATED:
        assert (tmp_path / rel).read_bytes() == (REPO / rel).read_bytes(), \
            f"{rel} is stale: run python3 tools/generate_warehouse.py"


def test_world_is_valid_xml_with_all_obstacles():
    root = ET.parse(REPO / "amr_gazebo" / "worlds" / "warehouse.sdf").getroot()
    names = {m.get("name") for m in root.iter("model")}
    layout = yaml.safe_load((REPO / "amr_gazebo" / "config" / "warehouse_layout.yaml").read_text())
    assert {o["name"] for o in layout["obstacles"]} <= names
    assert "forklift" in names


def world_to_pixel(meta, img, x, y):
    col = int((x - meta["origin"][0]) / meta["resolution"])
    row = img.shape[0] - 1 - int((y - meta["origin"][1]) / meta["resolution"])
    return row, col


def test_stations_and_spawn_are_free_in_the_map():
    meta = yaml.safe_load((REPO / "amr_navigation" / "maps" / "warehouse.yaml").read_text())
    img = read_pgm(REPO / "amr_navigation" / "maps" / "warehouse.pgm")
    layout = yaml.safe_load((REPO / "amr_gazebo" / "config" / "warehouse_layout.yaml").read_text())
    points = list(layout["stations"].values()) + [layout["robot_spawn"]]
    for x, y, _ in points:
        r, c = world_to_pixel(meta, img, x, y)
        patch = img[r - 7:r + 8, c - 7:c + 8]           # +-0.35 m around the robot centre
        assert (patch == 254).all(), f"station at ({x}, {y}) is not in free space"


def test_obstacles_are_occupied_in_the_map():
    meta = yaml.safe_load((REPO / "amr_navigation" / "maps" / "warehouse.yaml").read_text())
    img = read_pgm(REPO / "amr_navigation" / "maps" / "warehouse.pgm")
    layout = yaml.safe_load((REPO / "amr_gazebo" / "config" / "warehouse_layout.yaml").read_text())
    for o in layout["obstacles"]:
        r, c = world_to_pixel(meta, img, *o["center"])
        assert img[r, c] == 0, o["name"]


def test_masks_mark_the_zones():
    meta = yaml.safe_load((REPO / "amr_navigation" / "maps" / "keepout_mask.yaml").read_text())
    keepout = read_pgm(REPO / "amr_navigation" / "maps" / "keepout_mask.pgm")
    speed = read_pgm(REPO / "amr_navigation" / "maps" / "speed_mask.pgm")
    assert keepout[world_to_pixel(meta, keepout, 17.5, 9.3)] == 0
    assert keepout[world_to_pixel(meta, keepout, 2.0, 4.0)] == 254
    # speed mask (scale mode): value = (255 - p) / 255 * 100 -> 50 % limit means value ~50
    p = int(speed[world_to_pixel(meta, speed, 8.0, 5.5)])
    assert abs((255 - p) / 255 * 100 - 50) < 1.0
    assert speed[world_to_pixel(meta, speed, 2.0, 4.0)] == 255
