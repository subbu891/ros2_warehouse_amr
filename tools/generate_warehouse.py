from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import yaml

REPO = Path(__file__).resolve().parents[1]
DEFAULT_LAYOUT = REPO / "amr_gazebo" / "config" / "warehouse_layout.yaml"


# Perimeter walls as boxes placed just outside the inner floor area
def wall_boxes(layout: dict) -> list[dict]:
    sx, sy = layout["warehouse"]["size"]
    t = layout["warehouse"]["wall_thickness"]
    h = layout["warehouse"]["wall_height"]
    return [
        {"name": "wall_south", "center": [sx / 2, -t / 2], "size": [sx + 2 * t, t], "height": h},
        {"name": "wall_north", "center": [sx / 2, sy + t / 2], "size": [sx + 2 * t, t], "height": h},
        {"name": "wall_west", "center": [-t / 2, sy / 2], "size": [t, sy], "height": h},
        {"name": "wall_east", "center": [sx + t / 2, sy / 2], "size": [t, sy], "height": h},
    ]


def all_boxes(layout: dict) -> list[dict]:
    return wall_boxes(layout) + list(layout["obstacles"])


# Conversion between world coordinates and image pixels; row 0 is the top (max y)
class Grid:
    def __init__(self, layout: dict):
        sx, sy = layout["warehouse"]["size"]
        t = layout["warehouse"]["wall_thickness"]
        self.res = layout["map"]["resolution"]
        pad = layout["map"]["padding"]
        self.origin = (-t - pad, -t - pad)
        self.width = int(round((sx + 2 * (t + pad)) / self.res))
        self.height = int(round((sy + 2 * (t + pad)) / self.res))

    def box_slices(self, xmin, ymin, xmax, ymax):
        c0 = int(math.floor((xmin - self.origin[0]) / self.res))
        c1 = int(math.ceil((xmax - self.origin[0]) / self.res))
        r_top = self.height - int(math.ceil((ymax - self.origin[1]) / self.res))
        r_bot = self.height - int(math.floor((ymin - self.origin[1]) / self.res))
        c0, c1 = max(c0, 0), min(c1, self.width)
        r_top, r_bot = max(r_top, 0), min(r_bot, self.height)
        return slice(r_top, r_bot), slice(c0, c1)


def write_pgm(path: Path, img: np.ndarray) -> None:
    h, w = img.shape
    with open(path, "wb") as f:
        f.write(f"P5\n{w} {h}\n255\n".encode())
        f.write(img.astype(np.uint8).tobytes())


def write_map_yaml(path: Path, image: str, grid: Grid, mode="trinary",
                   occupied=0.65, free=0.25) -> None:
    data = {
        "image": image,
        "mode": mode,
        "resolution": grid.res,
        "origin": [round(grid.origin[0], 4), round(grid.origin[1], 4), 0.0],
        "negate": 0,
        "occupied_thresh": occupied,
        "free_thresh": free,
    }
    path.write_text(yaml.safe_dump(data, sort_keys=False))


# Occupancy map: unknown outside the walls, free floor inside, obstacles occupied
def build_occupancy(layout: dict, grid: Grid) -> np.ndarray:
    sx, sy = layout["warehouse"]["size"]
    img = np.full((grid.height, grid.width), 205, dtype=np.uint8)
    rs, cs = grid.box_slices(0.0, 0.0, sx, sy)
    img[rs, cs] = 254
    for b in all_boxes(layout):
        cx, cy = b["center"]
        hx, hy = b["size"][0] / 2, b["size"][1] / 2
        rs, cs = grid.box_slices(cx - hx, cy - hy, cx + hx, cy + hy)
        img[rs, cs] = 0
    return img


# Keepout mask: zones are lethal for the planner
def build_keepout_mask(layout: dict, grid: Grid) -> np.ndarray:
    img = np.full((grid.height, grid.width), 254, dtype=np.uint8)
    for z in layout.get("keepout_zones", []):
        rs, cs = grid.box_slices(*z["min"], *z["max"])
        img[rs, cs] = 0
    return img


# Speed mask in scale mode: pixel p gives mask value (255 - p) / 255 * 100,
# and the speed filter turns it into limit = 100 - value percent
def build_speed_mask(layout: dict, grid: Grid) -> np.ndarray:
    img = np.full((grid.height, grid.width), 255, dtype=np.uint8)
    for z in layout.get("speed_zones", []):
        value = 100 - int(z["percent"])
        pixel = int(round(255 - value / 100 * 255))
        rs, cs = grid.box_slices(*z["min"], *z["max"])
        img[rs, cs] = pixel
    return img


def box_model(b: dict, static=True, color="0.6 0.6 0.6 1") -> str:
    cx, cy = b["center"]
    sx, sy = b["size"]
    h = b["height"]
    return f"""
    <model name="{b['name']}">
      <static>{'true' if static else 'false'}</static>
      <pose>{cx} {cy} {h / 2} 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>{sx} {sy} {h}</size></box></geometry></collision>
        <visual name="visual">
          <geometry><box><size>{sx} {sy} {h}</size></box></geometry>
          <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
        </visual>
      </link>
    </model>"""


# Forklift driven by velocity commands, publishing its own pose
def forklift_model(layout: dict) -> str:
    f = layout["forklift"]
    sx, sy, h = f["size"]
    x, y = f["start"]
    return f"""
    <model name="forklift">
      <pose>{x} {y} {h / 2 + 0.01} 0 0 0</pose>
      <link name="link">
        <inertial><mass>200</mass>
          <inertia><ixx>50</ixx><iyy>50</iyy><izz>50</izz></inertia>
        </inertial>
        <collision name="collision">
          <geometry><box><size>{sx} {sy} {h}</size></box></geometry>
          <surface><friction><ode><mu>0.0</mu><mu2>0.0</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><box><size>{sx} {sy} {h}</size></box></geometry>
          <material><ambient>1 0.55 0 1</ambient><diffuse>1 0.55 0 1</diffuse></material>
        </visual>
      </link>
      <plugin filename="gz-sim-velocity-control-system" name="gz::sim::systems::VelocityControl"/>
      <plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher">
        <publish_link_pose>false</publish_link_pose>
        <publish_model_pose>true</publish_model_pose>
        <use_pose_vector_msg>false</use_pose_vector_msg>
        <update_frequency>10</update_frequency>
      </plugin>
    </model>"""


# Complete SDF world: physics, sensor systems, light, ground, obstacles and forklift
def build_world(layout: dict) -> str:
    colors = {"rack": "0.2 0.35 0.8 1", "charger_dock": "0.1 0.8 0.2 1", "pallets": "0.6 0.4 0.2 1"}
    models = []
    for b in all_boxes(layout):
        color = next((c for k, c in colors.items() if b["name"].startswith(k)), "0.7 0.7 0.7 1")
        models.append(box_model(b, color=color))
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <world name="warehouse">
    <physics name="default" type="ignored">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
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
          <material><ambient>0.85 0.85 0.8 1</ambient><diffuse>0.85 0.85 0.8 1</diffuse></material>
        </visual>
      </link>
    </model>
{''.join(models)}
{forklift_model(layout)}
  </world>
</sdf>
"""


# Write every generated artefact from one layout file
def generate(layout_path: Path = DEFAULT_LAYOUT, repo: Path = REPO) -> dict:
    layout = yaml.safe_load(layout_path.read_text())
    grid = Grid(layout)

    maps = repo / "amr_navigation" / "maps"
    maps.mkdir(parents=True, exist_ok=True)
    write_pgm(maps / "warehouse.pgm", build_occupancy(layout, grid))
    write_map_yaml(maps / "warehouse.yaml", "warehouse.pgm", grid)
    write_pgm(maps / "keepout_mask.pgm", build_keepout_mask(layout, grid))
    write_map_yaml(maps / "keepout_mask.yaml", "keepout_mask.pgm", grid)
    write_pgm(maps / "speed_mask.pgm", build_speed_mask(layout, grid))
    write_map_yaml(maps / "speed_mask.yaml", "speed_mask.pgm", grid,
                   mode="scale", occupied=1.0, free=0.0)

    worlds = repo / "amr_gazebo" / "worlds"
    worlds.mkdir(parents=True, exist_ok=True)
    (worlds / "warehouse.sdf").write_text(build_world(layout))

    stations = {name: {"x": p[0], "y": p[1], "yaw": p[2]} for name, p in layout["stations"].items()}
    mission_cfg = repo / "amr_mission" / "config"
    mission_cfg.mkdir(parents=True, exist_ok=True)
    (mission_cfg / "stations.yaml").write_text(yaml.safe_dump({"stations": stations}, sort_keys=False))

    # Dock database file format: docks is a map of dock name to its attributes
    docks = {"docks": {name: {"type": d["type"], "frame": "map", "pose": d["pose"]}
                       for name, d in layout["docks"].items()}}
    nav_cfg = repo / "amr_navigation" / "config"
    nav_cfg.mkdir(parents=True, exist_ok=True)
    (nav_cfg / "docks.yaml").write_text(yaml.safe_dump(docks, sort_keys=False))

    return {"grid": (grid.width, grid.height), "origin": grid.origin}


if __name__ == "__main__":
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_LAYOUT
    info = generate(path)
    print(f"Generated world, maps ({info['grid'][0]}x{info['grid'][1]} px) and station files.")
