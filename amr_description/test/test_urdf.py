import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

PKG = Path(__file__).resolve().parents[1]


# Expand the xacro model and return the URDF XML tree
def build_urdf(*args):
    out = subprocess.run(["xacro", str(PKG / "urdf" / "amr.urdf.xacro"), *args],
                         check=True, capture_output=True, text=True)
    return ET.fromstring(out.stdout)


def test_urdf_builds_with_expected_links_and_joints():
    root = build_urdf("controllers_file:=/tmp/c.yaml")
    links = {lk.get("name") for lk in root.iter("link")}
    assert {"base_footprint", "base_link", "laser_link", "imu_link",
            "left_wheel_link", "right_wheel_link"} <= links
    joints = {j.get("name"): j.get("type") for j in root.findall("joint")}
    assert joints["left_wheel_joint"] == "continuous" and joints["right_wheel_joint"] == "continuous"


# Gazebo silently drops links without inertia
def test_every_link_has_inertia_except_frames():
    root = build_urdf("controllers_file:=/tmp/c.yaml")
    for link in root.iter("link"):
        if link.get("name") in ("base_footprint", "camera_optical_frame"):
            continue
        assert link.find("inertial") is not None, f"{link.get('name')} has no inertial"


# A geometry mismatch with the controller config corrupts odometry
def test_wheel_geometry_matches_controller_config():
    root = build_urdf("controllers_file:=/tmp/c.yaml")
    joints = {j.get("name"): j for j in root.findall("joint")}
    y_left = float(joints["left_wheel_joint"].find("origin").get("xyz").split()[1])
    y_right = float(joints["right_wheel_joint"].find("origin").get("xyz").split()[1])
    radius = float(next(lk for lk in root.iter("link") if lk.get("name") == "left_wheel_link")
                   .find("collision/geometry/cylinder").get("radius"))
    cfg = yaml.safe_load((PKG / "config" / "amr_controllers.yaml").read_text())
    ddc = cfg["diff_drive_controller"]["ros__parameters"]
    assert abs((y_left - y_right) - ddc["wheel_separation"]) < 1e-6
    assert abs(radius - ddc["wheel_radius"]) < 1e-6


def test_real_hardware_variant_has_no_gazebo_plugins():
    root = build_urdf("use_sim:=false")
    assert not any(p.get("filename", "").startswith("gz") for p in root.iter("plugin"))
