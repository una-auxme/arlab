#!/usr/bin/env python3
"""Run xacro and post-process the generated URDF for the MIA hand in Gazebo/MoveIt.

The upstream mia_hand_description model couples the middle/ring/little fingers
through URDF <mimic> tags. RViz and MoveIt apply those mimic tags, while the
current Gazebo Sim setup only executes the actuated joints exposed through
ros2_control. That can make RViz show ring/little closing although Gazebo keeps
them open.

For this simulation project we want RViz/MoveIt to match Gazebo: thumb + index +
middle close, ring/little stay open. Therefore this wrapper keeps the mimic tags
for ring/little joints but changes their multiplier to 0.0. Keeping the mimic tag
is intentional: robot_state_publisher and MoveIt still treat those joints as
mimic joints, but their position remains fixed at the offset value.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import xml.etree.ElementTree as ET
from typing import Iterable


def _name_contains_ring_or_little(text: str | None) -> bool:
    if not text:
        return False
    value = text.lower()
    return "ring" in value or "little" in value


def _joint_should_be_frozen(joint: ET.Element) -> bool:
    """Return True for MIA ring/little mimic joints.

    The exact upstream joint names can differ slightly between package versions,
    so this checks the joint name and the parent/child link names.
    """
    if _name_contains_ring_or_little(joint.get("name")):
        return True

    child = joint.find("child")
    if child is not None and _name_contains_ring_or_little(child.get("link")):
        return True

    parent = joint.find("parent")
    if parent is not None and _name_contains_ring_or_little(parent.get("link")):
        return True

    return False


def freeze_ring_little_mimics(urdf_xml: str) -> str:
    """Keep ring/little mimic joints open by setting mimic multiplier to zero."""
    try:
        root = ET.fromstring(urdf_xml)
    except ET.ParseError as exc:
        print(f"[filter_mia_hand_urdf] Failed to parse xacro output: {exc}", file=sys.stderr)
        return urdf_xml

    changed = 0
    for joint in root.findall("joint"):
        mimic = joint.find("mimic")
        if mimic is None:
            continue
        if not _joint_should_be_frozen(joint):
            continue

        # Leave the mimic relationship in place, but make it static-open.
        mimic.set("multiplier", "0.0")
        mimic.set("offset", "0.0")
        changed += 1

    # IMPORTANT: Do not print informational messages to stdout or stderr here.
    # launch.substitutions.Command expects stdout to contain only the URDF XML
    # and treats any stderr output as a launch error.
    return ET.tostring(root, encoding="unicode")


def run_xacro(command: Iterable[str]) -> str:
    result = subprocess.run(
        list(command),
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )

    if result.returncode != 0:
        # Only print stderr when xacro actually failed. For successful xacro runs,
        # stderr must stay empty because launch treats any stderr as an exception.
        if result.stderr:
            print(result.stderr, file=sys.stderr, end="")
        print(
            f"[filter_mia_hand_urdf] xacro command failed with return code {result.returncode}",
            file=sys.stderr,
        )
        sys.exit(result.returncode)

    return result.stdout


def main() -> int:
    parser = argparse.ArgumentParser(description="Run xacro and optionally freeze MIA ring/little mimic joints.")
    parser.add_argument(
        "--filter-mia-ring-little-mimic",
        action="store_true",
        help="Set ring/little MIA mimic multipliers to zero so RViz matches Gazebo.",
    )
    parser.add_argument(
        "xacro_command",
        nargs=argparse.REMAINDER,
        help="xacro executable followed by its normal arguments.",
    )
    args = parser.parse_args()

    if not args.xacro_command:
        print("[filter_mia_hand_urdf] No xacro command provided", file=sys.stderr)
        return 2

    urdf_xml = run_xacro(args.xacro_command)

    if args.filter_mia_ring_little_mimic:
        urdf_xml = freeze_ring_little_mimics(urdf_xml)

    print(urdf_xml)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
