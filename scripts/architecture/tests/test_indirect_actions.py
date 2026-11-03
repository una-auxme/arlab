"""Tests for ROS2 action clients created through behaviour wrappers."""

import sys
import unittest
from pathlib import Path

ARCHITECTURE_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ARCHITECTURE_DIR))

from extract_ros2_interfaces_v3 import REPO_ROOT, extract_from_file  # noqa: E402


class TestIndirectActions(unittest.TestCase):
    def test_vision_snapshot_behaviour(self):
        source_file = (
            REPO_ROOT
            / "code/arlab_decision_making/arlab_decision_making/"
            "behaviours/common/vision_snapshot.py"
        )

        interfaces = extract_from_file(source_file)

        matches = [
            interface
            for interface in interfaces
            if interface["interface_kind"] == "action"
            and interface["role"] == "client"
            and interface["interface_name"] == "/vision/snapshot"
            and interface["interface_type"] == "VisionSnapshotAction"
        ]

        self.assertEqual(
            len(matches),
            1,
            "RosActionBehaviour should produce exactly one action client.",
        )


    def test_generic_manipulation_behaviour(self):
        source_file = (
            REPO_ROOT
            / "code/arlab_decision_making/arlab_decision_making/"
            "behaviours/manipulation/generic_manipulation.py"
        )

        interfaces = extract_from_file(source_file)

        matches = [
            interface
            for interface in interfaces
            if interface["interface_kind"] == "action"
            and interface["role"] == "client"
            and interface["interface_name"] == "/manipulation/action"
            and interface["interface_type"] == "ManipulationAction"
        ]

        self.assertEqual(
            len(matches),
            1,
            "GenericManipulation should produce exactly one resolved action client.",
        )


if __name__ == "__main__":
    unittest.main()
