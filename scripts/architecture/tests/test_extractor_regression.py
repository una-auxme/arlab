"""Regression tests for the ROS2 interface extractor."""

import csv
import sys
import unittest
from pathlib import Path

ARCHITECTURE_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ARCHITECTURE_DIR))

from extract_ros2_interfaces_v3 import SOURCE_ROOT, extract_from_file  # noqa: E402


class TestExtractorRegression(unittest.TestCase):
    def test_existing_interfaces_are_preserved(self):
        baseline_path = Path(__file__).resolve().parent / "fixtures" / "ros2_interfaces_baseline.csv"

        with baseline_path.open(newline="", encoding="utf-8") as file:
            baseline = list(csv.DictReader(file))

        current = []
        for source_file in sorted(SOURCE_ROOT.rglob("*.py")):
            if any(part in {"test", "tests"} for part in source_file.parts):
                continue
            current.extend(extract_from_file(source_file))

        # Stable interface identity: source line numbers and extraction
        # metadata may change without changing the ROS 2 interface.
        identity_fields = (
            "package",
            "source_file",
            "interface_kind",
            "interface_name",
            "interface_type",
            "role",
        )

        def identity(row):
            return tuple(row[field] for field in identity_fields)

        # Historical interfaces removed from the current ROS 2 source.
        # Keep these exceptions explicit rather than silently updating
        # the baseline to match the latest extractor output.
        removed_interfaces = {
            ("arlab_movement", "code/arlab_movement/arlab_movement/map_save_publisher.py", "topic", "/map_save", "Bool", "publisher"),
            *(
                ("arlab_movement", "code/arlab_movement/arlab_movement/movement_orchestrator.py", "topic", name, "Bool", "subscriber")
                for name in ("localization_bool", "mapping_bool", "nav_bool", "map_save")
            ),
        }

        current_set = {identity(row) for row in current}
        missing = [row for row in baseline if identity(row) not in current_set and identity(row) not in removed_interfaces]

        self.assertEqual(missing, [], f"Previously detected interfaces missing: {missing}")


if __name__ == "__main__":
    unittest.main()
