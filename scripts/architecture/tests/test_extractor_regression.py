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
        baseline_path = Path("/tmp/ros2_interfaces_baseline.csv")

        with baseline_path.open(newline="", encoding="utf-8") as file:
            baseline = list(csv.DictReader(file))

        current = []
        for source_file in sorted(SOURCE_ROOT.rglob("*.py")):
            if any(part in {"test", "tests"} for part in source_file.parts):
                continue
            current.extend(extract_from_file(source_file))

        def normalize(row):
            return tuple(sorted((key, str(value)) for key, value in row.items()))

        current_set = {normalize(row) for row in current}
        missing = [row for row in baseline if normalize(row) not in current_set]

        self.assertEqual(missing, [], f"Previously detected interfaces missing: {missing}")


if __name__ == "__main__":
    unittest.main()
