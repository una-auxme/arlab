#!/usr/bin/env python3
"""
Combine Python and C++ ROS 2 interface declarations.

The original extractor outputs remain unchanged.
No deduplication or semantic normalization is performed.
"""

import csv
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
INTERFACES = ROOT / "doc" / "architecture_analysis" / "generated" / "interfaces"

PYTHON_INPUT = INTERFACES / "ros2_interfaces_v3.csv"
CPP_INPUT = INTERFACES / "ros2_interfaces_cpp.csv"
OUTPUT = INTERFACES / "ros2_interfaces_combined.csv"


def read_interfaces(path):
    with path.open(encoding="utf-8-sig", newline="") as file:
        reader = csv.DictReader(file)

        if not reader.fieldnames:
            raise ValueError(f"Missing CSV header: {path}")

        columns = list(reader.fieldnames)
        rows = list(reader)

    return columns, rows


def combine_interfaces(python_path, cpp_path):
    python_columns, python_rows = read_interfaces(python_path)
    cpp_columns, cpp_rows = read_interfaces(cpp_path)

    if python_columns != cpp_columns:
        raise ValueError(
            "Python and C++ interface schemas differ:\n"
            f"Python: {python_columns}\n"
            f"C++: {cpp_columns}"
        )

    combined = python_rows + cpp_rows

    combined.sort(
        key=lambda row: (
            row["package"],
            row["source_file"],
            int(row["line"]),
            row["interface_kind"],
            row["interface_name"],
            row["role"],
        )
    )

    return python_columns, combined


def main():
    columns, rows = combine_interfaces(PYTHON_INPUT, CPP_INPUT)

    OUTPUT.parent.mkdir(parents=True, exist_ok=True)

    with OUTPUT.open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)

    print("Combined ROS 2 interfaces")
    print(f"Python input: {PYTHON_INPUT}")
    print(f"C++ input:    {CPP_INPUT}")
    print(f"Output:       {OUTPUT}")
    print(f"Total rows:   {len(rows)}")


if __name__ == "__main__":
    main()
