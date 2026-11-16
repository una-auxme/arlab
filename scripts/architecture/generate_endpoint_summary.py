#!/usr/bin/env python3
"""Generate a compact package-level ROS 2 endpoint summary."""

import csv
import math
import subprocess
from collections import Counter, defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
INPUT = ROOT / "doc/architecture_analysis/generated/interfaces/ros2_endpoint_analysis.csv"
OUTPUT_DIR = ROOT / "doc/architecture_analysis/generated/diagrams"

DOT = OUTPUT_DIR / "ros2_endpoint_summary.dot"
SVG = OUTPUT_DIR / "ros2_endpoint_summary.svg"


def quote(value):
    return '"' + str(value).replace("\\", "\\\\").replace('"', '\\"') + '"'


def main():
    with INPUT.open(encoding="utf-8-sig", newline="") as f:
        rows = list(csv.DictReader(f))

    packages = defaultdict(list)

    for row in rows:
        packages[row["package"]].append(row)

    ordered = sorted(
        packages.items(),
        key=lambda item: (-len(item[1]), item[0]),
    )

    columns = 4
    rows_count = math.ceil(len(ordered) / columns)

    import html

    cells = []

    for package, endpoints in ordered:
        counts = Counter(row["endpoint_status"] for row in endpoints)

        name = html.escape(package)

        cell = (
            '<TD BGCOLOR="#F8FAFC" BORDER="1" COLOR="#B8C4D2" '
            'WIDTH="245" HEIGHT="85" CELLPADDING="10">'
            f"<B>{name}</B><BR/>"
            f"Total: {len(endpoints)}<BR/>"
            f"No counterpart: {counts['no_counterpart_detected']}<BR/>"
            f"Launch: {counts['launch_configured_candidate']} | "
            f"Namespace: {counts['namespace_candidate']} | "
            f"Unresolved: {counts['unresolved']}"
            "</TD>"
        )

        cells.append(cell)

    while len(cells) % columns:
        cells.append('<TD BORDER="0"></TD>')

    table_rows = []

    for index in range(0, len(cells), columns):
        table_rows.append("<TR>" + "".join(cells[index : index + columns]) + "</TR>")

    table = '<TABLE BORDER="0" CELLBORDER="0" CELLSPACING="12">' + "".join(table_rows) + "</TABLE>"

    lines = [
        "digraph EndpointSummary {",
        '  graph [bgcolor="white", pad=0.2];',
        '  node [shape=plain, fontname="Helvetica", fontsize=11];',
        f"  summary [label=<{table}>];",
        "}",
    ]

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    DOT.write_text("\n".join(lines) + "\n", encoding="utf-8")

    subprocess.run(
        ["dot", "-Tsvg", str(DOT), "-o", str(SVG)],
        check=True,
    )

    print("ROS 2 Endpoint Summary")
    print("=" * 48)
    print(f"Packages: {len(packages)}")
    print(f"Endpoints: {len(rows)}")
    print(f"Grid: {columns} columns x {rows_count} rows")
    print()
    print(f"DOT: {DOT}")
    print(f"SVG: {SVG}")


if __name__ == "__main__":
    main()
