#!/usr/bin/env python3
"""Generate a grouped visualization of unmatched ROS 2 endpoints."""

import csv
import subprocess
from collections import Counter, defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
INPUT = ROOT / "doc/architecture_analysis/generated/interfaces/ros2_endpoint_analysis.csv"
OUTPUT_DIR = ROOT / "doc/architecture_analysis/generated/diagrams"

DOT = OUTPUT_DIR / "ros2_open_endpoints.dot"
SVG = OUTPUT_DIR / "ros2_open_endpoints.svg"

STATUS_COLORS = {
    "no_counterpart_detected": "#FCE8E6",
    "namespace_candidate": "#FFF2CC",
    "launch_configured_candidate": "#DCEEFF",
    "unresolved": "#E2E3E5",
}


def quote(value):
    return '"' + str(value).replace("\\", "\\\\").replace('"', '\\"') + '"'


def main():
    with INPUT.open(encoding="utf-8-sig", newline="") as f:
        rows = list(csv.DictReader(f))

    packages = defaultdict(list)

    for row in rows:
        packages[row["package"]].append(row)

    lines = [
        "digraph ROS2Endpoints {",
        "  graph [",
        "    rankdir=LR,",
        "    bgcolor=white,",
        "    fontname=Helvetica,",
        "    nodesep=0.25,",
        "    ranksep=0.65,",
        "    compound=true,",
        "    newrank=true",
        "  ];",
        "  node [fontname=Helvetica, fontsize=10];",
        "  edge [fontname=Helvetica, fontsize=9];",
    ]

    for package_index, (package, endpoints) in enumerate(sorted(packages.items())):
        lines.extend(
            [
                f"  subgraph cluster_{package_index} {{",
                f"    label={quote(package)};",
                '    color="#B8C4D2";',
                '    style="rounded";',
                '    bgcolor="#F8FAFC";',
            ]
        )

        for endpoint_index, row in enumerate(endpoints):
            node_id = f"endpoint_{package_index}_{endpoint_index}"

            status = row["endpoint_status"]
            color = STATUS_COLORS.get(status, "#FFFFFF")

            label = f"{row['role']}\\n{row['interface_name']}\\n[{row['interface_kind']}]"

            tooltip = (
                f"Package: {package}\n"
                f"Role: {row['role']}\n"
                f"Interface: {row['interface_name']}\n"
                f"Type: {row['interface_type']}\n"
                f"Status: {status}\n"
                f"Source: {row['source_file']}:{row['line']}\n"
                + (f"Candidate: {row['candidate_component']}\n" if row["candidate_component"] else "")
                + f"Notes: {row['notes']}"
            )

            lines.append(
                f"    {quote(node_id)} ["
                f"label={quote(label)}, "
                f"shape=box, "
                f"style={quote('rounded,filled')}, "
                f"fillcolor={quote(color)}, "
                f"tooltip={quote(tooltip)}"
                "];"
            )

        lines.append("  }")

    lines.append("}")

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    DOT.write_text("\n".join(lines) + "\n", encoding="utf-8")

    subprocess.run(
        ["dot", "-Tsvg", str(DOT), "-o", str(SVG)],
        check=True,
    )

    counts = Counter(row["endpoint_status"] for row in rows)

    print("ROS 2 Endpoint Diagram")
    print("=" * 48)
    print(f"Packages:                {len(packages)}")
    print(f"Endpoints:               {len(rows)}")
    print(f"No counterpart detected: {counts['no_counterpart_detected']}")
    print(f"Namespace candidates:    {counts['namespace_candidate']}")
    print(f"Launch candidates:       {counts['launch_configured_candidate']}")
    print(f"Unresolved:              {counts['unresolved']}")
    print()
    print(f"DOT: {DOT}")
    print(f"SVG: {SVG}")


if __name__ == "__main__":
    main()
