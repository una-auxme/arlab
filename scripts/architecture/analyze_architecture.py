#!/usr/bin/env python3
"""
Run the reproducible ROS 2 static architecture analysis pipeline.

Usage:
    python3 scripts/architecture/analyze_architecture.py

This script does not modify the manually curated architecture CSV.
"""

import csv
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / "scripts" / "architecture"
DOC = ROOT / "doc" / "architecture_analysis"
INTERFACES = DOC / "generated" / "interfaces"
VALIDATION = DOC / "generated" / "validation"
DIAGRAMS = DOC / "generated" / "diagrams"

STEPS = [
    (
        "Extract ROS2 interfaces",
        "extract_ros2_interfaces_v3.py",
    ),
    (
        "Match communication endpoints",
        "generate_ros2_connections.py",
    ),
    (
        "Merge architecture documentation",
        "merge_ros2_architecture.py",
    ),
    (
        "Generate complete architecture diagram",
        "generate_merged_architecture_diagram.py",
    ),
]

EXPECTED_OUTPUTS = [
    INTERFACES / "ros2_interfaces_v3.csv",
    INTERFACES / "ros2_connections_auto.csv",
    INTERFACES / "ros2_unmatched_interfaces.csv",
    VALIDATION / "ros2_architecture_merged.csv",
    VALIDATION / "ros2_architecture_merge_review.csv",
    VALIDATION / "ros2_architecture_merge_report.md",
    DIAGRAMS / "ros2_architecture_complete.dot",
    DIAGRAMS / "ros2_architecture_complete.svg",
]

REPORT = VALIDATION / "analysis_run_report.md"


def count_csv_rows(path):
    with path.open(encoding="utf-8-sig", newline="") as f:
        return sum(1 for _ in csv.DictReader(f))


def main():
    started = time.monotonic()
    timestamp = datetime.now(timezone.utc).isoformat(
        timespec="seconds"
    )

    print()
    print("ROS 2 Architecture Analysis Pipeline")
    print("=" * 55)
    print(f"Repository: {ROOT}")
    print(f"Timestamp:  {timestamp}")
    print()

    results = []

    for number, (description, script) in enumerate(STEPS, 1):
        script_path = SCRIPTS / script

        if not script_path.is_file():
            raise SystemExit(
                f"ERROR: Missing script: {script_path}"
            )

        print(
            f"[{number}/{len(STEPS)}] {description}",
            flush=True,
        )

        step_started = time.monotonic()

        result = subprocess.run(
            [sys.executable, str(script_path)],
            cwd=ROOT,
            check=False,
        )

        duration = time.monotonic() - step_started

        if result.returncode != 0:
            raise SystemExit(
                f"ERROR: Step failed: {script} "
                f"(exit code {result.returncode})"
            )

        results.append((description, duration))
        print(f"    Completed in {duration:.2f}s\n")

    missing = [
        str(path)
        for path in EXPECTED_OUTPUTS
        if not path.is_file()
    ]

    if missing:
        raise SystemExit(
            "ERROR: Expected output files missing:\n"
            + "\n".join(missing)
        )

    interface_count = count_csv_rows(
        INTERFACES / "ros2_interfaces_v3.csv"
    )

    connection_count = count_csv_rows(
        INTERFACES / "ros2_connections_auto.csv"
    )

    unmatched_count = count_csv_rows(
        INTERFACES / "ros2_unmatched_interfaces.csv"
    )

    merged_count = count_csv_rows(
        VALIDATION / "ros2_architecture_merged.csv"
    )

    review_count = count_csv_rows(
        VALIDATION / "ros2_architecture_merge_review.csv"
    )

    total_duration = time.monotonic() - started

    REPORT.parent.mkdir(parents=True, exist_ok=True)

    report_lines = [
        "# ROS 2 Architecture Analysis Run",
        "",
        f"- Timestamp (UTC): `{timestamp}`",
        f"- Repository: `{ROOT}`",
        f"- Python: `{sys.version.split()[0]}`",
        f"- Duration: `{total_duration:.2f}s`",
        "",
        "## Results",
        "",
        f"- Interface declarations: **{interface_count}**",
        f"- Static connection candidates: **{connection_count}**",
        f"- Unmatched declarations: **{unmatched_count}**",
        f"- Merged architecture relationships: **{merged_count}**",
        f"- Merge review entries: **{review_count}**",
        "",
        "## Pipeline",
        "",
    ]

    for description, duration in results:
        report_lines.append(
            f"- {description}: {duration:.2f}s"
        )

    report_lines.extend([
        "",
        "## Limitations",
        "",
        "- Static source analysis does not verify runtime communication.",
        "- C++ and external packages may not be fully represented.",
        "- Configured or conditional interfaces may differ at runtime.",
        "- Manual architecture entries are preserved, "
        "but are not automatically verified against current code.",
        "- Component mappings are partly manually maintained.",
        "",
    ])

    REPORT.write_text(
        "\n".join(report_lines),
        encoding="utf-8",
    )

    print("=" * 55)
    print("ANALYSIS COMPLETED")
    print("=" * 55)
    print(f"Interfaces:          {interface_count}")
    print(f"Connection candidates: {connection_count}")
    print(f"Unmatched declarations: {unmatched_count}")
    print(f"Merged relationships:   {merged_count}")
    print(f"Review entries:         {review_count}")
    print(f"Total duration:         {total_duration:.2f}s")
    print()
    print(f"Report: {REPORT}")


if __name__ == "__main__":
    main()
