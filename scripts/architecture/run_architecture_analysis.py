#!/usr/bin/env python3
"""Run the complete static ROS 2 architecture analysis pipeline."""

import subprocess
import sys
import time
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent

PIPELINE = [
    ("Extract Python interfaces", "extract_ros2_interfaces_v3.py"),
    ("Extract C++ interfaces", "extract_ros2_interfaces_cpp.py"),
    ("Combine interfaces", "combine_ros2_interfaces.py"),
    ("Detect communication candidates", "generate_ros2_connections.py"),
    ("Merge architecture documentation", "merge_ros2_architecture.py"),
    ("Analyze unmatched endpoints", "analyze_ros2_endpoints.py"),
    ("Generate architecture diagram", "generate_merged_architecture_diagram.py"),
    ("Generate endpoint detail diagram", "generate_endpoint_diagram.py"),
    ("Generate endpoint summary", "generate_endpoint_summary.py"),
]


def main():
    start = time.perf_counter()

    print("ARLab ROS 2 Architecture Analysis")
    print("=" * 60)

    for index, (description, filename) in enumerate(PIPELINE, start=1):
        script = SCRIPT_DIR / filename

        if not script.is_file():
            raise SystemExit(f"Missing pipeline script: {script}")

        print(f"\n[{index}/{len(PIPELINE)}] {description}", flush=True)
        print("-" * 60, flush=True)

        result = subprocess.run(
            [sys.executable, str(script)],
            cwd=SCRIPT_DIR.parents[1],
            check=False,
        )

        if result.returncode != 0:
            print(
                f"\nPipeline failed at step {index}: {filename}",
                file=sys.stderr,
            )
            return result.returncode

    elapsed = time.perf_counter() - start

    print("\n" + "=" * 60)
    print("Architecture analysis completed successfully.")
    print(f"Steps completed: {len(PIPELINE)}")
    print(f"Total runtime: {elapsed:.2f} seconds")

    return 0


if __name__ == "__main__":
    sys.exit(main())
