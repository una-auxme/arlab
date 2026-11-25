#!/usr/bin/env python3
"""Classify ROS 2 declarations without an exact static counterpart."""

import csv
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
INTERFACES = ROOT / "doc/architecture_analysis/generated/interfaces"

UNMATCHED = INTERFACES / "ros2_unmatched_interfaces.csv"
NAMESPACE = INTERFACES / "ros2_connections_namespace_candidates.csv"
LAUNCH = INTERFACES / "ros2_connections_launch_candidates.csv"

OUTPUT = INTERFACES / "ros2_endpoint_analysis.csv"
REPORT = INTERFACES / "ros2_endpoint_analysis_report.md"

FIELDS = [
    "package",
    "source_file",
    "line",
    "interface_kind",
    "interface_name",
    "interface_type",
    "role",
    "resolution_status",
    "endpoint_status",
    "candidate_component",
    "candidate_reference",
    "notes",
]


def read_csv(path):
    with path.open(encoding="utf-8-sig", newline="") as f:
        return list(csv.DictReader(f))


def is_unresolved(row):
    return row["resolution_status"].startswith("unresolved")


def reference(row):
    return f"{row['source_file']}:{row['line']}"


def main():
    unmatched = read_csv(UNMATCHED)
    namespace_candidates = read_csv(NAMESPACE)
    launch_candidates = read_csv(LAUNCH)

    candidate_index = {}

    for candidate in namespace_candidates:
        for side, opposite in (
            ("source", "target"),
            ("target", "source"),
        ):
            candidate_index.setdefault(candidate[f"{side}_reference"], []).append(
                (
                    candidate[f"{opposite}_component"],
                    candidate[f"{opposite}_reference"],
                )
            )

    launch_index = {}

    for candidate in launch_candidates:
        for side, opposite in (
            ("source", "target"),
            ("target", "source"),
        ):
            launch_index.setdefault(candidate[f"{side}_reference"], []).append(
                (
                    candidate[f"{opposite}_component"],
                    candidate[f"{opposite}_reference"],
                )
            )

    results = []

    for row in unmatched:
        output = {field: row.get(field, "") for field in FIELDS}

        matches = candidate_index.get(reference(row), [])
        launch_matches = launch_index.get(reference(row), [])

        if is_unresolved(row):
            output["endpoint_status"] = "unresolved"
            output["notes"] = "Interface name cannot be resolved statically"

        elif launch_matches:
            output["endpoint_status"] = "launch_configured_candidate"
            output["candidate_component"] = "; ".join(sorted({component for component, _ in launch_matches}))
            output["candidate_reference"] = "; ".join(sorted({ref for _, ref in launch_matches}))
            output["notes"] = (
                "Compatible counterpart identified using dashboard launch "
                "configuration; shared namespace and remapping assumptions "
                "require verification; runtime not verified"
            )

        elif matches:
            output["endpoint_status"] = "namespace_candidate"
            output["candidate_component"] = "; ".join(sorted({component for component, _ in matches}))
            output["candidate_reference"] = "; ".join(sorted({ref for _, ref in matches}))
            output["notes"] = "Potential counterpart detected; namespace and remapping require verification"

        else:
            output["endpoint_status"] = "no_counterpart_detected"
            output["notes"] = (
                "No exact or namespace-compatible counterpart "
                "detected in analyzed Python/C++ declarations; "
                "external or dynamically configured endpoints "
                "are not excluded"
            )

        results.append(output)

    results.sort(
        key=lambda row: (
            row["endpoint_status"],
            row["interface_kind"],
            row["package"],
            row["interface_name"],
            row["source_file"],
            int(row["line"] or 0),
        )
    )

    with OUTPUT.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDS, lineterminator="\n")
        writer.writeheader()
        writer.writerows(results)

    counts = Counter(row["endpoint_status"] for row in results)

    report = [
        "# ROS 2 Endpoint Analysis",
        "",
        "## Summary",
        "",
        f"- Unmatched declarations: {len(results)}",
        f"- Launch-configured candidates: {counts['launch_configured_candidate']}",
        f"- Namespace candidates: {counts['namespace_candidate']}",
        f"- Unresolved declarations: {counts['unresolved']}",
        f"- No counterpart detected: {counts['no_counterpart_detected']}",
        "",
        "## Interpretation",
        "",
        "Results are based on static Python/C++ analysis.",
        "No counterpart detected does not prove an endpoint is unused.",
        "External nodes, runtime namespaces, remapping and dynamic configuration may affect actual communication.",
        "",
        "## Endpoint details",
        "",
        "| Package | Role | Interface | Status |",
        "|---|---|---|---|",
    ]

    for row in results:
        report.append(f"| {row['package']} | {row['role']} | `{row['interface_name']}` | {row['endpoint_status']} |")

    REPORT.write_text("\n".join(report) + "\n", encoding="utf-8")

    print("ROS 2 Endpoint Analysis")
    print("=" * 48)
    print(f"Unmatched declarations:    {len(results)}")
    print(f"Launch candidates:         {counts['launch_configured_candidate']}")
    print(f"Namespace candidates:      {counts['namespace_candidate']}")
    print(f"Unresolved:                {counts['unresolved']}")
    print(f"No counterpart detected:   {counts['no_counterpart_detected']}")
    print()
    print(f"CSV: {OUTPUT}")
    print(f"Report: {REPORT}")


if __name__ == "__main__":
    main()
