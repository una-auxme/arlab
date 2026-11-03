#!/usr/bin/env python3
"""
Merge manually documented and statically inferred ROS 2 connections.

Inputs:
    doc/ros2_interfaces.csv
    doc/ros2_connections_auto.csv

Outputs:
    doc/ros2_architecture_merged.csv
    doc/ros2_architecture_merge_review.csv
    doc/ros2_architecture_merge_report.md

Principles:
- Preserve all manually documented connections.
- Preserve manual integration status.
- Match using interface kind, exact name and component identity.
- Never treat a static match as runtime verification.
- Keep unmatched automatic connections for review.
- Never modify input files.
"""

import csv
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DOC = ROOT / "doc" / "architecture_analysis"
INTERFACES = DOC / "generated" / "interfaces"
VALIDATION = DOC / "generated" / "validation"

MANUAL = DOC / "ros2_interfaces.csv"
AUTOMATIC = INTERFACES / "ros2_connections_auto.csv"
EXCLUSIONS = DOC / "ros2_connection_exclusions.csv"

MERGED = VALIDATION / "ros2_architecture_merged.csv"
REVIEW = VALIDATION / "ros2_architecture_merge_review.csv"
REPORT = VALIDATION / "ros2_architecture_merge_report.md"


# Explicit mapping from source files to functional components.
# These are provisional source-file identities, not verified ROS node IDs.
COMPONENT_MAP = {
    "arlab_computer_vision/object_detection.py": "Computer Vision",
    "arlab_knowledge/database_node.py": "Knowledge Database",
    "arlab_knowledge/knowledge_visualization.py": "Knowledge Visualization",
    "arlab_decision_making/behaviours/tdi_demo/main_controller.py": "Decision Making",
    "arlab_decision_making/behaviours/common/speech.py": "Decision Making",
    "arlab_ui/zirbi_display.py": "PyQt6 UI",
    "arlab_manipulation/orchestrator.py": "Manipulation Orchestrator",
    "arlab_manipulation/force_monitor.py": "Force Monitor",
    "arlab_manipulation/services/gripping_parameter.py": "Gripping Parameter Service",
    "arlab_movement/movement_orchestrator.py": "Movement Orchestrator",
    "arlab_movement/entity_placer.py": "Entity Placer",
    "arlab_templates/template_local_safety.py": "Local Safety Template",
    "arlab_safety/central_safety_node.py": "Central Safety Node",
    "arlab_templates/template_pubsub.py": "Template PubSub",
}

# Known equivalences in the manually maintained architecture.
# No equivalence is assumed for other component names.
COMPONENT_ALIASES = {
    "Computer Vision": "Computer Vision",
    "Knowledge Database": "Knowledge Database",
    "Knowledge Visualization": "Knowledge Visualization",
    "Decision Making": "Decision Making",
    "PyQt6 UI": "PyQt6 UI",
    "Manipulation Orchestrator": "Manipulation Orchestrator",
    "Force Monitor": "Force Monitor",
    "Gripping Parameter Service": "Gripping Parameter Service",
    "Movement Orchestrator": "Movement Orchestrator",
    "Local Safety Template": "Local Safety Template",
    "Central Safety Node": "Central Safety Node",
}


OUTPUT_FIELDS = [
    "source_component",
    "target_component",
    "interface_kind",
    "interface_name",
    "interface_type",
    "integration_status",
    "static_match",
    "merge_status",
    "source_reference",
    "target_reference",
    "source_resolution_status",
    "target_resolution_status",
    "source_activation_condition",
    "target_activation_condition",
    "documentation_source",
    "notes",
]

REVIEW_FIELDS = OUTPUT_FIELDS + ["review_reason"]


def read_csv(path):
    if not path.exists():
        raise FileNotFoundError(f"Missing input file: {path}")

    with path.open(
        "r",
        encoding="utf-8-sig",
        newline="",
    ) as f:
        return list(csv.DictReader(f))


def write_csv(path, rows, fields):
    with path.open(
        "w",
        encoding="utf-8",
        newline="",
    ) as f:
        writer = csv.DictWriter(
            f,
            fieldnames=fields,
            extrasaction="ignore",
        )
        writer.writeheader()
        writer.writerows(rows)


def reference_to_component(reference):
    """
    Resolve a source reference to a functional component.

    Example:
        code/arlab_knowledge/arlab_knowledge/database_node.py:140
        -> Knowledge Database

    A missing mapping remains unresolved.
    """
    if not reference:
        return None

    path = reference.rsplit(":", 1)[0].replace("\\", "/")

    matches = [component for suffix, component in COMPONENT_MAP.items() if path.endswith(suffix)]

    if len(set(matches)) == 1:
        return matches[0]

    return None


def normalize_component(component):
    return COMPONENT_ALIASES.get(component, component)


def connection_key(row):
    return (
        normalize_component(row["source_component"]),
        normalize_component(row["target_component"]),
        row["interface_kind"].strip().lower(),
        row["interface_name"].strip(),
    )


def manual_to_output(row):
    output = {field: "" for field in OUTPUT_FIELDS}

    for field in (
        "source_component",
        "target_component",
        "interface_kind",
        "interface_name",
        "interface_type",
        "integration_status",
        "source_reference",
        "notes",
    ):
        output[field] = row.get(field, "")

    output["static_match"] = "no"
    output["merge_status"] = "manual_only"
    output["documentation_source"] = "manual"

    return output


def automatic_to_output(row):
    output = {field: "" for field in OUTPUT_FIELDS}

    source = reference_to_component(row.get("source_reference", ""))
    target = reference_to_component(row.get("target_reference", ""))

    output["source_component"] = source or row.get("source_component", "")
    output["target_component"] = target or row.get("target_component", "")

    for field in (
        "interface_kind",
        "interface_name",
        "interface_type",
        "source_reference",
        "target_reference",
        "source_resolution_status",
        "target_resolution_status",
        "source_activation_condition",
        "target_activation_condition",
        "notes",
    ):
        output[field] = row.get(field, "")

    # This is a static candidate, not a verified integration.
    output["integration_status"] = "unverified"
    output["static_match"] = "yes"
    output["merge_status"] = "automatic_only"
    output["documentation_source"] = "automatic"

    return output, source is not None and target is not None


def append_note(existing, addition):
    existing = (existing or "").strip()
    addition = (addition or "").strip()

    if not addition:
        return existing

    if not existing:
        return addition

    if addition in existing:
        return existing

    return existing + "; " + addition


def main():
    manual_rows = read_csv(MANUAL)
    automatic_rows = read_csv(AUTOMATIC)
    exclusion_rows = read_csv(EXCLUSIONS)

    exclusion_index = {}
    for row in exclusion_rows:
        key = connection_key(row) + (row["interface_type"].strip(),)
        if key in exclusion_index:
            raise ValueError(f"Duplicate exclusion rule: {key}")
        if not row.get("reason", "").strip():
            raise ValueError(f"Missing exclusion reason: {key}")
        exclusion_index[key] = row["reason"].strip()

    merged = [manual_to_output(row) for row in manual_rows]

    # A key may occur more than once in the manual architecture.
    # We never silently choose between ambiguous matches.
    manual_index = {}

    for index, row in enumerate(merged):
        manual_index.setdefault(connection_key(row), []).append(index)

    review_rows = []
    matched_manual_indices = set()
    stats = Counter()

    for automatic in automatic_rows:
        candidate, components_resolved = automatic_to_output(automatic)

        key = connection_key(candidate)

        if not components_resolved:
            candidate["merge_status"] = "mapping_unresolved"
            candidate["notes"] = append_note(
                candidate["notes"],
                "Functional component mapping incomplete",
            )

            review_rows.append(
                {
                    **candidate,
                    "review_reason": "Component mapping unresolved",
                }
            )

            merged.append(candidate)
            stats["mapping_unresolved"] += 1
            continue

        matches = manual_index.get(key, [])

        if len(matches) > 1:
            candidate["merge_status"] = "ambiguous_match"

            review_rows.append(
                {
                    **candidate,
                    "review_reason": ("Multiple manual connections match this key"),
                }
            )

            merged.append(candidate)
            stats["ambiguous_match"] += 1
            continue

        if len(matches) == 1:
            index = matches[0]
            existing = merged[index]

            if index in matched_manual_indices:
                candidate["merge_status"] = "duplicate_static_candidate"

                review_rows.append(
                    {
                        **candidate,
                        "review_reason": ("Multiple static candidates match one manual connection"),
                    }
                )

                merged.append(candidate)
                stats["duplicate_static_candidate"] += 1
                continue

            matched_manual_indices.add(index)

            existing["static_match"] = "yes"
            existing["merge_status"] = "matched"
            existing["documentation_source"] = "manual+automatic"

            existing["target_reference"] = automatic.get("target_reference", "")

            # Preserve the manual reference, if one exists.
            manual_source_reference = existing.get("source_reference", "")

            auto_source_reference = automatic.get("source_reference", "")

            if not manual_source_reference:
                existing["source_reference"] = auto_source_reference
            elif manual_source_reference != auto_source_reference:
                existing["notes"] = append_note(
                    existing["notes"],
                    "Additional static source reference: " + auto_source_reference,
                )

            for field in (
                "source_resolution_status",
                "target_resolution_status",
                "source_activation_condition",
                "target_activation_condition",
            ):
                existing[field] = automatic.get(field, "")

            existing["notes"] = append_note(
                existing["notes"],
                "Matching source declarations found; runtime communication not verified",
            )

            manual_type = existing.get("interface_type", "").strip()

            automatic_type = automatic.get("interface_type", "").strip()

            if manual_type and automatic_type and manual_type.split("/")[-1] != automatic_type.split("/")[-1]:
                review_rows.append(
                    {
                        **existing,
                        "review_reason": (f"Interface type mismatch: manual={manual_type}; automatic={automatic_type}"),
                    }
                )
                stats["type_mismatch"] += 1

            elif not manual_type:
                existing["interface_type"] = automatic_type

            stats["matched"] += 1
            continue

        # No exact manual match.
        exclusion_key = key + (candidate["interface_type"].strip(),)

        if exclusion_key in exclusion_index:
            candidate["merge_status"] = "excluded"
            candidate["notes"] = append_note(
                candidate["notes"],
                "Excluded from operational architecture: " + exclusion_index[exclusion_key],
            )
            merged.append(candidate)
            stats["excluded"] += 1
            continue

        merged.append(candidate)

        review_rows.append(
            {
                **candidate,
                "review_reason": ("New static connection not present in manual architecture"),
            }
        )

        stats["automatic_only"] += 1

    # Preserve all manual-only relationships.
    for row in merged:
        if row["merge_status"] == "manual_only":
            stats["manual_only"] += 1

    write_csv(MERGED, merged, OUTPUT_FIELDS)
    write_csv(REVIEW, review_rows, REVIEW_FIELDS)

    report = [
        "# ROS 2 Architecture Merge Report",
        "",
        "## Input",
        "",
        f"- Manual connections: {len(manual_rows)}",
        f"- Automatic connection candidates: {len(automatic_rows)}",
        "",
        "## Merge Results",
        "",
        f"- Matched connections: {stats['matched']}",
        f"- Manual-only connections: {stats['manual_only']}",
        f"- Automatic-only connections: {stats['automatic_only']}",
        f"- Excluded connections: {stats['excluded']}",
        f"- Unresolved component mappings: {stats['mapping_unresolved']}",
        f"- Ambiguous matches: {stats['ambiguous_match']}",
        f"- Duplicate static candidates: {stats['duplicate_static_candidate']}",
        f"- Interface type mismatches: {stats['type_mismatch']}",
        "",
        f"- Total merged rows: {len(merged)}",
        f"- Review rows: {len(review_rows)}",
        "",
        "## Interpretation",
        "",
        "- A static match indicates matching declarations in source code, not verified runtime communication.",
        "- Manual integration statuses are preserved.",
        "- Automatic-only connections have integration_status `unverified`.",
        "- Manual-only connections may depend on C++, external packages, dynamic names, or analysis gaps.",
        "- Component names inferred from source files are provisional and do not establish ROS node identity.",
        "",
        "## Files",
        "",
        f"- Merged architecture: `{MERGED.name}`",
        f"- Review queue: `{REVIEW.name}`",
        "",
    ]

    REPORT.write_text(
        "\n".join(report),
        encoding="utf-8",
    )

    print()
    print("ROS 2 Architecture Merge")
    print("=" * 48)

    print(f"Manual connections:        {len(manual_rows)}")
    print(f"Automatic candidates:      {len(automatic_rows)}")
    print("-" * 48)

    print(f"Matched:                   {stats['matched']}")
    print(f"Manual only:               {stats['manual_only']}")
    print(f"Automatic only:            {stats['automatic_only']}")
    print(f"Excluded:                 {stats['excluded']}")
    print(f"Mapping unresolved:        {stats['mapping_unresolved']}")
    print(f"Ambiguous:                 {stats['ambiguous_match']}")
    print(f"Duplicate candidates:      {stats['duplicate_static_candidate']}")
    print(f"Type mismatches:           {stats['type_mismatch']}")

    print("-" * 48)
    print(f"Merged rows:               {len(merged)}")
    print(f"Review rows:               {len(review_rows)}")

    print()
    print(f"Merged: {MERGED}")
    print(f"Review: {REVIEW}")
    print(f"Report: {REPORT}")


if __name__ == "__main__":
    main()
