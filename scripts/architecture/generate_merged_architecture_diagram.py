#!/usr/bin/env python3
"""
Generate the merged ROS 2 communication architecture diagram.

Uses the original ARLab diagram design and adds static-analysis
provenance without claiming runtime verification.
"""

import tempfile
import importlib.util
import subprocess
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = Path(__file__).resolve().parent
INPUT = ROOT / "doc" / "architecture_analysis" / "generated" / "validation" / "ros2_architecture_merged.csv"
OUTPUT_DIR = ROOT / "doc" / "architecture_analysis" / "generated" / "diagrams"

DOT_FILE = OUTPUT_DIR / "ros2_architecture_complete.dot"
SVG_FILE = OUTPUT_DIR / "ros2_architecture_complete.svg"


def load_original_generator():
    path = SCRIPT_DIR / "generate_architecture_diagram.py"

    spec = importlib.util.spec_from_file_location(
        "arlab_original_architecture_generator",
        path,
    )

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    return module


original = load_original_generator()

# Preserve the original subsystem structure and extend it.
original.SUBSYSTEMS["Movement"].append("Entity Placer")
original.SUBSYSTEMS["Templates"] = ["Template PubSub"]
original.SUBSYSTEMS["Manipulation"].extend(
    [
        "UR Robot State Helper",
        "UR Dashboard Client",
    ]
)

# Preserve the original communication colors.
# Extend status handling for automatically discovered candidates.
original.STATUS_STYLES["unverified"] = {
    "style": "dashed",
    "label": "Unverified",
}
original.STATUS_STYLES["launch_configured_candidate"] = {
    "style": "dashed",
    "label": "Launch-configured candidate",
}
original.STATUS_STYLES["namespace_dependent"] = {
    "style": "dashed",
    "label": "Namespace-dependent candidate",
}


def evidence_marker(row):
    status = row.get("merge_status", "")

    return {
        "matched": "[S]",
        "manual_only": "[M]",
        "automatic_only": "[A]",
        "namespace_candidate": "[N]",
        "launch_candidate": "[L]",
    }.get(status, "[?]")


def generate_relationships(rows):
    lines = []

    for index, row in enumerate(rows, start=1):
        source = row["source_component"]
        target = row["target_component"]

        kind = row["interface_kind"]
        name = row["interface_name"]
        interface_type = row.get("interface_type", "")
        status = row["integration_status"]

        style = original.get_interface_style(row)
        if row.get("merge_status") == "launch_candidate":
            style = {**style, "color": "#3B82F6", "line_style": "dashed"}
        marker = evidence_marker(row)

        label = f"{marker} {name}\n[{interface_type}]"

        if status not in {"implemented", "configured"}:
            label += f"\n({status})"

        # The publisher is implemented, but the intended RViz consumer
        # has not been established by static or configuration analysis.
        rviz_unverified = (
            source == "Knowledge Visualization"
            and target == "RViz"
            and name == "/arlab/knowledge/visualization"
            and row.get("merge_status") == "manual_only"
        )

        if rviz_unverified:
            label += "\n(RViz subscriber not verified)"
            style = {**style, "line_style": "dashed"}

        tooltip_fields = [
            ("Source", source),
            ("Target", target),
            ("Kind", kind),
            ("Interface", name),
            ("Type", interface_type),
            ("Integration status", status),
            ("Evidence", marker),
            ("Merge status", row.get("merge_status", "")),
            ("Documentation source", row.get("documentation_source", "")),
            ("Static match", row.get("static_match", "")),
            ("Source reference", row.get("source_reference", "")),
            ("Target reference", row.get("target_reference", "")),
            ("Source resolution", row.get("source_resolution_status", "")),
            ("Target resolution", row.get("target_resolution_status", "")),
            ("Source activation", row.get("source_activation_condition", "")),
            ("Target activation", row.get("target_activation_condition", "")),
            ("Notes", row.get("notes", "")),
        ]

        if rviz_unverified:
            tooltip_fields.append(
                (
                    "Verification limitation",
                    "Publisher implemented; RViz subscriber not identified in inspected configuration; runtime communication not verified",
                )
            )

        tooltip = "\n".join(f"{key}: {value}" for key, value in tooltip_fields if value)

        lines.append(
            f"  {original.quote(source)} -> "
            f"{original.quote(target)} ["
            f"label={original.quote(label)}, "
            f"color={original.quote(style['color'])}, "
            f"fontcolor={original.quote(style['color'])}, "
            f"style={original.quote(style['line_style'])}, "
            f"tooltip={original.quote(tooltip)}, "
            f"id={original.quote(f'connection_{index}')}"
            "];"
        )

    return lines


def generate_legend():
    """Reuse the original legend and add evidence information."""

    lines = original.generate_legend()

    # Replace the outdated claim about source-code evidence.
    lines = [
        line.replace(
            "Implemented / Configured (source-code evidence)",
            "Implemented / Configured (documented status)",
        )
        for line in lines
    ]

    evidence_rows = """
    <TR>
        <TD COLSPAN="2" ALIGN="LEFT">
            <B>Evidence / Verification</B>
        </TD>
    </TR>

    <TR>
        <TD>[S]</TD>
        <TD ALIGN="LEFT">
            Documented and statically matched
        </TD>
    </TR>

    <TR>
        <TD>[M]</TD>
        <TD ALIGN="LEFT">
            Manual documentation only
        </TD>
    </TR>

    <TR>
        <TD>[A]</TD>
        <TD ALIGN="LEFT">
            Automatic candidate only
        </TD>
    </TR>

    <TR>
        <TD>[N]</TD>
        <TD ALIGN="LEFT">
            Namespace-dependent candidate
        </TD>
    </TR>

    <TR>
        <TD>[L]</TD>
        <TD ALIGN="LEFT">
            Launch-configured candidate; runtime not verified
        </TD>
    </TR>

    <TR>
        <TD>[O]</TD>
        <TD ALIGN="LEFT">
            Selected endpoint without detected static counterpart
        </TD>
    </TR>

    <TR>
        <TD COLSPAN="2" ALIGN="LEFT">
            No runtime communication verification performed.
        </TD>
    </TR>
"""

    return [
        line.replace(
            "</TABLE>",
            evidence_rows + "\n</TABLE>",
        )
        for line in lines
    ]


# Selected unmatched publishers for architecture review.
# Annotations do not represent verified communication relationships.
OPEN_ENDPOINTS = [
    ("Knowledge Visualization", "/arlab/knowledge/visualization"),
    ("Perception LiDAR Relay", "/relay_scan"),
    ("PyQt6 UI", "/ui_action"),
    ("Whisper", "/whisper/transcript"),
]


def generate_open_endpoint_annotations():
    """Generate source-linked annotations for selected open publishers."""
    import csv

    endpoint_file = ROOT / "doc/architecture_analysis/generated/interfaces/ros2_endpoint_analysis.csv"

    with endpoint_file.open(encoding="utf-8-sig", newline="") as f:
        endpoints = list(csv.DictReader(f))

    lines = []

    for index, (component, topic) in enumerate(OPEN_ENDPOINTS, start=1):
        matches = [
            row
            for row in endpoints
            if row["interface_name"] == topic
            and row["interface_kind"] == "topic"
            and row["role"] == "publisher"
            and row["endpoint_status"] == "no_counterpart_detected"
        ]

        if len(matches) != 1:
            raise RuntimeError(f"Expected exactly one open publisher for {topic}, found {len(matches)}")

        row = matches[0]
        node_id = f"open_endpoint_{index}"

        tooltip = " | ".join(
            [
                "No static counterpart detected",
                f"Package: {row['package']}",
                f"Source: {row['source_file']}:{row['line']}",
                f"Type: {row['interface_type']}",
            ]
        )

        lines.append(
            f"  {original.quote(node_id)} ["
            f"label={original.quote('[O] ' + topic)}, "
            "shape=note, "
            'style="filled,dashed", '
            'fillcolor="#F3F4F6", '
            'color="#808080", '
            'fontcolor="#555555", '
            f"tooltip={original.quote(tooltip)}"
            "];"
        )

        lines.append(
            f"  {original.quote(component)} -> "
            f"{original.quote(node_id)} ["
            "style=dashed, "
            'color="#909090", '
            "arrowhead=none, "
            "constraint=true, "
            f"tooltip={original.quote(tooltip)}"
            "];"
        )

    return lines, len(OPEN_ENDPOINTS)


def compose_svg(graph_path, legend_path, output_path):
    """Combine independently rendered SVGs side by side."""
    import copy
    import xml.etree.ElementTree as ET

    ns = "http://www.w3.org/2000/svg"
    ET.register_namespace("", ns)

    def load_svg(path):
        root = ET.parse(path).getroot()
        viewbox = root.get("viewBox")
        if not viewbox:
            raise RuntimeError(f"Missing SVG viewBox: {path}")
        x, y, width, height = map(float, viewbox.split())
        return root, (x, y, width, height)

    graph, (gx, gy, gw, gh) = load_svg(graph_path)
    legend, (lx, ly, lw, lh) = load_svg(legend_path)

    gap = 35.0
    padding = 15.0

    total_width = lw + gap + gw + 2 * padding
    total_height = max(lh, gh) + 2 * padding

    combined = ET.Element(
        f"{{{ns}}}svg",
        {
            "width": f"{total_width:.2f}pt",
            "height": f"{total_height:.2f}pt",
            "viewBox": f"0 0 {total_width:.2f} {total_height:.2f}",
        },
    )

    # One continuous white background for the complete diagram.
    ET.SubElement(
        combined,
        f"{{{ns}}}rect",
        {
            "x": "0",
            "y": "0",
            "width": f"{total_width:.2f}",
            "height": f"{total_height:.2f}",
            "fill": "#FFFFFF",
        },
    )

    def add_svg(source, x, y, sx, sy):
        group = ET.SubElement(
            combined,
            f"{{{ns}}}g",
            {"transform": f"translate({x:.2f},{y:.2f})"},
        )

        for child in source:
            if child.tag.endswith("title"):
                continue
            if child.tag.endswith("desc"):
                continue
            group.append(copy.deepcopy(child))

    add_svg(
        legend,
        padding - lx,
        padding - ly,
        lw,
        lh,
    )

    add_svg(
        graph,
        padding + lw + gap - gx,
        padding - gy,
        gw,
        gh,
    )

    ET.ElementTree(combined).write(
        output_path,
        encoding="utf-8",
        xml_declaration=True,
    )

    return total_width, total_height


def main():
    # Excluded connections and open endpoints remain in the merged CSV
    # for traceability, but do not represent component-to-component
    # communication relationships.
    non_relationship_statuses = {"excluded", "open_endpoint"}

    rows = [row for row in original.load_interfaces(INPUT) if row.get("merge_status", "") not in non_relationship_statuses]

    # Reuse the original graph styling and subsystem rendering.
    dot = original.generate_dot(rows)

    # Compact layout for the merged architecture.
    dot = dot.replace(
        "    nodesep=0.55,",
        "    nodesep=0.30,",
        1,
    )
    dot = dot.replace(
        "    ranksep=1.1,",
        "    ranksep=0.65,",
        1,
    )
    dot = dot.replace(
        "    splines=spline,",
        "    splines=spline,\n    ratio=compress,",
        1,
    )

    # Replace the original edge block with enriched relationships.
    old_edges = original.generate_relationships(rows)
    new_edges = generate_relationships(rows)

    old_block = "\n".join(old_edges)
    new_block = "\n".join(new_edges)

    if old_block not in dot:
        raise RuntimeError("Could not locate original relationship block.")

    dot = dot.replace(old_block, new_block, 1)

    # Add selected open publishers without changing relationship counts.
    open_annotations, open_count = generate_open_endpoint_annotations()

    closing_brace = dot.rfind("}")

    if closing_brace < 0:
        raise RuntimeError("Could not locate DOT closing brace.")

    dot = dot[:closing_brace] + chr(10).join(open_annotations) + chr(10) + dot[closing_brace:]

    # Replace the original legend block with the extended version.
    old_legend = "\n".join(original.generate_legend())

    if old_legend not in dot:
        raise RuntimeError("Could not locate original legend block.")

    # Render architecture and legend independently.
    dot = dot.replace(old_legend, "", 1)

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    DOT_FILE.write_text(dot, encoding="utf-8")

    # Render intermediate files in a temporary directory.
    # Only the final DOT and SVG files remain in OUTPUT_DIR.
    with tempfile.TemporaryDirectory(prefix="arlab_architecture_") as temp_dir:
        temp_path = Path(temp_dir)

        graph_svg = temp_path / "ros2_architecture_graph.svg"
        legend_svg = temp_path / "ros2_architecture_legend.svg"
        legend_dot = temp_path / "ros2_architecture_legend.dot"

        subprocess.run(
            ["dot", "-Tsvg", str(DOT_FILE), "-o", str(graph_svg)],
            check=True,
        )

        legend_content = "\n".join(
            [
                "digraph Legend {",
                "  graph [bgcolor=white, pad=0.1];",
                "  node [shape=plain];",
                *generate_legend(),
                "}",
            ]
        )

        legend_dot.write_text(legend_content, encoding="utf-8")

        subprocess.run(
            ["dot", "-Tsvg", str(legend_dot), "-o", str(legend_svg)],
            check=True,
        )

        width, height = compose_svg(
            graph_svg,
            legend_svg,
            SVG_FILE,
        )

    print(f"Combined SVG size: {width:.0f} x {height:.0f} pt")

    components = {
        component
        for row in rows
        for component in (
            row["source_component"],
            row["target_component"],
        )
    }

    kinds = Counter(row["interface_kind"] for row in rows)
    evidence = Counter(row.get("merge_status", "") for row in rows)

    print()
    print("ROS 2 Merged Architecture Diagram")
    print("=" * 50)
    print(f"Components:         {len(components)}")
    print(f"Relationships:      {len(rows)}")
    print(f"Topics:             {kinds['topic']}")
    print(f"Services:           {kinds['service']}")
    print(f"Actions:            {kinds['action']}")
    print(f"Static matches:     {evidence['matched']}")
    print(f"Manual only:        {evidence['manual_only']}")
    print(f"Automatic only:     {evidence['automatic_only']}")
    print(f"Namespace candidates: {evidence['namespace_candidate']}")
    print(f"Launch candidates:    {evidence['launch_candidate']}")
    print(f"Open endpoint annotations: {open_count}")
    print()
    print(f"DOT: {DOT_FILE}")
    print(f"SVG: {SVG_FILE}")


if __name__ == "__main__":
    main()
