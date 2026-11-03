#!/usr/bin/env python3
"""Generate a ROS 2 communication architecture diagram from a CSV inventory.

The diagram represents source-code-based communication relationships.
It does not verify whether interfaces are active at runtime.

Author: ARLab Robotics
"""

# ============================================================================
# IMPORTS
# ============================================================================

import argparse
import csv
import html
import shutil
import subprocess

from datetime import datetime
from pathlib import Path


# ============================================================================
# CONSTANTS AND CONFIGURATION
# ============================================================================

REPO_ROOT = Path(__file__).resolve().parents[2]

DEFAULT_INPUT_FILE = REPO_ROOT / "doc" / "architecture_analysis" / "ros2_interfaces.csv"

DEFAULT_OUTPUT_DIR = REPO_ROOT / "doc" / "generated" / "architecture"

INTERFACE_STYLES = {
    "action": {
        "color": "#0077B6",
        "label": "Action",
    },
    "service": {
        "color": "#00854A",
        "label": "Service",
    },
    "topic": {
        "color": "#CD5038",
        "label": "Topic",
    },
}

STATUS_STYLES = {
    "implemented": {
        "style": "solid",
        "label": "Implemented",
    },
    "configured": {
        "style": "solid",
        "label": "Configured",
    },
    "conditional": {
        "style": "dashed",
        "label": "Conditional",
    },
    "unconfirmed": {
        "style": "dashed",
        "label": "Unconfirmed",
    },
    "unconnected": {
        "style": "dotted",
        "label": "Unconnected",
    },
    "template_only": {
        "style": "dotted",
        "label": "Template only",
    },
}

SUBSYSTEMS = {
    "HRI": [
        "PyQt6 UI",
        "Whisper",
        "Text To Speech",
    ],
    "Decision Making": [
        "Decision Making",
    ],
    "Manipulation": [
        "Manipulation Orchestrator",
        "Manipulation C++ Orchestrator",
        "Gripping Parameter Service",
        "Force Monitor",
    ],
    "Movement": [
        "Movement Orchestrator",
    ],
    "Computer Vision": [
        "Computer Vision",
    ],
    "Knowledgebase": [
        "Knowledge Database",
        "Knowledge Visualization",
    ],
    "Perception": [
        "Perception LiDAR Relay",
    ],
    "Safety": [
        "Central Safety Node",
        "Local Safety Template",
    ],
}

EXTERNAL_COMPONENTS = {
    "RViz",
}

UNRESOLVED_COMPONENTS = {
    "Unconnected",
    "Unconfirmed",
}

REQUIRED_COLUMNS = {
    "source_component",
    "target_component",
    "interface_kind",
    "interface_name",
    "integration_status",
}


# ============================================================================
# HELPER FUNCTIONS
# ============================================================================


def quote(value):
    """Escape text for a Graphviz DOT string."""

    value = str(value)

    value = value.replace("\\", "\\\\")
    value = value.replace('"', '\\"')
    value = value.replace("\n", "\\n")

    return f'"{value}"'


def escape_html(value):
    """Escape text for Graphviz HTML-like labels."""

    return html.escape(str(value), quote=True)


def load_interfaces(input_file):
    """Load and validate communication relationships from CSV."""

    with input_file.open(
        "r",
        encoding="utf-8-sig",
        newline="",
    ) as csv_file:
        reader = csv.DictReader(csv_file)

        missing_columns = REQUIRED_COLUMNS - set(reader.fieldnames or [])

        if missing_columns:
            raise ValueError("Missing required CSV columns: " + ", ".join(sorted(missing_columns)))

        interfaces = []

        for row_number, row in enumerate(reader, start=2):
            interface = {key: (value or "").strip() for key, value in row.items() if key is not None}

            for column in REQUIRED_COLUMNS:
                if not interface.get(column):
                    raise ValueError(f"Missing value in column '{column}' at CSV row {row_number}")

            interface["interface_kind"] = interface["interface_kind"].lower()

            interface["integration_status"] = interface["integration_status"].lower()

            interfaces.append(interface)

    return interfaces


def get_all_components(interfaces):
    """Collect all components referenced by communication relationships."""

    components = set()

    for interface in interfaces:
        components.add(interface["source_component"])

        components.add(interface["target_component"])

    return components


def get_component_type(component):
    """Determine the visual category of a component."""

    if component in EXTERNAL_COMPONENTS:
        return "external"

    if component in UNRESOLVED_COMPONENTS:
        return "unresolved"

    return "component"


def get_interface_style(interface):
    """Resolve the visual style of a communication relationship."""

    kind = interface["interface_kind"]
    status = interface["integration_status"]

    kind_style = INTERFACE_STYLES.get(
        kind,
        {
            "color": "#64748B",
            "label": kind,
        },
    )

    status_style = STATUS_STYLES.get(
        status,
        {
            "style": "dashed",
            "label": status,
        },
    )

    return {
        "color": kind_style["color"],
        "line_style": status_style["style"],
        "status_label": status_style["label"],
    }


# ============================================================================
# ARCHITECTURE COMPONENTS
# ============================================================================


def generate_subsystems(components):
    """Generate Graphviz clusters for functional subsystems."""

    lines = []

    grouped_components = set()

    for index, (subsystem, members) in enumerate(SUBSYSTEMS.items()):
        present_members = [member for member in members if member in components]

        if not present_members:
            continue

        lines.extend(
            [
                f"  subgraph cluster_{index} {{",
                f"    label={quote(subsystem)};",
                '    style="rounded,filled";',
                '    color="#CBD5E1";',
                '    fillcolor="#F1F5F9";',
                '    fontname="Helvetica";',
                "    fontsize=14;",
                '    fontcolor="#334155";',
                "    margin=20;",
            ]
        )

        for member in present_members:
            lines.append(f"    {quote(member)};")

            grouped_components.add(member)

        lines.append("  }")
        lines.append("")

    return lines, grouped_components


def generate_external_components(components, grouped_components):
    """Generate nodes outside the functional subsystem clusters."""

    lines = []

    remaining_components = sorted(components - grouped_components)

    for component in remaining_components:
        component_type = get_component_type(component)

        if component_type == "external":
            lines.append(f'  {quote(component)} [shape=ellipse, style="filled", fillcolor="#DBEAFE", color="#60A5FA"];')

        elif component_type == "unresolved":
            lines.append(
                f'  {quote(component)} [shape=box, style="rounded,dashed", fillcolor="#FFFFFF", color="#94A3B8", fontcolor="#64748B"];'
            )

        else:
            lines.append(f"  {quote(component)};")

    return lines


# ============================================================================
# COMMUNICATION RELATIONSHIPS
# ============================================================================


def generate_relationships(interfaces):
    """Generate directed communication edges."""

    lines = []

    for interface in interfaces:
        source = interface["source_component"]
        target = interface["target_component"]

        name = interface["interface_name"]
        status = interface["integration_status"]

        interface_type = interface.get(
            "interface_type",
            "",
        )

        style = get_interface_style(interface)

        edge_label = name

        if status not in {
            "implemented",
            "configured",
        }:
            edge_label += f"\n[{status}]"

        tooltip = f"Interface: {name}\nType: {interface_type}\nStatus: {status}"

        lines.append(
            f"  {quote(source)} -> {quote(target)} ["
            f"label={quote(edge_label)}, "
            f"color={quote(style['color'])}, "
            f"fontcolor={quote(style['color'])}, "
            f"style={quote(style['line_style'])}, "
            f"tooltip={quote(tooltip)}"
            "];"
        )

    return lines


# ============================================================================
# DIAGRAM LEGEND
# ============================================================================


def generate_legend():
    """Generate a compact legend using an HTML-like Graphviz table."""

    legend_html = """
<
<TABLE BORDER="1"
       CELLBORDER="0"
       CELLSPACING="0"
       CELLPADDING="7"
       COLOR="#CBD5E1"
       BGCOLOR="#FFFFFF">

    <TR>
        <TD COLSPAN="2" BGCOLOR="#E2E8F0">
            <B>DIAGRAM LEGEND</B>
        </TD>
    </TR>

    <TR>
        <TD COLSPAN="2" ALIGN="LEFT">
            <B>Communication Type</B>
        </TD>
    </TR>

    <TR>
        <TD>
            <FONT COLOR="#0077B6">━━━━━━</FONT>
        </TD>
        <TD ALIGN="LEFT">Action</TD>
    </TR>

    <TR>
        <TD>
            <FONT COLOR="#00854A">━━━━━━</FONT>
        </TD>
        <TD ALIGN="LEFT">Service</TD>
    </TR>

    <TR>
        <TD>
            <FONT COLOR="#CD5038">━━━━━━</FONT>
        </TD>
        <TD ALIGN="LEFT">Topic</TD>
    </TR>

    <TR>
        <TD COLSPAN="2" ALIGN="LEFT">
            <B>Arrow Direction</B>
        </TD>
    </TR>

    <TR>
        <TD ALIGN="LEFT">Topic</TD>
        <TD ALIGN="LEFT">Publisher to Subscriber</TD>
    </TR>

    <TR>
        <TD ALIGN="LEFT">Service</TD>
        <TD ALIGN="LEFT">Client to Server</TD>
    </TR>

    <TR>
        <TD ALIGN="LEFT">Action</TD>
        <TD ALIGN="LEFT">Client to Server</TD>
    </TR>

    <TR>
        <TD COLSPAN="2" ALIGN="LEFT">
            <B>Integration Status (not runtime verified)</B>
        </TD>
    </TR>

    <TR>
        <TD>━━━━━━</TD>
        <TD ALIGN="LEFT">Implemented / Configured (source-code evidence)</TD>
    </TR>

    <TR>
        <TD>┄ ┄ ┄ ┄</TD>
        <TD ALIGN="LEFT">Conditional / Unconfirmed</TD>
    </TR>

    <TR>
        <TD>· · · · · ·</TD>
        <TD ALIGN="LEFT">Unconnected / Template only</TD>
    </TR>

    <TR>
        <TD COLSPAN="2" ALIGN="LEFT">
            <B>Architecture Elements</B>
        </TD>
    </TR>

    <TR>
        <TD>▭</TD>
        <TD ALIGN="LEFT">Subsystem: grouped functional area</TD>
    </TR>

    <TR>
        <TD>□</TD>
        <TD ALIGN="LEFT">Component: logical software component (may contain multiple ROS 2 nodes)</TD>
    </TR>

    <TR>
        <TD>○</TD>
        <TD ALIGN="LEFT">External system or application</TD>
    </TR>

    <TR>
        <TD>┄ □ ┄</TD>
        <TD ALIGN="LEFT">Unresolved communication endpoint</TD>
    </TR>

</TABLE>
>
"""

    return [
        "",
        "  subgraph cluster_diagram_legend {",
        '    label="";',
        '    color="#FFFFFF";',
        '    style="invis";',
        "",
        f"    diagram_legend [shape=plain, label={legend_html}];",
        "  }",
    ]


# ============================================================================
# GRAPHVIZ DIAGRAM GENERATION
# ============================================================================


def generate_dot(interfaces):
    """Build the complete Graphviz DOT document."""

    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    title = f"ROS 2 Communication Architecture\nStatic documentation snapshot: {timestamp}"

    lines = [
        "digraph ROS2Architecture {",
        "",
        "  graph [",
        "    rankdir=LR,",
        '    bgcolor="white",',
        '    fontname="Helvetica",',
        "    fontsize=20,",
        '    fontcolor="#0F172A",',
        "    labelloc=t,",
        f"    label={quote(title)},",
        "    compound=true,",
        "    nodesep=0.55,",
        "    ranksep=1.1,",
        "    splines=spline,",
        "    pad=0.35,",
        "    outputorder=edgesfirst",
        "  ];",
        "",
        "  node [",
        "    shape=box,",
        '    style="rounded,filled",',
        '    fillcolor="#FFFFFF",',
        '    color="#94A3B8",',
        '    fontcolor="#1E293B",',
        '    fontname="Helvetica",',
        "    fontsize=11,",
        '    margin="0.18,0.12"',
        "  ];",
        "",
        "  edge [",
        '    fontname="Helvetica",',
        "    fontsize=9,",
        "    arrowsize=0.75,",
        "    penwidth=1.5",
        "  ];",
        "",
    ]

    components = get_all_components(interfaces)

    subsystem_lines, grouped_components = generate_subsystems(components)

    lines.extend(subsystem_lines)

    lines.extend(
        generate_external_components(
            components,
            grouped_components,
        )
    )

    lines.append("")

    lines.extend(generate_relationships(interfaces))

    lines.extend(generate_legend())

    lines.append("")
    lines.append("}")

    return "\n".join(lines)


# ============================================================================
# OUTPUT GENERATION
# ============================================================================


def generate_diagram(input_file, output_dir):
    """Generate DOT and SVG files."""

    if not input_file.exists():
        raise FileNotFoundError(f"Communication inventory not found: {input_file}")

    if shutil.which("dot") is None:
        raise RuntimeError("Graphviz executable 'dot' was not found.")

    interfaces = load_interfaces(input_file)

    output_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    output_dot = output_dir / "ros2_communication_architecture.dot"

    output_svg = output_dir / "ros2_communication_architecture.svg"

    dot_content = generate_dot(interfaces)

    output_dot.write_text(
        dot_content,
        encoding="utf-8",
    )

    subprocess.run(
        [
            "dot",
            "-Tsvg",
            str(output_dot),
            "-o",
            str(output_svg),
        ],
        check=True,
    )

    print()
    print("ROS 2 architecture diagram generated.")
    print("------------------------------------")
    print(f"Communication relationships: {len(interfaces)}")
    print(f"DOT: {output_dot}")
    print(f"SVG: {output_svg}")
    print()


# ============================================================================
# MAIN PROGRAM
# ============================================================================


def main():
    """Parse command-line arguments and generate the diagram."""

    parser = argparse.ArgumentParser(description=("Generate a ROS 2 communication architecture diagram from a CSV inventory."))

    parser.add_argument(
        "--input",
        type=Path,
        default=DEFAULT_INPUT_FILE,
        help="Path to the communication inventory CSV.",
    )

    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help="Directory for generated architecture files.",
    )

    args = parser.parse_args()

    generate_diagram(
        input_file=args.input,
        output_dir=args.output_dir,
    )


if __name__ == "__main__":
    main()
