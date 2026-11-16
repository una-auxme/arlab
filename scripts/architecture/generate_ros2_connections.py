#!/usr/bin/env python3
"""
Generate candidate ROS 2 communication relationships.

Input:
    doc/ros2_interfaces_v3.csv

Outputs:
    doc/ros2_connections_auto.csv
    doc/ros2_unmatched_interfaces.csv

This is a static source-code analysis.
It does not verify runtime communication.
"""

import csv
from collections import Counter
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]

INPUT_FILE = REPO_ROOT / "doc" / "architecture_analysis" / "generated" / "interfaces" / "ros2_interfaces_combined.csv"

CONNECTIONS_FILE = REPO_ROOT / "doc" / "architecture_analysis" / "generated" / "interfaces" / "ros2_connections_auto.csv"

UNMATCHED_FILE = REPO_ROOT / "doc" / "architecture_analysis" / "generated" / "interfaces" / "ros2_unmatched_interfaces.csv"


ROLE_PAIRS = {
    "topic": ("publisher", "subscriber"),
    "service": ("client", "server"),
    "action": ("client", "server"),
}

CONNECTION_COLUMNS = [
    "source_component",
    "target_component",
    "interface_kind",
    "interface_name",
    "interface_type",
    "integration_status",
    "source_reference",
    "target_reference",
    "source_resolution_status",
    "target_resolution_status",
    "source_activation_condition",
    "target_activation_condition",
    "notes",
]

UNMATCHED_COLUMNS = [
    "package",
    "source_file",
    "line",
    "interface_kind",
    "interface_name",
    "interface_type",
    "role",
    "resolution_status",
    "reason",
]


def load_interfaces():
    with INPUT_FILE.open(
        "r",
        encoding="utf-8-sig",
        newline="",
    ) as file:
        return list(csv.DictReader(file))


def is_resolved(interface):
    return not interface["resolution_status"].startswith("unresolved")


def normalize_type(interface_type):
    """
    Normalize whitespace only.

    Do not assume that identically named Python classes
    from different packages represent the same ROS type.
    """
    return "".join(interface_type.split())


def matching_interfaces(left, right):
    if not is_resolved(left) or not is_resolved(right):
        return False

    if left["interface_kind"] != right["interface_kind"]:
        return False

    if left["interface_name"] != right["interface_name"]:
        return False

    if normalize_type(left["interface_type"]) != normalize_type(right["interface_type"]):
        return False

    return True


def matching_namespace_candidates(left, right):
    """Identify possible topic matches requiring namespace resolution."""
    if not is_resolved(left) or not is_resolved(right):
        return False

    if left["interface_kind"] != "topic" or right["interface_kind"] != "topic":
        return False

    left_name = left["interface_name"]
    right_name = right["interface_name"]

    if not left_name or not right_name or left_name == right_name:
        return False

    # Only consider one absolute and one relative name.
    if left_name.startswith("/") == right_name.startswith("/"):
        return False

    # A relative name containing '/' may refer to a nested namespace.
    # Keep this first candidate detector intentionally conservative.
    relative_name = left_name if not left_name.startswith("/") else right_name
    absolute_name = left_name if left_name.startswith("/") else right_name

    if "/" in relative_name or absolute_name != "/" + relative_name:
        return False

    return normalize_type(left["interface_type"]) == normalize_type(right["interface_type"])


def matching_launch_candidates(client, server):
    """Match services supported by the verified dashboard launch configuration."""
    if not is_resolved(client) or not is_resolved(server):
        return False

    if client["interface_kind"] != "service" or server["interface_kind"] != "service":
        return False

    if client["role"] != "client" or server["role"] != "server":
        return False

    if client["interface_name"] not in {
        "dashboard_client/play",
        "dashboard_client/restart_safety",
    }:
        return False

    if server["interface_name"] != "~/" + client["interface_name"].split("/")[-1]:
        return False

    if not server["source_file"].endswith("/dashboard_client_ros.cpp"):
        return False

    if not client["source_file"].endswith("/robot_state_helper.cpp"):
        return False

    return normalize_type(client["interface_type"]) == normalize_type(server["interface_type"])


def determine_status(source, target):
    """
    Classify static evidence conservatively.

    'conditional' means that at least one declaration
    appears inside a conditional code path.

    'configured' means that at least one name comes
    from a configurable default.
    """

    if source.get("activation_condition") or target.get("activation_condition"):
        return "conditional"

    configurable_statuses = {
        "resolved_parameter_default",
        "resolved_constructor_default",
        "resolved_constructor_argument",
        "resolved_default",
    }

    if source["resolution_status"] in configurable_statuses or target["resolution_status"] in configurable_statuses:
        return "configured"

    return "implemented"


def reference(interface):
    return f"{interface['source_file']}:{interface['line']}"


def make_connection(source, target):
    status = determine_status(source, target)

    notes = ["Statically inferred candidate; runtime not verified"]

    if source["package"] == target["package"]:
        notes.append("Communication within the same package")

    if status == "configured":
        notes.append("At least one interface name is configurable")

    if status == "conditional":
        notes.append("At least one interface declaration is conditional")

    return {
        "source_component": source["package"],
        "target_component": target["package"],
        "interface_kind": source["interface_kind"],
        "interface_name": source["interface_name"],
        "interface_type": source["interface_type"],
        "integration_status": status,
        "source_reference": reference(source),
        "target_reference": reference(target),
        "source_resolution_status": (source["resolution_status"]),
        "target_resolution_status": (target["resolution_status"]),
        "source_activation_condition": source.get("activation_condition", ""),
        "target_activation_condition": target.get("activation_condition", ""),
        "notes": "; ".join(notes),
    }


def write_csv(path, columns, rows):
    path.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    with path.open(
        "w",
        encoding="utf-8",
        newline="",
    ) as file:
        writer = csv.DictWriter(
            file,
            fieldnames=columns,
            lineterminator="\n",
        )

        writer.writeheader()
        writer.writerows(rows)


def main():
    interfaces = load_interfaces()

    connections = []
    matched_indices = set()

    for kind, (source_role, target_role) in ROLE_PAIRS.items():
        sources = [
            (index, interface)
            for index, interface in enumerate(interfaces)
            if (interface["interface_kind"] == kind and interface["role"] == source_role)
        ]

        targets = [
            (index, interface)
            for index, interface in enumerate(interfaces)
            if (interface["interface_kind"] == kind and interface["role"] == target_role)
        ]

        for source_index, source in sources:
            for target_index, target in targets:
                if not matching_interfaces(source, target):
                    continue

                # Avoid matching a declaration to itself.
                if source_index == target_index:
                    continue

                connections.append(make_connection(source, target))

                matched_indices.add(source_index)
                matched_indices.add(target_index)

    connections.sort(
        key=lambda row: (
            row["source_component"],
            row["target_component"],
            row["interface_kind"],
            row["interface_name"],
            row["source_reference"],
            row["target_reference"],
        )
    )

    namespace_candidates = []

    for kind, (source_role, target_role) in ROLE_PAIRS.items():
        if kind != "topic":
            continue

        sources = [interface for interface in interfaces if interface["interface_kind"] == kind and interface["role"] == source_role]
        targets = [interface for interface in interfaces if interface["interface_kind"] == kind and interface["role"] == target_role]

        for source in sources:
            for target in targets:
                if not matching_namespace_candidates(source, target):
                    continue

                candidate = make_connection(source, target)
                candidate["integration_status"] = "namespace_dependent"
                candidate["notes"] = (
                    "Potential connection only; relative ROS 2 name requires namespace and remapping verification; runtime not verified"
                )
                namespace_candidates.append(candidate)

    namespace_candidates.sort(
        key=lambda row: (
            row["source_component"],
            row["target_component"],
            row["interface_name"],
        )
    )

    namespace_file = CONNECTIONS_FILE.with_name("ros2_connections_namespace_candidates.csv")
    write_csv(
        namespace_file,
        CONNECTION_COLUMNS,
        namespace_candidates,
    )

    launch_candidates = []

    clients = [interface for interface in interfaces if interface["interface_kind"] == "service" and interface["role"] == "client"]
    servers = [interface for interface in interfaces if interface["interface_kind"] == "service" and interface["role"] == "server"]

    for client in clients:
        for server in servers:
            if not matching_launch_candidates(client, server):
                continue

            candidate = make_connection(client, server)
            candidate["integration_status"] = "launch_configured_candidate"
            candidate["notes"] = (
                "Dashboard server node name 'dashboard_client' defined in "
                "code/manipulator_robot_driver/launch/ur_dashboard_client.launch.py:57-67; "
                "included conditionally by ur_control.launch.py; "
                "robot_state_helper launched unless mock hardware is enabled; "
                "private server name resolves to the client name if both nodes "
                "share the expected namespace and no relevant remapping applies; "
                "runtime communication not verified"
            )
            launch_candidates.append(candidate)

    launch_candidates.sort(
        key=lambda row: (
            row["interface_name"],
            row["source_reference"],
            row["target_reference"],
        )
    )

    launch_file = CONNECTIONS_FILE.with_name("ros2_connections_launch_candidates.csv")
    write_csv(launch_file, CONNECTION_COLUMNS, launch_candidates)

    unmatched = []

    for index, interface in enumerate(interfaces):
        if index in matched_indices:
            continue

        if not is_resolved(interface):
            reason = "Unresolved interface name"

        else:
            reason = "No matching counterpart found in the analyzed Python and C++ declarations"

        unmatched.append(
            {
                "package": interface["package"],
                "source_file": interface["source_file"],
                "line": interface["line"],
                "interface_kind": interface["interface_kind"],
                "interface_name": interface["interface_name"],
                "interface_type": interface["interface_type"],
                "role": interface["role"],
                "resolution_status": (interface["resolution_status"]),
                "reason": reason,
            }
        )

    write_csv(
        CONNECTIONS_FILE,
        CONNECTION_COLUMNS,
        connections,
    )

    write_csv(
        UNMATCHED_FILE,
        UNMATCHED_COLUMNS,
        unmatched,
    )

    counts = Counter(row["interface_kind"] for row in connections)

    print()
    print("ROS 2 Automatic Connection Analysis")
    print("=" * 45)
    print(f"Interface declarations: {len(interfaces)}")
    print(f"Matched declarations: {len(matched_indices)}")
    print(f"Connection candidates: {len(connections)}")

    for kind in ("topic", "service", "action"):
        print(f"  {kind}: {counts[kind]}")

    print(f"Unmatched declarations: {len(unmatched)}")

    print()
    print(f"Connections: {CONNECTIONS_FILE}")
    print(f"Unmatched: {UNMATCHED_FILE}")


if __name__ == "__main__":
    main()
