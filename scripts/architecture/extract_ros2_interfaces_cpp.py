#!/usr/bin/env python3
"""Conservative Tree-sitter-based ROS 2 C++ action-server extractor."""

import csv
from pathlib import Path

import tree_sitter_cpp
from tree_sitter import Language, Parser


REPO_ROOT = Path(__file__).resolve().parents[2]
SOURCE_ROOT = REPO_ROOT / "code"

CSV_COLUMNS = [
    "package",
    "source_file",
    "line",
    "interface_kind",
    "interface_name",
    "interface_type",
    "role",
    "resolution_status",
    "resolution_source",
    "resolution_note",
    "activation_condition",
]


def walk(node):
    yield node
    for child in node.named_children:
        yield from walk(child)


def text(node):
    return node.text.decode("utf-8", errors="replace") if node else ""


_CPP_LANGUAGE = Language(tree_sitter_cpp.language())
_PARSE_CACHE = {}


def parse_cpp_file(path):
    """Parse each C++ source file once per extractor process."""
    path = Path(path).resolve()

    if path not in _PARSE_CACHE:
        parser = Parser(_CPP_LANGUAGE)
        tree = parser.parse(path.read_bytes())
        _PARSE_CACHE[path] = (parser, tree)

    return _PARSE_CACHE[path][1]


def resolve_string_expression(node, constants=None):
    """Resolve only simple, statically known C++ string expressions."""
    constants = constants or {}

    if node.type == "string_literal":
        return text(node).strip('"')

    if node.type == "concatenated_string":
        parts = [resolve_string_expression(child, constants) for child in node.named_children]
        if parts and all(part is not None for part in parts):
            return "".join(parts)

    if node.type == "identifier":
        return constants.get(text(node))

    return None


def resolve_ros_parameter_default(interface_node, name_node):
    """Resolve a local ROS parameter default, never a guaranteed runtime name."""
    if name_node.type != "binary_expression":
        return None

    left = name_node.child_by_field_name("left")
    right = name_node.child_by_field_name("right")

    if left is None or right is None or left.type != "identifier":
        return None

    suffix = resolve_string_expression(right)
    if suffix is None:
        return None

    variable = text(left)

    function = interface_node.parent
    while function is not None and function.type != "function_definition":
        function = function.parent

    if function is None or function.has_error:
        return None

    body = function.child_by_field_name("body")
    if body is None:
        return None

    declarations = {}
    assignments = []
    parameter_defaults = {}

    for node in walk(body):
        if node.start_byte >= interface_node.start_byte:
            continue
        if node.has_error:
            continue

        if node.type == "call_expression":
            callee = node.child_by_field_name("function")
            args = node.child_by_field_name("arguments")
            if callee is None or args is None:
                continue

            if callee.type != "field_expression":
                continue

            field = callee.child_by_field_name("field")
            receiver = callee.child_by_field_name("argument")

            if field is not None and text(field) == "declare_parameter" and receiver is not None and text(receiver) == "this":
                values = args.named_children
                if len(values) == 2 and all(v.type == "string_literal" for v in values):
                    key = resolve_string_expression(values[0])
                    default = resolve_string_expression(values[1])
                    parameter_defaults.setdefault(key, []).append((default, node.start_point.row + 1))

        if node.type == "init_declarator":
            declarator = node.child_by_field_name("declarator")
            value = node.child_by_field_name("value")
            if declarator is not None and text(declarator) == variable:
                declarations.setdefault(variable, []).append((value, node.start_point.row + 1))

        if node.type == "assignment_expression":
            target = node.child_by_field_name("left")
            if target is not None and text(target) == variable:
                assignments.append(node.start_point.row + 1)

        if node.type == "update_expression":
            argument = node.child_by_field_name("argument")
            if argument is not None and text(argument) == variable:
                assignments.append(node.start_point.row + 1)

    candidates = declarations.get(variable, [])
    if len(candidates) != 1 or assignments:
        return None

    value, assignment_line = candidates[0]
    if value is None or value.type != "call_expression":
        return None

    callee = value.child_by_field_name("function")
    if callee is None or callee.type != "field_expression":
        return None

    outer_field = callee.child_by_field_name("field")
    inner = callee.child_by_field_name("argument")

    if outer_field is None or text(outer_field) != "as_string" or inner is None or inner.type != "call_expression":
        return None

    inner_callee = inner.child_by_field_name("function")
    inner_args = inner.child_by_field_name("arguments")

    if inner_callee is None or inner_callee.type != "field_expression" or inner_args is None:
        return None

    inner_field = inner_callee.child_by_field_name("field")
    receiver = inner_callee.child_by_field_name("argument")

    if inner_field is None or text(inner_field) != "get_parameter" or receiver is None or text(receiver) != "this":
        return None

    arguments = inner_args.named_children
    if len(arguments) != 1 or arguments[0].type != "string_literal":
        return None

    parameter_name = resolve_string_expression(arguments[0])
    defaults = parameter_defaults.get(parameter_name, [])

    if len(defaults) != 1:
        return None

    default, declaration_line = defaults[0]

    if not (declaration_line < assignment_line <= interface_node.start_point.row + 1):
        return None

    note = (
        f"ROS parameter '{parameter_name}' default declared at line "
        f"{declaration_line}; assigned to '{variable}' at line "
        f"{assignment_line}. Default only: launch parameters and ROS "
        "remapping may change the runtime action name."
    )

    return default + suffix, note


def method_parameter_index(function_node, parameter_name):
    """Return the position of a named parameter in a C++ function definition."""
    if function_node.type != "function_definition":
        return None

    declarator = function_node.child_by_field_name("declarator")

    if declarator is None:
        return None

    parameters = [child for child in walk(declarator) if child.type == "parameter_declaration"]

    for index, parameter in enumerate(parameters):
        identifiers = [text(child) for child in walk(parameter) if child.type == "identifier"]

        if len(identifiers) == 1 and identifiers[0] == parameter_name:
            return index

    return None


def resolve_method_parameter(path, class_name, method_name, parameter_name, max_depth=10):
    """Resolve method arguments through statically traceable same-class calls.

    Returns (values, unresolved), where values maps each string to its
    evidence paths. This is a conservative, file-local analysis.
    """
    path = Path(path).resolve()
    tree = parse_cpp_file(path)

    definitions = {}
    ambiguous = set()

    for node in walk(tree.root_node):
        if node.type != "function_definition" or node.has_error:
            continue

        declarator = node.child_by_field_name("declarator")
        if declarator is None:
            continue

        signature = text(declarator).split("(", 1)[0]

        prefix = f"{class_name}::"
        if not signature.startswith(prefix):
            continue

        name = signature[len(prefix) :]

        if name in definitions:
            ambiguous.add(name)
        else:
            definitions[name] = node

    for name in ambiguous:
        definitions.pop(name, None)

    values = {}
    unresolved = []

    def trace(method, parameter, chain, visited, depth):
        if depth > max_depth or method in visited:
            unresolved.append("Recursion limit or cycle: " + " -> ".join(chain))
            return

        definition = definitions.get(method)

        if definition is None:
            unresolved.append("Missing or ambiguous method: " + method)
            return

        index = method_parameter_index(definition, parameter)

        if index is None:
            unresolved.append(f"Parameter {parameter} not found in {method}")
            return

        next_visited = visited | {method}
        calls_found = False

        for call in walk(tree.root_node):
            if call.type != "call_expression" or call.has_error:
                continue

            function = call.child_by_field_name("function")
            arguments = call.child_by_field_name("arguments")

            if function is None or arguments is None:
                continue

            if function.type != "identifier" or text(function) != method:
                continue

            calls_found = True
            args = arguments.named_children
            location = f"{path.relative_to(REPO_ROOT)}:{call.start_point.row + 1}"
            next_chain = chain + [location]

            if len(args) <= index:
                unresolved.append("Missing argument: " + " -> ".join(next_chain))
                continue

            argument = args[index]
            value = resolve_string_expression(argument)

            if value is not None:
                values.setdefault(value, set()).add(" -> ".join(next_chain))
                continue

            if argument.type != "identifier":
                unresolved.append("Unsupported expression: " + " -> ".join(next_chain))
                continue

            parent = call.parent
            while parent is not None and parent.type != "function_definition":
                parent = parent.parent

            if parent is None:
                unresolved.append("Unknown caller: " + " -> ".join(next_chain))
                continue

            declarator = parent.child_by_field_name("declarator")
            if declarator is None:
                unresolved.append("Unknown caller signature: " + " -> ".join(next_chain))
                continue

            caller_signature = text(declarator).split("(", 1)[0]
            prefix = f"{class_name}::"

            if not caller_signature.startswith(prefix):
                unresolved.append("Caller outside class: " + " -> ".join(next_chain))
                continue

            caller_method = caller_signature[len(prefix) :]

            trace(
                caller_method,
                text(argument),
                next_chain,
                next_visited,
                depth + 1,
            )

        if not calls_found:
            unresolved.append("No call sites found: " + method)

    trace(
        method_name,
        parameter_name,
        [f"{class_name}::{method_name}"],
        set(),
        0,
    )

    return {value: sorted(evidence) for value, evidence in sorted(values.items())}, sorted(set(unresolved))


def resolve_constructor_parameter(package, class_name, parameter_name):
    """Resolve a constructor parameter from unambiguous make_unique calls."""
    package_root = SOURCE_ROOT / package
    parameter_index = None
    constructor_found = False

    for path in sorted(package_root.rglob("*.cpp")):
        tree = parse_cpp_file(path)

        for node in walk(tree.root_node):
            if node.type != "function_definition" or node.has_error:
                continue

            declarator = node.child_by_field_name("declarator")

            if not declarator:
                continue

            function_name = text(declarator).split("(", 1)[0]

            if function_name != f"{class_name}::{class_name}":
                continue

            constructor_found = True

            parameters = [child for child in walk(declarator) if child.type == "parameter_declaration"]

            names = []

            for parameter in parameters:
                identifiers = [text(child) for child in walk(parameter) if child.type == "identifier"]

                names.append(identifiers[0] if len(identifiers) == 1 else None)

            if parameter_name not in names:
                return None

            index = names.index(parameter_name)

            if parameter_index is not None and parameter_index != index:
                return None

            parameter_index = index

    if not constructor_found or parameter_index is None:
        return None

    values = []
    evidence = []

    for path in sorted(package_root.rglob("*.cpp")):
        tree = parse_cpp_file(path)

        for node in walk(tree.root_node):
            if node.type != "call_expression" or node.has_error:
                continue

            function = node.child_by_field_name("function")
            arguments = node.child_by_field_name("arguments")

            if not function or not arguments:
                continue

            if text(function) != f"std::make_unique<{class_name}>":
                continue

            args = arguments.named_children

            if len(args) <= parameter_index:
                return None

            value = resolve_string_expression(args[parameter_index])

            if value is None:
                return None

            values.append(value)
            evidence.append(f"{path.relative_to(REPO_ROOT)}:{node.start_point.row + 1}")

    if not values or len(set(values)) != 1:
        return None

    return values[0], "; ".join(evidence)


def enclosing_constructor(node):
    """Return the enclosing C++ constructor class, if identifiable."""
    current = node.parent

    while current is not None:
        if current.type == "function_definition":
            declarator = current.child_by_field_name("declarator")

            if declarator:
                name = text(declarator).split("(", 1)[0]

                if "::" in name:
                    parts = name.split("::")

                    if len(parts) >= 2 and parts[-1] == parts[-2]:
                        return parts[-1]

            return None

        current = current.parent

    return None


def extract_dashboard_trigger_services(tree, package, relative_path):
    """Extract concrete ROS 2 Trigger services created through the dashboard helper."""
    if str(relative_path) != ("code/manipulator_robot_driver/src/dashboard_client_ros.cpp"):
        return []

    interfaces = []
    seen = set()

    for node in walk(tree.root_node):
        if node.type != "call_expression" or node.has_error:
            continue

        function = node.child_by_field_name("function")
        arguments = node.child_by_field_name("arguments")

        if function is None or arguments is None:
            continue

        if function.type != "identifier":
            continue

        if text(function) != "createDashboardTriggerSrv":
            continue

        args = arguments.named_children

        if not args:
            continue

        name = resolve_string_expression(args[0])

        if name is None:
            continue

        line = node.start_point.row + 1
        key = (str(relative_path), line, name, "server")

        if key in seen:
            continue

        seen.add(key)

        activation_condition = ""

        if name == "~/resume":
            activation_condition = "dashboard_policy == urcl::DashboardClient::ClientPolicy::POLYSCOPE_X"

        interfaces.append(
            {
                "package": package,
                "source_file": str(relative_path),
                "line": line,
                "interface_kind": "service",
                "interface_name": name,
                "interface_type": "std_srvs::srv::Trigger",
                "role": "server",
                "resolution_status": "resolved_static",
                "resolution_source": "cpp_helper_method",
                "resolution_note": (
                    "Created through createDashboardTriggerSrv(); "
                    "implementation: "
                    "code/manipulator_robot_driver/include/"
                    "ur_robot_driver/dashboard_client_ros.hpp:92"
                ),
                "activation_condition": activation_condition,
            }
        )

    return interfaces


def extract_from_file(path):
    path = Path(path).resolve()
    tree = parse_cpp_file(path)

    if tree.root_node.has_error:
        print(f"Warning: partial syntax errors in {path}; extracting only from error-free subtrees")

    relative_path = path.relative_to(REPO_ROOT)
    package = relative_path.parts[1]

    constants = {}

    # Collect simple constexpr character arrays.
    for node in walk(tree.root_node):
        if node.type != "declaration":
            continue

        declaration = text(node)

        if "constexpr" not in declaration:
            continue

        for child in walk(node):
            if child.type != "init_declarator":
                continue

            declarator = child.child_by_field_name("declarator")
            value = child.child_by_field_name("value")

            if not declarator or not value:
                continue

            if value.type != "string_literal":
                continue

            identifiers = [text(part) for part in walk(declarator) if part.type == "identifier"]

            if len(identifiers) == 1:
                constants[identifiers[0]] = text(value).strip('"')

    interfaces = []

    for node in walk(tree.root_node):
        if node.type != "call_expression":
            continue

        # Never extract an interface from a malformed call expression.
        if node.has_error:
            continue

        function = node.child_by_field_name("function")
        arguments = node.child_by_field_name("arguments")

        if not function or not arguments:
            continue

        if function.type not in ("qualified_identifier", "field_expression"):
            continue

        namespace = function.named_children[0]
        template = function.named_children[-1]

        if function.type == "qualified_identifier" and text(namespace) == "rclcpp_action" and template.type == "template_function":
            function_name = template.named_children[0]

            if text(function_name) == "create_server":
                interface_kind = "action"
                role = "server"
                name_index = 1
            elif text(function_name) == "create_client":
                interface_kind = "action"
                role = "client"
                name_index = 1
            else:
                continue

        elif function.type == "field_expression" and template.type == "template_method":
            method_name = text(template.named_children[0])

            method_interfaces = {
                "create_publisher": ("topic", "publisher"),
                "create_subscription": ("topic", "subscriber"),
                "create_service": ("service", "server"),
                "create_client": ("service", "client"),
            }

            if method_name not in method_interfaces:
                continue

            interface_kind, role = method_interfaces[method_name]
            name_index = 0

        else:
            continue

        template_arguments = [child for child in template.named_children if child.type == "template_argument_list"]

        if len(template_arguments) != 1:
            continue

        type_arguments = template_arguments[0].named_children

        if len(type_arguments) != 1:
            continue

        args = arguments.named_children

        if len(args) <= name_index:
            continue

        name_node = args[name_index]

        if name_node.type == "string_literal":
            name = text(name_node).strip('"')
            status = "literal"
            resolution_source = "string_literal"

        elif name_node.type == "concatenated_string" and all(child.type == "string_literal" for child in name_node.named_children):
            name = "".join(text(child).strip('"') for child in name_node.named_children)
            status = "resolved_static"
            resolution_source = "cpp_concatenated_string"

        elif name_node.type == "identifier" and text(name_node) in constants:
            name = constants[text(name_node)]
            status = "resolved_static"
            resolution_source = "cpp_constexpr"

        else:
            name = text(name_node)
            status = "unresolved_static"
            resolution_source = "cpp_expression"

        resolution_note = "C++ Tree-sitter extraction"

        if status == "unresolved_static":
            resolved_default = resolve_ros_parameter_default(node, name_node)

            if resolved_default is not None:
                name, resolution_note = resolved_default
                status = "resolved_default"
                resolution_source = "cpp_ros_parameter_default"

        if status == "unresolved_static" and name_node.type == "identifier":
            constructor_class = enclosing_constructor(node)

            if constructor_class:
                resolved = resolve_constructor_parameter(
                    package,
                    constructor_class,
                    text(name_node),
                )

                if resolved:
                    name, evidence = resolved
                    status = "resolved_static"
                    resolution_source = "cpp_constructor_parameter"
                    resolution_note = "Constructor parameter resolved from: " + evidence

        if status == "unresolved_static" and name_node.type == "identifier" and interface_kind == "action" and role == "client":
            parent = node.parent

            while parent is not None and parent.type != "function_definition":
                parent = parent.parent

            if parent is not None:
                declarator = parent.child_by_field_name("declarator")

                if declarator is not None:
                    signature = text(declarator).split("(", 1)[0]
                    parts = signature.split("::")

                    if len(parts) >= 2:
                        class_name = parts[-2]
                        method_name = parts[-1]

                        values, unresolved = resolve_method_parameter(
                            path,
                            class_name,
                            method_name,
                            text(name_node),
                        )

                        for resolved_name, evidence in values.items():
                            interfaces.append(
                                {
                                    "package": package,
                                    "source_file": str(relative_path),
                                    "line": node.start_point.row + 1,
                                    "interface_kind": interface_kind,
                                    "interface_name": resolved_name,
                                    "interface_type": text(type_arguments[0]),
                                    "role": role,
                                    "resolution_status": "resolved_static",
                                    "resolution_source": "cpp_method_parameter",
                                    "resolution_note": ("Method parameter resolved from: " + "; ".join(evidence)),
                                    "activation_condition": "",
                                }
                            )

                        if values and not unresolved:
                            continue

                        if values and unresolved:
                            resolution_note = "Additional unresolved method paths: " + "; ".join(unresolved)

        interfaces.append(
            {
                "package": package,
                "source_file": str(relative_path),
                "line": node.start_point.row + 1,
                "interface_kind": interface_kind,
                "interface_name": name,
                "interface_type": text(type_arguments[0]),
                "role": role,
                "resolution_status": status,
                "resolution_source": resolution_source,
                "resolution_note": resolution_note,
                "activation_condition": "",
            }
        )

    interfaces.extend(
        extract_dashboard_trigger_services(
            tree,
            package,
            relative_path,
        )
    )

    return interfaces


def main():
    interfaces = []

    for path in sorted(SOURCE_ROOT.rglob("*.cpp")):
        interfaces.extend(extract_from_file(path))

    output = REPO_ROOT / "doc/architecture_analysis/generated/interfaces/" / "ros2_interfaces_cpp.csv"

    output.parent.mkdir(parents=True, exist_ok=True)

    with output.open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=CSV_COLUMNS, lineterminator="\n")
        writer.writeheader()
        writer.writerows(interfaces)

    print(f"C++ interfaces extracted: {len(interfaces)}")
    print(f"Output: {output}")

    for interface in interfaces:
        print(
            f"{interface['interface_kind']} "
            f"{interface['role']}: "
            f"{interface['interface_name']} "
            f"({interface['interface_type']}) "
            f"[{interface['resolution_status']}]"
        )


if __name__ == "__main__":
    main()
