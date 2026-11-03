#!/usr/bin/env python3
"""
ROS 2 Interface Extractor v3.

Conservative static analysis of Python ROS 2 interfaces.

Supports:
- Publishers, subscribers, services and actions
- String literals, constants, variables, attributes and f-strings
- ROS 2 parameter defaults
- Python constructor defaults
- Simple statically identifiable constructor overrides
- Resolution provenance and conditional declarations

Does not execute analyzed source code.

Limitations:
- No general interprocedural analysis
- No automatic launch/YAML parameter override analysis
- No runtime verification
- No complete control-flow or namespace analysis

Output:
    doc/ros2_interfaces_v3.csv
"""

import ast
import csv
from collections import Counter
from dataclasses import dataclass
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SOURCE_ROOT = REPO_ROOT / "code"
OUTPUT_FILE = REPO_ROOT / "doc" / "architecture_analysis" / "generated" / "interfaces" / "ros2_interfaces_v3.csv"

INTERFACE_METHODS = {
    "create_publisher": ("topic", "publisher"),
    "create_subscription": ("topic", "subscriber"),
    "create_client": ("service", "client"),
    "create_service": ("service", "server"),
    "ActionClient": ("action", "client"),
    "ActionServer": ("action", "server"),
}

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


@dataclass(frozen=True)
class ResolvedValue:
    value: str
    status: str
    source: str
    note: str = ""


def expression_to_text(node):
    if node is None:
        return "unresolved"

    try:
        return ast.unparse(node)
    except Exception:
        return "unresolved"


def get_call_name(node):
    if isinstance(node, ast.Name):
        return node.id

    if isinstance(node, ast.Attribute):
        return node.attr

    return None


def get_argument(call, position, keyword_names):
    for keyword in call.keywords:
        if keyword.arg in keyword_names:
            return keyword.value

    if len(call.args) > position:
        return call.args[position]

    return None


def assignment_target_name(target):
    if isinstance(target, ast.Name):
        return target.id

    if isinstance(target, ast.Attribute):
        if isinstance(target.value, ast.Name):
            if target.value.id == "self":
                return f"self.{target.attr}"

    return None


def make_static(value, source="static_assignment"):
    return ResolvedValue(
        value=value,
        status="resolved_static",
        source=source,
    )


def combine_values(left, right):
    if left is None or right is None:
        return None

    statuses = {left.status, right.status}

    if "resolved_parameter_default" in statuses:
        status = "resolved_parameter_default"

    elif "resolved_constructor_default" in statuses:
        status = "resolved_constructor_default"

    elif "resolved_constructor_argument" in statuses:
        status = "resolved_constructor_argument"

    else:
        status = "resolved_static"

    sources = sorted(set(filter(None, [left.source, right.source])))

    notes = sorted(set(filter(None, [left.note, right.note])))

    return ResolvedValue(
        value=left.value + right.value,
        status=status,
        source=";".join(sources),
        note="; ".join(notes),
    )


def get_parameter_name(node):
    """
    Recognize:
        self.get_parameter("name")
        self.get_parameter("name").value
        self.get_parameter("name").get_parameter_value().string_value
    """

    if isinstance(node, ast.Attribute):
        return get_parameter_name(node.value)

    if isinstance(node, ast.Call):
        method = get_call_name(node.func)

        if method == "get_parameter":
            name_node = get_argument(node, 0, ["name"])

            if isinstance(name_node, ast.Constant) and isinstance(name_node.value, str):
                return name_node.value

        if method == "get_parameter_value" and isinstance(node.func, ast.Attribute):
            return get_parameter_name(node.func.value)

    return None


def resolve_expression(node, environment, parameters):
    if node is None:
        return None

    if isinstance(node, ast.Constant):
        if isinstance(node.value, str):
            return ResolvedValue(
                value=node.value,
                status="literal",
                source="string_literal",
            )

        return None

    if isinstance(node, ast.Name):
        return environment.get(node.id)

    parameter_name = get_parameter_name(node)

    if parameter_name is not None:
        return parameters.get(parameter_name)

    if isinstance(node, ast.Attribute):
        if isinstance(node.value, ast.Name):
            return environment.get(f"{node.value.id}.{node.attr}")

        return None

    if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Add):
        left = resolve_expression(node.left, environment, parameters)
        right = resolve_expression(node.right, environment, parameters)

        return combine_values(left, right)

    if isinstance(node, ast.JoinedStr):
        result = ResolvedValue(
            value="",
            status="resolved_static",
            source="f_string",
        )

        for part in node.values:
            if isinstance(part, ast.Constant):
                if not isinstance(part.value, str):
                    return None

                value = make_static(
                    part.value,
                    "f_string",
                )

            elif isinstance(part, ast.FormattedValue):
                if part.format_spec is not None or part.conversion != -1:
                    return None

                value = resolve_expression(part.value, environment, parameters)

            else:
                return None

            result = combine_values(result, value)

            if result is None:
                return None

        return result

    if isinstance(node, ast.Call):
        method = get_call_name(node.func)

        if method == "str" and len(node.args) == 1:
            return resolve_expression(
                node.args[0],
                environment,
                parameters,
            )

    return None


def get_constructor_defaults(function):
    """
    Map constructor argument names to statically known defaults.
    """

    defaults = {}

    positional = function.args.posonlyargs + function.args.args

    offset = len(positional) - len(function.args.defaults)

    for index, default_node in enumerate(function.args.defaults):
        argument = positional[offset + index]

        if isinstance(default_node, ast.Constant) and isinstance(default_node.value, str):
            defaults[argument.arg] = default_node.value

    for argument, default_node in zip(
        function.args.kwonlyargs,
        function.args.kw_defaults,
    ):
        if isinstance(default_node, ast.Constant) and isinstance(default_node.value, str):
            defaults[argument.arg] = default_node.value

    return defaults


def collect_constructor_calls(tree):
    """
    Collect simple direct class constructor calls.

    A class is considered to have a unique explicit argument
    only when every statically found call agrees on that value.
    """

    calls = {}

    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue

        if not isinstance(node.func, ast.Name):
            continue

        calls.setdefault(node.func.id, []).append(node)

    return calls


def resolve_constructor_arguments(
    class_name,
    constructor,
    constructor_calls,
):
    defaults = get_constructor_defaults(constructor)

    calls = constructor_calls.get(class_name, [])

    positional = constructor.args.posonlyargs + constructor.args.args

    positional = [argument for argument in positional if argument.arg != "self"]

    result = {}

    for argument_name, default_value in defaults.items():
        observed = []
        uncertain = False

        for call in calls:
            value_node = None

            for keyword in call.keywords:
                if keyword.arg == argument_name:
                    value_node = keyword.value
                    break

            if value_node is None:
                for index, argument in enumerate(positional):
                    if argument.arg == argument_name:
                        if len(call.args) > index:
                            value_node = call.args[index]
                        break

            if value_node is None:
                observed.append(default_value)

            elif isinstance(value_node, ast.Constant) and isinstance(value_node.value, str):
                observed.append(value_node.value)

            else:
                uncertain = True

        if uncertain:
            continue

        if calls and len(set(observed)) == 1:
            value = observed[0]

            if value == default_value:
                status = "resolved_constructor_default"
                source = "constructor_default"
            else:
                status = "resolved_constructor_argument"
                source = "constructor_call"

            result[argument_name] = ResolvedValue(
                value=value,
                status=status,
                source=source,
                note=(f"Constructor calls examined: {len(calls)}; not runtime-verified"),
            )

        elif not calls:
            result[argument_name] = ResolvedValue(
                value=default_value,
                status="resolved_constructor_default",
                source="constructor_default",
                note=("No direct constructor call identified; default may be overridden"),
            )

    return result


class InterfaceVisitor(ast.NodeVisitor):
    def __init__(
        self,
        package,
        relative_path,
        constructor_calls,
    ):
        self.package = package
        self.relative_path = relative_path
        self.constructor_calls = constructor_calls

        self.interfaces = []

        self.module_environment = {}
        self.environment = self.module_environment

        self.class_attributes = {}
        self.parameters = {}

        self.conditions = []

    def visit_Module(self, node):
        for statement in node.body:
            self.visit(statement)

    def visit_ClassDef(self, node):
        previous_environment = self.environment
        previous_attributes = self.class_attributes
        previous_parameters = self.parameters

        self.class_attributes = {}
        self.parameters = {}

        constructor = next(
            (
                statement
                for statement in node.body
                if isinstance(
                    statement,
                    (ast.FunctionDef, ast.AsyncFunctionDef),
                )
                and statement.name == "__init__"
            ),
            None,
        )

        constructor_values = {}

        if constructor is not None:
            constructor_values = resolve_constructor_arguments(
                node.name,
                constructor,
                self.constructor_calls,
            )

            self._visit_function(
                constructor,
                constructor_values,
            )

        for statement in node.body:
            if isinstance(
                statement,
                (ast.FunctionDef, ast.AsyncFunctionDef),
            ):
                if statement.name != "__init__":
                    self._visit_function(statement)

            elif isinstance(statement, ast.ClassDef):
                self.visit(statement)

        self.environment = previous_environment
        self.class_attributes = previous_attributes
        self.parameters = previous_parameters

    def visit_FunctionDef(self, node):
        self._visit_function(node)

    def visit_AsyncFunctionDef(self, node):
        self._visit_function(node)

    def _visit_function(self, node, argument_values=None):
        previous_environment = self.environment

        self.environment = {
            **self.module_environment,
            **self.class_attributes,
        }

        arguments = node.args.posonlyargs + node.args.args + node.args.kwonlyargs

        for argument in arguments:
            self.environment.pop(argument.arg, None)

        if node.args.vararg:
            self.environment.pop(node.args.vararg.arg, None)

        if node.args.kwarg:
            self.environment.pop(node.args.kwarg.arg, None)

        if argument_values:
            self.environment.update(argument_values)

        for statement in node.body:
            self.visit(statement)

        if node.name == "__init__":
            self.class_attributes = {key: value for key, value in self.environment.items() if key.startswith("self.") and value is not None}

        self.environment = previous_environment

    def visit_Assign(self, node):
        self.visit(node.value)

        value = resolve_expression(
            node.value,
            self.environment,
            self.parameters,
        )

        for target in node.targets:
            name = assignment_target_name(target)

            if name is not None:
                self.environment[name] = value

    def visit_AnnAssign(self, node):
        if node.value is not None:
            self.visit(node.value)

        name = assignment_target_name(node.target)

        if name is not None:
            self.environment[name] = resolve_expression(
                node.value,
                self.environment,
                self.parameters,
            )

    def visit_AugAssign(self, node):
        self.visit(node.value)

        name = assignment_target_name(node.target)

        if name is not None:
            self.environment[name] = None

    def visit_Call(self, node):
        method = get_call_name(node.func)

        if method == "declare_parameter":
            self._record_parameter(node)

        if method in INTERFACE_METHODS:
            interface = self._extract_interface(node)

            if interface is not None:
                self.interfaces.append(interface)

        self.generic_visit(node)

    def _record_parameter(self, node):
        name_node = get_argument(
            node,
            0,
            ["name"],
        )

        default_node = get_argument(
            node,
            1,
            ["value", "default_value"],
        )

        if not (isinstance(name_node, ast.Constant) and isinstance(name_node.value, str)):
            return

        resolved = resolve_expression(
            default_node,
            self.environment,
            self.parameters,
        )

        if resolved is None:
            return

        self.parameters[name_node.value] = ResolvedValue(
            value=resolved.value,
            status="resolved_parameter_default",
            source=f"ros_parameter:{name_node.value}",
            note=("ROS 2 parameter default; launch/YAML overrides not evaluated"),
        )

    def _extract_interface(self, node):
        method = get_call_name(node.func)
        interface_kind, role = INTERFACE_METHODS[method]

        if method in ("create_publisher", "create_subscription"):
            type_node = get_argument(node, 0, ["msg_type"])
            name_node = get_argument(node, 1, ["topic"])

        elif method in ("create_client", "create_service"):
            type_node = get_argument(node, 0, ["srv_type"])
            name_node = get_argument(node, 1, ["srv_name"])

        else:
            type_node = get_argument(node, 1, ["action_type"])
            name_node = get_argument(node, 2, ["action_name"])

        resolved = resolve_expression(
            name_node,
            self.environment,
            self.parameters,
        )

        if resolved is None:
            interface_name = expression_to_text(name_node)

            if isinstance(name_node, ast.Name) and name_node.id in {
                "srv_name",
                "action_name",
            }:
                resolution_status = "unresolved_interprocedural"
                resolution_source = "function_argument"

            else:
                resolution_status = "unresolved"
                resolution_source = "unknown"

            resolution_note = "Name could not be determined statically"

        else:
            interface_name = resolved.value
            resolution_status = resolved.status
            resolution_source = resolved.source
            resolution_note = resolved.note

        return {
            "package": self.package,
            "source_file": str(self.relative_path),
            "line": node.lineno,
            "interface_kind": interface_kind,
            "interface_name": interface_name,
            "interface_type": expression_to_text(type_node),
            "role": role,
            "resolution_status": resolution_status,
            "resolution_source": resolution_source,
            "resolution_note": resolution_note,
            "activation_condition": " AND ".join(self.conditions),
        }

    def visit_If(self, node):
        condition = expression_to_text(node.test)

        self._visit_uncertain_branches(
            (node.body, condition),
            (node.orelse, f"NOT ({condition})"),
        )

    def visit_For(self, node):
        self._visit_uncertain_branches(
            (node.body, "loop"),
            (node.orelse, "loop_else"),
        )

    def visit_AsyncFor(self, node):
        self._visit_uncertain_branches(
            (node.body, "async_loop"),
            (node.orelse, "async_loop_else"),
        )

    def visit_While(self, node):
        self._visit_uncertain_branches(
            (node.body, expression_to_text(node.test)),
            (node.orelse, "while_else"),
        )

    def visit_Try(self, node):
        branches = [
            (node.body, "try"),
            (node.orelse, "try_else"),
            (node.finalbody, "finally"),
        ]

        branches.extend((handler.body, "except") for handler in node.handlers)

        self._visit_uncertain_branches(*branches)

    def _visit_uncertain_branches(self, *branches):
        original_environment = self.environment
        original_parameters = self.parameters

        for statements, condition in branches:
            if not statements:
                continue

            self.environment = original_environment.copy()
            self.parameters = original_parameters.copy()

            self.conditions.append(condition)

            try:
                for statement in statements:
                    self.visit(statement)
            finally:
                self.conditions.pop()

        self.environment = original_environment
        self.parameters = original_parameters


def extract_from_file(source_file):
    try:
        tree = ast.parse(source_file.read_text(encoding="utf-8"))

    except (SyntaxError, UnicodeError) as error:
        print(f"Skipping {source_file}: {error}")
        return []

    relative_path = source_file.relative_to(REPO_ROOT)
    package = relative_path.parts[1]

    constructor_calls = collect_constructor_calls(tree)

    visitor = InterfaceVisitor(
        package,
        relative_path,
        constructor_calls,
    )

    visitor.visit(tree)

    return visitor.interfaces


def main():
    interfaces = []

    for source_file in sorted(SOURCE_ROOT.rglob("*.py")):
        if any(part in {"test", "tests"} for part in source_file.parts):
            continue

        interfaces.extend(extract_from_file(source_file))

    interfaces.sort(
        key=lambda row: (
            row["package"],
            row["source_file"],
            row["line"],
        )
    )

    OUTPUT_FILE.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    with OUTPUT_FILE.open(
        "w",
        newline="",
        encoding="utf-8",
    ) as csv_file:
        writer = csv.DictWriter(
            csv_file,
            fieldnames=CSV_COLUMNS,
        )

        writer.writeheader()
        writer.writerows(interfaces)

    counts = Counter(row["resolution_status"] for row in interfaces)

    print()
    print("ROS 2 Interface Extractor v3")
    print("=" * 45)
    print(f"Extracted interfaces: {len(interfaces)}")

    for status, count in sorted(counts.items()):
        print(f"{status}: {count}")

    unresolved = sum(count for status, count in counts.items() if status.startswith("unresolved"))

    print(f"Resolved: {len(interfaces) - unresolved}")
    print(f"Unresolved: {unresolved}")
    print(f"Output: {OUTPUT_FILE}")


if __name__ == "__main__":
    main()
