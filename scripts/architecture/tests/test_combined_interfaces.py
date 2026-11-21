import csv
import sys
import tempfile
import unittest
from pathlib import Path


SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS))

from combine_ros2_interfaces import combine_interfaces  # noqa: E402


COLUMNS = [
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


def make_row(package, source_file, line, name, status="literal"):
    return {
        "package": package,
        "source_file": source_file,
        "line": str(line),
        "interface_kind": "action",
        "interface_name": name,
        "interface_type": "ExampleAction",
        "role": "client",
        "resolution_status": status,
        "resolution_source": "test_source",
        "resolution_note": "Test provenance",
        "activation_condition": "",
    }


def write_csv(path, columns, rows):
    with path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)


class CombinedInterfacesTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)

        base = Path(self.temp.name)
        self.python_path = base / "python.csv"
        self.cpp_path = base / "cpp.csv"

    def test_preserves_all_rows_and_metadata(self):
        python_row = make_row("python_package", "node.py", 10, "/example")
        cpp_row = make_row("cpp_package", "node.cpp", 20, "/example", "resolved_default")

        write_csv(self.python_path, COLUMNS, [python_row])
        write_csv(self.cpp_path, COLUMNS, [cpp_row])

        columns, rows = combine_interfaces(self.python_path, self.cpp_path)

        self.assertEqual(columns, COLUMNS)
        self.assertEqual(len(rows), 2)
        self.assertIn(python_row, rows)
        self.assertIn(cpp_row, rows)

    def test_deterministic_order(self):
        first = make_row("z_package", "z.cpp", 30, "/z")
        second = make_row("a_package", "a.py", 10, "/a")

        write_csv(self.python_path, COLUMNS, [first])
        write_csv(self.cpp_path, COLUMNS, [second])

        _, rows = combine_interfaces(self.python_path, self.cpp_path)

        self.assertEqual(
            [row["package"] for row in rows],
            ["a_package", "z_package"],
        )

    def test_rejects_schema_mismatch(self):
        row = make_row("package", "node.py", 1, "/test")

        write_csv(self.python_path, COLUMNS, [row])
        write_csv(
            self.cpp_path,
            COLUMNS[:-1],
            [{key: row[key] for key in COLUMNS[:-1]}],
        )

        with self.assertRaises(ValueError):
            combine_interfaces(self.python_path, self.cpp_path)

    def test_preserves_duplicate_declarations(self):
        row = make_row("package", "node.py", 10, "/same")

        write_csv(self.python_path, COLUMNS, [row])
        write_csv(self.cpp_path, COLUMNS, [row])

        _, rows = combine_interfaces(self.python_path, self.cpp_path)

        self.assertEqual(len(rows), 2)
        self.assertEqual(rows[0], rows[1])


if __name__ == "__main__":
    unittest.main()
