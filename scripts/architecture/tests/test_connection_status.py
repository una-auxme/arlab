import sys
import unittest
from pathlib import Path


SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS))

from generate_ros2_connections import determine_status  # noqa: E402


class ConnectionStatusTests(unittest.TestCase):
    def make_interface(self, status, condition=""):
        return {
            "resolution_status": status,
            "activation_condition": condition,
        }

    def test_cpp_default_is_configured(self):
        source = self.make_interface("resolved_default")
        target = self.make_interface("literal")

        self.assertEqual(
            determine_status(source, target),
            "configured",
        )

    def test_static_interfaces_are_implemented(self):
        source = self.make_interface("resolved_static")
        target = self.make_interface("literal")

        self.assertEqual(
            determine_status(source, target),
            "implemented",
        )

    def test_conditional_takes_precedence(self):
        source = self.make_interface(
            "resolved_default",
            "self.enabled",
        )
        target = self.make_interface("literal")

        self.assertEqual(
            determine_status(source, target),
            "conditional",
        )


if __name__ == "__main__":
    unittest.main()
