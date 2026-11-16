"""Regression tests for the C++ ROS2 interface extractor."""

import sys
import unittest
from pathlib import Path

ARCHITECTURE_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ARCHITECTURE_DIR))

from extract_ros2_interfaces_cpp import REPO_ROOT, extract_from_file  # noqa: E402


class TestCppExtractor(unittest.TestCase):
    def test_orchestrator_action_server(self):
        path = REPO_ROOT / "code/arlab_manipulation_cpp/src/orchestrator_listener.cpp"

        interfaces = extract_from_file(path)

        matches = [interface for interface in interfaces if interface["interface_name"] == "/orchestrator/action"]

        self.assertEqual(len(matches), 1)

        interface = matches[0]

        self.assertEqual(interface["interface_kind"], "action")
        self.assertEqual(interface["interface_type"], "OrchestratorAction")
        self.assertEqual(interface["role"], "server")
        self.assertEqual(interface["resolution_status"], "resolved_static")

    def test_existing_action_servers(self):
        expected = {
            ("/orchestrator/action", "OrchestratorAction"),
            ("~/set_mode", "ur_dashboard_msgs::action::SetMode"),
            ("~/execute", "TrajectoryUntilAction"),
        }

        found = set()

        for path in (REPO_ROOT / "code").rglob("*.cpp"):
            for interface in extract_from_file(path):
                if interface["interface_kind"] == "action" and interface["role"] == "server":
                    found.add(
                        (
                            interface["interface_name"],
                            interface["interface_type"],
                        )
                    )

        self.assertTrue(
            expected.issubset(found),
            f"Missing action servers: {expected - found}",
        )

    def test_constructor_resolved_service_clients(self):
        expected = {
            "/force_monitor/activate": "ActivateForceMonitor",
            "/mia_hand/data_streams/fingers/forces/switch": "SetBool",
        }

        for filename in ("force_monitor_switch.cpp", "hand_force_switch.cpp"):
            path = REPO_ROOT / "code/arlab_manipulation_cpp/src" / filename

            interfaces = extract_from_file(path)

            matches = [interface for interface in interfaces if interface["interface_kind"] == "service" and interface["role"] == "client"]

            self.assertEqual(len(matches), 1)

            interface = matches[0]

            self.assertIn(interface["interface_name"], expected)
            self.assertEqual(
                interface["interface_type"],
                expected[interface["interface_name"]],
            )
            self.assertEqual(
                interface["resolution_status"],
                "resolved_static",
            )
            self.assertEqual(
                interface["resolution_source"],
                "cpp_constructor_parameter",
            )
            self.assertIn(
                "orchestrator_listener.cpp",
                interface["resolution_note"],
            )

    def test_method_parameter_resolves_multiple_actions(self):
        from extract_ros2_interfaces_cpp import resolve_method_parameter

        path = REPO_ROOT / "code/arlab_manipulation_cpp/src/hand_motion.cpp"

        values, unresolved = resolve_method_parameter(path, "HandMotion", "GetCreateClient", "action_name")

        expected = {
            "/mia_hand/grasps/cylindrical/action",
            "/mia_hand/grasps/pinch/action",
            "/mia_hand/grasps/lateral/action",
            "/mia_hand/grasps/point_up/action",
            "/mia_hand/grasps/point_down/action",
            "/mia_hand/grasps/spherical/action",
            "/mia_hand/grasps/tridigital/action",
        }

        self.assertEqual(set(values), expected)
        self.assertFalse(unresolved)
        self.assertEqual(
            len(values["/mia_hand/grasps/cylindrical/action"]),
            2,
        )

    def test_method_parameter_reports_unknown_argument(self):
        import tempfile
        from pathlib import Path
        from extract_ros2_interfaces_cpp import resolve_method_parameter

        original = REPO_ROOT / "code/arlab_manipulation_cpp/src/hand_motion.cpp"

        with tempfile.TemporaryDirectory(dir=original.parent) as directory:
            path = Path(directory) / "hand_motion_test.cpp"
            path.write_text(
                original.read_text()
                + """
void HandMotion::ArchitectureTestUnknown(
    const std::string &unknown_name)
{
    Grasp(unknown_name, 100, 100, std::chrono::milliseconds(100));
}
""",
                encoding="utf-8",
            )

            values, unresolved = resolve_method_parameter(path, "HandMotion", "GetCreateClient", "action_name")

            self.assertEqual(len(values), 7)
            self.assertTrue(unresolved)
            self.assertTrue(any("ArchitectureTestUnknown" in entry for entry in unresolved))

    def test_method_parameter_rejects_ambiguous_definition(self):
        import tempfile
        from pathlib import Path
        from extract_ros2_interfaces_cpp import resolve_method_parameter

        original = REPO_ROOT / "code/arlab_manipulation_cpp/src/hand_motion.cpp"

        with tempfile.TemporaryDirectory(dir=original.parent) as directory:
            path = Path(directory) / "hand_motion_test.cpp"
            path.write_text(
                original.read_text()
                + """
HandMotion::Client::SharedPtr
HandMotion::GetCreateClient(
    const std::string &action_name, bool alternative)
{
    return nullptr;
}
""",
                encoding="utf-8",
            )

            values, unresolved = resolve_method_parameter(path, "HandMotion", "GetCreateClient", "action_name")

            self.assertFalse(values)
            self.assertTrue(any("Missing or ambiguous method" in entry for entry in unresolved))

    def test_ros_parameter_default_action_client(self):
        path = REPO_ROOT / "code/manipulator_robot_driver/src/trajectory_until_node.cpp"

        interfaces = extract_from_file(path)

        matches = [
            interface
            for interface in interfaces
            if interface["interface_kind"] == "action"
            and interface["role"] == "client"
            and interface["interface_type"] == "FollowJointTrajectory"
        ]

        self.assertEqual(len(matches), 1)

        interface = matches[0]

        self.assertEqual(
            interface["interface_name"],
            "scaled_joint_trajectory_controller/follow_joint_trajectory",
        )
        self.assertEqual(
            interface["resolution_status"],
            "resolved_default",
        )
        self.assertEqual(
            interface["resolution_source"],
            "cpp_ros_parameter_default",
        )
        self.assertIn(
            "motion_controller",
            interface["resolution_note"],
        )

    def test_ros_parameter_default_rejects_ambiguous_declaration(self):
        import tempfile

        original = REPO_ROOT / "code/manipulator_robot_driver/src/trajectory_until_node.cpp"

        source = original.read_text(encoding="utf-8")

        declaration = 'this->declare_parameter("motion_controller", "scaled_joint_trajectory_controller");'

        self.assertEqual(source.count(declaration), 1)

        modified = source.replace(
            declaration,
            declaration + '\\n  this->declare_parameter("motion_controller", "other_controller");',
            1,
        )

        with tempfile.TemporaryDirectory(dir=original.parent) as directory:
            temporary = Path(directory) / "trajectory_until_test.cpp"
            temporary.write_text(modified, encoding="utf-8")

            interfaces = extract_from_file(temporary)

        matches = [
            interface
            for interface in interfaces
            if interface["interface_kind"] == "action"
            and interface["role"] == "client"
            and interface["interface_type"] == "FollowJointTrajectory"
        ]

        self.assertEqual(len(matches), 1)
        self.assertEqual(
            matches[0]["resolution_status"],
            "unresolved_static",
        )

    def test_ros_parameter_default_rejects_modified_variable(self):
        import tempfile

        original = REPO_ROOT / "code/manipulator_robot_driver/src/trajectory_until_node.cpp"

        source = original.read_text(encoding="utf-8")

        assignment = 'std::string motion_controller = this->get_parameter("motion_controller").as_string();'

        self.assertEqual(source.count(assignment), 1)

        modified = source.replace(
            assignment,
            assignment + '\\n  motion_controller += "_alternative";',
            1,
        )

        with tempfile.TemporaryDirectory(dir=original.parent) as directory:
            temporary = Path(directory) / "trajectory_until_test.cpp"
            temporary.write_text(modified, encoding="utf-8")

            interfaces = extract_from_file(temporary)

        matches = [
            interface
            for interface in interfaces
            if interface["interface_kind"] == "action"
            and interface["role"] == "client"
            and interface["interface_type"] == "FollowJointTrajectory"
        ]

        self.assertEqual(len(matches), 1)
        self.assertEqual(
            matches[0]["resolution_status"],
            "unresolved_static",
        )

    def test_extract_mia_hand_actions(self):
        from extract_ros2_interfaces_cpp import extract_from_file

        path = REPO_ROOT / "code/arlab_manipulation_cpp/src/hand_motion.cpp"

        interfaces = extract_from_file(path)

        actions = [
            row
            for row in interfaces
            if row["interface_kind"] == "action" and row["role"] == "client" and row["interface_name"].startswith("/mia_hand/grasps/")
        ]

        expected = {
            "/mia_hand/grasps/cylindrical/action",
            "/mia_hand/grasps/pinch/action",
            "/mia_hand/grasps/lateral/action",
            "/mia_hand/grasps/point_up/action",
            "/mia_hand/grasps/point_down/action",
            "/mia_hand/grasps/spherical/action",
            "/mia_hand/grasps/tridigital/action",
        }

        self.assertEqual(len(actions), 7)
        self.assertEqual(
            {row["interface_name"] for row in actions},
            expected,
        )

        for row in actions:
            self.assertEqual(
                row["resolution_status"],
                "resolved_static",
            )
            self.assertEqual(
                row["resolution_source"],
                "cpp_method_parameter",
            )
            self.assertIn(
                "hand_motion.cpp:",
                row["resolution_note"],
            )

        cylindrical = next(row for row in actions if row["interface_name"] == "/mia_hand/grasps/cylindrical/action")

        self.assertIn(
            "hand_motion.cpp:59",
            cylindrical["resolution_note"],
        )
        self.assertIn(
            "hand_motion.cpp:65",
            cylindrical["resolution_note"],
        )

        self.assertFalse(
            any(
                row["interface_kind"] == "action" and row["role"] == "client" and row["interface_name"] == "action_name"
                for row in interfaces
            ),
            "Resolved parameter must not remain unresolved",
        )


if __name__ == "__main__":
    unittest.main()
