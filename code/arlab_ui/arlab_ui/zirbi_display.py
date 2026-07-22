"""ROS 2 integrated PyQt6 HRI display prototype for Zirbi.

This module implements the current robot-facing display prototype as a ROS 2
Python package entry point. It combines a PyQt6 touchscreen-style user
interface with a small ROS 2 bridge running in a background thread.

Current ROS 2 integration:
- subscribes to /tts_output (std_msgs/msg/String) and displays speech output,
- publishes prototype UI feedback to /ui_action (std_msgs/msg/String),
- subscribes to /camera_gripper/color/image_raw (sensor_msgs/msg/Image) and
  renders incoming camera frames in the runtime view.

The /ui_action topic is a temporary prototype feedback channel. It is not the
final structured HRI interface. The final interface should be aligned with the
Decision Making and HRI Output architecture.
"""

# ============================================================================
# IMPORTS
# ============================================================================

import sys
from queue import Empty, Queue
from dataclasses import dataclass
from pathlib import Path

from PyQt6.QtCore import Qt, QRectF, QThread, QTimer, QTime, pyqtSignal
from PyQt6.QtGui import QCursor, QImage, QPainter, QPainterPath, QPixmap
from PyQt6.QtWidgets import (
    QApplication,
    QFrame,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

try:
    import rclpy
    from std_msgs.msg import String

    ROS_AVAILABLE = True
except ImportError:
    rclpy = None
    String = None
    ROS_AVAILABLE = False

try:
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image

    CAMERA_AVAILABLE = True
except ImportError:
    CvBridge = None
    Image = None
    CAMERA_AVAILABLE = False


# ============================================================================
# DATA MODELS
# ============================================================================


@dataclass
class RuntimeState:
    """Internal UI-side representation of the current HRI output state.

    The final HRI Output Interface from Decision Making is not finalized yet.
    This dataclass therefore models the information the display currently needs:
    the active challenge, visible status text, speech text, next action,
    available button labels and progress state.

    It keeps the prototype structured and makes the future interface boundary
    explicit instead of spreading task state across unrelated UI widgets.
    """

    challenge: str
    subtask: str
    message_type: str
    display_text: str
    speech_text: str
    next_action: str
    action_labels: tuple[str, str, str]
    progress: list[tuple[str, str, str]]


# ============================================================================
# CONSTANTS AND CONFIGURATION
# ============================================================================

WINDOW_WIDTH = 1280
WINDOW_HEIGHT = 720

# These labels are prototype defaults.
# In the final system, the available actions should be generated dynamically
# from the current challenge, task state, and HRI output message type.
DEFAULT_ACTION_LABELS = (
    "✓ Confirm",
    "↻ Repeat",
    "✕ Cancel",
)

# Assistance states already use a more specific placeholder variant.
# Later, these labels should also come from the runtime state or HRI output interface.
ASSISTANCE_ACTION_LABELS = (
    "✓ Help Done",
    "↻ Repeat Instruction",
    "✕ Cancel",
)

PAGE_HOME = 0
PAGE_PICK_CONTROL = 1
PAGE_RUNTIME = 2
PAGE_LAUNDRY_CONTROL = 3
PAGE_INITIAL_SCAN = 4
PAGE_MAINTENANCE = 5


# ============================================================================
# UI COMPONENTS
# ============================================================================


class CameraView(QWidget):
    """Rounded camera panel with placeholder fallback and live frame updates."""

    def __init__(self, image_path):
        super().__init__()
        self.image_path = image_path
        self.pixmap = QPixmap(str(image_path))
        self.setMinimumSize(300, 200)

    def set_frame(self, image):
        self.pixmap = QPixmap.fromImage(image)
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.setRenderHint(QPainter.RenderHint.SmoothPixmapTransform)

        rect = QRectF(self.rect())
        radius = 22

        path = QPainterPath()
        path.addRoundedRect(rect, radius, radius)

        painter.setClipPath(path)
        painter.fillRect(self.rect(), Qt.GlobalColor.black)

        if not self.pixmap.isNull():
            # The image is cropped to fill the available camera area.
            scaled = self.pixmap.scaled(
                self.size(),
                Qt.AspectRatioMode.KeepAspectRatioByExpanding,
                Qt.TransformationMode.SmoothTransformation,
            )

            x = (self.width() - scaled.width()) // 2
            y = (self.height() - scaled.height()) // 2

            painter.drawPixmap(x, y, scaled)
        else:
            painter.setPen(Qt.GlobalColor.white)
            painter.drawText(
                self.rect(),
                Qt.AlignmentFlag.AlignCenter,
                "No camera image",
            )


# ============================================================================
# ROS 2 INTEGRATION
# ============================================================================

# The PyQt event loop must stay responsive, so ROS 2 spinning runs in a
# dedicated QThread. Qt signals are used to safely pass data back to the UI.
# This keeps ROS communication separate from rendering and button handling.


class RosInterfaceThread(QThread):
    """Small ROS 2 bridge used by the display prototype.

    The node created in this thread is named ``zirbi_display_ros_interface``.
    It acts as the current prototype counterpart for display I/O:

    - ROS -> UI: speech text from /tts_output
    - UI -> ROS: button feedback on /ui_action
    - ROS -> UI: camera frames from /camera_gripper/color/image_raw

    The bridge is intentionally simple. It uses std_msgs/msg/String for UI
    actions until a structured HRI feedback message is defined.
    """

    speech_received = pyqtSignal(str)
    status_changed = pyqtSignal(str)
    action_published = pyqtSignal(str)
    camera_frame_received = pyqtSignal(QImage)
    camera_status_changed = pyqtSignal(str)

    def __init__(
        self,
        tts_topic_name="/tts_output",
        action_topic_name="/ui_action",
        camera_topic_name="/camera_gripper/color/image_raw",
    ):
        super().__init__()
        self.tts_topic_name = tts_topic_name
        self.action_topic_name = action_topic_name
        self.camera_topic_name = camera_topic_name
        self._running = True
        self.node = None
        self.action_publisher = None
        self.action_queue = Queue()
        self.bridge = CvBridge() if CAMERA_AVAILABLE else None

    def run(self):
        if not ROS_AVAILABLE:
            self.status_changed.emit("ROS integration: unavailable in current Python environment")
            return

        try:
            rclpy.init(args=None)
            self.node = rclpy.create_node("zirbi_display_ros_interface")

            self.node.create_subscription(
                String,
                self.tts_topic_name,
                self.handle_tts_message,
                10,
            )

            self.action_publisher = self.node.create_publisher(
                String,
                self.action_topic_name,
                10,
            )

            if CAMERA_AVAILABLE:
                self.node.create_subscription(
                    Image,
                    self.camera_topic_name,
                    self.handle_camera_message,
                    10,
                )
                self.camera_status_changed.emit(f"Camera source: waiting for {self.camera_topic_name}")
            else:
                self.camera_status_changed.emit("Camera source: placeholder image, camera dependencies unavailable")

            self.status_changed.emit(f"ROS integration: subscribed to {self.tts_topic_name}, publishing to {self.action_topic_name}")

            while self._running and rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=0.1)
                self.publish_pending_actions()

        except Exception as error:
            self.status_changed.emit(f"ROS integration error: {error}")

        finally:
            if self.node is not None:
                self.node.destroy_node()

            if ROS_AVAILABLE and rclpy.ok():
                rclpy.shutdown()

    def handle_tts_message(self, message):
        """Forward incoming /tts_output text to the PyQt runtime view."""

        text = message.data.strip()
        if text:
            self.speech_received.emit(text)

    def handle_camera_message(self, message):
        """Convert a ROS image message into a QImage for the camera panel."""

        if self.bridge is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding="rgb8",
            )

            height, width, channels = cv_image.shape
            bytes_per_line = channels * width

            qt_image = QImage(
                cv_image.data,
                width,
                height,
                bytes_per_line,
                QImage.Format.Format_RGB888,
            ).copy()

            self.camera_frame_received.emit(qt_image)
            self.camera_status_changed.emit(f"Camera source: live ROS topic {self.camera_topic_name}")

        except Exception as error:
            self.camera_status_changed.emit(f"Camera error: {error}")

    def queue_ui_action(self, action_text):
        """Queue a UI button action so it can be published from the ROS thread."""

        if action_text:
            self.action_queue.put(action_text)

    def publish_pending_actions(self):
        """Publish queued UI feedback messages on /ui_action."""

        if self.action_publisher is None:
            return

        while True:
            try:
                action_text = self.action_queue.get_nowait()
            except Empty:
                break

            message = String()
            message.data = action_text
            self.action_publisher.publish(message)
            self.action_published.emit(action_text)

    def stop(self):
        self._running = False


# ============================================================================
# MAIN WINDOW
# ============================================================================


class ZirbiDisplay(QMainWindow):
    # ============================================================================
    # INITIALIZATION
    # ============================================================================

    def __init__(self):
        super().__init__()

        self.setWindowTitle("Zirbi Display Prototype")
        self.resize(WINDOW_WIDTH, WINDOW_HEIGHT)
        self.setMinimumSize(WINDOW_WIDTH, WINDOW_HEIGHT)

        self.current_view = "Home"
        self.active_control_page = None
        self.last_action = "None"
        self.runtime_state = None
        self.ros_interface_thread = None
        self.ros_status_text = "ROS integration: not initialized"
        self.camera_status_text = "Camera source: placeholder image"

        self.setup_labels()
        self.setup_buttons()
        self.setup_pages()
        self.setup_clock()
        self.apply_styles()
        self.setup_ros_integration()

        self.apply_runtime_state(self.create_default_state())
        self.update_navigation()

    def setup_labels(self):
        self.page_title = QLabel("Zirbi Control")
        self.page_title.setObjectName("taskTitle")

        self.time_label = QLabel()
        self.time_label.setObjectName("smallText")

        self.message_label = QLabel()
        self.speech_label = QLabel()
        self.next_action_label = QLabel()

        self.dev_state_label = QLabel()
        self.dev_camera_label = QLabel()
        self.dev_hri_label = QLabel()
        self.dev_system_label = QLabel()

    def setup_buttons(self):
        self.home_top_button = QPushButton("← Home")
        self.home_top_button.setObjectName("homeTopButton")
        self.home_top_button.setCursor(QCursor(Qt.CursorShape.PointingHandCursor))
        self.home_top_button.clicked.connect(self.go_home)
        self.home_top_button.setVisible(False)

        self.back_top_button = QPushButton("← Control")
        self.back_top_button.setObjectName("homeTopButton")
        self.back_top_button.setCursor(QCursor(Qt.CursorShape.PointingHandCursor))
        self.back_top_button.clicked.connect(self.go_one_level_up)
        self.back_top_button.setVisible(False)

        self.jury_button = QPushButton("Jury View")
        self.developer_button = QPushButton("Developer View")

        self.primary_action_button = QPushButton(DEFAULT_ACTION_LABELS[0])
        self.secondary_action_button = QPushButton(DEFAULT_ACTION_LABELS[1])
        self.cancel_action_button = QPushButton(DEFAULT_ACTION_LABELS[2])

    def setup_pages(self):
        self.views = QStackedWidget()
        self.pages = QStackedWidget()

        main = QWidget()
        main.setObjectName("main")
        self.setCentralWidget(main)

        root = QVBoxLayout(main)
        root.setContentsMargins(24, 16, 24, 18)
        root.setSpacing(0)

        root.addLayout(self.build_top_bar())
        root.addSpacing(12)

        self.pages.addWidget(self.build_home_view())
        self.pages.addWidget(self.build_pick_and_place_control())
        self.pages.addWidget(self.build_runtime_view())
        self.pages.addWidget(self.build_laundry_control())
        self.pages.addWidget(self.build_initial_scan_view())
        self.pages.addWidget(self.build_maintenance_view())

        root.addWidget(self.pages, stretch=1)

    def setup_clock(self):
        self.update_clock()

        self.clock_timer = QTimer(self)
        self.clock_timer.timeout.connect(self.update_clock)
        self.clock_timer.start(1000)

    def setup_ros_integration(self):
        """Start the ROS 2 bridge thread if rclpy is available.

        The display can still be opened as a pure PyQt prototype without ROS 2.
        Inside the development container, this method starts the ROS bridge that
        subscribes to speech and camera topics and publishes UI feedback.
        """

        if ROS_AVAILABLE:
            self.ros_status_text = "ROS integration: available"
            self.ros_interface_thread = RosInterfaceThread("/tts_output", "/ui_action")
            self.ros_interface_thread.speech_received.connect(self.handle_ros_speech)
            self.ros_interface_thread.status_changed.connect(self.update_ros_status)
            self.ros_interface_thread.action_published.connect(self.handle_ros_action_published)
            self.ros_interface_thread.camera_frame_received.connect(self.handle_ros_camera_frame)
            self.ros_interface_thread.camera_status_changed.connect(self.update_camera_status)
            self.ros_interface_thread.start()
        else:
            self.ros_status_text = "ROS integration: unavailable in current Python environment"

    def update_clock(self):
        self.time_label.setText(QTime.currentTime().toString("HH:mm"))

    def closeEvent(self, event):
        if self.ros_interface_thread is not None:
            self.ros_interface_thread.stop()
            self.ros_interface_thread.wait(1000)

        event.accept()

    # ============================================================================
    # HOME SCREEN
    # ============================================================================

    def build_home_view(self):
        page = QWidget()

        root = QVBoxLayout(page)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(18)

        intro = QFrame()
        intro.setObjectName("homeIntroPanel")

        intro_layout = QVBoxLayout(intro)
        intro_layout.setContentsMargins(28, 24, 28, 24)
        intro_layout.setSpacing(8)

        title = QLabel("Home")
        title.setObjectName("homeTitle")

        subtitle = QLabel("Select a challenge or system mode.")
        subtitle.setObjectName("homeSubtitle")

        intro_layout.addWidget(title)
        intro_layout.addWidget(subtitle)

        root.addWidget(intro)

        grid = QHBoxLayout()
        grid.setSpacing(18)

        left_column = QVBoxLayout()
        right_column = QVBoxLayout()
        left_column.setSpacing(18)
        right_column.setSpacing(18)

        left_column.addWidget(
            self.home_card(
                "Pick and Place",
                "Table clearing, sorting, dishwasher and breakfast setup",
                self.open_pick_and_place_control,
            )
        )

        left_column.addWidget(
            self.home_card(
                "Laundry",
                "T-shirt detection, washer interaction and folding support",
                self.open_laundry_control,
            )
        )

        right_column.addWidget(
            self.home_card(
                "Initial Scan",
                "Mapping, semantic annotation and room setup",
                self.open_initial_scan,
            )
        )

        right_column.addWidget(
            self.home_card(
                "Maintenance",
                "Diagnostics, reset, topic status, audio and camera test",
                self.open_maintenance,
            )
        )

        grid.addLayout(left_column, stretch=1)
        grid.addLayout(right_column, stretch=1)

        root.addLayout(grid, stretch=1)

        return page

    def home_card(self, title, subtitle, callback):
        button = QPushButton()
        button.setObjectName("homeCardButton")
        button.setCursor(QCursor(Qt.CursorShape.PointingHandCursor))
        button.clicked.connect(callback)

        layout = QVBoxLayout(button)
        layout.setContentsMargins(24, 22, 24, 22)
        layout.setSpacing(10)

        title_label = QLabel(title)
        title_label.setObjectName("homeCardTitle")
        title_label.setWordWrap(True)

        subtitle_label = QLabel(subtitle)
        subtitle_label.setObjectName("homeCardText")
        subtitle_label.setWordWrap(True)

        action_label = QLabel("Open")
        action_label.setObjectName("homeCardAction")

        layout.addWidget(title_label)
        layout.addWidget(subtitle_label)
        layout.addStretch()
        layout.addWidget(action_label, alignment=Qt.AlignmentFlag.AlignRight)

        return button

    # ============================================================================
    # CONTROL SCREENS
    # ============================================================================

    def build_pick_and_place_control(self):
        start_items = [
            (
                "Start Full Challenge",
                "Run the complete Pick and Place workflow.",
                lambda: self.open_pick_runtime(
                    "Full Pick and Place",
                    "status",
                    "Starting full Pick and Place task.",
                    "I am starting the full Pick and Place task.",
                    "Detect work areas",
                ),
            ),
        ]

        subtask_items = [
            (
                "Clean Dining Table",
                "Clear objects from the dining table.",
                lambda: self.open_pick_runtime(
                    "Clean Dining Table",
                    "status",
                    "Dining table cleanup has been started.",
                    "I am starting to clear the dining table.",
                    "Detect objects on the table",
                ),
            ),
            (
                "Set Breakfast",
                "Prepare bowl, spoon, cereal and milk.",
                lambda: self.open_pick_runtime(
                    "Set Breakfast",
                    "status",
                    "Breakfast setup has been started.",
                    "I am starting to set up breakfast.",
                    "Check breakfast surface",
                ),
            ),
            (
                "Clean Extra Surface",
                "Clear additional surface and store objects.",
                lambda: self.open_pick_runtime(
                    "Clean Extra Surface",
                    "status",
                    "Extra surface cleanup has been started.",
                    "I am clearing the extra surface.",
                    "Detect objects on the extra surface",
                ),
            ),
            (
                "Sort to Cabinet",
                "Sort remaining objects into the cabinet.",
                lambda: self.open_pick_runtime(
                    "Sort to Cabinet",
                    "status",
                    "Cabinet sorting has been started.",
                    "I am sorting detected objects into the cabinet.",
                    "Determine target shelf",
                ),
            ),
        ]

        return self.build_challenge_control(
            title="Pick and Place Control",
            subtitle="Select a full challenge run or a specific Pick and Place subtask.",
            start_items=start_items,
            subtask_items=subtask_items,
        )

    def build_laundry_control(self):
        start_items = [
            (
                "Start Full Challenge",
                "Run the complete Laundry workflow.",
                lambda: self.open_laundry_runtime(
                    "Full Laundry",
                    "status",
                    "Starting full Laundry task.",
                    "I am starting the full Laundry task.",
                    "Locate laundry area",
                ),
            ),
        ]

        subtask_items = [
            (
                "Retrieve Laundry",
                "Pick up laundry from basket or washer.",
                lambda: self.open_laundry_runtime(
                    "Retrieve Laundry",
                    "status",
                    "Laundry retrieval has been started.",
                    "I am searching for and picking up a T-shirt.",
                    "Detect T-shirt",
                ),
            ),
            (
                "Open Washer",
                "Detect and open the washing machine.",
                lambda: self.open_laundry_runtime(
                    "Open Washer",
                    "status",
                    "Washer opening has been started.",
                    "I am checking the washing machine and opening the door.",
                    "Detect washer door",
                ),
            ),
            (
                "Fold T-Shirt",
                "Fold or request help to fold a T-shirt.",
                lambda: self.open_laundry_runtime(
                    "Fold T-Shirt",
                    "assistance",
                    "Folding assistance may be required.",
                    "I am preparing to fold the T-shirt.",
                    "Check whether folding assistance is required",
                ),
            ),
            (
                "Stack Clothes",
                "Stack folded clothes on the table.",
                lambda: self.open_laundry_runtime(
                    "Stack Clothes",
                    "status",
                    "Clothes stacking has been started.",
                    "I am stacking the folded clothes.",
                    "Detect placement position",
                ),
            ),
        ]

        return self.build_challenge_control(
            title="Laundry Control",
            subtitle="Select a full challenge run or a specific Laundry subtask.",
            start_items=start_items,
            subtask_items=subtask_items,
        )

    def build_challenge_control(self, title, subtitle, start_items, subtask_items):
        page = QWidget()

        root = QVBoxLayout(page)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        panel = QFrame()
        panel.setObjectName("mainPanel")

        panel_layout = QVBoxLayout(panel)
        panel_layout.setContentsMargins(28, 24, 28, 24)
        panel_layout.setSpacing(16)

        title_label = QLabel(title)
        title_label.setObjectName("homeTitle")

        subtitle_label = QLabel(subtitle)
        subtitle_label.setObjectName("homeSubtitle")
        subtitle_label.setWordWrap(True)

        panel_layout.addWidget(title_label)
        panel_layout.addWidget(subtitle_label)
        panel_layout.addWidget(self.control_section("Start", start_items))
        panel_layout.addWidget(self.control_section("Main Subtasks", subtask_items))
        panel_layout.addStretch()

        root.addWidget(panel, stretch=1)

        return page

    def control_section(self, title, items):
        frame = QFrame()
        frame.setObjectName("controlSection")

        root = QVBoxLayout(frame)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(10)

        title_label = QLabel(title)
        title_label.setObjectName("sectionTitle")

        root.addWidget(title_label)

        row = QHBoxLayout()
        row.setSpacing(14)

        for item_title, item_text, callback in items:
            row.addWidget(
                self.control_button(item_title, item_text, callback),
                stretch=1,
            )

        root.addLayout(row)

        return frame

    def control_button(self, title, text, callback):
        button = QPushButton()
        button.setObjectName("controlButton")
        button.setCursor(QCursor(Qt.CursorShape.PointingHandCursor))
        button.clicked.connect(callback)

        layout = QVBoxLayout(button)
        layout.setContentsMargins(20, 16, 20, 16)
        layout.setSpacing(8)

        title_label = QLabel(title)
        title_label.setObjectName("controlButtonTitle")
        title_label.setWordWrap(True)

        text_label = QLabel(text)
        text_label.setObjectName("controlButtonText")
        text_label.setWordWrap(True)

        layout.addWidget(title_label)
        layout.addWidget(text_label)
        layout.addStretch()

        return button

    # ============================================================================
    # RUNTIME SCREEN
    # ============================================================================

    def build_runtime_view(self):
        page = QWidget()

        root = QVBoxLayout(page)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        root.addLayout(self.build_layered_tabs())
        root.addWidget(self.build_main_panel(), stretch=1)

        return page

    def build_top_bar(self):
        layout = QHBoxLayout()

        layout.addWidget(self.page_title)
        layout.addStretch()
        layout.addWidget(self.back_top_button)
        layout.addSpacing(10)
        layout.addWidget(self.home_top_button)
        layout.addSpacing(16)
        layout.addWidget(self.time_label)

        return layout

    def build_layered_tabs(self):
        layout = QHBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(12)

        self.jury_button.setObjectName("activeTabButton")
        self.developer_button.setObjectName("tabButton")

        for button in (self.jury_button, self.developer_button):
            button.setCursor(QCursor(Qt.CursorShape.PointingHandCursor))
            button.setMinimumHeight(48)

        self.jury_button.clicked.connect(self.show_jury_view)
        self.developer_button.clicked.connect(self.show_developer_view)

        layout.addWidget(self.jury_button)
        layout.addWidget(self.developer_button)
        layout.addStretch()

        return layout

    def build_main_panel(self):
        panel = QFrame()
        panel.setObjectName("mainPanel")

        layout = QVBoxLayout(panel)
        layout.setContentsMargins(18, 18, 18, 18)
        layout.setSpacing(14)

        self.views.addWidget(self.build_jury_view())
        self.views.addWidget(self.build_developer_view())

        layout.addWidget(self.views, stretch=1)
        layout.addLayout(self.build_runtime_buttons())

        return panel

    def build_jury_view(self):
        page = QWidget()

        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(14)

        layout.addWidget(self.build_progress())
        layout.addLayout(self.build_jury_content(), stretch=1)

        return page

    def build_progress(self):
        frame = QFrame()
        frame.setObjectName("progressCard")

        self.progress_layout = QHBoxLayout(frame)
        self.progress_layout.setContentsMargins(0, 0, 0, 0)
        self.progress_layout.setSpacing(12)

        return frame

    def update_progress(self, steps):
        if not hasattr(self, "progress_layout"):
            return

        while self.progress_layout.count():
            item = self.progress_layout.takeAt(0)
            widget = item.widget()

            if widget is not None:
                widget.deleteLater()

        for icon, text, state in steps:
            self.progress_layout.addWidget(self.step_box(icon, text, state))

    def step_box(self, icon, text, state):
        frame = QFrame()
        frame.setObjectName("step_" + state)

        layout = QHBoxLayout(frame)
        layout.setContentsMargins(12, 7, 12, 7)
        layout.setSpacing(8)

        icon_label = QLabel(icon)
        icon_label.setObjectName("stepIcon")
        icon_label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        text_label = QLabel(text)
        text_label.setObjectName("stepText")
        text_label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        layout.addWidget(icon_label)
        layout.addWidget(text_label)

        return frame

    def build_jury_content(self):
        layout = QHBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(16)

        layout.addWidget(self.build_camera_area(), stretch=1)
        layout.addWidget(self.build_info_area(), stretch=1)

        return layout

    def build_camera_area(self):
        frame = QFrame()
        frame.setObjectName("card")

        layout = QVBoxLayout(frame)
        layout.setContentsMargins(0, 0, 0, 0)

        image_path = Path(__file__).parent / "camera_placeholder.png"
        self.camera_view = CameraView(image_path)
        self.camera_view.setObjectName("cameraView")

        layout.addWidget(self.camera_view, stretch=1)

        return frame

    def build_info_area(self):
        frame = QFrame()
        frame.setObjectName("card")
        frame.setFixedWidth(585)

        layout = QVBoxLayout(frame)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(14)

        self.message_label.setObjectName("infoBox")
        self.speech_label.setObjectName("infoBox")
        self.next_action_label.setObjectName("primaryInfoBox")

        for label in (
            self.message_label,
            self.speech_label,
            self.next_action_label,
        ):
            label.setWordWrap(True)
            label.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignTop)

        layout.addWidget(self.message_label)
        layout.addWidget(self.speech_label)
        layout.addWidget(self.next_action_label)
        layout.addStretch()

        return frame

    def build_runtime_buttons(self):
        layout = QHBoxLayout()
        layout.setSpacing(18)

        self.primary_action_button.setObjectName("confirmButton")
        self.secondary_action_button.setObjectName("repeatButton")
        self.cancel_action_button.setObjectName("cancelButton")

        for button in (
            self.primary_action_button,
            self.secondary_action_button,
            self.cancel_action_button,
        ):
            button.setCursor(QCursor(Qt.CursorShape.PointingHandCursor))

        self.primary_action_button.clicked.connect(self.primary_action)
        self.secondary_action_button.clicked.connect(self.secondary_action)
        self.cancel_action_button.clicked.connect(self.cancel_task)

        layout.addWidget(self.primary_action_button)
        layout.addWidget(self.secondary_action_button)
        layout.addWidget(self.cancel_action_button)

        return layout

    # ============================================================================
    # DEVELOPER VIEW
    # ============================================================================

    def build_developer_view(self):
        page = QWidget()

        root = QHBoxLayout(page)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(16)

        left_column = QVBoxLayout()
        right_column = QVBoxLayout()
        left_column.setSpacing(14)
        right_column.setSpacing(14)

        self.dev_state_label.setObjectName("debugText")
        self.dev_camera_label.setObjectName("debugText")
        self.dev_hri_label.setObjectName("debugText")
        self.dev_system_label.setObjectName("debugText")

        left_column.addWidget(self.debug_card_with_label("Runtime State", self.dev_state_label))

        left_column.addWidget(self.debug_card_with_label("Camera Source", self.dev_camera_label))

        right_column.addWidget(self.debug_card_with_label("HRI Output Model", self.dev_hri_label))

        right_column.addWidget(self.debug_card_with_label("System State", self.dev_system_label))

        root.addLayout(left_column, stretch=1)
        root.addLayout(right_column, stretch=1)

        return page

    def debug_card_with_label(self, title, label):
        frame = QFrame()
        frame.setObjectName("debugCard")

        layout = QVBoxLayout(frame)
        layout.setContentsMargins(22, 20, 22, 20)
        layout.setSpacing(10)

        title_label = QLabel(title)
        title_label.setObjectName("debugTitle")

        label.setWordWrap(True)

        layout.addWidget(title_label)
        layout.addWidget(label)
        layout.addStretch()

        return frame

    def debug_card(self, title, text):
        label = QLabel(text)
        label.setObjectName("debugText")
        label.setWordWrap(True)

        return self.debug_card_with_label(title, label)

    # ============================================================================
    # PLACEHOLDER SCREENS
    # ============================================================================

    def build_initial_scan_view(self):
        return self.build_placeholder_page(
            "Initial Scan",
            "Mapping and semantic setup",
            "Suggested functions:\n• Start mapping\n• Detect room landmarks\n• Save semantic labels\n• Confirm map ready",
            "Start Initial Scan",
        )

    def build_maintenance_view(self):
        return self.build_placeholder_page(
            "Maintenance",
            "Diagnostics and recovery",
            "Suggested functions:\n• Reset robot\n• Restart HRI\n• Camera test\n• Audio test\n• Topic / node status",
            "Open Diagnostics",
        )

    def build_placeholder_page(self, title, subtitle, body_text, button_text):
        page = QWidget()

        root = QVBoxLayout(page)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        panel = QFrame()
        panel.setObjectName("mainPanel")

        panel_layout = QVBoxLayout(panel)
        panel_layout.setContentsMargins(28, 24, 28, 24)
        panel_layout.setSpacing(18)

        title_label = QLabel(title)
        title_label.setObjectName("homeTitle")

        subtitle_label = QLabel(subtitle)
        subtitle_label.setObjectName("homeSubtitle")
        subtitle_label.setWordWrap(True)

        body_card = QFrame()
        body_card.setObjectName("debugCard")

        body_layout = QVBoxLayout(body_card)
        body_layout.setContentsMargins(22, 20, 22, 20)

        body_label = QLabel(body_text)
        body_label.setObjectName("debugText")
        body_label.setWordWrap(True)

        body_layout.addWidget(body_label)

        action_button = QPushButton(button_text)
        action_button.setObjectName("repeatButton")
        action_button.setCursor(QCursor(Qt.CursorShape.PointingHandCursor))
        action_button.setMinimumHeight(72)

        panel_layout.addWidget(title_label)
        panel_layout.addWidget(subtitle_label)
        panel_layout.addWidget(body_card)
        panel_layout.addWidget(action_button)
        panel_layout.addStretch()

        root.addWidget(panel, stretch=1)

        return page

    # ============================================================================
    # NAVIGATION
    # ============================================================================

    def open_pick_and_place_control(self):
        self.current_view = "Pick and Place Control"
        self.active_control_page = "pick"

        self.page_title.setText("Pick and Place")
        self.home_top_button.setVisible(True)
        self.back_top_button.setVisible(False)

        self.pages.setCurrentIndex(PAGE_PICK_CONTROL)

    def open_laundry_control(self):
        self.current_view = "Laundry Control"
        self.active_control_page = "laundry"

        self.page_title.setText("Laundry")
        self.home_top_button.setVisible(True)
        self.back_top_button.setVisible(False)

        self.pages.setCurrentIndex(PAGE_LAUNDRY_CONTROL)

    def open_pick_runtime(self, subtask, message_type, display_text, speech_text, next_action):
        state = self.create_pick_state(
            subtask=subtask,
            message_type=message_type,
            display_text=display_text,
            speech_text=speech_text,
            next_action=next_action,
        )

        self.open_runtime_page(
            title="Pick and Place",
            active_control_page="pick",
            state=state,
        )

    def open_laundry_runtime(self, subtask, message_type, display_text, speech_text, next_action):
        state = self.create_laundry_state(
            subtask=subtask,
            message_type=message_type,
            display_text=display_text,
            speech_text=speech_text,
            next_action=next_action,
        )

        self.open_runtime_page(
            title="Laundry",
            active_control_page="laundry",
            state=state,
        )

    def open_runtime_page(self, title, active_control_page, state):
        self.active_control_page = active_control_page

        self.page_title.setText(title)
        self.home_top_button.setVisible(True)
        self.back_top_button.setVisible(True)

        self.pages.setCurrentIndex(PAGE_RUNTIME)
        self.show_jury_view()
        self.apply_runtime_state(state)

    def open_initial_scan(self):
        self.current_view = "Initial Scan"
        self.active_control_page = None

        self.page_title.setText("Initial Scan")
        self.home_top_button.setVisible(True)
        self.back_top_button.setVisible(False)

        self.pages.setCurrentIndex(PAGE_INITIAL_SCAN)

    def open_maintenance(self):
        self.current_view = "Maintenance"
        self.active_control_page = None

        self.page_title.setText("Maintenance")
        self.home_top_button.setVisible(True)
        self.back_top_button.setVisible(False)

        self.pages.setCurrentIndex(PAGE_MAINTENANCE)

    def go_one_level_up(self):
        if self.pages.currentIndex() != PAGE_RUNTIME:
            return

        if self.active_control_page == "pick":
            self.open_pick_and_place_control()
        elif self.active_control_page == "laundry":
            self.open_laundry_control()

    def go_home(self):
        self.current_view = "Home"
        self.active_control_page = None

        self.page_title.setText("Zirbi Control")
        self.home_top_button.setVisible(False)
        self.back_top_button.setVisible(False)

        self.pages.setCurrentIndex(PAGE_HOME)

    def show_jury_view(self):
        self.views.setCurrentIndex(0)
        self.update_navigation()

    def show_developer_view(self):
        self.views.setCurrentIndex(1)
        self.update_navigation()

    def update_navigation(self):
        if self.views.currentIndex() == 0:
            self.jury_button.setObjectName("activeTabButton")
            self.developer_button.setObjectName("tabButton")
        else:
            self.jury_button.setObjectName("tabButton")
            self.developer_button.setObjectName("activeTabButton")

        for button in (self.jury_button, self.developer_button):
            button.style().unpolish(button)
            button.style().polish(button)
            button.update()

    # ============================================================================
    # RUNTIME STATE
    # ============================================================================

    def create_default_state(self):
        return RuntimeState(
            challenge="Pick and Place",
            subtask="Prototype Default",
            message_type="confirmation",
            display_text="Object could not be identified with sufficient confidence.",
            speech_text="Please confirm the detected object.",
            next_action="Confirm object",
            action_labels=DEFAULT_ACTION_LABELS,
            progress=self.get_pick_progress_steps(),
        )

    def create_pick_state(self, subtask, message_type, display_text, speech_text, next_action):
        return RuntimeState(
            challenge="Pick and Place",
            subtask=subtask,
            message_type=message_type,
            display_text=display_text,
            speech_text=speech_text,
            next_action=next_action,
            action_labels=DEFAULT_ACTION_LABELS,
            progress=self.get_pick_progress_steps(),
        )

    def create_laundry_state(self, subtask, message_type, display_text, speech_text, next_action):
        if message_type == "assistance":
            action_labels = ASSISTANCE_ACTION_LABELS
        else:
            action_labels = DEFAULT_ACTION_LABELS

        return RuntimeState(
            challenge="Laundry",
            subtask=subtask,
            message_type=message_type,
            display_text=display_text,
            speech_text=speech_text,
            next_action=next_action,
            action_labels=action_labels,
            progress=self.get_laundry_progress_steps(),
        )

    def apply_runtime_state(self, state):
        self.runtime_state = state
        self.current_view = state.subtask

        self.message_label.setText(f"Status ({state.message_type}):\n{state.display_text}")

        self.speech_label.setText(f"Speech Output:\n{state.speech_text}")

        self.next_action_label.setText(f"Next Action:\n{state.next_action}")

        self.primary_action_button.setText(state.action_labels[0])
        self.secondary_action_button.setText(state.action_labels[1])
        self.cancel_action_button.setText(state.action_labels[2])

        self.update_progress(state.progress)
        self.update_developer_state()

    def update_developer_state(self):
        if self.runtime_state is None:
            return

        state = self.runtime_state

        self.dev_state_label.setText(
            f"Challenge: {state.challenge}\nSubtask: {state.subtask}\nMessage type: {state.message_type}\nNext action: {state.next_action}"
        )

        self.dev_camera_label.setText(
            f"{self.camera_status_text}\n"
            "Fallback: camera_placeholder.png\n"
            "Expected message type: sensor_msgs/msg/Image\n"
            "Rendering: cv_bridge -> QImage -> QPixmap"
        )

        self.dev_hri_label.setText(
            "Prototype-side representation of a future HRI output message.\n\n"
            f"display_text: {state.display_text}\n"
            f"speech_text: {state.speech_text}\n"
            f"action_buttons: {state.action_labels[0]} | "
            f"{state.action_labels[1]} | {state.action_labels[2]}"
        )

        self.dev_system_label.setText(
            "ROS node: zirbi_display_ros_interface\n"
            "Decision Making format: not finalized\n"
            "Planned integration: HRI Output Interface\n"
            "Output channels: Display + TTS\n"
            "Subscribed topic: /tts_output\n"
            "Published topic: /ui_action\n"
            f"Last UI action: {self.last_action}\n"
            f"{self.ros_status_text}"
        )

    def update_ros_status(self, status_text):
        self.ros_status_text = status_text
        self.update_developer_state()

    def handle_ros_speech(self, speech_text):
        """Update the visible Speech Output card from /tts_output."""

        if self.runtime_state is None:
            self.speech_label.setText(f"Speech Output:\n{speech_text}")
            return

        updated_state = RuntimeState(
            challenge=self.runtime_state.challenge,
            subtask=self.runtime_state.subtask,
            message_type=self.runtime_state.message_type,
            display_text=self.runtime_state.display_text,
            speech_text=speech_text,
            next_action=self.runtime_state.next_action,
            action_labels=self.runtime_state.action_labels,
            progress=self.runtime_state.progress,
        )

        self.apply_runtime_state(updated_state)

    def update_camera_status(self, status_text):
        self.camera_status_text = status_text
        self.update_developer_state()

    def handle_ros_camera_frame(self, image):
        """Render the latest ROS camera frame in the runtime camera panel."""

        if hasattr(self, "camera_view"):
            self.camera_view.set_frame(image)

    def publish_ui_action(self, action_text):
        """Send prototype UI feedback to ROS.

        The current actions are simple strings such as confirm, repeat and
        cancel. They are published on /ui_action and should later be replaced
        or extended by a structured HRI feedback message.
        """

        if self.ros_interface_thread is not None and ROS_AVAILABLE:
            self.ros_interface_thread.queue_ui_action(action_text)

    def handle_ros_action_published(self, action_text):
        """Mirror the last published UI action in the Developer View."""

        self.last_action = action_text
        self.update_developer_state()

    def get_pick_progress_steps(self):
        return [
            ("✓", "1. Start", "done"),
            ("", "2. Detect Object", "active"),
            ("", "3. Grasp", "open"),
            ("", "4. Place", "open"),
        ]

    def get_laundry_progress_steps(self):
        return [
            ("✓", "1. Start", "done"),
            ("", "2. Locate Area", "active"),
            ("", "3. Pick T-Shirt", "open"),
            ("", "4. Place on Table", "open"),
            ("", "5. Fold", "open"),
        ]

    # ============================================================================
    # ACTIONS
    # ============================================================================

    def primary_action(self):
        if self.runtime_state is None:
            return

        if self.runtime_state.message_type == "assistance":
            display_text = "Human assistance has been confirmed."
            speech_text = "Thank you. I will continue the task."
            next_action = "Continue task"
        else:
            display_text = "Confirmation has been received."
            speech_text = "The input has been confirmed."
            next_action = "Continue task"

        updated_state = RuntimeState(
            challenge=self.runtime_state.challenge,
            subtask=self.runtime_state.subtask,
            message_type="status",
            display_text=display_text,
            speech_text=speech_text,
            next_action=next_action,
            action_labels=DEFAULT_ACTION_LABELS,
            progress=self.runtime_state.progress,
        )

        self.last_action = "confirm"
        self.publish_ui_action(self.last_action)
        self.apply_runtime_state(updated_state)

    def secondary_action(self):
        if self.runtime_state is None:
            return

        if self.runtime_state.message_type == "assistance":
            display_text = "The help instruction will be repeated."
            speech_text = "I will repeat the help instruction."
            next_action = "Wait for assistance"
            action_labels = ASSISTANCE_ACTION_LABELS
        else:
            display_text = "The current instruction will be repeated."
            speech_text = "I will repeat the current instruction."
            next_action = "Wait for user feedback"
            action_labels = DEFAULT_ACTION_LABELS

        updated_state = RuntimeState(
            challenge=self.runtime_state.challenge,
            subtask=self.runtime_state.subtask,
            message_type=self.runtime_state.message_type,
            display_text=display_text,
            speech_text=speech_text,
            next_action=next_action,
            action_labels=action_labels,
            progress=self.runtime_state.progress,
        )

        self.last_action = "repeat"
        self.publish_ui_action(self.last_action)
        self.apply_runtime_state(updated_state)

    def cancel_task(self):
        if self.runtime_state is None:
            return

        updated_state = RuntimeState(
            challenge=self.runtime_state.challenge,
            subtask=self.runtime_state.subtask,
            message_type="warning",
            display_text="The task has been cancelled.",
            speech_text="The task has been cancelled.",
            next_action="Select a new task",
            action_labels=DEFAULT_ACTION_LABELS,
            progress=self.runtime_state.progress,
        )

        self.last_action = "cancel"
        self.publish_ui_action(self.last_action)
        self.apply_runtime_state(updated_state)

    # ============================================================================
    # STYLING
    # ============================================================================

    def apply_styles(self):
        self.setStyleSheet("""
            QWidget#main {
                background-color: #08090c;
                color: #f4f4f5;
                font-family: Inter, Segoe UI Variable, Segoe UI, Arial;
            }

            QLabel#taskTitle {
                font-size: 30px;
                font-weight: 700;
                color: #f4f4f5;
            }

            QLabel#smallText {
                font-size: 20px;
                font-weight: 400;
                color: #b7bbc2;
            }

            QLabel#icon {
                font-size: 25px;
                color: #b7bbc2;
            }

            QLabel#homeTitle {
                font-size: 28px;
                font-weight: 700;
                color: #ffffff;
            }

            QLabel#homeSubtitle {
                font-size: 18px;
                font-weight: 450;
                color: #c9ccd3;
            }

            QLabel#sectionTitle {
                font-size: 18px;
                font-weight: 700;
                color: #ffffff;
                padding-top: 2px;
                padding-bottom: 2px;
            }

            QFrame#homeIntroPanel {
                background-color: rgba(255, 255, 255, 0.06);
                border: none;
                border-radius: 22px;
            }

            QPushButton#homeCardButton {
                background-color: rgba(255, 255, 255, 0.06);
                border: none;
                border-radius: 22px;
                text-align: left;
                min-height: 180px;
            }

            QPushButton#homeCardButton:hover {
                background-color: rgba(255, 255, 255, 0.10);
            }

            QLabel#homeCardTitle {
                font-size: 24px;
                font-weight: 700;
                color: #ffffff;
            }

            QLabel#homeCardText {
                font-size: 16px;
                font-weight: 450;
                color: #c9ccd3;
            }

            QLabel#homeCardAction {
                font-size: 16px;
                font-weight: 650;
                color: #ffffff;
            }

            QPushButton#homeTopButton {
                background-color: rgba(255, 255, 255, 0.06);
                border: 1px solid rgba(255, 255, 255, 0.10);
                border-radius: 16px;
                color: #f4f4f5;
                font-size: 16px;
                font-weight: 600;
                padding: 10px 16px;
            }

            QPushButton#homeTopButton:hover {
                background-color: rgba(255, 255, 255, 0.10);
            }

            QFrame#controlSection {
                background-color: transparent;
                border: none;
            }

            QPushButton#controlButton {
                background-color: rgba(255, 255, 255, 0.06);
                border: none;
                border-radius: 18px;
                text-align: left;
                min-height: 82px;
            }

            QPushButton#controlButton:hover {
                background-color: rgba(255, 255, 255, 0.10);
            }

            QLabel#controlButtonTitle {
                font-size: 18px;
                font-weight: 700;
                color: #ffffff;
            }

            QLabel#controlButtonText {
                font-size: 14px;
                font-weight: 450;
                color: #c9ccd3;
            }

            QPushButton#tabButton {
                background-color: rgba(255, 255, 255, 0.045);
                border: none;
                border-top-left-radius: 18px;
                border-top-right-radius: 18px;
                border-bottom-left-radius: 0px;
                border-bottom-right-radius: 0px;
                color: #b7bbc2;
                font-size: 18px;
                font-weight: 550;
                padding: 12px 26px;
            }

            QPushButton#activeTabButton {
                background-color: rgba(255, 255, 255, 0.06);
                border: none;
                border-top-left-radius: 18px;
                border-top-right-radius: 18px;
                border-bottom-left-radius: 0px;
                border-bottom-right-radius: 0px;
                color: #ffffff;
                font-size: 18px;
                font-weight: 650;
                padding: 12px 26px;
            }

            QPushButton#tabButton:hover {
                background-color: rgba(255, 255, 255, 0.065);
                color: #ffffff;
            }

            QPushButton#activeTabButton:hover {
                background-color: rgba(255, 255, 255, 0.06);
            }

            QFrame#mainPanel {
                background-color: rgba(255, 255, 255, 0.06);
                border: none;
                border-radius: 22px;
                border-top-left-radius: 0px;
            }

            QFrame#progressCard {
                background-color: transparent;
                border: none;
                border-radius: 0px;
            }

            QFrame#card {
                background-color: transparent;
                border: none;
                border-radius: 0px;
            }

            QWidget#cameraView {
                background-color: #0e0f13;
                border: none;
                border-radius: 22px;
            }

            QFrame#step_done {
                background-color: rgba(70, 180, 105, 0.22);
                border: 1px solid rgba(120, 224, 143, 0.35);
                border-radius: 14px;
            }

            QFrame#step_active {
                background-color: rgba(255, 255, 255, 0.12);
                border: 1px solid rgba(255, 255, 255, 0.20);
                border-radius: 14px;
            }

            QFrame#step_open {
                background-color: rgba(255, 255, 255, 0.06);
                border: 1px solid rgba(255, 255, 255, 0.11);
                border-radius: 14px;
            }

            QLabel#stepIcon {
                font-size: 24px;
                color: #78e08f;
                font-weight: 700;
                min-width: 28px;
            }

            QLabel#stepText {
                font-size: 18px;
                font-weight: 600;
                color: #f4f4f5;
            }

            QLabel#infoBox {
                background-color: rgba(255, 255, 255, 0.065);
                border: none;
                border-radius: 18px;
                padding: 22px;
                font-size: 20px;
                font-weight: 500;
                color: #e8e9ed;
            }

            QLabel#primaryInfoBox {
                background-color: rgba(255, 255, 255, 0.09);
                border: none;
                border-radius: 18px;
                padding: 22px;
                font-size: 20px;
                font-weight: 500;
                color: #ffffff;
            }

            QFrame#debugCard {
                background-color: rgba(255, 255, 255, 0.065);
                border: none;
                border-radius: 22px;
            }

            QLabel#debugTitle {
                font-size: 22px;
                font-weight: 650;
                color: #ffffff;
            }

            QLabel#debugText {
                font-size: 18px;
                font-weight: 450;
                color: #d2d5dc;
                line-height: 130%;
            }

            QPushButton {
                font-size: 24px;
                font-weight: 650;
                padding: 24px;
                border-radius: 18px;
                color: #f4f4f5;
            }

            QPushButton#confirmButton {
                background-color: rgba(70, 180, 105, 0.34);
                border: 1px solid rgba(120, 224, 143, 0.55);
            }

            QPushButton#repeatButton {
                background-color: rgba(255, 255, 255, 0.09);
                border: 1px solid rgba(255, 255, 255, 0.18);
            }

            QPushButton#cancelButton {
                background-color: rgba(185, 70, 85, 0.28);
                border: 1px solid rgba(230, 108, 120, 0.50);
            }

            QPushButton#confirmButton:hover {
                background-color: rgba(70, 180, 105, 0.48);
                border: 1px solid rgba(120, 224, 143, 0.75);
            }

            QPushButton#repeatButton:hover {
                background-color: rgba(255, 255, 255, 0.15);
                border: 1px solid rgba(255, 255, 255, 0.28);
            }

            QPushButton#cancelButton:hover {
                background-color: rgba(185, 70, 85, 0.42);
                border: 1px solid rgba(230, 108, 120, 0.75);
            }

            QPushButton#confirmButton:pressed {
                background-color: rgba(70, 180, 105, 0.25);
                padding-top: 26px;
                padding-bottom: 22px;
            }

            QPushButton#repeatButton:pressed {
                background-color: rgba(255, 255, 255, 0.06);
                padding-top: 26px;
                padding-bottom: 22px;
            }

            QPushButton#cancelButton:pressed {
                background-color: rgba(185, 70, 85, 0.22);
                padding-top: 26px;
                padding-bottom: 22px;
            }
        """)


# ============================================================================
# MAIN PROGRAM
# ============================================================================


def main():
    app = QApplication(sys.argv)

    window = ZirbiDisplay()
    window.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
