"""Pitch-demo recorder for the ObjectDetection node.

Records a short demo clip from the RealSense color stream while
periodically triggering `/vision/snapshot` inference and logging GPU
utilization. Intended to produce a ~30s clip for a pitch/demo video: it
starts by requesting only the dishwasher model, then partway through also
turns on the general-objects-and-people model (both requested together
from then on). A picture-in-picture inset shows the latest annotated
detection frame, and a text overlay tracks elapsed time, active models,
last inference result, and live GPU utilization/memory.

Prerequisites (separate terminals):
    # from arlab_docker/, camera_name must be "camera_gripper" to match
    # the topic remapping in object_detection_launch.py
    docker compose -f docker-compose.librealsense.desktop.yml up
    ros2 launch arlab_computer_vision object_detection_launch.py

Usage:
    python3 pitch_video_demo.py [--duration 30] [--interval 1.0] \
        [--switch-at 15] [--out pitch_demo.mp4] [--gpu-log gpu_usage.csv]

Maintainers:
    Simeon Wagner <simeon.wagner@uni-a.de>
"""

import argparse
import csv
import statistics
import subprocess
import time
from datetime import datetime

import cv2
import numpy as np
import rclpy
from arlab_common_interfaces.action import VisionSnapshotAction
from arlab_common_interfaces.msg import VisionSnapshotCommand, VisionSnapshotResponse
from cv_bridge import CvBridge
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import Image

_RESULT_NAMES = {
    VisionSnapshotResponse.SUCCESS: "SUCCESS",
    VisionSnapshotResponse.ERROR_UNKNOWN: "ERROR_UNKNOWN",
    VisionSnapshotResponse.ERROR_NO_IMAGE_DATA: "ERROR_NO_IMAGE_DATA",
}


def _query_gpu() -> dict | None:
    """Query utilization/memory/temperature for GPU 0 via nvidia-smi."""
    try:
        out = subprocess.run(
            [
                "nvidia-smi",
                "--query-gpu=utilization.gpu,memory.used,memory.total,temperature.gpu",
                "--format=csv,noheader,nounits",
                "-i",
                "0",
            ],
            capture_output=True,
            text=True,
            timeout=2.0,
            check=True,
        )
        util, mem_used, mem_total, temp = (v.strip() for v in out.stdout.strip().split(","))
        return {
            "util_pct": float(util),
            "mem_used_mib": float(mem_used),
            "mem_total_mib": float(mem_total),
            "temp_c": float(temp),
        }
    except Exception:
        return None


class PitchVideoDemo(Node):
    """Records a demo clip while cycling inference models and logging GPU load."""

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("pitch_video_demo")
        self._duration = args.duration
        self._interval = args.interval
        self._switch_at = args.switch_at
        self._output_fps = args.fps
        self._clear_database = args.clear_database

        self._bridge = CvBridge()
        self._video_writer: cv2.VideoWriter | None = None
        self._video_path = args.out
        self._codec = args.codec

        self._last_color_frame: np.ndarray | None = None
        self._last_annotated_frame: np.ndarray | None = None
        self._last_annotated_time: float | None = None

        self.finished = False
        self._frames_written = 0

        self._last_inference_result = "pending..."
        self._last_gpu: dict | None = None

        self._gpu_log_path = args.gpu_log
        self._gpu_log_file = open(self._gpu_log_path, "w", newline="")
        self._gpu_writer = csv.writer(self._gpu_log_file)
        self._gpu_writer.writerow(["elapsed_s", "timestamp", "gpu_util_pct", "mem_used_mib", "mem_total_mib", "temp_c"])

        self._action_client = ActionClient(self, VisionSnapshotAction, "/vision/snapshot")

        self.get_logger().info("Waiting for /vision/snapshot action server...")
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Action server not available. Is object_detection running?")
            raise RuntimeError("object_detection action server not available")

        self.create_subscription(Image, args.color_topic, self._on_color_image, qos_profile=1)
        self.create_subscription(Image, args.segmented_topic, self._on_segmented_image, qos_profile=1)

        # Timer cadence resets once we start spinning, not at construction time.
        self._start_time = time.monotonic()
        self.create_timer(1.0 / self._output_fps, self._on_video_tick)
        self.create_timer(self._interval, self._on_inference_tick)
        self.create_timer(1.0, self._on_gpu_tick)

    # -- subscriptions -----------------------------------------------------
    def _on_color_image(self, msg: Image) -> None:
        self._last_color_frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def _on_segmented_image(self, msg: Image) -> None:
        self._last_annotated_frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self._last_annotated_time = time.monotonic()

    # -- model phase ---------------------------------------------------------
    def _current_models(self, elapsed: float) -> tuple[list[int], str]:
        if elapsed < self._switch_at:
            return [VisionSnapshotCommand.MODEL_DISHWASHER], "Dishwasher"
        return (
            [VisionSnapshotCommand.MODEL_DISHWASHER, VisionSnapshotCommand.MODEL_GENERAL_OBJECTS_AND_PEOPLE],
            "Dishwasher + People",
        )

    # -- timers ----------------------------------------------------------------
    def _on_inference_tick(self) -> None:
        elapsed = time.monotonic() - self._start_time
        if elapsed > self._duration:
            return
        extra_models, label = self._current_models(elapsed)

        command = VisionSnapshotCommand()
        command.clear_database = self._clear_database
        command.mask_hand = False
        command.extra_models = extra_models

        goal = VisionSnapshotAction.Goal()
        goal.command = command

        self.get_logger().info(f"[t={elapsed:4.1f}s] Requesting snapshot ({label})")
        send_future = self._action_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._last_inference_result = "REJECTED"
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_inference_result)

    def _on_inference_result(self, future) -> None:
        response = future.result().result.response
        self._last_inference_result = _RESULT_NAMES.get(response.result, str(response.result))
        if response.error_msg:
            self.get_logger().warn(f"Snapshot error: {response.error_msg}")

    def _on_gpu_tick(self) -> None:
        stats = _query_gpu()
        if stats is None:
            return
        elapsed = time.monotonic() - self._start_time
        self._last_gpu = stats
        self._gpu_writer.writerow(
            [
                f"{elapsed:.2f}",
                datetime.now().isoformat(timespec="seconds"),
                stats["util_pct"],
                stats["mem_used_mib"],
                stats["mem_total_mib"],
                stats["temp_c"],
            ]
        )
        self._gpu_log_file.flush()

    def _on_video_tick(self) -> None:
        elapsed = time.monotonic() - self._start_time
        if elapsed >= self._duration:
            self._finish()
            return
        if self._last_color_frame is None:
            return

        frame = self._last_color_frame.copy()
        if self._video_writer is None:
            h, w = frame.shape[:2]
            self._video_writer = self._open_writer(w, h)

        self._draw_overlay(frame, elapsed)
        self._video_writer.write(frame)
        self._frames_written += 1

    def _open_writer(self, w: int, h: int) -> cv2.VideoWriter:
        """Open a VideoWriter, falling back to codecs that this OpenCV build supports.

        cv2.VideoWriter does not raise when a codec backend is missing; it just
        returns a writer whose isOpened() is False and silently drops every
        write(). We try the requested codec first, then progressively more
        portable ones, and abort loudly if none actually open.
        """
        # (codec, file extension) pairs, most-preferred first.
        candidates = [
            (self._codec, self._video_path),
            ("mp4v", self._swap_ext(self._video_path, ".mp4")),
            ("MJPG", self._swap_ext(self._video_path, ".avi")),
            ("XVID", self._swap_ext(self._video_path, ".avi")),
        ]
        for codec, path in candidates:
            fourcc = cv2.VideoWriter_fourcc(*codec)
            writer = cv2.VideoWriter(path, fourcc, self._output_fps, (w, h))
            if writer.isOpened():
                self._video_path = path
                self.get_logger().info(f"Recording {w}x{h} @ {self._output_fps}fps, codec={codec} -> {path}")
                return writer
            writer.release()
            self.get_logger().warn(f"Codec '{codec}' unavailable in this OpenCV build, trying next fallback")
        raise RuntimeError("No usable video codec found (tried mp4v, MJPG, XVID). Install ffmpeg-enabled OpenCV.")

    @staticmethod
    def _swap_ext(path: str, ext: str) -> str:
        import os

        return os.path.splitext(path)[0] + ext

    # -- rendering -------------------------------------------------------------
    def _draw_overlay(self, frame: np.ndarray, elapsed: float) -> None:
        _, label = self._current_models(elapsed)
        lines = [
            f"t = {elapsed:4.1f}s / {self._duration:.0f}s",
            f"Active models: {label}",
            f"Last inference: {self._last_inference_result}",
        ]
        if self._last_gpu is not None:
            lines.append(
                f"GPU: {self._last_gpu['util_pct']:.0f}% | "
                f"{self._last_gpu['mem_used_mib']:.0f}/{self._last_gpu['mem_total_mib']:.0f} MiB"
            )

        y = 30
        for line in lines:
            cv2.putText(frame, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(frame, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 1, cv2.LINE_AA)
            y += 28

        # Picture-in-picture inset showing the latest annotated detection frame.
        if self._last_annotated_frame is not None:
            h, w = frame.shape[:2]
            inset_w = w // 3
            inset_h = int(inset_w * self._last_annotated_frame.shape[0] / self._last_annotated_frame.shape[1])
            inset = cv2.resize(self._last_annotated_frame, (inset_w, inset_h))
            x0, y0 = w - inset_w - 10, h - inset_h - 10
            frame[y0 : y0 + inset_h, x0 : x0 + inset_w] = inset
            cv2.rectangle(frame, (x0, y0), (x0 + inset_w, y0 + inset_h), (0, 255, 0), 2)
            age = time.monotonic() - self._last_annotated_time
            cv2.putText(
                frame,
                f"detections ({age:.1f}s ago)",
                (x0, y0 - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
                cv2.LINE_AA,
            )

    # -- shutdown ---------------------------------------------------------------
    def finish(self) -> None:
        self._finish()

    def _finish(self) -> None:
        if self.finished:
            return
        self.finished = True
        if self._video_writer is not None:
            self._video_writer.release()
        self._gpu_log_file.close()

        import os

        size_mb = os.path.getsize(self._video_path) / 1e6 if os.path.exists(self._video_path) else 0.0
        self.get_logger().info(
            f"Done. Wrote {self._frames_written} frames ({size_mb:.1f} MB) to {self._video_path}; "
            f"GPU log at {self._gpu_log_path}"
        )
        if self._frames_written == 0:
            self.get_logger().warn(
                "0 frames captured - the color topic produced no images. "
                "Check --color-topic matches your camera (e.g. /camera/color/image_raw)."
            )
        elif size_mb < 0.05:
            self.get_logger().warn(
                "Output file is suspiciously small; the codec may not have encoded correctly. "
                "Try a different --codec (MJPG with a .avi --out is the most portable)."
            )


def _print_gpu_summary(gpu_log_path: str) -> None:
    utils, mems = [], []
    with open(gpu_log_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            utils.append(float(row["gpu_util_pct"]))
            mems.append(float(row["mem_used_mib"]))
    if not utils:
        print("No GPU samples recorded (nvidia-smi unavailable?).")
        return
    print(
        f"GPU summary: util avg={statistics.mean(utils):.1f}% max={max(utils):.1f}% | "
        f"mem avg={statistics.mean(mems):.0f}MiB max={max(mems):.0f}MiB"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--duration", type=float, default=30.0, help="Total recording length in seconds")
    parser.add_argument("--interval", type=float, default=1.0, help="Seconds between inference snapshots")
    parser.add_argument("--switch-at", type=float, default=15.0, help="Seconds after which the people model is added")
    parser.add_argument("--fps", type=float, default=15.0, help="Output video frame rate")
    parser.add_argument("--out", default="pitch_demo.mp4", help="Output video file path")
    parser.add_argument(
        "--codec",
        default="mp4v",
        help="Preferred FourCC codec. Falls back to mp4v/MJPG/XVID if unavailable. Use MJPG with a .avi --out for max portability.",
    )
    parser.add_argument("--gpu-log", default="gpu_usage.csv", help="Output GPU usage CSV path")
    parser.add_argument("--color-topic", default="/camera/color/image_raw", help="Raw color image topic")
    parser.add_argument("--segmented-topic", default="/vision/segmented_image", help="Annotated detection image topic")
    parser.add_argument("--clear-database", action="store_true", help="Clear KB entities before each snapshot")
    args = parser.parse_args()

    rclpy.init()
    try:
        node = PitchVideoDemo(args)
    except RuntimeError as e:
        rclpy.shutdown()
        print(f"Error: {e}")
        return

    try:
        end_time = time.monotonic() + args.duration + 5.0
        while rclpy.ok() and not node.finished and time.monotonic() < end_time:
            rclpy.spin_once(node, timeout_sec=0.05)
        if not node.finished:
            node.finish()
    except KeyboardInterrupt:
        node.finish()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    _print_gpu_summary(args.gpu_log)


if __name__ == "__main__":
    main()
