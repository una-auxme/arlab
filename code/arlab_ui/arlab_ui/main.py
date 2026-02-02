import json
import math
import subprocess
import tkinter as tk
from tkinter import ttk
from pathlib import Path

import threading
import queue

from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from rcl_interfaces.msg import Log


CONFIG_PATH = Path("/workspace/src/arlab/code/arlab_ui/config/buttons.json")


class RosBridge(Node):
    def __init__(
        self,
        sub_topic: str,
        pub_topic: str,
        incoming_queue: "queue.Queue[str]",
        outgoing_queue: "queue.Queue[str]",
        ros_error_queue: "queue.Queue[str]",
        rosout_topic: str = "/rosout",
        min_severity: int = Log.WARN,  # Log.WARN or Log.Error
    ):
        super().__init__("tk_ros_bridge")
        self.incoming_queue = incoming_queue
        self.outgoing_queue = outgoing_queue
        self.ros_error_queue = ros_error_queue
        self.min_severity = min_severity

        self.sub = self.create_subscription(String, sub_topic, self._on_msg, 10)
        self.pub = self.create_publisher(String, pub_topic, 10)

        # Subscribe to /rosout for logs/errors
        rosout_qos = QoSProfile(
            depth=100,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.rosout_sub = self.create_subscription(
            Log,
            rosout_topic,
            self._on_rosout,
            rosout_qos,
        )



        self.timer = self.create_timer(0.05, self._drain_outgoing)

    def _on_msg(self, msg: String):
        self.incoming_queue.put(msg.data)

    def _on_rosout(self, msg: Log):
        print("UI GOT ROSOUT:", msg.level, msg.name, msg.msg, flush=True)
        # Only forward errors (or WARN+ if configured)
        if msg.level >= self.min_severity:
            line = f"[ROS {msg.level}] {msg.name}: {msg.msg}"
            self.ros_error_queue.put(line)

    def _drain_outgoing(self):
        try:
            while True:
                text = self.outgoing_queue.get_nowait()
                self.pub.publish(String(data=text))
        except queue.Empty:
            pass


class App(tk.Tk):
    def __init__(self, sub_topic="/chatter", pub_topic="/chatter"):
        super().__init__()
        self.title("Zirbi Touchscreen UI")
        self.geometry("900x600")

        self._app_icon = tk.PhotoImage(file="src/arlab/code/arlab_ui/assets/Zirbi.png")
        self.iconphoto(True, self._app_icon)


        self.buttons_data = []
        self.secondary_enabled = tk.BooleanVar(value=False)

        # ROS plumbing
        self.sub_topic = sub_topic
        self.pub_topic = pub_topic
        self.incoming_queue: "queue.Queue[str]" = queue.Queue()
        self.outgoing_queue: "queue.Queue[str]" = queue.Queue()

        self.ros_error_queue: "queue.Queue[str]" = queue.Queue()
        self.ros_error_buffer: list[str] = [] 

        self.ros_node = None

        self._build_layout()
        self._load_config()
        self._rebuild_tabs()

        self.screensaver_seconds = 10  # inactivity timeout
        self._screensaver_on = False

        self._build_screensaver("/workspace/src/arlab/code/arlab_ui/assets/Zirbi.gif")

        # Any user interaction resets timer + hides saver
        self.bind_all("<Any-KeyPress>", self._on_user_activity)
        self.bind_all("<Any-Button>", self._on_user_activity)
        self.bind_all("<Motion>", self._on_user_activity)

        # Touchscreens sometimes behave like mouse button events; this covers most cases.
        self._reset_screensaver_timer()


        # ROS thread + Tk polling
        self.ros_thread = threading.Thread(target=self._ros_spin, daemon=True)
        self.ros_thread.start()

        self.after(50, self._drain_incoming)
        self.after(50, self._drain_ros_errors)

        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---------- Layout ----------
    def _build_layout(self):
        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)

        main = ttk.Frame(self)
        main.grid(row=0, column=0, sticky="nsew")
        main.columnconfigure(0, weight=1)
        main.rowconfigure(0, weight=1)
        main.rowconfigure(1, weight=2)

        # Chat
        chat = ttk.Frame(main, padding=8)
        chat.grid(row=0, column=0, sticky="nsew")
        chat.columnconfigure(0, weight=1)
        chat.rowconfigure(1, weight=1)

        ttk.Label(chat, text=f"Sub: {self.sub_topic}   Pub: {self.pub_topic}").grid(
            row=0, column=0, sticky="w"
        )

        self.chat_text = tk.Text(chat, wrap="word", height=6)
        self.chat_text.grid(row=1, column=0, sticky="nsew", pady=(6, 6))
        self.chat_text.insert("end", "Chatbox (ROS topic output)...\n")

        # Input box
        input_row = ttk.Frame(chat)
        input_row.grid(row=2, column=0, sticky="ew")
        input_row.columnconfigure(0, weight=1)

        self.chat_entry = ttk.Entry(input_row)
        self.chat_entry.grid(row=0, column=0, sticky="ew", padx=(0, 6))
        self.chat_entry.bind("<Return>", lambda e: self._send_chat())

        ttk.Button(input_row, text="Send", command=self._send_chat).grid(row=0, column=1)

        # Tabs
        below = ttk.Frame(main, padding=8)
        below.grid(row=1, column=0, sticky="nsew")
        below.columnconfigure(0, weight=1)
        below.rowconfigure(1, weight=1)

        header = ttk.Frame(below)
        header.grid(row=0, column=0, sticky="ew")
        header.columnconfigure(0, weight=1)

        ttk.Label(header, text="Tabs").grid(row=0, column=0, sticky="w")

        ttk.Checkbutton(
            header,
            text="Hide error box (8 buttons per tab)",
            variable=self.secondary_enabled,
            command=self._on_toggle,
        ).grid(row=0, column=1, sticky="e")

        self.notebook = ttk.Notebook(below)
        self.notebook.grid(row=1, column=0, sticky="nsew")

    # ---------- Chat send ----------
    def _send_chat(self):
        text = self.chat_entry.get().strip()
        if not text:
            return
        self.chat_entry.delete(0, "end")

        # show locally + queue to ROS publisher
        self._append_chat(f"[me] {text}")
        self.outgoing_queue.put(text)

    def _append_chat(self, line: str):
        self.chat_text.insert("end", line + "\n")
        self.chat_text.see("end")

    def _drain_incoming(self):
        try:
            while True:
                line = self.incoming_queue.get_nowait()
                self._append_chat(line)
        except queue.Empty:
            pass
        self.after(50, self._drain_incoming)

    # ---------- Config ----------
    def _load_config(self):
        try:
            raw = CONFIG_PATH.read_text(encoding="utf-8")
            cfg = json.loads(raw)
            data = cfg.get("buttons", [])
            if not isinstance(data, list):
                raise ValueError("'buttons' must be a list")
            self.buttons_data = data
        except Exception as e:
            self.buttons_data = [{"name": "Config load failed", "cmd": ""}]
            self._append_chat(f"[CONFIG] Failed to load {CONFIG_PATH}: {e}")

    # ---------- Tabs ----------
    def _rebuild_tabs(self):
        try:
            current_index = self.notebook.index(self.notebook.select())
        except Exception:
            current_index = 0

        for tab_id in self.notebook.tabs():
            self.notebook.forget(tab_id)

        per_tab = 8 if self.secondary_enabled.get() else 4
        total = len(self.buttons_data)
        num_tabs = max(1, math.ceil(total / per_tab))

        for t in range(num_tabs):
            start = t * per_tab
            end = min(start + per_tab, total)
            chunk = self.buttons_data[start:end]
            tab = self._make_tab_frame(tab_buttons=chunk)
            self.notebook.add(tab, text=f"Tab {t+1}")

        self.notebook.select(min(current_index, num_tabs - 1))

    def _make_tab_frame(self, tab_buttons):
        outer = ttk.Frame(self.notebook, padding=6)
        outer.columnconfigure(0, weight=1)
        outer.rowconfigure(0, weight=1)
        outer.rowconfigure(1, weight=1)

        top = ttk.LabelFrame(outer, text="Buttons")
        top.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)
        top.columnconfigure(0, weight=1)
        top.rowconfigure(0, weight=1)

        top_grid = ttk.Frame(top, padding=8)
        top_grid.grid(row=0, column=0, sticky="nsew")
        self._populate_button_grid(top_grid, tab_buttons[:4], slots=4)

        bottom = ttk.LabelFrame(outer, text="Errors / More")
        bottom.grid(row=1, column=0, sticky="nsew", padx=4, pady=4)
        bottom.columnconfigure(0, weight=1)
        bottom.rowconfigure(0, weight=1)

        if self.secondary_enabled.get():
            bottom_grid = ttk.Frame(bottom, padding=8)
            bottom_grid.grid(row=0, column=0, sticky="nsew")
            self._populate_button_grid(bottom_grid, tab_buttons[4:8], slots=4)
            outer.error_text = None
        else:
            txt = tk.Text(bottom, wrap="word")
            txt.grid(row=0, column=0, sticky="nsew")
            txt.insert("end", "ROS errors (from /rosout):\n")

            # preload buffer so switching/toggling doesn't lose history
            for line in self.ros_error_buffer:
                txt.insert("end", line + "\n")
            txt.see("end")

            outer.error_text = txt

        return outer

    def _populate_button_grid(self, parent, button_items, slots: int):
        cols = 2
        rows = slots // cols

        for c in range(cols):
            parent.columnconfigure(c, weight=1, uniform="btncol")
        for r in range(rows):
            parent.rowconfigure(r, weight=1)

        for i in range(slots):
            r = i // cols
            c = i % cols

            if i < len(button_items):
                item = button_items[i]
                name = str(item.get("name", f"Button {i+1}"))
                cmd = str(item.get("cmd", ""))

                btn = ttk.Button(
                    parent,
                    text=name,
                    command=lambda n=name, command_str=cmd: self._run_command(n, command_str),
                )
                btn.grid(row=r, column=c, sticky="nsew", padx=6, pady=6)
            else:
                ttk.Label(parent, text="").grid(row=r, column=c, sticky="nsew", padx=6, pady=6)

    def _on_toggle(self):
        self._rebuild_tabs()

    # --------- Screensaver ----------
    def _build_screensaver(self, image_path: str):
        self._screensaver_frame = tk.Frame(
            self, bg="white", bd=0, highlightthickness=0, relief="flat"
        )
        self._screensaver_frame.place_forget()

        self._screensaver_label = tk.Label(
            self._screensaver_frame,
            bg="white",
            bd=0,
            highlightthickness=0,
            padx=0,
            pady=0,
            relief="flat",
        )
        self._screensaver_label.pack(fill="both", expand=True)

        # animation state
        self._gif_frames = []
        self._gif_index = 0
        self._gif_after_id = None
        self._gif_delay_ms = 80  # default frame delay if not specified in GIF

        self._load_screensaver_media(image_path)

    def _reset_screensaver_timer(self):
        # Cancel old timer if exists
        if hasattr(self, "_screensaver_after_id") and self._screensaver_after_id is not None:
            try:
                self.after_cancel(self._screensaver_after_id)
            except Exception:
                pass

        self._screensaver_after_id = self.after(
            int(self.screensaver_seconds * 1000),
            self._show_screensaver
        )

    def _show_screensaver(self):
        self._screensaver_on = True
        self._screensaver_frame.place(x=0, y=0, relwidth=1, relheight=1)
        self._screensaver_frame.lift()
        self._start_gif()  # start animation if it's a GIF

    def _hide_screensaver(self):
        self._screensaver_on = False
        self._stop_gif()   # stop animation timer
        self._screensaver_frame.place_forget()


    def _on_user_activity(self, event=None):
        # Wake on first interaction
        if getattr(self, "_screensaver_on", False):
            self._hide_screensaver()

        self._reset_screensaver_timer()

    def _load_screensaver_media(self, path: str):
        # stop previous animation
        self._stop_gif()

        self._screensaver_path = path
        lower = path.lower()

        if lower.endswith(".gif"):
            self._gif_frames = self._load_gif_frames(path)
            self._gif_index = 0
            self._screensaver_label.configure(image=self._gif_frames[0])
        else:
            # static image (png/gif single-frame/etc.)
            self._screensaver_img = tk.PhotoImage(file=path)
            self._screensaver_label.configure(image=self._screensaver_img)

    def _load_gif_frames(self, path: str) -> list[tk.PhotoImage]:
        frames = []
        i = 0
        while True:
            try:
                frames.append(tk.PhotoImage(file=path, format=f"gif -index {i}"))
                i += 1
            except tk.TclError:
                break

        if not frames:
            raise RuntimeError(f"Could not read any frames from GIF: {path}")
        return frames

    def _start_gif(self):
        # only animate multiple frames are available and screensaver is on
        if not getattr(self, "_screensaver_on", False):
            return
        if not getattr(self, "_gif_frames", None) or len(self._gif_frames) <= 1:
            return
        if self._gif_after_id is not None:
            return  # already running

        def step():
            if not getattr(self, "_screensaver_on", False):
                self._gif_after_id = None
                return

            self._gif_index = (self._gif_index + 1) % len(self._gif_frames)
            self._screensaver_label.configure(image=self._gif_frames[self._gif_index])
            self._gif_after_id = self.after(self._gif_delay_ms, step)

        self._gif_after_id = self.after(self._gif_delay_ms, step)

    def _stop_gif(self):
        if getattr(self, "_gif_after_id", None) is not None:
            try:
                self.after_cancel(self._gif_after_id)
            except Exception:
                pass
            self._gif_after_id = None



    # ---------- ROS errors -> error box (when visible) ----------
    def _append_error(self, line: str):
        self.ros_error_buffer.append(line)

        tab_id = self.notebook.select()
        tab_widget = self.nametowidget(tab_id)
        target = getattr(tab_widget, "error_text", None)

        if target is not None:
            target.insert("end", line + "\n")
            target.see("end")

    def _drain_ros_errors(self):
        try:
            while True:
                line = self.ros_error_queue.get_nowait()
                self._append_error(line)
        except queue.Empty:
            pass
        self.after(50, self._drain_ros_errors)

    # ---------- Commands ----------
    def _run_command(self, name: str, cmd: str):
        def log(text: str):
            for line in text.splitlines():
                self._append_chat(f"[cmd] {line}")

        if not cmd.strip():
            log(f"[{name}] No command set.")
            return

        try:
            log(f"$ {cmd}")
            completed = subprocess.run(cmd, shell=True, text=True, capture_output=True)
            if completed.stdout:
                log(completed.stdout)
            if completed.stderr:
                log(completed.stderr)
        except Exception as e:
            log(f"Failed to run command: {e}")

    # ---------- ROS thread ----------
    def _ros_spin(self):
        rclpy.init(args=None)
        self.ros_node = RosBridge(
            sub_topic=self.sub_topic,
            pub_topic=self.pub_topic,
            incoming_queue=self.incoming_queue,
            outgoing_queue=self.outgoing_queue,
            ros_error_queue=self.ros_error_queue,
            rosout_topic="/rosout",
            min_severity=Log.WARN,  # Log.WARN or Log.Error
        )

        try:
            rclpy.spin(self.ros_node)
        except Exception as e:
            self.incoming_queue.put(f"[ROS ERROR] {e}")
        finally:
            try:
                self.ros_node.destroy_node()
            except Exception:
                pass
            try:
                rclpy.shutdown()
            except Exception:
                pass

    def _on_close(self):
        try:
            rclpy.shutdown()
        except Exception:
            pass
        self.destroy()


if __name__ == "__main__":
    App(sub_topic="/chatter", pub_topic="/chatter").mainloop()
