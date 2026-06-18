#!/usr/bin/env python3
from datetime import datetime
import os
from enum import Enum
from pathlib import Path
import queue
import shlex
import signal
import subprocess
import threading
from typing import Callable, Dict, Optional
import zipfile

try:
    import tkinter as tk
    from tkinter import messagebox, simpledialog, ttk
except ImportError:  # pragma: no cover
    tk = None
    messagebox = None
    simpledialog = None
    ttk = None

try:
    from PIL import Image, ImageTk  # type: ignore[import-untyped]
except ImportError:  # pragma: no cover
    Image = None
    ImageTk = None

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32, String
import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[4]
SCRIPTS_DIR = PROJECT_ROOT / "scripts"
RUNTIME_DIR = PROJECT_ROOT / "runtime"
ROOMS_FILE = RUNTIME_DIR / "rooms.yaml"
MAPS_DIR = RUNTIME_DIR / "maps"
DIAGNOSTICS_DIR = RUNTIME_DIR / "diagnostics"
ROS_LOG_DIR = PROJECT_ROOT / "workspace" / "log" / "control_center"
NAV2_CONFIG_FILE = PROJECT_ROOT / "workspace" / "src" / "virtual_indoor_nav" / "config" / "nav2_params.yaml"
EXPLORATION_CONFIG_FILE = PROJECT_ROOT / "workspace" / "src" / "virtual_indoor_nav" / "config" / "exploration_params.yaml"
SLAM_CONFIG_FILE = PROJECT_ROOT / "workspace" / "src" / "virtual_indoor_nav" / "config" / "slam_toolbox.yaml"


class AppState(Enum):
    IDLE = "IDLE"
    EXPLORING = "EXPLORING"
    EXPLORATION_DONE = "EXPLORATION_DONE"
    EXPLORING_FAILED = "EXPLORING_FAILED"
    MAP_SAVED = "MAP_SAVED"
    NAMED = "NAMED"
    NAVIGATING = "NAVIGATING"


class ManagedProcess:
    def __init__(self, name: str, process: subprocess.Popen[str]) -> None:
        self.name = name
        self.process = process


class StatusNode(Node):
    def __init__(
        self,
        on_exploration_status: Callable[[str], None],
        on_exploration_progress: Callable[[float], None],
        on_room_status: Callable[[str], None],
        on_exploration_coverage: Callable[[float], None],
    ) -> None:
        super().__init__("control_center_status")
        self.create_subscription(
            String,
            "/exploration_status",
            lambda msg: on_exploration_status(msg.data),
            10,
        )
        self.create_subscription(
            Float32,
            "/exploration_coverage",
            lambda msg: on_exploration_coverage(float(msg.data)),
            10,
        )
        self.create_subscription(
            String,
            "/room_status",
            lambda msg: on_room_status(msg.data),
            10,
        )


class ControlCenterApp:
    def __init__(self) -> None:
        if tk is None or ttk is None or messagebox is None or simpledialog is None:
            raise RuntimeError("Tkinter is not available.")

        rclpy.init(args=None)
        self.root = tk.Tk()
        self.root.title("Virtual Indoor Navigation v3")
        self.root.geometry("1280x820")
        self.root.minsize(1024, 720)

        self.state = AppState.IDLE
        self.log_queue: "queue.Queue[str]" = queue.Queue()
        self.processes: Dict[str, ManagedProcess] = {}
        self.progress_var = tk.DoubleVar(value=0.0)
        self.progress_text_var = tk.StringVar(value="覆盖率：0%")
        self.status_var = tk.StringVar(value="状态：空闲")
        self.detail_status_var = tk.StringVar(value="系统就绪，请点击「开始建图并自动探索」。")
        self.room_detail_var = tk.StringVar(value="区域列表：空")
        self.step_vars = {
            AppState.IDLE: tk.StringVar(value="● Step 1: 开始建图并自动探索"),
            AppState.EXPLORATION_DONE: tk.StringVar(value="○ Step 2: 保存地图"),
            AppState.EXPLORING_FAILED: tk.StringVar(value="✗ 探索未完成"),
            AppState.MAP_SAVED: tk.StringVar(value="○ Step 3: 命名区域"),
            AppState.NAMED: tk.StringVar(value="○ Step 4: 按名字导航"),
        }

        # Map preview state
        self.map_image_tk = None       # PhotoImage for Canvas
        self.map_pil_image = None      # Original PIL Image (PGM)
        self.map_resolution = 0.05     # meters/pixel
        self.map_origin_x = 0.0        # world x of map origin
        self.map_origin_y = 0.0        # world y of map origin
        self.map_image_offset_x = 0    # Canvas offset of the image
        self.map_image_offset_y = 0
        self.map_display_scale = 1.0   # display scale factor
        self.map_markers = []          # list of (canvas_id, name) for room markers

        self.status_node = StatusNode(
            self._queue_exploration_status,
            self._queue_exploration_progress,
            self._queue_room_status,
            self._queue_exploration_coverage,
        )
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.status_node)
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()

        self._build_layout()
        self._refresh_rooms()
        self._restore_saved_session()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.after(100, self._drain_log_queue)

    def _build_layout(self) -> None:
        main = ttk.Frame(self.root, padding=14)
        main.pack(fill="both", expand=True)
        main.columnconfigure(0, weight=0)
        main.columnconfigure(1, weight=1)
        main.rowconfigure(1, weight=1)

        title = ttk.Frame(main)
        title.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 12))
        title.columnconfigure(0, weight=1)
        ttk.Label(title, text="Virtual Indoor Navigation v3", font=("Ubuntu", 18, "bold")).grid(
            row=0, column=0, sticky="w"
        )
        ttk.Label(title, textvariable=self.status_var, font=("Ubuntu", 12)).grid(
            row=0, column=1, sticky="e"
        )

        left = ttk.Frame(main)
        left.grid(row=1, column=0, sticky="nsw", padx=(0, 14))
        right = ttk.Frame(main)
        right.grid(row=1, column=1, sticky="nsew")
        right.columnconfigure(0, weight=1)
        right.rowconfigure(1, weight=1)  # map_frame expands
        right.rowconfigure(2, weight=1)  # log_frame expands

        self._build_guide(left)
        self._build_rooms(left)
        self._build_logs(right)

    def _build_guide(self, parent) -> None:
        frame = ttk.LabelFrame(parent, text="流程引导", padding=12)
        frame.pack(fill="x", pady=(0, 12))

        ttk.Label(frame, textvariable=self.step_vars[AppState.IDLE], font=("Ubuntu", 11, "bold")).pack(
            anchor="w", pady=(0, 4)
        )
        self.start_button = ttk.Button(
            frame, text="开始建图并自动探索", command=self._start_auto_mapping
        )
        self.start_button.pack(fill="x", pady=(0, 10))

        ttk.Label(frame, textvariable=self.step_vars[AppState.EXPLORATION_DONE]).pack(
            anchor="w", pady=(0, 4)
        )
        self.save_map_button = ttk.Button(frame, text="保存地图", command=self._save_map)
        self.save_map_button.pack(fill="x", pady=(0, 10))

        ttk.Label(frame, textvariable=self.step_vars[AppState.MAP_SAVED]).pack(
            anchor="w", pady=(0, 4)
        )
        self.rename_button = ttk.Button(frame, text="重命名选中区域", command=self._rename_selected_room)
        self.rename_button.pack(fill="x", pady=(0, 10))

        ttk.Label(frame, textvariable=self.step_vars[AppState.NAMED]).pack(
            anchor="w", pady=(0, 4)
        )
        # Navigation button moved to rooms panel (with dropdown)

        self.progress_bar = ttk.Progressbar(
            frame, variable=self.progress_var, maximum=100.0, length=280
        )
        self.progress_bar.pack(fill="x")
        ttk.Label(frame, textvariable=self.progress_text_var).pack(anchor="w", pady=(6, 0))
        ttk.Label(frame, textvariable=self.detail_status_var, wraplength=300).pack(
            anchor="w", pady=(8, 10)
        )

        controls = ttk.Frame(frame)
        controls.pack(fill="x")
        ttk.Button(controls, text="停止全部", command=self._stop_all_clicked).pack(
            side="left", expand=True, fill="x", padx=(0, 6)
        )
        ttk.Button(controls, text="刷新", command=self._refresh_all).pack(
            side="left", expand=True, fill="x", padx=(6, 0)
        )
        self.diagnostics_button = ttk.Button(
            frame, text="导出诊断包", command=self._export_diagnostics
        )
        self.diagnostics_button.pack(fill="x", pady=(8, 0))

    def _build_rooms(self, parent) -> None:
        frame = ttk.LabelFrame(parent, text="区域列表", padding=12)
        frame.pack(fill="both", expand=True)

        # Navigation target dropdown
        nav_target_frame = ttk.Frame(frame)
        nav_target_frame.pack(fill="x", pady=(0, 6))
        ttk.Label(nav_target_frame, text="导航目标：").pack(side="left", padx=(0, 6))
        self.nav_target_var = tk.StringVar()
        self.nav_target_combo = ttk.Combobox(
            nav_target_frame,
            textvariable=self.nav_target_var,
            state="readonly",
            width=18,
        )
        self.nav_target_combo.pack(side="left", fill="x", expand=True)
        self.nav_target_combo.bind("<<ComboboxSelected>>", self._on_nav_target_selected)

        self.navigate_button = ttk.Button(
            frame,
            text="导航到选中区域（不重启）",
            command=self._navigate_selected_room,
        )
        self.navigate_button.pack(fill="x", pady=(0, 8))
        ttk.Button(frame, text="自动检测房间", command=self._auto_detect_rooms).pack(
            fill="x", pady=(0, 8)
        )

        self.rooms_listbox = tk.Listbox(frame, height=14, exportselection=False)
        self.rooms_listbox.pack(fill="both", expand=True)
        self.rooms_listbox.bind("<<ListboxSelect>>", self._on_listbox_select)

        ttk.Label(frame, textvariable=self.room_detail_var, wraplength=300).pack(
            fill="x", pady=(6, 6)
        )

    def _build_logs(self, parent) -> None:
        status = ttk.LabelFrame(parent, text="状态信息", padding=12)
        status.grid(row=0, column=0, sticky="ew", pady=(0, 12))
        ttk.Label(
            status,
            text=(
                "流程：开始自动探索 -> 完成后保存地图 -> "
                "点击地图命名区域 -> 选择区域导航（不重启）。"
            ),
            wraplength=720,
        ).pack(anchor="w")

        # Map preview panel
        map_frame = ttk.LabelFrame(parent, text="地图预览（点击 free 区域命名）", padding=4)
        map_frame.grid(row=1, column=0, sticky="nsew", pady=(0, 10))
        map_frame.columnconfigure(0, weight=1)
        map_frame.rowconfigure(0, weight=1)

        self.map_canvas = tk.Canvas(
            map_frame,
            width=560,
            height=320,
            bg="#2b2b2b",
            highlightthickness=0,
            cursor="crosshair",
        )
        self.map_canvas.grid(row=0, column=0, sticky="nsew")
        self.map_canvas.bind("<Button-1>", self._on_map_click)
        self.map_canvas.bind("<Configure>", self._on_map_resize)
        self.map_label = ttk.Label(
            map_frame,
            text="保存地图后将在此显示预览。点击白色区域可命名。",
            anchor="center",
        )
        self.map_label.grid(row=1, column=0, sticky="ew", pady=(4, 0))

        log_frame = ttk.LabelFrame(parent, text="日志输出", padding=12)
        log_frame.grid(row=2, column=0, sticky="nsew")
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)

        self.log_text = tk.Text(log_frame, height=16, wrap="word")
        self.log_text.grid(row=0, column=0, sticky="nsew")
        scroll = ttk.Scrollbar(log_frame, orient="vertical", command=self.log_text.yview)
        scroll.grid(row=0, column=1, sticky="ns")
        self.log_text.configure(yscrollcommand=scroll.set)

    def _set_state(self, state: AppState) -> None:
        self.state = state
        self.status_var.set(f"状态：{state.value}")

        self.start_button.configure(state="normal" if state in {AppState.IDLE, AppState.EXPLORING_FAILED} else "disabled")
        self.save_map_button.configure(
            state="normal" if state in {AppState.EXPLORATION_DONE, AppState.EXPLORING_FAILED, AppState.NAMED} else "disabled"
        )
        self.rename_button.configure(
            state="normal" if state in {AppState.MAP_SAVED, AppState.NAMED} else "disabled"
        )
        self.navigate_button.configure(
            state="normal" if state in {AppState.NAMED, AppState.MAP_SAVED} else "disabled"
        )

        self.step_vars[AppState.IDLE].set(
            ("●" if state == AppState.IDLE else "✓") + " Step 1: 开始建图并自动探索"
        )
        self.step_vars[AppState.EXPLORATION_DONE].set(
            ("●" if state == AppState.EXPLORATION_DONE else "✓" if state in {AppState.MAP_SAVED, AppState.NAMED, AppState.NAVIGATING} else "○")
            + " Step 2: 保存地图"
        )
        self.step_vars[AppState.EXPLORING_FAILED].set(
            ("✗" if state == AppState.EXPLORING_FAILED else "○") + " 探索未完成（可继续或手动保存）"
        )
        self.step_vars[AppState.MAP_SAVED].set(
            ("●" if state == AppState.MAP_SAVED else "✓" if state in {AppState.NAMED, AppState.NAVIGATING} else "○")
            + " Step 3: 重命名区域"
        )
        self.step_vars[AppState.NAMED].set(
            ("●" if state == AppState.NAMED else "○") + " Step 4: 导航到命名区域"
        )

    def _set_detail_status(self, text: str) -> None:
        self.detail_status_var.set(text)
        self._log(f"[status] {text}")

    def _restore_saved_session(self) -> None:
        rooms = self._load_rooms()
        has_map = (MAPS_DIR / "generated_map.pgm").exists() and (
            MAPS_DIR / "generated_map.yaml"
        ).exists()
        if has_map:
            self._load_pgm_for_display()
        if rooms:
            self._set_state(AppState.NAMED)
            self._set_detail_status("已加载已有地图和区域，可直接选择目标导航。")
            return
        if has_map:
            self._set_state(AppState.MAP_SAVED)
            self._set_detail_status("已加载已有地图，请点击地图命名区域。")
            return
        self._set_state(AppState.IDLE)
        self._set_detail_status("系统就绪，请点击「开始建图并自动探索」。")

    def _log(self, text: str) -> None:
        self.log_queue.put(text.rstrip())

    def _drain_log_queue(self) -> None:
        while True:
            try:
                line = self.log_queue.get_nowait()
            except queue.Empty:
                break
            self.log_text.insert("end", line + "\n")
            self.log_text.see("end")
        self._check_processes()
        self.root.after(100, self._drain_log_queue)

    def _check_processes(self) -> None:
        for key, managed in list(self.processes.items()):
            if managed.process.poll() is not None:
                self._log(f"[process] {managed.name} 已退出，返回码 {managed.process.returncode}")
                del self.processes[key]
                if key == "app_system":
                    if self.state == AppState.EXPLORING:
                        self._set_state(AppState.EXPLORING_FAILED)
                        self._set_detail_status(
                            "统一系统进程已退出，自动探索停止。请查看日志后重新开始。"
                        )
                    elif self.state == AppState.NAVIGATING:
                        rooms = self._load_rooms()
                        self._set_state(AppState.NAMED if rooms else AppState.MAP_SAVED)
                        self._set_detail_status(
                            "统一系统进程已退出，导航已停止。请查看日志后重新开始。"
                        )

    def _run_short_command(
        self,
        name: str,
        command: list[str],
        on_done: Optional[Callable[[int], None]] = None,
    ) -> None:
        def worker() -> None:
            self._log(f"[run] {name}: {' '.join(shlex.quote(part) for part in command)}")
            env = os.environ.copy()
            env["ROS_LOG_DIR"] = str(ROS_LOG_DIR)
            ROS_LOG_DIR.mkdir(parents=True, exist_ok=True)
            completed = subprocess.run(
                command,
                cwd=str(PROJECT_ROOT),
                env=env,
                text=True,
                capture_output=True,
            )
            if completed.stdout:
                self._log(completed.stdout)
            if completed.stderr:
                self._log(completed.stderr)
            if on_done:
                self.root.after(0, lambda: on_done(completed.returncode))

        threading.Thread(target=worker, daemon=True).start()

    def _start_long_process(self, key: str, name: str, command: list[str]) -> None:
        if key in self.processes and self.processes[key].process.poll() is None:
            self._set_detail_status(f"{name} 已经在运行")
            return

        self._log(f"[start] {name}: {' '.join(shlex.quote(part) for part in command)}")
        env = os.environ.copy()
        env["ROS_LOG_DIR"] = str(ROS_LOG_DIR)
        ROS_LOG_DIR.mkdir(parents=True, exist_ok=True)
        process = subprocess.Popen(
            command,
            cwd=str(PROJECT_ROOT),
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            start_new_session=True,
        )
        self.processes[key] = ManagedProcess(name, process)

        def reader() -> None:
            assert process.stdout is not None
            for line in process.stdout:
                self._log(f"[{name}] {line.rstrip()}")

        threading.Thread(target=reader, daemon=True).start()

    def _stop_managed_processes(self) -> None:
        for key, managed in list(self.processes.items()):
            process = managed.process
            if process.poll() is not None:
                del self.processes[key]
                continue
            try:
                os.killpg(process.pid, signal.SIGTERM)
                self._log(f"[stop] 已停止 {managed.name}")
            except ProcessLookupError:
                pass
            del self.processes[key]

    def _start_auto_mapping(self) -> None:
        # Confirm before clearing old map & rooms
        if self.state != AppState.IDLE and self.state != AppState.EXPLORING_FAILED:
            if not messagebox.askyesno(
                "开始新任务",
                "开始新的建图会清空当前区域命名和地图预览，是否继续？",
                parent=self.root,
            ):
                return

        self._set_state(AppState.EXPLORING)
        self.progress_var.set(0.0)
        self.progress_text_var.set("覆盖率：0%")
        self._set_detail_status("正在启动自动建图和自主探索。")
        self._reset_rooms_for_new_map()
        self._stop_managed_processes()

        def after_cleanup(return_code: int) -> None:
            if return_code != 0:
                self._set_state(AppState.IDLE)
                self._set_detail_status("启动前清理 Gazebo 失败。")
                return
            self._start_long_process(
                "app_system",
                "统一系统",
                ["bash", str(SCRIPTS_DIR / "run_app_system.sh")],
            )

        self._run_short_command(
            "cleanup_gazebo",
            ["bash", str(SCRIPTS_DIR / "cleanup_gazebo.sh")],
            on_done=after_cleanup,
        )

    def _save_map(self) -> None:
        self._set_detail_status("正在保存地图。")

        def after_save(return_code: int) -> None:
            if return_code != 0:
                self._set_detail_status("地图保存失败，请确认建图进程仍在运行且 /map 存在。")
                return
            self._set_state(AppState.MAP_SAVED)
            self._set_detail_status("地图已保存。旧区域已清空，请在预览图上点击命名。")
            self._reset_rooms_for_new_map()
            self._load_pgm_for_display()

        self._run_short_command(
            "save_map",
            ["bash", str(SCRIPTS_DIR / "save_map.sh")],
            on_done=after_save,
        )

    def _reset_rooms_for_new_map(self) -> None:
        """Clear rooms.yaml and UI for a new map session."""
        ROOMS_FILE.parent.mkdir(parents=True, exist_ok=True)
        ROOMS_FILE.write_text("rooms: []\n", encoding="utf-8")
        self._refresh_rooms()
        self._log("[rooms] 区域列表已清空（新地图）")

    # ── Diagnostics ───────────────────────────────────────────────────────

    def _export_diagnostics(self) -> None:
        self._set_detail_status("正在导出诊断包。")
        self.diagnostics_button.configure(state="disabled")

        def worker() -> None:
            try:
                zip_path = self._create_diagnostics_zip()
            except Exception as exc:  # noqa: BLE001
                self.root.after(0, lambda error=exc: self._finish_diagnostics_export(None, error))
                return
            self.root.after(0, lambda: self._finish_diagnostics_export(zip_path, None))

        threading.Thread(target=worker, daemon=True).start()

    def _finish_diagnostics_export(
        self, zip_path: Optional[Path], error: Optional[BaseException]
    ) -> None:
        self.diagnostics_button.configure(state="normal")
        if error is not None or zip_path is None:
            self._set_detail_status(f"诊断包导出失败：{error}")
            messagebox.showerror("导出诊断包失败", str(error), parent=self.root)
            return
        self._set_detail_status(f"诊断包已导出：{zip_path}")
        messagebox.showinfo("诊断包已导出", str(zip_path), parent=self.root)

    def _create_diagnostics_zip(self) -> Path:
        DIAGNOSTICS_DIR.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        zip_path = DIAGNOSTICS_DIR / f"diagnostics_{stamp}.zip"
        manifest_lines = [
            f"created_at: {datetime.now().isoformat(timespec='seconds')}",
            f"app_state: {self.state.value}",
            f"status: {self.status_var.get()}",
            f"detail_status: {self.detail_status_var.get()}",
            f"nav_target: {self.nav_target_var.get()}",
            "",
            "files:",
        ]

        files = self._diagnostic_files()
        with zipfile.ZipFile(zip_path, "w", compression=zipfile.ZIP_DEFLATED) as archive:
            for source, archive_name in files:
                if not source.exists() or not source.is_file():
                    manifest_lines.append(f"  missing: {source}")
                    continue
                archive.write(source, archive_name)
                manifest_lines.append(f"  included: {archive_name} <- {source}")
            archive.writestr("manifest.txt", "\n".join(manifest_lines) + "\n")

        return zip_path

    def _diagnostic_files(self) -> list[tuple[Path, str]]:
        files: list[tuple[Path, str]] = []
        for source, archive_name in (
            (ROOMS_FILE, "runtime/rooms.yaml"),
            (MAPS_DIR / "generated_map.yaml", "runtime/maps/generated_map.yaml"),
            (MAPS_DIR / "generated_map.pgm", "runtime/maps/generated_map.pgm"),
            (NAV2_CONFIG_FILE, "config/nav2_params.yaml"),
            (EXPLORATION_CONFIG_FILE, "config/exploration_params.yaml"),
            (SLAM_CONFIG_FILE, "config/slam_toolbox.yaml"),
        ):
            files.append((source, archive_name))

        log_patterns = (
            "planner_server_*.log",
            "controller_server_*.log",
            "bt_navigator_*.log",
            "behavior_server_*.log",
            "lifecycle_manager_*.log",
            "python3_*.log",
            "async_slam_toolbox_node_*.log",
        )
        for pattern in log_patterns:
            for index, log_file in enumerate(self._latest_files(ROS_LOG_DIR, pattern, limit=3), start=1):
                files.append((log_file, f"logs/{pattern.replace('*', str(index))}"))
        return files

    def _latest_files(self, directory: Path, pattern: str, limit: int) -> list[Path]:
        if not directory.exists():
            return []
        candidates = [path for path in directory.glob(pattern) if path.is_file()]
        candidates.sort(key=lambda path: path.stat().st_mtime, reverse=True)
        return candidates[:limit]

    # ── Map preview and click-to-name ──────────────────────────────────────

    def _load_pgm_for_display(self) -> None:
        """Load generated_map.pgm + .yaml and display on Canvas."""
        if Image is None or ImageTk is None:
            self._set_detail_status("Pillow 未安装，无法显示地图预览。请安装 python3-pil。")
            self.map_label.config(text="⚠ Pillow 未安装，地图预览不可用。")
            return

        pgm_path = RUNTIME_DIR / "maps" / "generated_map.pgm"
        yaml_path = RUNTIME_DIR / "maps" / "generated_map.yaml"

        if not pgm_path.exists():
            self._set_detail_status(f"PGM 地图文件不存在：{pgm_path}")
            self.map_label.config(text="⚠ 地图文件不存在，请先保存地图。")
            return

        # Parse YAML for resolution and origin
        if yaml_path.exists():
            try:
                map_meta = yaml.safe_load(yaml_path.read_text(encoding="utf-8")) or {}
                self.map_resolution = float(map_meta.get("resolution", 0.05))
                origin = map_meta.get("origin", [0.0, 0.0, 0.0])
                self.map_origin_x = float(origin[0])
                self.map_origin_y = float(origin[1])
            except Exception:
                self.map_resolution = 0.05
                self.map_origin_x = 0.0
                self.map_origin_y = 0.0
        else:
            self.map_resolution = 0.05
            self.map_origin_x = 0.0
            self.map_origin_y = 0.0

        try:
            self.map_pil_image = Image.open(str(pgm_path))
        except Exception as exc:
            self._set_detail_status(f"无法打开 PGM 文件：{exc}")
            self.map_label.config(text=f"⚠ 无法加载地图图片：{exc}")
            return

        self.map_label.config(
            text=f"地图已加载 | 分辨率: {self.map_resolution:.3f} m/pix | "
            f"尺寸: {self.map_pil_image.width}x{self.map_pil_image.height} px"
        )
        self._redraw_map_on_canvas()
        self._redraw_map_markers()
        self._set_detail_status(
            "地图预览已加载。点击白色（free）区域可为该位置命名。"
        )

    def _redraw_map_on_canvas(self) -> None:
        """Scale and display the PGM image on the Canvas."""
        if self.map_pil_image is None:
            return

        canvas_w = self.map_canvas.winfo_width()
        canvas_h = self.map_canvas.winfo_height()
        if canvas_w < 10 or canvas_h < 10:
            canvas_w = 560
            canvas_h = 320

        img_w = self.map_pil_image.width
        img_h = self.map_pil_image.height

        # Fit image to canvas while preserving aspect ratio
        scale = min(canvas_w / img_w, canvas_h / img_h, 1.0)
        self.map_display_scale = scale
        new_w = int(img_w * scale)
        new_h = int(img_h * scale)

        resized = self.map_pil_image.resize((new_w, new_h), Image.NEAREST)
        self.map_image_tk = ImageTk.PhotoImage(resized)

        # Center image in canvas
        self.map_image_offset_x = (canvas_w - new_w) // 2
        self.map_image_offset_y = (canvas_h - new_h) // 2

        self.map_canvas.delete("all")
        self.map_canvas.create_image(
            self.map_image_offset_x,
            self.map_image_offset_y,
            anchor="nw",
            image=self.map_image_tk,
            tags="map_bg",
        )

    def _on_map_resize(self, _event=None) -> None:
        """Re-draw map when Canvas is resized."""
        if self.map_pil_image is not None:
            self._redraw_map_on_canvas()
            self._redraw_map_markers()

    def _canvas_to_world(self, canvas_x: int, canvas_y: int) -> tuple[float, float]:
        """Convert Canvas pixel coordinates to world (map frame) coordinates."""
        if self.map_pil_image is None:
            return 0.0, 0.0

        # Canvas → image pixel
        image_x = (canvas_x - self.map_image_offset_x) / self.map_display_scale
        image_y = (canvas_y - self.map_image_offset_y) / self.map_display_scale

        img_h = self.map_pil_image.height

        # Image pixel → world (PGM y flips: image top-left vs map bottom-left)
        world_x = self.map_origin_x + image_x * self.map_resolution
        world_y = self.map_origin_y + (img_h - image_y) * self.map_resolution

        return world_x, world_y

    def _world_to_canvas(self, world_x: float, world_y: float) -> tuple[int, int]:
        """Convert world coordinates to Canvas pixel coordinates."""
        if self.map_pil_image is None:
            return 0, 0

        img_h = self.map_pil_image.height

        image_x = (world_x - self.map_origin_x) / self.map_resolution
        image_y = img_h - (world_y - self.map_origin_y) / self.map_resolution

        canvas_x = int(image_x * self.map_display_scale + self.map_image_offset_x)
        canvas_y = int(image_y * self.map_display_scale + self.map_image_offset_y)

        return canvas_x, canvas_y

    def _get_pgm_pixel(self, world_x: float, world_y: float) -> Optional[int]:
        """Return PGM pixel value (0-255) at world coordinates, or None if out of bounds."""
        if self.map_pil_image is None:
            return None

        img_h = self.map_pil_image.height
        image_x = int((world_x - self.map_origin_x) / self.map_resolution)
        image_y = int(img_h - (world_y - self.map_origin_y) / self.map_resolution)

        if (
            image_x < 0
            or image_y < 0
            or image_x >= self.map_pil_image.width
            or image_y >= self.map_pil_image.height
        ):
            return None

        return self.map_pil_image.getpixel((image_x, image_y))

    def _on_map_click(self, event: tk.Event) -> None:
        """Handle click on map Canvas: convert to world coords → naming popup."""
        if self.map_pil_image is None:
            return

        world_x, world_y = self._canvas_to_world(event.x, event.y)

        # Check pixel validity
        pixel = self._get_pgm_pixel(world_x, world_y)
        if pixel is None:
            messagebox.showinfo("提示", "点击位置超出地图范围。")
            return

        # map_saver trinary PGM: 254=free, 205=unknown, 0=occupied
        if pixel <= 50:
            messagebox.showinfo(
                "提示",
                f"该位置是障碍物区域（pixel={pixel}），无法命名。\n请点击白色可通行区域。",
            )
            return

        if pixel < 250:
            messagebox.showinfo(
                "提示",
                f"该区域未完全探索或未知（pixel={pixel}），无法命名。\n请点击白色可通行区域。",
            )
            return

        # Free cell (pixel >= 250) — allow naming
        name = simpledialog.askstring(
            "命名区域",
            f"请输入该区域名称\n（世界坐标: x={world_x:.2f}, y={world_y:.2f}）",
            parent=self.root,
        )
        if not name:
            return
        name = name.strip()
        if not name:
            return

        # Check for duplicate name
        existing_rooms = self._load_rooms()
        if name in {r.get("name") for r in existing_rooms}:
            overwrite = messagebox.askyesno(
                "名称已存在",
                f"区域名称「{name}」已存在。是否覆盖？",
                parent=self.root,
            )
            if not overwrite:
                return

        # Send set command to room_nav_node
        self._send_room_command(f"set {name} {world_x:.4f} {world_y:.4f} 0.0")

        # Refresh room list and redraw markers
        self.root.after(500, self._refresh_rooms)
        self._redraw_map_markers()
        self._set_detail_status(f"地图点击命名：{name} (x={world_x:.2f}, y={world_y:.2f})")
        self._log(f"[map_click] set {name} at x={world_x:.4f}, y={world_y:.4f}")

        if self.state in {AppState.MAP_SAVED, AppState.NAMED}:
            self._set_state(AppState.NAMED)

    def _redraw_map_markers(self) -> None:
        """Draw room markers (dots + labels) on the map Canvas."""
        # Remove old markers
        self.map_canvas.delete("room_marker")
        self.map_markers = []

        if self.map_pil_image is None:
            return

        rooms = self._load_rooms()
        for room in rooms:
            try:
                wx = float(room.get("x", 0.0))
                wy = float(room.get("y", 0.0))
                name = str(room.get("name", "?"))
                source = room.get("source", "manual")
            except (ValueError, TypeError):
                continue

            cx, cy = self._world_to_canvas(wx, wy)

            # Color by source
            color = "#00ff88" if source == "map_click" else "#ffaa00"

            # Draw dot
            dot_r = 5
            dot_id = self.map_canvas.create_oval(
                cx - dot_r,
                cy - dot_r,
                cx + dot_r,
                cy + dot_r,
                fill=color,
                outline="#ffffff",
                width=1,
                tags="room_marker",
            )
            # Draw label
            label_id = self.map_canvas.create_text(
                cx + 8,
                cy - 8,
                text=name,
                anchor="w",
                fill="#ffffff",
                font=("Ubuntu", 9, "bold"),
                tags="room_marker",
            )
            self.map_markers.append((dot_id, name))
            self.map_markers.append((label_id, name))

    # ── Room management ───────────────────────────────────────────────────

    def _auto_detect_rooms(self) -> None:
        """Manually trigger auto room detection with confirmation."""
        if self._load_rooms():
            if not messagebox.askyesno(
                "自动检测房间",
                "自动检测会覆盖当前自动检测结果，但不会覆盖手动点击命名结果。是否继续？",
                parent=self.root,
            ):
                return
        self._send_room_command("auto_rooms", self._after_auto_rooms)

    def _after_auto_rooms(self, return_code: int) -> None:
        self._refresh_rooms()
        if return_code != 0:
            self._set_detail_status("自动检测房间命令发送失败。")
            return
        self._set_detail_status("区域已检测。请把区域重命名为客厅、厨房、卧室等名称。")
        if self._all_rooms_named():
            self._set_state(AppState.NAMED)
        elif self.state != AppState.NAMED:
            self._set_state(AppState.MAP_SAVED)

    def _send_room_command(
        self,
        room_command: str,
        on_done: Optional[Callable[[int], None]] = None,
    ) -> None:
        self._run_short_command(
            f"room_command:{room_command}",
            ["bash", str(SCRIPTS_DIR / "send_room_command.sh"), room_command],
            on_done=on_done,
        )

    def _rename_selected_room(self) -> None:
        old_name = self._get_selected_room_name()
        if not old_name:
            messagebox.showinfo("提示", "请先选择一个区域。")
            return
        new_name = simpledialog.askstring(
            "重命名区域",
            f"请输入「{old_name}」的新名字，例如：客厅、厨房、卧室",
            parent=self.root,
        )
        if not new_name:
            return
        new_name = new_name.strip()
        if not new_name:
            return

        def after_rename(return_code: int) -> None:
            self._refresh_rooms()
            if return_code != 0:
                self._set_detail_status(f"重命名失败：{old_name} -> {new_name}")
                return
            self._set_detail_status(f"已发送重命名命令：{old_name} -> {new_name}")
            if self._all_rooms_named():
                self._set_state(AppState.NAMED)

        self._send_room_command(f"rename {old_name} {new_name}", after_rename)

    def _navigate_selected_room(self) -> None:
        """导航到下拉框选中的区域（不重启系统，不弹出输入框）。"""
        room_name = self.nav_target_var.get().strip()
        if not room_name:
            messagebox.showinfo("提示", "请先从下拉框选择一个目标区域。")
            return

        if room_name not in {room.get("name") for room in self._load_rooms()}:
            messagebox.showinfo("提示", f"没有找到区域：{room_name}")
            return

        # Do NOT set NAVIGATING yet — wait for room_nav_node to confirm "accepted"
        self._set_detail_status(f"正在发送导航目标：{room_name}（不重启系统）")
        self._log(f"[nav] 发送导航目标：goto {room_name}")
        self._send_goto(room_name)

    def _navigate_by_name(self) -> None:
        """保留旧 API 入口，重定向到新的选择导航。"""
        self._navigate_selected_room()

    def _send_goto(self, room_name: str) -> None:
        self._set_detail_status(f"正在发送导航目标：{room_name}")
        self._send_room_command(f"goto {room_name}", lambda _code: None)

    def _refresh_rooms(self) -> None:
        rooms = self._load_rooms()
        self.rooms_listbox.delete(0, "end")
        room_names = []
        for item in rooms:
            name = item.get("name", "(unnamed)")
            source = item.get("source", "manual")
            self.rooms_listbox.insert("end", f"{name} [{source}]")
            room_names.append(str(name))
        # Sync dropdown
        current_val = self.nav_target_var.get()
        self.nav_target_combo["values"] = room_names
        if current_val in room_names:
            self.nav_target_var.set(current_val)
        elif room_names:
            self.nav_target_var.set(room_names[0])
        else:
            self.nav_target_var.set("")
        self._update_room_detail()
        self._redraw_map_markers()

    def _refresh_all(self) -> None:
        self._refresh_rooms()
        self._set_detail_status("已刷新区域列表。")
        if self.state in {AppState.MAP_SAVED, AppState.NAMED} and self._all_rooms_named():
            self._set_state(AppState.NAMED)

    def _load_rooms(self) -> list[dict]:
        if not ROOMS_FILE.exists():
            return []
        data = yaml.safe_load(ROOMS_FILE.read_text(encoding="utf-8")) or {}
        return list(data.get("rooms", []))

    def _get_selected_room_name(self) -> Optional[str]:
        """Get name from dropdown (primary) or listbox (fallback)."""
        # Primary: dropdown
        combo_val = self.nav_target_var.get().strip()
        if combo_val:
            return combo_val
        # Fallback: listbox
        selection = self.rooms_listbox.curselection()
        if not selection:
            return None
        rooms = self._load_rooms()
        index = selection[0]
        if index >= len(rooms):
            return None
        name = rooms[index].get("name")
        return str(name) if name else None

    def _on_listbox_select(self, _event=None) -> None:
        """Sync dropdown when user clicks listbox."""
        self._update_room_detail()
        name = None
        selection = self.rooms_listbox.curselection()
        if selection:
            rooms = self._load_rooms()
            index = selection[0]
            if index < len(rooms):
                name = rooms[index].get("name")
        if name:
            self.nav_target_var.set(str(name))

    def _on_nav_target_selected(self, _event=None) -> None:
        """User picked from dropdown."""
        self._log(f"[nav] 下拉框选择目标：{self.nav_target_var.get()}")

    def _update_room_detail(self) -> None:
        rooms = self._load_rooms()
        selection = self.rooms_listbox.curselection()
        if not selection or selection[0] >= len(rooms):
            self.room_detail_var.set(f"区域列表：共 {len(rooms)} 个")
            return
        room = rooms[selection[0]]
        self.room_detail_var.set(
            f"{room.get('name', '(unnamed)')} | "
            f"x={float(room.get('x', 0.0)):.2f}, "
            f"y={float(room.get('y', 0.0)):.2f}, "
            f"source={room.get('source', 'manual')}"
        )

    def _all_rooms_named(self) -> bool:
        rooms = self._load_rooms()
        if not rooms:
            return False
        auto_prefixes = (
            "西北房间",
            "北侧房间",
            "东北房间",
            "西侧房间",
            "中央区域",
            "东侧房间",
            "西南房间",
            "南侧房间",
            "东南房间",
        )
        for room in rooms:
            name = str(room.get("name", ""))
            if not name or name.startswith(auto_prefixes):
                return False
        return True

    def _queue_exploration_status(self, text: str) -> None:
        self.root.after(0, lambda: self._handle_exploration_status(text))

    def _queue_exploration_progress(self, progress: float) -> None:
        self.root.after(0, lambda: self._handle_exploration_progress(progress))

    def _queue_exploration_coverage(self, coverage: float) -> None:
        self.root.after(0, lambda: self._handle_exploration_coverage(coverage))

    def _queue_room_status(self, text: str) -> None:
        self.root.after(0, lambda: self._handle_room_status(text))

    def _handle_exploration_status(self, text: str) -> None:
        self._log(f"[explore] {text}")
        # Only exact "exploration_complete" means true completion
        if text == "exploration_complete":
            self._set_state(AppState.EXPLORATION_DONE)
            self._set_detail_status("探索完成，机器人已停止。请点击「保存地图」。")
            return

        # Timeout / no forced target → failure, not completion
        if text.startswith("exploration_failed") or text.startswith("exploration_incomplete"):
            self._set_state(AppState.EXPLORING_FAILED)
            self._set_detail_status(
                "探索未完成：仍有未知区域或达到超时。"
                "请继续探索或手动保存当前地图。"
            )
            return

        if self.state == AppState.EXPLORING:
            self.detail_status_var.set(f"探索状态：{text}")

    def _handle_exploration_progress(self, progress: float) -> None:
        pass  # deprecated metric; use /exploration_coverage instead

    def _handle_exploration_coverage(self, coverage: float) -> None:
        percent = max(0.0, min(100.0, coverage * 100.0))
        self.progress_var.set(percent)
        self.progress_text_var.set(f"覆盖率：{percent:.0f}%")
        if self.state == AppState.EXPLORING:
            self._set_detail_status(f"探索中… 覆盖率 {percent:.0f}%")

    def _handle_room_status(self, text: str) -> None:
        self._log(f"[room] {text}")

        # Auto-detect results → refresh
        if text.startswith("Auto-detected"):
            self.root.after(300, self._refresh_rooms)
            return

        # Navigation failure → revert state
        if (
            "navigate_to_pose action server is not available" in text
            or text.startswith("Navigation failed")
            or "rejected" in text
            or "aborted" in text
        ):
            rooms = self._load_rooms()
            self._set_state(AppState.NAMED if rooms else AppState.MAP_SAVED)
            self._set_detail_status(f"导航失败：{text}")
            return

        # Navigation accepted → now we can say we're navigating
        if "Navigation goal accepted" in text:
            self._set_state(AppState.NAVIGATING)
            self._set_detail_status(text)
            return

        # Navigation finished → back to named
        if text.startswith("Navigation finished"):
            rooms = self._load_rooms()
            self._set_state(AppState.NAMED if rooms else AppState.MAP_SAVED)
            self._set_detail_status(text)
            return

        # Handle "Saved room" / "Set room" → refresh
        if text.startswith("Saved room") or text.startswith("Set room"):
            self.root.after(300, self._refresh_rooms)

    def _stop_all_clicked(self) -> None:
        self._set_detail_status("正在停止全部进程。")
        self._stop_managed_processes()
        self._run_short_command(
            "cleanup_gazebo",
            ["bash", str(SCRIPTS_DIR / "cleanup_gazebo.sh")],
            on_done=lambda _code: self._set_state(AppState.IDLE),
        )

    def _on_close(self) -> None:
        self._stop_managed_processes()
        self.executor.shutdown()
        self.status_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    if tk is None or not os.environ.get("DISPLAY"):
        print("This control center requires a desktop session with DISPLAY.")
        raise SystemExit(1)
    try:
        app = ControlCenterApp()
    except tk.TclError as exc:
        print(f"Could not open the control center window: {exc}")
        print("Run this on the Ubuntu desktop where Gazebo/RViz can open.")
        raise SystemExit(1) from exc
    app.run()


if __name__ == "__main__":
    main()
