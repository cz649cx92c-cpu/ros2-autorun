#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import os
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import autorun.backend as core  # noqa: E402

PROJECT_ROOT = Path(__file__).resolve().parent
BASE_CATKIN_WS = Path("/root/catkin_ws")
ODIN_ROOT = BASE_CATKIN_WS / "src" / "odin_ros_driver"
VENDOR_ROOT = PROJECT_ROOT / "vendor"
SHADOW_ODIN_ROOT = VENDOR_ROOT / "odin_ros_driver"
SHADOW_ODIN_CONFIG = SHADOW_ODIN_ROOT / "config" / "control_command.yaml"
SHADOW_ODIN_LAUNCH = SHADOW_ODIN_ROOT / "launch_ROS1" / "odin1_ros1.launch"
MISSIONS_DIR = PROJECT_ROOT / "missions"
LOG_DIR = PROJECT_ROOT / "logs"
ROS_LOG_DIR = LOG_DIR / "ros"
RUNTIME_DIR = PROJECT_ROOT / "runtime"
TEMP_CONFIG_DIR = RUNTIME_DIR / "configs"
SHADOW_CATKIN_WS = RUNTIME_DIR / "shadow_catkin_ws"
SHADOW_CATKIN_SRC = SHADOW_CATKIN_WS / "src"
SHADOW_CATKIN_SETUP = SHADOW_CATKIN_WS / "devel" / "setup.bash"
SHADOW_HOST_SDK_BIN = SHADOW_CATKIN_WS / "devel" / "lib" / "odin_ros_driver" / "host_sdk_sample"
LINERUN_ROOT = ROOT / "linerun"
LINERUN_PYTHON = Path("/opt/miniconda3/envs/py310/bin/python")

for path in (MISSIONS_DIR, LOG_DIR, ROS_LOG_DIR, RUNTIME_DIR, TEMP_CONFIG_DIR, VENDOR_ROOT, SHADOW_CATKIN_SRC):
    path.mkdir(parents=True, exist_ok=True)

core.PROJECT_ROOT = PROJECT_ROOT
core.MISSIONS_DIR = MISSIONS_DIR
core.LOG_DIR = LOG_DIR


def install_signal_handlers() -> None:
    def _handler(signum, frame):
        del signum, frame
        core.STOP_REQUESTED = True
        core.log("Stop requested. Finishing the current stage safely...")

    signal.signal(signal.SIGTERM, _handler)
    signal.signal(signal.SIGINT, _handler)


def _replace_yaml_scalar(text: str, key: str, value: str) -> str:
    pattern = rf"(^\s*{re.escape(key)}:\s*).*$"
    repl = rf"\g<1>{value}"
    new_text, count = re.subn(pattern, repl, text, flags=re.MULTILINE)
    if count == 0:
        raise RuntimeError(f"Key not found in Odin config: {key}")
    return new_text


def sync_shadow_package() -> None:
    if not SHADOW_ODIN_ROOT.exists():
        shutil.copytree(
            ODIN_ROOT,
            SHADOW_ODIN_ROOT,
            ignore=shutil.ignore_patterns(
                ".git",
                "__pycache__",
                "build",
                "devel",
                "log",
                "recorddata",
                "map",
                "*.pyc",
                "*.o",
            ),
        )
        core.log(f"Shadow odin_ros_driver initialized from {ODIN_ROOT}")
    launch_text = SHADOW_ODIN_LAUNCH.read_text(encoding="utf-8")
    launch_text, count = re.subn(
        r"\s*<!-- Launch RViz with configuration -->\s*<node name=\"rviz\" pkg=\"rviz\" type=\"rviz\" args=\"-d \$\(arg rviz_config\)\" output=\"screen\"/>\s*",
        "\n",
        launch_text,
        flags=re.MULTILINE,
    )
    if count:
        SHADOW_ODIN_LAUNCH.write_text(launch_text, encoding="utf-8")
        core.log("RViz launch node removed from the shadow odin_ros_driver launch file.")
    _ensure_shadow_workspace_built()
    core.log(f"Shadow odin_ros_driver prepared: {SHADOW_ODIN_ROOT}")


def _shadow_package_mtime() -> float:
    latest = 0.0
    for path in SHADOW_ODIN_ROOT.rglob("*"):
        if path.is_file():
            try:
                latest = max(latest, path.stat().st_mtime)
            except OSError:
                continue
    return latest


def _ensure_shadow_workspace_built() -> None:
    package_link = SHADOW_CATKIN_SRC / "odin_ros_driver"
    if package_link.is_symlink():
        if package_link.resolve() != SHADOW_ODIN_ROOT.resolve():
            package_link.unlink()
    elif package_link.exists():
        if package_link.is_dir():
            shutil.rmtree(package_link)
        else:
            package_link.unlink()
    if not package_link.exists():
        package_link.symlink_to(SHADOW_ODIN_ROOT, target_is_directory=True)

    source_mtime = _shadow_package_mtime()
    binary_mtime = SHADOW_HOST_SDK_BIN.stat().st_mtime if SHADOW_HOST_SDK_BIN.exists() else 0.0
    if SHADOW_HOST_SDK_BIN.exists() and binary_mtime >= source_mtime:
        return

    build_log = LOG_DIR / f"shadow_build_{time.strftime('%Y%m%d_%H%M%S')}.log"
    core.log("Building patched shadow odin_ros_driver workspace...")
    cmd = [
        "/bin/bash",
        "-lc",
        (
            f"source /opt/ros/noetic/setup.bash && "
            f"cd {SHADOW_CATKIN_WS} && "
            "catkin_make --pkg odin_ros_driver -DPYTHON_EXECUTABLE=/usr/bin/python3"
        ),
    ]
    result = subprocess.run(
        cmd,
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    build_log.write_text(result.stdout or "", encoding="utf-8")
    if result.returncode != 0 or not SHADOW_HOST_SDK_BIN.exists():
        raise RuntimeError(f"Failed to build patched shadow odin_ros_driver. See {build_log}")
    core.log(f"Patched shadow odin_ros_driver built successfully. Build log: {build_log}")


def write_odin_config(mode: int, *, map_path: Path | None = None, map_name: str = "") -> Path:
    sync_shadow_package()
    text = SHADOW_ODIN_CONFIG.read_text(encoding="utf-8")
    text = _replace_yaml_scalar(text, "custom_map_mode", str(mode))
    text = _replace_yaml_scalar(text, "use_host_ros_time", "2")
    text = _replace_yaml_scalar(text, "sendimu", "1")
    text = _replace_yaml_scalar(text, "enable_imu_smooth", "1")
    text = _replace_yaml_scalar(text, "imu_smooth_frequency", "400")
    text = _replace_yaml_scalar(text, "showpath", "1")
    text = _replace_yaml_scalar(text, "showcamerapose", "1")
    if mode == 1:
        map_dir = PROJECT_ROOT / "maps" / map_name
        map_dir.mkdir(parents=True, exist_ok=True)
        text = _replace_yaml_scalar(text, "relocalization_map_abs_path", '""')
        text = _replace_yaml_scalar(text, "mapping_result_dest_dir", f'"{map_dir}"')
        text = _replace_yaml_scalar(text, "mapping_result_file_name", f'"{map_name}.bin"')
        text = _replace_yaml_scalar(text, "resetalgo", "1")
    elif mode == 2:
        if map_path is None:
            raise RuntimeError("Relocalization mode requires a map path.")
        text = _replace_yaml_scalar(text, "relocalization_map_abs_path", f'"{map_path}"')
        text = _replace_yaml_scalar(text, "mapping_result_dest_dir", '""')
        text = _replace_yaml_scalar(text, "mapping_result_file_name", '""')
        text = _replace_yaml_scalar(text, "resetalgo", "0")
    temp_path = TEMP_CONFIG_DIR / f"odin_mode_{mode}_{int(time.time() * 1000)}.yaml"
    temp_path.write_text(text, encoding="utf-8")
    return temp_path


def request_map_save(target_file: Path, timeout_sec: float = 25.0, raw_log: Path | None = None) -> bool:
    command_file = Path("/tmp/odin_command.txt")

    def _send_save_command() -> None:
        command_file.write_text("set save_map 1\n", encoding="utf-8")

    _send_save_command()
    core.log("Sent Odin save_map=1 command.")
    deadline = time.monotonic() + timeout_sec
    last_size = -1
    stable_hits = 0
    last_send_at = time.monotonic()
    command_ack = False
    transfer_started = False
    last_status_log_at = 0.0
    resend_interval_sec = 4.0
    post_ack_resend_grace_sec = 6.0
    while time.monotonic() < deadline:
        now = time.monotonic()
        if raw_log is not None and raw_log.exists():
            try:
                log_text = raw_log.read_text(encoding="utf-8", errors="ignore")
            except Exception:
                log_text = ""
            if "Successfully set save_map = 1" in log_text:
                command_ack = True
            if "Map is saved on device" in log_text or "map get start success" in log_text:
                transfer_started = True
        if not command_ack and not command_file.exists():
            command_ack = True
            core.log("Odin consumed the save_map command file.")
        if not command_ack and (now - last_send_at) >= 3.0:
            _send_save_command()
            last_send_at = now
            core.log("Resent Odin save_map=1 command.")
        elif command_ack and not transfer_started and (now - last_send_at) >= post_ack_resend_grace_sec:
            _send_save_command()
            last_send_at = now
            core.log("Save command was acknowledged but map export has not started yet. Resending save_map=1.")
        if target_file.exists():
            size = target_file.stat().st_size
            if size > 0 and size == last_size:
                stable_hits += 1
            else:
                stable_hits = 0
            last_size = size
            if size > 0 and stable_hits >= 3:
                core.log(f"Map file saved: {target_file} ({size} bytes)")
                return True
            if transfer_started and size > 0 and stable_hits >= 1:
                core.log(f"Map file transfer started and file is present: {target_file} ({size} bytes)")
                return True
        if not transfer_started and (now - last_status_log_at) >= resend_interval_sec:
            status = "acknowledged" if command_ack else "pending"
            core.log(f"Waiting for Odin map export to start ({status}); current target: {target_file}")
            last_status_log_at = now
        time.sleep(1.0)
    return target_file.exists() and target_file.stat().st_size > 0


def cleanup_stale_odin_processes() -> None:
    patterns = [
        "host_sdk_sample",
        "odin1_ros1.launch",
        "pcd2depth_node",
        "cloud_reprojection_node",
        "image_overlay_node",
        "rviz",
        "rosmaster",
        "roscore",
    ]
    found: list[str] = []
    for pattern in patterns:
        result = subprocess.run(
            ["pgrep", "-af", pattern],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
        )
        lines = [
            line.strip()
            for line in result.stdout.splitlines()
            if line.strip() and "pgrep -af" not in line and "autorun_final/backend.py" not in line
        ]
        if not lines:
            continue
        found.append(pattern)
        core.log(f"Cleaning stale process pattern '{pattern}': {len(lines)} match(es)")
        subprocess.run(["pkill", "-f", pattern], check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if found:
        time.sleep(2.0)
        core.log("Stale Odin/ROS processes were cleaned before startup.")
    else:
        core.log("No stale Odin/ROS processes were found before startup.")


class OdinConfigOverride:
    def __init__(self, config_path: Path) -> None:
        self.config_path = config_path
        self.target_config = SHADOW_ODIN_CONFIG

    def apply(self) -> None:
        if not self.target_config.exists():
            raise RuntimeError(f"Shadow Odin config file does not exist: {self.target_config}")
        if not os.access(self.target_config, os.W_OK):
            raise RuntimeError(f"Shadow Odin config file is not writable: {self.target_config}")
        shutil.copy2(self.config_path, self.target_config)
        core.log(f"Odin config override applied: {self.config_path}")

    def restore(self) -> None:
        return


class OdinLaunchProcess(core.ManagedProcess):
    def __init__(self, config_path: Path, log_path: Path) -> None:
        del config_path
        setup_bash = SHADOW_CATKIN_SETUP if SHADOW_CATKIN_SETUP.exists() else BASE_CATKIN_WS / "devel" / "setup.bash"
        cmd = [
            "/bin/bash",
            "-lc",
            (
                f"source /opt/ros/noetic/setup.bash && "
                f"source {setup_bash} && "
                f"export ROS_PACKAGE_PATH={VENDOR_ROOT}:$ROS_PACKAGE_PATH && "
                f"export ROS_LOG_DIR={ROS_LOG_DIR} && "
                f"exec roslaunch {SHADOW_ODIN_LAUNCH}"
            ),
        ]
        super().__init__(cmd, PROJECT_ROOT, log_path)


class LocalizationSession:
    def __init__(self, map_path: Path, *, viz: str = "off") -> None:
        del viz
        self.map_path = map_path
        self.raw_log = LOG_DIR / f"localization_{time.strftime('%Y%m%d_%H%M%S')}.log"
        self.config_path = write_odin_config(2, map_path=map_path)
        self.override = OdinConfigOverride(self.config_path)
        self.proc = OdinLaunchProcess(self.config_path, self.raw_log)

    def start(self) -> None:
        cleanup_stale_odin_processes()
        self.override.apply()
        self.proc.start()
        core.log(f"Localization process started. Raw log: {self.raw_log}")

    def poll(self) -> int | None:
        return self.proc.poll()

    def stop(self) -> None:
        self.proc.stop()
        self.override.restore()
        self.config_path.unlink(missing_ok=True)


core.LocalizationSession = LocalizationSession


def cmd_map(args: argparse.Namespace) -> int:
    map_name = args.map_name or core.generated_name("map")
    log_path = LOG_DIR / f"mapping_{time.strftime('%Y%m%d_%H%M%S')}.log"
    config_path = write_odin_config(1, map_name=map_name)
    override = OdinConfigOverride(config_path)
    proc = OdinLaunchProcess(config_path, log_path)
    target_file = PROJECT_ROOT / "maps" / map_name / f"{map_name}.bin"
    core.log(f"Starting Odin SLAM session: {map_name}")
    core.log(f"Target map file: {target_file}")
    cleanup_stale_odin_processes()
    override.apply()
    proc.start()
    core.log(f"Mapping raw log: {log_path}")
    try:
        while not core.STOP_REQUESTED:
            code = proc.poll()
            if code is not None:
                return code
            time.sleep(0.2)
        core.log("Stop requested. Saving the current map before shutdown...")
        saved = request_map_save(target_file, timeout_sec=60.0, raw_log=log_path)
        if saved:
            core.log("Map save completed successfully.")
        else:
            core.log("Map save did not complete before timeout. Check USB link speed and Odin logs.")
        return 0
    finally:
        proc.stop()
        override.restore()
        config_path.unlink(missing_ok=True)


@dataclass
class LineStatus:
    state: str = "INIT"
    found: bool = False
    obstacle_blocked: bool = False
    lost_frames: int = 0
    reverse: bool = False
    drive_enable: bool = False
    fresh: bool = False
    updated_at: float = 0.0
    payload: dict[str, Any] | None = None


@dataclass
class TwistCommand:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    updated_at: float = 0.0
    fresh: bool = False


class LineRunRosBridge:
    def __init__(self, cmd_vel_topic: str, status_topic: str, drive_mode_topic: str) -> None:
        self.rospy = core.ensure_ros_node("autorun_final_bridge")
        from geometry_msgs.msg import Twist  # type: ignore
        from std_msgs.msg import String  # type: ignore

        self._string_type = String
        self._status_lock = threading.Lock()
        self._cmd_lock = threading.Lock()
        self._status = LineStatus()
        self._cmd = TwistCommand()
        self._mode_pub = self.rospy.Publisher(drive_mode_topic, String, queue_size=1)
        self.rospy.Subscriber(status_topic, String, self._on_status, queue_size=1)
        self.rospy.Subscriber(cmd_vel_topic, Twist, self._on_cmd_vel, queue_size=1)

    def _on_status(self, msg) -> None:
        now = time.monotonic()
        try:
            payload = json.loads(msg.data)
        except Exception:
            payload = {}
        state = str(payload.get("state", "UNKNOWN"))
        found = state == "TRACK" or str(payload.get("found", "")).lower() == "true"
        blocked = str(payload.get("obstacle_blocked", "False")).lower() == "true" or state == "BLOCKED"
        lost_frames = int(float(payload.get("lost_frames", 0) or 0))
        reverse = str(payload.get("reverse", "False")).lower() == "true"
        drive_enable = str(payload.get("drive_enable", "False")).lower() == "true"
        with self._status_lock:
            self._status = LineStatus(
                state=state,
                found=found,
                obstacle_blocked=blocked,
                lost_frames=lost_frames,
                reverse=reverse,
                drive_enable=drive_enable,
                fresh=True,
                updated_at=now,
                payload=payload if isinstance(payload, dict) else {},
            )

    def _on_cmd_vel(self, msg) -> None:
        with self._cmd_lock:
            self._cmd = TwistCommand(
                vx=float(msg.linear.x),
                vy=float(msg.linear.y),
                wz=float(msg.angular.z),
                updated_at=time.monotonic(),
                fresh=True,
            )

    def publish_mode(
        self,
        *,
        enable: bool,
        reverse: bool,
        cruise_vx: float,
        gear: str,
        low_beam: bool,
        target_center_offset_px: float = 0.0,
        vehicle_direction_angle_deg: float = 0.0,
    ) -> None:
        payload = {
            "enable": bool(enable),
            "reverse": bool(reverse),
            "cruise_vx": abs(float(cruise_vx)),
            "gear": str(gear),
            "low_beam": bool(low_beam),
            "target_center_offset_px": float(target_center_offset_px),
            "vehicle_direction_angle_deg": float(vehicle_direction_angle_deg),
        }
        self._mode_pub.publish(self._string_type(data=json.dumps(payload, ensure_ascii=True)))

    def status_snapshot(self) -> LineStatus:
        with self._status_lock:
            status = self._status
        age = time.monotonic() - status.updated_at if status.updated_at > 0.0 else 1e9
        if age > 1.0:
            status = LineStatus(
                state=status.state,
                found=status.found,
                obstacle_blocked=status.obstacle_blocked,
                lost_frames=status.lost_frames,
                reverse=status.reverse,
                drive_enable=status.drive_enable,
                fresh=False,
                updated_at=status.updated_at,
                payload=status.payload,
            )
        return status

    def cmd_snapshot(self) -> TwistCommand:
        with self._cmd_lock:
            cmd = self._cmd
        age = time.monotonic() - cmd.updated_at if cmd.updated_at > 0.0 else 1e9
        if age > 0.5:
            return TwistCommand(vx=cmd.vx, vy=cmd.vy, wz=cmd.wz, updated_at=cmd.updated_at, fresh=False)
        return cmd


class LineRunProcess(core.ManagedProcess):
    def __init__(self, args: argparse.Namespace, log_path: Path) -> None:
        line_camera_fps = int(round(float(args.line_camera_fps)))
        line_python = str(LINERUN_PYTHON if LINERUN_PYTHON.exists() else Path(sys.executable))
        cmd = [
            line_python,
            str(LINERUN_ROOT / "plant_row_runner.py"),
            "--model",
            args.line_model,
            "--source",
            args.line_source,
            "--classes",
            str(args.line_classes),
            "--target-class",
            str(args.line_target_class),
            "--camera-width",
            str(args.line_camera_width),
            "--camera-height",
            str(args.line_camera_height),
            "--camera-fps",
            str(line_camera_fps),
            "--camera-fourcc",
            args.line_camera_fourcc,
            "--max-fps",
            str(args.line_max_fps),
            "--cruise-vx",
            str(args.line_cruise_vx),
            "--target-center-offset-px",
            str(args.line_target_center_offset_px),
            "--vehicle-direction-angle-deg",
            str(args.line_vehicle_direction_angle_deg),
            "--steer-sign",
            str(args.line_steer_sign),
            "--kp-offset",
            str(args.line_kp_offset),
            "--kp-heading",
            str(args.line_kp_heading),
            "--max-wz",
            str(args.line_max_wz),
            "--reverse-wz-sign",
            "1.0",
            "--reverse-turn-scale",
            "1.0",
            "--reverse-kp-offset",
            str(args.line_kp_offset),
            "--reverse-kp-heading",
            str(args.line_kp_heading),
            "--reverse-max-wz",
            str(args.line_max_wz),
            "--reverse-control-lookahead-ratio",
            "0.78",
            "--reverse-crossing-reset-norm",
            "0.16",
            "--reverse-side-avoid-gain",
            "0.0",
            "--reverse-near-gain",
            "0.0",
            "--reverse-heading-turn-gain",
            "0.0",
            "--period-ms",
            str(args.line_period_ms),
            "--interface",
            args.interface,
            "--channel",
            args.channel,
            "--bitrate",
            str(args.bitrate),
            "--gear",
            "4t4d",
            "--lost-stop-frames",
            str(args.line_lost_stop_frames),
            "--ros-control",
            "--ros-publish",
            "--ros-cmd-vel-topic",
            args.ros_cmd_vel_topic,
            "--ros-status-topic",
            args.ros_status_topic,
            "--ros-drive-mode-topic",
            args.ros_drive_mode_topic,
            "--ros-image-topic",
            args.ros_image_topic,
            "--ros-preview-width",
            str(args.ros_preview_width),
            "--ros-preview-height",
            str(args.ros_preview_height),
            "--ros-preview-jpeg-quality",
            str(args.ros_preview_jpeg_quality),
            "--no-control",
        ]
        if args.line_require_npu:
            cmd.append("--require-npu")
        if args.line_low_beam:
            cmd.append("--low-beam")
        if args.line_no_latest_frame_reader:
            cmd.append("--no-latest-frame-reader")
        if args.line_flip != "none":
            cmd.extend(["--flip", args.line_flip])
        if args.line_npu_cores:
            cmd.extend(["--npu-cores", args.line_npu_cores])
        for extra in args.line_extra_arg:
            cmd.extend(["--debug"] if extra == "__DEBUG__" else [extra])
        super().__init__(cmd, LINERUN_ROOT, log_path)


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except Exception:
        return float(default)


def _linear_blend_amount(value: float, start: float, end: float) -> float:
    if end <= start:
        return 1.0 if value <= end else 0.0
    if value <= start:
        return 1.0
    if value >= end:
        return 0.0
    return (end - value) / (end - start)


def _find_next_maneuver_index(
    motions: list[dict[str, Any]],
    start_index: int,
    *,
    limit: int = 48,
) -> int | None:
    end = min(len(motions), start_index + max(1, limit))
    for idx in range(max(0, start_index), end):
        motion = motions[idx]
        gear = core._resolve_replay_gear(motion, None)
        vx = _safe_float(motion.get("vx"), 0.0)
        if vx < -0.03 or gear == "crab" or abs(_safe_float(motion.get("vy"), 0.0)) > 0.05:
            return idx
    return None


def _find_next_reverse_index(
    motions: list[dict[str, Any]],
    start_index: int,
    *,
    limit: int = 48,
) -> int | None:
    end = min(len(motions), start_index + max(1, limit))
    for idx in range(max(0, start_index), end):
        if _safe_float(motions[idx].get("vx"), 0.0) < -0.03:
            return idx
    return None


def _find_recent_reverse_index(
    motions: list[dict[str, Any]],
    start_index: int,
    *,
    limit: int = 24,
) -> int | None:
    begin = max(0, start_index - max(1, limit))
    for idx in range(start_index - 1, begin - 1, -1):
        if _safe_float(motions[idx].get("vx"), 0.0) < -0.03:
            return idx
    return None


def _find_reverse_segment_start(
    motions: list[dict[str, Any]],
    index: int,
) -> int | None:
    if index < 0 or index >= len(motions):
        return None
    if _safe_float(motions[index].get("vx"), 0.0) >= -0.03:
        return None
    start = index
    while start > 0 and _safe_float(motions[start - 1].get("vx"), 0.0) < -0.03:
        start -= 1
    return start


def _compute_hybrid_weights(
    *,
    args: argparse.Namespace,
    row_switch_mode: bool,
    reversing_mode: bool,
    near_finish: bool,
    local_status: LineStatus,
    local_cmd: TwistCommand,
    dist_to_row_switch: float | None,
    dist_to_reverse_start: float | None,
    dist_since_reverse_start: float | None,
) -> tuple[float, float, str]:
    configured_local_weight = max(0.0, min(1.0, float(args.local_weight_in_row)))
    configured_global_weight = max(0.0, min(1.0, float(args.global_weight_in_row)))
    local_enabled_by_weight = configured_local_weight > 1e-6

    if row_switch_mode:
        return 1.0, 0.0, "global-row-switch"
    if not local_enabled_by_weight:
        return 1.0, 0.0, "global-only"
    if local_status.obstacle_blocked:
        if near_finish or row_switch_mode:
            return 1.0, 0.0, "global-near-finish-blocked"
        return 0.0, 1.0, "local-block-stop"
    if not (local_status.fresh and local_status.state == "TRACK" and local_cmd.fresh):
        return 1.0, 0.0, "global-fallback"

    total = configured_global_weight + configured_local_weight
    if total <= 1e-6:
        base_global = 1.0
        base_local = 0.0
    else:
        base_global = configured_global_weight / total
        base_local = configured_local_weight / total

    if reversing_mode:
        if dist_since_reverse_start is None:
            return base_global, base_local, "blend-reverse-local"
        full_global_dist = max(0.05, float(getattr(args, "row_switch_full_global_dist", 1.50)))
        blend_start_dist = max(full_global_dist, float(getattr(args, "row_switch_blend_start_dist", 3.0)))
        if dist_since_reverse_start <= full_global_dist:
            anticipation = 1.0
        else:
            anticipation = _linear_blend_amount(dist_since_reverse_start, full_global_dist, blend_start_dist)
        if anticipation <= 1e-6:
            return base_global, base_local, "blend-reverse-local"
        global_weight = base_global + (1.0 - base_global) * anticipation
        local_weight = base_local * (1.0 - anticipation)
        total = global_weight + local_weight
        if total <= 1e-6:
            return 1.0, 0.0, "global-reverse-entry"
        return global_weight / total, local_weight / total, "blend-reverse-exit"

    if dist_to_row_switch is None and dist_to_reverse_start is None:
        return base_global, base_local, "blend-row"

    full_global_dist = max(0.05, float(getattr(args, "row_switch_full_global_dist", 1.50)))
    blend_start_dist = max(full_global_dist, float(getattr(args, "row_switch_blend_start_dist", 3.0)))
    anticipation = 0.0
    if dist_to_row_switch is not None:
        anticipation = max(anticipation, _linear_blend_amount(dist_to_row_switch, full_global_dist, blend_start_dist))
    if dist_to_reverse_start is not None:
        anticipation = max(anticipation, _linear_blend_amount(dist_to_reverse_start, full_global_dist, blend_start_dist))
    if anticipation <= 1e-6:
        return base_global, base_local, "blend-row"

    global_weight = base_global + (1.0 - base_global) * anticipation
    local_weight = base_local * (1.0 - anticipation)
    total = global_weight + local_weight
    if total <= 1e-6:
        return 1.0, 0.0, "global-anticipation"
    return global_weight / total, local_weight / total, "blend-approach-maneuver"


def _compute_global_command(
    pose: core.Pose2D,
    target: core.Pose2D,
    motion: dict[str, Any],
    gear: str,
    send_state: core.MotionSendState,
    tracking_heading: float,
) -> tuple[float, float, float, float, float]:
    dist = pose.distance_to(target)
    forward_err, lateral_err = core._body_frame_error(pose, target)
    heading_err = core.normalize_angle(tracking_heading - pose.yaw)
    heading_err_deg, lateral_err = core._smooth_tracking_errors(
        send_state,
        math.degrees(heading_err),
        lateral_err,
        alpha=0.24,
    )
    if abs(heading_err_deg) < 2.0:
        heading_err_deg = 0.0
    if abs(lateral_err) < 0.02:
        lateral_err = 0.0
    vx_cap = max(abs(_safe_float(motion.get("vx"), 0.0)), 0.22)
    wz_cap = max(abs(_safe_float(motion.get("wz"), 0.0)), 20.0)
    cmd_vx = core._clamp(forward_err * 0.75, -vx_cap, vx_cap)
    cmd_vy = 0.0
    if gear == "crab":
        vy_cap = max(abs(_safe_float(motion.get("vy"), 0.0)), 0.20)
        cmd_vx = 0.0
        cmd_vy = core._clamp(lateral_err * 0.95, -vy_cap, vy_cap)
        if abs(lateral_err) < 0.03:
            cmd_vy = 0.0
        if abs(heading_err_deg) > 18.0:
            cmd_vy = core._clamp(cmd_vy, -0.08, 0.08)
        cmd_wz = core._clamp(heading_err_deg * 0.50, -6.0, 6.0)
    else:
        cmd_vx = core._clamp(cmd_vx, -0.16, 0.16)
        if abs(lateral_err) > 0.05:
            cmd_vx = core._clamp(cmd_vx, -0.10, 0.10)
        if abs(lateral_err) > 0.08:
            cmd_vx = core._clamp(cmd_vx, -0.07, 0.07)
        if abs(lateral_err) > 0.11:
            cmd_vx = core._clamp(cmd_vx, -0.05, 0.05)
        cmd_wz = core._clamp(
            heading_err_deg * 0.72 + lateral_err * 18.0,
            -min(wz_cap, 12.0),
            min(wz_cap, 12.0),
        )
        if abs(heading_err_deg) > 45.0:
            cmd_vx = core._clamp(cmd_vx, -0.04, 0.04)
        if abs(heading_err_deg) > 65.0:
            cmd_vx = core._signed_crawl(forward_err, 0.025)
        if gear == "4t4d":
            cmd_wz = core._limit_4t4d_turn_rate(cmd_vx, cmd_wz)
    return cmd_vx, cmd_vy, cmd_wz, dist, heading_err_deg


def _build_body_command(
    gear: str,
    global_cmd: tuple[float, float, float],
    local_cmd: TwistCommand,
    local_status: LineStatus,
    global_weight: float,
    local_weight: float,
) -> core.BodyCommand:
    g_vx, g_vy, g_wz = global_cmd
    if gear == "crab":
        return core.BodyCommand(gear=gear, vx=g_vx, vy=g_vy, wz=g_wz)
    if local_weight <= 1e-6 or not local_cmd.fresh:
        return core.BodyCommand(gear=gear, vx=g_vx, vy=0.0, wz=g_wz)
    if local_status.obstacle_blocked:
        return core.BodyCommand(gear=gear, vx=0.0, vy=0.0, wz=0.0)
    vx = local_weight * local_cmd.vx + global_weight * g_vx
    wz = local_weight * local_cmd.wz + global_weight * g_wz
    return core.BodyCommand(gear=gear, vx=vx, vy=0.0, wz=wz)


def _motion_segment_mode(motion: dict[str, Any], current_gear: str | None = None) -> str:
    gear = core._resolve_replay_gear(motion, current_gear)
    if gear == "crab":
        return "crab"
    vx = _safe_float(motion.get("vx"), 0.0)
    if vx < -0.03:
        return "reverse"
    if vx > 0.03:
        return "forward"
    return f"hold:{gear}"


def _find_tracking_index_same_mode(
    points: list[core.Pose2D],
    motions: list[dict[str, Any]],
    current_index: int,
    pose: core.Pose2D,
    current_gear: str | None,
    window: int = 12,
) -> int:
    if not points:
        return 0
    start = max(0, current_index)
    end = min(len(points), current_index + max(2, window))
    mode = _motion_segment_mode(motions[min(start, len(motions) - 1)], current_gear)
    best_same_index = current_index
    best_same_dist = float("inf")
    best_any_index = current_index
    best_any_dist = float("inf")
    for idx in range(start, end):
        dist = pose.distance_to(points[idx])
        if dist < best_any_dist:
            best_any_dist = dist
            best_any_index = idx
        if _motion_segment_mode(motions[min(idx, len(motions) - 1)], current_gear) != mode:
            continue
        if dist < best_same_dist:
            best_same_dist = dist
            best_same_index = idx
    if best_same_dist < float("inf"):
        return best_same_index
    return best_any_index


def cmd_hybrid_autorun(args: argparse.Namespace) -> int:
    core.ensure_can_ready(args.channel, args.bitrate)
    mission = json.loads(Path(args.mission).read_text(encoding="utf-8"))
    mission_projection = mission.get("ground_projection", {})
    projection = core.GroundProjection(
        enabled=bool(mission_projection.get("enabled", False)),
        sensor_height_m=float(mission_projection.get("sensor_height_m", getattr(args, "sensor_height_m", 0.0)) or 0.0),
        body_x_offset_m=float(mission_projection.get("body_x_offset_m", getattr(args, "body_x_offset_m", 0.0)) or 0.0),
        body_y_offset_m=float(mission_projection.get("body_y_offset_m", getattr(args, "body_y_offset_m", 0.0)) or 0.0),
        roll_gain=float(mission_projection.get("roll_gain", getattr(args, "roll_gain", 0.65)) or 0.65),
        pitch_gain=float(mission_projection.get("pitch_gain", getattr(args, "pitch_gain", 1.0)) or 1.0),
        anchor_roll_rad=float(mission_projection.get("anchor_roll_rad", 0.0) or 0.0),
        anchor_pitch_rad=float(mission_projection.get("anchor_pitch_rad", 0.0) or 0.0),
    )
    if not projection.enabled:
        projection = core.projection_from_args(args)
    bound_map_db = str(mission.get("bound_map_db") or "").strip()
    selected_map_db = str(Path(args.db).resolve())
    if bound_map_db and Path(bound_map_db).resolve() != Path(selected_map_db):
        raise RuntimeError(
            "Mission map mismatch. "
            f"This mission was recorded on: {Path(bound_map_db).name}, "
            f"but the selected replay map is: {Path(selected_map_db).name}."
        )
    samples = mission.get("samples", [])
    if len(samples) < 2:
        raise RuntimeError("Mission has too few samples.")

    controller = core.FWMiniController(args.interface, args.channel, args.bitrate)
    can_reader = core.CANFeedbackReader(args.interface, args.channel, args.bitrate)
    points = [core._sample_pose(sample) for sample in samples]
    sample_period = float(mission.get("sample_period", 0.2) or 0.2)
    motions, used_fallback = core._repair_missing_motion(samples, sample_period)
    crab_fix_count = core._stabilize_crab_segments(motions)
    soften_count = core._soften_low_speed_turns(motions)
    start_index = core._choose_start_index(points, motions)
    current_gear: str | None = None
    send_state = core.MotionSendState.create()
    session: LocalizationSession | None = None
    tracker: core.TFPoseTracker | None = None
    linerun_proc: LineRunProcess | None = None
    ros_bridge: LineRunRosBridge | None = None
    line_log = LOG_DIR / f"linerun_{time.strftime('%Y%m%d_%H%M%S')}.log"
    use_local_guidance = max(0.0, float(args.local_weight_in_row)) > 1e-6
    try:
        if getattr(args, "reuse_localization", False):
            core.log("Hybrid autorun requested. Reusing the active localization session.")
            tracker = core.create_tracker_with_retry(
                args.map_frame,
                args.base_frame,
                "autorun_final_follow_reuse",
                args.localization_wait_sec,
                "Hybrid autorun",
            )
            pose_raw = core.wait_for_stable_pose(tracker, None, args.localization_wait_sec, "Hybrid autorun")
        else:
            core.log("Hybrid autorun requested. Waiting for localization to succeed first.")
            session, tracker, pose_raw = core.localize_with_retry(
                Path(args.db),
                args.map_frame,
                args.base_frame,
                "autorun_final_follow",
                args.localization_wait_sec,
                "Hybrid autorun",
            )
        if projection.enabled and abs(projection.anchor_roll_rad) < 1e-9 and abs(projection.anchor_pitch_rad) < 1e-9:
            projection = core.anchor_projection_to_pose(projection, pose_raw)
        pose = core.project_pose_to_ground(pose_raw, projection)
        if use_local_guidance:
            linerun_proc = LineRunProcess(args, line_log)
            linerun_proc.start()
            core.log(f"linerun subprocess started. Raw log: {line_log}")
            ros_bridge = LineRunRosBridge(args.ros_cmd_vel_topic, args.ros_status_topic, args.ros_drive_mode_topic)
            time.sleep(1.0)
        else:
            core.log("Hybrid autorun local weight is 0. linerun is disabled; running pure global replay.")
        if used_fallback:
            core.log("Hybrid autorun fallback: mission motion data was missing, using path-derived motion estimates.")
        if crab_fix_count:
            core.log(f"Hybrid autorun cleanup: stabilized {crab_fix_count} crab transition samples from the mission.")
        if soften_count:
            core.log(f"Hybrid autorun cleanup: softened {soften_count} low-speed turns from the mission.")
        core._approach_start_point(tracker, controller, can_reader, points, start_index, send_state)
        core.log(f"Hybrid autorun starts replay from mission sample #{start_index}.")
        last_cmd_log = 0.0
        last_feedback_log = 0.0
        current_mode = "global"
        current_gear = None
        while not core.STOP_REQUESTED and start_index < len(points):
            assert tracker is not None
            pose_raw = tracker.lookup()
            if pose_raw is None:
                time.sleep(0.05)
                continue
            pose = core.project_pose_to_ground(pose_raw, projection)
            can_reader.poll(timeout=0.0, limit=20)
            snapshot = can_reader.snapshot()
            io_state = snapshot.get("io", {})
            if bool(io_state.get("remote_control", False)):
                if not send_state.remote_paused:
                    send_state.remote_paused = True
                    send_state.reset_motion()
                    core.log("Remote controller took over. Hybrid autorun paused and progress is preserved.")
                if ros_bridge is not None:
                    ros_bridge.publish_mode(
                        enable=False,
                        reverse=False,
                        cruise_vx=args.line_cruise_vx,
                        gear="4t4d",
                        low_beam=args.line_low_beam,
                        target_center_offset_px=args.line_target_center_offset_px,
                        vehicle_direction_angle_deg=args.line_vehicle_direction_angle_deg,
                    )
                time.sleep(0.10)
                continue
            if send_state.remote_paused:
                send_state.remote_paused = False
                send_state.reset_motion()
                nearest_resume = core._find_tracking_index(points, start_index, pose, window=24)
                start_index = max(start_index, nearest_resume)
                current_gear = None
                core.log(f"Remote controller released. Hybrid autorun resumed from nearest mission sample #{start_index}.")

            nearest_index = core._find_tracking_index(points, start_index, pose, window=14)
            start_index = max(start_index, nearest_index)
            motion_here = motions[min(start_index, len(motions) - 1)]
            reversing_here = _safe_float(motion_here.get("vx"), 0.0) < -0.03
            if send_state.crab_locked_until >= start_index and send_state.crab_target_index >= 0:
                gear = "crab"
                target_index = core._select_crab_progress_target(
                    start_index,
                    send_state.crab_locked_until,
                    max(start_index + 1, send_state.crab_target_index),
                )
                target = points[target_index]
                motion = motions[target_index]
                if core._resolve_replay_gear(motion, None) != "crab":
                    send_state.crab_locked_until = -1
                    send_state.crab_target_index = -1
                    gear = core._resolve_replay_gear(motion_here, current_gear)
                    base_lookahead = 2 if gear == "crab" else (
                        3 if reversing_here else (4 if abs(_safe_float(motion_here.get("wz"), 0.0)) < 10.0 else 3)
                    )
                    target_index = min(len(points) - 1, start_index + base_lookahead)
                    target = points[target_index]
                    motion = motions[target_index]
            else:
                base_lookahead = 2 if core._resolve_replay_gear(motion_here, current_gear) == "crab" else (
                    3 if reversing_here else (4 if abs(_safe_float(motion_here.get("wz"), 0.0)) < 10.0 else 3)
                )
                target_index = min(len(points) - 1, start_index + base_lookahead)
                target = points[target_index]
                motion = motions[target_index]
                gear = core._resolve_replay_gear(motion, current_gear)
            if gear != "crab":
                upcoming_crab = core._find_future_gear_start(motions, start_index, "crab", limit=20)
                if upcoming_crab is not None:
                    crab_entry = points[upcoming_crab]
                    if pose.distance_to(crab_entry) <= 0.35:
                        crab_end = core._find_gear_segment_end(motions, upcoming_crab, "crab")
                        crab_active = core._find_first_active_crab_index(motions, upcoming_crab, crab_end)
                        crab_active_end = core._find_last_active_crab_index(motions, crab_active, crab_end)
                        send_state.crab_locked_until = crab_active_end
                        send_state.crab_target_index = crab_active
                        target_index = core._select_crab_progress_target(
                            start_index,
                            crab_active_end,
                            crab_active,
                        )
                        target = points[target_index]
                        motion = motions[target_index]
                        gear = "crab"
            row_switch_mode = gear == "crab" or abs(_safe_float(motion.get("vy"), 0.0)) > 0.05
            reversing_mode = gear == "reverse" or _safe_float(motion.get("vx"), 0.0) < -0.03
            upcoming_row_switch_index = None if row_switch_mode else _find_next_maneuver_index(motions, start_index + 1)
            dist_to_row_switch = None
            if upcoming_row_switch_index is not None:
                dist_to_row_switch = pose.distance_to(points[upcoming_row_switch_index])
            next_reverse_index = None if reversing_mode else _find_next_reverse_index(motions, start_index + 1)
            dist_to_reverse_start = None
            if next_reverse_index is not None:
                reverse_start_index = _find_reverse_segment_start(motions, next_reverse_index)
                if reverse_start_index is not None:
                    dist_to_reverse_start = pose.distance_to(points[reverse_start_index])
            dist_since_reverse_start = None
            if reversing_mode:
                reverse_start_index = _find_reverse_segment_start(motions, start_index)
                if reverse_start_index is not None:
                    dist_since_reverse_start = pose.distance_to(points[reverse_start_index])
            near_finish = target_index >= len(points) - 4

            if gear != current_gear:
                core.log(f"Hybrid autorun mode switched to {gear}.")
                current_gear = gear

            if gear == "crab":
                if dist + 0.03 < send_state.crab_best_dist:
                    send_state.crab_best_dist = dist
                    send_state.crab_diverge_count = 0
                elif dist > send_state.crab_best_dist + 0.20:
                    send_state.crab_diverge_count += 1
                else:
                    send_state.crab_diverge_count = max(0, send_state.crab_diverge_count - 1)
                if send_state.crab_diverge_count >= 6:
                    core.log(
                        f"Hybrid autorun crab fallback: target distance kept increasing "
                        f"(best={send_state.crab_best_dist:.2f}, now={dist:.2f}). Returning to 4t4d tracking."
                    )
                    send_state.crab_locked_until = -1
                    send_state.crab_target_index = -1
                    send_state.crab_best_dist = float("inf")
                    send_state.crab_diverge_count = 0
                    current_gear = None
                    send_state.last_sent_gear = None
                    continue
                ref_point = points[start_index]
                axis_dx = target.x - ref_point.x
                axis_dy = target.y - ref_point.y
                dist = pose.distance_to(target)
                if abs(axis_dy) >= abs(axis_dx):
                    if core._axis_progress_reached(pose.y, target.y, axis_dy, 0.10):
                        start_index = min(len(points) - 1, target_index + 1)
                        if start_index > send_state.crab_locked_until:
                            send_state.crab_locked_until = -1
                            send_state.crab_target_index = -1
                            send_state.crab_best_dist = float("inf")
                            send_state.crab_diverge_count = 0
                        continue
                else:
                    if core._axis_progress_reached(pose.x, target.x, axis_dx, 0.10):
                        start_index = min(len(points) - 1, target_index + 1)
                        if start_index > send_state.crab_locked_until:
                            send_state.crab_locked_until = -1
                            send_state.crab_target_index = -1
                            send_state.crab_best_dist = float("inf")
                            send_state.crab_diverge_count = 0
                        continue
                if near_finish and dist < 0.10:
                    core._hold_current_gear_stop(controller, send_state, current_gear)
                    core.log(f"Hybrid autorun reached final area near mission end (dist={dist:.2f}). Stopping without final alignment.")
                    break
            else:
                dist = pose.distance_to(target)
                tracking_heading = core._tracking_heading(points, motions, target_index, gear)
                yaw_err = core.normalize_angle(tracking_heading - pose.yaw)
                if dist < 0.12 and abs(math.degrees(yaw_err)) < 12.0:
                    send_state.crab_best_dist = float("inf")
                    send_state.crab_diverge_count = 0
                    start_index = min(len(points) - 1, target_index + 1)
                    continue
                _, lateral_err = core._body_frame_error(pose, target)
                if abs(lateral_err) > 0.07:
                    protected_target_index = min(len(points) - 1, start_index + 2)
                    if protected_target_index != target_index:
                        target_index = protected_target_index
                        target = points[target_index]
                        motion = motions[target_index]
                        near_finish = target_index >= len(points) - 4
                        tracking_heading = core._tracking_heading(points, motions, target_index, gear)
                        yaw_err = core.normalize_angle(tracking_heading - pose.yaw)
                        dist = pose.distance_to(target)

            g_vx, g_vy, g_wz, dist, heading_err_deg = _compute_global_command(
                pose,
                target,
                motion,
                gear,
                send_state,
                tracking_heading,
            )
            if near_finish and dist < 0.10:
                core._hold_current_gear_stop(controller, send_state, current_gear)
                core.log(f"Hybrid autorun reached final area near mission end (dist={dist:.2f}). Stopping without final alignment.")
                break

            local_enabled_by_weight = max(0.0, min(1.0, float(args.local_weight_in_row))) > 1e-6
            local_enable = (not row_switch_mode) and local_enabled_by_weight
            if ros_bridge is not None:
                ros_bridge.publish_mode(
                    enable=local_enable,
                    reverse=reversing_here,
                    cruise_vx=args.line_cruise_vx,
                    gear="4t4d",
                    low_beam=args.line_low_beam,
                    target_center_offset_px=args.line_target_center_offset_px,
                    vehicle_direction_angle_deg=args.line_vehicle_direction_angle_deg,
                )
                local_status = ros_bridge.status_snapshot()
                local_cmd = ros_bridge.cmd_snapshot()
            else:
                local_status = LineStatus()
                local_cmd = TwistCommand()

            global_weight, local_weight, mode_name = _compute_hybrid_weights(
                args=args,
                row_switch_mode=row_switch_mode,
                reversing_mode=reversing_mode,
                near_finish=near_finish,
                local_status=local_status,
                local_cmd=local_cmd,
                dist_to_row_switch=dist_to_row_switch,
                dist_to_reverse_start=dist_to_reverse_start,
                dist_since_reverse_start=dist_since_reverse_start,
            )

            body_cmd = _build_body_command(
                gear,
                (g_vx, g_vy, g_wz),
                local_cmd,
                local_status,
                global_weight,
                local_weight,
            )
            body_cmd.gear = gear
            if (
                gear == "crab"
                and dist < 0.16
                and abs(body_cmd.vy) < 0.10
                and target_index >= max(start_index + 1, send_state.crab_locked_until - 1)
            ):
                start_index = min(len(points) - 1, target_index + 1)
                continue
            body_cmd.vx, body_cmd.vy, body_cmd.wz = core._smooth_drive_command(
                send_state,
                gear,
                body_cmd.vx,
                body_cmd.vy,
                body_cmd.wz,
            )
            if gear == "crab":
                send_state.last_vx = 0.0
                body_cmd.vx = 0.0
            waiting_unlock, unlock_now, sent_vx, sent_vy, sent_wz = core._send_drive(
                controller,
                send_state,
                snapshot,
                gear,
                body_cmd.vx,
                body_cmd.vy,
                body_cmd.wz,
            )
            now = time.monotonic()
            if mode_name != current_mode:
                current_mode = mode_name
                core.log(
                    f"Hybrid control switched to {mode_name}: "
                    f"local_state={local_status.state} row_switch={row_switch_mode} "
                    f"gw={global_weight:.2f} lw={local_weight:.2f} "
                    f"switch_dist={(dist_to_row_switch if dist_to_row_switch is not None else -1.0):.2f}"
                )
            if now - last_cmd_log >= 1.0:
                core.log(
                    f"Hybrid command: mode={mode_name} gear={gear} "
                    f"global_vx={g_vx:.2f} global_vy={g_vy:.2f} global_wz={g_wz:.1f} "
                    f"local_vx={local_cmd.vx:.2f} local_wz={local_cmd.wz:.2f} "
                    f"sent_vx={sent_vx:.2f} sent_vy={sent_vy:.2f} sent_wz={sent_wz:.1f} "
                    f"nearest_index={nearest_index} target_index={target_index} dist={dist:.2f} "
                    f"switch_dist={(dist_to_row_switch if dist_to_row_switch is not None else -1.0):.2f}"
                )
                if waiting_unlock or unlock_now:
                    core.log(f"Hybrid unlock: waiting_unlock={waiting_unlock} unlock_pulse={unlock_now}")
                last_cmd_log = now
            if now - last_feedback_log >= 1.0:
                core._log_feedback("Hybrid autorun", snapshot, pose)
                core.log(
                    f"Hybrid local: state={local_status.state} found={local_status.found} "
                    f"blocked={local_status.obstacle_blocked} lost_frames={local_status.lost_frames} "
                    f"fresh={local_status.fresh} heading_err_deg={heading_err_deg:.1f}"
                )
                last_feedback_log = now
            time.sleep(max(0.03, min(0.10, sample_period)))

        core._hold_current_gear_stop(controller, send_state, current_gear)
        core.log("Hybrid autorun finished.")
        return 0
    finally:
        try:
            if ros_bridge is not None:
                ros_bridge.publish_mode(
                    enable=False,
                    reverse=False,
                    cruise_vx=args.line_cruise_vx,
                    gear="4t4d",
                    low_beam=args.line_low_beam,
                    target_center_offset_px=args.line_target_center_offset_px,
                    vehicle_direction_angle_deg=args.line_vehicle_direction_angle_deg,
                )
        except Exception:
            pass
        try:
            core._hold_current_gear_stop(controller, send_state, current_gear)
        except Exception:
            pass
        can_reader.close()
        controller.close()
        if linerun_proc is not None:
            linerun_proc.stop()
        if session is not None:
            session.stop()


def _add_hybrid_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--db", required=True)
    parser.add_argument("--mission", required=True)
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--base-frame", default="odin1_base_link")
    parser.add_argument("--localization-wait-sec", type=float, default=30.0)
    parser.add_argument("--reuse-localization", action="store_true")
    parser.add_argument("--line-model", required=True)
    parser.add_argument("--line-source", default="/dev/video0")
    parser.add_argument("--line-classes", type=int, default=1)
    parser.add_argument("--line-target-class", type=int, default=0)
    parser.add_argument("--line-cruise-vx", type=float, default=0.12)
    parser.add_argument("--line-max-fps", type=float, default=9.0)
    parser.add_argument("--line-camera-width", type=int, default=1024)
    parser.add_argument("--line-camera-height", type=int, default=768)
    parser.add_argument("--line-camera-fps", type=float, default=10.0)
    parser.add_argument("--line-camera-fourcc", default="MJPG")
    parser.add_argument("--line-target-center-offset-px", type=float, default=0.0)
    parser.add_argument("--line-vehicle-direction-angle-deg", type=float, default=0.0)
    parser.add_argument("--line-steer-sign", type=float, default=-1.0)
    parser.add_argument("--line-kp-offset", type=float, default=7.0)
    parser.add_argument("--line-kp-heading", type=float, default=0.08)
    parser.add_argument("--line-max-wz", type=float, default=1.6)
    parser.add_argument("--line-period-ms", type=int, default=20)
    parser.add_argument("--line-lost-stop-frames", type=int, default=12)
    parser.add_argument("--line-require-npu", action="store_true")
    parser.add_argument("--line-low-beam", action="store_true")
    parser.add_argument("--line-no-latest-frame-reader", action="store_true")
    parser.add_argument("--line-flip", choices=["none", "horizontal", "vertical", "both"], default="none")
    parser.add_argument("--line-npu-cores", default="0_1_2")
    parser.add_argument("--line-extra-arg", action="append", default=[], help="Pass-through extra arg for plant_row_runner")
    parser.add_argument("--ros-cmd-vel-topic", default="/linerun/cmd_vel")
    parser.add_argument("--ros-status-topic", default="/linerun/status")
    parser.add_argument("--ros-drive-mode-topic", default="/linerun/drive_mode")
    parser.add_argument("--ros-image-topic", default="/linerun/preview/compressed")
    parser.add_argument("--ros-preview-width", type=int, default=640)
    parser.add_argument("--ros-preview-height", type=int, default=360)
    parser.add_argument("--ros-preview-jpeg-quality", type=int, default=35)
    parser.add_argument("--local-weight-in-row", type=float, default=0.75)
    parser.add_argument("--global-weight-in-row", type=float, default=0.25)
    parser.add_argument("--row-switch-blend-start-dist", type=float, default=3.0)
    parser.add_argument("--row-switch-full-global-dist", type=float, default=1.5)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="UGV autorun_final: Odin global localization + linerun local row guidance"
    )
    parser.add_argument("--interface", default="socketcan")
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--bitrate", type=int, default=500000)
    parser.add_argument("--sensor-height-m", type=float, default=1.2)
    parser.add_argument("--body-x-offset-m", type=float, default=0.0)
    parser.add_argument("--body-y-offset-m", type=float, default=0.0)
    parser.add_argument("--roll-gain", type=float, default=0.65)
    parser.add_argument("--pitch-gain", type=float, default=1.0)
    sub = parser.add_subparsers(dest="command", required=True)

    map_p = sub.add_parser("map", help="Start Odin SLAM mapping and save a .bin map")
    map_p.add_argument("--map-name", default="")
    map_p.add_argument("--viz", choices=["on", "off"], default="off")
    map_p.set_defaults(func=cmd_map)

    loc_p = sub.add_parser("localization", help="Run Odin relocalization only")
    loc_p.add_argument("--db", required=True, help="Path to the Odin .bin map file")
    loc_p.add_argument("--map-frame", default="map")
    loc_p.add_argument("--base-frame", default="odin1_base_link")
    loc_p.add_argument("--localization-wait-sec", type=float, default=30.0)
    loc_p.set_defaults(func=core.cmd_localization)

    rec_p = sub.add_parser("record", help="Relocalize first, then record a taught path")
    rec_p.add_argument("--db", required=True, help="Path to the Odin .bin map file")
    rec_p.add_argument("--mission-name", default="")
    rec_p.add_argument("--map-frame", default="map")
    rec_p.add_argument("--base-frame", default="odin1_base_link")
    rec_p.add_argument("--sample-period", type=float, default=0.2)
    rec_p.add_argument("--localization-wait-sec", type=float, default=30.0)
    rec_p.add_argument("--reuse-localization", action="store_true")
    rec_p.set_defaults(func=core.cmd_record)

    run_p = sub.add_parser("autorun", help="Hybrid replay: global mission + linerun row following")
    _add_hybrid_args(run_p)
    run_p.set_defaults(func=cmd_hybrid_autorun)

    replay_p = sub.add_parser("replay", help="Alias of autorun")
    _add_hybrid_args(replay_p)
    replay_p.set_defaults(func=cmd_hybrid_autorun)

    return parser


def main() -> int:
    install_signal_handlers()
    args = build_parser().parse_args()
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
