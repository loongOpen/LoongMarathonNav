#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS 机器人控制面板 — 统一管理传感器、定位、导航和路径跟踪模块
用法: python3 robot_control_panel.py  (自动 source devel/setup.bash，无需手动执行)
"""

import collections
import html as html_mod
import os
import pty
import re
import signal
import socket
import struct
import subprocess
import sys
import threading
import time
import math

from PyQt5.QtCore import QEvent, Qt, QTimer
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QApplication,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QPushButton,
    QSizePolicy,
    QSplitter,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

WORKSPACE = os.path.dirname(os.path.abspath(__file__))
CATKIN_WS = os.path.dirname(WORKSPACE)

ANSI_ESCAPE = re.compile(
    r"\x1b\[[0-9;]*[a-zA-Z]"
    r"|\x1b\][^\x07]*(?:\x07|\x1b\\)"
)


def _build_ros_env():
    """source devel/setup.bash 并返回完整的环境变量字典，供子进程继承。"""
    setup = os.path.join(CATKIN_WS, "devel", "setup.bash")
    if not os.path.isfile(setup):
        return None
    try:
        out = subprocess.check_output(
            ["bash", "-c", f"source {setup} && env -0"],
            stderr=subprocess.DEVNULL, timeout=10,
        )
        env = {}
        for entry in out.decode("utf-8", errors="replace").split("\0"):
            if "=" in entry:
                k, v = entry.split("=", 1)
                env[k] = v
        return env
    except Exception:
        return None


ROS_ENV = _build_ros_env() or os.environ.copy()


def _reap_dead_popen(proc, timeout=3.0):
    """子进程已退出时必须 wait，否则在 Linux 上会残留僵尸进程。"""
    if proc is None:
        return
    if proc.poll() is None:
        return
    try:
        proc.wait(timeout=timeout)
    except Exception:
        pass


# ---------------------------------------------------------------------------
# 进程管理
# ---------------------------------------------------------------------------
class ModuleManager:
    def __init__(self, name, commands, node_names, log_queue=None):
        self.name = name
        self.commands = commands
        self.node_names = node_names
        self.log_queue = log_queue
        self.processes: list[subprocess.Popen] = []
        self._stop_lock = threading.Lock()

    @property
    def is_running(self):
        return any(p.poll() is None for p in self.processes)

    def start(self):
        if self.is_running:
            return False
        self.processes.clear()
        for cmd in self.commands:
            master_fd, slave_fd = pty.openpty()
            proc = subprocess.Popen(
                cmd, preexec_fn=os.setsid, env=ROS_ENV,
                stdout=slave_fd, stderr=slave_fd, stdin=subprocess.DEVNULL,
            )
            os.close(slave_fd)
            self.processes.append(proc)
            if self.log_queue is not None:
                threading.Thread(
                    target=self._read_output, args=(master_fd,), daemon=True
                ).start()
            else:
                os.close(master_fd)
        return True

    def _read_output(self, fd):
        buf = b""
        try:
            while True:
                try:
                    data = os.read(fd, 4096)
                except OSError:
                    break
                if not data:
                    break
                buf += data
                while b"\n" in buf:
                    line_bytes, buf = buf.split(b"\n", 1)
                    text = ANSI_ESCAPE.sub("", line_bytes.decode("utf-8", errors="replace")).strip()
                    if text and self.log_queue is not None:
                        self.log_queue.append(f"[{self.name}] {text}")
            if buf:
                text = ANSI_ESCAPE.sub("", buf.decode("utf-8", errors="replace")).strip()
                if text and self.log_queue is not None:
                    self.log_queue.append(f"[{self.name}] {text}")
        except Exception:
            pass
        finally:
            try:
                os.close(fd)
            except Exception:
                pass

    def stop(self):
        with self._stop_lock:
            if not self.processes:
                return
            for proc in self.processes:
                if proc.poll() is None:
                    try:
                        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                    except (ProcessLookupError, PermissionError):
                        pass
            deadline = time.time() + 5
            for proc in self.processes:
                try:
                    proc.wait(timeout=max(0.1, deadline - time.time()))
                except subprocess.TimeoutExpired:
                    pass
            for proc in self.processes:
                if proc.poll() is None:
                    try:
                        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                    except (ProcessLookupError, PermissionError):
                        pass
                    try:
                        proc.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        pass
            for node_name in self.node_names:
                try:
                    subprocess.Popen(["rosnode", "kill", node_name], env=ROS_ENV,
                                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                except FileNotFoundError:
                    pass
            time.sleep(0.3)
            self.processes.clear()


# ---------------------------------------------------------------------------
# 速度通讯管理
# ---------------------------------------------------------------------------
UDP_PACK_FMT = "<ii4f"
UDP_CHECKER = 1872


class SpeedComm:
    """订阅 /cmd_vel → UDP 发送，支持键盘覆盖，自动记录速度日志"""

    def __init__(self, ros_env):
        self.lock = threading.Lock()
        self._ros_env = ros_env

        # ---- 从 /cmd_vel 接收 ----
        self.ros_vx = 0.0
        self.ros_vy = 0.0
        self.ros_wz = 0.0
        self._last_ros_time = 0.0
        self._sub_proc = None

        # ---- 键盘覆盖 ----
        self.kb_active = False
        self.kb_vx = 0.0
        self.kb_vy = 0.0
        self.kb_wz = 0.0
        self.kb_vx_speed = 2.4
        self.kb_vy_speed = 1.0
        self.kb_wz_speed = 0.8
        self.kb_vx_step = 0.2
        self.kb_vy_step = 0.2
        self.kb_wz_step = 0.2
        self._pressed = set()

        # ---- UDP 发送 ----
        self.udp_ip = "192.168.1.204"
        self.udp_port = 8002
        # 实际发出的 vx 对称限幅于 [-udp_vx_max, +udp_vx_max]（与键盘/cmd_vel 源无关）
        self.udp_vx_max = 2.5
        self.send_hz = 100.0
        self.send_enabled = False
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._send_running = False
        self._send_thread = None

        # ---- 实际发送 ----
        self.sent_vx = 0.0
        self.sent_vy = 0.0
        self.sent_wz = 0.0
        self.send_source = ""

        # ---- 速度日志 ----
        self._log_fp = None
        self._log_path = ""

    # ====================== cmd_vel 订阅 ======================
    def start_sub(self):
        if self._sub_proc and self._sub_proc.poll() is None:
            return
        _reap_dead_popen(self._sub_proc)
        self._sub_proc = subprocess.Popen(
            ["rostopic", "echo", "/cmd_vel", "-p"],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid, env=self._ros_env,
        )
        threading.Thread(target=self._read_loop, daemon=True).start()

    def restart_sub_if_needed(self):
        if self._sub_proc and self._sub_proc.poll() is not None:
            self.start_sub()

    def _read_loop(self):
        proc = self._sub_proc
        col_vx = col_vy = col_wz = None
        try:
            for raw in iter(proc.stdout.readline, b""):
                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue
                if line.startswith("%"):
                    for i, h in enumerate(line.split(",")):
                        if "linear.x" in h:
                            col_vx = i
                        elif "linear.y" in h:
                            col_vy = i
                        elif "angular.z" in h:
                            col_wz = i
                    continue
                if col_vx is None or col_wz is None:
                    continue
                parts = line.split(",")
                needed = max(col_vx, col_wz)
                if col_vy is not None:
                    needed = max(needed, col_vy)
                if len(parts) <= needed:
                    continue
                try:
                    vx = float(parts[col_vx])
                    vy = float(parts[col_vy]) if col_vy is not None else 0.0
                    wz = float(parts[col_wz])
                except ValueError:
                    continue
                with self.lock:
                    self.ros_vx = vx
                    self.ros_vy = vy
                    self.ros_wz = wz
                    self._last_ros_time = time.time()
        except Exception:
            pass
        finally:
            try:
                proc.stdout.close()
            except Exception:
                pass

    # ====================== UDP 发送 ======================
    def start_send(self):
        if self._send_running:
            return
        self.send_enabled = True
        self._send_running = True
        self._send_thread = threading.Thread(target=self._send_loop, daemon=True)
        self._send_thread.start()

    def stop_send(self):
        self._send_running = False
        if self._send_thread and self._send_thread.is_alive():
            self._send_thread.join(timeout=0.2)
        self.send_enabled = False
        try:
            data = struct.pack(UDP_PACK_FMT, UDP_CHECKER, 1, 0.0, 0.0, 0.0, 0.0)
            self._sock.sendto(data, (self.udp_ip, self.udp_port))
        except Exception:
            pass
        with self.lock:
            self.sent_vx = self.sent_vy = self.sent_wz = 0.0
            self.send_source = ""

    def _send_loop(self):
        log_counter = 0
        while self._send_running:
            with self.lock:
                kb_active = self.kb_active
                kb_driving = abs(self.kb_vx) + abs(self.kb_vy) + abs(self.kb_wz) > 1e-9
                ros_fresh = (time.time() - self._last_ros_time) < 0.5

                if kb_active and kb_driving:
                    vx, vy, wz = self.kb_vx, self.kb_vy, self.kb_wz
                    source = "键盘"
                elif ros_fresh:
                    vx, vy, wz = self.ros_vx, self.ros_vy, self.ros_wz
                    source = "cmd_vel"
                else:
                    vx, vy, wz = 0.0, 0.0, 0.0
                    source = "等待"

                lim = self.udp_vx_max
                vx = max(-lim, min(lim, vx))

                ip, port, hz = self.udp_ip, self.udp_port, self.send_hz
                r_vx, r_vy, r_wz = self.ros_vx, self.ros_vy, self.ros_wz

            data = struct.pack(UDP_PACK_FMT, UDP_CHECKER, 1, vx, vy, wz, 0.0)
            try:
                self._sock.sendto(data, (ip, port))
            except Exception:
                pass

            with self.lock:
                self.sent_vx, self.sent_vy, self.sent_wz = vx, vy, wz
                self.send_source = source

            log_counter += 1
            if self._log_fp and log_counter % 10 == 0:
                ts = f"{time.time():.3f}"
                self._log_fp.write(
                    f"{ts},{r_vx:.4f},{r_vy:.4f},{r_wz:.4f},{vx:.4f},{vy:.4f},{wz:.4f}\n"
                )
                if log_counter % 100 == 0:
                    self._log_fp.flush()

            time.sleep(1.0 / max(1.0, hz))

    # ====================== 键盘控制 ======================
    def key_press(self, key):
        if key in ("w", "s", "a", "d", "q", "e"):
            self._pressed.add(key)
            self._update_kb()

    def key_release(self, key):
        self._pressed.discard(key)
        self._update_kb()

    def clear_keys(self):
        self._pressed.clear()
        with self.lock:
            self.kb_vx = self.kb_vy = self.kb_wz = 0.0

    def _update_kb(self):
        vxs, vys, wzs = self.kb_vx_speed, self.kb_vy_speed, self.kb_wz_speed
        vx = (vxs if "w" in self._pressed else 0.0) - (vxs if "s" in self._pressed else 0.0)
        vy = (vys if "a" in self._pressed else 0.0) - (vys if "d" in self._pressed else 0.0)
        wz = (wzs if "q" in self._pressed else 0.0) - (wzs if "e" in self._pressed else 0.0)
        with self.lock:
            self.kb_vx, self.kb_vy, self.kb_wz = vx, vy, wz

    # ====================== 速度日志 ======================
    def start_log(self):
        log_dir = os.path.join(CATKIN_WS, "log")
        os.makedirs(log_dir, exist_ok=True)
        fname = time.strftime("speed_%Y%m%d_%H%M%S.txt")
        self._log_path = os.path.join(log_dir, fname)
        self._log_fp = open(self._log_path, "w")
        self._log_fp.write("timestamp,recv_vx,recv_vy,recv_wz,sent_vx,sent_vy,sent_wz\n")

    def log_tick(self):
        if self._log_fp and not self._send_running:
            with self.lock:
                r_vx, r_vy, r_wz = self.ros_vx, self.ros_vy, self.ros_wz
                s_vx, s_vy, s_wz = self.sent_vx, self.sent_vy, self.sent_wz
            ts = f"{time.time():.3f}"
            self._log_fp.write(
                f"{ts},{r_vx:.4f},{r_vy:.4f},{r_wz:.4f},{s_vx:.4f},{s_vy:.4f},{s_wz:.4f}\n"
            )

    # ====================== 清理 ======================
    def cleanup(self):
        self._send_running = False
        self.send_enabled = False
        if self._send_thread and self._send_thread.is_alive():
            self._send_thread.join(timeout=0.5)
        if self._sub_proc and self._sub_proc.poll() is None:
            try:
                os.killpg(os.getpgid(self._sub_proc.pid), signal.SIGINT)
                self._sub_proc.wait(timeout=3)
            except Exception:
                pass
            if self._sub_proc.poll() is None:
                try:
                    os.killpg(os.getpgid(self._sub_proc.pid), signal.SIGKILL)
                except Exception:
                    pass
                try:
                    self._sub_proc.wait(timeout=2)
                except Exception:
                    pass
        elif self._sub_proc is not None:
            _reap_dead_popen(self._sub_proc)
        self._sub_proc = None
        if self._log_fp:
            try:
                self._log_fp.flush()
                self._log_fp.close()
            except Exception:
                pass
        try:
            self._sock.close()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# 路径进度（/rtk_odom + /preset_path 弧长）
# ---------------------------------------------------------------------------
class OdomDistanceTracker:
    """
    通过 `rostopic echo /rtk_odom -p` 获取实时位置，并以 /preset_path（由 path.yaml 经 rtk_odom 发布）为参考折线。
    - 零点：面板启动后首次有效定位时，将机器人投影到路径上的弧长记为 start_s，显示从该点起的相对里程。
    - 进度：投影弧长相对 start_s 只增不减（抗抖）。
    """

    _RTK_ODOM_TOPIC = "/rtk_odom"
    _PRESET_PATH_TOPIC = "/preset_path"

    def __init__(self, ros_env):
        self._ros_env = ros_env
        self.lock = threading.Lock()
        self._sub_proc = None
        self._topic = self._RTK_ODOM_TOPIC

        self._path_xy = None  # list[(x, y)]
        self._cum_dist = None  # list[float], len==N
        self._path_total_m = 0.0

        self._progress_m = 0.0
        self._last_nearest_i = 0
        self._start_s = None  # 启动时的路径弧长（作为0点）

        # 限定窗口搜索，避免每帧全量扫描
        self._back_window = 50
        self._fwd_window = 800

    @property
    def topic(self):
        return self._topic

    @property
    def total_km(self):
        with self.lock:
            return self._progress_m / 1000.0

    @property
    def path_total_km(self):
        with self.lock:
            return self._path_total_m / 1000.0

    def _fetch_preset_path_once(self):
        """
        使用 rostopic 一次性读取 /preset_path（latched），解析其中的 poses[].pose.position.x/y。
        不依赖 rospy，避免改变现有线程模型。
        """
        try:
            out = subprocess.check_output(
                ["rostopic", "echo", "-n", "1", self._PRESET_PATH_TOPIC],
                stderr=subprocess.DEVNULL,
                timeout=3,
                env=self._ros_env,
            ).decode("utf-8", errors="replace")
        except Exception:
            return False

        pts = []
        cur_x = None
        cur_y = None
        in_position = False
        for raw in out.splitlines():
            line = raw.strip()
            if not line:
                continue
            if line.startswith("position:"):
                in_position = True
                cur_x = None
                cur_y = None
                continue
            if in_position and line.startswith("x:"):
                try:
                    cur_x = float(line.split(":", 1)[1].strip())
                except ValueError:
                    cur_x = None
                continue
            if in_position and line.startswith("y:"):
                try:
                    cur_y = float(line.split(":", 1)[1].strip())
                except ValueError:
                    cur_y = None
                continue
            if in_position and line.startswith("z:"):
                # position 字段结束（x/y已经拿到就收集）
                if cur_x is not None and cur_y is not None:
                    pts.append((cur_x, cur_y))
                in_position = False
                continue

        if len(pts) < 2:
            return False

        cum = [0.0]
        total = 0.0
        last_x, last_y = pts[0]
        for x, y in pts[1:]:
            total += math.hypot(x - last_x, y - last_y)
            cum.append(total)
            last_x, last_y = x, y

        with self.lock:
            self._path_xy = pts
            self._cum_dist = cum
            self._path_total_m = total
            self._progress_m = 0.0
            self._last_nearest_i = 0
            self._start_s = None
        return True

    def start_sub(self):
        if self._sub_proc and self._sub_proc.poll() is None:
            return
        _reap_dead_popen(self._sub_proc)
        # 先尝试加载一次 /preset_path（失败也不阻塞，后续会重试）
        self._fetch_preset_path_once()
        self._sub_proc = subprocess.Popen(
            ["rostopic", "echo", self._RTK_ODOM_TOPIC, "-p"],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid,
            env=self._ros_env,
        )
        threading.Thread(target=self._read_loop, daemon=True).start()

    def restart_sub_if_needed(self):
        if self._sub_proc and self._sub_proc.poll() is not None:
            self.start_sub()

    def _read_loop(self):
        proc = self._sub_proc
        col_x = col_y = None
        try:
            for raw in iter(proc.stdout.readline, b""):
                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue
                if line.startswith("%"):
                    cols = line.split(",")
                    for i, h in enumerate(cols):
                        if h.endswith(".pose.pose.position.x"):
                            col_x = i
                        elif h.endswith(".pose.pose.position.y"):
                            col_y = i
                    continue
                if col_x is None or col_y is None:
                    continue
                parts = line.split(",")
                if len(parts) <= max(col_x, col_y):
                    continue
                try:
                    x = float(parts[col_x])
                    y = float(parts[col_y])
                except ValueError:
                    continue
                with self.lock:
                    pts = self._path_xy
                    cum = self._cum_dist
                    last_i = self._last_nearest_i

                if pts is None or cum is None:
                    # /preset_path 可能尚未发布，周期性重试加载
                    self._fetch_preset_path_once()
                    continue

                n = len(pts)
                s0 = max(0, last_i - self._back_window)
                s1 = min(n, last_i + self._fwd_window)
                best_i = s0
                best_d2 = float("inf")
                for i in range(s0, s1):
                    px, py = pts[i]
                    dx = x - px
                    dy = y - py
                    d2 = dx * dx + dy * dy
                    if d2 < best_d2:
                        best_d2 = d2
                        best_i = i

                # 在 best_i 附近用“线段投影”做一次弧长细化
                seg_i = min(best_i, n - 2)
                ax, ay = pts[seg_i]
                bx, by = pts[seg_i + 1]
                abx = bx - ax
                aby = by - ay
                apx = x - ax
                apy = y - ay
                ab2 = abx * abx + aby * aby
                if ab2 > 1e-12:
                    t = (apx * abx + apy * aby) / ab2
                    t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
                else:
                    t = 0.0
                proj_len = math.hypot(abx, aby) * t
                cur_s = cum[seg_i] + proj_len

                with self.lock:
                    if self._start_s is None:
                        # 首次有效定位点：将其投影弧长作为“启动0点”
                        self._start_s = cur_s
                        self._progress_m = 0.0
                    else:
                        rel_s = cur_s - self._start_s
                        if rel_s < 0.0:
                            rel_s = 0.0
                        # 只增不减：避免抖动/短时回退导致显示下降
                        if rel_s > self._progress_m:
                            self._progress_m = rel_s
                    self._last_nearest_i = best_i
        except Exception:
            pass
        finally:
            try:
                proc.stdout.close()
            except Exception:
                pass

    def cleanup(self):
        if not self._sub_proc:
            return
        if self._sub_proc.poll() is not None:
            _reap_dead_popen(self._sub_proc)
            self._sub_proc = None
            return
        try:
            os.killpg(os.getpgid(self._sub_proc.pid), signal.SIGINT)
            self._sub_proc.wait(timeout=2)
        except Exception:
            pass
        if self._sub_proc.poll() is None:
            try:
                os.killpg(os.getpgid(self._sub_proc.pid), signal.SIGKILL)
            except Exception:
                pass
            try:
                self._sub_proc.wait(timeout=2)
            except Exception:
                pass
        _reap_dead_popen(self._sub_proc)
        self._sub_proc = None


# ---------------------------------------------------------------------------
# 主窗口
# ---------------------------------------------------------------------------
class ControlPanel(QMainWindow):
    MODULE_DEFS = [
        ("sensor", "传感器", "开启传感器", "关闭传感器",
         [["roslaunch", os.path.join(WORKSPACE, "livox_ros_driver2/launch_ROS1/mid360.launch")],
          ["roslaunch", os.path.join(WORKSPACE, "qx_gps_driver/launch/qx_evk_driver.launch")]],
         ["/livox_lidar_publisher2", "/qx_data_sender"]),
        ("localization", "定位", "开启定位", "关闭定位",
         [["roslaunch", os.path.join(WORKSPACE, "FAST_LIO_SAM/launch/map.launch")]],
         ["/laserMapping", "/rtk_odom_node", "/odom_fusion_node", "/rviz"]),
        ("navigation", "导航", "开启导航", "关闭导航",
         [["roslaunch", os.path.join(WORKSPACE, "FAST_LIO_SAM/launch/localplanner.launch")]],
         ["/bodyToSensorPublisher", "/mapTocameraPublisher",
          "/localPlanner", "/pathFollower",
          "/terrainAnalysis", "/terrainAnalysisExt",
          "/sensorScanGeneration", "/loamInterface"]),
        ("tracking", "路径跟踪", "开始跟踪", "停止跟踪",
         [["python3", os.path.join(WORKSPACE, "sub_path_waypoint_follower.py")]],
         ["/path_follower"]),
    ]
    RECORD_TOPICS = ["/livox/imu", "/livox/lidar", "/qx/evk", "/qx/evk/heading"]
    RECORD_DIR = os.path.expanduser("~/rosbag")

    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROS 机器人控制面板")
        self.setMinimumSize(780, 520)
        self.resize(900, 600)

        self.roscore_proc = None
        self.roscore_started_by_us = False
        self.modules: dict[str, ModuleManager] = {}
        self.module_order: list[str] = []
        self.ui: dict[str, dict] = {}
        self._log_queue: collections.deque = collections.deque(maxlen=500)
        self._rosbag_proc = None
        self._record_start_time = None
        self._rtk_proc = None
        self._rtk_lock = threading.Lock()
        self._rtk_data = None
        self._busy_keys: set = set()
        self._stop_done: collections.deque = collections.deque()
        self._rec_busy = False

        self._speed_comm = SpeedComm(ROS_ENV)
        self._odom_tracker = OdomDistanceTracker(ROS_ENV)

        self._init_modules()
        self._build_ui()
        for btn in self.findChildren(QPushButton):
            btn.setFocusPolicy(Qt.NoFocus)
        QApplication.instance().installEventFilter(self)
        self._ensure_roscore()
        self._speed_comm.start_sub()
        self._speed_comm.start_log()
        self._odom_tracker.start_sub()
        self._start_rtk_monitor()
        self._log(
            f"监控已启动 | 速度日志: {self._speed_comm._log_path} | 路径里程: {self._odom_tracker.topic}+{OdomDistanceTracker._PRESET_PATH_TOPIC}"
        )

        self._status_timer = QTimer(self)
        self._status_timer.timeout.connect(self._update_all_status)
        self._status_timer.start(1000)

        self._rtk_timer = QTimer(self)
        self._rtk_timer.timeout.connect(self._refresh_rtk_status)
        self._rtk_timer.start(100)

        self._log_drain_timer = QTimer(self)
        self._log_drain_timer.timeout.connect(self._drain_log_queue)
        self._log_drain_timer.start(200)

        self._speed_timer = QTimer(self)
        self._speed_timer.timeout.connect(self._refresh_speed)
        self._speed_timer.start(100)

    def _init_modules(self):
        for key, name, _s, _e, cmds, nodes in self.MODULE_DEFS:
            lq = None if key == "sensor" else self._log_queue
            self.modules[key] = ModuleManager(name, cmds, nodes, lq)
            self.module_order.append(key)

    # ==================================================================
    #  UI 构建 — 双栏布局
    # ==================================================================
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setSpacing(6)
        root.setContentsMargins(10, 8, 10, 8)

        # ---- 顶部: RTK 状态 (全宽) ----
        rtk_bar = QWidget()
        rtk_bar.setObjectName("rtkBar")
        rtk_h = QHBoxLayout(rtk_bar)
        rtk_h.setContentsMargins(14, 6, 14, 6)
        self._rtk_label = QLabel("● 无数据")
        self._rtk_label.setFont(QFont("", 14, QFont.Bold))
        self._rtk_label.setStyleSheet("color:#95a5a6;")
        rtk_h.addWidget(self._rtk_label)
        rtk_h.addStretch()
        self._rtk_coord_label = QLabel("")
        self._rtk_coord_label.setFont(QFont("Monospace", 11))
        self._rtk_coord_label.setStyleSheet("color:#bdc3c7;")
        rtk_h.addWidget(self._rtk_coord_label)
        root.addWidget(rtk_bar)

        # ---- 主体: 左右分栏 ----
        body = QSplitter(Qt.Horizontal)
        body.setChildrenCollapsible(False)

        # == 左列: 控制 ==
        left_w = QWidget()
        left_lay = QVBoxLayout(left_w)
        left_lay.setContentsMargins(0, 0, 0, 0)
        left_lay.setSpacing(6)

        ctrl_group = QGroupBox("模块控制")
        ctrl_v = QVBoxLayout(ctrl_group)
        ctrl_v.setSpacing(5)
        ctrl_v.setContentsMargins(8, 6, 8, 6)

        for key, _name, start_txt, stop_txt, _c, _n in self.MODULE_DEFS:
            row = QHBoxLayout()
            row.setSpacing(6)
            status = QLabel("● 已停止")
            status.setFont(QFont("", 10, QFont.Bold))
            status.setStyleSheet("color:#e74c3c;")
            status.setMinimumWidth(72)
            row.addWidget(status)
            btn = QPushButton(start_txt)
            btn.setProperty("role", "start")
            btn.setMinimumHeight(30)
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            btn.clicked.connect(lambda _, k=key: self._on_toggle(k))
            row.addWidget(btn)
            ctrl_v.addLayout(row)
            self.ui[key] = {"status": status, "btn": btn,
                            "start_txt": start_txt, "stop_txt": stop_txt}

        left_lay.addWidget(ctrl_group)

        comm_group = QGroupBox("底层通讯")
        comm_v = QVBoxLayout(comm_group)
        comm_v.setContentsMargins(8, 6, 8, 6)
        comm_v.setSpacing(5)

        ip_row = QHBoxLayout()
        ip_row.setSpacing(4)
        ip_row.addWidget(QLabel("IP:"))
        self._ip_edit = QLineEdit(self._speed_comm.udp_ip)
        self._ip_edit.setFixedWidth(120)
        self._ip_edit.setReadOnly(True)
        self._ip_edit.setStyleSheet(
            "background:#1e1e1e; color:#ecf0f1; border:1px solid #555; "
            "border-radius:3px; padding:2px 4px;"
        )
        ip_row.addWidget(self._ip_edit)
        ip_unlock_btn = QPushButton("编辑")
        ip_unlock_btn.setFixedSize(44, 26)
        ip_unlock_btn.setStyleSheet(
            "background-color:#7f8c8d; color:#fff; font-size:11px; "
            "border:none; border-radius:3px;"
        )
        ip_unlock_btn.clicked.connect(self._unlock_ip_edit)
        ip_row.addWidget(ip_unlock_btn)
        ip_apply_btn = QPushButton("应用")
        ip_apply_btn.setFixedSize(44, 26)
        ip_apply_btn.setStyleSheet(
            "background-color:#3498db; color:#fff; font-size:11px; "
            "border:none; border-radius:3px;"
        )
        ip_apply_btn.clicked.connect(self._apply_ip)
        ip_row.addWidget(ip_apply_btn)
        ip_row.addStretch()
        comm_v.addLayout(ip_row)

        btn_row = QHBoxLayout()
        btn_row.setSpacing(6)
        self._comm_btn = QPushButton("开启通讯")
        self._comm_btn.setProperty("role", "start")
        self._comm_btn.setMinimumHeight(28)
        self._comm_btn.clicked.connect(self._toggle_comm)
        btn_row.addWidget(self._comm_btn)
        self._kb_btn = QPushButton("键盘控制")
        self._kb_btn.setMinimumHeight(28)
        self._kb_btn.setStyleSheet(
            "background-color:#7f8c8d; color:#fff; font-size:12px; "
            "font-weight:bold; border:none; border-radius:5px;"
        )
        self._kb_btn.clicked.connect(self._toggle_keyboard)
        btn_row.addWidget(self._kb_btn)
        comm_v.addLayout(btn_row)

        kb_hint = QLabel("WASD移动 QE旋转 空格通讯开关")
        kb_hint.setStyleSheet("color:#7f8c8d; font-size:9px;")
        comm_v.addWidget(kb_hint)

        vxlim_row = QHBoxLayout()
        vxlim_row.setSpacing(6)
        vxlim_row.addWidget(QLabel("UDP Vx 上限:"))
        self._vx_max_minus_btn = QPushButton("−")
        self._vx_max_minus_btn.setFixedSize(40, 28)
        self._vx_max_minus_btn.setStyleSheet(
            "background-color:#e67e22; color:#fff; font-size:16px; font-weight:bold; "
            "border:none; border-radius:5px;"
        )
        self._vx_max_minus_btn.clicked.connect(lambda: self._adjust_udp_vx_max(-0.2))
        vxlim_row.addWidget(self._vx_max_minus_btn)
        self._vx_max_label = QLabel(f"{self._speed_comm.udp_vx_max:.1f}")
        self._vx_max_label.setAlignment(Qt.AlignCenter)
        self._vx_max_label.setFont(QFont("Monospace", 12, QFont.Bold))
        self._vx_max_label.setStyleSheet("color:#3498db;")
        self._vx_max_label.setMinimumWidth(48)
        vxlim_row.addWidget(self._vx_max_label, stretch=1)
        self._vx_max_plus_btn = QPushButton("+")
        self._vx_max_plus_btn.setFixedSize(40, 28)
        self._vx_max_plus_btn.setStyleSheet(
            "background-color:#27ae60; color:#fff; font-size:16px; font-weight:bold; "
            "border:none; border-radius:5px;"
        )
        self._vx_max_plus_btn.clicked.connect(lambda: self._adjust_udp_vx_max(0.2))
        vxlim_row.addWidget(self._vx_max_plus_btn)
        vxlim_step = QLabel("±0.2")
        vxlim_step.setStyleSheet("color:#7f8c8d; font-size:9px;")
        vxlim_row.addWidget(vxlim_step)
        comm_v.addLayout(vxlim_row)

        _small_btn_ss = (
            "background-color:#555; color:#fff; font-size:13px; "
            "font-weight:bold; border:none; border-radius:3px;"
        )
        _val_ss = "color:#3498db; font-size:10px;"
        self._kb_speed_labels = {}
        sc = self._speed_comm
        for axis, label, val, step in (
            ("vx", "Vx", sc.kb_vx_speed, sc.kb_vx_step),
            ("vy", "Vy", sc.kb_vy_speed, sc.kb_vy_step),
            ("wz", "Wz", sc.kb_wz_speed, sc.kb_wz_step),
        ):
            sp_row = QHBoxLayout()
            sp_row.setSpacing(3)
            sp_row.addWidget(QLabel(f"{label}:"))
            minus = QPushButton("−")
            minus.setFixedSize(24, 22)
            minus.setStyleSheet(_small_btn_ss)
            minus.clicked.connect(lambda _, a=axis, s=-step: self._adjust_kb_speed(a, s))
            sp_row.addWidget(minus)
            val_lbl = QLabel(f"{val:.1f}")
            val_lbl.setAlignment(Qt.AlignCenter)
            val_lbl.setFont(QFont("Monospace", 10, QFont.Bold))
            val_lbl.setStyleSheet(_val_ss)
            val_lbl.setMinimumWidth(36)
            sp_row.addWidget(val_lbl)
            plus = QPushButton("+")
            plus.setFixedSize(24, 22)
            plus.setStyleSheet(_small_btn_ss)
            plus.clicked.connect(lambda _, a=axis, s=step: self._adjust_kb_speed(a, s))
            sp_row.addWidget(plus)
            step_lbl = QLabel(f"±{step}")
            step_lbl.setStyleSheet("color:#7f8c8d; font-size:9px;")
            sp_row.addWidget(step_lbl)
            sp_row.addStretch()
            comm_v.addLayout(sp_row)
            self._kb_speed_labels[axis] = val_lbl

        left_lay.addWidget(comm_group)

        left_lay.addStretch()

        left_w.setMinimumWidth(210)
        left_w.setMaximumWidth(320)
        body.addWidget(left_w)

        # == 右列: 速度显示 + 日志 ==
        right_w = QWidget()
        right_lay = QVBoxLayout(right_w)
        right_lay.setContentsMargins(0, 0, 0, 0)
        right_lay.setSpacing(4)

        top_status_rec = QWidget()
        top_h = QHBoxLayout(top_status_rec)
        top_h.setContentsMargins(0, 0, 0, 0)
        top_h.setSpacing(10)

        _status_font = QFont("Monospace", 9)
        status_block = QWidget()
        status_v = QVBoxLayout(status_block)
        status_v.setContentsMargins(0, 0, 0, 0)
        status_v.setSpacing(0)

        self._recv_label = QLabel("接收: Vx +0.000 | Vy +0.000 | Wz +0.000")
        self._recv_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self._recv_label.setFont(_status_font)
        self._recv_label.setStyleSheet("color:#2ecc71; margin:0; padding:0;")
        status_v.addWidget(self._recv_label)

        self._sent_label = QLabel("发送: 未启用")
        self._sent_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self._sent_label.setFont(_status_font)
        self._sent_label.setStyleSheet("color:#7f8c8d; margin:0; padding:0;")
        status_v.addWidget(self._sent_label)

        self._distance_label = QLabel("路径: 0.000 / 0.000 km")
        self._distance_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self._distance_label.setFont(_status_font)
        self._distance_label.setStyleSheet("color:#f1c40f; margin:0; padding:0;")
        status_v.addWidget(self._distance_label)

        rec_group = QGroupBox("数据录制")
        rec_v = QVBoxLayout(rec_group)
        rec_v.setContentsMargins(6, 4, 6, 4)
        rec_row = QHBoxLayout()
        rec_row.setSpacing(4)
        self._rec_status = QLabel("● 未录制")
        self._rec_status.setFont(QFont("", 9, QFont.Bold))
        self._rec_status.setStyleSheet("color:#95a5a6;")
        self._rec_status.setMinimumWidth(56)
        rec_row.addWidget(self._rec_status)
        self._rec_btn = QPushButton("开始录制")
        self._rec_btn.setProperty("role", "start")
        self._rec_btn.setMinimumHeight(26)
        self._rec_btn.setMinimumWidth(88)
        self._rec_btn.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        self._rec_btn.clicked.connect(self._on_record_toggle)
        rec_row.addWidget(self._rec_btn)
        rec_v.addLayout(rec_row)
        rec_group.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Preferred)

        top_h.addWidget(status_block, 1)
        top_h.addWidget(rec_group, 0, Qt.AlignVCenter)
        right_lay.addWidget(top_status_rec)

        log_title = QLabel("日志")
        log_title.setStyleSheet("color:#bdc3c7;")
        right_lay.addWidget(log_title)
        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setFont(QFont("Monospace", 9))
        self.log_box.document().setMaximumBlockCount(1000)
        right_lay.addWidget(self.log_box, stretch=1)

        body.addWidget(right_w)

        body.setStretchFactor(0, 0)
        body.setStretchFactor(1, 1)
        body.setSizes([240, 560])
        root.addWidget(body, stretch=1)

    # ==================================================================
    #  日志
    # ==================================================================
    _MODULE_COLORS = {
        "传感器": "#e67e22", "定位": "#9b59b6",
        "导航": "#1abc9c", "路径跟踪": "#e74c3c",
    }

    def _log(self, msg):
        ts = time.strftime("%H:%M:%S")
        safe = html_mod.escape(msg)
        self.log_box.append(
            f'<span style="color:#666666">{ts}</span> '
            f'<span style="color:#3498db">[系统]</span> '
            f'<span style="color:#ecf0f1">{safe}</span>'
        )

    def _drain_log_queue(self):
        count = 0
        while count < 80:
            try:
                line = self._log_queue.popleft()
            except IndexError:
                break
            count += 1
            ts = time.strftime("%H:%M:%S")
            tag_color, tag_name, content = "#aaaaaa", "", line
            for name, color in self._MODULE_COLORS.items():
                prefix = f"[{name}]"
                if line.startswith(prefix):
                    tag_color, tag_name = color, name
                    content = line[len(prefix):].strip()
                    break
            safe = html_mod.escape(content)
            if tag_name:
                self.log_box.append(
                    f'<span style="color:#666666">{ts}</span> '
                    f'<span style="color:{tag_color}">[{tag_name}]</span> '
                    f'<span style="color:#bdc3c7">{safe}</span>'
                )
            else:
                self.log_box.append(
                    f'<span style="color:#666666">{ts}</span> '
                    f'<span style="color:#bdc3c7">{safe}</span>'
                )
        if count:
            sb = self.log_box.verticalScrollBar()
            sb.setValue(sb.maximum())

    # ==================================================================
    #  roscore
    # ==================================================================
    @staticmethod
    def _roscore_alive():
        try:
            return subprocess.run(
                ["rostopic", "list"], stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL, timeout=3, env=ROS_ENV,
            ).returncode == 0
        except Exception:
            return False

    def _ensure_roscore(self):
        if self._roscore_alive():
            self._log("检测到 roscore 已在运行")
            return
        self._log("roscore 未运行，正在启动 ...")
        self.roscore_proc = subprocess.Popen(
            ["roscore"], preexec_fn=os.setsid, env=ROS_ENV,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        self.roscore_started_by_us = True
        for _ in range(20):
            time.sleep(0.5)
            if self._roscore_alive():
                self._log("roscore 启动完成")
                return
        self._log("roscore 启动超时，请手动检查")

    # ==================================================================
    #  RTK 状态 (100 ms 刷新)
    # ==================================================================
    def _start_rtk_monitor(self):
        if self._rtk_proc and self._rtk_proc.poll() is None:
            return
        _reap_dead_popen(self._rtk_proc)
        self._rtk_proc = subprocess.Popen(
            ["rostopic", "echo", "/qx/evk", "-p"],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid, env=ROS_ENV,
        )
        threading.Thread(target=self._read_rtk_topic, daemon=True).start()

    def _read_rtk_topic(self):
        proc = self._rtk_proc
        col_st = col_cov = col_lat = col_lon = None
        try:
            for raw in iter(proc.stdout.readline, b""):
                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue
                if line.startswith("%"):
                    for i, h in enumerate(line.split(",")):
                        if "status.status" in h and "service" not in h:
                            col_st = i
                        elif h.endswith(".position_covariance0"):
                            col_cov = i
                        elif h.endswith(".latitude"):
                            col_lat = i
                        elif h.endswith(".longitude"):
                            col_lon = i
                    continue
                if col_st is None:
                    continue
                parts = line.split(",")
                try:
                    st = int(parts[col_st])
                    cov = float(parts[col_cov]) if col_cov is not None else -1
                    lat = float(parts[col_lat]) if col_lat is not None else 0
                    lon = float(parts[col_lon]) if col_lon is not None else 0
                except (ValueError, IndexError):
                    continue
                with self._rtk_lock:
                    self._rtk_data = {"status": st, "cov0": cov,
                                      "lat": lat, "lon": lon, "time": time.time()}
        except Exception:
            pass
        finally:
            try:
                proc.stdout.close()
            except Exception:
                pass

    _RTK_STYLES = {
        "no_data": ("● 无数据",     "#95a5a6"),
        "no_fix":  ("● 无定位",     "#95a5a6"),
        "rtk_fix": ("● RTK 固定解", "#2ecc71"),
        "rtk_flt": ("● RTK 浮点解", "#f1c40f"),
        "sbas":    ("● SBAS 差分",  "#e67e22"),
        "ins":     ("● 惯导定位",    "#e67e22"),
        "single":  ("● 单点定位",    "#e74c3c"),
    }

    def _refresh_rtk_status(self):
        if self._rtk_proc and self._rtk_proc.poll() is not None:
            self._start_rtk_monitor()
        with self._rtk_lock:
            data = dict(self._rtk_data) if self._rtk_data else None
        if data is None or time.time() - data["time"] > 3.0:
            key = "no_data"
        else:
            st, cov = data["status"], data["cov0"]
            if st == -1:
                key = "no_fix"
            elif st == 2 and cov < 0.01:
                key = "rtk_fix"
            elif st == 2:
                key = "rtk_flt"
            elif st == 1:
                key = "sbas"
            elif st == 0 and 0 <= cov <= 0.5:
                key = "ins"
            else:
                key = "single"
        text, color = self._RTK_STYLES[key]
        self._rtk_label.setText(text)
        self._rtk_label.setStyleSheet(f"color:{color}; font-weight:bold;")
        if data and time.time() - data["time"] <= 3.0:
            self._rtk_coord_label.setText(f"{data['lat']:.7f}°N  {data['lon']:.7f}°E")
        else:
            self._rtk_coord_label.setText("")

    # ==================================================================
    #  模块 开/关 (切换按钮)
    # ==================================================================
    def _on_toggle(self, key):
        if key in self._busy_keys:
            return
        self._busy_keys.add(key)
        btn = self.ui[key]["btn"]
        btn.setEnabled(False)
        m = self.modules[key]
        if m.is_running:
            btn.setText("正在停止...")
            self._log(f"正在停止 {m.name} ...")
            threading.Thread(
                target=self._stop_module_bg, args=(key,), daemon=True
            ).start()
        else:
            self._log(f"启动 {m.name} ...")
            m.start()
            self._log(f"{m.name} 已启动")
            self._busy_keys.discard(key)
            self._refresh_ui(key)

    def _stop_module_bg(self, key):
        try:
            self.modules[key].stop()
        except Exception:
            pass
        self._stop_done.append(key)

    def _refresh_ui(self, key):
        running = self.modules[key].is_running
        w = self.ui[key]
        w["status"].setText("● 运行中" if running else "● 已停止")
        w["status"].setStyleSheet(f"color:{'#2ecc71' if running else '#e74c3c'};")
        btn = w["btn"]
        btn.setText(w["stop_txt"] if running else w["start_txt"])
        btn.setProperty("role", "stop" if running else "start")
        btn.setEnabled(True)
        btn.style().unpolish(btn)
        btn.style().polish(btn)

    # ==================================================================
    #  录制 (切换按钮)
    # ==================================================================
    def _on_record_toggle(self):
        if self._rec_busy:
            return
        recording = self._rosbag_proc and self._rosbag_proc.poll() is None
        if recording:
            self._rec_busy = True
            self._rec_btn.setEnabled(False)
            self._rec_btn.setText("正在停止...")
            elapsed = int(time.time() - self._record_start_time) if self._record_start_time else 0
            self._log("正在停止录制 ...")
            threading.Thread(
                target=self._stop_record_bg, args=(elapsed,), daemon=True
            ).start()
        else:
            os.makedirs(self.RECORD_DIR, exist_ok=True)
            prefix = os.path.join(self.RECORD_DIR, "record")
            self._rosbag_proc = subprocess.Popen(
                ["rosbag", "record", "-o", prefix] + self.RECORD_TOPICS,
                preexec_fn=os.setsid, env=ROS_ENV,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            )
            self._record_start_time = time.time()
            self._log(f"开始录制 → {self.RECORD_DIR}/")
            self._refresh_record_ui()

    def _stop_record_bg(self, elapsed):
        proc = self._rosbag_proc
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=5)
        except Exception:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                pass
            try:
                proc.wait(timeout=2)
            except Exception:
                pass
        self._stop_done.append(("_record", elapsed))

    def _refresh_record_ui(self):
        recording = self._rosbag_proc is not None and self._rosbag_proc.poll() is None
        if recording and self._record_start_time:
            elapsed = int(time.time() - self._record_start_time)
            mins, secs = divmod(elapsed, 60)
            self._rec_status.setText(f"● {mins:02d}:{secs:02d}")
            self._rec_status.setStyleSheet("color:#e74c3c; font-weight:bold;")
        else:
            self._rec_status.setText("● 未录制")
            self._rec_status.setStyleSheet("color:#95a5a6;")
        self._rec_btn.setText("停止录制" if recording else "开始录制")
        self._rec_btn.setProperty("role", "stop" if recording else "start")
        self._rec_btn.setEnabled(True)
        self._rec_btn.style().unpolish(self._rec_btn)
        self._rec_btn.style().polish(self._rec_btn)

    # ==================================================================
    #  底层通讯控制
    # ==================================================================
    def _unlock_ip_edit(self):
        self._ip_edit.setReadOnly(False)
        self._ip_edit.setFocus()
        self._ip_edit.selectAll()

    def _apply_ip(self):
        ip = self._ip_edit.text().strip()
        if ip:
            with self._speed_comm.lock:
                self._speed_comm.udp_ip = ip
            self._log(f"UDP 目标 IP 已更新: {ip}")
        self._ip_edit.setReadOnly(True)
        self.log_box.setFocus()

    def _toggle_comm(self):
        if self._speed_comm.send_enabled:
            self._speed_comm.stop_send()
            self._comm_btn.setText("开启通讯")
            self._comm_btn.setProperty("role", "start")
            self._comm_btn.style().unpolish(self._comm_btn)
            self._comm_btn.style().polish(self._comm_btn)
            self._log("底层通讯已关闭")
        else:
            ip = self._ip_edit.text().strip()
            with self._speed_comm.lock:
                if ip:
                    self._speed_comm.udp_ip = ip
            self._speed_comm.start_send()
            self._comm_btn.setText("关闭通讯")
            self._comm_btn.setProperty("role", "stop")
            self._comm_btn.style().unpolish(self._comm_btn)
            self._comm_btn.style().polish(self._comm_btn)
            self._log(f"底层通讯已开启 → {self._speed_comm.udp_ip}:{self._speed_comm.udp_port}")

    def _toggle_keyboard(self):
        if self._speed_comm.kb_active:
            self._speed_comm.kb_active = False
            self._speed_comm.clear_keys()
            self._kb_btn.setStyleSheet(
                "background-color:#7f8c8d; color:#fff; font-size:12px; "
                "font-weight:bold; border:none; border-radius:5px;"
            )
            self._kb_btn.setText("键盘控制")
            self._log("键盘控制已关闭")
        else:
            self._speed_comm.kb_active = True
            self._kb_btn.setStyleSheet(
                "background-color:#e74c3c; color:#fff; font-size:12px; "
                "font-weight:bold; border:none; border-radius:5px;"
            )
            self._kb_btn.setText("关闭键盘 (活跃)")
            self._log("键盘控制已开启 — WASD移动, QE旋转, 空格通讯开关")

    def _adjust_udp_vx_max(self, delta):
        sc = self._speed_comm
        with sc.lock:
            sc.udp_vx_max = max(0.0, round(sc.udp_vx_max + delta, 2))
            val = sc.udp_vx_max
        self._vx_max_label.setText(f"{val:.1f}")

    def _adjust_kb_speed(self, axis, delta):
        sc = self._speed_comm
        if axis == "vx":
            sc.kb_vx_speed = max(0.0, round(sc.kb_vx_speed + delta, 2))
            self._kb_speed_labels["vx"].setText(f"{sc.kb_vx_speed:.1f}")
        elif axis == "vy":
            sc.kb_vy_speed = max(0.0, round(sc.kb_vy_speed + delta, 2))
            self._kb_speed_labels["vy"].setText(f"{sc.kb_vy_speed:.1f}")
        elif axis == "wz":
            sc.kb_wz_speed = max(0.0, round(sc.kb_wz_speed + delta, 2))
            self._kb_speed_labels["wz"].setText(f"{sc.kb_wz_speed:.1f}")
        if sc.kb_active:
            sc._update_kb()

    def eventFilter(self, obj, event):
        if event.type() == QEvent.WindowDeactivate and obj is self:
            if self._speed_comm.kb_active and self._speed_comm._pressed:
                self._speed_comm.clear_keys()
            return super().eventFilter(obj, event)

        if not self.isActiveWindow():
            return super().eventFilter(obj, event)

        if event.type() == QEvent.KeyPress:
            if event.key() == Qt.Key_Space and not event.isAutoRepeat():
                self._toggle_comm()
                return True
            if self._speed_comm.kb_active:
                key = event.text().lower()
                if key in ("w", "a", "s", "d", "q", "e"):
                    if not event.isAutoRepeat():
                        self._speed_comm.key_press(key)
                    return True

        elif event.type() == QEvent.KeyRelease:
            if self._speed_comm.kb_active:
                key = event.text().lower()
                if key in ("w", "a", "s", "d", "q", "e"):
                    if not event.isAutoRepeat():
                        self._speed_comm.key_release(key)
                    return True

        return super().eventFilter(obj, event)

    # ==================================================================
    #  速度显示刷新 (100ms)
    # ==================================================================
    def _refresh_speed(self):
        self._speed_comm.restart_sub_if_needed()
        self._odom_tracker.restart_sub_if_needed()
        self._speed_comm.log_tick()
        with self._speed_comm.lock:
            r_vx = self._speed_comm.ros_vx
            r_vy = self._speed_comm.ros_vy
            r_wz = self._speed_comm.ros_wz
            s_vx = self._speed_comm.sent_vx
            s_vy = self._speed_comm.sent_vy
            s_wz = self._speed_comm.sent_wz
            source = self._speed_comm.send_source
            sending = self._speed_comm.send_enabled
        self._recv_label.setText(
            f"接收: Vx {r_vx:+.3f} | Vy {r_vy:+.3f} | Wz {r_wz:+.3f}"
        )
        if sending:
            self._sent_label.setText(
                f"发送: Vx {s_vx:+.3f} | Vy {s_vy:+.3f} | Wz {s_wz:+.3f}  [{source}]"
            )
            self._sent_label.setStyleSheet("color:#3498db; margin:0; padding:0;")
        else:
            self._sent_label.setText("发送: 未启用")
            self._sent_label.setStyleSheet("color:#7f8c8d; margin:0; padding:0;")

        km = self._odom_tracker.total_km
        total_km = self._odom_tracker.path_total_km
        self._distance_label.setText(f"路径: {km:.3f} / {total_km:.3f} km")

    # ==================================================================
    #  定时刷新
    # ==================================================================
    def _update_all_status(self):
        while True:
            try:
                item = self._stop_done.popleft()
            except IndexError:
                break
            if isinstance(item, tuple) and item[0] == "_record":
                _, elapsed = item
                self._rosbag_proc = None
                self._record_start_time = None
                self._rec_busy = False
                mins, secs = divmod(elapsed, 60)
                self._log(f"录制已停止 (时长 {mins:02d}:{secs:02d})")
            else:
                self._busy_keys.discard(item)
                self._log(f"{self.modules[item].name} 已停止")
        for key in self.module_order:
            if key not in self._busy_keys:
                self._refresh_ui(key)
        if not self._rec_busy:
            self._refresh_record_ui()

    # ==================================================================
    #  关闭清理
    # ==================================================================
    def _kill_pg(self, proc, timeout=5):
        if proc and proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                proc.wait(timeout=timeout)
            except Exception:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except Exception:
                    pass
                try:
                    proc.wait(timeout=2)
                except Exception:
                    pass

    def _shutdown_all(self):
        self._kill_pg(self._rosbag_proc)
        self._kill_pg(self._rtk_proc, 2)
        self._speed_comm.kb_active = False
        self._speed_comm.cleanup()
        self._odom_tracker.cleanup()
        for key in reversed(self.module_order):
            m = self.modules[key]
            if m.is_running or key in self._busy_keys:
                self._log(f"清理 {m.name} ...")
                m.stop()
        if self.roscore_started_by_us and self.roscore_proc and self.roscore_proc.poll() is None:
            self._log("关闭 roscore ...")
            self._kill_pg(self.roscore_proc)

    def closeEvent(self, event):
        self._shutdown_all()
        event.accept()


# ---------------------------------------------------------------------------
# 样式表
# ---------------------------------------------------------------------------
STYLESHEET = """
QMainWindow { background-color: #2b2b2b; }
QWidget#rtkBar {
    background-color: #34495e;
    border: 1px solid #4a6785;
    border-radius: 6px;
}
QGroupBox {
    background-color: #3c3f41;
    border: 1px solid #555;
    border-radius: 6px;
    margin-top: 12px;
    padding: 8px 6px 6px 6px;
    color: #ecf0f1;
    font-weight: bold;
    font-size: 12px;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 10px;
    padding: 0 4px;
}
QPushButton[role="start"] {
    background-color: #27ae60; color: #fff;
    border: none; border-radius: 5px;
    font-size: 13px; font-weight: bold;
    padding: 4px 12px;
}
QPushButton[role="start"]:hover   { background-color: #2ecc71; }
QPushButton[role="start"]:pressed { background-color: #1e8449; }
QPushButton[role="stop"] {
    background-color: #c0392b; color: #fff;
    border: none; border-radius: 5px;
    font-size: 13px; font-weight: bold;
    padding: 4px 12px;
}
QPushButton[role="stop"]:hover   { background-color: #e74c3c; }
QPushButton[role="stop"]:pressed { background-color: #922b21; }
QLabel { color: #ecf0f1; }
QTextEdit {
    background-color: #1e1e1e; color: #2ecc71;
    border: 1px solid #555; border-radius: 4px;
    padding: 4px;
}
QSplitter::handle { background-color: #555; }
QSplitter::handle:horizontal { width: 3px; }
QSplitter::handle:vertical   { height: 3px; }
"""


def main():
    app = QApplication(sys.argv)
    app.setStyleSheet(STYLESHEET)
    window = ControlPanel()
    window.show()
    signal.signal(signal.SIGINT, lambda *_: window.close())
    heartbeat = QTimer()
    heartbeat.start(500)
    heartbeat.timeout.connect(lambda: None)
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()

