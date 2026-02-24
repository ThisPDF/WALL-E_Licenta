#!/usr/bin/env python3
"""
human_follower.py — v10:
  [v10-1] STICKY TARGET LOCK pe LIDAR — fara limita de persoane detectate
           Scor confidence = hits * w_hits + yolo_align_bonus * w_yolo
           YOLO imbunatateste scorul candidatului cel mai aliniat cu bbox-ul
  [v10-2] OBSTACLE DETECTION = PRIORITATE ABSOLUTA (grad 1)
           on_scan() seteaza _obs_emergency daca front_min < obs_stop_dist
           control_step() verifica PRIMA DATA obs_emergency, inainte de orice
  [v10-3] Eliminat lidar_max_persons / lidar_multi_suppressed complet
"""
import math
import time
import logging
import logging.handlers
import os
from collections import deque
from typing import Optional, Tuple, Dict

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PointStamped, Twist
from sensor_msgs.msg import LaserScan


# ── Utilitare ─────────────────────────────────────────────────────────────────

def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def median(buf) -> float:
    s = sorted(buf)
    return s[len(s) // 2]


def normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


# ── Structura obstacole pe sectoare ───────────────────────────────────────────

class ObstacleSectors:
    def __init__(self):
        self.front_left  = float("inf")
        self.front_ctr   = float("inf")
        self.front_right = float("inf")
        self.side_left   = float("inf")
        self.side_right  = float("inf")
        self.timestamp: Optional[float] = None

    @property
    def front_min(self) -> float:
        return min(self.front_left, self.front_ctr, self.front_right)

    @property
    def clear_side(self) -> str:
        return "LEFT" if self.side_left >= self.side_right else "RIGHT"

    def is_valid(self, timeout: float = 0.5) -> bool:
        return self.timestamp is not None and (time.time() - self.timestamp) <= timeout


# ── Nod principal ─────────────────────────────────────────────────────────────

class HumanFollowerNode(Node):
    def __init__(self):
        super().__init__("human_follower")

        # ── Topics ────────────────────────────────────────────────────────
        self.declare_parameter("cmd_vel_topic",          "/cmd_vel")
        self.declare_parameter("lidar_detections_topic", "/human_detections")
        self.declare_parameter("yolo_target_topic",      "/yolo_person_target")
        self.declare_parameter("scan_topic",             "/scan")

        # ── Comportament ──────────────────────────────────────────────────
        self.declare_parameter("approach_distance",   0.8)
        self.declare_parameter("max_linear",          1.2)
        self.declare_parameter("max_angular",         1.8)
        self.declare_parameter("k_lin",               2.5)
        self.declare_parameter("k_ang",               2.0)
        self.declare_parameter("target_timeout",      1.0)
        self.declare_parameter("dist_tinta",          1.2)
        self.declare_parameter("dist_dead_zone",      0.05)
        self.declare_parameter("dist_max_back",       0.3)
        self.declare_parameter("dist_collision_stop", 0.4)

        # ── LiDAR tracking ────────────────────────────────────────────────
        self.declare_parameter("lidar_angle_offset",     0.0)
        self.declare_parameter("lidar_dist_filter_size", 3)
        self.declare_parameter("lidar_max_dist_jump",    1.5)
        self.declare_parameter("lidar_spike_reset_time", 2.0)
        self.declare_parameter("lidar_max_track_angle",  120.0)

        # ── [v10] Sticky target lock ───────────────────────────────────────
        # Scor candidat = hits * w_hits + yolo_bonus * w_yolo
        self.declare_parameter("lock_max_dist_m",      0.6)
        self.declare_parameter("lock_min_hits",        3)
        self.declare_parameter("lock_timeout",         1.5)
        self.declare_parameter("lock_max_candidates",  8)
        self.declare_parameter("lock_ema_alpha",       0.4)
        # Unghi maxim intre directia YOLO si directia candidatului
        # pentru a acorda yolo_bonus (radiani)
        self.declare_parameter("lock_yolo_align_deg",  25.0)
        # Pondere hits vs bonus YOLO in scorul total
        self.declare_parameter("lock_w_hits",          1.0)
        self.declare_parameter("lock_w_yolo",          30.0)  # echivalent ~30 hits

        # ── Obstacole LaserScan ───────────────────────────────────────────
        self.declare_parameter("obs_stop_dist",       0.20)
        self.declare_parameter("obs_warn_dist",       0.45)
        self.declare_parameter("obs_front_half_deg",  20.0)
        self.declare_parameter("obs_front_full_deg",  50.0)
        self.declare_parameter("obs_side_deg",        90.0)
        self.declare_parameter("obs_scan_timeout",    0.5)
        self.declare_parameter("obs_k_ang",           1.5)
        self.declare_parameter("obs_lin_reduce",      0.5)
        self.declare_parameter("obs_min_range",       0.25)

        # ── OBSTACLE_DODGE ────────────────────────────────────────────────
        self.declare_parameter("dodge_timeout",          3.0)
        self.declare_parameter("dodge_angular_speed",    0.6)
        self.declare_parameter("dodge_enabled",          True)
        self.declare_parameter("dodge_back_duration",    0.5)
        self.declare_parameter("dodge_back_speed",       0.20)
        self.declare_parameter("dodge_cooldown",         1.5)

        # ── Debounce stare ────────────────────────────────────────────────
        self.declare_parameter("follow_enter_delay", 0.2)
        self.declare_parameter("follow_exit_delay",  0.8)

        # ── Smoothing ─────────────────────────────────────────────────────
        self.declare_parameter("control_rate_hz",     20.0)
        self.declare_parameter("max_lin_accel",        8.0)
        self.declare_parameter("max_ang_accel",        8.0)
        self.declare_parameter("publish_epsilon_lin",  0.002)
        self.declare_parameter("publish_epsilon_ang",  0.005)

        # ── Misc ──────────────────────────────────────────────────────────
        self.declare_parameter("forward_sign",        1.0)
        self.declare_parameter("centered_threshold",  0.10)

        # ── YOLO ──────────────────────────────────────────────────────────
        self.declare_parameter("h_min_valid",      0.05)
        self.declare_parameter("h_max_valid",      0.85)
        self.declare_parameter("yolo_filter_size", 3)
        self.declare_parameter("h_set",            0.30)

        # ── SEARCH / RECOVERY ─────────────────────────────────────────────
        self.declare_parameter("search_angular_speed",  0.5)
        self.declare_parameter("search_timeout",        8.0)
        self.declare_parameter("search_alternating",    True)
        self.declare_parameter("search_lin_speed",      0.15)
        self.declare_parameter("spin180_angular_speed", 1.2)
        self.declare_parameter("spin180_enabled",       True)

        # ── Logging ───────────────────────────────────────────────────────
        self.declare_parameter("log_file",              "~/follow_logs/follower.log")
        self.declare_parameter("log_file_max_bytes",    5_000_000)
        self.declare_parameter("log_file_backup_count", 3)

        # ── Citire parametri ──────────────────────────────────────────────
        self.approach_distance    = float(self.get_parameter("approach_distance").value)
        self.max_linear           = float(self.get_parameter("max_linear").value)
        self.max_angular          = float(self.get_parameter("max_angular").value)
        self.k_lin                = float(self.get_parameter("k_lin").value)
        self.k_ang                = float(self.get_parameter("k_ang").value)
        self.target_timeout       = float(self.get_parameter("target_timeout").value)
        self.dist_tinta           = float(self.get_parameter("dist_tinta").value)
        self.dist_dead_zone       = float(self.get_parameter("dist_dead_zone").value)
        self.dist_max_back        = float(self.get_parameter("dist_max_back").value)
        self.dist_collision_stop  = float(self.get_parameter("dist_collision_stop").value)
        self.follow_enter_delay   = float(self.get_parameter("follow_enter_delay").value)
        self.follow_exit_delay    = float(self.get_parameter("follow_exit_delay").value)
        self.control_rate_hz      = float(self.get_parameter("control_rate_hz").value)
        self.max_lin_accel        = float(self.get_parameter("max_lin_accel").value)
        self.max_ang_accel        = float(self.get_parameter("max_ang_accel").value)
        self.publish_eps_lin      = float(self.get_parameter("publish_epsilon_lin").value)
        self.publish_eps_ang      = float(self.get_parameter("publish_epsilon_ang").value)
        self.forward_sign         = float(self.get_parameter("forward_sign").value)
        self.centered_threshold   = float(self.get_parameter("centered_threshold").value)
        self.lidar_angle_offset   = float(self.get_parameter("lidar_angle_offset").value)
        self.lidar_max_dist_jump  = float(self.get_parameter("lidar_max_dist_jump").value)
        self.lidar_spike_reset_t  = float(self.get_parameter("lidar_spike_reset_time").value)
        self.lidar_max_track_angle = math.radians(
            float(self.get_parameter("lidar_max_track_angle").value)
        )
        self.h_min_valid          = float(self.get_parameter("h_min_valid").value)
        self.h_max_valid          = float(self.get_parameter("h_max_valid").value)
        self.h_set                = float(self.get_parameter("h_set").value)
        self.search_angular_speed = float(self.get_parameter("search_angular_speed").value)
        self.search_timeout       = float(self.get_parameter("search_timeout").value)
        self.search_alternating   = bool(self.get_parameter("search_alternating").value)
        self.search_lin_speed     = float(self.get_parameter("search_lin_speed").value)
        self.spin180_angular_speed= float(self.get_parameter("spin180_angular_speed").value)
        self.spin180_enabled      = bool(self.get_parameter("spin180_enabled").value)

        # Lock params
        self.lock_max_dist        = float(self.get_parameter("lock_max_dist_m").value)
        self.lock_min_hits        = int(self.get_parameter("lock_min_hits").value)
        self.lock_timeout         = float(self.get_parameter("lock_timeout").value)
        self.lock_max_cands       = int(self.get_parameter("lock_max_candidates").value)
        self.lock_ema_alpha       = float(self.get_parameter("lock_ema_alpha").value)
        self.lock_yolo_align_rad  = math.radians(
            float(self.get_parameter("lock_yolo_align_deg").value)
        )
        self.lock_w_hits          = float(self.get_parameter("lock_w_hits").value)
        self.lock_w_yolo          = float(self.get_parameter("lock_w_yolo").value)

        # Obstacole
        self.obs_stop_dist    = float(self.get_parameter("obs_stop_dist").value)
        self.obs_warn_dist    = float(self.get_parameter("obs_warn_dist").value)
        self.obs_front_half   = math.radians(float(self.get_parameter("obs_front_half_deg").value))
        self.obs_front_full   = math.radians(float(self.get_parameter("obs_front_full_deg").value))
        self.obs_side_deg_r   = math.radians(float(self.get_parameter("obs_side_deg").value))
        self.obs_scan_timeout = float(self.get_parameter("obs_scan_timeout").value)
        self.obs_k_ang        = float(self.get_parameter("obs_k_ang").value)
        self.obs_lin_reduce   = float(self.get_parameter("obs_lin_reduce").value)
        self.obs_min_range    = float(self.get_parameter("obs_min_range").value)

        # Dodge
        self.dodge_timeout       = float(self.get_parameter("dodge_timeout").value)
        self.dodge_angular_speed = float(self.get_parameter("dodge_angular_speed").value)
        self.dodge_enabled       = bool(self.get_parameter("dodge_enabled").value)
        self.dodge_back_duration = float(self.get_parameter("dodge_back_duration").value)
        self.dodge_back_speed    = float(self.get_parameter("dodge_back_speed").value)
        self.dodge_cooldown      = float(self.get_parameter("dodge_cooldown").value)

        self._h_max_valid_base = self.h_max_valid
        self._spin180_duration = math.pi / max(0.1, self.spin180_angular_speed)

        filt_l = int(self.get_parameter("lidar_dist_filter_size").value)
        filt_y = int(self.get_parameter("yolo_filter_size").value)

        # ── Logger ────────────────────────────────────────────────────────
        log_path = os.path.expanduser(str(self.get_parameter("log_file").value))
        os.makedirs(os.path.dirname(log_path), exist_ok=True)
        self._flog = logging.getLogger("follower_file")
        self._flog.setLevel(logging.DEBUG)
        self._flog.propagate = False
        if not self._flog.handlers:
            h = logging.handlers.RotatingFileHandler(
                log_path,
                maxBytes=int(self.get_parameter("log_file_max_bytes").value),
                backupCount=int(self.get_parameter("log_file_backup_count").value),
                encoding="utf-8",
            )
            h.setFormatter(logging.Formatter(
                fmt="%(asctime)s.%(msecs)03d  %(message)s",
                datefmt="%Y-%m-%d %H:%M:%S",
            ))
            self._flog.addHandler(h)

        # ── Buffere filtre ────────────────────────────────────────────────
        self.lidar_dist_buf: deque = deque(maxlen=filt_l)
        self.lidar_ang_buf:  deque = deque(maxlen=filt_l)
        self.last_lidar_raw_dist: Optional[float] = None
        self.last_lidar_raw_time: Optional[float] = None

        self.yolo_x_buf: deque = deque(maxlen=filt_y)
        self.yolo_h_buf: deque = deque(maxlen=filt_y)

        # ── Stare senzori ─────────────────────────────────────────────────
        self.last_lidar:      Optional[Tuple[float, float, float]] = None
        self.last_yolo:       Optional[Tuple[float, float, float]] = None
        self.last_lidar_raw:  Optional[Tuple[float, float, float]] = None
        self.last_yolo_raw:   Optional[Tuple[float, float, float]] = None
        self.last_yolo_rejected: Optional[str] = None
        self.lidar_person_count: int = 0

        # ── [v10] Sticky lock ─────────────────────────────────────────────
        # candidat: {"x", "y", "hits", "last_seen", "yolo_bonus"}
        self._lock_candidates: Dict[int, dict] = {}
        self._lock_id_counter: int = 0
        self._locked_id: Optional[int] = None

        # ── [v10] Obstacole — stare PRIORITATE 1 ─────────────────────────
        self.obs: ObstacleSectors = ObstacleSectors()
        self._obs_hard_stop_active: bool = False
        # Flag setat direct de on_scan() — verificat PRIMUL in control_step
        self._obs_emergency: bool = False

        # ── Memorie directie ──────────────────────────────────────────────
        self.last_known_x_norm: float = 0.0
        self.last_known_side:   str   = "FRONT"

        # ── Masina de stari ───────────────────────────────────────────────
        self.state      = "APPROACH"
        self.last_state = None

        self.yolo_since:         Optional[float] = None
        self.lidar_since:        Optional[float] = None
        self.yolo_missing_since: Optional[float] = None

        self.search_start_time:  Optional[float] = None
        self.spin180_start_time: Optional[float] = None
        self.dodge_start_time:   Optional[float] = None
        self._dodge_exit_time:   Optional[float] = None
        self._pre_dodge_state:   str             = "FOLLOW"
        self._dodge_dir:         float           = 1.0
        self._search_dir:        float           = 1.0

        # ── Comenzi ───────────────────────────────────────────────────────
        self.last_cmd_lin     = 0.0
        self.last_cmd_ang     = 0.0
        self.last_publish_lin = 0.0
        self.last_publish_ang = 0.0
        self.step_count = 0

        # ── Publisher / Subscribers / Timer ──────────────────────────────
        self.cmd_pub = self.create_publisher(
            Twist, self.get_parameter("cmd_vel_topic").value, 10
        )
        self.create_subscription(
            PoseArray,
            self.get_parameter("lidar_detections_topic").value,
            self.on_lidar, 10
        )
        self.create_subscription(
            PointStamped,
            self.get_parameter("yolo_target_topic").value,
            self.on_yolo, 10
        )
        self.create_subscription(
            LaserScan,
            self.get_parameter("scan_topic").value,
            self.on_scan, 10
        )

        timer_dt = 1.0 / max(1.0, self.control_rate_hz)
        self.timer = self.create_timer(timer_dt, self.control_step)

        self.get_logger().info(
            f"[v10] START  lock_min_hits={self.lock_min_hits}  "
            f"lock_timeout={self.lock_timeout}s  "
            f"lock_w_yolo={self.lock_w_yolo}  "
            f"obs_stop={self.obs_stop_dist}m  obs_warn={self.obs_warn_dist}m"
        )
        self._flog.info("=" * 70)
        self._flog.info("  HUMAN FOLLOWER v10 — START")
        self._flog.info(
            f"  STICKY LOCK: min_hits={self.lock_min_hits}  timeout={self.lock_timeout}s  "
            f"max_dist={self.lock_max_dist}m  yolo_align={math.degrees(self.lock_yolo_align_rad):.0f}deg"
        )
        self._flog.info(
            f"  SCORE: w_hits={self.lock_w_hits}  w_yolo={self.lock_w_yolo}"
        )
        self._flog.info(
            f"  OBS [P1]: stop={self.obs_stop_dist}m  warn={self.obs_warn_dist}m  "
            f"min_range={self.obs_min_range}m"
        )
        self._flog.info("=" * 70)

    # ── Wrapper logging ───────────────────────────────────────────────────
    def _log(self, msg: str, level: str = "info"):
        getattr(self._flog, level)(msg)
        if level == "warn":
            self.get_logger().warn(msg)
        elif level == "error":
            self.get_logger().error(msg)

    # ═══════════════════════════════════════════════════════════════════════
    # CALLBACKS SENZORI
    # ═══════════════════════════════════════════════════════════════════════

    # ── [v10-P1] on_scan — seteaza _obs_emergency imediat ─────────────────
    def on_scan(self, msg: LaserScan):
        obs = ObstacleSectors()
        obs.timestamp = time.time()

        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            if r < self.obs_min_range:
                continue
            angle = normalize_angle(msg.angle_min + i * msg.angle_increment)
            abs_a = abs(angle)
            if abs_a <= self.obs_front_half:
                if r < obs.front_ctr:
                    obs.front_ctr = r
            elif abs_a <= self.obs_front_full:
                if angle > 0:
                    if r < obs.front_left:
                        obs.front_left = r
                else:
                    if r < obs.front_right:
                        obs.front_right = r
            elif abs_a <= self.obs_side_deg_r:
                if angle > 0:
                    if r < obs.side_left:
                        obs.side_left = r
                else:
                    if r < obs.side_right:
                        obs.side_right = r

        self.obs = obs

        # [v10-P1] Seteaza flag de urgenta direct din callback, fara sa astepte control_step
        prev_emergency = self._obs_emergency
        self._obs_emergency = obs.front_min < self.obs_stop_dist
        if self._obs_emergency and not prev_emergency:
            self._log(
                f"[OBS-P1] *** EMERGENCY SET *** front_min={obs.front_min:.3f}m "
                f"< stop={self.obs_stop_dist}m  clear={obs.clear_side}",
                "warn"
            )
        elif not self._obs_emergency and prev_emergency:
            self._log(
                f"[OBS-P1] Emergency cleared  front_min={obs.front_min:.3f}m",
                "warn"
            )

        self._flog.info(
            f"  [SCAN] FL={obs.front_left:.2f}m FC={obs.front_ctr:.2f}m "
            f"FR={obs.front_right:.2f}m SL={obs.side_left:.2f}m SR={obs.side_right:.2f}m "
            f"front_min={obs.front_min:.2f}m  emergency={self._obs_emergency}"
        )

    # ── [v10] on_lidar — sticky lock fara limita de persoane ──────────────
    def on_lidar(self, msg: PoseArray):
        n_detected = len(msg.poses)
        self.lidar_person_count = n_detected
        now = time.time()

        if n_detected == 0:
            return

        # Actualizeaza candidatii cu TOATE detectiile
        self._update_lock_candidates(msg.poses, now)

        # Alege candidatul locked
        chosen = self._pick_locked_detection(now)
        if chosen is None:
            return

        raw_x, raw_y = chosen["x"], chosen["y"]
        raw_d   = math.hypot(raw_x, raw_y)
        raw_ang = math.atan2(raw_y, raw_x)

        # Filtru unghi: ignora detectii prea mult in spate
        corr_ang_check = normalize_angle(raw_ang + self.lidar_angle_offset)
        if abs(corr_ang_check) > self.lidar_max_track_angle:
            self._log(
                f"[LIDAR CB] IGNORAT — unghi {math.degrees(corr_ang_check):.1f}deg "
                f"> max_track={math.degrees(self.lidar_max_track_angle):.0f}deg",
                "warn"
            )
            return

        # Anti-spike
        if self.last_lidar_raw_time is not None:
            pauza = now - self.last_lidar_raw_time
            if pauza > self.lidar_spike_reset_t:
                self._log(
                    f"[LIDAR] Pauza {pauza:.1f}s > {self.lidar_spike_reset_t}s — reset",
                    "warn"
                )
                self.last_lidar_raw_dist = None
                self.lidar_dist_buf.clear()
                self.lidar_ang_buf.clear()

        if self.last_lidar_raw_dist is not None:
            jump = abs(raw_d - self.last_lidar_raw_dist)
            if jump > self.lidar_max_dist_jump:
                self._log(
                    f"[LIDAR] SPIKE RESPINS: {raw_d:.2f}m "
                    f"prev={self.last_lidar_raw_dist:.2f}m jump={jump:.2f}m",
                    "warn"
                )
                return

        self.last_lidar_raw_dist = raw_d
        self.last_lidar_raw_time = now
        self.last_lidar_raw      = (raw_d, math.degrees(raw_ang), now)

        corr_ang = normalize_angle(raw_ang + self.lidar_angle_offset)
        self.lidar_dist_buf.append(raw_d)
        self.lidar_ang_buf.append(corr_ang)

        med_d   = median(self.lidar_dist_buf)
        med_ang = median(self.lidar_ang_buf)
        med_x   = med_d * math.cos(med_ang)
        med_y   = med_d * math.sin(med_ang)
        self.last_lidar = (med_x, med_y, now)

        score = self._candidate_score(chosen)
        self._log(
            f"[LIDAR CB] lock=#{self._locked_id}  "
            f"persons={n_detected}  hits={chosen['hits']}  "
            f"yolo_bonus={chosen['yolo_bonus']:.1f}  score={score:.1f}  "
            f"d={med_d:.3f}m ang={math.degrees(med_ang):.1f}deg "
            f"x={med_x:.3f} y={med_y:.3f}  "
            f"{'FATA' if med_x >= 0 else '*** SPATE ***'}"
        )

    # ── on_yolo — actualizeaza si yolo_bonus pe candidati ─────────────────
    def on_yolo(self, msg: PointStamped):
        x_norm = float(msg.point.x)
        h_norm = float(msg.point.y)
        self.last_yolo_raw = (x_norm, h_norm, time.time())

        if h_norm < self.h_min_valid:
            self.last_yolo_rejected = f"h={h_norm:.3f} < h_min={self.h_min_valid}"
            return
        if h_norm > self.h_max_valid:
            self.last_yolo_rejected = f"h={h_norm:.3f} > h_max={self.h_max_valid:.2f}"
            return

        self.last_yolo_rejected = None
        self.yolo_x_buf.append(x_norm)
        self.yolo_h_buf.append(h_norm)
        self.last_yolo = (median(self.yolo_x_buf), median(self.yolo_h_buf), time.time())
        self._update_known_side(median(self.yolo_x_buf))

        # [v10] Acorda yolo_bonus candidatilor aliniati cu bbox-ul YOLO
        self._apply_yolo_bonus_to_candidates(x_norm)

    # ═══════════════════════════════════════════════════════════════════════
    # [v10] STICKY LOCK HELPERS
    # ═══════════════════════════════════════════════════════════════════════

    def _candidate_score(self, c: dict) -> float:
        """Scor = hits * w_hits + yolo_bonus * w_yolo"""
        return c["hits"] * self.lock_w_hits + c["yolo_bonus"] * self.lock_w_yolo

    def _update_lock_candidates(self, poses, now: float):
        """Asociaza fiecare pose cu candidat existent (EMA) sau creeaza unul nou."""
        alpha = self.lock_ema_alpha
        used_cids = set()

        for p in poses:
            px, py = p.position.x, p.position.y

            best_cid  = None
            best_dist = float("inf")
            for cid, c in self._lock_candidates.items():
                if cid in used_cids:
                    continue
                d = math.hypot(px - c["x"], py - c["y"])
                if d < best_dist and d < self.lock_max_dist:
                    best_dist = d
                    best_cid  = cid

            if best_cid is not None:
                c = self._lock_candidates[best_cid]
                c["x"]         = alpha * px + (1.0 - alpha) * c["x"]
                c["y"]         = alpha * py + (1.0 - alpha) * c["y"]
                c["hits"]      = min(c["hits"] + 1, 9999)
                c["last_seen"] = now
                used_cids.add(best_cid)
            else:
                if len(self._lock_candidates) >= self.lock_max_cands:
                    # Sterge candidatul cu scorul cel mai mic (ne-locked)
                    worst = min(
                        (cid for cid in self._lock_candidates
                         if cid != self._locked_id),
                        key=lambda cid: self._candidate_score(self._lock_candidates[cid]),
                        default=None
                    )
                    if worst is not None:
                        del self._lock_candidates[worst]

                self._lock_id_counter += 1
                new_id = self._lock_id_counter
                self._lock_candidates[new_id] = {
                    "x":          px,
                    "y":          py,
                    "hits":       1,
                    "last_seen":  now,
                    "yolo_bonus": 0.0,
                }
                self._log(
                    f"[LOCK] Candidat nou #{new_id} la x={px:.2f} y={py:.2f}"
                )

    def _apply_yolo_bonus_to_candidates(self, yolo_x_norm: float):
        """
        Calculeaza directia YOLO in radiani (estimata din x_norm normalizat)
        si acorda bonus candidatilor al caror unghi e aproape de directia YOLO.
        yolo_x_norm in [-1, 1]: -1=stanga, 0=centru, +1=dreapta
        Aproximare unghi YOLO: atan(x_norm * tan(FOV/2)), FOV ~60deg
        """
        yolo_ang = math.atan(yolo_x_norm * math.tan(math.radians(30.0)))

        for cid, c in self._lock_candidates.items():
            cand_ang = normalize_angle(
                math.atan2(c["y"], c["x"]) + self.lidar_angle_offset
            )
            diff = abs(normalize_angle(cand_ang - yolo_ang))
            if diff < self.lock_yolo_align_rad:
                # Bonus proportional cu alinierea (1.0 la 0 grade, 0.0 la limita)
                bonus = 1.0 - (diff / self.lock_yolo_align_rad)
                old_bonus = c["yolo_bonus"]
                # EMA pe bonus pentru a evita spike-uri
                c["yolo_bonus"] = 0.7 * old_bonus + 0.3 * bonus
                self._flog.info(
                    f"  [LOCK] YOLO bonus #{cid}: "
                    f"diff={math.degrees(diff):.1f}deg  "
                    f"bonus={c['yolo_bonus']:.3f}  "
                    f"score={self._candidate_score(c):.1f}"
                )
            else:
                # Decay bonus daca YOLO nu mai confirma
                c["yolo_bonus"] = c["yolo_bonus"] * 0.85

    def _pick_locked_detection(self, now: float) -> Optional[dict]:
        """
        Returneaza candidatul activ:
        - Curata candidatii expirati
        - Pastreaza lock daca e valid
        - Altfel alege candidatul cu scorul maxim (hits + yolo_bonus)
        """
        # Curata expirati
        expired = [
            cid for cid, c in self._lock_candidates.items()
            if (now - c["last_seen"]) > self.lock_timeout
        ]
        for cid in expired:
            if self._locked_id == cid:
                self._log(
                    f"[LOCK] Candidat locked #{cid} EXPIRAT → unlock",
                    "warn"
                )
                self._locked_id = None
            del self._lock_candidates[cid]

        if not self._lock_candidates:
            return None

        # Verifica lock curent
        if self._locked_id is not None:
            locked = self._lock_candidates.get(self._locked_id)
            if locked and locked["hits"] >= self.lock_min_hits:
                return locked
            self._log(
                f"[LOCK] Lock #{self._locked_id} invalid → recalculez",
                "warn"
            )
            self._locked_id = None

        # Alege candidatul cu scorul maxim
        best_cid = max(
            self._lock_candidates,
            key=lambda cid: self._candidate_score(self._lock_candidates[cid])
        )
        best = self._lock_candidates[best_cid]

        if best["hits"] >= self.lock_min_hits:
            if self._locked_id != best_cid:
                self._log(
                    f"[LOCK] *** LOCK pe #{best_cid}  "
                    f"hits={best['hits']}  yolo_bonus={best['yolo_bonus']:.3f}  "
                    f"score={self._candidate_score(best):.1f}  "
                    f"x={best['x']:.2f} y={best['y']:.2f} ***",
                    "warn"
                )
                self._locked_id = best_cid
                self.lidar_dist_buf.clear()
                self.lidar_ang_buf.clear()
                self.last_lidar_raw_dist = None
            return best

        # Fallback la cel mai aproape daca nu exista candidat suficient de consistent
        closest_cid = min(
            self._lock_candidates,
            key=lambda cid: math.hypot(
                self._lock_candidates[cid]["x"],
                self._lock_candidates[cid]["y"]
            )
        )
        return self._lock_candidates[closest_cid]

    # ═══════════════════════════════════════════════════════════════════════
    # UTILITARE CONTROL
    # ═══════════════════════════════════════════════════════════════════════

    def _apply_obstacle_modulation(
        self, lin: float, ang: float, moving_forward: bool
    ) -> Tuple[float, float, bool]:
        if not self.obs.is_valid(self.obs_scan_timeout):
            return lin, ang, False
        if not moving_forward or lin <= 0.0:
            return lin, ang, False
        fm = self.obs.front_min
        if fm < self.obs_stop_dist:
            self._flog.info(
                f"  [OBS] HARD STOP front_min={fm:.3f}m < stop={self.obs_stop_dist}m"
            )
            self._obs_hard_stop_active = True
            return 0.0, ang, True
        self._obs_hard_stop_active = False
        if fm < self.obs_warn_dist:
            factor  = (fm - self.obs_stop_dist) / (self.obs_warn_dist - self.obs_stop_dist)
            lin_mod = lin * clamp(factor, 0.0, 1.0) * self.obs_lin_reduce
            clear   = self.obs.clear_side
            obs_ang_offset = (
                self.obs_k_ang * (1.0 - factor)
                if clear == "LEFT"
                else -self.obs_k_ang * (1.0 - factor)
            )
            ang_mod = clamp(ang + obs_ang_offset, -self.max_angular, self.max_angular)
            self._flog.info(
                f"  [OBS] WARN front_min={fm:.3f}m factor={factor:.2f} "
                f"lin:{lin:.3f}→{lin_mod:.3f} ang_offset={obs_ang_offset:+.2f} clear={clear}"
            )
            return lin_mod, ang_mod, False
        return lin, ang, False

    def _lin_from_dist(self, d: float) -> Optional[float]:
        if d <= self.dist_collision_stop:
            return None
        err = d - self.dist_tinta
        if abs(err) < self.dist_dead_zone:
            return 0.0
        return clamp(self.k_lin * err * self.forward_sign, -self.dist_max_back, self.max_linear)

    def _update_h_max_valid(self, d: Optional[float]):
        self.h_max_valid = 0.99 if (d is not None and d < self.dist_tinta + 0.5) else self._h_max_valid_base

    def _update_known_side(self, x_norm: float):
        self.last_known_x_norm = x_norm
        if x_norm < -self.centered_threshold:
            self.last_known_side = "LEFT"
        elif x_norm > self.centered_threshold:
            self.last_known_side = "RIGHT"
        else:
            self.last_known_side = "FRONT"

    def publish_cmd(self, lin_x: float, ang_z: float, dt: float) -> Tuple[float, float]:
        lin_x = clamp(lin_x, -self.max_linear, self.max_linear)
        ang_z = clamp(ang_z, -self.max_angular, self.max_angular)
        max_dlin = self.max_lin_accel * dt
        max_dang = self.max_ang_accel * dt
        lin_x = clamp(lin_x, self.last_cmd_lin - max_dlin, self.last_cmd_lin + max_dlin)
        ang_z = clamp(ang_z, self.last_cmd_ang - max_dang, self.last_cmd_ang + max_dang)
        self.last_cmd_lin = lin_x
        self.last_cmd_ang = ang_z
        if (abs(lin_x - self.last_publish_lin) >= self.publish_eps_lin or
                abs(ang_z - self.last_publish_ang) >= self.publish_eps_ang):
            cmd = Twist()
            cmd.linear.x  = -lin_x
            cmd.angular.z = ang_z
            self.cmd_pub.publish(cmd)
            self.last_publish_lin = lin_x
            self.last_publish_ang = ang_z
        return lin_x, ang_z

    def _search_commands(self, elapsed: float) -> Tuple[float, float]:
        T   = self.search_timeout
        ang = self._search_dir * self.search_angular_speed
        if elapsed < T / 3.0:
            lin, phase = 0.0, "ROT-LOC"
        elif elapsed < 2.0 * T / 3.0:
            ang = -self._search_dir * self.search_angular_speed
            lin, phase = 0.0, "ROT-LOC-REV"
        else:
            lin, phase = self.search_lin_speed, "ROT+LIN"
        self._flog.info(
            f"  [SEARCH] faza={phase} elapsed={elapsed:.1f}/{T:.1f}s "
            f"lin={lin:.2f} ang={ang:.2f}"
        )
        return lin, ang

    def _rotation_safe(self, direction: float) -> bool:
        if not self.obs.is_valid(self.obs_scan_timeout):
            return True
        return (self.obs.side_left > self.obs_stop_dist if direction > 0
                else self.obs.side_right > self.obs_stop_dist)

    # ═══════════════════════════════════════════════════════════════════════
    # CONTROL LOOP
    # ═══════════════════════════════════════════════════════════════════════

    def control_step(self):
        now = time.time()
        dt  = 1.0 / max(1.0, self.control_rate_hz)
        self.step_count += 1

        lidar_ok = (self.last_lidar is not None and
                    (now - self.last_lidar[2]) <= self.target_timeout)
        yolo_ok  = (self.last_yolo is not None and
                    (now - self.last_yolo[2]) <= self.target_timeout)

        current_dist = None
        if lidar_ok and self.last_lidar:
            lx, ly, _ = self.last_lidar
            current_dist = math.hypot(lx, ly)
        self._update_h_max_valid(current_dist)

        if yolo_ok:
            if self.yolo_since is None:
                self.yolo_since = now
            self.yolo_missing_since = None
        else:
            self.yolo_since = None
            if self.yolo_missing_since is None:
                self.yolo_missing_since = now

        if lidar_ok:
            if self.lidar_since is None:
                self.lidar_since = now
        else:
            self.lidar_since = None

        self._flog.info(f"── STEP {self.step_count:05d} ──────────────────────────────────")

        # ── Log senzori ────────────────────────────────────────────────────
        locked_tag = (
            f"lock=#{self._locked_id} "
            f"hits={self._lock_candidates[self._locked_id]['hits']} "
            f"score={self._candidate_score(self._lock_candidates[self._locked_id]):.1f}"
            if self._locked_id and self._locked_id in self._lock_candidates
            else "no-lock"
        )
        self._flog.info(
            f"  LIDAR: persons={self.lidar_person_count}  ok={lidar_ok}  "
            f"candidates={len(self._lock_candidates)}  {locked_tag}"
        )
        if lidar_ok and self.last_lidar:
            lx, ly, _ = self.last_lidar
            ld   = math.hypot(lx, ly)
            lang = math.degrees(math.atan2(ly, lx))
            self._flog.info(
                f"  LIDAR filt: d={ld:.3f}m ang={lang:.1f}deg "
                f"x={lx:.3f} y={ly:.3f}  {'FATA' if lx >= 0 else 'SPATE'}"
            )
        if self.last_yolo_raw:
            yx, yh, yt = self.last_yolo_raw
            if self.last_yolo_rejected:
                self._flog.info(
                    f"  YOLO raw: x={yx:.3f} h={yh:.3f} age={now-yt:.3f}s "
                    f"RESPINS ({self.last_yolo_rejected})"
                )
            else:
                self._flog.info(
                    f"  YOLO raw: x={yx:.3f} h={yh:.3f} age={now-yt:.3f}s ok={yolo_ok}"
                )
                if yolo_ok and self.last_yolo:
                    fx, fh, _ = self.last_yolo
                    side = ("DREAPTA" if fx > self.centered_threshold else
                            "STANGA"  if fx < -self.centered_threshold else "CENTRU")
                    self._flog.info(f"  YOLO filt: x={fx:.3f} h={fh:.3f}  pozitie={side}")
        else:
            self._flog.info("  YOLO: niciun mesaj")

        if self.obs.is_valid(self.obs_scan_timeout):
            self._flog.info(
                f"  OBS[P1]: FL={self.obs.front_left:.2f}m FC={self.obs.front_ctr:.2f}m "
                f"FR={self.obs.front_right:.2f}m SL={self.obs.side_left:.2f}m "
                f"SR={self.obs.side_right:.2f}m front_min={self.obs.front_min:.2f}m "
                f"clear={self.obs.clear_side}  emergency={self._obs_emergency}"
            )

        self._flog.info(
            f"  last_known_side={self.last_known_side} x={self.last_known_x_norm:.2f}"
        )

        # ══════════════════════════════════════════════════════════════════
        # [v10-P1] PRIORITATE 1: OBSTACLE EMERGENCY
        # Verificat PRIMUL, inainte de orice logica de stare sau tracking
        # ══════════════════════════════════════════════════════════════════
        if self._obs_emergency and self.obs.is_valid(self.obs_scan_timeout):
            # Forteaza intrarea in DODGE daca nu suntem deja acolo
            if self.state != "OBSTACLE_DODGE":
                dodge_cooldown_ok = (
                    self._dodge_exit_time is None or
                    (now - self._dodge_exit_time) >= self.dodge_cooldown
                )
                if self.dodge_enabled and dodge_cooldown_ok:
                    self._pre_dodge_state = self.state
                    self.state = "OBSTACLE_DODGE"
                    self.dodge_start_time = now
                    self._dodge_dir = 1.0 if self.obs.clear_side == "LEFT" else -1.0
                    self._log(
                        f"[P1-TRANZITIE] {self._pre_dodge_state} → OBSTACLE_DODGE "
                        f"(EMERGENCY front_min={self.obs.front_min:.3f}m "
                        f"dir={self._dodge_dir:+.0f} clear={self.obs.clear_side})",
                        "warn"
                    )
                else:
                    # Dodge dezactivat sau in cooldown — hard stop direct
                    self.publish_cmd(0.0, 0.0, dt)
                    self._flog.info(
                        f"  [P1] HARD STOP (dodge disabled/cooldown) "
                        f"front_min={self.obs.front_min:.3f}m"
                    )
                    return

        # ── Tranzitii stare ─────────────────────────────────────────────────
        yolo_stable_s = (now - self.yolo_since)         if self.yolo_since         else 0.0
        yolo_miss_s   = (now - self.yolo_missing_since) if self.yolo_missing_since else 0.0
        any_sensor    = lidar_ok or yolo_ok

        if self.state in ("APPROACH", "FOLLOW"):
            if self.state == "APPROACH":
                if self.yolo_since is not None and yolo_stable_s >= self.follow_enter_delay:
                    self.state = "FOLLOW"
                elif lidar_ok and self.last_lidar:
                    x, y, _ = self.last_lidar
                    if math.hypot(x, y) <= self.approach_distance:
                        if self.yolo_since is not None and yolo_stable_s >= 0.05:
                            self.state = "FOLLOW"

            elif self.state == "FOLLOW":
                if self.yolo_missing_since is not None and yolo_miss_s >= self.follow_exit_delay:
                    if not lidar_ok:
                        if self._obs_hard_stop_active:
                            self._flog.info(
                                "  [TRANZITIE] YOLO disparut + LIDAR lipseste "
                                "dar HARD STOP activ → raman in FOLLOW"
                            )
                        elif self.spin180_enabled:
                            self.state = "SPIN180"
                            self.spin180_start_time = now
                            self._search_dir = (
                                -1.0 if self.last_known_side == "RIGHT" else 1.0
                            )
                            self._log(
                                f"[TRANZITIE] FOLLOW → SPIN180 "
                                f"(side={self.last_known_side} dir={self._search_dir:+.0f} "
                                f"durata={self._spin180_duration:.2f}s)",
                                "warn"
                            )
                        else:
                            self.state = "SEARCH"
                            self.search_start_time = now
                            self._search_dir = (
                                1.0 if self.last_known_side in ("LEFT", "FRONT") else -1.0
                            )
                            self._log(
                                f"[TRANZITIE] FOLLOW → SEARCH "
                                f"(side={self.last_known_side} dir={self._search_dir:+.0f})",
                                "warn"
                            )
                    else:
                        self._flog.info("  [TRANZITIE] YOLO disparut dar LiDAR ok → LIDAR-ONLY")

        elif self.state == "OBSTACLE_DODGE":
            elapsed_dodge = (now - self.dodge_start_time) if self.dodge_start_time else 0.0
            if self.obs.is_valid(self.obs_scan_timeout) and self.obs.front_min > self.obs_warn_dist:
                self._log(
                    f"[TRANZITIE] OBSTACLE_DODGE → {self._pre_dodge_state} "
                    f"(cale libera front_min={self.obs.front_min:.3f}m)",
                    "warn"
                )
                self.state = self._pre_dodge_state
                self.dodge_start_time = None
                self._dodge_exit_time = now
            elif elapsed_dodge >= self.dodge_timeout:
                self._log(
                    f"[TRANZITIE] OBSTACLE_DODGE → {self._pre_dodge_state} "
                    f"(timeout {elapsed_dodge:.1f}s)",
                    "warn"
                )
                self.state = self._pre_dodge_state
                self.dodge_start_time = None
                self._dodge_exit_time = now

        elif self.state == "SPIN180":
            if any_sensor:
                self._log("[TRANZITIE] SPIN180 → FOLLOW (regasit!)", "warn")
                self.state = "FOLLOW"
                self.spin180_start_time = None
            elif self.spin180_start_time is not None:
                elapsed_spin = now - self.spin180_start_time
                if elapsed_spin >= self._spin180_duration:
                    self._log(
                        f"[TRANZITIE] SPIN180 → SEARCH "
                        f"({math.degrees(self.spin180_angular_speed * elapsed_spin):.0f}° completat)",
                        "warn"
                    )
                    self.state = "SEARCH"
                    self.search_start_time = now

        elif self.state == "SEARCH":
            if any_sensor:
                self._log("[TRANZITIE] SEARCH → FOLLOW (recuperat!)", "warn")
                self.state = "FOLLOW"
                self.search_start_time = None
            elif self.search_start_time is not None:
                elapsed_search = now - self.search_start_time
                if elapsed_search >= self.search_timeout:
                    self._log(
                        f"[TRANZITIE] SEARCH → LOST dupa {elapsed_search:.1f}s",
                        "warn"
                    )
                    self.state = "LOST"

        elif self.state == "LOST":
            if any_sensor:
                self._log("[TRANZITIE] LOST → APPROACH (regasit!)", "warn")
                self.state = "APPROACH"
                self.spin180_start_time = None
                self.search_start_time  = None

        if self.state != self.last_state:
            self._log(f"*** STARE: {self.last_state} → {self.state} ***", "warn")
            self.last_state = self.state

        self._flog.info(
            f"  STARE={self.state}  yolo_stable={yolo_stable_s:.2f}s "
            f"yolo_missing={yolo_miss_s:.2f}s  "
            f"dodge_cooldown_left="
            f"{max(0.0, self.dodge_cooldown - (now - self._dodge_exit_time)) if self._dodge_exit_time else 0.0:.1f}s"
        )

        # ══════════════════════════════════════════════════════════════════
        # EXECUTIE PE STARE
        # ══════════════════════════════════════════════════════════════════

        if self.state == "OBSTACLE_DODGE":
            elapsed_dodge = (now - self.dodge_start_time) if self.dodge_start_time else 0.0
            if elapsed_dodge < self.dodge_back_duration:
                lin_dodge, ang_dodge, phase = -self.dodge_back_speed, 0.0, "BACK"
            else:
                if not self._rotation_safe(self._dodge_dir):
                    self._dodge_dir = -self._dodge_dir
                    self._log(
                        f"[DODGE] Inversat directie → {self._dodge_dir:+.0f}",
                        "warn"
                    )
                lin_dodge, ang_dodge, phase = 0.0, self._dodge_dir * self.dodge_angular_speed, "ROTATE"
            lin_out, ang_out = self.publish_cmd(lin_dodge, ang_dodge, dt)
            self._flog.info(
                f"  => OBSTACLE_DODGE [{phase}] elapsed={elapsed_dodge:.2f}s "
                f"front_min={self.obs.front_min:.3f}m CMD: lin={lin_out:.3f} ang={ang_out:.3f}"
            )
            return

        if self.state == "SPIN180":
            elapsed_spin = (now - self.spin180_start_time) if self.spin180_start_time else 0.0
            spin_speed = self.spin180_angular_speed * (0.5 if not self._rotation_safe(self._search_dir) else 1.0)
            lin_out, ang_out = self.publish_cmd(0.0, self._search_dir * spin_speed, dt)
            self._flog.info(
                f"  => SPIN180 {math.degrees(self.spin180_angular_speed * elapsed_spin):.0f}°/180° "
                f"CMD: lin={lin_out:.3f} ang={ang_out:.3f}"
            )
            return

        if self.state == "SEARCH":
            elapsed_search = (now - self.search_start_time) if self.search_start_time else 0.0
            lin_s, ang_s = self._search_commands(elapsed_search)
            lin_s, ang_s, hard_stop = self._apply_obstacle_modulation(
                lin_s, ang_s, moving_forward=(lin_s > 0)
            )
            if hard_stop:
                lin_s = 0.0
            lin_out, ang_out = self.publish_cmd(lin_s, ang_s, dt)
            self._flog.info(f"  => SEARCH CMD: lin={lin_out:.3f} ang={ang_out:.3f}")
            return

        if self.state == "LOST":
            self.publish_cmd(0.0, 0.0, dt)
            self._flog.info("  => LOST — STOP")
            return

        if not lidar_ok and not yolo_ok:
            self.publish_cmd(0.0, 0.0, dt)
            self._flog.info("  => STOP (niciun senzor)")
            return

        def compute_lin_lidar(d: float, tag: str) -> Optional[float]:
            lin = self._lin_from_dist(d)
            if lin is None:
                self._log(
                    f"HARD STOP {tag} d={d:.3f}m <= collision_stop={self.dist_collision_stop:.3f}m",
                    "warn"
                )
            return lin

        # ── FUSED: LiDAR + YOLO ────────────────────────────────────────────
        if lidar_ok and yolo_ok:
            x_lidar, y_lidar, _ = self.last_lidar
            x_norm,  h_norm,  _ = self.last_yolo
            d = math.hypot(x_lidar, y_lidar)
            lin_raw = compute_lin_lidar(d, "FUSED")
            if lin_raw is None:
                self.publish_cmd(0.0, 0.0, dt)
                return
            ang_raw = 0.0 if abs(x_norm) < self.centered_threshold else self.k_ang * x_norm
            lin_mod, ang_mod, hard_stop = self._apply_obstacle_modulation(
                lin_raw, ang_raw, moving_forward=(lin_raw > 0)
            )
            if hard_stop:
                self.publish_cmd(0.0, ang_mod, dt)
                return
            lin_out, ang_out = self.publish_cmd(lin_mod, ang_mod, dt)
            self._flog.info(
                f"  MOD=FUSED dist={d:.3f}m xnorm={x_norm:.3f} "
                f"lin={lin_raw:.3f}→{lin_mod:.3f} ang={ang_raw:.3f}→{ang_mod:.3f} "
                f"CMD lin={lin_out:.3f} ang={ang_out:.3f}"
            )
            return

        # ── LIDAR ONLY ─────────────────────────────────────────────────────
        if lidar_ok:
            x, y, _ = self.last_lidar
            d   = math.hypot(x, y)
            ang = normalize_angle(math.atan2(y, x))
            lin_raw = compute_lin_lidar(d, "LIDAR-ONLY")
            if lin_raw is None:
                self.publish_cmd(0.0, 0.0, dt)
                return
            ang_raw = 0.0 if abs(ang) < self.centered_threshold else self.k_ang * ang
            lin_mod, ang_mod, hard_stop = self._apply_obstacle_modulation(
                lin_raw, ang_raw, moving_forward=(lin_raw > 0)
            )
            if hard_stop:
                self.publish_cmd(0.0, ang_mod, dt)
                return
            lin_out, ang_out = self.publish_cmd(lin_mod, ang_mod, dt)
            self._flog.info(
                f"  MOD=LIDAR-ONLY dist={d:.3f}m ang={math.degrees(ang):.1f}deg "
                f"lin={lin_raw:.3f}→{lin_mod:.3f} ang={ang_raw:.3f}→{ang_mod:.3f} "
                f"CMD lin={lin_out:.3f} ang={ang_out:.3f}"
            )
            return

        # ── YOLO ONLY ──────────────────────────────────────────────────────
        if yolo_ok:
            x_norm, h_norm, _ = self.last_yolo
            H_STOP = 0.75
            if h_norm >= H_STOP:
                self.publish_cmd(0.0, 0.0, dt)
                self._log(
                    f"HARD STOP YOLO-ONLY h={h_norm:.3f} >= H_STOP={H_STOP}",
                    "warn"
                )
                return
            lin_raw = self.k_lin * (self.h_set - h_norm) * self.forward_sign
            ang_raw = 0.0 if abs(x_norm) < self.centered_threshold else self.k_ang * x_norm
            lin_mod, ang_mod, hard_stop = self._apply_obstacle_modulation(
                lin_raw, ang_raw, moving_forward=(lin_raw > 0)
            )
            if hard_stop:
                self.publish_cmd(0.0, ang_mod, dt)
                return
            lin_out, ang_out = self.publish_cmd(lin_mod, ang_mod, dt)
            side = ("DREAPTA" if x_norm > self.centered_threshold else
                    "STANGA"  if x_norm < -self.centered_threshold else "CENTRU")
            self._flog.info(
                f"  MOD=YOLO-ONLY pozitie={side} bbox_h={h_norm:.3f} "
                f"lin={lin_raw:.3f}→{lin_mod:.3f} ang={ang_raw:.3f}→{ang_mod:.3f} "
                f"CMD lin={lin_out:.3f} ang={ang_out:.3f}"
            )
            return

        self.publish_cmd(0.0, 0.0, dt)
        self._flog.info("  => STOP (mismatch rezidual)")


def main():
    rclpy.init()
    node = HumanFollowerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
