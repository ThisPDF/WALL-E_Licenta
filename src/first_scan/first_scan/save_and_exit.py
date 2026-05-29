#!/usr/bin/env python3
"""
Watcher pentru first_scan:
- monitorizeaza explore_lite (frontier exploration)
- detecteaza "explore done" prin urmatoarele heuristici (in ordine):
    a) /explore/frontiers e gol (zero markere active) pentru
       no_frontiers_seconds — PRIMARA, semnal direct ca nu mai exista
       frontiere de explorat.
    b) /explore/resume publica False (explore_lite semnaleaza done)
    c) timeout global (max_explore_seconds)
    d) fallback: robotul nu mai primeste goaluri/nu se misca pentru
       done_idle_seconds (foarte lung, doar daca robotul e blocat fizic)
- la "done": apeleaza /slam_toolbox/serialize_map cu calea data ca param
- iese cu rclpy.shutdown() → launch-ul detecteaza prin OnProcessExit si
  porneste urmarirea.

De ce nod separat: explore_lite nu are mecanism nativ de "save & quit". Acesta
e watcher-ul minimal care leaga capatul exploratorului de salvarea slam.
"""
import os
import time
from typing import Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray, Marker


class SaveAndExit(Node):
    def __init__(self):
        super().__init__("first_scan_save_and_exit")

        self.declare_parameter("map_save_path",       "~/maps/carucior_market")
        # Fallback: idle indelungat (robot blocat fizic, nu primeste goaluri)
        self.declare_parameter("done_idle_seconds",   180.0)
        # Timeout global de siguranta — explore_lite nu se opreste niciodata
        self.declare_parameter("max_explore_seconds", 600.0)  # 10 min
        # Minim timp inainte sa putem declara "done" (sa nu salvam la primul tick)
        self.declare_parameter("min_explore_seconds", 60.0)
        # PRIMAR: dupa cat timp fara frontiere active declaram done.
        # Trebuie sa fie suficient cat explore_lite sa nu publice intamplator
        # un mesaj gol (in timpul re-planning-ului), dar nu prea mare.
        self.declare_parameter("no_frontiers_seconds", 25.0)

        self.map_save_path = os.path.expanduser(
            str(self.get_parameter("map_save_path").value)
        )
        self.done_idle_s   = float(self.get_parameter("done_idle_seconds").value)
        self.max_explore_s = float(self.get_parameter("max_explore_seconds").value)
        self.min_explore_s = float(self.get_parameter("min_explore_seconds").value)
        self.no_front_s    = float(self.get_parameter("no_frontiers_seconds").value)

        os.makedirs(os.path.dirname(self.map_save_path), exist_ok=True)

        self._start_time    = time.time()
        self._last_goal_t:  Optional[float] = None
        self._last_motion_t: Optional[float] = None
        self._last_pose = None
        self._explore_resumed = True
        # Ultima oara cand am vazut frontiere active (markere ADD/MODIFY > 0).
        # None = niciodata vazut. Folosim pentru detectia "no more frontiers".
        self._last_active_frontiers_t: Optional[float] = None
        self._saved = False

        # Subscriptii diagnostic
        self.create_subscription(
            Bool, "/explore/resume", self._on_resume, 10
        )
        self.create_subscription(
            PoseStamped, "/explore/frontier", self._on_frontier_goal, 10
        )
        self.create_subscription(
            Odometry, "/odom", self._on_odom, 10
        )
        # PRIMARUL semnal de "done": cand explore_lite nu mai are frontiere,
        # publica un MarkerArray cu 0 markere active (toate sunt DELETE).
        self.create_subscription(
            MarkerArray, "/explore/frontiers", self._on_frontiers_viz, 10
        )

        # Service client pentru salvare
        from slam_toolbox.srv import SerializePoseGraph
        self._srv_type = SerializePoseGraph
        self.save_cli = self.create_client(
            SerializePoseGraph, "/slam_toolbox/serialize_map"
        )

        # Timer de monitorizare
        self.create_timer(2.0, self._tick)

        self.get_logger().info(
            f"[first_scan] watcher pornit | map={self.map_save_path} | "
            f"no_frontiers_to_done={self.no_front_s}s | "
            f"idle_to_done={self.done_idle_s}s | max={self.max_explore_s}s"
        )

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _on_resume(self, msg: Bool):
        # Cand explore_lite termina, in unele versiuni publica /explore/resume = False
        # (daca return_to_init=True nu se face niciodata, deci nu ne bazam doar pe asta)
        if not msg.data and self._explore_resumed:
            self.get_logger().info("[first_scan] /explore/resume=False → posibil done")
            self._explore_resumed = False

    def _on_frontier_goal(self, msg: PoseStamped):
        self._last_goal_t = time.time()

    def _on_frontiers_viz(self, msg: MarkerArray):
        """Detectie 'done' prin marker array publicat de explore_lite.

        explore_lite publica /explore/frontiers ca MarkerArray:
        - Daca exista frontiere → markere cu action=ADD (=0)
        - Cand nu mai exista frontiere → array gol SAU markere DELETE (=2)

        Consideram "frontiere active" = numar de markere cu action != DELETE
        si scale > 0 (ca sa filtram markere reziduale).
        """
        active = 0
        for m in msg.markers:
            if m.action == Marker.DELETE or m.action == Marker.DELETEALL:
                continue
            # Markere de tip POINTS de la explore_lite au points list cu
            # punctele frontierei. Daca lista e goala, nu e o frontiera reala.
            if m.type == Marker.POINTS:
                if len(m.points) > 0:
                    active += 1
            elif m.type == Marker.SPHERE:
                # markerul de centroid; daca scale > 0, e activ
                if m.scale.x > 0:
                    active += 1
            else:
                active += 1

        if active > 0:
            self._last_active_frontiers_t = time.time()

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        now = time.time()
        if self._last_pose is None:
            self._last_pose = (p.x, p.y, now)
            return
        lx, ly, lt = self._last_pose
        d = ((p.x - lx) ** 2 + (p.y - ly) ** 2) ** 0.5
        if d > 0.05:    # > 5cm miscare
            self._last_motion_t = now
            self._last_pose = (p.x, p.y, now)
        elif now - lt > 1.0:
            # actualizeaza referinta de pozitie periodic chiar fara miscare
            self._last_pose = (p.x, p.y, now)

    # ── Loop ──────────────────────────────────────────────────────────────

    def _tick(self):
        if self._saved:
            return

        now      = time.time()
        elapsed  = now - self._start_time
        idle_motion = (now - self._last_motion_t) if self._last_motion_t else elapsed
        idle_goal   = (now - self._last_goal_t)   if self._last_goal_t   else elapsed
        # idle_frontiers: cat timp a trecut de cand am vazut frontiere active.
        # None = niciodata am vazut, contam de la start (explore_lite poate
        # n-a pornit inca, sau a inceput cu zero frontiere).
        idle_frontiers = (now - self._last_active_frontiers_t) \
            if self._last_active_frontiers_t else elapsed

        self.get_logger().info(
            f"[first_scan] tick elapsed={elapsed:.0f}s "
            f"idle_frontiers={idle_frontiers:.0f}s "
            f"idle_motion={idle_motion:.0f}s idle_goal={idle_goal:.0f}s"
        )

        # Conditie 1: timeout global (safety net)
        if elapsed >= self.max_explore_s:
            self.get_logger().warn(
                f"[first_scan] TIMEOUT global ({elapsed:.0f}s). Salvez si ies."
            )
            self._do_save_and_exit()
            return

        if elapsed < self.min_explore_s:
            return

        # Conditie 2 (PRIMARA): nu mai exista frontiere active de explorat.
        # Daca am vazut macar o data frontiere active si de atunci au fost
        # ZERO frontiere pentru no_frontiers_seconds → exploration done.
        if self._last_active_frontiers_t is not None and \
           idle_frontiers >= self.no_front_s:
            self.get_logger().info(
                f"[first_scan] DONE: zero frontiere active timp de "
                f"{idle_frontiers:.0f}s. Harta completa. Salvez."
            )
            self._do_save_and_exit()
            return

        # Conditie 3: explore_lite a semnalat done explicit
        if not self._explore_resumed:
            self.get_logger().info("[first_scan] explore_lite a semnalat done. Salvez.")
            self._do_save_and_exit()
            return

        # Conditie 4 (FALLBACK): robotul blocat fizic — fara miscare SI fara
        # goaluri pentru done_idle_s. Acopera cazuri patologice cand
        # explore_lite e activ dar nu poate trimite goaluri valide.
        if idle_motion >= self.done_idle_s and idle_goal >= self.done_idle_s:
            self.get_logger().warn(
                f"[first_scan] FALLBACK: robot blocat fara miscare/goaluri "
                f"{self.done_idle_s:.0f}s. Salvez."
            )
            self._do_save_and_exit()

    # ── Save & exit ───────────────────────────────────────────────────────

    def _do_save_and_exit(self):
        # Atomic flag — previne reintrare daca _tick e apelat in timp ce
        # request-ul e in zbor.
        if self._saved:
            return
        self._saved = True

        if not self.save_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                "/slam_toolbox/serialize_map indisponibil — nu pot salva"
            )
            rclpy.shutdown()
            return

        req = self._srv_type.Request()
        req.filename = self.map_save_path
        future = self.save_cli.call_async(req)

        # NU folosim spin_until_future_complete (suntem deja in spin).
        # Atasam un callback care va fi invocat cand raspunsul ajunge.
        # Setam si un timer de siguranta — daca raspunsul nu vine, iesim oricum.
        future.add_done_callback(self._on_save_done)
        self.get_logger().info(
            f"[first_scan] Cerere serialize_map trimisa → {self.map_save_path}"
        )
        # Fallback: forteaza shutdown dupa 15s indiferent de raspuns
        self._save_timeout_timer = self.create_timer(15.0, self._save_timeout)

    def _on_save_done(self, future):
        try:
            r = future.result()
            ok = (r is not None and getattr(r, "result", -1) == 0)
            if ok:
                self.get_logger().info(
                    f"[first_scan] HARTA SALVATA: {self.map_save_path}.posegraph + .data"
                )
            else:
                self.get_logger().error(f"[first_scan] save raspuns: {r}")
        except Exception as e:
            self.get_logger().error(f"[first_scan] callback save eroare: {e}")
        try:
            rclpy.shutdown()
        except Exception:
            pass

    def _save_timeout(self):
        self.get_logger().warn(
            "[first_scan] timeout 15s pentru save → ies oricum"
        )
        try:
            rclpy.shutdown()
        except Exception:
            pass


def main():
    rclpy.init()
    try:
        n = SaveAndExit()
    except Exception:
        rclpy.shutdown()
        raise
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            n.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
