#!/usr/bin/env python3
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image

try:
    import cv2  # type: ignore
except Exception:
    cv2 = None

try:
    from cv_bridge import CvBridge  # type: ignore
except Exception:
    CvBridge = None

try:
    from ultralytics import YOLO  # type: ignore
except Exception:
    YOLO = None


@dataclass
class Target:
    x_norm: float
    y_norm: float
    h_norm: float
    conf: float


class YoloPersonTrackerNode(Node):
    """
    Runs YOLO on camera images, selects the best person (class=0) and publishes
    a PointStamped:
      point.x = x_norm ([-1,1])
      point.y = h_norm ([0,1])  (proxy for distance)
      point.z = confidence

    FIX: cand nu se detecteaza nimeni, publica ULTIMA pozitie valida pentru
         maximum `last_seen_timeout` secunde, in loc de h=0.
         Dupa timeout, publica h=0 (semnal de pierdere).
    """

    def __init__(self):
        super().__init__("yolo_person_tracker")

        self.declare_parameter("image_topic",         "/camera/rgb/image_raw")
        self.declare_parameter("model",               "yolov8x.pt")
        self.declare_parameter("conf",                0.25)
        self.declare_parameter("iou",                 0.5)
        self.declare_parameter("device",              "")
        self.declare_parameter("target_topic",        "/yolo_person_target")
        self.declare_parameter("publish_heartbeat",   True)
        self.declare_parameter("force_predict",       True)
        self.declare_parameter("log_every_n",         30)
        self.declare_parameter("publish_debug_image", False)
        self.declare_parameter("debug_image_topic",   "/yolo_person_debug")

        # [NOU] Cat timp sa tinem minte ultima pozitie dupa pierdere (secunde)
        # In acest interval, robotul primeste inca pozitia veche → poate cauta.
        # Dupa expirare, publica h=0 → human_follower intra in SEARCH.
        self.declare_parameter("last_seen_timeout",   3.0)   # 1.5→3.0: republica ultima pozitie mai mult, pod peste clipirile YOLO

        if YOLO is None:
            self.get_logger().error("ultralytics not found. pip install ultralytics")
            raise RuntimeError("Missing dependency: ultralytics")
        if CvBridge is None:
            self.get_logger().error("cv_bridge not found.")
            raise RuntimeError("Missing dependency: cv_bridge")

        self.bridge = CvBridge()

        model_path = self.get_parameter("model").value
        self.model  = YOLO(model_path)

        self.conf               = float(self.get_parameter("conf").value)
        self.iou                = float(self.get_parameter("iou").value)
        self.device             = str(self.get_parameter("device").value) or None
        self.publish_heartbeat  = bool(self.get_parameter("publish_heartbeat").value)
        self.force_predict      = bool(self.get_parameter("force_predict").value)
        self.log_every_n        = int(self.get_parameter("log_every_n").value)
        self.last_seen_timeout  = float(self.get_parameter("last_seen_timeout").value)
        self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)
        self.debug_image_topic  = str(self.get_parameter("debug_image_topic").value)

        self._frame_i = 0

        # [NOU] Memorie ultima detectie valida
        self._last_valid_target: Optional[Target] = None
        self._last_valid_time:   Optional[float]  = None
        import time
        self._time = time  # referinta la modul time

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
        )
        image_topic  = self.get_parameter("image_topic").value
        target_topic = self.get_parameter("target_topic").value

        self.pub = self.create_publisher(PointStamped, target_topic, 10)
        self.sub = self.create_subscription(Image, image_topic, self.on_image, qos)

        # Publisher imagine debug (cu bbox-uri desenate)
        self.dbg_pub = None
        if self.publish_debug_image:
            if cv2 is None:
                self.get_logger().warn(
                    "publish_debug_image=true dar OpenCV (cv2) nu e disponibil → dezactivat"
                )
                self.publish_debug_image = False
            else:
                self.dbg_pub = self.create_publisher(Image, self.debug_image_topic, 5)
                self.get_logger().info(f"YOLO debug image -> {self.debug_image_topic}")

        self.get_logger().info(
            f"YOLO -> {image_topic}  pub -> {target_topic} | "
            f"model={model_path} conf={self.conf} iou={self.iou} "
            f"force_predict={self.force_predict} "
            f"last_seen_timeout={self.last_seen_timeout}s"
        )

    # ── Publish ──────────────────────────────────────────────────────────

    def _publish(self, header, target: Optional[Target], *, stale: bool = False):
        """
        Publica pozitia.
        stale=True inseamna ca e ultima pozitie memorata, nu una proaspata.
        """
        out = PointStamped()
        out.header = header

        if target is None:
            out.point.x = 0.0
            out.point.y = 0.0
            out.point.z = 0.0
        else:
            out.point.x = float(np.clip(target.x_norm, -1.0, 1.0))
            out.point.y = float(np.clip(target.h_norm,  0.0, 1.0))
            out.point.z = float(np.clip(target.conf,    0.0, 1.0))

        self.pub.publish(out)

        if self.log_every_n > 0 and (self._frame_i % self.log_every_n == 0):
            tag = " [STALE]" if stale else ""
            self.get_logger().info(
                f"target{tag}: conf={out.point.z:.2f}, "
                f"err_x={out.point.x:.2f}, h={out.point.y:.2f}"
            )

    # ── Main callback ─────────────────────────────────────────────────────

    def on_image(self, msg: Image):
        self._frame_i += 1
        now = self._time.time()

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge failed: {e}")
            self._publish_fallback(msg.header, now)
            return

        # ── Run YOLO ─────────────────────────────────────────────────────
        try:
            if self.force_predict:
                results = self.model.predict(
                    frame, conf=self.conf, iou=self.iou,
                    device=self.device, verbose=False, classes=[0],
                )
            else:
                results = self.model.track(
                    frame, conf=self.conf, iou=self.iou,
                    device=self.device, persist=True,
                    verbose=False, classes=[0],
                )
        except Exception as e:
            self.get_logger().error(f"YOLO inference failed: {e}")
            self._publish_fallback(msg.header, now)
            self._publish_debug(msg.header, frame, [], None)
            return

        if not results:
            self._publish_fallback(msg.header, now)
            self._publish_debug(msg.header, frame, [], None)
            return

        r = results[0]
        if r.boxes is None or len(r.boxes) == 0:
            self._publish_fallback(msg.header, now)
            self._publish_debug(msg.header, frame, [], None)
            return

        h, w = frame.shape[:2]

        # ── Selecteaza cel mai bun om + colecteaza toate boxurile pentru debug
        best: Optional[Target] = None
        all_boxes: List[Tuple[int, int, int, int, float]] = []
        best_box: Optional[Tuple[int, int, int, int, float]] = None
        for b in r.boxes:
            try:
                conf = float(b.conf.item())
                xyxy = b.xyxy[0].cpu().numpy().tolist()
            except Exception:
                continue

            x1, y1, x2, y2 = xyxy
            bx = (x1 + x2) / 2.0
            bh = max(1.0, (y2 - y1))

            x_norm = ((bx / w) - 0.5) * 2.0
            y_norm = (((y1 + y2) / 2.0 / h) - 0.5) * 2.0
            h_norm = float(np.clip(bh / h, 0.0, 1.0))

            box_int = (int(x1), int(y1), int(x2), int(y2), conf)
            all_boxes.append(box_int)

            t = Target(
                x_norm=float(x_norm),
                y_norm=float(y_norm),
                h_norm=float(h_norm),
                conf=float(conf),
            )
            if best is None or t.conf > best.conf:
                best = t
                best_box = box_int

        if best is None:
            self._publish_fallback(msg.header, now)
            self._publish_debug(msg.header, frame, all_boxes, None)
            return

        # ── Detectie reusita — actualizeaza memoria ───────────────────────
        self._last_valid_target = best
        self._last_valid_time   = now
        self._publish(msg.header, best, stale=False)
        self._publish_debug(msg.header, frame, all_boxes, best_box)

    # ── Publica imaginea debug cu bounding boxes ──────────────────────────

    def _publish_debug(self, header, frame, all_boxes, best_box):
        """
        Deseneaza bounding boxes peste imagine si publica pe debug_image_topic.
        - boxuri gri pentru toate detectiile
        - box verde si gros pentru cel ales (target)
        """
        if not self.publish_debug_image or self.dbg_pub is None or cv2 is None:
            return
        try:
            dbg = frame.copy()
            h, w = dbg.shape[:2]

            # Toate boxurile - gri subtire
            for (x1, y1, x2, y2, conf) in all_boxes:
                if best_box is not None and (x1, y1, x2, y2) == best_box[:4]:
                    continue  # desenam targetul separat, mai jos
                cv2.rectangle(dbg, (x1, y1), (x2, y2), (180, 180, 180), 1)
                cv2.putText(
                    dbg, f"{conf:.2f}", (x1, max(0, y1 - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1, cv2.LINE_AA
                )

            # Target - verde, gros + cruce centrala
            if best_box is not None:
                x1, y1, x2, y2, conf = best_box
                cv2.rectangle(dbg, (x1, y1), (x2, y2), (0, 255, 0), 3)
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2
                cv2.drawMarker(dbg, (cx, cy), (0, 255, 0),
                               markerType=cv2.MARKER_CROSS,
                               markerSize=18, thickness=2)
                cv2.putText(
                    dbg, f"TARGET {conf:.2f}", (x1, max(0, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA
                )

            # Linia verticala centrala (alinierea robotului)
            cv2.line(dbg, (w // 2, 0), (w // 2, h), (255, 255, 0), 1)

            # Numar detectii + status
            status = f"persons={len(all_boxes)}"
            if best_box is None:
                status += "  [NO TARGET]"
            cv2.putText(
                dbg, status, (8, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2, cv2.LINE_AA
            )

            out = self.bridge.cv2_to_imgmsg(dbg, encoding="bgr8")
            out.header = header
            self.dbg_pub.publish(out)
        except Exception as e:
            self.get_logger().warn(f"debug image publish failed: {e}")

    # ── Fallback cu ultima pozitie valida ─────────────────────────────────

    def _publish_fallback(self, header, now: float):
        """
        Daca avem o pozitie recenta (< last_seen_timeout secunde), o retrimitem.
        Altfel, publicam h=0 (semnal pierdut).
        """
        if (self._last_valid_target is not None and
                self._last_valid_time is not None and
                (now - self._last_valid_time) < self.last_seen_timeout):
            # Retrimitem ultima pozitie cunoscuta cu conf scazut progresiv
            age       = now - self._last_valid_time
            fade      = max(0.05, 1.0 - age / self.last_seen_timeout)
            stale_tgt = Target(
                x_norm=self._last_valid_target.x_norm,
                y_norm=self._last_valid_target.y_norm,
                h_norm=self._last_valid_target.h_norm,
                conf=self._last_valid_target.conf * fade,
            )
            self._publish(header, stale_tgt, stale=True)
        else:
            # Pozitia a expirat → semnal pierdut
            if self.publish_heartbeat:
                self._publish(header, None, stale=False)


def main():
    rclpy.init()
    try:
        node = YoloPersonTrackerNode()
    except Exception:
        rclpy.shutdown()
        raise
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()