#!/usr/bin/env python3
"""
map_uploader — trimite harta SLAM la serverul de delivery pentru afisare in app.

Aboneaza la /map (nav_msgs/OccupancyGrid), o transforma in PNG grayscale si o
POST-eaza la `POST {server_url}/api/robot/map` cu metadata in WORLD frame, exact
formatul asteptat de RobotController.java + LiveStoreMap.tsx:

  { resolution, width, height, originX, originY, timestamp, image(base64 PNG) }

Conventii (validate pe proiect):
  - Frame: map = world + (0, 11.2)  →  world = map − 11.2 pe Y, yaw 0.
    Deci originY_world = grid.origin.y + world_offset_y (world_offset_y = −11.2).
  - Imaginea e FLIP-uita pe Y (np.flipud): OccupancyGrid are randul 0 jos (Y min),
    PNG are randul 0 sus. App-ul face row = height − (wy−originY)/resolution, deci
    asteapta randul 0 = Y max. flipud potriveste.

Encoder PNG pur-python (doar zlib din stdlib) → fara dependenta de cv2/PIL.
HTTP prin urllib (ca delivery_manager) → fara dependenta de `requests`.
"""
import base64
import json
import struct
import threading
import urllib.request
import zlib

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid


def encode_png_gray(arr: np.ndarray) -> bytes:
    """Encodeaza un array HxW uint8 ca PNG grayscale (8-bit). Doar zlib."""
    h, w = arr.shape
    raw = bytearray()
    for row in arr:
        raw.append(0)              # filtru "None" pe fiecare rand
        raw.extend(row.tobytes())

    def chunk(typ: bytes, data: bytes) -> bytes:
        crc = zlib.crc32(typ + data) & 0xFFFFFFFF
        return struct.pack(">I", len(data)) + typ + data + struct.pack(">I", crc)

    sig  = b"\x89PNG\r\n\x1a\n"
    ihdr = struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0)   # 8-bit, color type 0 = grayscale
    idat = zlib.compress(bytes(raw), 6)
    return sig + chunk(b"IHDR", ihdr) + chunk(b"IDAT", idat) + chunk(b"IEND", b"")


class MapUploader(Node):
    def __init__(self):
        super().__init__("map_uploader")

        self.declare_parameter("map_topic",            "/map")
        self.declare_parameter("server_url",           "http://localhost:8080")
        self.declare_parameter("upload_path",          "/api/robot/map")
        self.declare_parameter("world_offset_x",       0.0)
        self.declare_parameter("world_offset_y",      -11.2)   # world = map + offset
        self.declare_parameter("min_upload_interval",  3.0)    # [s] anti-spam
        self.declare_parameter("occupied_threshold",   50)     # >= → perete (negru)

        self.map_topic   = str(self.get_parameter("map_topic").value)
        self.server_url  = str(self.get_parameter("server_url").value).rstrip("/")
        self.upload_path = str(self.get_parameter("upload_path").value)
        self.off_x       = float(self.get_parameter("world_offset_x").value)
        self.off_y       = float(self.get_parameter("world_offset_y").value)
        self.min_interval = float(self.get_parameter("min_upload_interval").value)
        self.occ_thresh  = int(self.get_parameter("occupied_threshold").value)

        self._last_upload = -1e9
        self._busy = False

        # /map e publicat latched (transient_local) de slam_toolbox.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(OccupancyGrid, self.map_topic, self.on_map, qos)

        self.get_logger().info(
            f"map_uploader: {self.map_topic} -> {self.server_url}{self.upload_path} | "
            f"world_offset=({self.off_x}, {self.off_y}) min_interval={self.min_interval}s")

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_map(self, msg: OccupancyGrid):
        now = self.now()
        if self._busy or (now - self._last_upload) < self.min_interval:
            return
        self._last_upload = now

        w = msg.info.width
        h = msg.info.height
        if w == 0 or h == 0:
            return

        data = np.array(msg.data, dtype=np.int16).reshape(h, w)

        # OccupancyGrid → grayscale: necunoscut=gri, liber=alb, ocupat=negru.
        img = np.full((h, w), 205, dtype=np.uint8)        # -1 unknown
        img[data == 0] = 254                              # liber
        occ = (data >= self.occ_thresh)
        img[occ] = 0                                      # perete
        # zona partial ocupata (1..thresh) → gradient gri-inchis
        mid = (data > 0) & (data < self.occ_thresh)
        img[mid] = (254 - (data[mid].astype(np.float32) / self.occ_thresh) * 254).astype(np.uint8)

        # PNG: randul 0 = sus = Y max → flip pe verticala fata de OccupancyGrid.
        img = np.flipud(img)
        png = encode_png_gray(img)

        payload = {
            "resolution": float(msg.info.resolution),
            "width":      int(w),
            "height":     int(h),
            "originX":    float(msg.info.origin.position.x + self.off_x),
            "originY":    float(msg.info.origin.position.y + self.off_y),
            "timestamp":  int(now),
            "image":      base64.b64encode(png).decode("ascii"),
        }

        self._busy = True
        threading.Thread(target=self._post, args=(payload, len(png)), daemon=True).start()

    def _post(self, payload: dict, n_bytes: int):
        try:
            req = urllib.request.Request(
                self.server_url + self.upload_path,
                data=json.dumps(payload).encode("utf-8"),
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            with urllib.request.urlopen(req, timeout=5.0) as resp:
                resp.read()
            self.get_logger().info(
                f"harta urcata: {payload['width']}x{payload['height']} "
                f"({n_bytes} B PNG) origin=({payload['originX']:.2f},{payload['originY']:.2f})")
        except Exception as e:
            self.get_logger().warn(f"upload esuat ({self.server_url}{self.upload_path}): {e}")
        finally:
            self._busy = False


def main():
    rclpy.init()
    node = MapUploader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
