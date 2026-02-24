# human_follower (ROS 2 Jazzy)

Pachet ROS2 (ament_python) care:
1) detectează oameni din LiDAR folosind **DR‑SPAAM** (wrapper ROS2),
2) detectează / urmărește persoane pe cameră cu **YOLO (Ultralytics)**,
3) publică `cmd_vel` ca să se ducă la om (APPROACH) și să îl urmărească (FOLLOW).

## Dependențe
- ROS 2 Jazzy: `rclpy`, `sensor_msgs`, `geometry_msgs`, `visualization_msgs`, `cv_bridge`
- Python:
  - DR‑SPAAM (proiectul original) instalat în același environment: modulul `dr_spaam`
  - Ultralytics YOLO: `pip install ultralytics`

> Note: fișierul de weights pentru DR‑SPAAM (`.pth`) nu era în zip-ul tău; îl pui tu și setezi parametru `weight_file`.

## Build
```bash
cd ~/ros2_ws/src
# copiază folderul human_follower aici
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Run
Editează `config/params.yaml` (topic-uri și căi către modele), apoi:
```bash
ros2 launch human_follower human_follower.launch.py
```

## Topic-uri
- Input:
  - `/scan` (`sensor_msgs/LaserScan`)
  - `/camera/image_raw` (`sensor_msgs/Image`)
- Output:
  - `/human_detections` (`geometry_msgs/PoseArray`) – (x,y) în frame-ul laserului
  - `/human_detections_marker` (`visualization_msgs/Marker`) – pentru RViz
  - `/yolo_person_target` (`geometry_msgs/PointStamped`) – x: eroare orizontală [-1..1], y: înălțime bbox [0..1], z: conf
  - `/cmd_vel` (`geometry_msgs/Twist`)

## Tuning rapid
- `human_follower.max_linear`, `max_angular`
- `human_follower.k_lin`, `k_ang`
- `human_follower.approach_distance`
- În FOLLOW: `h_set` (în `follower_node.py`) – dacă robotul stă prea departe / prea aproape, ajustează.

## Integrare cu ce ai în zip
În zip ai un pachet ROS1 `dr_spaam_ros`. Eu l-am **portat** în ROS2 sub `dr_spaam_detector_node.py`.
Dacă deja ai DR‑SPAAM instalat și weights, node-ul ar trebui să ruleze imediat.
