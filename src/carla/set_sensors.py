#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CARLA camera/LiDAR/IMU/GNSS viewer & sensor-pose editor (terminal-driven controls)
- OpenCV windows only display frames (cv2.waitKey(1) keeps them responsive).
- Controls are typed in the terminal and confirmed with Enter (e.g., "wwwaajl", "P", "Q").
- Lowercase keys = movement/rotation; UPPERCASE keys = actions (S/R/P/Q). 't' or 'T' toggles target.
"""

import carla
import cv2
import numpy as np
import time
import random
import os
import sys
import json
import datetime
import select
from glob import glob

# ================= user config =================
SERVER_HOST = "192.168.86.78"   # or 'localhost'
SERVER_PORT = 2000
TOWN = "Town03"
VEHICLE_BP_ID = "vehicle.carlamotors.european_hgv"

# Camera
IMG_W, IMG_H = 1280, 720
FOV = 90

# LiDAR
LIDAR_CHANNELS   = 32
LIDAR_PPS        = 600_000
LIDAR_ROT_HZ     = 10
LIDAR_RANGE      = 80.0
LIDAR_UP_FOV     = 10.0
LIDAR_LOW_FOV    = -30.0

# LiDAR BEV preview
BEV_SIZE   = 800
BEV_METERS = 80.0

# Save dir (absolute path)
SAVE_DIR = os.path.abspath("sensor_poses")
os.makedirs(SAVE_DIR, exist_ok=True)

# Step sizes
STEP_POS = 0.10   # m
STEP_ROT = 2.0    # deg
# =================================================


# ---------------- function utils ----------------
def to_bgr(image: carla.Image) -> np.ndarray:
    arr = np.frombuffer(image.raw_data, dtype=np.uint8).reshape((image.height, image.width, 4))
    return cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)

def safe_set_attr(bp: carla.ActorBlueprint, name: str, value):
    try:
        if bp.has_attribute(name):
            bp.set_attribute(name, str(value))
    except RuntimeError:
        pass

def clamp_angle(a: float) -> float:
    while a > 180.0: a -= 360.0
    while a < -180.0: a += 360.0
    return a

def tf_to_dict(tf: carla.Transform) -> dict:
    return {
        "location": {"x": tf.location.x, "y": tf.location.y, "z": tf.location.z},
        "rotation": {"yaw": tf.rotation.yaw, "pitch": tf.rotation.pitch, "roll": tf.rotation.roll},
    }

def dict_to_tf(d: dict) -> carla.Transform:
    loc = d["location"]; rot = d["rotation"]
    return carla.Transform(
        carla.Location(x=loc["x"], y=loc["y"], z=loc["z"]),
        carla.Rotation(yaw=rot["yaw"], pitch=rot["pitch"], roll=rot["roll"])
    )

def bev_from_lidar(points_xyz: np.ndarray) -> np.ndarray:
    bev = np.zeros((BEV_SIZE, BEV_SIZE), dtype=np.uint8)
    half = BEV_SIZE // 2
    res = (BEV_METERS / 2) / half  # m/px

    x = points_xyz[:, 0]
    y = points_xyz[:, 1]
    m = (np.abs(x) < BEV_METERS/2) & (np.abs(y) < BEV_METERS/2)
    u = ((x[m] / res) + half).astype(np.int32)
    v = ((y[m] / res) + half).astype(np.int32)

    valid = (u >= 0) & (u < BEV_SIZE) & (v >= 0) & (v < BEV_SIZE)
    bev[u[valid], v[valid]] = 255

    bev = cv2.rotate(bev, cv2.ROTATE_90_COUNTERCLOCKWISE)
    bev = cv2.flip(bev, 0)
    bev_bgr = cv2.cvtColor(bev, cv2.COLOR_GRAY2BGR)
    cv2.putText(bev_bgr, "LiDAR BEV (±40m)", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
    return bev_bgr

def put_hud(img: np.ndarray, rel_tf: carla.Transform, mode: str, imu_last, gnss_last):
    loc, rot = rel_tf.location, rel_tf.rotation
    lines = [
        f"[T/t] target: {mode.upper()}",
        f"[w/s,a/d,z/x] xyz: ({loc.x:+.2f}, {loc.y:+.2f}, {loc.z:+.2f}) m",
        f"[j/l,i/k,u/o] rpy: (yaw {rot.yaw:+.1f}, pitch {rot.pitch:+.1f}, roll {rot.roll:+.1f}) deg",
        "[S] save poses  [R] reload poses  [0] reset  [P] save frame  [Q] quit",
        "(type in terminal; e.g., wwwaajlP)",
    ]
    if imu_last is not None:
        ax, ay, az, gx, gy, gz, comp = imu_last
        lines.append(
            f"IMU acc=({ax:+.2f},{ay:+.2f},{az:+.2f}) m/s^2  "
            f"gyro=({gx:+.2f},{gy:+.2f},{gz:+.2f}) rad/s  compass={comp:+.2f}"
        )
    if gnss_last is not None:
        lat, lon, alt = gnss_last
        lines.append(f"GNSS lat={lat:.7f}, lon={lon:.7f}, alt={alt:.2f} m")
    y = 28
    for line in lines:
        cv2.putText(img, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2, cv2.LINE_AA)
        y += 28

def stdin_readline_nonblocking():
    """Return one line from stdin (case preserved) if available, else None."""
    r, _, _ = select.select([sys.stdin], [], [], 0)
    if r:
        return sys.stdin.readline().strip()
    return None

def print_controls_help():
    print(r"""
=== Controls ===
T/t : toggle target (camera → lidar → imu → gnss → ...)
w/s,a/d,z/x : translate x/y/z
j/l,i/k,u/o : rotate yaw/pitch/roll
0    : reset current target to defaults
S    : save poses to JSON
R    : reload last saved JSON
P    : save camera frame
Q    : quit
(You can enter multiple keys in one line, e.g., "wwwaajlP")
""")


# ---------------- OOP main app ----------------
class CarlaSensorApp:
    def __init__(self, server_host, server_port, town, vehicle_bp):
        self.server_host = server_host
        self.server_port = server_port
        self.town = town
        self.vehicle_bp_id = vehicle_bp

        self.client = None
        self.world = None
        self.original_settings = None

        self.vehicle = None
        self.camera = None
        self.lidar = None
        self.imu = None
        self.gnss = None

        # Default relative poses (attach_to=vehicle)
        # Camera: x=3.6, z=3.0
        self.rel_tf_cam   = carla.Transform(
            carla.Location(x=3.6, y=0.0, z=3.0),
            carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0)
        )
        # LiDAR: x=3.6, z=3.9
        self.rel_tf_lidar = carla.Transform(
            carla.Location(x=3.6, y=0.0, z=3.9),
            carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0)
        )
        # IMU/GNSS on roof-ish by default
        self.rel_tf_imu   = carla.Transform(carla.Location(x=0.0, y=0.0, z=1.2))
        self.rel_tf_gnss  = carla.Transform(carla.Location(x=0.0, y=0.0, z=1.8))

        # Last data buffers
        self.last_cam_img = None
        self.last_cam_frame_id = -1
        self.last_bev_img = None
        self.imu_last = None       # (ax, ay, az, gx, gy, gz, compass)
        self.gnss_last = None      # (lat, lon, alt)

        # Control
        self.targets = ["camera", "lidar", "imu", "gnss"]
        self.control_target = "camera"
        self.last_pose_path = None
        self.last_save_time = 0.0

    # ---- connect/load ----
    def connect(self):
        self.client = carla.Client(self.server_host, self.server_port)
        self.client.set_timeout(10.0)
        preload = self._pick_pose_file_for_start()
        if preload:
            with open(preload) as f:
                data = json.load(f)
            map_to_load = data.get("town", self.town)
            if map_to_load != self.town:
                print(f"[INFO] switching town to '{map_to_load}' based on JSON")
                self.town = map_to_load 
        self.world = self.client.load_world(self.town)
        print("Loaded map:", self.world.get_map().name)
        self.original_settings = self.world.get_settings()

        # Apply poses before spawning sensors (if available)
        if preload:
            self._apply_poses_from_file(preload)
            self.last_pose_path = preload
            print(f"[INFO] initial poses loaded from {preload}")

    def spawn_vehicle(self):
        bp_lib = self.world.get_blueprint_library()
        veh_bp = bp_lib.find(self.vehicle_bp_id)
        spawn_point = random.choice(self.world.get_map().get_spawn_points())
        self.vehicle = self.world.spawn_actor(veh_bp, spawn_point)
        print("Spawned vehicle:", self.vehicle.type_id)

    def spawn_sensors(self):
        bp_lib = self.world.get_blueprint_library()

        # Camera
        cam_bp = bp_lib.find("sensor.camera.rgb")
        for k, v in [("image_size_x", IMG_W), ("image_size_y", IMG_H),
                     ("fov", FOV), ("sensor_tick", 0.0)]:
            safe_set_attr(cam_bp, k, v)
        self.camera = self.world.spawn_actor(cam_bp, self.rel_tf_cam, attach_to=self.vehicle)
        print("Camera attached:", self.camera.type_id)
        self.camera.listen(self._on_camera)

        # LiDAR
        lidar_bp = bp_lib.find("sensor.lidar.ray_cast")
        for k, v in [
            ("channels", LIDAR_CHANNELS),
            ("points_per_second", LIDAR_PPS),
            ("rotation_frequency", LIDAR_ROT_HZ),
            ("range", LIDAR_RANGE),
            ("upper_fov", LIDAR_UP_FOV),
            ("lower_fov", LIDAR_LOW_FOV),
            ("noise_stddev", 0.0),
            ("dropoff_general_rate", 0.0),
            ("dropoff_intensity_limit", 1.0),
            ("dropoff_zero_intensity", 0.0),
            ("sensor_tick", 0.0),
        ]:
            safe_set_attr(lidar_bp, k, v)
        self.lidar = self.world.spawn_actor(lidar_bp, self.rel_tf_lidar, attach_to=self.vehicle)
        print("LiDAR attached:", self.lidar.type_id)
        self.lidar.listen(self._on_lidar)

        # IMU
        imu_bp = bp_lib.find("sensor.other.imu")
        safe_set_attr(imu_bp, "sensor_tick", 0.02)  # 50 Hz
        self.imu = self.world.spawn_actor(imu_bp, self.rel_tf_imu, attach_to=self.vehicle)
        print("IMU attached:", self.imu.type_id)
        self.imu.listen(self._on_imu)

        # GNSS
        gnss_bp = bp_lib.find("sensor.other.gnss")
        safe_set_attr(gnss_bp, "sensor_tick", 0.20)  # 5 Hz
        self.gnss = self.world.spawn_actor(gnss_bp, self.rel_tf_gnss, attach_to=self.vehicle)
        print("GNSS attached:", self.gnss.type_id)
        self.gnss.listen(self._on_gnss)

        # spectator (top-down)
        spec = self.world.get_spectator()
        spec.set_transform(carla.Transform(
            location=self.vehicle.get_transform().location + carla.Location(z=35),
            rotation=carla.Rotation(pitch=-90)
        ))

    # ---- callbacks ----
    def _on_camera(self, image: carla.Image):
        self.last_cam_img = to_bgr(image)
        self.last_cam_frame_id = image.frame

    def _on_lidar(self, meas: carla.LidarMeasurement):
        pts = np.frombuffer(meas.raw_data, dtype=np.float32).reshape(-1, 4)[:, :3]
        self.last_bev_img = bev_from_lidar(pts)

    def _on_imu(self, m: carla.IMUMeasurement):
        self.imu_last = (m.accelerometer.x, m.accelerometer.y, m.accelerometer.z,
                         m.gyroscope.x, m.gyroscope.y, m.gyroscope.z,
                         m.compass)

    def _on_gnss(self, m: carla.GnssMeasurement):
        self.gnss_last = (m.latitude, m.longitude, m.altitude)

    # ---- display/input ---- (창 띄우기 및 키보드 입력 받기)
    def setup_windows(self):
        cv2.namedWindow("CARLA Camera", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("CARLA Camera", 960, 540)
        cv2.namedWindow("LiDAR BEV", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("LiDAR BEV", 600, 600)
        print_controls_help()

    def update_display(self):
        if self.last_cam_img is not None:
            if self.control_target == "camera":
                tf = self.rel_tf_cam
            elif self.control_target == "lidar":
                tf = self.rel_tf_lidar
            elif self.control_target == "imu":
                tf = self.rel_tf_imu
            else:
                tf = self.rel_tf_gnss
            hud = self.last_cam_img.copy()
            put_hud(hud, tf, self.control_target, self.imu_last, self.gnss_last)
            cv2.imshow("CARLA Camera", hud)
        if self.last_bev_img is not None:
            cv2.imshow("LiDAR BEV", self.last_bev_img)
        cv2.waitKey(1)

    def _apply_current_transform(self):
        if self.control_target == "camera" and self.camera:
            self.camera.set_transform(self.rel_tf_cam)
        elif self.control_target == "lidar" and self.lidar:
            self.lidar.set_transform(self.rel_tf_lidar)
        elif self.control_target == "imu" and self.imu:
            self.imu.set_transform(self.rel_tf_imu)
        elif self.control_target == "gnss" and self.gnss:
            self.gnss.set_transform(self.rel_tf_gnss)

    def process_input_line(self, line: str) -> bool:
        for key in line:
            # toggle + actions
            if key in ('T', 't'):
                i = self.targets.index(self.control_target)
                self.control_target = self.targets[(i + 1) % len(self.targets)]
                print("Target:", self.control_target.upper())
                continue
            elif key == 'S':
                self.save_poses();  continue
            elif key == 'R':
                self.load_last_poses();  continue
            elif key == 'P':
                self.save_camera_frame();  continue
            elif key == 'Q':
                print("Quitting...");  return False

            # select current transform
            if   self.control_target == "camera": cur_tf = self.rel_tf_cam
            elif self.control_target == "lidar":  cur_tf = self.rel_tf_lidar
            elif self.control_target == "imu":    cur_tf = self.rel_tf_imu
            else:                                  cur_tf = self.rel_tf_gnss

            # movement/rotation (lowercase)
            if   key == 'w': cur_tf.location.x += STEP_POS
            elif key == 's': cur_tf.location.x -= STEP_POS
            elif key == 'a': cur_tf.location.y -= STEP_POS
            elif key == 'd': cur_tf.location.y += STEP_POS
            elif key == 'z': cur_tf.location.z -= STEP_POS
            elif key == 'x': cur_tf.location.z += STEP_POS
            elif key == 'j': cur_tf.rotation.yaw   = clamp_angle(cur_tf.rotation.yaw   - STEP_ROT)
            elif key == 'l': cur_tf.rotation.yaw   = clamp_angle(cur_tf.rotation.yaw   + STEP_ROT)
            elif key == 'i': cur_tf.rotation.pitch = clamp_angle(cur_tf.rotation.pitch + STEP_ROT)
            elif key == 'k': cur_tf.rotation.pitch = clamp_angle(cur_tf.rotation.pitch - STEP_ROT)
            elif key == 'u': cur_tf.rotation.roll  = clamp_angle(cur_tf.rotation.roll  - STEP_ROT)
            elif key == 'o': cur_tf.rotation.roll  = clamp_angle(cur_tf.rotation.roll  + STEP_ROT)
            elif key == '0':
                if self.control_target == "camera":
                    self.rel_tf_cam.location = carla.Location(x=3.6, y=0.0, z=3.0)
                    self.rel_tf_cam.rotation = carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0)
                elif self.control_target == "lidar":
                    self.rel_tf_lidar.location = carla.Location(x=3.6, y=0.0, z=3.9)
                    self.rel_tf_lidar.rotation = carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0)
                elif self.control_target == "imu":
                    self.rel_tf_imu.location = carla.Location(x=0.0, y=0.0, z=1.2)
                    self.rel_tf_imu.rotation = carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0)
                else:
                    self.rel_tf_gnss.location = carla.Location(x=0.0, y=0.0, z=1.8)
                    self.rel_tf_gnss.rotation = carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0)
                print(f"{self.control_target} transform reset.")
            else:
                # ignore other keys
                pass

            # apply change
            self._apply_current_transform()

        return True

    # ---- save/load ----
    def save_poses(self):
        ts = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        path = os.path.join(SAVE_DIR, f"poses_{ts}.json")
        data = {
            "timestamp": ts,
            "town": self.town,
            "vehicle": self.vehicle_bp_id,
            "camera": tf_to_dict(self.rel_tf_cam),
            "lidar":  tf_to_dict(self.rel_tf_lidar),
            "imu":    tf_to_dict(self.rel_tf_imu),
            "gnss":   tf_to_dict(self.rel_tf_gnss),
            "note": "Transforms are relative to the vehicle (attach_to=vehicle). Units: meters, degrees."
        }
        try:
            with open(path, "w") as f:
                json.dump(data, f, indent=2)
            self.last_pose_path = path
            print("Saved sensor poses ->", path)
        except Exception as e:
            print(f"[ERROR] failed to save sensor poses: {e}")

    def load_last_poses(self):
        path = self.last_pose_path or self._find_latest_pose_file()
        if not path:
            print("No saved pose to load yet.");  return
        self._apply_poses_from_file(path)
        print("Loaded sensor poses <-", path)

        # apply immediately to sensors (if already spawned)
        if self.camera: self.camera.set_transform(self.rel_tf_cam)
        if self.lidar:  self.lidar.set_transform(self.rel_tf_lidar)
        if self.imu:    self.imu.set_transform(self.rel_tf_imu)
        if self.gnss:   self.gnss.set_transform(self.rel_tf_gnss)

    def _apply_poses_from_file(self, path: str):
        try:
            with open(path) as f:
                data = json.load(f)
            if "camera" in data: self.rel_tf_cam   = dict_to_tf(data["camera"])
            if "lidar"  in data: self.rel_tf_lidar = dict_to_tf(data["lidar"])
            if "imu"    in data: self.rel_tf_imu   = dict_to_tf(data["imu"])
            if "gnss"   in data: self.rel_tf_gnss  = dict_to_tf(data["gnss"])
            self.last_pose_path = path
        except Exception as e:
            print(f"[ERROR] failed to load sensor poses ({path}): {e}")

    def _find_latest_pose_file(self):
        files = sorted(glob(os.path.join(SAVE_DIR, "poses_*.json")),
                       key=os.path.getmtime, reverse=True)
        return files[0] if files else None

    def _pick_pose_file_for_start(self):
        """Choose a pose file at startup: env(POSES_JSON) > latest file > None."""
        env_path = os.environ.get("POSES_JSON")
        if env_path and os.path.isfile(env_path):
            return env_path
        return self._find_latest_pose_file()

    def save_camera_frame(self):
        now = time.time()
        if self.last_cam_img is not None and now - self.last_save_time > 0.5:
            fname = os.path.join(os.getcwd(), f"carla_cam_{int(now)}.png")
            try:
                cv2.imwrite(fname, self.last_cam_img)
                self.last_save_time = now
                print("Saved:", fname)
            except Exception as e:
                print(f"[ERROR] failed to save frame: {e}")

    # ---- main loop / cleanup ----
    def run(self):
        try:
            self.connect()
            self.spawn_vehicle()
            self.spawn_sensors()
            self.setup_windows()

            while True:
                self.update_display()
                line = stdin_readline_nonblocking()
                if line is None:
                    continue
                if not self.process_input_line(line):
                    break
        finally:
            self.cleanup()

    def cleanup(self):
        try:  self.camera and self.camera.stop()
        except: pass
        try:  self.lidar and self.lidar.stop()
        except: pass
        try:  self.imu and self.imu.stop()
        except: pass
        try:  self.gnss and self.gnss.stop()
        except: pass

        for actor in [self.camera, self.lidar, self.imu, self.gnss, self.vehicle]:
            try:
                actor and actor.destroy()
            except:
                pass

        try:
            self.world and self.original_settings and self.world.apply_settings(self.original_settings)
        except:
            pass

        try:
            cv2.destroyAllWindows()
        except:
            pass
        print("Cleaned up.")


# ---------------- entrypoint ----------------
def main():
    app = CarlaSensorApp(
        server_host=SERVER_HOST,
        server_port=SERVER_PORT,
        town=TOWN,
        vehicle_bp=VEHICLE_BP_ID,
    )
    app.run()

if __name__ == "__main__":
    main()
