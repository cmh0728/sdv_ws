#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 이상하게 움직이긴하는데, 일단 컨트롤 되는거는 확인했음. 컨트롤 로직 이거 들어가면 됨 .
import os, json, random, math, time, curses
import carla
from glob import glob

SERVER_HOST = "192.168.86.78"
SERVER_PORT = 2000
TOWN = "Town05"  # 없으면 자동으로 Town03으로 대체
VEHICLE_BP_ID = "vehicle.carlamotors.european_hgv"  # 없으면 테슬라로 대체
SAVE_DIR = os.path.abspath("sensor_poses")
os.makedirs(SAVE_DIR, exist_ok=True)

FOLLOW_DISTANCE = float(os.environ.get("FOLLOW_DIST", "8.0"))   # 3인칭 카메라 뒤 거리(m)
FOLLOW_HEIGHT   = float(os.environ.get("FOLLOW_Z", "3.0"))      # 높이(m)
FOLLOW_PITCH    = float(os.environ.get("FOLLOW_PITCH", "-15.0"))# 아래로 보는 각도(°)
FOLLOW_ENABLE   = os.environ.get("FOLLOW", "1") == "1"          # 따라가기 on/off

def find_latest_pose_file():
    files = sorted(glob(os.path.join(SAVE_DIR, "poses_*.json")),
                   key=os.path.getmtime, reverse=True)
    return files[0] if files else None

def dict_to_tf(d: dict) -> carla.Transform:
    loc, rot = d["location"], d["rotation"]
    return carla.Transform(
        carla.Location(x=loc["x"], y=loc["y"], z=loc["z"]),
        carla.Rotation(yaw=rot["yaw"], pitch=rot["pitch"], roll=rot["roll"])
    )

def load_optional_poses(path, app):
    try:
        with open(path) as f:
            data = json.load(f)
        if "camera" in data: app.rel_tf_cam   = dict_to_tf(data["camera"])
        if "lidar"  in data: app.rel_tf_lidar = dict_to_tf(data["lidar"])
        if "imu"    in data: app.rel_tf_imu   = dict_to_tf(data["imu"])
        if "gnss"   in data: app.rel_tf_gnss  = dict_to_tf(data["gnss"])
        print(f"[INFO] Loaded poses from {path}")
        return data
    except Exception as e:
        print(f"[WARN] Pose load failed: {e}")
        return {}

class CarlaTeleopApp:
    def __init__(self, host, port, town, vehicle_bp):
        self.host, self.port = host, port
        self.town, self.vehicle_bp_id = town, vehicle_bp
        self.client = self.world = self.vehicle = None
        self.spectator = None
        # default relative sensor TFs
        self.rel_tf_cam   = carla.Transform(carla.Location(x=3.6, y=0.0, z=3.0))
        self.rel_tf_lidar = carla.Transform(carla.Location(x=3.6, y=0.0, z=3.9))
        self.rel_tf_imu   = carla.Transform(carla.Location(x=0.0, y=0.0, z=1.2))
        self.rel_tf_gnss  = carla.Transform(carla.Location(x=0.0, y=0.0, z=1.8))

    def connect(self):
        self.client = carla.Client(self.host, self.port)
        self.client.set_timeout(10.0)

        preload = find_latest_pose_file()
        if preload:
            data = load_optional_poses(preload, self)
            map_to_load = data.get("town", self.town)
            if map_to_load:
                self.town = map_to_load

        # 맵 로드 (없으면 Town03로 폴백)
        try:
            self.world = self.client.load_world(self.town)
        except RuntimeError:
            print(f"[WARN] Map '{self.town}' not found. Falling back to Town03.")
            self.world = self.client.load_world("Town03")
            self.town = "Town03"

        print("[INFO] Loaded map:", self.world.get_map().name)

        # 🔹 디버깅/공유 서버에선 비동기 모드가 안전
        s = self.world.get_settings()
        s.synchronous_mode = False         # 화면 멈춤 방지
        s.fixed_delta_seconds = None
        # s.no_rendering_mode = False       # 렌더링 건드리지 않기
        self.world.apply_settings(s)

        self.spectator = self.world.get_spectator()

    def spawn_vehicle(self):
        bp_lib = self.world.get_blueprint_library()
        try:
            veh_bp = bp_lib.find(self.vehicle_bp_id)
        except:
            print(f"[WARN] Blueprint '{self.vehicle_bp_id}' not found. Using Tesla Model 3.")
            veh_bp = bp_lib.find("vehicle.tesla.model3")

        spawn_points = self.world.get_map().get_spawn_points()
        random.shuffle(spawn_points)
        self.vehicle = None
        for sp in spawn_points[:10]:  # 몇 군데 시도
            self.vehicle = self.world.try_spawn_actor(veh_bp, sp)
            if self.vehicle: break
        if not self.vehicle:
            self.vehicle = self.world.spawn_actor(veh_bp, spawn_points[0])

        self.vehicle.set_autopilot(False)
        print(f"[INFO] Spawned vehicle: {self.vehicle.type_id}  (id={self.vehicle.id})")

        # 스폰 직후 한 번 카메라 위치 세팅
        if FOLLOW_ENABLE:
            self.update_spectator(force=True)

    def speed_kmh(self):
        v = self.vehicle.get_velocity()
        return (v.x*v.x + v.y*v.y + v.z*v.z) ** 0.5 * 3.6

    def update_spectator(self, force=False):
        if not (self.vehicle and self.spectator and FOLLOW_ENABLE):
            return
        tf = self.vehicle.get_transform()
        yaw = math.radians(tf.rotation.yaw)
        # 차량 뒤 FOLLOW_DISTANCE 만큼, 높이 FOLLOW_HEIGHT로 위치
        offset = carla.Location(x=-FOLLOW_DISTANCE*math.cos(yaw),
                                y=-FOLLOW_DISTANCE*math.sin(yaw),
                                z= FOLLOW_HEIGHT)
        cam_tf = carla.Transform(tf.location + offset,
                                 carla.Rotation(pitch=FOLLOW_PITCH, yaw=tf.rotation.yaw))
        # 너무 자주 set_transform하면 떨릴 수 있어 프레임 간격 조절
        if force or (int(time.time()*10) % 2 == 0):  # 0.5초마다 갱신
            self.spectator.set_transform(cam_tf)

        # 차량 위에 속도/ID 라벨 (0.2초 유지)
        self.world.debug.draw_string(
            tf.location + carla.Location(z=2.5),
            f"ID {self.vehicle.id} | {self.speed_kmh():.1f} km/h",
            life_time=0.2,
            color=carla.Color(0, 255, 0)
        )

    def teleop_loop_headless(self):
        # 터미널 토글 조종
        stdscr = curses.initscr()
        curses.noecho(); curses.cbreak()
        stdscr.nodelay(True); stdscr.keypad(True)

        throttle_on = False
        reverse_on = False
        steer = 0.0
        last_print = 0.0
        try:
            running = True
            while running:
                ch = stdscr.getch()  # -1이면 입력 없음
                if ch != -1:
                    if ch in (ord('q'), ord('Q')): running = False
                    elif ch in (ord('w'), ord('W')): throttle_on, reverse_on = True, False
                    elif ch in (ord('s'), ord('S')): throttle_on, reverse_on = True, True
                    elif ch in (ord('0'), ord('x'), ord('X')): throttle_on = False
                    elif ch == ord(' '): throttle_on = False  # 브레이크는 아래에서 처리
                    elif ch in (ord('a'), ord('A')): steer = -0.5
                    elif ch in (ord('d'), ord('D')): steer =  0.5
                    elif ch in (ord('c'), ord('C')): steer =  0.0  # center

                # 차량 제어 
                ctrl = carla.VehicleControl(
                    throttle = 0.6 if throttle_on else 0.0,
                    reverse  = reverse_on,
                    steer    = steer,
                    brake    = 1.0 if ch == ord(' ') else 0.0
                )
                if self.vehicle: self.vehicle.apply_control(ctrl)

                # 비동기 모드에서 서버 프레임 진행 대기
                self.world.wait_for_tick(1.0)

                # 3인칭 카메라 추적 + 디버그 라벨
                self.update_spectator()

                # 주기적으로 콘솔에도 좌표/속도 출력
                now = time.time()
                if now - last_print > 0.5:
                    tf = self.vehicle.get_transform()
                    print(f"[{self.world.get_snapshot().frame}] "
                          f"pos=({tf.location.x:.1f},{tf.location.y:.1f}) "
                          f"yaw={tf.rotation.yaw:.1f}°  v={self.speed_kmh():.1f} km/h")
                    last_print = now
        finally:
            curses.nocbreak(); stdscr.keypad(False)
            curses.echo(); curses.endwin()

    def cleanup(self):
        try:
            if self.vehicle: self.vehicle.destroy()
        except: pass
        try:
            s = self.world.get_settings()
            s.synchronous_mode = False
            s.fixed_delta_seconds = None
            self.world.apply_settings(s)
        except: pass
        print("[INFO] Cleaned up.")

def main():
    app = CarlaTeleopApp(SERVER_HOST, SERVER_PORT, TOWN, VEHICLE_BP_ID)
    try:
        app.connect()
        app.spawn_vehicle()
        # 기본: 헤드리스(터미널) 루프
        app.teleop_loop_headless()
    finally:
        app.cleanup()

if __name__ == "__main__":
    main()
