#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
A kiki - CARLA 멀티클라이언트 스폰 + 키보드 조작(홀드형 가속/조향)
+ (옵션) spectator 스무스 팔로우 + (옵션) manual_control 스타일 카메라(pygame)

키:
  - W: 가속(꾹 누르면 램프업, 떼면 BASE_THROTTLE로 램프다운)
  - S: 정지(브레이크 펄스 + 스로틀 초기화)
  - Q: 후진 토글(속도 <= REV_TOGGLE_V 일 때만)
  - Space: 누르는 동안 브레이크
  - 0, X: 스로틀 즉시 해제
  - A / D: 좌/우 조향 (꾹 누르는 동안만 유효, 떼면 센터로 복귀)
  - C: 조향 즉시 센터
  - (PYGAME=1일 때) TAB: 카메라 위치 토글, ` 또는 N: 센서 전환(RGB)
환경변수 예:
  export CARLA_HOST=192.168.86.78
  export CARLA_PORT=2000
  export ROLE_NAME=A_kiki01 SPAWN_INDEX=3 PYGAME=0 FOLLOW_MODE=smooth
"""

import os, json, random, math, time, curses
from glob import glob
import numpy as np
import carla

# ── (옵션) pygame: manual_control 스타일 시점용 ──
USE_PYGAME = os.environ.get("PYGAME", "0") == "1"
if USE_PYGAME:
    try:
        import pygame
        from pygame.locals import (
            K_w, K_s, K_q, K_SPACE, K_a, K_d, K_c, K_ESCAPE,
            K_TAB, K_BACKQUOTE, K_n, K_x, K_0
        )
    except Exception as e:
        print(f"[WARN] pygame import 실패({e}) → headless로 전환")
        USE_PYGAME = False

# ── 환경설정 ──────────────────────────────────────────
ROLE_NAME   = os.environ.get("ROLE_NAME", "A_kiki01")
SPAWN_INDEX = int(os.environ.get("SPAWN_INDEX", "-1"))
SERVER_HOST = os.environ.get("CARLA_HOST", "192.168.86.78")
SERVER_PORT = int(os.environ.get("CARLA_PORT", "2000"))
VEHICLE_BP_ID = os.environ.get("VEHICLE_BP_ID", "vehicle.carlamotors.european_hgv")
TOWN        = os.environ.get("TOWN", "Town05")  # 서버에서 고정 권장

SAVE_DIR = os.path.abspath("sensor_poses"); os.makedirs(SAVE_DIR, exist_ok=True)

# 제어/동작 파라미터
CTRL_DT = float(os.environ.get("CTRL_DT", "0.02"))               # 제어 주기(초)
BASE_THROTTLE = float(os.environ.get("BASE_THROTTLE", "0.0"))    # 키 떼면 수렴값
THROTTLE_RAMP_UP = float(os.environ.get("THROTTLE_RAMP_UP", "1.5"))    # /s
THROTTLE_RAMP_DOWN = float(os.environ.get("THROTTLE_RAMP_DOWN", "3.0")) # /s
KEY_HOLD_GRACE = float(os.environ.get("KEY_HOLD_GRACE", "0.12"))       # W 유예
STOP_BRAKE_SECS = float(os.environ.get("STOP_BRAKE_SECS", "0.5"))
REV_TOGGLE_V = float(os.environ.get("REV_TOGGLE_V", "1.0"))      # km/h

# 조향 파라미터
STEER_ALPHA = float(os.environ.get("STEER_ALPHA", "0.35"))       # 조향 스무딩 0~1
MAX_STEER = float(os.environ.get("MAX_STEER", "0.7"))
STEER_HOLD_GRACE = float(os.environ.get("STEER_HOLD_GRACE", "0.12"))  # curses용 조향 홀드 유예

# spectator(관전자) 팔로우 설정
FOLLOW_MODE = os.environ.get("FOLLOW_MODE", "smooth")  # "smooth" | "oneshot" | "none"
FOLLOW_DISTANCE = float(os.environ.get("FOLLOW_DIST", "8.0"))
FOLLOW_HEIGHT   = float(os.environ.get("FOLLOW_Z", "3.0"))
FOLLOW_PITCH    = float(os.environ.get("FOLLOW_PITCH", "-15.0"))
CAM_POS_LERP    = float(os.environ.get("CAM_POS_LERP", "0.18"))  # 0.0~1.0
CAM_YAW_LERP    = float(os.environ.get("CAM_YAW_LERP", "0.2"))   # 0.0~1.0

# pygame 창 해상도 (PYGAME=1일 때만)
RES = os.environ.get("RES", "1280x720")
WIN_W, WIN_H = [int(x) for x in RES.split("x")]

# ── 유틸 ─────────────────────────────────────────────
def set_attr_safe(bp: carla.ActorBlueprint, name: str, value):
    try: bp.set_attribute(name, str(value))
    except Exception: pass

def find_latest_pose_file():
    files = sorted(glob(os.path.join(SAVE_DIR, "poses_*.json")), key=os.path.getmtime, reverse=True)
    return files[0] if files else None

def dict_to_tf(d: dict) -> carla.Transform:
    loc, rot = d["location"], d["rotation"]
    return carla.Transform(
        carla.Location(x=loc["x"], y=loc["y"], z=loc["z"]),
        carla.Rotation(yaw=rot["yaw"], pitch=rot["pitch"], roll=rot["roll"])
    )

# ── CameraManager (manual_control 스타일, 경량版) ───────────
class CameraManager:
    """차량에 부착된 카메라를 여러 트랜스폼(SpringArm 포함)으로 토글."""
    def __init__(self, vehicle, width, height, gamma=1.0):
        self.vehicle = vehicle
        self.world = vehicle.get_world()
        self.width = width; self.height = height
        self.surface = None
        self.sensor = None

        ext = vehicle.bounding_box.extent
        bx, by, bz = 0.5 + ext.x, 0.5 + ext.y, 0.5 + ext.z
        Attachment = carla.AttachmentType
        self.transforms = [
            (carla.Transform(carla.Location(x=-2.0*bx, y=0.0, z=2.0*bz),
                             carla.Rotation(pitch=8.0)),  Attachment.SpringArmGhost),
            (carla.Transform(carla.Location(x=+0.8*bx, y=0.0, z=1.3*bz)),
             Attachment.Rigid),
            (carla.Transform(carla.Location(x=-2.8*bx, y=0.0, z=4.6*bz),
                             carla.Rotation(pitch=6.0)),  Attachment.SpringArmGhost),
            (carla.Transform(carla.Location(x=+1.9*bx, y=+1.0*by, z=1.2*bz)),
             Attachment.SpringArmGhost),
        ]
        self.t_index = 0

        bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        set_attr_safe(bp, 'image_size_x', self.width)
        set_attr_safe(bp, 'image_size_y', self.height)
        if bp.has_attribute('gamma'):
            set_attr_safe(bp, 'gamma', gamma)
        self.rgb_bp = bp

        self._spawn()

    def _spawn(self):
        if self.sensor:
            try: self.sensor.stop(); self.sensor.destroy()
            except Exception: pass
            self.sensor = None
        tf, attach = self.transforms[self.t_index]
        self.sensor = self.world.spawn_actor(
            self.rgb_bp, tf, attach_to=self.vehicle, attachment_type=attach
        )
        weak_self = self
        self.sensor.listen(lambda im: CameraManager._on_rgb(weak_self, im))

    def toggle_transform(self):
        self.t_index = (self.t_index + 1) % len(self.transforms)
        self._spawn()

    @staticmethod
    def _on_rgb(self, image: carla.Image):
        try:
            array = np.frombuffer(image.raw_data, dtype=np.uint8)
            array = array.reshape((image.height, image.width, 4))[:, :, :3][:, :, ::-1]
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        except Exception:
            self.surface = None

    def render(self, display):
        if self.surface is not None:
            display.blit(self.surface, (0, 0))

    def destroy(self):
        try:
            if self.sensor: self.sensor.stop(); self.sensor.destroy()
        except Exception:
            pass
        self.sensor = None

# ── 메인 앱 ──────────────────────────────────────────
class CarlaTeleopApp:
    def __init__(self, host, port, town, vehicle_bp):
        self.host, self.port = host, port
        self.town, self.vehicle_bp_id = town, vehicle_bp
        self.client = self.world = self.vehicle = None
        self.spectator = None

        # 센서 상대 TF (있으면 파일로 덮어씀)
        self.rel_tf_cam   = carla.Transform(carla.Location(x=3.6, y=0.0, z=3.0))
        self.rel_tf_lidar = carla.Transform(carla.Location(x=3.6, y=0.0, z=3.9))
        self.rel_tf_imu   = carla.Transform(carla.Location(x=0.0, y=0.0, z=1.2))
        self.rel_tf_gnss  = carla.Transform(carla.Location(x=0.0, y=0.0, z=1.8))

        self._orig_settings = None

        # 제어 상태
        self.reverse = False
        self.brake_until = 0.0
        self.steer = 0.0
        self.steer_target = 0.0
        self.throttle = BASE_THROTTLE
        self.throttle_cmd = BASE_THROTTLE
        self.last_w_seen = 0.0  # W 감지 시각

        # 조향(헤드리스 홀드 판정용)
        self.last_a_seen = 0.0
        self.last_d_seen = 0.0

        # spectator 스무스 팔로우 상태
        self._cam_last_tf = None

        # pygame 카메라 매니저(옵션)
        self.cam_mgr = None
        self.has_window = False

    # ── 연결/환경 ───────────────────────────────
    def connect(self):
        self.client = carla.Client(self.host, self.port)
        self.client.set_timeout(10.0)

        preload = find_latest_pose_file()
        if preload:
            try:
                with open(preload) as f:
                    data = json.load(f)
                if "camera" in data: self.rel_tf_cam   = dict_to_tf(data["camera"])
                if "lidar"  in data: self.rel_tf_lidar = dict_to_tf(data["lidar"])
                if "imu"    in data: self.rel_tf_imu   = dict_to_tf(data["imu"])
                if "gnss"   in data: self.rel_tf_gnss  = dict_to_tf(data["gnss"])
                print(f"[INFO] Loaded poses from {preload}")
            except Exception as e:
                print(f"[WARN] Pose load failed: {e}")

        # 다중 PC: 맵은 서버에서만 설정, 클라는 get_world()만
        self.world = self.client.get_world()
        print("[INFO] Map:", self.world.get_map().name)

        # 비동기 설정(호환성)
        self._orig_settings = self.world.get_settings()
        s = self.world.get_settings()
        s.synchronous_mode = False
        s.fixed_delta_seconds = 0.0
        if hasattr(s, "substepping"): s.substepping = True
        if hasattr(s, "max_substep_delta_time"): s.max_substep_delta_time = 0.01
        if hasattr(s, "max_substeps"): s.max_substeps = 10
        self.world.apply_settings(s)

        self.spectator = self.world.get_spectator()

    # ── 스폰 ────────────────────────────────────
    def spawn_vehicle(self):
        bp_lib = self.world.get_blueprint_library()
        try:
            bp = bp_lib.find(self.vehicle_bp_id)
        except Exception:
            print(f"[WARN] BP '{self.vehicle_bp_id}' 없음 → Tesla로 대체")
            bp = bp_lib.find("vehicle.tesla.model3")
        set_attr_safe(bp, "role_name", ROLE_NAME)

        sps = self.world.get_map().get_spawn_points()
        cands = [sps[SPAWN_INDEX]] if 0 <= SPAWN_INDEX < len(sps) else sps[:]
        random.shuffle(cands)

        for sp in cands:
            v = self.world.try_spawn_actor(bp, sp)
            if v:
                self.vehicle = v
                break
        if not self.vehicle:
            raise RuntimeError("Spawn failed: no free spawn point")

        self.vehicle.set_autopilot(False)
        print(f"[INFO] Spawned: {self.vehicle.type_id} id={self.vehicle.id} role={ROLE_NAME}")

        # spectator 시점 설정
        if self.spectator and FOLLOW_MODE in ("smooth", "oneshot"):
            tf0 = self._compute_chase_tf(self.vehicle.get_transform())
            self.spectator.set_transform(tf0)
            self._cam_last_tf = tf0 if FOLLOW_MODE == "smooth" else None

        # (옵션) pygame 카메라: manual_control 스타일
        if USE_PYGAME:
            self._init_pygame()
            self.cam_mgr = CameraManager(self.vehicle, self.win_w, self.win_h, gamma=1.0)

    # ── spectator 스무스 팔로우 ───────────────────
    def _compute_chase_tf(self, veh_tf: carla.Transform) -> carla.Transform:
        yaw = math.radians(veh_tf.rotation.yaw)
        offset = carla.Location(
            x=-FOLLOW_DISTANCE * math.cos(yaw),
            y=-FOLLOW_DISTANCE * math.sin(yaw),
            z= FOLLOW_HEIGHT
        )
        return carla.Transform(
            veh_tf.location + offset,
            carla.Rotation(pitch=FOLLOW_PITCH, yaw=veh_tf.rotation.yaw)
        )

    @staticmethod
    def _lerp(a, b, t):
        return a + (b - a) * t

    @staticmethod
    def _lerp_yaw(yaw_from, yaw_to, t):
        dy = ((yaw_to - yaw_from + 180) % 360) - 180
        return yaw_from + dy * t

    def update_spectator_smooth(self, dt: float):
        if not self.spectator or FOLLOW_MODE != "smooth" or not self.vehicle:
            return
        target = self._compute_chase_tf(self.vehicle.get_transform())

        if self._cam_last_tf is None:
            self._cam_last_tf = self.spectator.get_transform()

        cur = self._cam_last_tf
        new_loc = carla.Location(
            x=self._lerp(cur.location.x, target.location.x, CAM_POS_LERP),
            y=self._lerp(cur.location.y, target.location.y, CAM_POS_LERP),
            z=self._lerp(cur.location.z, target.location.z, CAM_POS_LERP),
        )
        new_yaw   = self._lerp_yaw(cur.rotation.yaw,   target.rotation.yaw,   CAM_YAW_LERP)
        new_pitch = self._lerp(cur.rotation.pitch,     target.rotation.pitch, CAM_YAW_LERP)
        new_rot = carla.Rotation(pitch=new_pitch, yaw=new_yaw, roll=0.0)

        tf = carla.Transform(new_loc, new_rot)
        self.spectator.set_transform(tf)
        self._cam_last_tf = tf

    # ── pygame 초기화(있으면 창, 실패 시 더미로) ─────
    def _init_pygame(self):
        import pygame
        self.win_w, self.win_h = WIN_W, WIN_H
        pygame.init(); pygame.font.init()
        try:
            if os.environ.get("SDL_VIDEODRIVER") == "dummy":
                raise pygame.error("dummy requested")
            pygame.display.init()
            self.screen = pygame.display.set_mode((self.win_w, self.win_h),
                                                  pygame.HWSURFACE | pygame.DOUBLEBUF)
            self.has_window = True
        except Exception as e:
            print(f"[WARN] pygame window init 실패({e}) → headless pygame로 전환")
            os.environ["SDL_VIDEODRIVER"] = "dummy"
            try: pygame.display.quit()
            except Exception: pass
            pygame.display.init()
            self.screen = pygame.display.set_mode((1, 1))
            self.has_window = False
        self.clock = pygame.time.Clock()

    # ── 속도/보조 ───────────────────────────────
    def speed_kmh(self):
        v = self.vehicle.get_velocity()
        return math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z) * 3.6

    # ── curses 루프 ─────────────────────────────
    def teleop_loop_headless(self):
        stdscr = curses.initscr()
        curses.noecho(); curses.cbreak()
        stdscr.nodelay(True); stdscr.keypad(True)

        last_print = 0.0
        try:
            running = True
            while running:
                ch = stdscr.getch()
                now = time.time()

                if ch == 27:  # ESC
                    running = False

                # ── 가속(홀드/감쇠)
                if ch in (ord('w'), ord('W')):
                    self.last_w_seen = now
                w_held = (now - self.last_w_seen) < KEY_HOLD_GRACE
                if w_held:
                    self.throttle_cmd = min(1.0, self.throttle_cmd + THROTTLE_RAMP_UP * CTRL_DT)
                else:
                    if ch in (ord('x'), ord('X'), ord('0')):
                        self.throttle_cmd = BASE_THROTTLE
                    elif self.throttle_cmd > BASE_THROTTLE:
                        self.throttle_cmd = max(BASE_THROTTLE, self.throttle_cmd - THROTTLE_RAMP_DOWN * CTRL_DT)

                # 정지(S)
                if ch in (ord('s'), ord('S')):
                    self.throttle_cmd = BASE_THROTTLE
                    self.brake_until = now + STOP_BRAKE_SECS

                # 후진 토글(Q)
                if ch in (ord('q'), ord('Q')):
                    if self.speed_kmh() <= REV_TOGGLE_V:
                        self.reverse = not self.reverse
                    else:
                        self.brake_until = max(self.brake_until, now + 0.1)

                # ── 조향: 기본 센터, 꾹 누르는 동안만 좌/우 유지
                if ch in (ord('a'), ord('A')):
                    self.last_a_seen = now
                elif ch in (ord('d'), ord('D')):
                    self.last_d_seen = now
                elif ch in (ord('c'), ord('C')):
                    self.last_a_seen = self.last_d_seen = 0.0  # 즉시 센터

                a_held = (now - self.last_a_seen) < STEER_HOLD_GRACE
                d_held = (now - self.last_d_seen) < STEER_HOLD_GRACE

                self.steer_target = 0.0
                if a_held and not d_held:
                    self.steer_target = -MAX_STEER
                elif d_held and not a_held:
                    self.steer_target = +MAX_STEER
                # 둘 다면 센터

                # 브레이크
                braking = (ch == ord(' ')) or (time.time() < self.brake_until)
                brake = 1.0 if braking else 0.0

                # 스무딩/적용
                self.steer += (self.steer_target - self.steer) * STEER_ALPHA
                steer_cmd = float(max(-1.0, min(1.0, self.steer)))
                self.throttle += (self.throttle_cmd - self.throttle) * 0.5
                throttle = float(max(0.0, min(1.0, self.throttle)))

                ctrl = carla.VehicleControl(
                    throttle=0.0 if braking else throttle,
                    brake=brake,
                    reverse=self.reverse,
                    steer=steer_cmd
                )
                self.vehicle.apply_control(ctrl)

                # spectator 스무스 팔로우
                if FOLLOW_MODE == "smooth":
                    self.update_spectator_smooth(CTRL_DT)

                # 로그(2Hz)
                if time.time() - last_print > 0.5:
                    stdscr.erase()
                    stdscr.addstr(0, 0,
                        (f"[{self.world.get_snapshot().frame}] "
                         f"gear={'R' if self.reverse else 'D'} th={throttle:.2f} br={brake:.2f} "
                         f"st={steer_cmd:.2f} v={self.speed_kmh():.1f} km/h"))
                    stdscr.refresh()
                    last_print = time.time()

                time.sleep(CTRL_DT)
        finally:
            curses.nocbreak(); stdscr.keypad(False); curses.echo(); curses.endwin()

    # ── pygame 루프 ─────────────────────────────
    def teleop_loop_pygame(self):
        pygame = __import__("pygame")
        K_w, K_s, K_q, K_SPACE, K_a, K_d, K_c, K_ESCAPE, K_TAB, K_BACKQUOTE, K_n, K_x, K_0 = \
            pygame.K_w, pygame.K_s, pygame.K_q, pygame.K_SPACE, pygame.K_a, pygame.K_d, pygame.K_c, \
            pygame.K_ESCAPE, pygame.K_TAB, pygame.K_BACKQUOTE, pygame.K_n, pygame.K_x, pygame.K_0

        last_log = 0.0
        running = True
        while running:
            dt = self.clock.tick_busy_loop(int(1/CTRL_DT)) / 1000.0

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == K_ESCAPE: running = False
                    elif event.key == K_TAB and self.cam_mgr:
                        self.cam_mgr.toggle_transform()
                    elif event.key in (K_BACKQUOTE, K_n) and self.cam_mgr:
                        pass

            keys = pygame.key.get_pressed()

            # ── 가속(홀드/감쇠)
            if keys[K_w]:
                self.throttle_cmd = min(1.0, self.throttle_cmd + THROTTLE_RAMP_UP * dt)
            else:
                if keys[K_x] or keys[K_0]:
                    self.throttle_cmd = BASE_THROTTLE
                elif self.throttle_cmd > BASE_THROTTLE:
                    self.throttle_cmd = max(BASE_THROTTLE, self.throttle_cmd - THROTTLE_RAMP_DOWN * dt)

            # 정지(S)
            if keys[K_s]:
                self.throttle_cmd = BASE_THROTTLE
                self.brake_until = time.time() + STOP_BRAKE_SECS

            # 후진(Q)
            if keys[K_q]:
                if self.speed_kmh() <= REV_TOGGLE_V:
                    self.reverse = not self.reverse
                else:
                    self.brake_until = max(self.brake_until, time.time() + 0.1)

            # ── 조향: 기본 센터, 누르는 동안만 좌/우
            self.steer_target = 0.0
            if keys[K_a] and not keys[K_d]:
                self.steer_target = -MAX_STEER
            elif keys[K_d] and not keys[K_a]:
                self.steer_target = +MAX_STEER
            elif keys[K_c]:
                self.steer_target = 0.0  # 즉시 센터

            # 브레이크
            braking = keys[K_SPACE] or (time.time() < self.brake_until)
            brake = 1.0 if braking else 0.0

            # 스무딩/적용
            self.steer += (self.steer_target - self.steer) * STEER_ALPHA
            steer_cmd = float(max(-1.0, min(1.0, self.steer)))
            self.throttle += (self.throttle_cmd - self.throttle) * 0.5
            throttle = float(max(0.0, min(1.0, self.throttle)))

            ctrl = carla.VehicleControl(
                throttle=0.0 if braking else throttle,
                brake=brake,
                reverse=self.reverse,
                steer=steer_cmd
            )
            self.vehicle.apply_control(ctrl)

            # spectator 스무스 팔로우
            if FOLLOW_MODE == "smooth":
                self.update_spectator_smooth(dt)

            # 렌더링
            if self.has_window:
                self.screen.fill((0, 0, 0))
                if self.cam_mgr: self.cam_mgr.render(self.screen)
                pygame.display.flip()

            # 로그(2Hz)
            if time.time() - last_log > 0.5:
                print(f"\r[{self.world.get_snapshot().frame}] "
                      f"gear={'R' if self.reverse else 'D'} th={throttle:.2f} br={brake:.2f} "
                      f"st={steer_cmd:.2f} v={self.speed_kmh():.1f} km/h", end="", flush=True)
                last_log = time.time()

        print()
        pygame.quit()

    # ── 정리 ─────────────────────────────────────
    def cleanup(self):
        if self.cam_mgr: self.cam_mgr.destroy()
        try:
            if self.vehicle: self.vehicle.destroy()
        except Exception: pass
        try:
            if self._orig_settings and self.world:
                self.world.apply_settings(self._orig_settings)
        except Exception: pass
        print("\n[INFO] Cleaned up.")

# ── 실행부 ───────────────────────────────────────────
def main():
    app = CarlaTeleopApp(SERVER_HOST, SERVER_PORT, TOWN, VEHICLE_BP_ID)
    try:
        app.connect()
        app.spawn_vehicle()
        if USE_PYGAME:
            app.teleop_loop_pygame()
        else:
            app.teleop_loop_headless()
    finally:
        app.cleanup()

if __name__ == "__main__":
    main()
