import carla
import cv2
import numpy as np
import time
import random

# ====== 사용자 설정 ======
SERVER_HOST = "localhost"   # Ubuntu CARLA 서버 IP
SERVER_PORT = 2000          # 기본 2000
TOWN = "Town03"
VEHICLE_BP_ID = "vehicle.carlamotors.european_hgv"

# 카메라 설정
IMG_W, IMG_H = 1280, 720
FOV = 90

# LiDAR 설정
LIDAR_CHANNELS = 32
LIDAR_PPS = 600000                 # points_per_second
LIDAR_ROT_HZ = 10                  # rotation_frequency
LIDAR_RANGE = 80.0
LIDAR_UP_FOV = 10.0
LIDAR_LOW_FOV = -30.0

# BEV 미리보기 (라이다)
BEV_SIZE = 800                     # 픽셀
BEV_METERS = 80.0                  # ±40 m
# ========================

# 이동/회전 스텝
STEP_POS = 0.10   # m
STEP_ROT = 2.0    # deg

def to_bgr(image: carla.Image) -> np.ndarray:
    arr = np.frombuffer(image.raw_data, dtype=np.uint8)
    arr = arr.reshape((image.height, image.width, 4))
    return cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)

def safe_set_attr(bp, name, value):
    try:
        if bp.has_attribute(name):
            bp.set_attribute(name, str(value))
    except RuntimeError:
        pass

def clamp_angle(a):
    while a > 180.0: a -= 360.0
    while a < -180.0: a += 360.0
    return a

def put_hud(img, rel_tf, mode, imu_last):
    loc = rel_tf.location
    rot = rel_tf.rotation
    lines = [
        f"[T] control target: {mode.upper()}",
        f"[W/S,A/D,Z/X] xyz: ({loc.x:+.2f}, {loc.y:+.2f}, {loc.z:+.2f}) m",
        f"[J/L,I/K,U/O] rpy: (yaw {rot.yaw:+.1f}, pitch {rot.pitch:+.1f}, roll {rot.roll:+.1f}) deg",
        "[0] reset  [P] save frame  [Q] quit",
    ]
    if imu_last is not None:
        ax, ay, az, gx, gy, gz, comp = imu_last
        lines.append(f"IMU acc(m/s^2)=({ax:+.2f},{ay:+.2f},{az:+.2f}) "
                     f"gyro(rad/s)=({gx:+.2f},{gy:+.2f},{gz:+.2f}) compass={comp:+.2f}")
    y = 28
    for line in lines:
        cv2.putText(img, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2, cv2.LINE_AA)
        y += 28

def bev_from_lidar(points_xyz):
    """
    points_xyz: (N, 3) in vehicle frame (sensor attached to vehicle)
    BEV: x-forward, y-right. We render x,y into a square image.
    """
    bev = np.zeros((BEV_SIZE, BEV_SIZE), dtype=np.uint8)
    half = BEV_SIZE // 2
    res = (BEV_METERS / 2) / half  # meters per pixel

    # clip to range
    x = points_xyz[:, 0]
    y = points_xyz[:, 1]
    m = (np.abs(x) < BEV_METERS/2) & (np.abs(y) < BEV_METERS/2)
    x = x[m]; y = y[m]

    # map to pixels
    u = (x / res + half).astype(np.int32)
    v = (y / res + half).astype(np.int32)

    valid = (u >= 0) & (u < BEV_SIZE) & (v >= 0) & (v < BEV_SIZE)
    bev[u[valid], v[valid]] = 255

    # rotate image so forward is up if you prefer:
    bev = cv2.rotate(bev, cv2.ROTATE_90_COUNTERCLOCKWISE)
    bev = cv2.flip(bev, 0)
    bev_bgr = cv2.cvtColor(bev, cv2.COLOR_GRAY2BGR)
    cv2.putText(bev_bgr, "LiDAR BEV (±40m)", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
    return bev_bgr

def main():
    client = carla.Client(SERVER_HOST, SERVER_PORT)
    client.set_timeout(10.0)
    world = client.load_world(TOWN)
    print("Loaded map:", world.get_map().name)

    original_settings = world.get_settings()

    bp_lib = world.get_blueprint_library()
    veh_bp = bp_lib.find(VEHICLE_BP_ID)

    spawn_point = random.choice(world.get_map().get_spawn_points())
    vehicle = world.spawn_actor(veh_bp, spawn_point)
    print("Spawned vehicle:", vehicle.type_id)

    # ---- Camera ----
    cam_bp = bp_lib.find("sensor.camera.rgb")
    safe_set_attr(cam_bp, "image_size_x", IMG_W)
    safe_set_attr(cam_bp, "image_size_y", IMG_H)
    safe_set_attr(cam_bp, "fov", FOV)
    safe_set_attr(cam_bp, "sensor_tick", 0.0)  # as fast as possible

    rel_tf_cam = carla.Transform(
        carla.Location(x=1.5, y=0.0, z=1.5),
        carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0)
    )
    camera = world.spawn_actor(cam_bp, rel_tf_cam, attach_to=vehicle)
    print("Camera attached:", camera.type_id)

    # ---- LiDAR ----
    lidar_bp = bp_lib.find("sensor.lidar.ray_cast")
    safe_set_attr(lidar_bp, "channels", LIDAR_CHANNELS)
    safe_set_attr(lidar_bp, "points_per_second", LIDAR_PPS)
    safe_set_attr(lidar_bp, "rotation_frequency", LIDAR_ROT_HZ)
    safe_set_attr(lidar_bp, "range", LIDAR_RANGE)
    safe_set_attr(lidar_bp, "upper_fov", LIDAR_UP_FOV)
    safe_set_attr(lidar_bp, "lower_fov", LIDAR_LOW_FOV)
    # 옵션(있을 때만 적용)
    safe_set_attr(lidar_bp, "noise_stddev", 0.0)
    safe_set_attr(lidar_bp, "dropoff_general_rate", 0.0)
    safe_set_attr(lidar_bp, "dropoff_intensity_limit", 1.0)
    safe_set_attr(lidar_bp, "dropoff_zero_intensity", 0.0)
    safe_set_attr(lidar_bp, "sensor_tick", 0.0)

    rel_tf_lidar = carla.Transform(
        carla.Location(x=0.8, y=0.0, z=3.0),   # 트럭 지붕 쪽
        carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0)
    )
    lidar = world.spawn_actor(lidar_bp, rel_tf_lidar, attach_to=vehicle)
    print("LiDAR attached:", lidar.type_id)

    # ---- IMU ----
    imu_bp = bp_lib.find("sensor.other.imu")
    # 필요시 노이즈 설정 (값이 있으면)
    safe_set_attr(imu_bp, "sensor_tick", 0.02)  # 50Hz
    imu = world.spawn_actor(imu_bp, carla.Transform(
        carla.Location(x=0.0, y=0.0, z=1.2)), attach_to=vehicle)
    print("IMU attached:", imu.type_id)

    # ---- Callbacks ----
    last_cam = {"img": None, "frame_id": -1}
    last_bev = {"img": None}
    imu_last = {"vals": None}

    def on_image(image: carla.Image):
        last_cam["img"] = to_bgr(image)
        last_cam["frame_id"] = image.frame

    def on_lidar(meas: carla.LidarMeasurement):
        # raw: float32 [x,y,z, intensity]
        pts = np.frombuffer(meas.raw_data, dtype=np.float32).reshape(-1, 4)[:, :3]
        last_bev["img"] = bev_from_lidar(pts)

    def on_imu(m: carla.IMUMeasurement):
        imu_last["vals"] = (m.accelerometer.x, m.accelerometer.y, m.accelerometer.z,
                            m.gyroscope.x, m.gyroscope.y, m.gyroscope.z,
                            m.compass)

    camera.listen(on_image)
    lidar.listen(on_lidar)
    imu.listen(on_imu)

    # spectator (옵션)
    spec = world.get_spectator()
    spec.set_transform(carla.Transform(
        location=vehicle.get_transform().location + carla.Location(z=35),
        rotation=carla.Rotation(pitch=-90)
    ))

    print("""
=== Controls ===
[T] toggle control target (CAMERA/LIDAR)
Move:     W/S = +x/-x   A/D = -y/+y   Z/X = -z/+z
Rotate:   J/L = -yaw/+yaw   I/K = +pitch/-pitch   U/O = -roll/+roll
Others:   0 = reset   P = save camera frame   Q = quit
Note: x=forward(+), y=right(+), z=up(+)
""")

    cv2.namedWindow("CARLA Camera", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("CARLA Camera", 960, 540)
    cv2.namedWindow("LiDAR BEV", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("LiDAR BEV", 600, 600)

    control_target = "camera"  # or "lidar"
    last_save_t = 0.0

    try:
        while True:
            cam_img = last_cam["img"]
            bev_img = last_bev["img"]
            imu_vals = imu_last["vals"]

            if cam_img is not None:
                hud_img = cam_img.copy()
                tf = rel_tf_cam if control_target == "camera" else rel_tf_lidar
                put_hud(hud_img, tf, control_target, imu_vals)
                cv2.imshow("CARLA Camera", hud_img)

            if bev_img is not None:
                cv2.imshow("LiDAR BEV", bev_img)

            key = cv2.waitKey(1) & 0xFF
            if key != 255:
                # 모드 토글
                if key == ord('t'):
                    control_target = "lidar" if control_target == "camera" else "camera"
                    print("Control target:", control_target.upper())
                # 현재 대상 transform 참조
                cur_tf = rel_tf_cam if control_target == "camera" else rel_tf_lidar

                # 이동
                if key == ord('w'): cur_tf.location.x += STEP_POS
                elif key == ord('s'): cur_tf.location.x -= STEP_POS
                elif key == ord('a'): cur_tf.location.y -= STEP_POS
                elif key == ord('d'): cur_tf.location.y += STEP_POS
                elif key == ord('z'): cur_tf.location.z -= STEP_POS
                elif key == ord('x'): cur_tf.location.z += STEP_POS

                # 회전
                elif key == ord('j'): cur_tf.rotation.yaw   = clamp_angle(cur_tf.rotation.yaw   - STEP_ROT)
                elif key == ord('l'): cur_tf.rotation.yaw   = clamp_angle(cur_tf.rotation.yaw   + STEP_ROT)
                elif key == ord('i'): cur_tf.rotation.pitch = clamp_angle(cur_tf.rotation.pitch + STEP_ROT)
                elif key == ord('k'): cur_tf.rotation.pitch = clamp_angle(cur_tf.rotation.pitch - STEP_ROT)
                elif key == ord('u'): cur_tf.rotation.roll  = clamp_angle(cur_tf.rotation.roll  - STEP_ROT)
                elif key == ord('o'): cur_tf.rotation.roll  = clamp_angle(cur_tf.rotation.roll  + STEP_ROT)

                # 초기화 (대상별)
                elif key == ord('0'):
                    if control_target == "camera":
                        rel_tf_cam.location = carla.Location(x=1.5, y=0.0, z=1.5)
                        rel_tf_cam.rotation = carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0)
                    else:
                        rel_tf_lidar.location = carla.Location(x=0.8, y=0.0, z=3.0)
                        rel_tf_lidar.rotation = carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0)
                    print(f"{control_target} transform reset.")

                # 저장 (카메라 프레임)
                elif key == ord('p'):
                    now = time.time()
                    if cam_img is not None and now - last_save_t > 0.5:
                        fname = f"carla_cam_{last_cam['frame_id']}.png"
                        cv2.imwrite(fname, cam_img)
                        last_save_t = now
                        print("Saved:", fname)

                # 종료
                elif key == ord('q'):
                    print("Quitting...")
                    break

                # 적용
                if control_target == "camera":
                    camera.set_transform(rel_tf_cam)
                else:
                    lidar.set_transform(rel_tf_lidar)

    finally:
        # 정리
        try: camera.stop()
        except: pass
        try: lidar.stop()
        except: pass
        try: imu.stop()
        except: pass

        for actor in [camera, lidar, imu, vehicle]:
            try:
                if actor is not None:
                    actor.destroy()
            except:
                pass

        try:
            world.apply_settings(original_settings)
        except:
            pass

        cv2.destroyAllWindows()
        print("Cleaned up.")

if __name__ == "__main__":
    main()
