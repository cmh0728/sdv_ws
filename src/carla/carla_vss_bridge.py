# carla_vss_bridge.py
import math, time, json, threading
import numpy as np
import cv2

import carla

# ----- KUKSA Databroker -----
# pip: kuksa-databroker-client
from kuksa_databroker_client import DatabrokerClient

# ----- Zenoh (이미지/포인트클라우드 등 대용량) -----
import zenoh

KUKSA_HOST = "127.0.0.1"
KUKSA_PORT = 55556

ZENOH_LOCATOR = "tcp/127.0.0.1:7447"  # 필요 시 원격 라우터 주소로 변경

CARLA_HOST = "127.0.0.1"
CARLA_PORT = 2000

def to_kmh(vel_vec):
    v = math.sqrt(vel_vec.x**2 + vel_vec.y**2 + vel_vec.z**2)
    return v * 3.6

def main():
    # ----- Connect CARLA -----
    client = carla.Client(CARLA_HOST, CARLA_PORT)
    client.set_timeout(5.0)
    world = client.get_world()

    # ego vehicle (이미 하나 있다면 필터/선택 로직으로 교체)
    vehicle = None
    for actor in world.get_actors():
        if actor.type_id.startswith("vehicle."):
            vehicle = actor
            break
    if vehicle is None:
        blueprint_library = world.get_blueprint_library()
        bp = blueprint_library.find("vehicle.tesla.model3")
        spawn = world.get_map().get_spawn_points()[0]
        vehicle = world.spawn_actor(bp, spawn)

    # ----- Sensors: GNSS + IMU + Camera -----
    bp_lib = world.get_blueprint_library()

    # GNSS
    gnss_bp = bp_lib.find("sensor.other.gnss")
    gnss_bp.set_attribute("sensor_tick", "0.05")
    gnss = world.spawn_actor(gnss_bp, carla.Transform(), attach_to=vehicle)

    # IMU
    imu_bp = bp_lib.find("sensor.other.imu")
    imu_bp.set_attribute("sensor_tick", "0.02")
    imu = world.spawn_actor(imu_bp, carla.Transform(), attach_to=vehicle)

    # Camera (RGB)
    cam_bp = bp_lib.find("sensor.camera.rgb")
    cam_bp.set_attribute("image_size_x", "640")
    cam_bp.set_attribute("image_size_y", "360")
    cam_bp.set_attribute("fov", "90")
    cam_bp.set_attribute("sensor_tick", "0.05")
    cam = world.spawn_actor(cam_bp, carla.Transform(carla.Location(x=1.5, z=1.6)), attach_to=vehicle)

    # ----- Connect KUKSA Databroker -----
    db = DatabrokerClient(address=f"{KUKSA_HOST}:{KUKSA_PORT}")
    # Databroker는 기본 인증 없음. 필요시 TLS/토큰 설정.

    # ----- Connect Zenoh -----
    conf = zenoh.Config()
    # 로컬 라우터가 아닐 경우:
    # conf.insert_json5("connect/endpoints", f'["{ZENOH_LOCATOR}"]')
    z = zenoh.open(conf)
    pub_cam = z.declare_publisher("carla/camera/front")  # 바이너리 JPEG
    # LiDAR 등 추가 시 pub_lidar = z.declare_publisher("carla/lidar/front")

    # ----- Sensor callbacks -----
    latest_gnss = {"lat": None, "lon": None, "alt": None}
    latest_imu = {"roll": None, "pitch": None, "yaw": None}

    def on_gnss(data):
        latest_gnss["lat"] = float(data.latitude)
        latest_gnss["lon"] = float(data.longitude)
        latest_gnss["alt"] = float(data.altitude)

    def on_imu(data):
        latest_imu["roll"] = float(data.roll)
        latest_imu["pitch"] = float(data.pitch)
        latest_imu["yaw"] = float(data.yaw)

    def on_cam(image):
        # CARLA → numpy → JPEG 인코딩 → Zenoh 퍼블리시
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((image.height, image.width, 4))[:, :, :3]  # BGRA → BGR
        ok, jpg = cv2.imencode(".jpg", array, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
        if ok:
            pub_cam.put(jpg.tobytes())

    gnss.listen(on_gnss)
    imu.listen(on_imu)
    cam.listen(on_cam)

    print("[A kiki] Bridge running... (Ctrl+C to exit)")
    try:
        while True:
            ctrl = vehicle.get_control()
            vel = vehicle.get_velocity()
            trf = vehicle.get_transform()

            speed_kmh = to_kmh(vel)
            throttle = float(ctrl.throttle) * 100.0
            brake = float(ctrl.brake) * 100.0
            steer_deg = float(ctrl.steer) * 450.0  # 필요시 스티어링 비율 맞춤(예: ±450deg)
            gear = vehicle.get_control().gear  # 드라이브/리버스 상황에 유의

            # ----- KUKSA VSS 업데이트 -----
            # 존재하는 VSS 신호만 set. 필요 시 overlay로 확장 가능.
            updates = {
                "Vehicle.Speed": speed_kmh,
                "Vehicle.Accelerator.PedalPosition": throttle,
                "Vehicle.Brake.PedalPosition": brake,
                "Vehicle.Cabin.SteeringWheel.Angle": steer_deg,
                "Vehicle.Powertrain.Transmission.Gear": gear,
            }
            if latest_gnss["lat"] is not None:
                updates["Vehicle.CurrentLocation.Latitude"]  = latest_gnss["lat"]
                updates["Vehicle.CurrentLocation.Longitude"] = latest_gnss["lon"]
                updates["Vehicle.CurrentLocation.Altitude"]  = latest_gnss["alt"]

            # DatabrokerClient: set_current_values(dict[str, Any])
            db.set_current_values(updates)

            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        cam.stop(); imu.stop(); gnss.stop()
        cam.destroy(); imu.destroy(); gnss.destroy()
        z.close()
        print("[A kiki] Bridge stopped.")

if __name__ == "__main__":
    main()

