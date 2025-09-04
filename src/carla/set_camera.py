import carla
import cv2
import numpy as np
import time
import random
import sys
import select  # ★ 비차단 입력용

# ====== 사용자 설정 ======
SERVER_HOST = "192.168.86.78"
SERVER_PORT = 2000
TOWN = "Town03"
VEHICLE_BP_ID = "vehicle.carlamotors.european_hgv"

IMG_W, IMG_H = 1280, 720
FOV = 90

STEP_POS = 0.10
STEP_ROT = 2.0
# ========================

def to_bgr(image: carla.Image) -> np.ndarray:
    array = np.frombuffer(image.raw_data, dtype=np.uint8).reshape((image.height, image.width, 4))
    return cv2.cvtColor(array, cv2.COLOR_BGRA2BGR)

def clamp_angle(a):
    while a > 180.0: a -= 360.0
    while a < -180.0: a += 360.0
    return a

def put_hud(img, rel_tf: carla.Transform):
    loc = rel_tf.location
    rot = rel_tf.rotation
    lines = [
        f"[W/S,A/D,Z/X] xyz: ({loc.x:+.2f}, {loc.y:+.2f}, {loc.z:+.2f}) m",
        f"[J/L,I/K,U/O] rpy: (yaw {rot.yaw:+.1f}, pitch {rot.pitch:+.1f}, roll {rot.roll:+.1f}) deg",
        "[0] reset  [P] save frame  [Q] quit  [H] help",
    ]
    y = 28
    for line in lines:
        cv2.putText(img, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2, cv2.LINE_AA)
        y += 28

def read_cmd_nonblocking() -> str | None:
    # 표준입력에 읽을 게 있으면 한 줄을 읽고, 없으면 None
    r, _, _ = select.select([sys.stdin], [], [], 0)
    if r:
        return sys.stdin.readline().strip().lower()
    return None

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

    cam_bp = bp_lib.find("sensor.camera.rgb")
    cam_bp.set_attribute("image_size_x", str(IMG_W))
    cam_bp.set_attribute("image_size_y", str(IMG_H))
    cam_bp.set_attribute("fov", str(FOV))
    cam_bp.set_attribute("sensor_tick", "0.0")

    rel_tf = carla.Transform(
        location=carla.Location(x=1.5, y=0.0, z=1.5),
        rotation=carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0)
    )
    camera = world.spawn_actor(cam_bp, rel_tf, attach_to=vehicle)
    print("Camera attached:", camera.type_id)

    last_frame = {"img": None, "frame_id": -1}
    def on_image(image: carla.Image):
        last_frame["img"] = to_bgr(image)
        last_frame["frame_id"] = image.frame
    camera.listen(on_image)

    spec = world.get_spectator()
    spec.set_transform(carla.Transform(
        location=vehicle.get_transform().location + carla.Location(z=30),
        rotation=carla.Rotation(pitch=-90)
    ))

    cv2.namedWindow("CARLA Camera", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("CARLA Camera", 960, 540)

    print("""
=== Controls (터미널에서 입력하고 Enter) ===
w/s: x +/-
a/d: y -/+
z/x: z -/+
j/l: yaw -/+
i/k: pitch +/-
u/o: roll -/+
0  : reset
p  : save frame
q  : quit
h  : help
여러 글자를 한 줄에 연속 입력 가능 (예: wwwaajl)
===========================================
""")

    last_save_t = 0.0
    try:
        while True:
            # 1) 최신 프레임 표시 (GUI 이벤트 펌프를 위해 waitKey(1) 유지)
            img = last_frame["img"]
            if img is not None:
                hud = img.copy()
                put_hud(hud, rel_tf)
                cv2.imshow("CARLA Camera", hud)
                cv2.waitKey(1)  # 창 갱신/이벤트 처리용 (입력은 터미널에서 함)

            # 2) 터미널에서 비차단으로 명령 읽기
            cmdline = read_cmd_nonblocking()
            if not cmdline:
                continue  # 입력이 없으면 즉시 다음 루프로 → 영상은 계속 갱신됨

            # 3) 한 줄에 여러 글자 입력 가능: 예) "wwwaajl"
            for key in cmdline:
                if key == 'w': rel_tf.location.x += STEP_POS
                elif key == 's': rel_tf.location.x -= STEP_POS
                elif key == 'a': rel_tf.location.y -= STEP_POS
                elif key == 'd': rel_tf.location.y += STEP_POS
                elif key == 'z': rel_tf.location.z -= STEP_POS
                elif key == 'x': rel_tf.location.z += STEP_POS
                elif key == 'j': rel_tf.rotation.yaw   = clamp_angle(rel_tf.rotation.yaw - STEP_ROT)
                elif key == 'l': rel_tf.rotation.yaw   = clamp_angle(rel_tf.rotation.yaw + STEP_ROT)
                elif key == 'i': rel_tf.rotation.pitch = clamp_angle(rel_tf.rotation.pitch + STEP_ROT)
                elif key == 'k': rel_tf.rotation.pitch = clamp_angle(rel_tf.rotation.pitch - STEP_ROT)
                elif key == 'u': rel_tf.rotation.roll  = clamp_angle(rel_tf.rotation.roll - STEP_ROT)
                elif key == 'o': rel_tf.rotation.roll  = clamp_angle(rel_tf.rotation.roll + STEP_ROT)
                elif key == '0':
                    rel_tf = carla.Transform(
                        location=carla.Location(x=1.5, y=0.0, z=1.5),
                        rotation=carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0)
                    )
                    print("Transform reset.")
                elif key == 'p':
                    now = time.time()
                    if img is not None and now - last_save_t > 0.5:
                        fname = f"carla_cam_{last_frame['frame_id']}.png"
                        cv2.imwrite(fname, img)
                        last_save_t = now
                        print("Saved:", fname)
                elif key == 'h':
                    print("w/s a/d z/x j/l i/k u/o 0 p q  (여러 글자 연속 입력 가능)")
                elif key == 'q':
                    print("Quitting...")
                    raise KeyboardInterrupt
                else:
                    # 무시
                    pass

                # 변경 즉시 적용
                camera.set_transform(rel_tf)

    except KeyboardInterrupt:
        pass
    finally:
        try: camera.stop()
        except: pass
        for actor in [camera, vehicle]:
            try: actor.destroy()
            except: pass
        try: world.apply_settings(original_settings)
        except: pass
        cv2.destroyAllWindows()
        print("Cleaned up.")

if __name__ == "__main__":
    main()
