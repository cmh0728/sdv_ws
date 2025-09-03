# import sys
# sys.path.append("/Users/choeminhyeog/Downloads/CARLA_0.9.15/PythonAPI/carla/dist/carla-0.9.15-py3.7-linux-x86_64.egg")

import carla
import cv2
import numpy as np
import time
import random

# ====== 사용자 설정 ======
SERVER_HOST = "localhost"  # Ubuntu CARLA 서버 IP로 변경
SERVER_PORT = 2000            # 서버 포트(기본 2000)
TOWN = "Town03"
VEHICLE_BP_ID = "vehicle.mini.cooper_s"

# 센서 해상도/FOV
IMG_W, IMG_H = 1280, 720
FOV = 90

# 이동/회전 스텝
STEP_POS = 0.10   # 미터
STEP_ROT = 2.0    # 도(deg)
# ========================

def to_bgr(image: carla.Image) -> np.ndarray:
    """CARLA BGRA raw → OpenCV BGR"""
    array = np.frombuffer(image.raw_data, dtype=np.uint8)
    array = array.reshape((image.height, image.width, 4))
    bgr = cv2.cvtColor(array, cv2.COLOR_BGRA2BGR)
    return bgr

def put_hud(img, rel_tf: carla.Transform):
    """현재 상대 Transform 텍스트 HUD"""
    loc = rel_tf.location
    rot = rel_tf.rotation
    lines = [
        f"[W/S,A/D,Z/X] xyz: ({loc.x:+.2f}, {loc.y:+.2f}, {loc.z:+.2f}) m",
        f"[J/L,I/K,U/O] rpy: (yaw {rot.yaw:+.1f}, pitch {rot.pitch:+.1f}, roll {rot.roll:+.1f}) deg",
        "[0] reset  [P] save frame  [Q] quit",
    ]
    y = 28
    for line in lines:
        cv2.putText(img, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
        y += 28

def clamp_angle(a):
    """-180~180도로 보정"""
    while a > 180.0: a -= 360.0
    while a < -180.0: a += 360.0
    return a

def main():
    client = carla.Client(SERVER_HOST, SERVER_PORT)
    client.set_timeout(10.0)

    # 맵 로드
    world = client.load_world(TOWN)
    print("Loaded map:", world.get_map().name)

    # 스폰 시 가끔 물리/트래픽 영향 줄이려면 잠깐 월드 동결 (옵션)
    original_settings = world.get_settings()

    # 블루프린트
    bp_lib = world.get_blueprint_library()
    veh_bp = bp_lib.find(VEHICLE_BP_ID)

    # 스폰 포인트
    spawn_points = world.get_map().get_spawn_points()
    spawn_point = random.choice(spawn_points)

    # 차량 스폰
    vehicle = world.spawn_actor(veh_bp, spawn_point)
    print("Spawned vehicle:", vehicle.type_id)

    # 카메라 블루프린트 + 속성
    cam_bp = bp_lib.find("sensor.camera.rgb")
    cam_bp.set_attribute("image_size_x", str(IMG_W))
    cam_bp.set_attribute("image_size_y", str(IMG_H))
    cam_bp.set_attribute("fov", str(FOV))
    # 센서 주기(초) - 0이면 가능한 빨리. 필요시 0.05(=20Hz) 등으로 조정.
    cam_bp.set_attribute("sensor_tick", "0.0")

    # 초기 상대 위치/방향: 앞유리 근처 예시
    rel_tf = carla.Transform(
        location=carla.Location(x=1.5, y=0.0, z=1.5),
        rotation=carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0)
    )

    # 센서 스폰 (vehicle에 부착)
    camera = world.spawn_actor(cam_bp, rel_tf, attach_to=vehicle)
    print("Camera attached:", camera.type_id)

    # 최신 프레임 공유 변수
    last_frame = {"img": None, "frame_id": -1}

    def on_image(image: carla.Image):
        img = to_bgr(image)
        last_frame["img"] = img
        last_frame["frame_id"] = image.frame

    camera.listen(on_image)

    # spectator를 위로 올려서 차량 확인(옵션)
    spec = world.get_spectator()
    spec.set_transform(carla.Transform(
        location=vehicle.get_transform().location + carla.Location(z=30),
        rotation=carla.Rotation(pitch=-90)
    ))

    print("""
=== Controls ===
Move:     W/S = +x/-x   A/D = -y/+y   Z/X = -z/+z
Rotate:   J/L = -yaw/+yaw   I/K = +pitch/-pitch   U/O = -roll/+roll
Others:   0 = reset   P = save frame   Q = quit
Note: x=forward(+), y=right(+), z=up(+)
""")

    cv2.namedWindow("CARLA Camera", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("CARLA Camera", 960, 540)

    # 메인 루프
    try:
        last_save_t = 0.0
        while True:
            img = last_frame["img"]
            if img is not None:
                # HUD 오버레이
                hud_img = img.copy()
                put_hud(hud_img, rel_tf)
                cv2.imshow("CARLA Camera", hud_img)

            key = cv2.waitKey(1) & 0xFF
            if key != 255:  # 키 입력 있음
                # 이동
                if key == ord('w'): rel_tf.location.x += STEP_POS
                elif key == ord('s'): rel_tf.location.x -= STEP_POS
                elif key == ord('a'): rel_tf.location.y -= STEP_POS
                elif key == ord('d'): rel_tf.location.y += STEP_POS
                elif key == ord('z'): rel_tf.location.z -= STEP_POS
                elif key == ord('x'): rel_tf.location.z += STEP_POS

                # 회전
                elif key == ord('j'): rel_tf.rotation.yaw  = clamp_angle(rel_tf.rotation.yaw  - STEP_ROT)
                elif key == ord('l'): rel_tf.rotation.yaw  = clamp_angle(rel_tf.rotation.yaw  + STEP_ROT)
                elif key == ord('i'): rel_tf.rotation.pitch = clamp_angle(rel_tf.rotation.pitch + STEP_ROT)
                elif key == ord('k'): rel_tf.rotation.pitch = clamp_angle(rel_tf.rotation.pitch - STEP_ROT)
                elif key == ord('u'): rel_tf.rotation.roll = clamp_angle(rel_tf.rotation.roll - STEP_ROT)
                elif key == ord('o'): rel_tf.rotation.roll = clamp_angle(rel_tf.rotation.roll + STEP_ROT)

                # 초기화
                elif key == ord('0'):
                    rel_tf = carla.Transform(
                        location=carla.Location(x=1.5, y=0.0, z=1.5),
                        rotation=carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0)
                    )
                    print("Transform reset.")

                # 저장 (0.5초 쿨다운)
                elif key == ord('p'):
                    now = time.time()
                    if img is not None and now - last_save_t > 0.5:
                        fname = f"carla_cam_{last_frame['frame_id']}.png"
                        cv2.imwrite(fname, img)
                        last_save_t = now
                        print("Saved:", fname)

                # 종료
                elif key == ord('q'):
                    print("Quitting...")
                    break

                # 변경된 Transform를 즉시 적용 (부모=vehicle 기준의 상대 좌표)
                camera.set_transform(rel_tf)

    finally:
        # 정리
        try:
            camera.stop()
        except:
            pass
        for actor in [camera, vehicle]:
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
