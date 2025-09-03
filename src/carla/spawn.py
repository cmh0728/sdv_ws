import carla
import random
import time

def main():
    # 서버 연결
    client = carla.Client("localhost", 2000)  # Ubuntu 서버 IP와 포트
    # client.set_timeout(10.0)

    # 맵 로드 (Town03)
    world = client.load_world("Town03")
    # print("Loaded map:", world.get_map().name)

    # 블루프린트 라이브러리 가져오기
    blueprint_library = world.get_blueprint_library()

    # Mini Cooper 블루프린트 선택
    vehicle_bp = blueprint_library.find("vehicle.carlamotors.european_hgv")

    # 스폰 지점 중 하나 선택
    spawn_points = world.get_map().get_spawn_points()
    # print("the spqwn points : ",spawn_points)
    spawn_point = random.choice(spawn_points)

    # 차량 스폰
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)
    print("Spawned vehicle:", vehicle.type_id)

    # 센서 블루프린트 가져오기
    camera_bp = blueprint_library.find("sensor.camera.rgb")

    # 속성 수정 (청사진 속성 테이블 참고)
    camera_bp.set_attribute("image_size_x", "800")
    camera_bp.set_attribute("image_size_y", "600")
    camera_bp.set_attribute("fov", "110")   # 시야각

    # 차량 앞유리 위치에 카메라 부착
    camera_transform = carla.Transform(
        carla.Location(x=1.5, z=2.4),    # 차량 앞쪽 1.5m, 위로 2.4m
        carla.Rotation(pitch=0, yaw=0, roll=0)
    )

    # 센서 생성 (차량에 attach)
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
    print("Camera attached:", camera.type_id)



    # 카메라 시점 이동 (차량 따라가기)
    spectator = world.get_spectator()
    transform = carla.Transform(
        spawn_point.location + carla.Location(z=50),  # 차량 위쪽
        carla.Rotation(pitch=-90)                     # 위에서 아래로 보기
    )
    spectator.set_transform(transform)

    # 8. 몇 초간 유지
    # time.sleep(10)

    # 9. 차량 제거
    # vehicle.destroy()
    # print("Vehicle destroyed.")

if __name__ == "__main__":
    main()
