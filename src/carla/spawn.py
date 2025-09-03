import carla
import random
import time

def main():
    # 1. 서버 연결
    client = carla.Client("localhost", 2000)  # Ubuntu 서버 IP와 포트
    # client.set_timeout(10.0)

    # 2. 맵 로드 (Town03)
    world = client.load_world("Town03")
    print("Loaded map:", world.get_map().name)

    # 3. 블루프린트 라이브러리 가져오기
    blueprint_library = world.get_blueprint_library()

    # 4. Mini Cooper 블루프린트 선택
    vehicle_bp = blueprint_library.find("vehicle.mini.cooper_s")

    # 5. 스폰 지점 중 하나 선택
    spawn_points = world.get_map().get_spawn_points()
    print("the spqwn points : ",spawn_points)
    spawn_point = random.choice(spawn_points)

    # 6. 차량 스폰
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)
    print("Spawned vehicle:", vehicle.type_id)

    # 7. 카메라 시점 이동 (차량 따라가기)
    spectator = world.get_spectator()
    transform = carla.Transform(
        spawn_point.location + carla.Location(z=50),  # 차량 위쪽
        carla.Rotation(pitch=-90)                     # 위에서 아래로 보기
    )
    spectator.set_transform(transform)

    # 8. 몇 초간 유지
    time.sleep(10)

    # 9. 차량 제거
    # vehicle.destroy()
    # print("Vehicle destroyed.")

if __name__ == "__main__":
    main()
