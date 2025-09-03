import carla
import random

def main():
    # Connect to the client and retrieve the world object
    client = carla.Client('localhost', 2000) # 기본2000번 포트, 다 사용가능.
    world = client.get_world()
    client.load_world('Town03') # 맵 바꾸기
    print('sucess to connect server with client') # 디버깅 확인 완료. 연결됨 

    # Retrieve the spectator object
    spectator = world.get_spectator()

    # Get the location and rotation of the spectator through its transform
    transform = spectator.get_transform()

    location = transform.location
    rotation = transform.rotation

    # Set the spectator with an empty transform
    spectator.set_transform(carla.Transform())
    # This will set the spectator at the origin of the map, with 0 degrees
    # pitch, yaw and roll - a good way to orient yourself in the map

if __name__ == "__main__":
    main()