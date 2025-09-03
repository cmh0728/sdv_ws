import carla
import random
import time

def main():
    # Connect to the client and retrieve the world object
    # client = carla.Client('192.168.86.74', 2000) # 기본2000번 포트, 다 사용가능.동민이형 
    client = carla.Client('localhost', 2000) # 기본2000번 포트, 다 사용가능.

    world = client.get_world()
    client.load_world('Town03') # 맵 바꾸기
    print('sucess to connect server with client') # 디버깅 확인 완료. 연결됨 


    ################# switching perspective of server#########################
    # # Retrieve the spectator object
    # spectator = world.get_spectator()

    # # Get the location and rotation of the spectator through its transform
    # transform = spectator.get_transform()

    # location = transform.location
    # rotation = transform.rotation

    # # Set the spectator with an empty transform
    # spectator.set_transform(carla.Transform())
    # # This will set the spectator at the origin of the map, with 0 degrees
    # # pitch, yaw and roll - a good way to orient yourself in the map
    ###########################################################################

    ###################### add NPC ############################################
    # Get the blueprint library and filter for the vehicle blueprints 
    vehicle_blueprints = world.get_blueprint_library().filter('*vehicle*') # 차량 선택해야 함

    # Get the map's spawn points (Ego차량 스폰 지점 설정)
    spawn_points = world.get_map().get_spawn_points()

    # Spawn 50 vehicles randomly distributed throughout the map 
    # for each spawn point, we choose a random vehicle from the blueprint library
    for i in range(0,50):
        world.try_spawn_actor(random.choice(vehicle_blueprints), random.choice(spawn_points))
    
    ego_vehicle = world.spawn_actor(random.choice(vehicle_blueprints), random.choice(spawn_points))


    ###########################################################################

if __name__ == "__main__":
    main()