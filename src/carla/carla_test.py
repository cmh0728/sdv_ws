import carla
import random

def main():
    # Connect to the client and retrieve the world object
    client = carla.Client('localhost', 2000)
    world = client.get_world()
    print('sucess to connect server with client')

if __name__ == "main":
    main()