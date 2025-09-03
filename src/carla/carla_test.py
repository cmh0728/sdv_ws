import carla
import random

def main():
    # Connect to the client and retrieve the world object
    client = carla.Client('192.168.86.74', 2000) # 기본2000번 포트, 다 사용가능.
    world = client.get_world()
    client.load_world('Town05')
    print('sucess to connect server with client') # 디버깅 확인 완료. 연결됨 

if __name__ == "__main__":
    main()