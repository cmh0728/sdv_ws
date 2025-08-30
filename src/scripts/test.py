from kuksa_client.grpc import VSSClient

with VSSClient('127.0.0.1', 55556) as client:
    client.setValue('Vehicle.Speed', 42)
    val = client.getValue('Vehicle.Speed')
    print(val)

