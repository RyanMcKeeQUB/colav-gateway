from obstacleUpdate_pb2 import ObstacleUpdate as ProtobufObstaclesUpdate

obstacles_udpate = ProtobufObstaclesUpdate()

for x in range(1,5):
    print (x)

obstacles_udpate.timestamp = "24/01/2025 12:02:44"
obstacles_udpate.timestep = "00000000000001"
print ('test')