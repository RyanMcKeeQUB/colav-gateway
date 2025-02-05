from obstaclesUpdate_pb2 import ObstaclesUpdate as ProtobufObstaclesUpdate
import socket

obstacles_update = ProtobufObstaclesUpdate()
obstacles_update.mission_tag = "MOCK_MISSION"

# add dynamic obstacles
for x in range(0,4):
    obstacles_update.dynamic_obstacles.add()
    obstacles_update.dynamic_obstacles[x].id.tag = f"dynamic_obstacle_{x}"
    obstacles_update.dynamic_obstacles[x].id.type = ProtobufObstaclesUpdate.ObstacleType.VESSEL

    obstacles_update.dynamic_obstacles[x].state.pose.position.x = float(10)
    obstacles_update.dynamic_obstacles[x].state.pose.position.y = float(12)
    obstacles_update.dynamic_obstacles[x].state.pose.position.z = float(1)

    obstacles_update.dynamic_obstacles[x].state.pose.orientation.x = float(1)
    obstacles_update.dynamic_obstacles[x].state.pose.orientation.y = float(2)
    obstacles_update.dynamic_obstacles[x].state.pose.orientation.z = float(3)
    obstacles_update.dynamic_obstacles[x].state.pose.orientation.w = float(4)

    obstacles_update.dynamic_obstacles[x].geometry.acceptance_radius = float(5)
    
    for i in range(1,10):
        obstacles_update.dynamic_obstacles[x].geometry.polyshape_points.add()
        obstacles_update.dynamic_obstacles[x].geometry.polyshape_points[i-1].position.x = i+5
        obstacles_update.dynamic_obstacles[x].geometry.polyshape_points[i-1].position.y = i+2
        obstacles_update.dynamic_obstacles[x].geometry.polyshape_points[i-1].position.z = i-1

        obstacles_update.dynamic_obstacles[x].geometry.polyshape_points[i-1].orientation.x = i/2
        obstacles_update.dynamic_obstacles[x].geometry.polyshape_points[i-1].orientation.y = i/2
        obstacles_update.dynamic_obstacles[x].geometry.polyshape_points[i-1].orientation.z = i/2
        obstacles_update.dynamic_obstacles[x].geometry.polyshape_points[i-1].orientation.w = i/2

    obstacles_update.dynamic_obstacles[x].state.velocity = float(15.0)
    obstacles_update.dynamic_obstacles[x].state.yaw_rate = float(0.2)

# add static obstacles

for x in range(0,5): 
    obstacles_update.static_obstacles.add()
    obstacles_update.static_obstacles[x].id.tag = f"static_obstacle_{x}"
    obstacles_update.static_obstacles[x].id.type = ProtobufObstaclesUpdate.ObstacleType.VESSEL

    obstacles_update.static_obstacles[x].pose.position.x = float(10)
    obstacles_update.static_obstacles[x].pose.position.y = float(12)
    obstacles_update.static_obstacles[x].pose.position.z = float(1)

    obstacles_update.static_obstacles[x].pose.orientation.x = float(1)
    obstacles_update.static_obstacles[x].pose.orientation.y = float(2)
    obstacles_update.static_obstacles[x].pose.orientation.z = float(3)
    obstacles_update.static_obstacles[x].pose.orientation.w = float(4)

    obstacles_update.static_obstacles[x].geometry.acceptance_radius = float(5)
    
    for i in range(1,10):
        obstacles_update.static_obstacles[x].geometry.polyshape_points.add()
        obstacles_update.static_obstacles[x].geometry.polyshape_points[i-1].position.x = i+5
        obstacles_update.static_obstacles[x].geometry.polyshape_points[i-1].position.y = i+2
        obstacles_update.static_obstacles[x].geometry.polyshape_points[i-1].position.z = i-1

        obstacles_update.static_obstacles[x].geometry.polyshape_points[i-1].orientation.x = i/2
        obstacles_update.static_obstacles[x].geometry.polyshape_points[i-1].orientation.y = i/2
        obstacles_update.static_obstacles[x].geometry.polyshape_points[i-1].orientation.z = i/2
        obstacles_update.static_obstacles[x].geometry.polyshape_points[i-1].orientation.w = i/2

obstacles_update.timestamp = "24/01/2025 12:02:44"
obstacles_update.timestep = "00000000000001"

data = obstacles_update.SerializeToString()

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

import time
while True:
    sock.sendto(data, ('0.0.0.0', 7200))
    time.sleep(5.0)