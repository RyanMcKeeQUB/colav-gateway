from proto_gen.missionRequest_pb2 import MissionRequest
import socket


mission_request = MissionRequest()
mission_request.tag = "MISSION"
mission_request.mission_start_timestamp = "12345"

mission_request.vessel.tag = "EF12_24"
mission_request.vessel.type = (
    MissionRequest.Vessel.VesselType.HYDROFOIL
)

mission_request.vessel.vessel_constraints.max_acceleration = 2.0
mission_request.vessel.vessel_constraints.max_deceleration = -1.0
mission_request.vessel.vessel_constraints.max_velocity = 30.0
mission_request.vessel.vessel_constraints.min_velocity = 15.0
mission_request.vessel.vessel_constraints.max_yaw_rate = 0.2

mission_request.vessel.vessel_geometry.safety_threshold = 5

x = 1
y = 1
z = 0
# Define the polyshape of the vessel
for i in range(1, 5):
    point = mission_request.vessel.vessel_geometry.polyshape_points.add()
    point.x = float(x)
    point.y = float(y)
    point.z = float(z)
    x += 1
    y += 1

mission_request.mission_init_position.x = float(1.0)
mission_request.mission_init_position.y = float(1.0)
mission_request.mission_init_position.z = float(0.0)

mission_request.mission_goal_position.x = float(1.0)
mission_request.mission_goal_position.z = float(1.0)
mission_request.mission_goal_position.y = float(1.0)

mission_request.mission_goal_acceptance_radius = float(5.0)

serialized_msg = mission_request.SerializeToString()

print(serialized_msg)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(serialized_msg, ("0.0.0.0", 7000))
