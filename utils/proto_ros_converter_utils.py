from agentUpdate_pb2 import AgentUpdate as ProtoAgentUpdate
from obstaclesUpdate_pb2 import ObstaclesUpdate as ProtoObstaclesUpdate

from colav_interfaces.msg import ObstaclesConfig as ROSObstaclesUpdate
from colav_interfaces.msg import AgentConfig as ROSAgentUpdate

from colav_interfaces.msg import DynamicObstacleConfig
from colav_interfaces.msg import StaticObstacleConfig
from colav_interfaces.msg import MissionRequest as ROSMissionRequest

from missionRequest_pb2 import MissionRequest as ProtobufMissionRequest

from colav_interfaces.msg import Vessel
from colav_interfaces.msg import VesselConstraints
from colav_interfaces.msg import VesselGeometry

from geometry_msgs.msg import Pose, Point, Quaternion, Polygon, Point32

class ProtoToROSUtils: 
    @staticmethod
    def parse_mission_request(msg: bytes) -> ROSMissionRequest:
        """Parse mission request protobuf to ros"""
        try: 
            protobuf_mission_request = ProtobufMissionRequest()
            protobuf_mission_request.ParseFromString(msg)

            return ROSMissionRequest(
                mission_tag = protobuf_mission_request.tag,
                mission_sent_timestamp = protobuf_mission_request.mission_start_timestamp,
                vessel = ProtoToROSUtils.parse_vessel(protobuf_mission_request.vessel),
                mission_init_position = ProtoToROSUtils.parse_point(protobuf_mission_request.mission_init_position),
                mission_goal_position = ProtoToROSUtils.parse_point(protobuf_mission_request.mission_goal_position),
            )
        except Exception as e: 
            raise ValueError(f"Error parsing mission request: {e}") from e

    @staticmethod
    def parse_point(point):
        """Parse a protobuf point to ros."""
        try: 
            return Point32(
                x=point.x,
                y=point.x,
                z=point.z
            )
        except Exception as e: 
            raise ValueError("Error parsing point") from e

    @staticmethod
    def parse_vessel(vessel):
        """Parse vessel protobuf to ros"""
        return Vessel(
            tag = vessel.tag,
            type = ProtobufMissionRequest.Vessel.VesselType.Name(
                vessel.type
            ),
            dynamic_constraints = VesselConstraints(
                max_acceleration = vessel.vessel_constraints.max_acceleration,
                max_deceleration = vessel.vessel_constraints.max_deceleration,
                max_velocity = vessel.vessel_constraints.max_velocity,
                min_velocity = vessel.vessel_constraints.min_velocity,
                max_yaw_rate = vessel.vessel_constraints.max_yaw_rate,
            ),
            geometry = VesselGeometry(
                polyshape = ProtoToROSUtils._parse_polygon(vessel.vessel_geometry.polyshape_points),
                acceptance_radius = vessel.vessel_geometry.safety_threshold
            )
        )
    @staticmethod
    def parse_agent_proto(msg: bytes) -> ROSAgentUpdate:
        """Parse agent configuration protobuf to ros"""
        try:
            protobuf_agent_update = ProtoAgentUpdate()
            protobuf_agent_update.ParseFromString(msg)

            return ROSAgentUpdate(
                agent_tag=protobuf_agent_update.agent_tag,
                pose=ProtoToROSUtils._parse_pose(protobuf_agent_update.state.pose),
                velocity=protobuf_agent_update.state.velocity,
                acceleration=protobuf_agent_update.state.acceleration,
                yaw_rate=protobuf_agent_update.state.yaw_rate,
                timestamp=protobuf_agent_update.timestamp,
                timestep=protobuf_agent_update.timestep
            )
        except Exception as e:
            raise ValueError(f"Error parsing agent protobuf: {e}") from e

    @staticmethod
    def parse_obstacles_proto(msg: bytes) -> ROSObstaclesUpdate:
        """Parse Obstacle update received via protobuf and publish it to ros topic"""
        try:
            protobuf_obstacles_update = ProtoObstaclesUpdate()
            protobuf_obstacles_update.ParseFromString(msg)

            return ROSObstaclesUpdate(
                dynamic_obstacles=ProtoToROSUtils._parse_dynamic_obstacles(protobuf_obstacles_update.dynamic_obstacles),
                static_obstacles=ProtoToROSUtils._parse_static_obstacles(protobuf_obstacles_update.static_obstacles),
                timestamp=protobuf_obstacles_update.timestamp,
                timestep=protobuf_obstacles_update.timestep
            )
        except Exception as e:
            raise ValueError(f"Error parsing obstacles protobuf: {e}") from e

    @staticmethod
    def _parse_dynamic_obstacles(dynamic_obstacles) -> list[DynamicObstacleConfig]:
        """Convert dynamic obstacles from protobuf to ROS"""
        return [
            DynamicObstacleConfig(
                id=dynamic_obstacle.id.tag,
                type=ProtoObstaclesUpdate.ObstacleType.Name(dynamic_obstacle.id.type),
                pose=ProtoToROSUtils._parse_pose(dynamic_obstacle.state.pose),
                velocity=dynamic_obstacle.state.velocity,
                yaw_rate=dynamic_obstacle.state.yaw_rate,
                geometry=ProtoToROSUtils._parse_geometry(dynamic_obstacle.geometry),
                safety_radius=dynamic_obstacle.geometry.acceptance_radius
            )
            for dynamic_obstacle in dynamic_obstacles
        ]

    @staticmethod
    def _parse_static_obstacles(static_obstacles) -> list[StaticObstacleConfig]:
        """Convert static obstacles from protobuf to ROS"""
        return [
            StaticObstacleConfig(
                id=static_obstacle.id.tag,
                type=ProtoObstaclesUpdate.ObstacleType.Name(static_obstacle.id.type),
                pose=ProtoToROSUtils._parse_pose(static_obstacle.pose),
                geometry=ProtoToROSUtils._parse_geometry(static_obstacle.geometry),
                safety_radius=static_obstacle.geometry.acceptance_radius
            )
            for static_obstacle in static_obstacles
        ]

    @staticmethod
    def _parse_pose(pose) -> Pose:
        """Convert protobuf pose to ROS msg"""
        return Pose(
            position=Point(x=pose.position.x, y=pose.position.y, z=pose.position.z),
            orientation=Quaternion(x=pose.orientation.x, y=pose.orientation.y, z=pose.orientation.z, w=pose.orientation.w)
        )

    @staticmethod
    def _parse_geometry(geometry) -> Polygon:
        """Convert protobuf geometry to ROS Polygon"""
        points = [Point32(x=point.position.x, y=point.position.y, z=point.position.z) for point in geometry.polyshape_points]
        return Polygon(points=points)
    
    def _parse_polygon(points) -> Polygon:
        points = [Point32(x=point.x, y=point.y, z=point.z) for point in points]
        return Polygon(points=points)