#!/usr/bin/env python
"""
Unit tests for proto_ros_converter utils
"""

from utils.proto_ros_converter_utils import ProtoToROSUtils
import unittest

from missionRequest_pb2 import MissionRequest
from obstaclesUpdate_pb2 import ObstaclesUpdate
from agentUpdate_pb2 import AgentUpdate
# import the controller feedback ting
from controllerFeedback_pb2 import ControllerFeedback

class TestProtoToROSUtils(unittest.TestCase):

    def setUp(self):
        pass

    def test_parse_mission_request(self):
        pass

    def parse_agent_proto(self):
        pass

    def parse_obstacles_proto(self):
        pass  

    