# from typing import Type
# from rclpy.type_support import Srv, SrvRequestT, SrvResponseT
from typing import Optional
import os
import uuid
import rclpy
from rclpy.node import Node
from uav.utils import camel_to_snake
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from abc import ABC, abstractmethod

class ScoringNode(Node):
    """
    Base class for ROS 2 nodes that
    """

    @classmethod
    def node_name(cls):
        return camel_to_snake(cls.__name__)

    @classmethod
    def service_name(cls):
        return f'world/{cls.node_name()}'
    
    @classmethod
    def __str__(cls):
        return cls.node_name()

    def __init__(self):
        """
        Initialize the WorldNode.

        Args:

        """
        super().__init__(self.__class__.__name__)
        
        
