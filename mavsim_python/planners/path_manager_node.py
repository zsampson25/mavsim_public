import numpy as np

import rclpy
from rclpy.node import Node

# Examples of importing a message types
from rosplane_msgs.msg import State, Waypoint, CurrentPath

# Useful when the message type might be changed
WaypointsType = Waypoint
EstimatedStateType = State
CurrentPathType = CurrentPath

# Topic names
WaypointsTopic = 'new_waypoints'
EstimatedStateTopic = 'state'
CurrentPathTopic = 'current_path'

from rosplane.path_manager import PathManager



class PathManagerNode(Node):

    def __init__(self):
        super().__init__('path_manager')

        # Create the path publisher
        self.current_path_publisher = self.create_publisher(CurrentPathType, CurrentPathTopic, 10)
 

        # Create estimated state subscriber
        self.estimated_state_subscription = self.create_subscription(
            EstimatedStateType,
            EstimatedStateTopic,
            self.state_callback,
            10)
        self.estimated_state_subscription  # prevent annoying warnings

        # Create new waypoints subscriber
        self.new_waypoints_subscription = self.create_subscription(
            WaypointsType,
            WaypointsTopic,
            self.new_waypoints_callback,
            10)
        self.new_waypoints_subscription  # prevent annoying warnings

        self.state = EstimatedStateType()
        self.waypoints = WaypointsType()
        self.path_manager = PathManager()
        

    def state_callback(self, msg):
        self.state = msg
        radius = 100
        path = self.path_manager.update(self.state, radius)
        self.command_publisher.publish(path)
        

    def new_waypoints_callback(self, msg):
        self.path_manager.waypoints_list.append(msg.w)
        
        




def main(args=None):
    rclpy.init(args=args)
    node = PathManagerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
