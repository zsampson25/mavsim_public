# path planner for mavsim_python
#
# mavsim_python
#     - Beard & McLain, PUP, 2012
#     - Last updated:
#         4/3/2019 - RWB
#         3/30/2022 - RWB
#         7/13/2023 - RWB
#         4/1/2024 - RWB
import numpy as np
from message_types.msg_waypoints import MsgWaypoints
from message_types.msg_state import MsgState
from message_types.msg_world_map import MsgWorldMap
from planners.rrt_straight_line_test import RRTStraightLine
# from planners.rrt_star_straight_line import RRTStar


class PathPlanner:
    def __init__(self, type):  # 'rrt_straight', 'simple_straight', 'rrt_star'
        # waypoints definition
        self.waypoints = MsgWaypoints()
        if type == 'rrt_straight':
            self.rrt_straight_line = RRTStraightLine()

        # if type == 'rrt_star':
        #     self.rrt_star = RRTStar()
        self._type = type

    def update(self, 
               world_map: MsgWorldMap, 
               state: MsgState, 
               radius: float):
        print('planning...')
        if self._type == 'simple_straight':
            Va = 25
            self.waypoints.type = 'fillet'
            self.waypoints.add(np.array([[0, 0, -100]]).T, Va, np.inf, np.inf, 0, 0)
            #self.waypoints.add(np.array([[1000, 0, -100]]).T, Va, np.inf, np.inf, 0, 0)
            #self.waypoints.add(np.array([[0, 1000, -100]]).T, Va, np.inf, np.inf, 0, 0)
            self.waypoints.add(np.array([[1000, 1000, -100]]).T, Va, np.inf, np.inf, 0, 0)

        elif self._type == 'rrt_straight':
            self.waypoints.type = 'fillet'
            desired_airspeed = 25
            desired_altitude = 100
            # start pose is current pose
            start_pose = np.array([[state.north], 
                                   [state.east], 
                                   [-desired_altitude]])
            # desired end pose
            if np.linalg.norm(start_pose[0:2]) < world_map.city_width / 2:
                end_pose = np.array([[world_map.city_width], 
                                     [world_map.city_width],
                                     [-desired_altitude]])
            else:  # or to the bottom-left corner of world_map
                end_pose = np.array([[0], [0], [-desired_altitude]])
            self.waypoints = self.rrt_straight_line.update(
                start_pose, 
                end_pose, 
                desired_airspeed, 
                world_map, 
                radius)
            self.waypoints_not_smooth = self.rrt_straight_line.waypoints_not_smoothed
            self.tree = self.rrt_straight_line.tree
        
        # elif self._type == 'rrt_star':
        #     self.waypoints.type = 'fillet'
        #     desired_airspeed = 25
        #     desired_altitude = 100
        #     # start pose is current pose
        #     start_pose = np.array([[state.north], 
        #                            [state.east], 
        #                            [-desired_altitude]])
        #     # desired end pose
        #     if np.linalg.norm(start_pose[0:2]) < world_map.city_width / 2:
        #         end_pose = np.array([[world_map.city_width], 
        #                              [world_map.city_width],
        #                              [-desired_altitude]])
        #     else:  # or to the bottom-left corner of world_map
        #         end_pose = np.array([[0], [0], [-desired_altitude]])
        #     self.waypoints = self.rrt_star.update(
        #         start_pose, 
        #         end_pose, 
        #         desired_airspeed, 
        #         world_map, 
        #         radius)
        #     self.waypoints_not_smooth = self.rrt_star.waypoints_not_smoothed
        #     self.tree = self.rrt_star.tree
        
        else:
            print("Error in Path Planner: Undefined planner type.")
        self.waypoints.plot_updated = False
        print('...done planning.')
        return self.waypoints
