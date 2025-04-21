# rrt straight line path planner for mavsim_python
import numpy as np
from message_types.msg_waypoints import MsgWaypoints
import random

class RRTStraightLine:
    def __init__(self):
        self.segment_length = 300  # standard length of path segments
        self.max_iterations = 1000
        self.goal_threshold = 150  # how close to get to goal
        self.tree = MsgWaypoints()
        self.waypoints_not_smoothed = MsgWaypoints()

    def update(self, start_pose, end_pose, Va, world_map, radius):
        self.tree = MsgWaypoints()
        self.tree.type = 'fillet'
        
        # Add start pose to the tree
        self.tree.add(ned=start_pose, airspeed=Va, cost=0, parent=-1, connect_to_goal=0)
        
        # Check if direct connection is possible
        if not self.collision(start_pose, end_pose, world_map):
            self.tree.add(ned=end_pose, airspeed=Va, 
                         cost=np.linalg.norm(end_pose - start_pose), 
                         parent=0, connect_to_goal=1)
            waypoints_not_smoothed = self.find_minimum_path(self.tree, end_pose)
            waypoints = self.smooth_path(waypoints_not_smoothed, world_map)
            self.waypoints_not_smoothed = waypoints_not_smoothed
            return waypoints
        
        # Main RRT loop
        for i in range(self.max_iterations):
            # Random sampling (with bias toward goal)
            if random.random() < 0.1:  # 10% chance to sample goal directly
                rand_pose = end_pose
            else:
                rand_pose = self.random_pose(world_map, start_pose[2,0])
            
            # Find nearest node in tree
            nearest_idx, nearest_node = self.find_nearest_node(rand_pose)
            
            # Create new node in direction of random point
            direction = rand_pose - nearest_node
            distance = np.linalg.norm(direction)
            if distance == 0:
                continue
            direction = direction / distance
            
            new_pose = nearest_node + min(self.segment_length, distance) * direction
            
            # Collision check
            if not self.collision(nearest_node, new_pose, world_map):
                # Add to tree
                cost = self.tree.cost[nearest_idx] + np.linalg.norm(new_pose - nearest_node)
                connect_to_goal = 0
                
                # Check if new node connects to goal
                if not self.collision(new_pose, end_pose, world_map):
                    connect_to_goal = 1
                
                self.tree.add(ned=new_pose, airspeed=Va, 
                             cost=cost, parent=nearest_idx, 
                             connect_to_goal=connect_to_goal)
                
                # If connected to goal, find path
                if connect_to_goal == 1:
                    # Add goal node
                    goal_cost = cost + np.linalg.norm(end_pose - new_pose)
                    self.tree.add(ned=end_pose, airspeed=Va,
                                cost=goal_cost, parent=self.tree.num_waypoints-1,
                                connect_to_goal=1)
                    
                    # Find and smooth path
                    waypoints_not_smoothed = self.find_minimum_path(self.tree, end_pose)
                    waypoints = self.smooth_path(waypoints_not_smoothed, world_map)
                    self.waypoints_not_smoothed = waypoints_not_smoothed
                    return waypoints
        
        # If no path found after max iterations, return straight line (even if it collides)
        waypoints = MsgWaypoints()
        waypoints.add(ned=start_pose, airspeed=Va)
        waypoints.add(ned=end_pose, airspeed=Va)
        waypoints.type = 'fillet'
        return waypoints

    def find_nearest_node(self, pose):
        if self.tree.num_waypoints == 0:
            return None, None
        
        distances = []
        for i in range(self.tree.num_waypoints):
            node = self.tree.ned[:,i].reshape(-1,1)
            distances.append(np.linalg.norm(pose - node))
        
        nearest_idx = np.argmin(distances)
        nearest_node = self.tree.ned[:,nearest_idx].reshape(-1,1)
        return nearest_idx, nearest_node

    def random_pose(self, world_map, pd):
        # generate a random pose within city limits
        pn = world_map.city_width * np.random.rand()
        pe = world_map.city_width * np.random.rand()
        pose = np.array([[pn], [pe], [pd]])
        return pose

    def collision(self, start_pose, end_pose, world_map):
        # check if path between poses collides with buildings
        points = self.points_along_path(start_pose, end_pose, 20)
        for i in range(points.shape[1]):
            if self.height_above_ground(world_map, points[:,i].reshape(-1,1)) <= 0:
                return True
        return False

    def height_above_ground(self, world_map, point):
        # find the altitude of point above ground level
        point_height = -point.item(2)
        tmp = np.abs(point.item(0)-world_map.building_north)
        d_n = np.min(tmp)
        idx_n = np.argmin(tmp)
        tmp = np.abs(point.item(1)-world_map.building_east)
        d_e = np.min(tmp)
        idx_e = np.argmin(tmp)
        if (d_n<world_map.building_width) and (d_e<world_map.building_width):
            map_height = world_map.building_height[idx_n, idx_e]
        else:
            map_height = 0
        h_agl = point_height - map_height
        return h_agl

    def points_along_path(self, start_pose, end_pose, N):
        # returns points along path separated by Del
        points = start_pose
        q = (end_pose - start_pose)
        L = np.linalg.norm(q)
        if L == 0:
            return start_pose
        q = q / L
        w = start_pose
        for i in range(1, N):
            w = w + (L / N) * q
            points = np.append(points, w, axis=1)
        return points

    def find_minimum_path(self, tree, end_pose):
        # find the lowest cost path to the end node
        connecting_nodes = []
        for i in range(tree.num_waypoints):
            if tree.connect_to_goal.item(i) == 1:
                connecting_nodes.append(i)
        
        if not connecting_nodes:
            # No path found, return direct connection
            waypoints = MsgWaypoints()
            waypoints.add(ned=tree.ned[:,0].reshape(-1,1), airspeed=tree.airspeed[0])
            waypoints.add(ned=end_pose, airspeed=tree.airspeed[0])
            return waypoints
        
        # find minimum cost last node
        idx = np.argmin(tree.cost[connecting_nodes])
        
        # construct lowest cost path order
        path = [connecting_nodes[idx]]  # last node that connects to end node
        parent_node = tree.parent.item(connecting_nodes[idx])
        while parent_node >= 0:
            path.insert(0, int(parent_node))
            parent_node = tree.parent.item(int(parent_node))
        
        # construct waypoint path
        waypoints = MsgWaypoints()
        for i in path:
            waypoints.add(ned=tree.ned[:,i].reshape(-1,1),
                          airspeed=tree.airspeed.item(i),
                          cost=tree.cost.item(i),
                          parent=tree.parent.item(i),
                          connect_to_goal=tree.connect_to_goal.item(i))
        
        # Add goal node
        waypoints.add(ned=end_pose,
                      airspeed=tree.airspeed[-1],
                      cost=tree.cost[-1],
                      parent=tree.parent[-1],
                      connect_to_goal=tree.connect_to_goal[-1])
        waypoints.type = tree.type
        return waypoints

    def smooth_path(self, waypoints, world_map):
        if waypoints.num_waypoints < 3:
            return waypoints
        
        smooth_waypoints = MsgWaypoints()
        smooth_waypoints.type = waypoints.type
        
        # Start with first waypoint
        smooth_waypoints.add(ned=waypoints.ned[:,0].reshape(-1,1), 
                            airspeed=waypoints.airspeed[0])
        
        i = 0
        while i < waypoints.num_waypoints - 1:
            j = waypoints.num_waypoints - 1
            while j > i + 1:
                start = waypoints.ned[:,i].reshape(-1,1)
                end = waypoints.ned[:,j].reshape(-1,1)
                if not self.collision(start, end, world_map):
                    break
                j -= 1
            smooth_waypoints.add(ned=waypoints.ned[:,j].reshape(-1,1),
                                airspeed=waypoints.airspeed[j])
            i = j
        
        return smooth_waypoints