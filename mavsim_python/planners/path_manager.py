"""
mavsim_python: drawing tools
    - Beard & McLain, PUP, 2012
    - Update history:
        4/15/2019 - RWB
        3/30/2022 - RWB
        7/13/2023 - RWB
        3/25/2024 - RWB
"""

import numpy as np
from planners.dubins_parameters import DubinsParameters
from message_types.msg_state import MsgState
from message_types.msg_path import MsgPath
from message_types.msg_waypoints import MsgWaypoints


class PathManager:
    '''
        Path manager

        Attributes
        ----------
        path : MsgPath
            path message sent to path follower
        num_waypoints : int
            number of waypoints
        ptr_previous : int
            pointer to previous waypoint
            MAV is traveling from previous to current waypoint
        ptr_current : int
            pointer to current waypoint
        ptr_next : int
            pointer to next waypoint
        halfspace_n : np.nparray (3x1)
            the normal vector that defines the current halfspace plane
        halfspace_r : np.nparray (3x1)
            the inertial vector that defines a point on the current halfspace plane
        manager_state : int
            state of the manager state machine
        manager_requests_waypoints : bool
            a flag for handshaking with the path planner
            True when new waypoints are needed, i.e., at the end of waypoint list.
        dubins_path : DubinsParameters
            A class that defines a dubins path      

        Methods
        -------
        update(waypoints, radius, state)

        _initialize_pointers() :
            initialize the points to 0(previous), 1(current), 2(next)  
        _increment_pointers() :  
            add one to every pointer - currently does it modulo num_waypoints          
        _inHalfSpace(pos):
            checks to see if the position pos is in the halfspace define

        _line_manager(waypoints, state):
            Assumes straight-line paths.  Transition is from one line to the next
            _construct_line(waypoints): 
                used by line manager to construct the next line path

        _fillet_manager(waypoints, radius, state):
            Assumes straight-line waypoints.  Constructs a fillet turn between lines.
            _construct_fillet_line(waypoints, radius):
                used by _fillet_manager to construct the next line path
            _construct_fillet_circle(waypoints, radius):
                used by _fillet_manager to construct the fillet orbit
            
        _dubins_manager(waypoints, radius, state):
            Assumes dubins waypoints.  Constructs Dubin's path between waypoints
            _construct_dubins_circle_start(waypoints, dubins_path):
                used by _dubins_manager to construct the start orbit
            _construct_dubins_line(waypoints, dubins_path):
                used by _dubins_manager to construct the middle line
            _construct_dubins_circle_end(waypoints, dubins_path):
                used by _dubins_manager to construct the end orbit
    '''
    def __init__(self):
        self._path = MsgPath()
        self._num_waypoints = 0
        self._ptr_previous = 0
        self._ptr_current = 1
        self._ptr_next = 2
        self._halfspace_n = np.inf * np.ones((3,1))
        self._halfspace_r = np.inf * np.ones((3,1))
        self._manager_state = 1
        self.manager_requests_waypoints = True
        self.dubins_path = DubinsParameters()


    def update(self, 
               waypoints: MsgWaypoints, 
               radius: float, 
               state: MsgState) -> MsgPath:
        if waypoints.num_waypoints == 0:
            self.manager_requests_waypoints = True
        if self.manager_requests_waypoints is True \
                and waypoints.flag_waypoints_changed is True:
            self.manager_requests_waypoints = False
        if waypoints.type == 'straight_line':
            self._line_manager(waypoints, state)
        elif waypoints.type == 'fillet':
            self._fillet_manager(waypoints, radius, state)
        elif waypoints.type == 'dubins':
            self._dubins_manager(waypoints, radius, state)
        else:
            print('Error in Path Manager: Undefined waypoint type.')
        return self._path

    def _line_manager(self,  
                      waypoints: MsgWaypoints, 
                      state: MsgState):
        mav_pos = np.array([[state.north, state.east, -state.altitude]]).T
        # if the waypoints have changed, update the waypoint pointer
        if waypoints.flag_waypoints_changed is True:
            waypoints.flag_waypoints_changed = False
            self._num_waypoints = waypoints.num_waypoints
            self._initialize_pointers()
            self._construct_line(waypoints)

        if self._inHalfSpace(mav_pos):
            # if the MAV is in the halfspace, increment the pointers
            self._increment_pointers()
            # construct the next line
            self._construct_line(waypoints)
            if self._ptr_current == 0:
                # if the pointer is at the end of the list, request new waypoints
                self.manager_requests_waypoints = True
        
        ##### TODO ######
        # Use functions - self._initialize_pointers(), self._construct_line()
        # self._inHalfSpace(mav_pos), self._increment_pointers(), self._construct_line()

        # Use variables - self._ptr_current, self.manager_requests_waypoints,
        # waypoints.__, radius

    def _fillet_manager(self,  
                            waypoints: MsgWaypoints,
                            radius: float,
                            state: MsgState):
            mav_pos = np.array([[state.north, state.east, -state.altitude]]).T
        
            if waypoints.flag_waypoints_changed is True:
                waypoints.flag_waypoints_changed = False
                self._num_waypoints = waypoints.num_waypoints
                self._initialize_pointers()
                self._construct_fillet_line(waypoints, radius)
                self._manager_state = 1
        
            if self._manager_state == 1:  # Following a straight-line path
                if self._inHalfSpace(mav_pos):
                    print('Curve')
                    self._manager_state = 2
                    self._construct_fillet_circle(waypoints, radius)
        
            elif self._manager_state == 2:  # Starting fillet turn and verify that the MAV has left the halfspace            
                if not self._inHalfSpace(mav_pos):
                    print('HalfSpace')
                    self._manager_state = 3
            
            elif self._manager_state == 3:  # Following a straight line path
                if self._inHalfSpace(mav_pos):
                    print('Straight')
                    self._increment_pointers()
                    self._construct_fillet_line(waypoints, radius)
                    if self._ptr_next == 0:
                        current = waypoints.ned[:, self._ptr_current:self._ptr_current+1]
                        self._halfspace_r = current
                        self.manager_state = 4
                    else:
                        self.manager_state = 1

            elif self._manager_state == 4:  # Following a straight line path
                if self._inHalfSpace(mav_pos):
                    self.manager_requests_waypoints = True
                   

        

    def _dubins_manager(self,  
                        waypoints: MsgWaypoints, 
                        radius: float, 
                        state: MsgState):
        mav_pos = np.array([[state.north, state.east, -state.altitude]]).T
        # if the waypoints have changed, update the waypoint pointer

        ##### TODO #####
        # Use functions - self._initialize_pointers(), self._dubins_path.update(),
        # self._construct_dubins_circle_start(), self._construct_dubins_line(),
        # self._inHalfSpace(), self._construct_dubins_circle_end(), self._increment_pointers(),

        # Use variables - self._num_waypoints, self._dubins_path, self._ptr_current,
        # self._ptr_previous, self._manager_state, self.manager_requests_waypoints,
        # waypoints.__, radius


    def _initialize_pointers(self):
        if self._num_waypoints >= 3:
            ##### TODO #####
            self._ptr_previous = 0
            self._ptr_current = 1
            self._ptr_next = 2
        else:
            print('Error Path Manager: need at least three waypoints')

    def _increment_pointers(self):
        ##### TODO #####
        self._ptr_previous = (self._ptr_previous + 1) % self._num_waypoints
        self._ptr_current = (self._ptr_current + 1) % self._num_waypoints
        self._ptr_next = (self._ptr_next + 1) % self._num_waypoints

    def _construct_line(self, 
                        waypoints: MsgWaypoints):
        previous = waypoints.ned[:, self._ptr_previous:self._ptr_previous+1]
        ##### TODO #####
        current = waypoints.ned[:, self._ptr_current:self._ptr_current+1]
        next = waypoints.ned[:, self._ptr_next:self._ptr_next+1]

        # q stuff
        q_previous = (current - previous) / np.linalg.norm(current - previous)
        q_next = (next - current) / np.linalg.norm(next - current)
        self._path.line_origin = previous
        self._path.line_direction = q_previous
        self._path.airspeed = waypoints.airspeed[self._ptr_current]
        self._path.type = 'line'



        # update halfspace variables
        self._halfspace_n = ((q_previous + q_next) / 2.0)
        self._halfspace_n = self._halfspace_n / np.linalg.norm(self._halfspace_n)
        self._halfspace_r = current
        
        # Update path variables

    def _construct_fillet_line(self,
                               waypoints: MsgWaypoints,
                               radius: float):
        previous = waypoints.ned[:, self._ptr_previous:self._ptr_previous+1]
        current = waypoints.ned[:, self._ptr_current:self._ptr_current+1]
        next_wp = waypoints.ned[:, self._ptr_next:self._ptr_next+1]

        q_previous = (current - previous) / np.linalg.norm(current - previous)
        q_next = (next_wp - current) / np.linalg.norm(next_wp - current)
       
        angle = np.arccos(-q_previous.T @ q_next)

        # Update the halfspace variables
        # n = vector
        # r = point on the plane
        #z = current - (radius / np.tan(angle / 2.0)) * q_previous      # what the book says
        z = current + (radius / np.sin(angle / (2.0))) * q_next        

        self._halfspace_n = q_previous
        self._halfspace_r = z
       
        # Update path variables
        self._path.type = 'line'
        self._path.line_origin = previous
        self._path.line_direction = q_previous
        self._path.airspeed = waypoints.airspeed[self._ptr_current]
       
    def _construct_fillet_circle(self,
                                 waypoints: MsgWaypoints,
                                 radius: float):
        wi_next = waypoints.ned[:, self._ptr_previous:self._ptr_previous+1]
        wi_current = waypoints.ned[:, self._ptr_current:self._ptr_current+1]
        wi_previous = waypoints.ned[:, self._ptr_next:self._ptr_next+1]

        q_previous = (wi_current - wi_previous) / np.linalg.norm(wi_current - wi_previous)
        q_current = (wi_next - wi_current) / np.linalg.norm(wi_next - wi_current)
       
        angle = np.arccos(-q_previous.T @ q_current)

        center = wi_current - (radius / np.sin(angle / 2.0)) * ((q_previous - q_current) / np.linalg.norm(q_previous - q_current))
        #z = wi_current + (radius / np.sin(angle / (2.0))) * q_current           # what the book says
        z = wi_current - (radius / np.sin(angle / 2.0)) * q_previous
       
        # print('z', z)
        # print('center', center)
        # print('z_previous', wi_current - (radius / np.tan(angle / 2.0)) * q_previous)
        # print('angle', angle)
        # print('radius', radius)

        # Update halfspace variables
        # n = vector
        # r = point on the plane
        self._halfspace_n = q_current  
        self._halfspace_r = z
       
        # Update path variables
        self._path.type = 'orbit'
        self._path.orbit_center = center
        self._path.orbit_radius = radius
        if np.sign(np.cross(q_previous.flatten(), q_current.flatten())[2]) == 1:
            self._path.orbit_direction = 'CCW'  #CW orig
        else:
            self._path.orbit_direction = 'CW'   #CCW orig
        self._path.airspeed = waypoints.airspeed[self._ptr_current]
       

    def _construct_dubins_circle_start(self, 
                                       waypoints: MsgWaypoints, 
                                       dubins_path: DubinsParameters):
        ##### TODO #####
        # update halfspace variables
        # self._halfspace_n =
        # self._halfspace_r = 
        
        # Update path variables
        # self._path.__ =
        pass

    def _construct_dubins_line(self, 
                               waypoints: MsgWaypoints, 
                               dubins_path: DubinsParameters):
        ##### TODO #####
        # update halfspace variables
        # self._halfspace_n =
        # self._halfspace_r = 
        
        # Update path variables
        # self._path.__ =
        pass

    def _construct_dubins_circle_end(self, 
                                     waypoints: MsgWaypoints, 
                                     dubins_path: DubinsParameters):
        ##### TODO #####
        # update halfspace variables
        # self._halfspace_n =
        # self._halfspace_r = 
        
        # Update path variables
        # self._path.__ =
        pass

    def _inHalfSpace(self, 
                     pos: np.ndarray)->bool:
        # '''Is pos in the half space defined by r and n?'''):
        pos_half_space = (pos - self._halfspace_r).T @ self._halfspace_n
        if np.all(pos_half_space >= 0):
            return True
        else:
            return False

