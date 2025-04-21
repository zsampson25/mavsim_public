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
from message_types.msg_waypoints import MsgWaypoints
from message_types.msg_state import MsgState
from message_types.msg_path import MsgPath


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
        self.return_flag = False

    def update(self,
               waypoints: MsgWaypoints,
               state: MsgState,
               radius: float,
               ) -> MsgPath:
        if waypoints.num_waypoints == 0:
            self.manager_requests_waypoints = True
        
        if self.return_flag:
            # switch over to your home‑orbit behaviour
            self.return_home(waypoints)
            self.return_flag = False
        
        if self.manager_requests_waypoints is True \
                and waypoints.flag_waypoints_changed is True:
            self.manager_requests_waypoints = False
            
        if waypoints.num_waypoints == 2:
            self.back_and_forth(waypoints, state)
        elif waypoints.type == 'straight_line':
            self._line_manager(waypoints, state)
        elif waypoints.type == 'fillet':
            self._fillet_manager(waypoints, state, radius)

        else:
            print('Error in Path Manager: Undefined waypoint type.')
        return self._path
    
    def return_home(self, waypoints: MsgWaypoints):
        # Reset path manager pointers
        self._num_waypoints = 1
        self._ptr_previous = 0
        self._ptr_current = 0
        self._ptr_next = 0
        self._manager_state = 1

        # Set path to orbit
        self._path.set(
            type='orbit',
            airspeed=25.0,
            orbit_center=np.array([[0.0], [0.0], [-100.0]]),
            orbit_radius=200.0,
            orbit_direction=1,
        )
        self._path.plot_updated = False 

    def back_and_forth(self, waypoints: MsgWaypoints, state):
        """Straight‐line shuttle back and forth between waypoints[0] and waypoints[1]."""
        # 1) On a new set, reset shuttle direction to “forward”
        if waypoints.flag_waypoints_changed:
            waypoints.flag_waypoints_changed = False
            self._shuttle_forward = True

        waypoints.num_waypoints = 2

        # 2) Choose origin, target, airspeed
        wp0 = waypoints.ned[:, 0:1]
        wp1 = waypoints.ned[:, 1:2]
        if self._shuttle_forward:
            origin      = wp0
            target      = wp1
            airspeed    = waypoints.airspeed.item(0)
        else:
            origin      = wp1
            target      = wp0
            airspeed    = waypoints.airspeed.item(1)

        direction = (target - origin) / np.linalg.norm(target - origin)

        self._path.set(
            type='line',
            airspeed=airspeed,
            line_origin=origin,
            line_direction=direction,
        )
        self._path.plot_updated = False

        # 5) Set half‐space so that when the MAV crosses into it, we flip
        self._halfspace_n = direction
        self._halfspace_r = target

        # 6) If we’re in that half‐space, flip direction
        mav_pos = np.array([[state.north, state.east, -state.altitude]]).T
        if self._inHalfSpace(mav_pos):
            self._path.set(
                type='line',
                airspeed=airspeed,
                line_origin=target,
                line_direction=-direction,
            )
        waypoints.num_waypoints = 2
        


    def _line_manager(self,
                      waypoints: MsgWaypoints,
                      state: MsgState):
        mav_pos = np.array([[state.north, state.east, -state.altitude]]).T
        
        if waypoints.flag_waypoints_changed is True:
            self.manager_requests_waypoints = False
            waypoints.flag_waypoints_changed = False
            self._num_waypoints = waypoints.num_waypoints
            self._initialize_pointers()
            self._construct_line(waypoints)

        if self._inHalfSpace(mav_pos):
            self._increment_pointers()
            self._construct_line(waypoints)
            if self._ptr_current == 0:
                self.manager_requests_waypoints = True

    def _fillet_manager(self,
                        waypoints: MsgWaypoints,
                        state: MsgState,
                        radius: float,
                        ):
        mav_pos = np.array([[state.north, state.east, -state.altitude]]).T
        # if the waypoints have changed, update the waypoint pointer
        if waypoints.flag_waypoints_changed is True:
            waypoints.flag_manager_requests_waypoints = False
            waypoints.flag_waypoints_changed = False
            self._num_waypoints = waypoints.num_waypoints
            self._initialize_pointers()
            self._construct_fillet_line(waypoints, radius)
            self._manager_state = 1
        
        if self._manager_state == 1:
            if self._inHalfSpace(mav_pos):
                print('Curve')
                self._construct_fillet_circle(waypoints, radius)
                self._manager_state = 2
        
        elif self._manager_state == 2:
            if not self._inHalfSpace(mav_pos):
                print('HalfSpace')
                self._manager_state = 3
        
        elif self._manager_state == 3:
            if self._inHalfSpace(mav_pos):
                print('Straight')
                self._increment_pointers()
                self._construct_fillet_line(waypoints, radius)
                if self._ptr_next == 0:
                    current = waypoints.ned[:, self._ptr_current:self._ptr_current+1]
                    self._halfspace_r = current
                    self._manager_state = 4
                else:
                    self._manager_state = 1
        
        elif self._manager_state == 4:
            if self._inHalfSpace(mav_pos):
                self.manager_requests_waypoints = True
    

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
                        waypoints: MsgWaypoints,
                        ):
        # Read in Waypoint Variables
        previous = waypoints.ned[:, self._ptr_previous:self._ptr_previous+1]
        current = waypoints.ned[:, self._ptr_current:self._ptr_current+1]
        next = waypoints.ned[:, self._ptr_next:self._ptr_next+1]
        q_previous = (current - previous) / np.linalg.norm(current - previous)
        q_next = (next - current) / np.linalg.norm(next - current)
        
        # Update Path Variable
        self._path.set(
            type='line',
            airspeed=waypoints.airspeed.item(self._ptr_current),
            line_origin=previous,
            line_direction=q_previous,
        )
       
        # Update Halfspace Variables
        self._halfspace_n = (q_previous + q_next) / 2
        self._halfspace_n = self._halfspace_n / np.linalg.norm(self._halfspace_n)
        self._halfspace_r = current
        self._path.plot_updated = False
      
    def _construct_fillet_line(self,
                               waypoints: MsgWaypoints,
                               radius: float,
                               ):
        # Read in Waypoint Variables
        previous = waypoints.ned[:, self._ptr_previous:self._ptr_previous+1]
        current = waypoints.ned[:, self._ptr_current:self._ptr_current+1]
        next = waypoints.ned[:, self._ptr_next:self._ptr_next+1]
        q_previous = (current - previous) / np.linalg.norm(current - previous)
        q_next = (next - current) / np.linalg.norm(next - current)
        
        # Update Path Variable
        self._path.set(
            type='line',
            airspeed=waypoints.airspeed.item(self._ptr_current),
            line_origin=previous,
            line_direction=q_previous,
        )
        
        # Update Halfspace Variables
        theta = np.arccos(-q_previous.T @ q_next)
        self._halfspace_n = q_previous
        self._halfspace_r = current - radius / np.tan(theta / 2) * q_previous
        self._path.plot_updated = False

    def _construct_fillet_circle(self,
                                 waypoints: MsgWaypoints,
                                 radius: float):
        # Read in Waypoint Variables
        previous = waypoints.ned[:, self._ptr_previous:self._ptr_previous+1]
        current = waypoints.ned[:, self._ptr_current:self._ptr_current+1]
        next = waypoints.ned[:, self._ptr_next:self._ptr_next+1]
        q_previous = (current - previous) / np.linalg.norm(current - previous)
        q_next = (next - current) / np.linalg.norm(next - current)
        
        # Calculate the angle between the two lines
        theta = np.arccos(-q_previous.T @ q_next)
        
        # Update Path Variable
        self._path.set(
            type = 'orbit',
            airspeed=waypoints.airspeed.item(self._ptr_current),
            orbit_center=current - radius / np.sin(theta / 2.0) * (q_previous - q_next) / np.linalg.norm(q_previous - q_next),
            orbit_radius=radius,
        )
        
        # Update Halfspace Variables
        if np.sign(q_previous.item(0)*q_next.item(1) - q_previous.item(1)*q_next.item(0)) > 0:
            self._path.orbit_direction = 'CW'
        else:
            self._path.orbit_direction = 'CCW'
        self._halfspace_n = q_next
        self._halfspace_r = current + radius / np.tan(theta / 2.0) * q_next
        self._path.plot_updated = False
    
    def _inHalfSpace(self, 
                     pos: np.ndarray)->bool:
        '''Is pos in the half space defined by r and n?'''
        pos_half_space = (pos - self._halfspace_r).T @ self._halfspace_n
        if np.all(pos_half_space >= 0): 
            return True
        else:
            return False
    
