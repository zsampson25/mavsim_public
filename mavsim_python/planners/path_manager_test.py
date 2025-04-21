"""
mavsim_python: drawing tools
    - Beard & McLain, PUP, 2012
    - Update history:
        4/15/2019 - RWB
        3/30/2022 - RWB
        7/13/2023 - RWB
        3/25/2024 - RWB
"""
from IPython.core.debugger import set_trace

import numpy as np
#from planners.dubins_parameters import DubinsParameters
from message_types.msg_state import MsgState
from message_types.msg_path import MsgPath
from message_types.msg_waypoints import MsgWaypoints


class PathManager:

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
        #self.dubins_path = DubinsParameters()

    def update(self,
               waypoints: MsgWaypoints,
               state: MsgState,
               radius: float,
               ) -> MsgPath:
        if waypoints.num_waypoints == 0:
            self.manager_requests_waypoints = True
        if self.manager_requests_waypoints is True \
                and waypoints.flag_waypoints_changed is True:
            self.manager_requests_waypoints = False
        if waypoints.type == 'straight_line':
            self._line_manager(waypoints, state)
        elif waypoints.type == 'fillet':
            self._fillet_manager(waypoints, state, radius)
        #elif waypoints.type == 'dubins':
            #elf._dubins_manager(waypoints, state, radius)
        else:
            print('Error in Path Manager: Undefined waypoint type.')
        return self._path

    def _line_manager(self,
                      waypoints: MsgWaypoints,
                      state: MsgState):
        mav_pos = np.array([[state.north, state.east, -state.altitude]]).T
        # if the waypoints have changed, update the waypoint pointer
        if waypoints.flag_waypoints_changed is True:
            waypoints.flag_manager_requests_waypoints = False
            waypoints.flag_waypoints_changed = False
            self._num_waypoints = waypoints.num_waypoints
            self._initialize_pointers()
            self._construct_line(waypoints)
        # entered into the half plane separating waypoint segments
        if self._inHalfSpace(mav_pos):
            self._increment_pointers()
            self._construct_line(waypoints)
            # requests new waypoints when reach end of current list
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
        # state machine for fillet path
        if self._manager_state == 1:
            # follow straight line path from previous to current
            if self._inHalfSpace(mav_pos):
                # entered into the half plane H1±
                self._construct_fillet_circle(waypoints, radius)
                self._manager_state = 2
        elif self._manager_state == 2:
            # follow start orbit until out of H2
            if not self._inHalfSpace(mav_pos):
                self._manager_state = 3
        elif self._manager_state == 3:
            # follow orbit from previous->current to current->next
            if self._inHalfSpace(mav_pos):
                # entered into the half plane H2
                self._increment_pointers()
                # requests new waypoints when reach end of current list
                self._construct_fillet_line(waypoints, radius)
                if self._ptr_next == 0:
                    current = waypoints.ned[:, self._ptr_current:self._ptr_current+1]
                    self._halfspace_r = current
                    self._manager_state = 4
                else:
                    self._manager_state = 1
        elif self._manager_state == 4:
            # this state is for the last waypoint.
            # It flies to the end and then request a new plan
            if self._inHalfSpace(mav_pos):
                self.manager_requests_waypoints = True

    ''' DUBINS
    def _dubins_manager(self,
                        waypoints: MsgWaypoints,
                        state: MsgState,
                        radius: float,
                        ):
        mav_pos = np.array([[state.north, state.east, -state.altitude]]).T
        close_distance = 10
        # if the waypoints have changed, update the waypoint pointer
        if waypoints.flag_waypoints_changed is True:
            waypoints.flag_waypoints_changed = False
            self._num_waypoints = waypoints.num_waypoints
            self._initialize_pointers()
            # dubins path parameters
            self.dubins_path.update(
                ps=waypoints.ned[:, self._ptr_previous:self._ptr_previous+1],
                chis=waypoints.course.item(self._ptr_previous),
                pe=waypoints.ned[:, self._ptr_current:self._ptr_current+1],
                chie=waypoints.course.item(self._ptr_current),
                R=radius)
            self._construct_dubins_circle_start(waypoints, self.dubins_path)
            if np.linalg.norm(self.dubins_path.p_s - self.dubins_path.r1) < close_distance:
                self._construct_dubins_line(waypoints, self.dubins_path)
                self._manager_state = 3
            elif self._inHalfSpace(mav_pos):
                self._manager_state = 1
            else:
                self._manager_state = 2
        # state machine for dubins path
        if self._manager_state == 1:
            # skip the first circle if distance along circle is small
            if ((np.linalg.norm(self.dubins_path.p_s - self.dubins_path.r1) < close_distance)
                    # follow start orbit until out of H1
                    or not self._inHalfSpace(mav_pos)):
                self._manager_state=2
        elif self._manager_state == 2:
            # skip the first circle if distance along circle is small
            if ((np.linalg.norm(self.dubins_path.p_s - self.dubins_path.r1) < close_distance)
                    # follow start orbit until cross into H1
                    or self._inHalfSpace(mav_pos)):
                self._construct_dubins_line(waypoints, self.dubins_path)
                self._manager_state = 3
        elif self._manager_state == 3:
            # skip line if it is short
            if ((np.linalg.norm(self.dubins_path.r1 - self.dubins_path.r2) < close_distance)
                    or self._inHalfSpace(mav_pos)):
                self._construct_dubins_circle_end(waypoints, self.dubins_path)
                if self._inHalfSpace(mav_pos):
                    self._manager_state = 4
                else:
                    self._manager_state = 5
        elif self._manager_state == 4:
            # distance along end orbit is small
            if ((np.linalg.norm(self.dubins_path.r2 - self.dubins_path.p_e) < close_distance)
                    # follow start orbit until out of H3
                    or not self._inHalfSpace(mav_pos)):
                self._manager_state = 5
        elif self._manager_state == 5:
            # skip circle if small
            if ((np.linalg.norm(self.dubins_path.r2 - self.dubins_path.p_e) < close_distance)
                    # follow start orbit until cross into H3
                    or self._inHalfSpace(mav_pos)):
                self._increment_pointers()
                set_trace()
                self.dubins_path.update(
                    waypoints.ned[:, self._ptr_previous:self._ptr_previous+1],
                    waypoints.course.item(self._ptr_previous),
                    waypoints.ned[:, self._ptr_current:self._ptr_current+1],
                    waypoints.course.item(self._ptr_current),
                    radius)
                self._construct_dubins_circle_start(waypoints, self.dubins_path)
                self._manager_state = 1
                # requests new waypoints when reach end of current list
                if self._ptr_current == 0:
                    self.manager_requests_waypoints = True
    '''
    
    def _initialize_pointers(self):
        if self._num_waypoints >= 3:
            self._ptr_previous = 0
            self._ptr_current = 1
            self._ptr_next = 2
        else:
            print('Error Path Manager: need at least three waypoints')

    def _increment_pointers(self):
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
        beta = np.arccos(-q_previous.T @ q_next)
        self._halfspace_n = q_previous
        self._halfspace_r = current - radius / np.tan(beta/2) * q_previous
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
        
        # Calculate the angle and a tmp value
        varrho = np.arccos(-q_previous.T @ q_next)
        q_tmp = (q_previous - q_next) / np.linalg.norm(q_previous - q_next)
        
        # Update Path Variable
        self._path.set(
            type = 'orbit',
            airspeed=waypoints.airspeed.item(self._ptr_current),
            orbit_center=current-radius/np.sin(varrho/2.0)*q_tmp,
            orbit_radius=radius,
        )
        
        # Update Halfspace Variables
        if np.sign(q_previous.item(0)*q_next.item(1) - q_previous.item(1)*q_next.item(0)) > 0:
            # TODO: change msg type to use integer +-1, instead of string, b/c path_follower uses +-1
            self._path.orbit_direction = 'CW'
        else:
            self._path.orbit_direction = 'CCW'
        self._halfspace_n = q_next
        self._halfspace_r = current + radius / np.tan(varrho/2.0) * q_next
        self._path.plot_updated = False

    def _inHalfSpace(self,
                     pos: np.ndarray)->bool:
        '''Is pos in the half space defined by r and n?'''
        pos_half_space = (pos-self._halfspace_r).T @ self._halfspace_n
        # set_trace()
        half_bool = np.all(pos_half_space >= 0)
        if  half_bool:
            return True
        else:
            return False
'''
    def _construct_dubins_circle_start(self,
                                       waypoints: MsgWaypoints,
                                       dubins_path: DubinsParameters):
        if dubins_path.dir_s == 1:
            direction = 'CW'
        else:
            direction = 'CCW'
        self._path.set(
            type='orbit',
            airspeed=waypoints.airspeed.item(self._ptr_current),
            orbit_radius=dubins_path.radius,
            orbit_center=dubins_path.center_s,
            orbit_direction=direction,
        )
        self._halfspace_n = dubins_path.n1
        self._halfspace_r = dubins_path.r1
        self._path.plot_updated = False

    def _construct_dubins_line(self,
                               waypoints: MsgWaypoints,
                               dubins_path: DubinsParameters):
        self._path.set(
            type='line',
            airspeed=waypoints.airspeed.item(self._ptr_current),
            line_origin=dubins_path.r1,
            line_direction=dubins_path.n1,
        )
        self._halfspace_n = dubins_path.n1
        self._halfspace_r = dubins_path.r2
        self._path.plot_updated = False

    def _construct_dubins_circle_end(self,
                                     waypoints: MsgWaypoints,
                                     dubins_path: DubinsParameters):
        if dubins_path.dir_e == 1:
            dir = 'CW'
        else:
            dir = 'CCW'
        self._path.set(
            type='orbit',
            airspeed=waypoints.airspeed.item(self._ptr_current),
            orbit_radius=dubins_path.radius,
            orbit_center=dubins_path.center_e,
            orbit_direction=dir,
        )
        self._halfspace_n = dubins_path.n3
        self._halfspace_r = dubins_path.r3
        self._path.plot_updated = False

'''