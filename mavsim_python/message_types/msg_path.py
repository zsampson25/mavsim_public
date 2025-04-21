"""
msg_path
    - messages type for input to path follower
    
part of mavsim_python
    - Beard & McLain, PUP, 2012
    - Last update:
        3/11/2019 - RWB
"""
import numpy as np


class MsgPath:
    '''
        Message class that defines a path
        'line' paths are defined by
            airspeed
            line_origin
            line_direction
        'orbit' paths are defined by
            orbit center
            orbit radius
            orbit direction
        plot_updated is for drawing purposes
    '''
    def __init__(self):
        # type='line' means straight line following, type='orbit' means orbit following
        self.type = 'line'
        #self.type = 'orbit'
        # desired airspeed along the path
        self.airspeed = float(25)
        # origin of the straight path line (r)
        self.line_origin = np.array([[0.0, 0.0, 0.0]]).T
        # direction of line -unit vector- (q)
        self.line_direction = np.array([[1.0, 0.0, 0.0]]).T
        # center of the orbit (c)
        self.orbit_center = np.array([[0.0, 0.0, 0.0]]).T
        # radius of the orbit (rho)
        self.orbit_radius = float(50)
        # orbit direction: 'CW'==clockwise, 'CCW'==counter clockwise
        self.orbit_direction = 'CW'
        # flag that indicates that path has been plotted
        self.plot_updated = bool(False)
        
    def set(self,
            type: str='line',
            airspeed: float=25,
            line_origin: np.ndarray=np.array([[0.], [0.], [0.]]),
            line_direction: np.ndarray=np.array([[1.], [0.], [0.]]),
            orbit_center: np.ndarray=np.array([[0.], [0.], [0.]]),
            orbit_radius: float=100,
            orbit_direction: str='CW',
            helix_start_angle: float=0,
            helix_climb_angle: float=0,
        ):
        if type=='line':
            self.type = type
            self.airspeed = airspeed
            self.line_origin = line_origin
            self.line_direction = line_direction/np.linalg.norm(line_direction)
            self.plot_updated=False
            return
        if type=='orbit':
            self.type = type
            self.airspeed = airspeed
            self.orbit_center = orbit_center
            self.orbit_radius = orbit_radius
            self.orbit_direction = orbit_direction
            self.plot_updated=False
            return
        if type=='helix':
            self.type = type
            self.airspeed = airspeed
            self.orbit_center = orbit_center
            self.orbit_radius = orbit_radius
            self.orbit_direction = orbit_direction
            self.helix_start_angle = helix_start_angle
            self.helix_climb_angle = helix_climb_angle
            self.plot_updated=False