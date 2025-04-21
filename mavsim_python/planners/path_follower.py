import numpy as np
from math import sin, cos
from message_types.msg_state import MsgState
from message_types.msg_path import MsgPath
from message_types.msg_autopilot import MsgAutopilot
from tools.wrap import wrap


class PathFollower:
    def __init__(self):
        ##### TODO #####
        self.chi_inf = np.radians(50)  # approach angle for large distance from straight-line path
        self.k_path = 0.05          # path gain for straight-line path following
        self.k_orbit = 5.0          # path gain for orbit following
        self.gravity = 9.81
        self.autopilot_commands = MsgAutopilot()  # message sent to autopilot

    def update(self,
               path: MsgPath,
               state: MsgState)->MsgAutopilot:
        if path.type == 'line':
            self._follow_straight_line(path, state)
        elif path.type == 'orbit':
            self._follow_orbit(path, state)
        return self.autopilot_commands

    def _follow_straight_line(self,
                              path: MsgPath,
                              state: MsgState):

        #### GENERAL STUFF START ####

        # path definition
        r = np.array([path.line_origin[0,0],
                      path.line_origin[1,0],
                      path.line_origin[2,0]])
        q = np.array([path.line_direction[0, 0],
                      path.line_direction[1, 0],
                      path.line_direction[2, 0]])
       
        # TODO: does this need to be normalized?
        q = q / np.linalg.norm(q)  

        # MAV position
        p = np.array([state.north,
                      state.east,
                      state.altitude])
       
        # course (X)
        chi = state.chi

        #### GENERAL STUFF END ####
       
        #### ALTITUDE STUFF START ####

        # projection (s) of the relative error vector
        e_pi = p - r
        k_i = np.array([0, 0, 1])

        # error catching
        #print("k_i shape: ", k_i.shape)
        #print("q shape: ", q.flatten().shape)

        n = np.cross(k_i, q) / np.linalg.norm(np.cross(k_i, q))
        s = e_pi - np.dot(e_pi, n) * n

        # command altitude using equation 10.5
        h_command = -r[2] - np.sqrt(s[0]**2 + s[1]**2) * q[2] / np.sqrt(q[0]**2 + q[1]**2)

        #### ALTITUDE STUFF END ####

        #### COURSE STUFF START ####

        # command course
        chi_q = np.arctan2(q[1], q[0])
        while chi_q - chi < -np.pi:
            chi_q += 2 * np.pi
        while chi_q - chi > np.pi:
            chi_q -= 2 * np.pi

        # e_py
        e_py = -np.sin(chi_q) * (p[0] - r[0]) + np.cos(chi_q) * (p[1] - r[1])

        # command course computation w/ equ. 10.8

        #### COURSE STUFF END ####
        chi_command = chi_q - self.chi_inf * (2.0 / np.pi) * np.arctan(self.k_path * e_py)

        #airspeed command
        self.autopilot_commands.airspeed_command = path.airspeed        #### TODO ####
       
        self.autopilot_commands.course_command = chi_command            #### TODO ####  
        #print("course command: ", self.autopilot_commands.course_command)

        # altitude command
        self.autopilot_commands.altitude_command = h_command            #### TODO ####
        #print("altitude command: ", self.autopilot_commands.altitude_command)

        # feedforward roll angle for straight line is zero
        self.autopilot_commands.phi_feedforward = 0.0                   #### TODO ####
        #print("phi feedforward: ", self.autopilot_commands.phi_feedforward)

    def _follow_orbit(self,
                      path: MsgPath,
                      state: MsgState):
        #### GENERAL STUFF START ####
        c = np.array([path.orbit_center[0, 0],
                      path.orbit_center[1, 0],
                      path.orbit_center[2, 0]]) # orbit center (cn, ce, cd)
       
        rho = path.orbit_radius # orbit radius
        lambda_orbit = 1 if path.orbit_direction == 'CW' else -1 # orbit direction
       
        # MAV position
        p = np.array([state.north,
                      state.east,
                      state.altitude])
       
        chi = state.chi # course (X)
       
        h_command = -c[2] # altitude command

        # Find the distance from the MAV to the orbit center
        d = np.sqrt((p[0] - c[0])**2 + (p[1] - c[1])**2)

        # Find the angle from the orbit center to the MAV
        #theta = np.arctan((p[1] - c[1], p[0] - c[0]))

        theta = wrap(np.arctan2(p[1] - c[1], p[0] - c[0]), 2 * np.pi)

        # Wrap
        while theta - chi < -np.pi:
            theta += 2 * np.pi
        while theta - chi > np.pi:
            theta -= 2 * np.pi

        # Find the compute command course angle from equation 10.14
        chi_command = theta + lambda_orbit * ((np.pi / 2.0) + np.arctan(self.k_orbit * (d - rho) / rho))
       
        # Compute ground speed components
        wn, we = state.wn, state.we
        Vg_x = state.Vg * np.cos(chi) + wn
        Vg_y = state.Vg * np.sin(chi) + we
        Vg = np.sqrt(Vg_x**2 + Vg_y**2)

        # Wind correctionf actors
        wind_correction_x = we * np.cos(chi) - wn * np.sin(chi)
        wind_correction_y = wn * np.cos(chi) + we * np.sin(chi)
        wind_correction = np.sqrt(1 - (wind_correction_x / state.Va)**2)

        # Find feedforward roll angle
        feedforward_command = lambda_orbit * np.arctan((Vg**2 / (self.gravity * rho * wind_correction)))

        # airspeed command
        self.autopilot_commands.airspeed_command = path.airspeed

        # course command
        self.autopilot_commands.course_command = chi_command

        # altitude command
        self.autopilot_commands.altitude_command = h_command
       
        # roll feedforward command
        self.autopilot_commands.phi_feedforward = feedforward_command


#### path output ####
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
#### state output #####
# state.(something)
# something could be: north, east, altitude, phi, theta, psi, Va, alpha, beta, p, q, r, Vg, gamma, chi, wn, we, bx, by, bz, camera_az, camera_el