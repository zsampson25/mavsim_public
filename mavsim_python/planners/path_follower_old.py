import numpy as np
from math import sin, cos
from message_types.msg_state import MsgState
from message_types.msg_path import MsgPath
from message_types.msg_autopilot import MsgAutopilot
from message_types.msg_autopilot import MsgAutopilot
from tools.wrap import wrap


class PathFollower:
    def __init__(self):
        ##### TODO #####
        self.chi_inf = np.radians(50)  # approach angle for large distance from straight-line path
        self.k_path = 0.05  # path gain for straight-line path following
        self.k_orbit = 5  # path gain for orbit following
        self.gravity = 9.81  # gravity constant
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
        ##### TODO #####
        rn = path.line_origin[0,0]
        re = path.line_origin[1,0]
        rd = path.line_origin[2,0]
        r = np.array(path.line_origin[0,0],
                      path.line_origin[1,0],)

        pn = state.north
        pe = state.east
        pd = state.altitude
        p = np.array([[pn], [pe], [pd]])
        chi = state.chi

        qn = path.line_direction[0,0]
        qe = path.line_direction[1,0]
        qd = path.line_direction[2,0]
        q = np.array([qn, qe, qd])
        q = q / np.linalg.norm(q)

        ep_i = p - r
        ki = np.array([0, 0, 1])
        n = np.cross(ki , q) / np.linalg.norm(np.cross(ki, q))
        s = ep_i - np.dot(ep_i, n) * n
    

        #airspeed command
        self.autopilot_commands.airspeed_command = path.airspeed

        # course command
        chi_q = np.arctan2(qe, qn)
        while chi_q - chi < -np.pi:
            chi_q += 2 * np.pi
        while chi_q - chi > np.pi:
            chi_q -= 2 * np.pi

        ep_y = -np.sin(chi_q) * (p[0]- r[0]) + np.cos(chi_q) * (p[1]- r[1])
        self.autopilot_commands.course_command = chi_q - self.chi_inf * 2/np.pi * np.arctan(self.k_path * ep_y)
        # altitude command
        self.autopilot_commands.altitude_command = -rd - np.sqrt(s[0]**2 + s[1]**2) * (qd / np.sqrt(qn**2 + qe**2))

        # feedforward roll angle for straight line is zero
        self.autopilot_commands.phi_feedforward = 0

    def _follow_orbit(self, 
                      path: MsgPath, 
                      state: MsgState):
        ##### TODO #####
        # airspeed command
        self.autopilot_commands.airspeed_command = 0

        # course command
        self.autopilot_commands.course_command = 0

        # altitude command
        self.autopilot_commands.altitude_command = 0
        
        # roll feedforward command
        self.autopilot_commands.phi_feedforward = 0




