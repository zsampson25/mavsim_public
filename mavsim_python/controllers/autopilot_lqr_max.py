"""
autopilot block for mavsim_python
    - Beard & McLain, PUP, 2012
    - Last Update:
        2/10/22 - RWB
"""
import numpy as np
from numpy import array, sin, cos, radians, concatenate, zeros, diag
from scipy.linalg import solve_continuous_are, inv
import parameters.control_parameters as AP
from tools.wrap import wrap
import models.model_coef as M
from message_types.msg_state import MsgState
from message_types.msg_delta import MsgDelta

def saturate(input, low_limit, up_limit):
    if input <= low_limit:
        output = low_limit
    elif input >= up_limit:
        output = up_limit
    else:
        output = input
    return output


class Autopilot:
    def __init__(self, ts_control):
        self.Ts = ts_control
        # initialize integrators and delay variables
        self.integratorCourse = 0
        self.integratorAltitude = 0
        self.integratorAirspeed = 0
        self.errorCourseD1 = 0
        self.errorAltitudeD1 = 0
        self.errorAirspeedD1 = 0
        
        # compute LQR gains
        CrLat = array([[0., 0., 0., 0., 1.0]]) # or is it all zeros?
        AAlat = concatenate((
                    concatenate((M.A_lat, zeros((5,1))), axis=1),
                    concatenate((CrLat, zeros((1,1))), axis=1)),
                    axis=0)
        BBlat = concatenate((M.B_lat, zeros((1,2))), axis=0)
        Qlat = diag([0.001, 0.01, 0.1, 100.0, 1.0, 100.0]) # v, p, r, phi, chi, intChi
        Rlat = diag([1.0, 1.0]) # a, r
        Plat = solve_continuous_are(AAlat, BBlat, Qlat, Rlat)
        self.Klat = inv(Rlat) @ BBlat.T @ Plat
        CrLon = array([[0., 0., 0., 0., 1.0], [1.0/AP.Va0, 1.0/AP.Va0, 0., 0., 0.]])
        AAlon = concatenate((
                    concatenate((M.A_lon, zeros((5,2))), axis=1),
                    concatenate((CrLon, zeros((2,2))), axis=1)),
                    axis=0)
        BBlon = concatenate((M.B_lon, zeros((2, 2))), axis=0)
        Qlon = diag([10.0, 10.0, 0.001, 0.001, 10.0, 100.0, 100.0]) # u, w, q, theta, h, intH, intVa
        Rlon = diag([1.0, 1.0])  # e, t
        Plon = solve_continuous_are(AAlon, BBlon, Qlon, Rlon)
        self.Klon = inv(Rlon) @ BBlon.T @ Plon
        self.commanded_state = MsgState()

    def update(self, cmd, state):
        # lateral autopilot
        errorAirspeed = state.Va - cmd.airspeed_command
        chi_c = wrap(cmd.course_command, state.chi)
        errorCourse = saturate(state.chi - chi_c, -radians(15), radians(15))
        self.integratorCourse += (self.Ts / 2.0) * (errorCourse + self.errorCourseD1)
        self.errorCourseD1 = errorCourse

        xLat = array([[errorAirspeed * sin(state.beta)],
                      [state.p],
                      [state.r],
                      [state.phi],
                      [errorCourse],
                      [self.integratorCourse]])
        
        temp = -self.Klat @ xLat
        delta_a = saturate(temp.item(0), -radians(30), radians(30))
        delta_r = saturate(temp.item(1), -radians(30), radians(30))

        # longitudinal autopilot
        altitude_c = saturate(cmd.altitude_command, state.altitude - 0.2*AP.altitude_zone, state.altitude + 0.2*AP.altitude_zone)
        errorAltitude = state.altitude - altitude_c
        self.integratorAltitude = self.integratorAltitude + (self.Ts / 2.0) * (errorAltitude + self.errorAltitudeD1)
        self.errorAltitudeD1 = errorAltitude
       
        self.integratorAirspeed = self.integratorAirspeed + (self.Ts / 2.0) * (errorAirspeed + self.errorAirspeedD1)
        self.errorAirspeedD1 = errorAirspeed

        xLon = array([[errorAirspeed * cos(state.alpha)],
                      [errorAirspeed * sin(state.alpha)],
                      [state.q],
                      [state.theta],
                      [errorAltitude],
                      [self.integratorAltitude],
                      [self.integratorAirspeed]])
        
        temp = -self.Klon @ xLon
        delta_e = saturate(temp.item(0), -radians(30), radians(30))
        delta_t = saturate(temp.item(1), 0.0, 1.0)

        # construct control outputs and commanded states
        delta = MsgDelta(elevator=delta_e,
                         aileron=delta_a,
                         rudder=delta_r,
                         throttle=delta_t)
        self.commanded_state.altitude = cmd.altitude_command
        self.commanded_state.Va = cmd.airspeed_command
        self.commanded_state.phi = state.phi       #state.phi
        self.commanded_state.theta = state.theta     #state.theta
        self.commanded_state.chi = state.chi       #state.chi
        return delta, self.commanded_state

"""
autopilot block for mavsim_python
    - Beard & McLain, PUP, 2012
    - Last Update:
        2/10/22 - RWB

import numpy as np
from numpy import array, sin, cos, radians, concatenate, zeros, diag
from scipy.linalg import solve_continuous_are, inv
import parameters.control_parameters as AP
from tools.wrap import wrap
import models.model_coef as M
from message_types.msg_state import MsgState
from message_types.msg_delta import MsgDelta

def saturate(input, low_limit, up_limit):
    if input <= low_limit:
        output = low_limit
    elif input >= up_limit:
        output = up_limit
    else:
        output = input
    return output


class Autopilot:
    def __init__(self, ts_control):
        self.Ts = ts_control
        # initialize integrators and delay variables
        self.integratorCourse = 0
        self.integratorAltitude = 0
        self.integratorAirspeed = 0
        self.errorCourseD1 = 0
        self.errorAltitudeD1 = 0
        self.errorAirspeedD1 = 0
        # compute LQR gains
        CrLat = array([[0., 0., 0., 0., 1.0]])

        AAlat = concatenate((
                    concatenate((M.A_lat, zeros((5,1))), axis=1),
                    concatenate((CrLat, zeros((1,1))), axis=1)),
                    axis=0)
        BBlat = concatenate((M.B_lat, zeros((1,2))), axis=0)
        Qlat = diag([0.001, 0.01, 0.1, 100.0, 1.0, 100.0]) # v, p, r, phi, chi, intChi
        Rlat = diag([1.0, 1.0]) # a, r
        Plat = solve_continuous_are(AAlat, BBlat, Qlat, Rlat)
        # Plat = Plon = np.zeros((6,6))
        self.Klat = inv(Rlat) @ BBlat.T @ Plat
        CrLon = array([[0., 0., 0., 0., 1.0], [1/AP.Va0, 1/AP.Va0, 0., 0., 0.]])
        AAlon = concatenate((
                    concatenate((M.A_lon, zeros((5,2))), axis=1),
                    concatenate((CrLon, zeros((2,2))), axis=1)),
                    axis=0)
        BBlon = concatenate((M.B_lon, zeros((2, 2))), axis=0)
        Qlon = diag([10.0, 10.0, .001, .01, 10.0, 100.0, 100.0]) # u, w, q, theta, h, intH, intVa
        Rlon = diag([1.0, 1.0])  # e, t
        print("AAlon shape:", AAlon.shape)
        print("BBlon shape:", BBlon.shape)
        print("Qlon shape:", Qlon.shape)
        print("Rlon shape:", Rlon.shape)
        eig_AAlon = np.linalg.eigvals(AAlon)
        print("Eigenvalues of AAlon:", eig_AAlon)
        eig_Qlon = np.linalg.eigvals(Qlon)
        print("Eigenvalues of Qlon:", eig_Qlon)
        eig_Rlon = np.linalg.eigvals(Rlon)
        print("Eigenvalues of Rlon:", eig_Rlon)
        from numpy.linalg import matrix_rank

        Ctrb_matrix = np.hstack([BBlon, AAlon @ BBlon, AAlon @ AAlon @ BBlon])
        rank_Ctrb = matrix_rank(Ctrb_matrix)

        print("Controllability matrix rank:", rank_Ctrb)
        print("Expected rank:", AAlon.shape[0])  # Should be equal to n (full rank)


        Plon = solve_continuous_are(AAlon, BBlon, Qlon, Rlon)
        # Plon = np.zeros((7,7))
        self.Klon = inv(Rlon) @ BBlon.T @ Plon
        
        self.commanded_state = MsgState()

    def update(self, cmd, state):
        # lateral autopilot
        errorAirspeed = state.Va - cmd.airspeed_command
        chi_c = wrap(cmd.course_command, state.chi)
        errorCourse = saturate(chi_c - state.chi, radians(-15), radians(15))
        self.integratorCourse += (self.Ts / 2) * (errorCourse + self.errorCourseD1)
        self.errorCourseD1 = errorCourse
        xLat = array([[errorAirspeed* sin(state.beta)],
                      [state.p],
                      [state.r],
                      [state.phi],
                      [errorCourse],
                      [self.integratorCourse]])
        tmp = -self.Klat @ xLat
        delta_a = saturate(tmp.item(0), -radians(30), radians(30))
        delta_r = saturate(tmp.item(1), -radians(30), radians(30))


        # longitudinal autopilot
        altitude_c = saturate(cmd.altitude_command,
                              state.altitude - 0.2*AP.altitude_zone,
                              state.altitude + 0.2*AP.altitude_zone)
        errorAltitude = state.altitude - altitude_c
        self.integratorAltitude += (self.Ts / 2) * (errorAltitude + self.errorAltitudeD1)
        self.errorAltitudeD1 = errorAltitude
        self.integratorAirspeed += (self.Ts / 2) * (errorAirspeed + self.errorAirspeedD1)
        self.errorAirspeedD1 = errorAirspeed
        xlon = array([[errorAirspeed*cos(state.alpha)],
                      [errorAirspeed*sin(state.alpha)],
                      [state.q],
                      [state.theta],
                      [errorAltitude],
                      [self.integratorAltitude],
                      [self.integratorAirspeed]])
        tmp = -self.Klon @ xlon
        delta_e = saturate(tmp.item(0), radians(-30), radians(30))
        delta_t = saturate(tmp.item(1), 0.0, 1.0)



        # construct control outputs and commanded states
        delta = MsgDelta(elevator=delta_e,
                         aileron=delta_a,
                         rudder=delta_r,
                         throttle=delta_t)
        self.commanded_state.altitude = cmd.altitude_command
        self.commanded_state.Va = cmd.airspeed_command
        self.commanded_state.phi = state.phi
        self.commanded_state.theta = state.theta
        self.commanded_state.chi = chi_c
        return delta, self.commanded_state
"""