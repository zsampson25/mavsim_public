"""
point_gimbal
    - point gimbal at target
part of mavsim
    - Beard & McLain, PUP, 2012
    - Update history:  
        3/31/2022 - RWB
        7/13/2023 - RWB
"""
import numpy as np
from tools.rotations import euler_to_rotation
import parameters.camera_parameters as CAM


class Gimbal:
    def pointAtGround(self, mav):
        az_d = 0
        el_d = np.radians(-90)
        k_az = 2.0
        k_el = 2.0
        # proportional control for gimbal
        u_az = k_az * (az_d - mav.gimbal_az)
        u_el = k_el * (el_d - mav.gimbal_el)
        return( np.array([[u_az], [u_el]]) )
            
    def pointAtPosition(self, mav, target_position):
        # line-of-sight vector in the inertial frame
        # target_position is in the inertial frame
        # mav_position is in the inertial frame
        # gimbal position is in the body frame
        target_position = np.reshape(target_position, (3, 1))  # ensure column vector
        mav_position = np.array([[mav.north], [mav.east], [-mav.altitude]])
        ell_i = target_position - mav_position
        # rotate line-of-sight vector into body frame and normalize
        R = euler_to_rotation(mav.phi, mav.theta, mav.psi)
        ell_b = R @ ell_i
        ell_b = ell_b / np.linalg.norm(ell_b)
        return( self.pointAlongVector(ell_b, mav.gimbal_az, mav.gimbal_el) )

    def pointAlongVector(self, ell, azimuth, elevation):
        # point gimbal so that optical axis aligns with unit vector ell
        # ell is assumed to be aligned in the body frame
        # given current azimuth and elevation angles of the gimbal
        # compute control inputs to align gimbal
        # desired azimuth and elevation
        az_d = np.arctan2(ell[1, 0], ell[0, 0])           # equation 13.13
        el_d = np.arcsin(-ell[2, 0])                      # equation 13.14

        # simple proportional control (you can tune the gains)
        k_az = 2.0
        k_el = 2.0
        u_az = k_az * (az_d - azimuth)
        u_el = k_el * (el_d - elevation)

        return( np.array([[u_az], [u_el]]) )




