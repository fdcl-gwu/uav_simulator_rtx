from matrix_utils import hat, vee, deriv_unit_vector, saturate
from integral_utils import IntegralError, IntegralErrorVec3

import datetime
import numpy as np
import pdb


class Control:
    """Controller for the UAV trajectory tracking.

    This class detemines the control outputs for a quadrotor UAV given it's
    current state and the desired states.

    Attributes:
        t0: (datetime object) time at epoch
        t: (float) current time since epoch [s]
        t_prev: (float) time since epoch in the previous loop [s]
        dt: (float) time difference between two calls to the controller [s]

        x: (3x1 numpy array) current position of the UAV [m]
        v: (3x1 numpy array) current velocity of the UAV [m/s]
        a: (3x1 numpy array) current acceleration of the UAV [m/s^s]
        R: (3x3 numpy array) current attitude of the UAV in SO(3)
        W: (3x1 numpy array) current angular velocity of the UAV [rad/s]

        xd: (3x1 numpy array) desired position of the UAV [m]
        xd_dot: (3x1 numpy array) desired velocity of the UAV [m/s]
        xd_2dot: (3x1 numpy array) desired accleration of the UAV [m/s^2]
        xd_3dot: (3x1 numpy array) desired third derivative of the UAV 
            position [m/s^3]
        xd_4dot: (3x1 numpy array) desired fourth derivative of the UAV 
            position [m/s^4]

        b1d: (3x1 numpy array) desired direction of the first body axis
        b1d_dot: (3x1 numpy array) first time derivative of the desired  
            direction of the first body axis
        b1d_2dot: (3x1 numpy array) second time desired direction of the first 
            body axis

        Wd: (3x1 numpy array) desired body angular velocity [rad/s]
        Wd_dot: (3x1 numpy array) desired body angular acceleration [rad/s^2]

        Rd: (3x3 numpy array) desired attitude in SO(3)

        b3d: (3x1 numpy array) desired direction of the third body axis
        b3d_dot: (3x1 numpy array)  first time derivative of the desired  
            direction of the third body axis
        b3d_2dot: (3x1 numpy array)  second time desired direction of the third 
            body axis

        b1c: (3x1 numpy array) calculated direction of the first body axis
        wc3: (float) angular velocity around third body axis [rad/s]
        wc3_dot: (float) angular acceleration around third body axis [rad/s^s]

        use_integral: (bool) flag to enable/disable integral control 

        e1 = (3x1 numpy array) direction of the e1 axis
        e2 = (3x1 numpy array) direction of the e2 axis
        e3 = (3x1 numpy array) direction of the e3 axis

        m: (float) mass of the rover [kg]
        g: (float) gravitational acceleration [m/s^2]
        c_tf: (float) torsional moment generated by the propellers
        l: (float) length of the rover arm [m]
        J: (3x3 numpy array) inertia matrix of the rover

        kR: (3x3 numpy array) attitude gains
        kW: (3x3 numpy array) angular rate gains

        ky: (float) yaw gain for decoupled-yaw controller
        kwy: (float) yaw angular rate gain for decoupled-yaw controller

        kX: (3x3 numpy array) position gains
        kV: (3x3 numpy array) velocity gains

        kIR: (float) attitude integral gain
        ki: (float) position integral gain
        kI: (float) ttitude integral gain for roll and pitch
        kyI: (float) attitude integral gain for yaw
        kIX: (float) position integral gains
        c1: (float) parameters for decoupled-yaw integral control
        c2: (float) parameters for decoupled-yaw integral control
        c3: (float) parameters for decoupled-yaw integral control

        fM: (4x1 numpy array) force-moment vector
        f_total: (float) calculated forces required by each moter

        fM_to_forces: (4x4 numpy array) force to force-moment conversion
            matrix
        fM_to_forces_inv: (4x4 numpy array) force-moment to force conversion
            matrix

        eIX: (IntegralErrorVec3) position integral error
        eIR: (IntegralErrorVec3) attitude integral error
        eI1: (IntegralError) attitude integral error for roll axis
        eI2: (IntegralError) attitude integral error for pitch axis
        eIy: (IntegralError) attitude integral error for yaw axis
        eIX: (IntegralError) position integral error

        sat_sigma
    """

    def __init__(self):
        """
        Controller for the UAV trajectory tracking.

        This class detemines the control outputs for a quadrotor UAV given it's
        current state and the desired states.
        """

        ## time variables
        
        self.t0 = datetime.datetime.now()
        """time at epoch"""
        
        self.t = 0.0
        """current time since epoch [s]"""
        
        self.t_pre = 0.0
        """time since epoch in the previous loop [s]"""
        
        self.dt = 1e-9
        """time difference between two calls to the controller [s]"""

      
        ## current state"""
        
        self.x = np.zeros(3)
        """current position of the UAV [m]"""
        
        self.v = np.zeros(3)
        """current velocity of the UAV [m/s]"""
        
        self.a = np.zeros(3)
        """current acceleration of the UAV [m/s^s]"""
        
        self.R = np.identity(3)
        """current attitude of the UAV in SO(3)"""
        
        self.W = np.zeros(3)
        """current angular velocity of the UAV [rad/s]"""

      
        ## desired states
        
        self.xd = np.zeros(3)
        """desired position of the UAV [m]"""
        
        self.xd_dot = np.zeros(3)
        """desired velocity of the UAV [m/s]"""
        
        self.xd_2dot = np.zeros(3)
        """desired accleration of the UAV [m/s^2]"""
        
        self.xd_3dot = np.zeros(3)
        """desired third derivative of the UAV position [m/s^3]"""
        
        self.xd_4dot = np.zeros(3)
        """desired fourth derivative of the UAV position [m/s^4]"""

        
        self.b1d = np.zeros(3)
        """desired direction of the first body axis"""
        
        self.b1d[0] = 1.0
        
        self.b1d_dot = np.zeros(3)
        """first time derivative of the desired direction of the first body axis"""
        
        self.b1d_2dot = np.zeros(3)
        """second time desired direction of the first body axis"""

        
        self.Wd = np.zeros(3)
        """desired body angular velocity [rad/s]"""
        
        self.Wd_dot = np.zeros(3)
        """desired body angular acceleration [rad/s^2]"""

        
        self.Rd = np.identity(3)
        """desired attitude in SO(3)"""

        
        self.b3d = np.zeros(3)
        """desired direction of the third body axis"""
        
        self.b3d_dot = np.zeros(3)
        """first time derivative of the desired direction of the third body axis"""
        
        self.b3d_2dot = np.zeros(3)
        """second time desired direction of the third body axis"""

        
        self.b1c = np.zeros(3)
        """calculated direction of the first body axis"""
        
        self.wc3 = 0.0
        """angular velocity around third body axis [rad/s]"""
        
        self.wc3_dot = 0.0
        """angular acceleration around third body axis [rad/s^s]"""

      
        
        self.use_integral = False
        """flag to enable/disable integral control"""

      
        ## direction vectors
        
        self.e1 = np.zeros(3)
        """direction of the e1 axis"""
        
        self.e1[0] = 1.0
        
        self.e2 = np.zeros(3)
        """direction of the e2 axis"""
        
        self.e2[1] = 1.0
        
        self.e3 = np.zeros(3)
        """direction of the e3 axis"""
        
        self.e3[2] = 1.0

      
        ## physical parameters
        
        self.m = 1.95
        """mass of the rover [kg]"""
        
        self.g = 9.81
        """gravitational acceleration [m/s^2]"""
        
        self.c_tf = 0.0135
        """torsional moment generated by the propellers"""
        
        self.l = 0.23
        """length of the rover arm [m]"""
        
        self.J = np.diag([0.02, 0.02, 0.04])
        """inertia matrix of the rover"""

      
        ## attitude gains
        
        self.kR = np.diag([1.6, 1.6, 0.6])
        """attitude gains"""
        
        self.kW = np.diag([0.40, 0.40, 0.10])
        """angular rate gains"""

      
        ## decoupled-yaw controller gains
        
        self.ky = self.kR[2, 2]
        """yaw gain for decoupled-yaw controller"""
        
        self.kwy = self.kR[2, 2]
        """yaw angular rate gain for decoupled-yaw controller"""

      
        ## position gains
        
        self.kX = np.diag([16.0, 16.0, 20.0])
        """position gains"""
        
        self.kV = np.diag([12.0, 12.0, 12.0])
        """velocity gains"""

      
        ## integral gains
        
        self.kIR = 0.015
        """attitude integral gain"""
        
        self.ki = 0.01
        """position integral gain"""
        
        self.kI = 0.01
        """attitude integral gain for roll and pitch"""
        
        self.kyI = 0.02
        """attitude integral gain for yaw"""
        
        self.kIX = 4.0
        """position integral gains"""
        
        self.c1 = 1.0
        """parameters for decoupled-yaw integral control"""
        
        self.c2 = 1.0
        """parameters for decoupled-yaw integral control"""
        
        self.c3 = 1.0
        """parameters for decoupled-yaw integral control"""

      
        ## force and moment variables
        
        self.fM = np.zeros((4, 1))
        """force-moment vector"""
        
        self.f_total = 0.0
        """calculated forces required by each motor"""

      
        ## conversion matrices
        fM_to_forces = np.array([
            [1.0, 1.0, 1.0, 1.0],
            [0.0, -self.l, 0.0, self.l],
            [self.l, 0.0, -self.l, 0.0],
            [-self.c_tf, self.c_tf, -self.c_tf, self.c_tf]
        ])
        
        self.fM_to_forces_inv = np.linalg.inv(fM_to_forces)
        """force to force-moment conversion matrix"""

      
        ## integral errors
        
        self.eIX = IntegralErrorVec3()
        """position integral error"""
        
        self.eIR = IntegralErrorVec3()
        """attitude integral error"""
        
        self.eI1 = IntegralError()
        """attitude integral error for roll axis"""
        
        self.eI2 = IntegralError()
        """attitude integral error for pitch axis"""
        
        self.eIy = IntegralError()
        """attitude integral error for yaw axis"""
        
        self.eIX = IntegralError()
        """position integral error"""

      
        
        self.sat_sigma = 1.8
        """saturation limit"""


    def run(self, states, desired):
        """Run the controller to get the force-moments required to achieve the 
        the desired states from the current state.

        Args:
            state: (x, v, a, R, W) current states of the UAV
            desired: (xd, xd_dot, xd_2dot, xd_3dot, xd_4dot, b1d, b1d_dot,
                b1d_2dot, is_landed) desired states of the UAV

        Return:
            fM: (4x1 numpy array) force-moments vector
        """

        self.x, self.v, self.a, self.R, self.W = states
        self.xd, self.xd_dot, self.xd_2dot, self.xd_3dot, self.xd_4dot, \
            self.b1d, self.b1d_dot, self.b1d_2dot, is_landed = desired

        # If the UAV is landed, do not run the controller and produce zero
        # force-moments.
        if is_landed:
            return np.zeros(4)
            
        self.position_control()
        self.attitude_control()

        return self.fM


    def position_control(self):
        """Position controller to determine desired attitude and angular rates
        to achieve the deisred states.

        This uses the controller defined in "Control of Complex Maneuvers
        for a Quadrotor UAV using Geometric Methods on SE(3)"
        URL: https://arxiv.org/pdf/1003.2005.pdf
        """

        m = self.m
        g = self.g
        e3 = self.e3

        kX = self.kX
        kV = self.kV
        kIX = self.kIX

        R = self.R
        R_T = self.R.T

        x = self.x
        v = self.v
        W = self.W

        b1d = self.b1d
        b1d_dot = self.b1d_dot
        b1d_2dot = self.b1d_2dot
        
        xd = self.xd
        xd_dot = self.xd_dot
        xd_2dot = self.xd_2dot
        xd_3dot = self.xd_3dot
        xd_4dot = self.xd_4dot

        self.update_current_time()
        self.dt = self.t - self.t_pre

        # Translational error functions
        eX = x - xd  # position error - eq (11)
        eV = v - xd_dot  # velocity error - eq (12)

        # Position integral terms
        if self.use_integral:
            self.eIX.integrate(self.c1 * eX + eV, self.dt)  # eq (13)
            self.eIX.error = saturate(self.eIX.error, \
                -self.sat_sigma, self.sat_sigma)
        else:
            self.eIX.set_zero()

        # Force 'f' along negative b3-axis - eq (14)
        # This term equals to R.e3
        A = - kX @ eX \
            - kV @ eV \
            - kIX * self.eIX.error \
            - m * g * e3 \
            + m * xd_2dot

        hatW = hat(W)

        b3 = R @ e3
        b3_dot = R @ hatW @ e3  # eq (22)
        f_total = -A @ b3

        # Intermediate terms for rotational errors
        ea = g * e3 \
            - f_total / m * b3 \
            - xd_2dot
        A_dot = - kX @ eV \
            - kV @ ea \
            + m * xd_3dot

        fdot = - A_dot @ b3 \
            - A @ b3_dot
        eb = - fdot / m * b3 \
            - f_total / m * b3_dot \
            - xd_3dot
        A_2dot = - kX @ ea \
            - kV @ eb \
            + m * xd_4dot

        b3c, b3c_dot, b3c_2dot = deriv_unit_vector(-A, -A_dot, -A_2dot)

        hat_b1d = hat(b1d)
        hat_b1d_dot = hat(b1d_dot)

        A2 = -hat_b1d @ b3c
        A2_dot = - hat_b1d_dot @ b3c - hat_b1d @ b3c_dot
        A2_2dot = - hat(b1d_2dot) @ b3c \
            - 2.0 * hat_b1d_dot @ b3c_dot \
            - hat_b1d @ b3c_2dot

        b2c, b2c_dot, b2c_2dot = deriv_unit_vector(A2, A2_dot, A2_2dot)

        hat_b2c = hat(b2c)
        hat_b2c_dot = hat(b2c_dot)

        b1c = hat_b2c @ b3c
        b1c_dot = hat_b2c_dot @ b3c + hat_b2c @ b3c_dot
        b1c_2dot = hat(b2c_2dot) @ b3c \
            + 2.0 * hat_b2c_dot @ b3c_dot \
            + hat_b2c @ b3c_2dot

        Rd = np.vstack((b1c, b2c, b3c)).T
        Rd_dot = np.vstack((b1c_dot, b2c_dot, b3c_dot)).T
        Rd_2dot = np.vstack((b1c_2dot, b2c_2dot, b3c_2dot)).T

        Rd_T = Rd.T
        Wd = vee(Rd_T @ Rd_dot)

        hat_Wd = hat(Wd)
        Wd_dot = vee(Rd_T @ Rd_2dot - hat_Wd @ hat_Wd)

        self.f_total = f_total
        self.Rd = Rd
        self.Wd = Wd
        self.Wd_dot = Wd_dot

        # Roll / pitch
        self.b3d = b3c
        self.b3d_dot = b3c_dot
        self.b3d_2dot = b3c_2dot

        # Yaw
        self.b1c = b1c
        self.wc3 = e3 @ (R_T @ Rd @ Wd)
        self.wc3_dot = e3 @ (R_T @ Rd @ Wd_dot) \
            - e3 @ (hatW @ R_T @ Rd @ Wd)


    def attitude_control(self) -> None:
        """Deacouple-yaw attitude controller to achieve the desired attitude and
        angular rates.
        
        This uses the controller defined in "Geometric Controls of a Quadrotor
        with a Decoupled Yaw Control" 
        URL: https://doi.org/10.23919/ACC.2019.8815189
        """

        R = self.R
        R_T = self.R.T

        Rd = self.Rd
        Rd_T = self.Rd.T

        b3d = self.b3d
        b3d_dot = self.b3d_dot
        b3d_2dot = self.b3d_2dot

        W = self.W
        Wd = self.Wd

        J = self.J
        
        b1 = R @ self.e1
        b2 = R @ self.e2
        b3 = R @ self.e3

        hat_b3 = hat(b3)

        # Roll/pitch angular velocity vector
        W_12 = W[0] * b1 + W[1] * b2
        b3_dot = hat(W_12) @ b3  # eq (26)

        hat_b3d = hat(b3d)
        W_12d = hat_b3d @ b3d_dot
        W_12d_dot = hat_b3d @ b3d_2dot

        eb = hat_b3d @ b3  # eq (27)
        ew = W_12 + hat_b3 @ hat_b3 @ W_12d  # eq (28)

        # Yaw
        ey = -b2 @ self.b1c
        ewy = W[2] - self.wc3

        # Attitude integral terms
        eI = ew + self.c2 * eb

        self.eI1.integrate(eI @ b1, self.dt)  # b1 axis - eq (29)
        self.eI2.integrate(eI @ b2, self.dt)  # b2 axis - eq (30)
        self.eIy.integrate(ewy + self.c3 * ey, self.dt)

        # Control moment for the roll/pitch dynamics - eq (31)
        tau = -self.kR[0, 0] * eb \
            - self.kW[0, 0] * ew \
            - J[0, 0] * b3.T @ W_12d * b3_dot \
            - J[0, 0] * hat_b3 @ hat_b3 @ W_12d_dot
        if self.use_integral:
            tau += -self.kI * self.eI1.error * b1 \
                - self.kI * self.eI2.error * b2

        # Control moment around b1 axis - roll - eq (24)
        M1 = b1.T @ tau + J[2, 2] * W[2] * W[1]

        # Control moment around b2 axis - pitch - eq (24)
        M2 = b2.T @ tau - J[2, 2] * W[2] * W[0]

        # Control moment around b3 axis - yaw - eq (52)
        M3 = - self.ky * ey \
            - self.kwy * ewy \
            + self.J[2, 2] * self.wc3_dot
        
        if self.use_integral:
            M3 += - self.kyI * self.eIy.error

        # Gazebo uses ENU frame, but NED frame is used in FDCL.
        M = np.array([M1, -M2, -M3])

        self.fM[0] = self.f_total
        for i in range(3):
            self.fM[i + 1] = M[i]
            
        f_motor = self.fM_to_forces_inv @ self.fM

        # For saving:
        RdtR = Rd_T @ R
        eR = 0.5 * vee(RdtR - RdtR.T)
        self.eIR.error = np.array([self.eI1.error, self.eI2.error, \
            self.eIy.error])
        eW = W - R_T @ Rd @ Wd


    def set_integral_errors_to_zero(self) -> None:
        """Set all integrals to zero."""
        self.eIX.set_zero()
        self.eIR.set_zero()
        self.eI1.set_zero()
        self.eI2.set_zero()
        self.eIy.set_zero()
        self.eIX.set_zero()


    def update_current_time(self) -> None:
        """Update the current time since epoch."""
        self.t_pre = self.t

        t_now = datetime.datetime.now()
        self.t = (t_now - self.t0).total_seconds()


    def get_current_time(self) -> float:
        """Return the current time since epoch.
        
        Return:
            t: (float) time since epoch [s]
        """
        t_now = datetime.datetime.now()
        return (t_now - self.t0).total_seconds()
