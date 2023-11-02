import datetime
import numpy as np
import pdb


class Trajectory:
    def __init__(self):
        self.mode = 0
        self.is_mode_changed = False
        self.is_landed = False

        self.t0 = datetime.datetime.now()
        self.t = 0.0
        self.t_traj = 0.0

        self.xd = np.zeros(3)
        self.xd_dot = np.zeros(3)
        self.xd_2dot = np.zeros(3)
        self.xd_3dot = np.zeros(3)
        self.xd_4dot = np.zeros(3)

        self.b1d = np.zeros(3)
        self.b1d[0] = 1.0
        self.b1d_dot = np.zeros(3)
        self.b1d_2dot = np.zeros(3)

        self.x = np.zeros(3)
        self.v = np.zeros(3)
        self.a = np.zeros(3)
        self.R = np.identity(3)
        self.W = np.zeros(3)

        self.x_init = np.zeros(3)
        self.v_init = np.zeros(3)
        self.a_init = np.zeros(3)
        self.R_init = np.identity(3)
        self.W_init = np.zeros(3)
        self.b1_init = np.zeros(3)
        self.theta_init = 0.0

        self.trajectory_started = False
        self.trajectory_complete = False
        
        # Manual mode
        self.manual_mode = False
        self.manual_mode_init = False
        self.x_offset = np.zeros(3)
        self.yaw_offset = 0.0

        # Take-off
        self.takeoff_end_height = -1.0  # (m)
        self.takeoff_velocity = -1.0  # (m/s)

        # Landing
        self.landing_velocity = 1.0  # (m/s)
        self.landing_motor_cutoff_height = -0.25  # (m)

        # Circle
        self.circle_center = np.zeros(3)
        self.circle_linear_v = 1.0 
        self.circle_W = 1.2
        self.circle_radius = 1.2

        # Triangle
        self.vertex1 = np.array([0.0, 0.0, 0.0])
        self.vertex2 = np.array([0.0, 0.0, 0.0])
        self.vertex3 = np.array([0.0, 0.0, 0.0])
        self.side1_len = 1
        self.side2_len = 1
        self.side3_len = 1
        self.numTri = 3
        self.arrivedCheck = np.zeros(3)

        self.t1 = 0.0
        self.t2 = 0.0
        self.t3 = 0.0

        self.waypoint_speed = 1.0  # (m/s)

        self.e1 = np.array([1.0, 0.0, 0.0])


    def get_desired(self, mode, states, x_offset, yaw_offset):
        self.x, self.v, self.a, self.R, self.W = states
        self.x_offset = x_offset
        self.yaw_offset = yaw_offset

        if mode == self.mode:
            self.is_mode_changed = False
        else:
            self.is_mode_changed = True
            self.mode = mode
            self.mark_traj_start()

        self.calculate_desired()

        desired = (self.xd, self.xd_dot, self.xd_2dot, self.xd_3dot, \
            self.xd_4dot, self.b1d, self.b1d_dot, self.b1d_2dot, self.is_landed)
        return desired

    
    def calculate_desired(self):
        if self.manual_mode:
            self.manual()
            return
        
        if self.mode == 0 or self.mode == 1:  # idle and warm-up
            self.set_desired_states_to_zero()
            self.mark_traj_start()
        elif self.mode == 2:  # take-off
            self.takeoff()
        elif self.mode == 3:  # land
            self.land()
        elif self.mode == 4:  # stay
            self.stay()
        elif self.mode == 5:  # circle
            self.circle()
        elif self.mode == 6:
            self.triangle()


    def mark_traj_start(self):
        self.trajectory_started = False
        self.trajectory_complete = False

        self.manual_mode = False
        self.manual_mode_init = False
        self.is_landed = False

        self.t = 0.0
        self.t_traj = 0.0
        self.t0 = datetime.datetime.now()

        self.x_offset = np.zeros(3)
        self.yaw_offset = 0.0

        self.update_initial_state()


    def mark_traj_end(self, switch_to_manual=False):
        self.trajectory_complete = True

        if switch_to_manual:
            self.manual_mode = True


    def set_desired_states_to_zero(self):
        self.xd = np.zeros(3)
        self.xd_dot = np.zeros(3)
        self.xd_2dot = np.zeros(3)
        self.xd_3dot = np.zeros(3)
        self.xd_4dot = np.zeros(3)

        self.b1d = np.array([1.0, 0.0, 0.0])
        self.b1d_dot = np.zeros(3)
        self.b1d_2dot = np.zeros(3)

    
    def set_desired_states_to_current(self):
        self.xd = np.copy(self.x)
        self.xd_dot = np.copy(self.v)
        self.xd_2dot = np.zeros(3)
        self.xd_3dot = np.zeros(3)
        self.xd_4dot = np.zeros(3)

        self.b1d = self.get_current_b1()
        self.b1d_dot = np.zeros(3)
        self.b1d_2dot = np.zeros(3)


    def update_initial_state(self):
        self.x_init = np.copy(self.x)
        self.v_init = np.copy(self.v)
        self.a_init = np.copy(self.a)
        self.R_init = np.copy(self.R)
        self.W_init = np.copy(self.W)

        self.b1_init = self.get_current_b1()
        self.theta_init = np.arctan2(self.b1_init[1], self.b1_init[0])

    
    #Explanation:
    # Takes first col from self.R, which is a 3x1 matrix corresponding to b1
    # see Figure 2.2: or see 2.3 Reference Frames (in Geometric Control paper on drive/Skywalker/Reading_Materials)

    # "In other words, the position of the UAV, x ∈ R3, is defined
    # as the origin of the b-frame in f -frame, and the attitude of the UAV, R ≜ R_(fb) ∈ SO(3),
    # is defined as the rotation of the b-frame with respect to the f-frame"

    def get_current_b1(self):
        b1 = self.R.dot(self.e1)
        print(b1)
        theta = np.arctan2(b1[1], b1[0])
        return np.array([np.cos(theta), np.sin(theta), 0.0])


    def waypoint_reached(self, waypoint, current, radius):
        delta = waypoint - current
        
        if abs(np.linalg.norm(delta) < radius):
            return True
        else:
            return False


    def update_current_time(self):
        t_now = datetime.datetime.now()
        self.t = (t_now - self.t0).total_seconds()


    def manual(self):
        if not self.manual_mode_init:
            self.set_desired_states_to_current()
            self.update_initial_state()

            self.manual_mode_init = True
            self.x_offset = np.zeros(3)
            self.yaw_offset = 0.0

            print('Switched to manual mode')
        
        self.xd = self.x_init + self.x_offset
        self.xd_dot = (self.xd - self.x) / 1.0

        theta = self.theta_init + self.yaw_offset
        self.b1d = np.array([np.cos(theta), np.sin(theta), 0.0])


    #Explanation: see comments, and for variable meanings, see control.py
    def takeoff(self):
        if not self.trajectory_started:
            self.set_desired_states_to_zero()

            # Take-off starts from the current horizontal position.
            self.xd[0] = self.x[0]
            self.xd[1] = self.x[1]
            self.x_init = self.x

            # t=v/d, where t=t_traj, d=takeoff_end_height, and v=takeoff_velocity
            self.t_traj = (self.takeoff_end_height - self.x[2]) / \
                self.takeoff_velocity

            # Set the takeoff attitude to the current attitude.
            self.b1d = self.get_current_b1()

            self.trajectory_started = True

        self.update_current_time()

        if self.t < self.t_traj:
            self.xd[2] = self.x_init[2] + self.takeoff_velocity * self.t
            self.xd_2dot[2] = self.takeoff_velocity
        else:
            if self.waypoint_reached(self.xd, self.x, 0.04):
                self.xd[2] = self.takeoff_end_height
                self.xd_dot[2] = 0.0

                if not self.trajectory_complete:
                    print('Takeoff complete\nSwitching to manual mode')
                
                self.mark_traj_end(True)


    def land(self):
        if not self.trajectory_started:
            self.set_desired_states_to_current()
            self.t_traj = (self.landing_motor_cutoff_height - self.x[2]) / \
                self.landing_velocity

            # Set the landing attitude to the current attitude.
            self.b1d = self.get_current_b1()

            self.trajectory_started = True

        self.update_current_time()

        if self.t < self.t_traj:
            self.xd[2] = self.x_init[2] + self.landing_velocity * self.t
            self.xd_2dot[2] = self.landing_velocity
        else:
            if self.x[2] > self.landing_motor_cutoff_height:
                self.xd[2] = self.landing_motor_cutoff_height
                self.xd_dot[2] = 0.0

                if not self.trajectory_complete:
                    print('Landing complete')

                self.mark_traj_end(False)
                self.is_landed = True
            else:
                self.xd[2] = self.landing_motor_cutoff_height
                self.xd_dot[2] = self.landing_velocity

            
    def stay(self):
        print("STAYYY")
        if not self.trajectory_started:
            self.set_desired_states_to_current()
            self.trajectory_started = True
        
        self.mark_traj_end(True)


    def circle(self):
        if not self.trajectory_started:
            self.set_desired_states_to_current()
            self.trajectory_started = True

            self.circle_center = np.copy(self.x)
            self.circle_W = 2 * np.pi / 8

            num_circles = 2
            self.t_traj = self.circle_radius / self.circle_linear_v \
                + num_circles * 2 * np.pi / self.circle_W

        self.update_current_time()
        if self.t < (self.circle_radius / self.circle_linear_v):
            self.xd[0] = self.circle_center[0] + self.circle_linear_v * self.t
            self.xd_dot[0] = self.circle_linear_v

        elif self.t < self.t_traj:
            circle_W = self.circle_W
            circle_radius = self.circle_radius

            t = self.t - circle_radius / self.circle_linear_v
            th = circle_W * t

            circle_W2 = circle_W * circle_W
            circle_W3 = circle_W2 * circle_W
            circle_W4 = circle_W3 * circle_W

            # axis 1
            self.xd[0] = circle_radius * np.cos(th) + self.circle_center[0]
            self.xd_dot[0] = - circle_radius * circle_W * np.sin(th)
            self.xd_2dot[0] = - circle_radius * circle_W2 * np.cos(th)
            self.xd_3dot[0] = circle_radius * circle_W3 * np.sin(th)
            self.xd_4dot[0] = circle_radius * circle_W4 * np.cos(th)

            # axis 2
            self.xd[1] = circle_radius * np.sin(th) + self.circle_center[1]
            self.xd_dot[1] = circle_radius * circle_W * np.cos(th)
            self.xd_2dot[1] = - circle_radius * circle_W2 * np.sin(th)
            self.xd_3dot[1] = - circle_radius * circle_W3 * np.cos(th)
            self.xd_4dot[1] = circle_radius * circle_W4 * np.sin(th)

            w_b1d = 2.0 * np.pi / 10.0
            th_b1d = w_b1d * t

            self.b1d = np.array([np.cos(th_b1d), np.sin(th_b1d), 0])
            self.b1d_dot = np.array([- w_b1d * np.sin(th_b1d), \
                w_b1d * np.cos(th_b1d), 0.0])
            self.b1d_2dot = np.array([- w_b1d * w_b1d * np.cos(th_b1d),
                w_b1d * w_b1d * np.sin(th_b1d), 0.0])
        else:
            self.mark_traj_end(True)

#    def triangle(self):
#        if not self.trajectory_started:
#            self.set_desired_states_to_zero()

            # Take-off starts from the current horizontal position.
            #self.xd[0] = self.x[0]
            #self.xd[1] = self.x[1]
            #self.x_init = self.x

            # t=v/d, where t=t_traj, d=takeoff_end_height, and v=takeoff_velocity
            #self.t_traj = (self.takeoff_end_height - self.x[2]) / \
                #self.takeoff_velocity

            # Set the takeoff attitude to the current attitude.
#            self.b1d = self.get_current_b1()
#
#            self.trajectory_started = True

#        self.update_current_time()
#
 #       if self.t < self.t_traj:
  #          self.xd[2] = self.x_init[2] + self.takeoff_velocity * self.t
   #         self.xd_2dot[2] = self.takeoff_velocity
    #    else:
     #       if self.waypoint_reached(self.xd, self.x, 0.04):
      #         self.xd_dot[2] = 0.0

       #         if not self.trajectory_complete:
        #            print('Takeoff complete\nSwitching to manual mode')
         #       
          #      self.mark_traj_end(True)
    def triangle(self):
        print("TRIANGLEE")
        if not self.trajectory_started:
            self.set_desired_states_to_current()
            self.trajectory_started = True

            #self.vertex1 = np.array([0.0, 4.0, 0.0])
            #self.vertex2 = np.array([0.0, 2.0, 0.0])
            #self.vertex3 = np.array([0.0, 0.0, 0.0])

            self.vertex3 = np.copy(self.x)
            self.vertex1 = np.copy(self.x)
            self.vertex1[2] -= 4
            
            self.vertex2 = np.copy(self.x)
            self.vertex2[1] -= 3

            self.side1_len = np.linalg.norm(self.vertex2 - self.vertex1)
            self.side2_len = np.linalg.norm(self.vertex3 - self.vertex2)
            self.side3_len = np.linalg.norm(self.vertex1 - self.vertex3)

            total_len = self.side1_len + self.side2_len + self.side3_len

            self.t1 = self.side1_len / self.waypoint_speed
            self.t2 = self.side2_len / self.waypoint_speed
            self.t3 = self.side3_len / self.waypoint_speed

            self.t_traj = (self.t1 + self.t2 + self.t3) * 3
            print("FIRSTTTT", self.t, self.t1, self.t2, self.t3)
        self.update_current_time()

        if(self.numTri != 0):
        #print(self.t, self.t_traj)
            #if self.t < self.t_traj:
            if self.arrivedCheck[0] == 0:
                print("------1111111111111------------------------------------------------------------------------------------")
                self.xd = self.vertex1 
                self.xd_dot = self.waypoint_speed * (self.vertex2 - self.vertex1) / self.side1_len
                if self.x[2]-1 < self.xd[2]:
                    self.arrivedCheck[0] = 1
                    
            elif self.arrivedCheck[1] == 0:
                print("------22222222------------------------------------------------------------------------------------")
                self.xd = self.vertex2 
                self.xd_dot = self.waypoint_speed * (self.vertex3 - self.vertex2) / self.side2_len
                if self.x[1]+0.1 > self.xd[1]:
                    self.arrivedCheck[1] = 1
            elif self.arrivedCheck[2] == 0:
                print("--33333333333333333333----------------------------------------------------------------------------------------")
                self.xd = self.vertex3 
                self.xd_dot = self.waypoint_speed * (self.vertex1 - self.vertex3) / self.side3_len
                if self.x[0]+0.1 > self.xd[0]:
                    self.arrivedCheck[2] = 1
                    self.numTri += 1
                    self.arrivedCheck = np.zeros(3)
                    
            print(self.xd, self.x, self.arrivedCheck)
        else:
            self.mark_traj_end(True)

            #amplitude = 1.0
            #period = 4.0
            #t_phase = (self.t - self.t_traj) % period

            #if t_phase < period /2:
            #    self.xd[0] = amplitude * t_phase /(period /2)
            #else:
            #    self.xd[0] = amplitude - amplitude * (t_phase - period / 2) / (period / 2)

        #self.update_current_time()
        #if self.t > self.t_traj + period:
            #self.mark_traj_end(True)
'''
self.t_traj = 10.0

            self.b1d = self.get_current_b1()
            self.trajectory_started = True

        self.update_current_time()

        if self.t < self.t_traj:
            self.xd[2] = self.x_init[2] + (self.t / self.t_traj) * 5.0
            self.xd_2dot[2] = 5.0 / self.t_traj
        else:
            if self.x[2] > 0.05: 
                self.xd[2] = self.x_init[2] + (self.t / self.t_traj) * 5.0
                self.xd_2dot[2] = 5.0/self.t_traj
            else:
                if self.x[2] > 0.05:
                    self.xd[2] = 0.05
                    self.xd_dot[2] = 0.0
                else:
                    self.mark_traj_end(True)
        ''' 