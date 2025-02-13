#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

import matplotlib.pyplot as plt
import math
import numpy as np


class KalmanFilter(object):
    def __init__(self, h, d, x_0, Q, R, P_0):
        self.h = h
        self.d = d

        self.Q = Q
        self.R = R
        self.P = P_0
        self.x = x_0

        self.u = 0  # initialize the cmd_vel input
        self.phi = 0  # initialize the measurement input

        self.time = rospy.get_time() # get initial time
        self.dt = 0 # initialize time step

        # Lists to store estimated position, covariance, and time
        self.est_positions = []
        self.covariances = []
        self.times = []

        self.state_pub = rospy.Publisher("state", Float64, queue_size=1)
        self.scan_sub = rospy.Subscriber(
            "scan_angle", Float64, self.scan_callback, queue_size=1
        )
        self.cmd_sub = rospy.Subscriber("cmd_vel_noisy", Twist, self.cmd_callback)

    def cmd_callback(self, cmd_msg):
        self.u = cmd_msg.linear.x

    ## updates self.phi with the most recent measurement of the tower.
    def scan_callback(self, msg):
        self.phi = msg.data

    ## call within run_kf to update the state with the measurement
    def predict(self, u=0):
        """
        TODO: update state via the motion model, and update the covariance with the process noise (completed)
        """

        # State prediction (xk+1 = Akxk + Bkuk)
        # Ak = 1, Bk = dt
        current = rospy.get_time()
        self.dt = current - self.time
        self.x = self.x + self.dt * self.u
        self.time = current

        # State covariance prediction
        self.P = self.P + self.Q

        # Measurement covariance prediction, set D matrix (1D)
        self.D = h/((d - self.x) ** 2 + h ** 2)
        self.S = self.D * self.P * self.D + self.R

        # Updated Kalman Gain
        self.W = self.P * self.D * (1/self.S)

        # Updated State covariance
        self.P = self.P - self.W * self.S * self.W
        self.covariances.append(self.P)
        self.times.append(self.time)

        return

    ## call within run_kf to update the state with the measurement
    def measurement_update(self):
        """
        TODO: update state when a new measurement has arrived using this function (completed)
        """
        self.phi_k = np.arctan(h/(d - self.x)) # measurement model, use for measurement prediction
        return

    def run_kf(self):
        current_input = self.u
        current_measurement = self.phi

        """
        TODO: complete this function to update the state with current_input and current_measurement (completed)
        """

        # Check valid measurement
        if not math.isnan(current_measurement):
            # Measurement prediction
            self.measurement_update()

            # State prediction and updated covariances
            self.predict()

            # Measurement residual
            self.s = current_measurement - self.phi_k

            # Updated state estimate
            self.x = self.x + self.W * self.s
            self.est_positions.append(self.x)
        
        else:
            # Keep current position
            self.predict()
            self.est_positions.append(self.x)

        # Plot estimate position and covariances over time, past last coordinate
        if self.x > 3.1:
            plt.plot(self.times, self.covariances, label =  "Covariance")
            plt.plot(self.times, self.est_positions, label =  "Estimated Position")
            plt.legend()
            plt.show()

        self.state_pub.publish(self.x)

class Controller(object):
    def __init__(self, kalman_filter):

        # Access Kalman filter in controller to get position
        self.kalman_filter = kalman_filter

        # publish motor commands
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        # subscribe to detected line index
        self.color_sub = rospy.Subscriber(
            "line_idx", UInt32, self.camera_callback, queue_size=1
        )
        self.line = 0

    def camera_callback(self, msg):
        """Callback for line index."""
        # access the value using msg.data
        self.line = msg.data

    def follow_the_line(self):
        """
        TODO: complete the function to follow the line (completed)
        """
        # PART 1: Test line-following control (uncomment below line)
        # self.p_i_d()

        # PART 2: Unfiltered package delivery
        self.p_i_d_unfiltered()
  
    def p_i_d(self):
        twist=Twist()
        rate = rospy.Rate(10)
        start = rospy.get_time()
        k_p = 0.005
        k_i = 0.0002
        k_d = 0.006
        integral = 0
        lasterror = 0
        while not rospy.is_shutdown():
            actual = self.line
            desired = 320
            error = desired - actual
            integral = integral + error
            deriv = error - lasterror
            print(actual, error*k_p, integral*k_i, deriv*k_d)
            twist.linear.x = 0.11
            twist.angular.z = error * k_p + integral * k_i + deriv * k_d
            lasterror = error
            self.cmd_pub.publish(twist)
            rate.sleep()
    
    def p_i_d_unfiltered(self):
        twist=Twist()
        rate = rospy.Rate(10)
        current = rospy.get_time()  # Start time for velocity integration
        k_p = 0.005
        k_i = 0.0002
        k_d = 0.006
        integral = 0
        lasterror = 0

        # Initialize position to 0
        position = 0

        # coordinates in m: 0.61, 1.22, 2.44, 3.05
        coordinates = [0.61, 1.22, 2.44, 3.05]
        i = 0

        while i < 4 and position < 3.1:
            
            # Update position by integrating velocity
            velocity = self.kalman_filter.u
            new = rospy.get_time()
            dt = new - current
            position += velocity * dt
            current = new # reset for new time step

            # PID
            actual = self.line
            desired = 320
            error = desired - actual
            integral = integral + error
            deriv = error - lasterror
            print(actual, error*k_p, integral*k_i, deriv*k_d)
            twist.linear.x = 0.1
            twist.angular.z = error * k_p + integral * k_i + deriv * k_d
            lasterror = error
            self.cmd_pub.publish(twist)

            # Stop for 2 seconds, already reached desired coordinate
            if position > coordinates[i]:
                twist.linear.x = 0
                twist.angular.z = 0
                self.cmd_pub.publish(twist)
                i += 1
                rospy.sleep(2)

            rate.sleep()
        
        # Stop at end
        twist.linear.x = 0
        twist.angular.z = 0
        self.cmd_pub.publish(twist)

if __name__ == "__main__":
    rospy.init_node("lab4")

    h = 0.60  # y distance to tower
    d = 0.60 * 3  # x distance to tower (from origin)

    x_0 = 0  # initial state position

    Q = 1  # TODO: Set process noise covariance
    R = 1  # TODO: measurement noise covariance
    P_0 = 1  # TODO: Set initial state covariance

    # PART 4: EKF Implementation
    kf = KalmanFilter(h, d, x_0, Q, R, P_0)
    controller = Controller(kf)
    controller.follow_the_line()
    rospy.sleep(1)

    # For running kalman filter
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        kf.run_kf()
        rate.sleep()