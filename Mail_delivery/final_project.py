#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import String, UInt32MultiArray
import numpy as np
import colorsys


class BayesLoc:
    def __init__(self, p0, colour_codes, colour_map):
        self.colour_sub = rospy.Subscriber(
            "mean_img_rgb", UInt32MultiArray, self.colour_callback
        )
        self.line_sub = rospy.Subscriber("line_idx", String, self.line_callback)
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        self.num_states = len(p0)
        self.colour_codes = colour_codes
        self.colour_map = colour_map
        self.probability = p0
        self.state_prediction = np.zeros(self.num_states)

        self.cur_colour = None  # most recent measured colour
        self.line = 0 # location of lightest pixel

    def colour_callback(self, msg):
        """
        callback function that receives the most recent colour measurement from the camera.
        """
        self.cur_colour = np.array(msg.data)  # [r, g, b]
        print(self.cur_colour)

    def line_callback(self, msg):
        """
        TODO: Complete this with your line callback function from lab 3.
        """
        self.line = msg.data
        return

    def wait_for_colour(self):
        """Loop until a colour is received."""
        rate = rospy.Rate(100)
        while not rospy.is_shutdown() and self.cur_colour is None:
            rate.sleep()

    def state_model(self, u):
        """
        State model: p(x_{k+1} | x_k, u)

        TODO: complete this function
        """

    def measurement_model(self, x):
        """
        Measurement model p(z_k | x_k = colour) - given the pixel intensity,
        what's the probability that of each possible colour z_k being observed?
        """

        """
        TODO: You need to compute the probability of states. You should return a 1x5 np.array
        Hint: find the euclidean distance between the measured RGB values (self.cur_colour)
            and the reference RGB values of each colour (self.ColourCodes).
        """
        if self.cur_colour is None:
            self.wait_for_colour()

        prob = np.zeros(len(colourCodes))

        for i in range(len(colourCodes)):
            colourCodes[i][0] = r
            colourCodes[i][1] = g
            colourCodes[i][2] = b
            prob[i] = 1/(np.sqrt((r - self.cur_colour[0])**2 + (g - self.cur_colour[1])**2 + (b - self.cur_colour[2])**2)

        prob = prob/sum(prob)

        return prob

    def state_predict(self):
        rospy.loginfo("predicting state")
        """
        TODO: Complete the state prediction function: update
        self.state_prediction with the predicted probability of being at each
        state (office)
        """

    def state_update(self):
        rospy.loginfo("updating state")
        """
        TODO: Complete the state update function: update self.probabilities
        with the probability of being at each state
        """

    def p_i_d(self):
        twist=Twist()
        rate = rospy.Rate(10)
        current = rospy.get_time()  # Start time for velocity integration
        k_p = 0.001
        k_i = 0
        k_d = 0
        integral = 0
        lasterror = 0
        mailed = False
        x = 0.6

        while not rospy.is_shutdown():

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
            #print('d', desired, 'a', actual, 'e', error)
            integral = integral + error
            deriv = error - lasterror
            #print(actual, error*k_p, integral*k_i, deriv*k_d)
            twist.linear.x = 0.09
            twist.angular.z = error * k_p + integral * k_i + deriv * k_d
            lasterror = error
            self.cmd_pub.publish(twist)

            colour_prob = measurement_model()
            if colour_prob[5] < x and mailed = False:
                twist.linear.x = 0
                twist.angular.z = np.pi/4
                self.cmd_pub.publish(twist)
                rospy.sleep(2)
                twist.angular.z = -np.pi/4
                self.cmd_pub.publish(twist)
                rospy.sleep(2)
                mailed = True

            if colour_prob[5] > x:
                mailed = False

            rate.sleep()

        # Stop at end
        twist.linear.x = 0
        twist.angular.z = 0
        self.cmd_pub.publish(twist)

if __name__ == "__main__":

    # This is the known map of offices by colour
    # 0: red, 1: green, 2: blue, 3: yellow, 4: line
    # current map starting at cell #2 and ending at cell #12
    colour_map = [3, 0, 1, 2, 2, 0, 1, 2, 3, 0, 1]

    # TODO calibrate these RGB values to recognize when you see a colour
    # NOTE: you may find it easier to compare colour readings using a different
    # colour system, such as HSV (hue, saturation, value). To convert RGB to
    # HSV, use:
    # h, s, v = colorsys.rgb_to_hsv(r / 255.0, g / 255.0, b / 255.0)
    colour_codes = [
        [167, 146, 158],  # red
        [163, 184, 100],  # green
        [173, 166, 171],  # blue
        [167, 170, 117],  # yellow
        [150, 150, 150],  # line
    ]

    # initial probability of being at a given office is uniform
    p0 = np.ones_like(colour_map) / len(colour_map)

    localizer = BayesLoc(p0, colour_codes, colour_map)

    rospy.init_node("final_project")
    rospy.sleep(0.5)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        """
        TODO: complete this main loop by calling functions from BayesLoc, and
        adding your own high level and low level planning + control logic
        """
        localizer.p_i_d()
        rate.sleep()

    rospy.loginfo("finished!")
    rospy.loginfo(localizer.probability)
