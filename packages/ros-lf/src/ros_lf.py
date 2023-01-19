#!/usr/bin/env python3

import cv2
import numpy as np
import os
import rospy
import yaml
import math

from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from sensor_msgs.msg import Range
from std_msgs.msg import Float32
from duckietown_msgs.msg import WheelsCmdStamped



Kp = 0.4
Speed = 4



class LineFollow(DTROS):
    """LineFollow Behaviour

    This node implements linefollow vehicle behavior on a Duckiebot.

    Args:
        node_name (:obj:`str`): a unique, descriptive name for the node
            that ROS will use

    Configuration:
        ~gain (:obj:`float`): scaling factor applied to the desired
            velocity, taken from the robot-specific kinematics
            calibration
        ~trim (:obj:`float`): trimming factor that is typically used
            to offset differences in the behaviour of the left and
            right motors, it is recommended to use a value that results
            in the robot moving in a straight line when forward command
            is given, taken from the robot-specific kinematics calibration
        ~baseline (:obj:`float`): the distance between the two wheels
            of the robot, taken from the robot-specific kinematics
            calibration
        ~radius (:obj:`float`): radius of the wheel, taken from the
            robot-specific kinematics calibration
        ~k (:obj:`float`): motor constant, assumed equal for both
            motors, taken from the robot-specific kinematics calibration
        ~limit (:obj:`float`): limits the final commands sent to the
            motors, taken from the robot-specific kinematics calibration

    Publisher:
        ~wheels_cmd (:obj:`duckietown_msgs.msg.WheelsCmdStamped`): The
            wheel commands that the motors will execute

    """

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(LineFollow, self).__init__(node_name=node_name,
                                              node_type=NodeType.BEHAVIOR)
        #self.veh_name = rospy.get_namespace().strip("/")
        self.veh_name = os.environ['VEHICLE_NAME']

        # Set parameters using a robot-specific yaml file if such exists
        self.readParamFromFile()

        # Get static parameters
        self._baseline = rospy.get_param('~baseline')
        self._radius = rospy.get_param('~radius')
        self._k = rospy.get_param('~k')
        self.distance = 0.09

        self.error = 0.0
        self.range = 0.0

        # Get editable parameters
        self._gain = DTParam(
            '~gain',
            param_type=ParamType.FLOAT,
            min_value=0.0,
            max_value=3.0
        )
        self._trim = DTParam(
            '~trim',
            param_type=ParamType.FLOAT,
            min_value=0.0,
            max_value=3.0
        )
        self._limit = DTParam(
            '~limit',
            param_type=ParamType.FLOAT,
            min_value=0.0,
            max_value=9.0
        )

        # Wait for the automatic gain control
        # of the camera to settle, before we stop it
        rospy.sleep(2.0)
        #rospy.set_param('/%s/camera_node/exposure_mode'
        #                self.veh_name, 'off')
        
        self.sub = rospy.Subscriber('line_array', Float32, self.lf_callback)

        self.tof_sub = rospy.Subscriber('/'+self.veh_name+'/front_center_tof_driver_node/range', Range, self.range_callback)
        
        self.pub = rospy.Publisher('/'+self.veh_name+'/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=0)

        self.log("Initialized")
        
    def lf_callback(self, data):
        self.error=data.data

    def range_callback(self, data):
        self.range = data.range

    def speedToCmd(self, speed_l, speed_r):
        """Applies the robot-specific gain and trim to the
        output velocities

        Applies the motor constant k to convert the deisred wheel speeds
        to wheel commands. Additionally, applies the gain and trim from
        the robot-specific kinematics configuration.

        Args:
            speed_l (:obj:`float`): Desired speed for the left
                wheel (e.g between 0 and 1)
            speed_r (:obj:`float`): Desired speed for the right
                wheel (e.g between 0 and 1)

        Returns:
            The respective left and right wheel commands that need to be
                packed in a `WheelsCmdStamped` message

        """

        # assuming same motor constants k for both motors
        k_r = self._k
        k_l = self._k

        # adjusting k by gain and trim
        k_r_inv = (self._gain.value + self._trim.value) / k_r
        k_l_inv = (self._gain.value - self._trim.value) / k_l

        # conversion from motor rotation rate to duty cycle
        u_r = speed_r * k_r_inv
        u_l = speed_l * k_l_inv

        # limiting output to limit, which is 1.0 for the duckiebot
        u_r_limited = self.trim(u_r,
                                -self._limit.value,
                                self._limit.value)
        u_l_limited = self.trim(u_l,
                                -self._limit.value,
                                self._limit.value)

        return u_l_limited, u_r_limited

    def readParamFromFile(self):
        """
        Reads the saved parameters from
        `/data/config/calibrations/kinematics/DUCKIEBOTNAME.yaml` or
        uses the default values if the file doesn't exist. Adjsuts
        the ROS paramaters for the node with the new values.

        """
        # Check file existence
        fname = self.getFilePath(self.veh_name)
        # Use the default values from the config folder if a
        # robot-specific file does not exist.
        if not os.path.isfile(fname):
            self.log("Kinematics calibration file %s does not "
                     "exist! Using the default file." % fname, type='warn')
            fname = self.getFilePath('default')

        with open(fname, 'r') as in_file:
            try:
                yaml_dict = yaml.safe_load(in_file)
            except yaml.YAMLError as exc:
                self.log("YAML syntax error. File: %s fname. Exc: %s"
                         %(fname, exc), type='fatal')
                rospy.signal_shutdown()
                return

        # Set parameters using value in yaml file
        if yaml_dict is None:
            # Empty yaml file
            return
        for param_name in ["gain", "trim", "baseline", "k", "radius", "limit"]:
            param_value = yaml_dict.get(param_name)
            if param_name is not None:
                rospy.set_param("~"+param_name, param_value)
            else:
                # Skip if not defined, use default value instead.
                pass

    def getFilePath(self, name):
        """
        Returns the path to the robot-specific configuration file,
        i.e. `/data/config/calibrations/kinematics/DUCKIEBOTNAME.yaml`.

        Args:
            name (:obj:`str`): the Duckiebot name

        Returns:
            :obj:`str`: the full path to the robot-specific
                calibration file

        """
        cali_file_folder = '/data/config/calibrations/kinematics/'
        cali_file = cali_file_folder + name + ".yaml"
        return cali_file
    
    def error_to_speed(self, error):
        left_speed = Speed
        right_speed = Speed
        max_speed = 4
        left_speed = left_speed - Kp*error
        right_speed = right_speed + Kp*error
        return left_speed, right_speed
        
        
    def trim(self, value, low, high):
        """
        Trims a value to be between some bounds.

        Args:
            value: the value to be trimmed
            low: the minimum bound
            high: the maximum bound

        Returns:
            the trimmed value
        """

        return max(min(value, high), low)

    def on_shutdown(self):
        """Shutdown procedure.

        Publishes a zero velocity command at shutdown."""

        # MAKE SURE THAT THE LAST WHEEL COMMAND YOU PUBLISH IS ZERO,
        # OTHERWISE YOUR DUCKIEBOT WILL CONTINUE MOVING AFTER
        # THE NODE IS STOPPED

        # PUT YOUR CODE HERE
        speed = WheelsCmdStamped()
        super(LineFollow, self).on_shutdown()
        speed.header.stamp = speed.header.stamp
        speed.vel_left = 0
        speed.vel_right = 0
        self.pub.publish(speed)
        
    def run(self):
        # publish message every 1 second
        speed = WheelsCmdStamped()
        last_average = 0
        rate = rospy.Rate(20) # 10Hz
        while not rospy.is_shutdown():
            if self.range > self.distance or self.range<0.05:           
                # Put the wheel commands in a message and publish
                speed.header.stamp = speed.header.stamp
                l, r = self.error_to_speed(self.error)
                rospy.loginfo(l)
                rospy.loginfo(r)
                speed_l, speed_r = self.speedToCmd(l,r)
                speed.vel_left = speed_l
                speed.vel_right = speed_r
                self.pub.publish(speed)
                rate.sleep()
            else:
                speed.header.stamp = speed.header.stamp
                speed.vel_left = 0
                speed.vel_right = 0
                self.pub.publish(speed)
                
    def looking_line(self,avg):
        if avg > 4:
            return self.speedToCmd(0.5,2)
        else:
            return self.speedToCmd(2,0.5)

if __name__ == '__main__':
    # Initialize the node
    LF_node = LineFollow(node_name='linefollow')
    rospy.on_shutdown(LF_node.on_shutdown)
    # Keep it spinning to keep the node alive
    LF_node.run()
    rospy.spin()
