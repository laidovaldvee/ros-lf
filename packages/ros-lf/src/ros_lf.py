#!/usr/bin/env python3

import cv2
import numpy as np
import os
import rospy
import yaml
import math

from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
#from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import WheelsCmdStamped
from smbus2 import SMBus

busno = 12 #/dev/i2c-12
addr = 62 #0x3e
reg = 17 #0x11

bus = SMBus(busno)


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
            max_value=1.0
        )

        # Wait for the automatic gain control
        # of the camera to settle, before we stop it
        rospy.sleep(2.0)
        #rospy.set_param('/%s/camera_node/exposure_mode'
        #                self.veh_name, 'off')
        
        self.pub = rospy.Publisher('/'+self.veh_name+'/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=0)

        self.log("Initialized")

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

    def to_binary(self,number):
        count = 0
        zero = "00000000"
        binary = bin(number).replace("0b", "")
        binary = zero[:(8-len(binary))] + binary
        for i in binary:
            if i == "1":
                count+=1
        return binary
        
    def binary_to_speed(self, binary):
        left_speed = 0.4
        right_speed = 0.4
        max_speed = 9
        speed_step = 2
        for i in range(4):
            left_speed = left_speed + int(binary[-1-i])*(max_speed-pow(speed_step,i))
            right_speed = right_speed + int(binary[i])*(max_speed-pow(speed_step,i))
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
        rate = rospy.Rate(10) # 10Hz
        while not rospy.is_shutdown():
            b = bus.read_byte_data(addr,reg)
            #bin_no = self.to_binary(b)
            #if density > 0 and int(bin_no,2) != 0:
            #	average = math.log2(int(bin_no,2))/density
            #else:
            #    average = -1
            #rospy.loginfo(average)
            rospy.loginfo(self.to_binary(b))
            #rospy.loginfo(density)
            
            #if average < 0:
            #    speed_l, speed_r = self.looking_line(last_average)
            #elif average >=4: 
            #    speed_l, speed_r = self.speedToCmd(0.5,3)
            #elif average <= 3:
            #    speed_l, speed_r = self.speedToCmd(3,0.5)
            #else:
            #    speed_l, speed_r = self.speedToCmd(3,3)
            #last_average = average
            
            # Put the wheel commands in a message and publish
            speed.header.stamp = speed.header.stamp
            l, r = self.binary_to_speed(self.to_binary(b))
            rospy.loginfo(l)
            rospy.loginfo(r)
            speed_l, speed_r = self.speedToCmd(l,r)
            speed.vel_left = speed_l
            speed.vel_right = speed_r
            self.pub.publish(speed)
            rate.sleep()

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
