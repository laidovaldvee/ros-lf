#!/usr/bin/env python3

import os
import rospy
import numpy as np
import math

from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import Float32
from smbus2 import SMBus

busno = 12 #/dev/i2c-12
addr = 62 #0x3e
reg = 17 #0x11

error_list = [32 , 16 , 8 , 2 , -2 , -8 , -16 , -32]
bus = SMBus(busno)

class MyPublisherNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        self.veh_name = os.environ['VEHICLE_NAME']
        self.pub = rospy.Publisher('/'+self.veh_name+'/line_array', Float32, queue_size=10)

    def run(self):
        # publish message every 1 second
        rate = rospy.Rate(20) # 10Hz
        while not rospy.is_shutdown():
            b = bus.read_byte_data(addr,reg)
            message = self.binary_to_error(self.to_binary(b))
            #rospy.loginfo("Publishing message: '%s'" % message)
            self.pub.publish(message)
            rate.sleep()
            
    def to_binary(self,number):
        zero_list = [0,0,0,0,0,0,0,0]
        binary = bin(int(number)).replace("0b", "")
        for i in binary:
            zero_list.append(int(i))
        binary_list = zero_list[-8:]
        return binary_list
        
    def binary_to_error(self, binary):
        esum_list = np.multiply(error_list,binary)
        if (np.count_nonzero(esum_list) != 0):
            error = sum(esum_list)/np.count_nonzero(esum_list)
        else:
            error = 0
        return error

if __name__ == '__main__':
    # create the node
    node = MyPublisherNode(node_name='Line_array_publisher_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()
