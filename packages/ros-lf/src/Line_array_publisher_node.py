#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import int8
from smbus2 import SMBus

busno = 12 #/dev/i2c-12
addr = 62 #0x3e
reg = 17 #0x11

bus = SMBus(busno)

class MyPublisherNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        self.pub = rospy.Publisher('line_array', int8, queue_size=10)

    def run(self):
        # publish message every 1 second
        rate = rospy.Rate(10) # 1Hz
        while not rospy.is_shutdown():
            b = bus.read_byte_data(addr,reg)
            message = b
            rospy.loginfo("Publishing message: '%s'" % message)
            self.pub.publish(message)
            rate.sleep()

if __name__ == '__main__':
    # create the node
    node = MyPublisherNode(node_name='Line_array_publisher_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()
