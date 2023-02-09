#!/usr/bin/env python3

import time
import smbus
import struct
import rospy
import os
import numpy as np
from sensor_msgs.msg import Temperature, Imu
from tf.transformations import quaternion_about_axis


DEFAULT_ADDRESS = 0x68  # MPU6050 default i2c address w/ AD0 low
DEVICE_ID = 0x68  # The correct MPU6050_WHO_AM_I value

SELF_TEST_X = 0x0D  # Self test factory calibrated values register
SELF_TEST_Y = 0x0E  # Self test factory calibrated values register
SELF_TEST_Z = 0x0F  # Self test factory calibrated values register
SELF_TEST_A = 0x10  # Self test factory calibrated values register
SMPLRT_DIV = 0x19  # sample rate divisor register
CONFIG = 0x1A  # General configuration register
GYRO_CONFIG = 0x1B  # Gyro specfic configuration register
ACCEL_CONFIG = 0x1C  # Accelerometer specific configration register
INT_PIN_CONFIG = 0x37  # Interrupt pin configuration register
#ACCEL_OUT = 0x3B  # base address for sensor data reads
TEMP_OUT_H = 0x41  # Temperature data high byte register
#GYRO_OUT = 0x43  # base address for sensor data reads
SIG_PATH_RESET = 0x68  # register to reset sensor signal paths
USER_CTRL = 0x6A  # FIFO and I2C Master control register
PWR_MGMT_1 = 0x6B  # Primary power/sleep control register
PWR_MGMT_2 = 0x6C  # Secondary power/sleep control register
WHO_AM_I = 0x75  # Divice ID register
ACCEL_XOUT_H = 0x3B # Accelerometer X axis register
ACCEL_YOUT_H = 0x3D # Accelerometer Y axis register
ACCEL_ZOUT_H = 0x3F # Accelerometer Z axis register
GYRO_XOUT_H = 0x43 # Gyro X axis register
GYRO_YOUT_H = 0x45 # Gyro Y axis register
GYRO_ZOUT_H = 0x47 # Gyro Z axis register

ADDR = None
bus = None
IMU_FRAME = None

# read_word and read_word_2c from http://blog.bitify.co.uk/2013/11/reading-data-from-mpu-6050-on-raspberry.html
def read_word(adr):
    high = bus.read_byte_data(ADDR, adr)
    low = bus.read_byte_data(ADDR, adr+1)
    val = (high << 8) + low
    return val

def read_word_2c(adr):
    val = read_word(adr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

def publish_temp(timer_event):
    temp_msg = Temperature()
    temp_msg.header.frame_id = IMU_FRAME
    temp_msg.temperature = read_word_2c(TEMP_OUT_H)/340.0 + 36.53
    temp_msg.header.stamp = rospy.Time.now()
    temp_pub.publish(temp_msg)


def publish_imu(timer_event):
    imu_msg = Imu()
    imu_msg.header.frame_id = IMU_FRAME

    # Read the acceleration vals
    accel_x = read_word_2c(ACCEL_XOUT_H) / 16384.0
    accel_y = read_word_2c(ACCEL_YOUT_H) / 16384.0
    accel_z = read_word_2c(ACCEL_ZOUT_H) / 16384.0
    
    # Calculate a quaternion representing the orientation
    '''accel = accel_x, accel_y, accel_z
    ref = np.array([0, 0, 1])
    acceln = accel / np.linalg.norm(accel)
    axis = np.cross(acceln, ref)
    angle = np.arccos(np.dot(acceln, ref))
    orientation = quaternion_about_axis(angle, axis)'''

    # Read the gyro vals
    gyro_x = read_word_2c(GYRO_XOUT_H) / 131.0
    gyro_y = read_word_2c(GYRO_YOUT_H) / 131.0
    gyro_z = read_word_2c(GYRO_ZOUT_H) / 131.0
    
    # Load up the IMU message
    '''o = imu_msg.orientation
    o.x, o.y, o.z, o.w = orientation'''

    imu_msg.linear_acceleration.x = accel_x*9.8
    imu_msg.linear_acceleration.y = accel_y*9.8
    imu_msg.linear_acceleration.z = accel_z*9.8

    imu_msg.angular_velocity.x = gyro_x*0.0174
    imu_msg.angular_velocity.y = gyro_y*0.0174
    imu_msg.angular_velocity.z = gyro_z*0.0174

    imu_msg.header.stamp = rospy.Time.now()

    imu_pub.publish(imu_msg)


temp_pub = None
imu_pub = None

if __name__ == '__main__':
    rospy.init_node('imu_node')
    veh_name = os.environ['VEHICLE_NAME']
    bus = smbus.SMBus(rospy.get_param('~bus', 1))
    ADDR = rospy.get_param('~device_address', DEFAULT_ADDRESS)
    if type(ADDR) == str:
        ADDR = int(ADDR, 16)

    IMU_FRAME = rospy.get_param('~imu_frame', 'imu_link')

    bus.write_byte_data(ADDR, PWR_MGMT_1, 0)

    temp_pub = rospy.Publisher('/'+veh_name+'/temperature', Temperature,queue_size=10)
    imu_pub = rospy.Publisher('/'+veh_name+'/imu/data_raw', Imu,queue_size=10)
    imu_timer = rospy.Timer(rospy.Duration(0.02), publish_imu)
    temp_timer = rospy.Timer(rospy.Duration(10), publish_temp)
    rospy.spin()
