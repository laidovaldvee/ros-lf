#!/usr/bin/env python3

import time
import smbus
import struct
import rospy
import os
import numpy as np

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

ADDR = DEFAULT_ADDRESS


bus = smbus.SMBus(1)
bus.write_byte_data(ADDR, PWR_MGMT_1, 0)

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
    
def get_data():
# Read the acceleration vals
    accel_x = read_word_2c(ACCEL_XOUT_H) / 16384.0
    accel_y = read_word_2c(ACCEL_YOUT_H) / 16384.0
    accel_z = read_word_2c(ACCEL_ZOUT_H) / 16384.0

# Read the gyro vals
    gyro_x = read_word_2c(GYRO_XOUT_H) / 131.0
    gyro_y = read_word_2c(GYRO_YOUT_H) / 131.0
    gyro_z = read_word_2c(GYRO_ZOUT_H) / 131.0

    return accel_x,accel_y,accel_y,gyro_x,gyro_y,gyro_z

def gyro_calibration(calibration_time=10):
    """
        Description: This is a function to get the offset values
            for gyro calibration for mpu6050.
        
        Parameters:
        
        calibration_time[int]: Time in seconds you want to calibrate
            mpu6050. The longer the time the more accurate the
            calibration
    
        Outputs: Array with offsets pertaining to three axes of
            rotation [offset_gx, offset_gy, offset_gz]. Add these
            offsets to your sensor readins later on for more
            accurate readings!
    """
    print('--' * 25)
    print('Beginning Gyro Calibration - Do not move the MPU6050')
    
    # placeholder for the average of tuples in mpu_gyro_array
    a_offsets = [0, 0, 0]
    g_offsets = [0, 0, 0]
    # placeholder for number of calculations we get from the mpu
    num_of_points = 0
    
    # We get the current time and add the calibration time
    end_loop_time = time.time() + calibration_time
    # We end the loop once the calibration time has passed
    while end_loop_time > time.time():
        num_of_points += 1
        (ax, ay, az, gx, gy, gz) = get_data()
        a_offsets[0] += ax
        a_offsets[1] += ay
        a_offsets[2] += az
        g_offsets[0] += gx
        g_offsets[1] += gy
        g_offsets[2] += gz
        
        # This is just to show you its still calibrating :)
        if num_of_points % 100 == 0:
            print('Still Calibrating Gyro... %d points so far' % num_of_points)
        
    print('Calibration for Gyro is Complete! %d points total' % num_of_points)
    a_offsets = [i/num_of_points for i in a_offsets] # we divide by the length to get the mean
    g_offsets = [i/num_of_points for i in g_offsets]
    return a_offsets, g_offsets

if __name__ == '__main__':
    rospy.init_node('imu_node')
    print(gyro_calibration(10))

#[0.011572124639076035, -0.037229917964990374, -0.037229917964990374], [-0.016920262436723527, -0.3424975571049673, -0.6717924604544868]