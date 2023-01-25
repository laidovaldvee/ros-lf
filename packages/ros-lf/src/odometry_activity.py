import numpy as np 
#from duckietown_msgs.msg import WheelEncoderStamped, WheelsCmdStamped
import os, sys
#from unit_test import UnitTestOdometry

R = 0.0318 # insert value measured by ruler, in *meters*
baseline_wheel2wheel = 0.1


def delta_phi(encoder_msg, prev_ticks):
    """
        Args:
            encoder_msg: ROS encoder message (ENUM)
            prev_ticks: Previous tick count from the encoders (int)
        Return:
            rotation_wheel: Rotation of the wheel in radians (double)
            ticks: current number of ticks (int)
    """
    ticks = encoder_msg.data
    delta_ticks = ticks - prev_ticks     
    
    N_tot = encoder_msg.resolution #total number of ticks per wheel revolution
    alpha = 2*np.pi/N_tot # rotation per tick in radians 

    delta_phi = delta_ticks*alpha

    return delta_phi, ticks

def delta_phi(ticks,prev_ticks,resolution):
    
    delta_ticks = ticks - prev_ticks     
    N_tot = resolution #total number of ticks per wheel revolution
    alpha = 2*np.pi/N_tot # rotation per tick in radians 

    delta_phi = delta_ticks*alpha

    return delta_phi, ticks

def pose_estimation( R, # radius of wheel (assumed identical) - this is fixed in simulation, and will be imported from your saved calibration for the physical robot
                    baseline_wheel2wheel, # distance from wheel to wheel; 2L of the theory
                    x_prev, # previous x estimate - assume given
                    y_prev, # previous y estimate - assume given
                    theta_prev, # previous orientation estimate - assume given
                    delta_phi_left, # left wheel rotation (rad)
                    delta_phi_right): # right wheel rotation (rad)
    
    """
        Calculate the current Duckiebot pose using the dead-reckoning approach.

        Returns x,y,theta current estimates:
            x_curr, y_curr, theta_curr (:double: values)
    """
    d_A = (delta_phi_right*R+delta_phi_left*R)/2 # robot distance travelled in robot frame [meters]
    Delta_Theta = (delta_phi_right*R-delta_phi_left*R)/2*baseline_wheel2wheel # [radians]
    Delta_x = d_A*np.cos(theta_prev)
    Delta_y = d_A*np.sin(theta_prev)

    x_curr = x_prev+Delta_x 
    y_curr = y_prev+Delta_y 
    theta_curr = theta_prev+Delta_Theta 

    return x_curr, y_curr, theta_curr
"""
x0 = y0 = 0 # meters
theta0 = 0 # radians
prev_tick_left = 0
prev_tick_right = 0

# How much would the wheels rotate with the above tick measurements? 
wheel_ticks = [[3,5],[5,8],[15,18],[30,30],[50,60],[100,120],[150,160],[200,190],[250,220],[300,260],[350,290],[400,310]]

for tick in wheel_ticks:
    l_tick, l_phi = DeltaPhi(tick[0],prev_tick_left)
    r_tick, r_phi = DeltaPhi(tick[1],prev_tick_right)
    prev_tick_left = l_tick
    prev_tick_right = r_tick
    x, y, theta = poseEstimation(R,baseline_wheel2wheel,x0,y0,theta0,l_phi,r_phi)
    x0 = x
    y0 = y
    theta0 = theta
    print(f"The robot has rotated: {np.rad2deg(theta)} degrees")
    print(f"The robot motion in world reference frame: {x} meters on x-axis and {y} meters on y-axis")
"""
#UnitTestOdometry(R, baseline_wheel2wheel, poseEstimation)