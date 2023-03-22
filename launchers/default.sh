#!/bin/bash

source /environment.sh
source /code/catkin_ws/devel/setup.bash --extend

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# launching app
#roscore &
#sleep 5


#dt-exec rosrun ros_lf Line_array_publisher_node.py
#dt-exec rosrun ros_lf ros_lf.py
#dt-exec roslaunch mpu_6050_driver imu.launch 
dt-exec rosrun mpu_6050_driver imu_calibration.py
#dt-exec roslaunch ros-lf multiple_nodes.launch veh:=$VEHICLE_NAME

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
