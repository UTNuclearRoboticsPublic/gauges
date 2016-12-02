#!/usr/bin/env python

# Launch rviz and other plugins in an rqt GUI framework

import subprocess
import rospkg
import time

# Clear the terminal
subprocess.call("reset")

# The main GUI window
process = subprocess.Popen(['rqt', '--clear-config'])
time.sleep(1) # Give the main window time to open

## A container to hold the gauges
## so they can be moved together
#process = subprocess.Popen(['rqt', '--command-start-plugin', 'container'])
#process.wait()

# The first gauge
process = subprocess.Popen(['rqt', '--command-start-plugin', 'rqt_gauges'])
process.wait()

# Parameters for the first gauge
process = subprocess.Popen(['rosparam', 'set', '/rqt_gauges/topic1', '/pitch'])
process = subprocess.Popen(['rosparam', 'set', '/rqt_gauges/gauge_name1', 'Pitch'])

# Publish data for the first gauge
process = subprocess.Popen(['rostopic', 'pub', '-r', '1', '/pitch', 'std_msgs/Float64', '13'])

# The 2nd gauge
process = subprocess.Popen(['rqt', '--command-start-plugin', 'rqt_gauges'])
process.wait()

# Parameters for the 2nd gauge
process = subprocess.Popen(['rosparam', 'set', '/rqt_gauges/topic2', '/yaw'])
process = subprocess.Popen(['rosparam', 'set', '/rqt_gauges/gauge_name2', 'Yaw'])

# Publish data for the second gauge
process = subprocess.Popen(['rostopic', 'pub', '-r', '1', '/yaw', 'std_msgs/Float64', '57'])
