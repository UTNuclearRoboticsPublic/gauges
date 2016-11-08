Define and launch virtual gauges for ROS topics. Usage:

roslaunch gauges pitch_yaw_gauges.launch
rostopic pub -r 1 /pitch std_msgFloat64 53.1        (Publish data)
rostopic pub -r 10  /yaw std_msgFloat64 89.95       (Publish data)
