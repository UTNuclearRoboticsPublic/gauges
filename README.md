Define and launch virtual gauges for ROS topics. Usage:

roslaunch gauges pitch_yaw_gauges.launch

Publish data:

rostopic pub -r 1 /pitch std_msgs/Float64 53.1

rostopic pub -r 10 -- /yaw std_msgs/Float64 -10.5
