# One arm pose in joint space: the six axes of the GoFa.
#
# Radians, like the rest of ROS and like the /joint_states this driver
# publishes - a value read from there can go straight back into a goal. A RAPID
# jointtarget is in degrees; the driver converts on the way out.
float64 j1
float64 j2
float64 j3
float64 j4
float64 j5
float64 j6
