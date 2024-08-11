# modes
int8 IGNORE_MODE = 0
int8 POSITION_MODE = 1
int8 VELOCITY_MODE = 2
int8 TORQUE_MODE = 3

# variables
std_msgs/Header header
string[] name
float64[] position
float64[] velocity
float64[] torque
int8[] mode
