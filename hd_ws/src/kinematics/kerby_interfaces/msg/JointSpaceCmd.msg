# constants
float64                     CURRENT_STATE = 10000000000.0     # huge value that will never be given as a real command
# modes
uint8                       RELATIVE = 0
uint8                       ABSOLUTE = 1


# variables
uint8                       mode
std_msgs/Float64MultiArray  states