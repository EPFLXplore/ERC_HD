# object types
uint8                       BOX = 0

# operations
uint8                       ADD = 0
uint8                       REMOVE = 1

# variables
uint8                       type
uint8                       operation
string                      name
geometry_msgs/Pose          pose
std_msgs/Float64MultiArray  shape