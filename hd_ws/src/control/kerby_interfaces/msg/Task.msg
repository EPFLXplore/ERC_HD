# task types
uint8               NO_TASK = 0
uint8               BUTTON = 1
uint8               NAMED_TARGET = 2
uint8               PLUG_VOLTMETER = 3

# message variables
uint8               type
uint8               id
geometry_msgs/Pose  pose
string              str_slot    // only used for a named target command for now