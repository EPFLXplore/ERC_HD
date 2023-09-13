# task types
uint8               NO_TASK = 0
uint8               BUTTON = 1
uint8               NAMED_TARGET = 2
uint8               PLUG_VOLTMETER = 3
uint8               ETHERNET_CABLE = 4
uint8               PICK_ROCK = 5
uint8               RASSOR_SAMPLE = 6

# message variables
uint8               type
uint8               id
geometry_msgs/Pose  pose
string              str_slot    // only used for a named target command for now