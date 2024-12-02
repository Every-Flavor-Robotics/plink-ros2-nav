std_msgs/Header header


# Constants for Control Mode
uint8 CONTROL_MODE_NONE = 0
uint8 CONTROL_MODE_VOLTAGE = 1
uint8 CONTROL_MODE_VELOCITY = 2
uint8 CONTROL_MODE_POSITION = 3
uint8 CONTROL_MODE_VELOCITY_OPEN_LOOP = 4

uint8 motor_1_control_mode
uint8 motor_2_control_mode
uint8 motor_3_control_mode
uint8 motor_4_control_mode


float32 motor_1_command
float32 motor_2_command
float32 motor_3_command
float32 motor_4_command


