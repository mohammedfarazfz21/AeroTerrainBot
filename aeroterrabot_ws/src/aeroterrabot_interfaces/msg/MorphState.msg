std_msgs/Header header

# Morphing physical states
float64 servo_1_angle          # Angle of front morphing servo
float64 servo_2_angle          # Angle of rear morphing servo
float64 linear_act_1_extension # Extension of left linear actuator
float64 linear_act_2_extension # Extension of right linear actuator

# Logical state
bool is_drone_mode             # True if in drone mode, False if in tank mode
