# TransformState.msg
# Reports the current transformation state of each subsystem

float64[] leg_positions       # Current leg joint positions [4 legs x 2 joints]
float64[] wheel_velocities    # Current wheel velocities [4 wheels]
float64[] arm_positions       # Current arm joint positions [4 arms]
float64[] rotor_velocities    # Current rotor velocities [4 rotors]
bool transformation_complete  # Whether the last transformation completed
uint8 active_mode             # Currently active morph mode
builtin_interfaces/Time stamp
