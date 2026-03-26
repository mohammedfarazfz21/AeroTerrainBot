# SystemStatus.msg
# Reports the overall system status of AeroTerraBot

uint8 current_mode            # Current active morph mode
bool arms_deployed            # Whether quadrotor arms are deployed
bool legs_retracted           # Whether legs are in retracted position
bool imu_active               # IMU sensor health
bool px4_connected            # PX4 SITL connection status
bool gazebo_running           # Gazebo simulation status
bool transformation_safe      # Whether transformation is safe to execute
string status_message         # Human-readable status string
builtin_interfaces/Time stamp
