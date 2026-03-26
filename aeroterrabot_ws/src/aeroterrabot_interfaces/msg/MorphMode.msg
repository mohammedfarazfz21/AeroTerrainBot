# MorphMode.msg
# Defines the morphing mode for the AeroTerraBot hybrid platform
#
# Supported modes:
#   WHEEL_MODE  = 0  - Standard wheeled locomotion
#   LEG_MODE    = 1  - Leg elevation stabilization
#   HYBRID_MODE = 2  - Combined wheel + leg terrain traversal
#   FLIGHT_MODE = 3  - Quadrotor propulsion flight

uint8 WHEEL_MODE=0
uint8 LEG_MODE=1
uint8 HYBRID_MODE=2
uint8 FLIGHT_MODE=3

uint8 mode                    # Current requested mode
string mode_name              # Human-readable mode name
builtin_interfaces/Time stamp # Timestamp of mode request
