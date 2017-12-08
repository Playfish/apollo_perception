# GNSS solution type.
# It is recommended not using the GNSS solution if solution type is INVALID
# or PROPAGATED.
uint8 INVALID = 0 # Invalid solution due to insufficient observations,

  # integrity warning, etc.
uint8 PROPAGATED = 1  # Propagated by a Kalman filter without new observations.

 # It is recommended using the following types of solution.
uint8 SINGLE = 2 # Standard GNSS solution without any corrections.
uint8 PSRDIFF = 3  # Pseudorange differential solution, including WAAS/SBAS
                   # solution.
uint8 PPP = 4  # Precise Point Positioning (PPP) solution.
uint8 RTK_FLOAT = 5  # Real Time Kinematic (RTK) float solution.
uint8 RTK_INTEGER= 6  # RTK integer solution.
