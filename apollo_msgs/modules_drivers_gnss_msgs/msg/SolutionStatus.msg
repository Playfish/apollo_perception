uint8 SOL_COMPUTED = 0  # solution computed
uint8 INSUFFICIENT_OBS = 1  # insufficient observations
uint8 NO_CONVERGENCE = 2  # no convergence
uint8 SINGULARITY = 3  # singularity at parameters matrix
uint8 COV_TRACE = 4  # covariance trace exceeds maximum (trace > 1000 m)
uint8 TEST_DIST = 5  # test distance exceeded (max of 3 rejections if distance > 10
                      # km)
uint8 COLD_START = 6  # not yet converged from cold start
uint8 V_H_LIMIT = 7  # height or velocity limits exceeded
uint8 VARIANCE = 8  # variance exceeds limits
uint8 RESIDUALS = 9  # residuals are too large
uint8 INTEGRITY_WARNING = 13  # large residuals make position questionable
uint8 PENDING = 18  # receiver computes its position and determines if the fixed
                     # position is valid
uint8 INVALID_FIX = 19  # the fixed position entered using the fix position
                         # command is invalid
uint8 UNAUTHORIZED = 20  # position type is unauthorized
uint8 INVALID_RATE = 22  # selected logging rate is not supported for this solution type
