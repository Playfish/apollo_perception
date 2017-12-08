#Invalid solution due to insufficient observations, no initial GNSS, ...
uint8 INVALID = 0

#Use with caution. The covariance matrix may be unavailable or incorrect.
#High-variance result due to aligning, insufficient vehicle dynamics, ...
uint8 CONVERGING = 1

#Safe to use. The INS has fully converged.
uint8 GOOD = 2


Header header

uint8 type
