# Solution from a Global Navigation Satellite System (GNSS) receiver without
# fused with any IMU.
modules_common_msgs/Header header 

  # The time of position measurement, seconds since the GPS epoch (Jan 6, 1980).
float64 measurement_time  # In seconds.

  # When velocity is computed from differentiating successive position
  # computations, a non-zero latency is incurred. The velocity refers to the
  # time measurement_time - velocity_latency.
  # When velocity is computed using instantaneous Doppler frequency, there is
  # no latency. We should have velocity_latency = 0.
float32 velocity_latency # default = 0.0  In seconds.

  # Position of the GNSS antenna phase center.
modules_common_msgs/PointLLH position 

  # East/north/up in meters.
modules_common_msgs/Point3D position_std_dev
  # East/north/up in meters per second.
modules_common_msgs/Point3D linear_velocity 
  # East/north/up in meters per second.
modules_common_msgs/Point3D linear_velocity_std_dev 

int32 num_sats   # Number of satellites in position solution.

modules_drivers_gnss_msgs/GnssSolutionType type 
uint32 solution_status
uint32 position_type 
