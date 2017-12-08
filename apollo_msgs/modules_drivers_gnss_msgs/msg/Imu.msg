# Measurements from an inertial measurement unit (IMU). The solution is with
# respect to the IMU by default.
modules_common_msgs/Header header

  # The time of IMU measurement, seconds since the GPS epoch (Jan 6, 1980).
float64 measurement_time   # In seconds.

  # When measurement_span is non-zero, the gyroscope and accelerometer
  # measurements are averaged for the period from
  # (measurement_time - measurement_span) to measurement_time. Usually,
  #      measurement_span = 1 / sampling_frequency.
  #
  # When measurement_span is 0, angular_velocity and linear_acceleration are
  # instantaneous at measurement_time.
float32 measurement_span  #[default = 0.0] In seconds.

  # Forward/left/up in meters per square second.
modules_common_msgs/Point3D linear_acceleration

  # Around forward/left/up axes in radians per second.
modules_common_msgs/Point3D angular_velocity 
