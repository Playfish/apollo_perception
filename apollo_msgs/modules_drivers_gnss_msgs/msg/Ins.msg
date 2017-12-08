# Solution from a inertial navigation system (INS), which usually fuses GNSS
# and IMU measurements.
modules_common_msgs/Header header 

# The time of position measurement, seconds since the GPS epoch (01/06/1980).
float64 measurement_time   # In seconds.

modules_drivers_gnss_msgs/InsType type

#Position of the IMU.
modules_common_msgs/PointLLH position

# Roll/pitch/yaw that represents a rotation of the intrinsic sequence z-y-x.
# Note: our definition differs from what NovAtel and aviation use.

#Roll/pitch/yaw in radians.
modules_common_msgs/Point3D euler_angles

#East/north/up in meters per second.
modules_common_msgs/Point3D linear_velocity 

#Around forward/left/up axes in radians per second.
modules_common_msgs/Point3D angular_velocity 

#Forward/left/up in meters per square second.
modules_common_msgs/Point3D linear_acceleration 


# The size of a covariance matrix field may be
#  3: then the elements are xx, yy, zz.
#  9: then the elements are xx, xy, xz, yx, yy, yz, zx, zy, zz.
# If the field size is not 3 or 9, treat the field invalid.

# 3-by-3 covariance matrix, in m^2.
float32[] position_covariance # [packed = true]

# 3-by-3 covariance matrix, in rad^2.
float32[] euler_angles_covariance # [packed = true]

# 3-by-3 covariance matrix, in m^2/s^2.
float32[] linear_velocity_covariance # [packed = true]

# 3-by-3 covariance matrix, in rad^2/s^2.
float32[] angular_velocity_covariance # [packed = true]

# 3-by-3 covariance matrix, in m^2/s^4.
float32[] linear_acceleration_covariance # [packed = true]
