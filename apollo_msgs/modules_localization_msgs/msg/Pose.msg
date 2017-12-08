#Position of the vehicle reference point (VRP) in the map reference frame.
#The VRP is the center of rear axle.
modules_common_msgs/PointENU position

#A quaternion that represents the rotation from the map coordinate
#(East/North/Up) to the
#vehicle coordinate (Right/Forward/Up).
modules_common_msgs/Quaternion orientation

#Linear velocity of the VRP in the map reference frame.
#East/north/up in meters per second.
modules_common_msgs/Point3D linear_velocity

#Linear acceleration of the VRP in the map reference frame.
#East/north/up in meters per second.
modules_common_msgs/Point3D linear_acceleration

#Angular velocity of the vehicle in the map reference frame.
#Around east/north/up axes in radians per second.
modules_common_msgs/Point3D angular_velocity

#heading
float64 heading

#Linear acceleration of the VRP in the vehicle reference frame.
#Right/forward/up in meters per square second.
modules_common_msgs/Point3D linear_acceleration_vrf

#Angular velocity of the vehicle in the vehicle reference frame.
#Around right/forward/up axes in radians per second.
modules_common_msgs/Point3D angular_velocity_vrf

#Roll/pitch/yaw that represents a rotation of the intrinsic sequence z-y-x.

#Roll/pitch/yaw in radians.
modules_common_msgs/Point3D euler_angles
