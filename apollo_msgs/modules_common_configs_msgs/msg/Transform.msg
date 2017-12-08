#Also known as "frame_id."
string source_frame

#Also known as "child_frame_id."
string target_frame

#Position of target in the source frame.
modules_common_msgs/Point3D translation

#Activate rotation from the source frame to the target frame.
modules_common_msgs/Quaternion rotation
