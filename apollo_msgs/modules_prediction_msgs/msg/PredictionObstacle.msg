modules_perception_config_manager_msgs/PerceptionObstacle perception_obstacle #optional
float64 timestamp #optional GPS time in seconds

# the length of the time for this prediction (e.g. 10s)
float64 predicted_period #optional

# can have multiple trajectories per obstacle
Trajectory[] trajectory  
