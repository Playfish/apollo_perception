ConfigServer server

Component[] modules
Component[] hardware
Command tools

#Available options in system, which will show in HMI.
string maps_dir #[default"modules/map/data"]

string vehicles_dir #[default"modules/calibration/data"]

#Target paths for config files.
string global_flagfile
      #[default "modules/common/data/global_flagfile.txt"]

string vehicle_param_pb_target_path
      #[default "modules/common/data/mkz_config.pb.txt"]

string calibration_table_target_path
      #[default "modules/control/conf/lincoln.pb.txt"]

#Paths relative to ROS root directory.
string velodyne_params_target_path
      #[default "share/velodyne_pointcloud/params"]

string velodyne_launch_target_path
      #[default "share/velodyne/launch/start_velodyne.launch"]

string gnss_driver_target_path
      #[default "share/gnss_driver/launch/gnss_driver.launch"]

string gnss_conf_target_path
      #[default "share/gnss_driver/conf/gnss_conf_mkz.txt"]
