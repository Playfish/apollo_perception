  # The driver reads data from this port. This port should always be provided.
modules_drivers_gnss_msgs/Stream data

  # If given, the driver sends commands to this port. If not given, the driver
  # sends commands to the data port.
modules_drivers_gnss_msgs/Stream command

  # The driver gets RTK correction data from this remote port. Usually this is
  # an NTRIP port.
modules_drivers_gnss_msgs/Stream rtk_from

  # If given, the driver sends RTK correction data to this port. If not given,
  # the driver sends RTK correction data to the data port.
modules_drivers_gnss_msgs/Stream rtk_to

string[] login_commands

string[] logout_commands

modules_drivers_gnss_msgs/device_config device_config


modules_drivers_gnss_msgs/RtkSolutionType rtk_solution_type
