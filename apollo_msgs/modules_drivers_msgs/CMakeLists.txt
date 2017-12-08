cmake_minimum_required(VERSION 2.8.3)
project(modules_drivers_msgs)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation std_msgs modules_common_msgs modules_canbus_msgs)

add_message_files(
  DIRECTORY msg
  FILES
  Lka_766.msg
  Lka_767.msg
  Lka_768.msg
  Lka_769.msg
  Num_76b.msg
  Mobileye.msg
  Next_76c.msg
  Next_76d.msg
  DelphiESR.msg
  Motionpower.msg
  Details_73a.msg
  Details_73b.msg
  Details_737.msg
  Details_738.msg
  Details_739.msg
  Esr_sim1_5c0.msg
  Vehicle1_4f0.msg
  Vehicle2_4f1.msg
  Vehicle3_5f2.msg
  Vehicle4_5f3.msg
  Vehicle5_5f4.msg
  Vehicle6_5f5.msg
  Reference_76a.msg
  Esr_valid1_5d0.msg
  Esr_valid2_5d1.msg
  Esr_status1_4e0.msg
  Esr_status2_4e1.msg
  Esr_status3_4e2.msg
  Esr_status4_4e3.msg
  Esr_status5_5e4.msg
  Esr_status6_5e5.msg
  Esr_status7_5e6.msg
  Esr_status8_5e7.msg
  Esr_status9_5e8.msg
  Esr_track01_500.msg
  Aftermarket_669.msg
  Acm_inst_req_7e0.msg
  Acm_inst_resp_7e4.msg
  Can_rx_sim_statusType.msg
  Can_tx_mr_lr_modeType.msg
  Can_rx_wheel_slipType.msg
  Can_rx_clear_faultsType.msg
  Can_rx_sim_functionType.msg
  Can_rx_sim_track_idType.msg
  Can_rx_wiper_statusType.msg
  Can_tx_found_targetType.msg
  Can_tx_track_statusType.msg
  Can_rx_grouping_modeType.msg
  Can_tx_grouping_modeType.msg
  Can_tx_raw_data_modeType.msg
  Esr_trackmotionpower_540.msg
  Can_tx_internal_errorType.msg
  Can_tx_overheat_errorType.msg
  Can_tx_track_oncomingType.msg
  Can_rx_fac_align_cmd_1Type.msg
  Can_rx_fac_align_cmd_2Type.msg
  Can_rx_mmr_upside_downType.msg
  Can_rx_raw_data_enableType.msg
  Can_rx_serv_align_typeType.msg
  Can_rx_blockage_disableType.msg
  Can_rx_lr_only_transmitType.msg
  Can_rx_mr_only_transmitType.msg
  Can_tx_partial_blockageType.msg
  Can_tx_range_perf_errorType.msg
  Can_tx_truck_target_detType.msg
  Can_tx_xcvr_operationalType.msg
  Can_rx_radar_cmd_radiateType.msg
  Can_rx_serv_align_enableType.msg
  Can_rx_yaw_rate_validityType.msg
  Can_tx_sidelobe_blockageType.msg
  Can_tx_system_power_modeType.msg
  Can_rx_auto_align_disableType.msg
  Can_rx_lat_accel_validityType.msg
  Can_rx_turn_signal_statusType.msg
  Can_rx_long_accel_validityType.msg
  Can_rx_steering_angle_signType.msg
  Can_rx_yaw_rate_bias_shiftType.msg
  Can_tx_track_bridge_objectType.msg
  Can_rx_auto_align_convergedType.msg
  Can_tx_recommend_unconvergeType.msg
  Can_tx_track_med_range_modeType.msg
  Can_rx_use_angle_misalignmentType.msg
  Can_rx_vehicle_speed_validityType.msg
  Can_tx_factory_align_status_1Type.msg
  Can_tx_factory_align_status_2Type.msg
  Can_tx_track_grouping_changedType.msg
  Can_tx_vertical_align_updatedType.msg
  Can_rx_steering_angle_validityType.msg
  Can_rx_vehicle_speed_directionType.msg
  Can_rx_steering_angle_rate_signType.msg
  Can_tx_lr_only_grating_lobe_detType.msg

)

generate_messages(DEPENDENCIES geometry_msgs std_msgs modules_common_msgs modules_canbus_msgs)

catkin_package(
  CATKIN_DEPENDS geometry_msgs message_runtime std_msgs modules_common_msgs modules_canbus_msgs)
