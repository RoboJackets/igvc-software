%inline
%{

size_t player_devaddr_t_sizeof(void)
{
  return(sizeof(player_devaddr_t));
}
player_devaddr_t* buf_to_player_devaddr_t(void* buf)
{
  return((player_devaddr_t*)(buf));
}
void* player_devaddr_t_to_buf(player_devaddr_t* msg)
{
  return((void*)(msg));
}
size_t player_msghdr_t_sizeof(void)
{
  return(sizeof(player_msghdr_t));
}
player_msghdr_t* buf_to_player_msghdr_t(void* buf)
{
  return((player_msghdr_t*)(buf));
}
void* player_msghdr_t_to_buf(player_msghdr_t* msg)
{
  return((void*)(msg));
}
size_t player_point_2d_t_sizeof(void)
{
  return(sizeof(player_point_2d_t));
}
player_point_2d_t* buf_to_player_point_2d_t(void* buf)
{
  return((player_point_2d_t*)(buf));
}
void* player_point_2d_t_to_buf(player_point_2d_t* msg)
{
  return((void*)(msg));
}
size_t player_point_3d_t_sizeof(void)
{
  return(sizeof(player_point_3d_t));
}
player_point_3d_t* buf_to_player_point_3d_t(void* buf)
{
  return((player_point_3d_t*)(buf));
}
void* player_point_3d_t_to_buf(player_point_3d_t* msg)
{
  return((void*)(msg));
}
size_t player_pose_t_sizeof(void)
{
  return(sizeof(player_pose_t));
}
player_pose_t* buf_to_player_pose_t(void* buf)
{
  return((player_pose_t*)(buf));
}
void* player_pose_t_to_buf(player_pose_t* msg)
{
  return((void*)(msg));
}
size_t player_pose3d_t_sizeof(void)
{
  return(sizeof(player_pose3d_t));
}
player_pose3d_t* buf_to_player_pose3d_t(void* buf)
{
  return((player_pose3d_t*)(buf));
}
void* player_pose3d_t_to_buf(player_pose3d_t* msg)
{
  return((void*)(msg));
}
size_t player_bbox_t_sizeof(void)
{
  return(sizeof(player_bbox_t));
}
player_bbox_t* buf_to_player_bbox_t(void* buf)
{
  return((player_bbox_t*)(buf));
}
void* player_bbox_t_to_buf(player_bbox_t* msg)
{
  return((void*)(msg));
}
size_t player_bbox3d_t_sizeof(void)
{
  return(sizeof(player_bbox3d_t));
}
player_bbox3d_t* buf_to_player_bbox3d_t(void* buf)
{
  return((player_bbox3d_t*)(buf));
}
void* player_bbox3d_t_to_buf(player_bbox3d_t* msg)
{
  return((void*)(msg));
}
size_t player_segment_t_sizeof(void)
{
  return(sizeof(player_segment_t));
}
player_segment_t* buf_to_player_segment_t(void* buf)
{
  return((player_segment_t*)(buf));
}
void* player_segment_t_to_buf(player_segment_t* msg)
{
  return((void*)(msg));
}
size_t player_color_t_sizeof(void)
{
  return(sizeof(player_color_t));
}
player_color_t* buf_to_player_color_t(void* buf)
{
  return((player_color_t*)(buf));
}
void* player_color_t_to_buf(player_color_t* msg)
{
  return((void*)(msg));
}
size_t player_bool_t_sizeof(void)
{
  return(sizeof(player_bool_t));
}
player_bool_t* buf_to_player_bool_t(void* buf)
{
  return((player_bool_t*)(buf));
}
void* player_bool_t_to_buf(player_bool_t* msg)
{
  return((void*)(msg));
}
size_t player_actarray_actuator_t_sizeof(void)
{
  return(sizeof(player_actarray_actuator_t));
}
player_actarray_actuator_t* buf_to_player_actarray_actuator_t(void* buf)
{
  return((player_actarray_actuator_t*)(buf));
}
void* player_actarray_actuator_t_to_buf(player_actarray_actuator_t* msg)
{
  return((void*)(msg));
}
size_t player_actarray_data_t_sizeof(void)
{
  return(sizeof(player_actarray_data_t));
}
player_actarray_data_t* buf_to_player_actarray_data_t(void* buf)
{
  return((player_actarray_data_t*)(buf));
}
void* player_actarray_data_t_to_buf(player_actarray_data_t* msg)
{
  return((void*)(msg));
}
size_t player_actarray_actuatorgeom_t_sizeof(void)
{
  return(sizeof(player_actarray_actuatorgeom_t));
}
player_actarray_actuatorgeom_t* buf_to_player_actarray_actuatorgeom_t(void* buf)
{
  return((player_actarray_actuatorgeom_t*)(buf));
}
void* player_actarray_actuatorgeom_t_to_buf(player_actarray_actuatorgeom_t* msg)
{
  return((void*)(msg));
}
size_t player_actarray_geom_t_sizeof(void)
{
  return(sizeof(player_actarray_geom_t));
}
player_actarray_geom_t* buf_to_player_actarray_geom_t(void* buf)
{
  return((player_actarray_geom_t*)(buf));
}
void* player_actarray_geom_t_to_buf(player_actarray_geom_t* msg)
{
  return((void*)(msg));
}
size_t player_actarray_position_cmd_t_sizeof(void)
{
  return(sizeof(player_actarray_position_cmd_t));
}
player_actarray_position_cmd_t* buf_to_player_actarray_position_cmd_t(void* buf)
{
  return((player_actarray_position_cmd_t*)(buf));
}
void* player_actarray_position_cmd_t_to_buf(player_actarray_position_cmd_t* msg)
{
  return((void*)(msg));
}
size_t player_actarray_speed_cmd_t_sizeof(void)
{
  return(sizeof(player_actarray_speed_cmd_t));
}
player_actarray_speed_cmd_t* buf_to_player_actarray_speed_cmd_t(void* buf)
{
  return((player_actarray_speed_cmd_t*)(buf));
}
void* player_actarray_speed_cmd_t_to_buf(player_actarray_speed_cmd_t* msg)
{
  return((void*)(msg));
}
size_t player_actarray_home_cmd_t_sizeof(void)
{
  return(sizeof(player_actarray_home_cmd_t));
}
player_actarray_home_cmd_t* buf_to_player_actarray_home_cmd_t(void* buf)
{
  return((player_actarray_home_cmd_t*)(buf));
}
void* player_actarray_home_cmd_t_to_buf(player_actarray_home_cmd_t* msg)
{
  return((void*)(msg));
}
size_t player_actarray_power_config_t_sizeof(void)
{
  return(sizeof(player_actarray_power_config_t));
}
player_actarray_power_config_t* buf_to_player_actarray_power_config_t(void* buf)
{
  return((player_actarray_power_config_t*)(buf));
}
void* player_actarray_power_config_t_to_buf(player_actarray_power_config_t* msg)
{
  return((void*)(msg));
}
size_t player_actarray_brakes_config_t_sizeof(void)
{
  return(sizeof(player_actarray_brakes_config_t));
}
player_actarray_brakes_config_t* buf_to_player_actarray_brakes_config_t(void* buf)
{
  return((player_actarray_brakes_config_t*)(buf));
}
void* player_actarray_brakes_config_t_to_buf(player_actarray_brakes_config_t* msg)
{
  return((void*)(msg));
}
size_t player_actarray_speed_config_t_sizeof(void)
{
  return(sizeof(player_actarray_speed_config_t));
}
player_actarray_speed_config_t* buf_to_player_actarray_speed_config_t(void* buf)
{
  return((player_actarray_speed_config_t*)(buf));
}
void* player_actarray_speed_config_t_to_buf(player_actarray_speed_config_t* msg)
{
  return((void*)(msg));
}
size_t player_aio_data_t_sizeof(void)
{
  return(sizeof(player_aio_data_t));
}
player_aio_data_t* buf_to_player_aio_data_t(void* buf)
{
  return((player_aio_data_t*)(buf));
}
void* player_aio_data_t_to_buf(player_aio_data_t* msg)
{
  return((void*)(msg));
}
size_t player_aio_cmd_t_sizeof(void)
{
  return(sizeof(player_aio_cmd_t));
}
player_aio_cmd_t* buf_to_player_aio_cmd_t(void* buf)
{
  return((player_aio_cmd_t*)(buf));
}
void* player_aio_cmd_t_to_buf(player_aio_cmd_t* msg)
{
  return((void*)(msg));
}
size_t player_audio_data_t_sizeof(void)
{
  return(sizeof(player_audio_data_t));
}
player_audio_data_t* buf_to_player_audio_data_t(void* buf)
{
  return((player_audio_data_t*)(buf));
}
void* player_audio_data_t_to_buf(player_audio_data_t* msg)
{
  return((void*)(msg));
}
size_t player_audio_cmd_t_sizeof(void)
{
  return(sizeof(player_audio_cmd_t));
}
player_audio_cmd_t* buf_to_player_audio_cmd_t(void* buf)
{
  return((player_audio_cmd_t*)(buf));
}
void* player_audio_cmd_t_to_buf(player_audio_cmd_t* msg)
{
  return((void*)(msg));
}
size_t player_audiodsp_data_t_sizeof(void)
{
  return(sizeof(player_audiodsp_data_t));
}
player_audiodsp_data_t* buf_to_player_audiodsp_data_t(void* buf)
{
  return((player_audiodsp_data_t*)(buf));
}
void* player_audiodsp_data_t_to_buf(player_audiodsp_data_t* msg)
{
  return((void*)(msg));
}
size_t player_audiodsp_cmd_t_sizeof(void)
{
  return(sizeof(player_audiodsp_cmd_t));
}
player_audiodsp_cmd_t* buf_to_player_audiodsp_cmd_t(void* buf)
{
  return((player_audiodsp_cmd_t*)(buf));
}
void* player_audiodsp_cmd_t_to_buf(player_audiodsp_cmd_t* msg)
{
  return((void*)(msg));
}
size_t player_audiodsp_config_t_sizeof(void)
{
  return(sizeof(player_audiodsp_config_t));
}
player_audiodsp_config_t* buf_to_player_audiodsp_config_t(void* buf)
{
  return((player_audiodsp_config_t*)(buf));
}
void* player_audiodsp_config_t_to_buf(player_audiodsp_config_t* msg)
{
  return((void*)(msg));
}
size_t player_audiomixer_cmd_t_sizeof(void)
{
  return(sizeof(player_audiomixer_cmd_t));
}
player_audiomixer_cmd_t* buf_to_player_audiomixer_cmd_t(void* buf)
{
  return((player_audiomixer_cmd_t*)(buf));
}
void* player_audiomixer_cmd_t_to_buf(player_audiomixer_cmd_t* msg)
{
  return((void*)(msg));
}
size_t player_audiomixer_config_t_sizeof(void)
{
  return(sizeof(player_audiomixer_config_t));
}
player_audiomixer_config_t* buf_to_player_audiomixer_config_t(void* buf)
{
  return((player_audiomixer_config_t*)(buf));
}
void* player_audiomixer_config_t_to_buf(player_audiomixer_config_t* msg)
{
  return((void*)(msg));
}
size_t player_blinkenlight_data_t_sizeof(void)
{
  return(sizeof(player_blinkenlight_data_t));
}
player_blinkenlight_data_t* buf_to_player_blinkenlight_data_t(void* buf)
{
  return((player_blinkenlight_data_t*)(buf));
}
void* player_blinkenlight_data_t_to_buf(player_blinkenlight_data_t* msg)
{
  return((void*)(msg));
}
size_t player_blinkenlight_cmd_t_sizeof(void)
{
  return(sizeof(player_blinkenlight_cmd_t));
}
player_blinkenlight_cmd_t* buf_to_player_blinkenlight_cmd_t(void* buf)
{
  return((player_blinkenlight_cmd_t*)(buf));
}
void* player_blinkenlight_cmd_t_to_buf(player_blinkenlight_cmd_t* msg)
{
  return((void*)(msg));
}
size_t player_blinkenlight_cmd_power_t_sizeof(void)
{
  return(sizeof(player_blinkenlight_cmd_power_t));
}
player_blinkenlight_cmd_power_t* buf_to_player_blinkenlight_cmd_power_t(void* buf)
{
  return((player_blinkenlight_cmd_power_t*)(buf));
}
void* player_blinkenlight_cmd_power_t_to_buf(player_blinkenlight_cmd_power_t* msg)
{
  return((void*)(msg));
}
size_t player_blinkenlight_cmd_color_t_sizeof(void)
{
  return(sizeof(player_blinkenlight_cmd_color_t));
}
player_blinkenlight_cmd_color_t* buf_to_player_blinkenlight_cmd_color_t(void* buf)
{
  return((player_blinkenlight_cmd_color_t*)(buf));
}
void* player_blinkenlight_cmd_color_t_to_buf(player_blinkenlight_cmd_color_t* msg)
{
  return((void*)(msg));
}
size_t player_blinkenlight_cmd_period_t_sizeof(void)
{
  return(sizeof(player_blinkenlight_cmd_period_t));
}
player_blinkenlight_cmd_period_t* buf_to_player_blinkenlight_cmd_period_t(void* buf)
{
  return((player_blinkenlight_cmd_period_t*)(buf));
}
void* player_blinkenlight_cmd_period_t_to_buf(player_blinkenlight_cmd_period_t* msg)
{
  return((void*)(msg));
}
size_t player_blinkenlight_cmd_dutycycle_t_sizeof(void)
{
  return(sizeof(player_blinkenlight_cmd_dutycycle_t));
}
player_blinkenlight_cmd_dutycycle_t* buf_to_player_blinkenlight_cmd_dutycycle_t(void* buf)
{
  return((player_blinkenlight_cmd_dutycycle_t*)(buf));
}
void* player_blinkenlight_cmd_dutycycle_t_to_buf(player_blinkenlight_cmd_dutycycle_t* msg)
{
  return((void*)(msg));
}
size_t player_blobfinder_blob_t_sizeof(void)
{
  return(sizeof(player_blobfinder_blob_t));
}
player_blobfinder_blob_t* buf_to_player_blobfinder_blob_t(void* buf)
{
  return((player_blobfinder_blob_t*)(buf));
}
void* player_blobfinder_blob_t_to_buf(player_blobfinder_blob_t* msg)
{
  return((void*)(msg));
}
size_t player_blobfinder_data_t_sizeof(void)
{
  return(sizeof(player_blobfinder_data_t));
}
player_blobfinder_data_t* buf_to_player_blobfinder_data_t(void* buf)
{
  return((player_blobfinder_data_t*)(buf));
}
void* player_blobfinder_data_t_to_buf(player_blobfinder_data_t* msg)
{
  return((void*)(msg));
}
size_t player_blobfinder_color_config_t_sizeof(void)
{
  return(sizeof(player_blobfinder_color_config_t));
}
player_blobfinder_color_config_t* buf_to_player_blobfinder_color_config_t(void* buf)
{
  return((player_blobfinder_color_config_t*)(buf));
}
void* player_blobfinder_color_config_t_to_buf(player_blobfinder_color_config_t* msg)
{
  return((void*)(msg));
}
size_t player_blobfinder_imager_config_t_sizeof(void)
{
  return(sizeof(player_blobfinder_imager_config_t));
}
player_blobfinder_imager_config_t* buf_to_player_blobfinder_imager_config_t(void* buf)
{
  return((player_blobfinder_imager_config_t*)(buf));
}
void* player_blobfinder_imager_config_t_to_buf(player_blobfinder_imager_config_t* msg)
{
  return((void*)(msg));
}
size_t player_bumper_data_t_sizeof(void)
{
  return(sizeof(player_bumper_data_t));
}
player_bumper_data_t* buf_to_player_bumper_data_t(void* buf)
{
  return((player_bumper_data_t*)(buf));
}
void* player_bumper_data_t_to_buf(player_bumper_data_t* msg)
{
  return((void*)(msg));
}
size_t player_bumper_define_t_sizeof(void)
{
  return(sizeof(player_bumper_define_t));
}
player_bumper_define_t* buf_to_player_bumper_define_t(void* buf)
{
  return((player_bumper_define_t*)(buf));
}
void* player_bumper_define_t_to_buf(player_bumper_define_t* msg)
{
  return((void*)(msg));
}
size_t player_bumper_geom_t_sizeof(void)
{
  return(sizeof(player_bumper_geom_t));
}
player_bumper_geom_t* buf_to_player_bumper_geom_t(void* buf)
{
  return((player_bumper_geom_t*)(buf));
}
void* player_bumper_geom_t_to_buf(player_bumper_geom_t* msg)
{
  return((void*)(msg));
}
size_t player_camera_data_t_sizeof(void)
{
  return(sizeof(player_camera_data_t));
}
player_camera_data_t* buf_to_player_camera_data_t(void* buf)
{
  return((player_camera_data_t*)(buf));
}
void* player_camera_data_t_to_buf(player_camera_data_t* msg)
{
  return((void*)(msg));
}
size_t player_dio_data_t_sizeof(void)
{
  return(sizeof(player_dio_data_t));
}
player_dio_data_t* buf_to_player_dio_data_t(void* buf)
{
  return((player_dio_data_t*)(buf));
}
void* player_dio_data_t_to_buf(player_dio_data_t* msg)
{
  return((void*)(msg));
}
size_t player_dio_cmd_t_sizeof(void)
{
  return(sizeof(player_dio_cmd_t));
}
player_dio_cmd_t* buf_to_player_dio_cmd_t(void* buf)
{
  return((player_dio_cmd_t*)(buf));
}
void* player_dio_cmd_t_to_buf(player_dio_cmd_t* msg)
{
  return((void*)(msg));
}
size_t player_energy_data_t_sizeof(void)
{
  return(sizeof(player_energy_data_t));
}
player_energy_data_t* buf_to_player_energy_data_t(void* buf)
{
  return((player_energy_data_t*)(buf));
}
void* player_energy_data_t_to_buf(player_energy_data_t* msg)
{
  return((void*)(msg));
}
size_t player_energy_chargepolicy_config_t_sizeof(void)
{
  return(sizeof(player_energy_chargepolicy_config_t));
}
player_energy_chargepolicy_config_t* buf_to_player_energy_chargepolicy_config_t(void* buf)
{
  return((player_energy_chargepolicy_config_t*)(buf));
}
void* player_energy_chargepolicy_config_t_to_buf(player_energy_chargepolicy_config_t* msg)
{
  return((void*)(msg));
}
size_t player_fiducial_item_t_sizeof(void)
{
  return(sizeof(player_fiducial_item_t));
}
player_fiducial_item_t* buf_to_player_fiducial_item_t(void* buf)
{
  return((player_fiducial_item_t*)(buf));
}
void* player_fiducial_item_t_to_buf(player_fiducial_item_t* msg)
{
  return((void*)(msg));
}
size_t player_fiducial_data_t_sizeof(void)
{
  return(sizeof(player_fiducial_data_t));
}
player_fiducial_data_t* buf_to_player_fiducial_data_t(void* buf)
{
  return((player_fiducial_data_t*)(buf));
}
void* player_fiducial_data_t_to_buf(player_fiducial_data_t* msg)
{
  return((void*)(msg));
}
size_t player_fiducial_geom_t_sizeof(void)
{
  return(sizeof(player_fiducial_geom_t));
}
player_fiducial_geom_t* buf_to_player_fiducial_geom_t(void* buf)
{
  return((player_fiducial_geom_t*)(buf));
}
void* player_fiducial_geom_t_to_buf(player_fiducial_geom_t* msg)
{
  return((void*)(msg));
}
size_t player_fiducial_fov_t_sizeof(void)
{
  return(sizeof(player_fiducial_fov_t));
}
player_fiducial_fov_t* buf_to_player_fiducial_fov_t(void* buf)
{
  return((player_fiducial_fov_t*)(buf));
}
void* player_fiducial_fov_t_to_buf(player_fiducial_fov_t* msg)
{
  return((void*)(msg));
}
size_t player_fiducial_id_t_sizeof(void)
{
  return(sizeof(player_fiducial_id_t));
}
player_fiducial_id_t* buf_to_player_fiducial_id_t(void* buf)
{
  return((player_fiducial_id_t*)(buf));
}
void* player_fiducial_id_t_to_buf(player_fiducial_id_t* msg)
{
  return((void*)(msg));
}
size_t player_gps_data_t_sizeof(void)
{
  return(sizeof(player_gps_data_t));
}
player_gps_data_t* buf_to_player_gps_data_t(void* buf)
{
  return((player_gps_data_t*)(buf));
}
void* player_gps_data_t_to_buf(player_gps_data_t* msg)
{
  return((void*)(msg));
}
size_t player_graphics2d_cmd_points_t_sizeof(void)
{
  return(sizeof(player_graphics2d_cmd_points_t));
}
player_graphics2d_cmd_points_t* buf_to_player_graphics2d_cmd_points_t(void* buf)
{
  return((player_graphics2d_cmd_points_t*)(buf));
}
void* player_graphics2d_cmd_points_t_to_buf(player_graphics2d_cmd_points_t* msg)
{
  return((void*)(msg));
}
size_t player_graphics2d_cmd_polyline_t_sizeof(void)
{
  return(sizeof(player_graphics2d_cmd_polyline_t));
}
player_graphics2d_cmd_polyline_t* buf_to_player_graphics2d_cmd_polyline_t(void* buf)
{
  return((player_graphics2d_cmd_polyline_t*)(buf));
}
void* player_graphics2d_cmd_polyline_t_to_buf(player_graphics2d_cmd_polyline_t* msg)
{
  return((void*)(msg));
}
size_t player_graphics2d_cmd_polygon_t_sizeof(void)
{
  return(sizeof(player_graphics2d_cmd_polygon_t));
}
player_graphics2d_cmd_polygon_t* buf_to_player_graphics2d_cmd_polygon_t(void* buf)
{
  return((player_graphics2d_cmd_polygon_t*)(buf));
}
void* player_graphics2d_cmd_polygon_t_to_buf(player_graphics2d_cmd_polygon_t* msg)
{
  return((void*)(msg));
}
size_t player_graphics3d_cmd_draw_t_sizeof(void)
{
  return(sizeof(player_graphics3d_cmd_draw_t));
}
player_graphics3d_cmd_draw_t* buf_to_player_graphics3d_cmd_draw_t(void* buf)
{
  return((player_graphics3d_cmd_draw_t*)(buf));
}
void* player_graphics3d_cmd_draw_t_to_buf(player_graphics3d_cmd_draw_t* msg)
{
  return((void*)(msg));
}
size_t player_gripper_data_t_sizeof(void)
{
  return(sizeof(player_gripper_data_t));
}
player_gripper_data_t* buf_to_player_gripper_data_t(void* buf)
{
  return((player_gripper_data_t*)(buf));
}
void* player_gripper_data_t_to_buf(player_gripper_data_t* msg)
{
  return((void*)(msg));
}
size_t player_gripper_cmd_t_sizeof(void)
{
  return(sizeof(player_gripper_cmd_t));
}
player_gripper_cmd_t* buf_to_player_gripper_cmd_t(void* buf)
{
  return((player_gripper_cmd_t*)(buf));
}
void* player_gripper_cmd_t_to_buf(player_gripper_cmd_t* msg)
{
  return((void*)(msg));
}
size_t player_gripper_geom_t_sizeof(void)
{
  return(sizeof(player_gripper_geom_t));
}
player_gripper_geom_t* buf_to_player_gripper_geom_t(void* buf)
{
  return((player_gripper_geom_t*)(buf));
}
void* player_gripper_geom_t_to_buf(player_gripper_geom_t* msg)
{
  return((void*)(msg));
}
size_t player_ir_data_t_sizeof(void)
{
  return(sizeof(player_ir_data_t));
}
player_ir_data_t* buf_to_player_ir_data_t(void* buf)
{
  return((player_ir_data_t*)(buf));
}
void* player_ir_data_t_to_buf(player_ir_data_t* msg)
{
  return((void*)(msg));
}
size_t player_ir_pose_t_sizeof(void)
{
  return(sizeof(player_ir_pose_t));
}
player_ir_pose_t* buf_to_player_ir_pose_t(void* buf)
{
  return((player_ir_pose_t*)(buf));
}
void* player_ir_pose_t_to_buf(player_ir_pose_t* msg)
{
  return((void*)(msg));
}
size_t player_ir_power_req_t_sizeof(void)
{
  return(sizeof(player_ir_power_req_t));
}
player_ir_power_req_t* buf_to_player_ir_power_req_t(void* buf)
{
  return((player_ir_power_req_t*)(buf));
}
void* player_ir_power_req_t_to_buf(player_ir_power_req_t* msg)
{
  return((void*)(msg));
}
size_t player_joystick_data_t_sizeof(void)
{
  return(sizeof(player_joystick_data_t));
}
player_joystick_data_t* buf_to_player_joystick_data_t(void* buf)
{
  return((player_joystick_data_t*)(buf));
}
void* player_joystick_data_t_to_buf(player_joystick_data_t* msg)
{
  return((void*)(msg));
}
size_t player_laser_data_t_sizeof(void)
{
  return(sizeof(player_laser_data_t));
}
player_laser_data_t* buf_to_player_laser_data_t(void* buf)
{
  return((player_laser_data_t*)(buf));
}
void* player_laser_data_t_to_buf(player_laser_data_t* msg)
{
  return((void*)(msg));
}
size_t player_laser_data_scanpose_t_sizeof(void)
{
  return(sizeof(player_laser_data_scanpose_t));
}
player_laser_data_scanpose_t* buf_to_player_laser_data_scanpose_t(void* buf)
{
  return((player_laser_data_scanpose_t*)(buf));
}
void* player_laser_data_scanpose_t_to_buf(player_laser_data_scanpose_t* msg)
{
  return((void*)(msg));
}
size_t player_laser_geom_t_sizeof(void)
{
  return(sizeof(player_laser_geom_t));
}
player_laser_geom_t* buf_to_player_laser_geom_t(void* buf)
{
  return((player_laser_geom_t*)(buf));
}
void* player_laser_geom_t_to_buf(player_laser_geom_t* msg)
{
  return((void*)(msg));
}
size_t player_laser_config_t_sizeof(void)
{
  return(sizeof(player_laser_config_t));
}
player_laser_config_t* buf_to_player_laser_config_t(void* buf)
{
  return((player_laser_config_t*)(buf));
}
void* player_laser_config_t_to_buf(player_laser_config_t* msg)
{
  return((void*)(msg));
}
size_t player_laser_power_config_t_sizeof(void)
{
  return(sizeof(player_laser_power_config_t));
}
player_laser_power_config_t* buf_to_player_laser_power_config_t(void* buf)
{
  return((player_laser_power_config_t*)(buf));
}
void* player_laser_power_config_t_to_buf(player_laser_power_config_t* msg)
{
  return((void*)(msg));
}
size_t player_limb_data_t_sizeof(void)
{
  return(sizeof(player_limb_data_t));
}
player_limb_data_t* buf_to_player_limb_data_t(void* buf)
{
  return((player_limb_data_t*)(buf));
}
void* player_limb_data_t_to_buf(player_limb_data_t* msg)
{
  return((void*)(msg));
}
size_t player_limb_home_cmd_t_sizeof(void)
{
  return(sizeof(player_limb_home_cmd_t));
}
player_limb_home_cmd_t* buf_to_player_limb_home_cmd_t(void* buf)
{
  return((player_limb_home_cmd_t*)(buf));
}
void* player_limb_home_cmd_t_to_buf(player_limb_home_cmd_t* msg)
{
  return((void*)(msg));
}
size_t player_limb_stop_cmd_t_sizeof(void)
{
  return(sizeof(player_limb_stop_cmd_t));
}
player_limb_stop_cmd_t* buf_to_player_limb_stop_cmd_t(void* buf)
{
  return((player_limb_stop_cmd_t*)(buf));
}
void* player_limb_stop_cmd_t_to_buf(player_limb_stop_cmd_t* msg)
{
  return((void*)(msg));
}
size_t player_limb_setpose_cmd_t_sizeof(void)
{
  return(sizeof(player_limb_setpose_cmd_t));
}
player_limb_setpose_cmd_t* buf_to_player_limb_setpose_cmd_t(void* buf)
{
  return((player_limb_setpose_cmd_t*)(buf));
}
void* player_limb_setpose_cmd_t_to_buf(player_limb_setpose_cmd_t* msg)
{
  return((void*)(msg));
}
size_t player_limb_setposition_cmd_t_sizeof(void)
{
  return(sizeof(player_limb_setposition_cmd_t));
}
player_limb_setposition_cmd_t* buf_to_player_limb_setposition_cmd_t(void* buf)
{
  return((player_limb_setposition_cmd_t*)(buf));
}
void* player_limb_setposition_cmd_t_to_buf(player_limb_setposition_cmd_t* msg)
{
  return((void*)(msg));
}
size_t player_limb_vecmove_cmd_t_sizeof(void)
{
  return(sizeof(player_limb_vecmove_cmd_t));
}
player_limb_vecmove_cmd_t* buf_to_player_limb_vecmove_cmd_t(void* buf)
{
  return((player_limb_vecmove_cmd_t*)(buf));
}
void* player_limb_vecmove_cmd_t_to_buf(player_limb_vecmove_cmd_t* msg)
{
  return((void*)(msg));
}
size_t player_limb_power_req_t_sizeof(void)
{
  return(sizeof(player_limb_power_req_t));
}
player_limb_power_req_t* buf_to_player_limb_power_req_t(void* buf)
{
  return((player_limb_power_req_t*)(buf));
}
void* player_limb_power_req_t_to_buf(player_limb_power_req_t* msg)
{
  return((void*)(msg));
}
size_t player_limb_brakes_req_t_sizeof(void)
{
  return(sizeof(player_limb_brakes_req_t));
}
player_limb_brakes_req_t* buf_to_player_limb_brakes_req_t(void* buf)
{
  return((player_limb_brakes_req_t*)(buf));
}
void* player_limb_brakes_req_t_to_buf(player_limb_brakes_req_t* msg)
{
  return((void*)(msg));
}
size_t player_limb_geom_req_t_sizeof(void)
{
  return(sizeof(player_limb_geom_req_t));
}
player_limb_geom_req_t* buf_to_player_limb_geom_req_t(void* buf)
{
  return((player_limb_geom_req_t*)(buf));
}
void* player_limb_geom_req_t_to_buf(player_limb_geom_req_t* msg)
{
  return((void*)(msg));
}
size_t player_limb_speed_req_t_sizeof(void)
{
  return(sizeof(player_limb_speed_req_t));
}
player_limb_speed_req_t* buf_to_player_limb_speed_req_t(void* buf)
{
  return((player_limb_speed_req_t*)(buf));
}
void* player_limb_speed_req_t_to_buf(player_limb_speed_req_t* msg)
{
  return((void*)(msg));
}
size_t player_localize_hypoth_t_sizeof(void)
{
  return(sizeof(player_localize_hypoth_t));
}
player_localize_hypoth_t* buf_to_player_localize_hypoth_t(void* buf)
{
  return((player_localize_hypoth_t*)(buf));
}
void* player_localize_hypoth_t_to_buf(player_localize_hypoth_t* msg)
{
  return((void*)(msg));
}
size_t player_localize_data_t_sizeof(void)
{
  return(sizeof(player_localize_data_t));
}
player_localize_data_t* buf_to_player_localize_data_t(void* buf)
{
  return((player_localize_data_t*)(buf));
}
void* player_localize_data_t_to_buf(player_localize_data_t* msg)
{
  return((void*)(msg));
}
size_t player_localize_set_pose_t_sizeof(void)
{
  return(sizeof(player_localize_set_pose_t));
}
player_localize_set_pose_t* buf_to_player_localize_set_pose_t(void* buf)
{
  return((player_localize_set_pose_t*)(buf));
}
void* player_localize_set_pose_t_to_buf(player_localize_set_pose_t* msg)
{
  return((void*)(msg));
}
size_t player_localize_particle_t_sizeof(void)
{
  return(sizeof(player_localize_particle_t));
}
player_localize_particle_t* buf_to_player_localize_particle_t(void* buf)
{
  return((player_localize_particle_t*)(buf));
}
void* player_localize_particle_t_to_buf(player_localize_particle_t* msg)
{
  return((void*)(msg));
}
size_t player_localize_get_particles_t_sizeof(void)
{
  return(sizeof(player_localize_get_particles_t));
}
player_localize_get_particles_t* buf_to_player_localize_get_particles_t(void* buf)
{
  return((player_localize_get_particles_t*)(buf));
}
void* player_localize_get_particles_t_to_buf(player_localize_get_particles_t* msg)
{
  return((void*)(msg));
}
size_t player_log_set_write_state_t_sizeof(void)
{
  return(sizeof(player_log_set_write_state_t));
}
player_log_set_write_state_t* buf_to_player_log_set_write_state_t(void* buf)
{
  return((player_log_set_write_state_t*)(buf));
}
void* player_log_set_write_state_t_to_buf(player_log_set_write_state_t* msg)
{
  return((void*)(msg));
}
size_t player_log_set_read_state_t_sizeof(void)
{
  return(sizeof(player_log_set_read_state_t));
}
player_log_set_read_state_t* buf_to_player_log_set_read_state_t(void* buf)
{
  return((player_log_set_read_state_t*)(buf));
}
void* player_log_set_read_state_t_to_buf(player_log_set_read_state_t* msg)
{
  return((void*)(msg));
}
size_t player_log_set_read_rewind_t_sizeof(void)
{
  return(sizeof(player_log_set_read_rewind_t));
}
player_log_set_read_rewind_t* buf_to_player_log_set_read_rewind_t(void* buf)
{
  return((player_log_set_read_rewind_t*)(buf));
}
void* player_log_set_read_rewind_t_to_buf(player_log_set_read_rewind_t* msg)
{
  return((void*)(msg));
}
size_t player_log_get_state_t_sizeof(void)
{
  return(sizeof(player_log_get_state_t));
}
player_log_get_state_t* buf_to_player_log_get_state_t(void* buf)
{
  return((player_log_get_state_t*)(buf));
}
void* player_log_get_state_t_to_buf(player_log_get_state_t* msg)
{
  return((void*)(msg));
}
size_t player_log_set_filename_t_sizeof(void)
{
  return(sizeof(player_log_set_filename_t));
}
player_log_set_filename_t* buf_to_player_log_set_filename_t(void* buf)
{
  return((player_log_set_filename_t*)(buf));
}
void* player_log_set_filename_t_to_buf(player_log_set_filename_t* msg)
{
  return((void*)(msg));
}
size_t player_map_info_t_sizeof(void)
{
  return(sizeof(player_map_info_t));
}
player_map_info_t* buf_to_player_map_info_t(void* buf)
{
  return((player_map_info_t*)(buf));
}
void* player_map_info_t_to_buf(player_map_info_t* msg)
{
  return((void*)(msg));
}
size_t player_map_data_t_sizeof(void)
{
  return(sizeof(player_map_data_t));
}
player_map_data_t* buf_to_player_map_data_t(void* buf)
{
  return((player_map_data_t*)(buf));
}
void* player_map_data_t_to_buf(player_map_data_t* msg)
{
  return((void*)(msg));
}
size_t player_map_data_vector_t_sizeof(void)
{
  return(sizeof(player_map_data_vector_t));
}
player_map_data_vector_t* buf_to_player_map_data_vector_t(void* buf)
{
  return((player_map_data_vector_t*)(buf));
}
void* player_map_data_vector_t_to_buf(player_map_data_vector_t* msg)
{
  return((void*)(msg));
}
size_t player_mcom_data_t_sizeof(void)
{
  return(sizeof(player_mcom_data_t));
}
player_mcom_data_t* buf_to_player_mcom_data_t(void* buf)
{
  return((player_mcom_data_t*)(buf));
}
void* player_mcom_data_t_to_buf(player_mcom_data_t* msg)
{
  return((void*)(msg));
}
size_t player_mcom_config_t_sizeof(void)
{
  return(sizeof(player_mcom_config_t));
}
player_mcom_config_t* buf_to_player_mcom_config_t(void* buf)
{
  return((player_mcom_config_t*)(buf));
}
void* player_mcom_config_t_to_buf(player_mcom_config_t* msg)
{
  return((void*)(msg));
}
size_t player_mcom_return_t_sizeof(void)
{
  return(sizeof(player_mcom_return_t));
}
player_mcom_return_t* buf_to_player_mcom_return_t(void* buf)
{
  return((player_mcom_return_t*)(buf));
}
void* player_mcom_return_t_to_buf(player_mcom_return_t* msg)
{
  return((void*)(msg));
}
size_t player_opaque_data_t_sizeof(void)
{
  return(sizeof(player_opaque_data_t));
}
player_opaque_data_t* buf_to_player_opaque_data_t(void* buf)
{
  return((player_opaque_data_t*)(buf));
}
void* player_opaque_data_t_to_buf(player_opaque_data_t* msg)
{
  return((void*)(msg));
}
size_t player_planner_data_t_sizeof(void)
{
  return(sizeof(player_planner_data_t));
}
player_planner_data_t* buf_to_player_planner_data_t(void* buf)
{
  return((player_planner_data_t*)(buf));
}
void* player_planner_data_t_to_buf(player_planner_data_t* msg)
{
  return((void*)(msg));
}
size_t player_planner_cmd_t_sizeof(void)
{
  return(sizeof(player_planner_cmd_t));
}
player_planner_cmd_t* buf_to_player_planner_cmd_t(void* buf)
{
  return((player_planner_cmd_t*)(buf));
}
void* player_planner_cmd_t_to_buf(player_planner_cmd_t* msg)
{
  return((void*)(msg));
}
size_t player_planner_waypoints_req_t_sizeof(void)
{
  return(sizeof(player_planner_waypoints_req_t));
}
player_planner_waypoints_req_t* buf_to_player_planner_waypoints_req_t(void* buf)
{
  return((player_planner_waypoints_req_t*)(buf));
}
void* player_planner_waypoints_req_t_to_buf(player_planner_waypoints_req_t* msg)
{
  return((void*)(msg));
}
size_t player_planner_enable_req_t_sizeof(void)
{
  return(sizeof(player_planner_enable_req_t));
}
player_planner_enable_req_t* buf_to_player_planner_enable_req_t(void* buf)
{
  return((player_planner_enable_req_t*)(buf));
}
void* player_planner_enable_req_t_to_buf(player_planner_enable_req_t* msg)
{
  return((void*)(msg));
}
size_t player_device_devlist_t_sizeof(void)
{
  return(sizeof(player_device_devlist_t));
}
player_device_devlist_t* buf_to_player_device_devlist_t(void* buf)
{
  return((player_device_devlist_t*)(buf));
}
void* player_device_devlist_t_to_buf(player_device_devlist_t* msg)
{
  return((void*)(msg));
}
size_t player_device_driverinfo_t_sizeof(void)
{
  return(sizeof(player_device_driverinfo_t));
}
player_device_driverinfo_t* buf_to_player_device_driverinfo_t(void* buf)
{
  return((player_device_driverinfo_t*)(buf));
}
void* player_device_driverinfo_t_to_buf(player_device_driverinfo_t* msg)
{
  return((void*)(msg));
}
size_t player_device_req_t_sizeof(void)
{
  return(sizeof(player_device_req_t));
}
player_device_req_t* buf_to_player_device_req_t(void* buf)
{
  return((player_device_req_t*)(buf));
}
void* player_device_req_t_to_buf(player_device_req_t* msg)
{
  return((void*)(msg));
}
size_t player_device_data_req_t_sizeof(void)
{
  return(sizeof(player_device_data_req_t));
}
player_device_data_req_t* buf_to_player_device_data_req_t(void* buf)
{
  return((player_device_data_req_t*)(buf));
}
void* player_device_data_req_t_to_buf(player_device_data_req_t* msg)
{
  return((void*)(msg));
}
size_t player_device_datamode_req_t_sizeof(void)
{
  return(sizeof(player_device_datamode_req_t));
}
player_device_datamode_req_t* buf_to_player_device_datamode_req_t(void* buf)
{
  return((player_device_datamode_req_t*)(buf));
}
void* player_device_datamode_req_t_to_buf(player_device_datamode_req_t* msg)
{
  return((void*)(msg));
}
size_t player_device_auth_req_t_sizeof(void)
{
  return(sizeof(player_device_auth_req_t));
}
player_device_auth_req_t* buf_to_player_device_auth_req_t(void* buf)
{
  return((player_device_auth_req_t*)(buf));
}
void* player_device_auth_req_t_to_buf(player_device_auth_req_t* msg)
{
  return((void*)(msg));
}
size_t player_device_nameservice_req_t_sizeof(void)
{
  return(sizeof(player_device_nameservice_req_t));
}
player_device_nameservice_req_t* buf_to_player_device_nameservice_req_t(void* buf)
{
  return((player_device_nameservice_req_t*)(buf));
}
void* player_device_nameservice_req_t_to_buf(player_device_nameservice_req_t* msg)
{
  return((void*)(msg));
}
size_t player_add_replace_rule_req_t_sizeof(void)
{
  return(sizeof(player_add_replace_rule_req_t));
}
player_add_replace_rule_req_t* buf_to_player_add_replace_rule_req_t(void* buf)
{
  return((player_add_replace_rule_req_t*)(buf));
}
void* player_add_replace_rule_req_t_to_buf(player_add_replace_rule_req_t* msg)
{
  return((void*)(msg));
}
size_t player_position1d_data_t_sizeof(void)
{
  return(sizeof(player_position1d_data_t));
}
player_position1d_data_t* buf_to_player_position1d_data_t(void* buf)
{
  return((player_position1d_data_t*)(buf));
}
void* player_position1d_data_t_to_buf(player_position1d_data_t* msg)
{
  return((void*)(msg));
}
size_t player_position1d_cmd_vel_t_sizeof(void)
{
  return(sizeof(player_position1d_cmd_vel_t));
}
player_position1d_cmd_vel_t* buf_to_player_position1d_cmd_vel_t(void* buf)
{
  return((player_position1d_cmd_vel_t*)(buf));
}
void* player_position1d_cmd_vel_t_to_buf(player_position1d_cmd_vel_t* msg)
{
  return((void*)(msg));
}
size_t player_position1d_cmd_pos_t_sizeof(void)
{
  return(sizeof(player_position1d_cmd_pos_t));
}
player_position1d_cmd_pos_t* buf_to_player_position1d_cmd_pos_t(void* buf)
{
  return((player_position1d_cmd_pos_t*)(buf));
}
void* player_position1d_cmd_pos_t_to_buf(player_position1d_cmd_pos_t* msg)
{
  return((void*)(msg));
}
size_t player_position1d_geom_t_sizeof(void)
{
  return(sizeof(player_position1d_geom_t));
}
player_position1d_geom_t* buf_to_player_position1d_geom_t(void* buf)
{
  return((player_position1d_geom_t*)(buf));
}
void* player_position1d_geom_t_to_buf(player_position1d_geom_t* msg)
{
  return((void*)(msg));
}
size_t player_position1d_power_config_t_sizeof(void)
{
  return(sizeof(player_position1d_power_config_t));
}
player_position1d_power_config_t* buf_to_player_position1d_power_config_t(void* buf)
{
  return((player_position1d_power_config_t*)(buf));
}
void* player_position1d_power_config_t_to_buf(player_position1d_power_config_t* msg)
{
  return((void*)(msg));
}
size_t player_position1d_velocity_mode_config_t_sizeof(void)
{
  return(sizeof(player_position1d_velocity_mode_config_t));
}
player_position1d_velocity_mode_config_t* buf_to_player_position1d_velocity_mode_config_t(void* buf)
{
  return((player_position1d_velocity_mode_config_t*)(buf));
}
void* player_position1d_velocity_mode_config_t_to_buf(player_position1d_velocity_mode_config_t* msg)
{
  return((void*)(msg));
}
size_t player_position1d_reset_odom_config_t_sizeof(void)
{
  return(sizeof(player_position1d_reset_odom_config_t));
}
player_position1d_reset_odom_config_t* buf_to_player_position1d_reset_odom_config_t(void* buf)
{
  return((player_position1d_reset_odom_config_t*)(buf));
}
void* player_position1d_reset_odom_config_t_to_buf(player_position1d_reset_odom_config_t* msg)
{
  return((void*)(msg));
}
size_t player_position1d_position_mode_req_t_sizeof(void)
{
  return(sizeof(player_position1d_position_mode_req_t));
}
player_position1d_position_mode_req_t* buf_to_player_position1d_position_mode_req_t(void* buf)
{
  return((player_position1d_position_mode_req_t*)(buf));
}
void* player_position1d_position_mode_req_t_to_buf(player_position1d_position_mode_req_t* msg)
{
  return((void*)(msg));
}
size_t player_position1d_set_odom_req_t_sizeof(void)
{
  return(sizeof(player_position1d_set_odom_req_t));
}
player_position1d_set_odom_req_t* buf_to_player_position1d_set_odom_req_t(void* buf)
{
  return((player_position1d_set_odom_req_t*)(buf));
}
void* player_position1d_set_odom_req_t_to_buf(player_position1d_set_odom_req_t* msg)
{
  return((void*)(msg));
}
size_t player_position1d_speed_pid_req_t_sizeof(void)
{
  return(sizeof(player_position1d_speed_pid_req_t));
}
player_position1d_speed_pid_req_t* buf_to_player_position1d_speed_pid_req_t(void* buf)
{
  return((player_position1d_speed_pid_req_t*)(buf));
}
void* player_position1d_speed_pid_req_t_to_buf(player_position1d_speed_pid_req_t* msg)
{
  return((void*)(msg));
}
size_t player_position1d_position_pid_req_t_sizeof(void)
{
  return(sizeof(player_position1d_position_pid_req_t));
}
player_position1d_position_pid_req_t* buf_to_player_position1d_position_pid_req_t(void* buf)
{
  return((player_position1d_position_pid_req_t*)(buf));
}
void* player_position1d_position_pid_req_t_to_buf(player_position1d_position_pid_req_t* msg)
{
  return((void*)(msg));
}
size_t player_position1d_speed_prof_req_t_sizeof(void)
{
  return(sizeof(player_position1d_speed_prof_req_t));
}
player_position1d_speed_prof_req_t* buf_to_player_position1d_speed_prof_req_t(void* buf)
{
  return((player_position1d_speed_prof_req_t*)(buf));
}
void* player_position1d_speed_prof_req_t_to_buf(player_position1d_speed_prof_req_t* msg)
{
  return((void*)(msg));
}
size_t player_position2d_data_t_sizeof(void)
{
  return(sizeof(player_position2d_data_t));
}
player_position2d_data_t* buf_to_player_position2d_data_t(void* buf)
{
  return((player_position2d_data_t*)(buf));
}
void* player_position2d_data_t_to_buf(player_position2d_data_t* msg)
{
  return((void*)(msg));
}
size_t player_position2d_cmd_vel_t_sizeof(void)
{
  return(sizeof(player_position2d_cmd_vel_t));
}
player_position2d_cmd_vel_t* buf_to_player_position2d_cmd_vel_t(void* buf)
{
  return((player_position2d_cmd_vel_t*)(buf));
}
void* player_position2d_cmd_vel_t_to_buf(player_position2d_cmd_vel_t* msg)
{
  return((void*)(msg));
}
size_t player_position2d_cmd_pos_t_sizeof(void)
{
  return(sizeof(player_position2d_cmd_pos_t));
}
player_position2d_cmd_pos_t* buf_to_player_position2d_cmd_pos_t(void* buf)
{
  return((player_position2d_cmd_pos_t*)(buf));
}
void* player_position2d_cmd_pos_t_to_buf(player_position2d_cmd_pos_t* msg)
{
  return((void*)(msg));
}
size_t player_position2d_cmd_car_t_sizeof(void)
{
  return(sizeof(player_position2d_cmd_car_t));
}
player_position2d_cmd_car_t* buf_to_player_position2d_cmd_car_t(void* buf)
{
  return((player_position2d_cmd_car_t*)(buf));
}
void* player_position2d_cmd_car_t_to_buf(player_position2d_cmd_car_t* msg)
{
  return((void*)(msg));
}
size_t player_position2d_geom_t_sizeof(void)
{
  return(sizeof(player_position2d_geom_t));
}
player_position2d_geom_t* buf_to_player_position2d_geom_t(void* buf)
{
  return((player_position2d_geom_t*)(buf));
}
void* player_position2d_geom_t_to_buf(player_position2d_geom_t* msg)
{
  return((void*)(msg));
}
size_t player_position2d_power_config_t_sizeof(void)
{
  return(sizeof(player_position2d_power_config_t));
}
player_position2d_power_config_t* buf_to_player_position2d_power_config_t(void* buf)
{
  return((player_position2d_power_config_t*)(buf));
}
void* player_position2d_power_config_t_to_buf(player_position2d_power_config_t* msg)
{
  return((void*)(msg));
}
size_t player_position2d_velocity_mode_config_t_sizeof(void)
{
  return(sizeof(player_position2d_velocity_mode_config_t));
}
player_position2d_velocity_mode_config_t* buf_to_player_position2d_velocity_mode_config_t(void* buf)
{
  return((player_position2d_velocity_mode_config_t*)(buf));
}
void* player_position2d_velocity_mode_config_t_to_buf(player_position2d_velocity_mode_config_t* msg)
{
  return((void*)(msg));
}
size_t player_position2d_reset_odom_config_t_sizeof(void)
{
  return(sizeof(player_position2d_reset_odom_config_t));
}
player_position2d_reset_odom_config_t* buf_to_player_position2d_reset_odom_config_t(void* buf)
{
  return((player_position2d_reset_odom_config_t*)(buf));
}
void* player_position2d_reset_odom_config_t_to_buf(player_position2d_reset_odom_config_t* msg)
{
  return((void*)(msg));
}
size_t player_position2d_position_mode_req_t_sizeof(void)
{
  return(sizeof(player_position2d_position_mode_req_t));
}
player_position2d_position_mode_req_t* buf_to_player_position2d_position_mode_req_t(void* buf)
{
  return((player_position2d_position_mode_req_t*)(buf));
}
void* player_position2d_position_mode_req_t_to_buf(player_position2d_position_mode_req_t* msg)
{
  return((void*)(msg));
}
size_t player_position2d_set_odom_req_t_sizeof(void)
{
  return(sizeof(player_position2d_set_odom_req_t));
}
player_position2d_set_odom_req_t* buf_to_player_position2d_set_odom_req_t(void* buf)
{
  return((player_position2d_set_odom_req_t*)(buf));
}
void* player_position2d_set_odom_req_t_to_buf(player_position2d_set_odom_req_t* msg)
{
  return((void*)(msg));
}
size_t player_position2d_speed_pid_req_t_sizeof(void)
{
  return(sizeof(player_position2d_speed_pid_req_t));
}
player_position2d_speed_pid_req_t* buf_to_player_position2d_speed_pid_req_t(void* buf)
{
  return((player_position2d_speed_pid_req_t*)(buf));
}
void* player_position2d_speed_pid_req_t_to_buf(player_position2d_speed_pid_req_t* msg)
{
  return((void*)(msg));
}
size_t player_position2d_position_pid_req_t_sizeof(void)
{
  return(sizeof(player_position2d_position_pid_req_t));
}
player_position2d_position_pid_req_t* buf_to_player_position2d_position_pid_req_t(void* buf)
{
  return((player_position2d_position_pid_req_t*)(buf));
}
void* player_position2d_position_pid_req_t_to_buf(player_position2d_position_pid_req_t* msg)
{
  return((void*)(msg));
}
size_t player_position2d_speed_prof_req_t_sizeof(void)
{
  return(sizeof(player_position2d_speed_prof_req_t));
}
player_position2d_speed_prof_req_t* buf_to_player_position2d_speed_prof_req_t(void* buf)
{
  return((player_position2d_speed_prof_req_t*)(buf));
}
void* player_position2d_speed_prof_req_t_to_buf(player_position2d_speed_prof_req_t* msg)
{
  return((void*)(msg));
}
size_t player_position3d_data_t_sizeof(void)
{
  return(sizeof(player_position3d_data_t));
}
player_position3d_data_t* buf_to_player_position3d_data_t(void* buf)
{
  return((player_position3d_data_t*)(buf));
}
void* player_position3d_data_t_to_buf(player_position3d_data_t* msg)
{
  return((void*)(msg));
}
size_t player_position3d_cmd_pos_t_sizeof(void)
{
  return(sizeof(player_position3d_cmd_pos_t));
}
player_position3d_cmd_pos_t* buf_to_player_position3d_cmd_pos_t(void* buf)
{
  return((player_position3d_cmd_pos_t*)(buf));
}
void* player_position3d_cmd_pos_t_to_buf(player_position3d_cmd_pos_t* msg)
{
  return((void*)(msg));
}
size_t player_position3d_cmd_vel_t_sizeof(void)
{
  return(sizeof(player_position3d_cmd_vel_t));
}
player_position3d_cmd_vel_t* buf_to_player_position3d_cmd_vel_t(void* buf)
{
  return((player_position3d_cmd_vel_t*)(buf));
}
void* player_position3d_cmd_vel_t_to_buf(player_position3d_cmd_vel_t* msg)
{
  return((void*)(msg));
}
size_t player_position3d_geom_t_sizeof(void)
{
  return(sizeof(player_position3d_geom_t));
}
player_position3d_geom_t* buf_to_player_position3d_geom_t(void* buf)
{
  return((player_position3d_geom_t*)(buf));
}
void* player_position3d_geom_t_to_buf(player_position3d_geom_t* msg)
{
  return((void*)(msg));
}
size_t player_position3d_power_config_t_sizeof(void)
{
  return(sizeof(player_position3d_power_config_t));
}
player_position3d_power_config_t* buf_to_player_position3d_power_config_t(void* buf)
{
  return((player_position3d_power_config_t*)(buf));
}
void* player_position3d_power_config_t_to_buf(player_position3d_power_config_t* msg)
{
  return((void*)(msg));
}
size_t player_position3d_position_mode_req_t_sizeof(void)
{
  return(sizeof(player_position3d_position_mode_req_t));
}
player_position3d_position_mode_req_t* buf_to_player_position3d_position_mode_req_t(void* buf)
{
  return((player_position3d_position_mode_req_t*)(buf));
}
void* player_position3d_position_mode_req_t_to_buf(player_position3d_position_mode_req_t* msg)
{
  return((void*)(msg));
}
size_t player_position3d_velocity_mode_config_t_sizeof(void)
{
  return(sizeof(player_position3d_velocity_mode_config_t));
}
player_position3d_velocity_mode_config_t* buf_to_player_position3d_velocity_mode_config_t(void* buf)
{
  return((player_position3d_velocity_mode_config_t*)(buf));
}
void* player_position3d_velocity_mode_config_t_to_buf(player_position3d_velocity_mode_config_t* msg)
{
  return((void*)(msg));
}
size_t player_position3d_set_odom_req_t_sizeof(void)
{
  return(sizeof(player_position3d_set_odom_req_t));
}
player_position3d_set_odom_req_t* buf_to_player_position3d_set_odom_req_t(void* buf)
{
  return((player_position3d_set_odom_req_t*)(buf));
}
void* player_position3d_set_odom_req_t_to_buf(player_position3d_set_odom_req_t* msg)
{
  return((void*)(msg));
}
size_t player_position3d_reset_odom_config_t_sizeof(void)
{
  return(sizeof(player_position3d_reset_odom_config_t));
}
player_position3d_reset_odom_config_t* buf_to_player_position3d_reset_odom_config_t(void* buf)
{
  return((player_position3d_reset_odom_config_t*)(buf));
}
void* player_position3d_reset_odom_config_t_to_buf(player_position3d_reset_odom_config_t* msg)
{
  return((void*)(msg));
}
size_t player_position3d_speed_pid_req_t_sizeof(void)
{
  return(sizeof(player_position3d_speed_pid_req_t));
}
player_position3d_speed_pid_req_t* buf_to_player_position3d_speed_pid_req_t(void* buf)
{
  return((player_position3d_speed_pid_req_t*)(buf));
}
void* player_position3d_speed_pid_req_t_to_buf(player_position3d_speed_pid_req_t* msg)
{
  return((void*)(msg));
}
size_t player_position3d_position_pid_req_t_sizeof(void)
{
  return(sizeof(player_position3d_position_pid_req_t));
}
player_position3d_position_pid_req_t* buf_to_player_position3d_position_pid_req_t(void* buf)
{
  return((player_position3d_position_pid_req_t*)(buf));
}
void* player_position3d_position_pid_req_t_to_buf(player_position3d_position_pid_req_t* msg)
{
  return((void*)(msg));
}
size_t player_position3d_speed_prof_req_t_sizeof(void)
{
  return(sizeof(player_position3d_speed_prof_req_t));
}
player_position3d_speed_prof_req_t* buf_to_player_position3d_speed_prof_req_t(void* buf)
{
  return((player_position3d_speed_prof_req_t*)(buf));
}
void* player_position3d_speed_prof_req_t_to_buf(player_position3d_speed_prof_req_t* msg)
{
  return((void*)(msg));
}
size_t player_power_data_t_sizeof(void)
{
  return(sizeof(player_power_data_t));
}
player_power_data_t* buf_to_player_power_data_t(void* buf)
{
  return((player_power_data_t*)(buf));
}
void* player_power_data_t_to_buf(player_power_data_t* msg)
{
  return((void*)(msg));
}
size_t player_power_chargepolicy_config_t_sizeof(void)
{
  return(sizeof(player_power_chargepolicy_config_t));
}
player_power_chargepolicy_config_t* buf_to_player_power_chargepolicy_config_t(void* buf)
{
  return((player_power_chargepolicy_config_t*)(buf));
}
void* player_power_chargepolicy_config_t_to_buf(player_power_chargepolicy_config_t* msg)
{
  return((void*)(msg));
}
size_t player_ptz_data_t_sizeof(void)
{
  return(sizeof(player_ptz_data_t));
}
player_ptz_data_t* buf_to_player_ptz_data_t(void* buf)
{
  return((player_ptz_data_t*)(buf));
}
void* player_ptz_data_t_to_buf(player_ptz_data_t* msg)
{
  return((void*)(msg));
}
size_t player_ptz_cmd_t_sizeof(void)
{
  return(sizeof(player_ptz_cmd_t));
}
player_ptz_cmd_t* buf_to_player_ptz_cmd_t(void* buf)
{
  return((player_ptz_cmd_t*)(buf));
}
void* player_ptz_cmd_t_to_buf(player_ptz_cmd_t* msg)
{
  return((void*)(msg));
}
size_t player_ptz_geom_t_sizeof(void)
{
  return(sizeof(player_ptz_geom_t));
}
player_ptz_geom_t* buf_to_player_ptz_geom_t(void* buf)
{
  return((player_ptz_geom_t*)(buf));
}
void* player_ptz_geom_t_to_buf(player_ptz_geom_t* msg)
{
  return((void*)(msg));
}
size_t player_ptz_req_generic_t_sizeof(void)
{
  return(sizeof(player_ptz_req_generic_t));
}
player_ptz_req_generic_t* buf_to_player_ptz_req_generic_t(void* buf)
{
  return((player_ptz_req_generic_t*)(buf));
}
void* player_ptz_req_generic_t_to_buf(player_ptz_req_generic_t* msg)
{
  return((void*)(msg));
}
size_t player_ptz_req_control_mode_t_sizeof(void)
{
  return(sizeof(player_ptz_req_control_mode_t));
}
player_ptz_req_control_mode_t* buf_to_player_ptz_req_control_mode_t(void* buf)
{
  return((player_ptz_req_control_mode_t*)(buf));
}
void* player_ptz_req_control_mode_t_to_buf(player_ptz_req_control_mode_t* msg)
{
  return((void*)(msg));
}
size_t player_simulation_data_t_sizeof(void)
{
  return(sizeof(player_simulation_data_t));
}
player_simulation_data_t* buf_to_player_simulation_data_t(void* buf)
{
  return((player_simulation_data_t*)(buf));
}
void* player_simulation_data_t_to_buf(player_simulation_data_t* msg)
{
  return((void*)(msg));
}
size_t player_simulation_cmd_t_sizeof(void)
{
  return(sizeof(player_simulation_cmd_t));
}
player_simulation_cmd_t* buf_to_player_simulation_cmd_t(void* buf)
{
  return((player_simulation_cmd_t*)(buf));
}
void* player_simulation_cmd_t_to_buf(player_simulation_cmd_t* msg)
{
  return((void*)(msg));
}
size_t player_simulation_pose2d_req_t_sizeof(void)
{
  return(sizeof(player_simulation_pose2d_req_t));
}
player_simulation_pose2d_req_t* buf_to_player_simulation_pose2d_req_t(void* buf)
{
  return((player_simulation_pose2d_req_t*)(buf));
}
void* player_simulation_pose2d_req_t_to_buf(player_simulation_pose2d_req_t* msg)
{
  return((void*)(msg));
}
size_t player_simulation_property_int_req_t_sizeof(void)
{
  return(sizeof(player_simulation_property_int_req_t));
}
player_simulation_property_int_req_t* buf_to_player_simulation_property_int_req_t(void* buf)
{
  return((player_simulation_property_int_req_t*)(buf));
}
void* player_simulation_property_int_req_t_to_buf(player_simulation_property_int_req_t* msg)
{
  return((void*)(msg));
}
size_t player_simulation_property_float_req_t_sizeof(void)
{
  return(sizeof(player_simulation_property_float_req_t));
}
player_simulation_property_float_req_t* buf_to_player_simulation_property_float_req_t(void* buf)
{
  return((player_simulation_property_float_req_t*)(buf));
}
void* player_simulation_property_float_req_t_to_buf(player_simulation_property_float_req_t* msg)
{
  return((void*)(msg));
}
size_t player_simulation_property_string_req_t_sizeof(void)
{
  return(sizeof(player_simulation_property_string_req_t));
}
player_simulation_property_string_req_t* buf_to_player_simulation_property_string_req_t(void* buf)
{
  return((player_simulation_property_string_req_t*)(buf));
}
void* player_simulation_property_string_req_t_to_buf(player_simulation_property_string_req_t* msg)
{
  return((void*)(msg));
}
size_t player_sonar_data_t_sizeof(void)
{
  return(sizeof(player_sonar_data_t));
}
player_sonar_data_t* buf_to_player_sonar_data_t(void* buf)
{
  return((player_sonar_data_t*)(buf));
}
void* player_sonar_data_t_to_buf(player_sonar_data_t* msg)
{
  return((void*)(msg));
}
size_t player_sonar_geom_t_sizeof(void)
{
  return(sizeof(player_sonar_geom_t));
}
player_sonar_geom_t* buf_to_player_sonar_geom_t(void* buf)
{
  return((player_sonar_geom_t*)(buf));
}
void* player_sonar_geom_t_to_buf(player_sonar_geom_t* msg)
{
  return((void*)(msg));
}
size_t player_sonar_power_config_t_sizeof(void)
{
  return(sizeof(player_sonar_power_config_t));
}
player_sonar_power_config_t* buf_to_player_sonar_power_config_t(void* buf)
{
  return((player_sonar_power_config_t*)(buf));
}
void* player_sonar_power_config_t_to_buf(player_sonar_power_config_t* msg)
{
  return((void*)(msg));
}
size_t player_sound_cmd_t_sizeof(void)
{
  return(sizeof(player_sound_cmd_t));
}
player_sound_cmd_t* buf_to_player_sound_cmd_t(void* buf)
{
  return((player_sound_cmd_t*)(buf));
}
void* player_sound_cmd_t_to_buf(player_sound_cmd_t* msg)
{
  return((void*)(msg));
}
size_t player_speech_cmd_t_sizeof(void)
{
  return(sizeof(player_speech_cmd_t));
}
player_speech_cmd_t* buf_to_player_speech_cmd_t(void* buf)
{
  return((player_speech_cmd_t*)(buf));
}
void* player_speech_cmd_t_to_buf(player_speech_cmd_t* msg)
{
  return((void*)(msg));
}
size_t player_speech_recognition_data_t_sizeof(void)
{
  return(sizeof(player_speech_recognition_data_t));
}
player_speech_recognition_data_t* buf_to_player_speech_recognition_data_t(void* buf)
{
  return((player_speech_recognition_data_t*)(buf));
}
void* player_speech_recognition_data_t_to_buf(player_speech_recognition_data_t* msg)
{
  return((void*)(msg));
}
size_t player_truth_pose_t_sizeof(void)
{
  return(sizeof(player_truth_pose_t));
}
player_truth_pose_t* buf_to_player_truth_pose_t(void* buf)
{
  return((player_truth_pose_t*)(buf));
}
void* player_truth_pose_t_to_buf(player_truth_pose_t* msg)
{
  return((void*)(msg));
}
size_t player_truth_fiducial_id_t_sizeof(void)
{
  return(sizeof(player_truth_fiducial_id_t));
}
player_truth_fiducial_id_t* buf_to_player_truth_fiducial_id_t(void* buf)
{
  return((player_truth_fiducial_id_t*)(buf));
}
void* player_truth_fiducial_id_t_to_buf(player_truth_fiducial_id_t* msg)
{
  return((void*)(msg));
}
size_t player_waveform_data_t_sizeof(void)
{
  return(sizeof(player_waveform_data_t));
}
player_waveform_data_t* buf_to_player_waveform_data_t(void* buf)
{
  return((player_waveform_data_t*)(buf));
}
void* player_waveform_data_t_to_buf(player_waveform_data_t* msg)
{
  return((void*)(msg));
}
size_t player_wifi_link_t_sizeof(void)
{
  return(sizeof(player_wifi_link_t));
}
player_wifi_link_t* buf_to_player_wifi_link_t(void* buf)
{
  return((player_wifi_link_t*)(buf));
}
void* player_wifi_link_t_to_buf(player_wifi_link_t* msg)
{
  return((void*)(msg));
}
size_t player_wifi_data_t_sizeof(void)
{
  return(sizeof(player_wifi_data_t));
}
player_wifi_data_t* buf_to_player_wifi_data_t(void* buf)
{
  return((player_wifi_data_t*)(buf));
}
void* player_wifi_data_t_to_buf(player_wifi_data_t* msg)
{
  return((void*)(msg));
}
size_t player_wifi_mac_req_t_sizeof(void)
{
  return(sizeof(player_wifi_mac_req_t));
}
player_wifi_mac_req_t* buf_to_player_wifi_mac_req_t(void* buf)
{
  return((player_wifi_mac_req_t*)(buf));
}
void* player_wifi_mac_req_t_to_buf(player_wifi_mac_req_t* msg)
{
  return((void*)(msg));
}
size_t player_wifi_iwspy_addr_req_t_sizeof(void)
{
  return(sizeof(player_wifi_iwspy_addr_req_t));
}
player_wifi_iwspy_addr_req_t* buf_to_player_wifi_iwspy_addr_req_t(void* buf)
{
  return((player_wifi_iwspy_addr_req_t*)(buf));
}
void* player_wifi_iwspy_addr_req_t_to_buf(player_wifi_iwspy_addr_req_t* msg)
{
  return((void*)(msg));
}
size_t player_rfid_tag_t_sizeof(void)
{
  return(sizeof(player_rfid_tag_t));
}
player_rfid_tag_t* buf_to_player_rfid_tag_t(void* buf)
{
  return((player_rfid_tag_t*)(buf));
}
void* player_rfid_tag_t_to_buf(player_rfid_tag_t* msg)
{
  return((void*)(msg));
}
size_t player_rfid_data_t_sizeof(void)
{
  return(sizeof(player_rfid_data_t));
}
player_rfid_data_t* buf_to_player_rfid_data_t(void* buf)
{
  return((player_rfid_data_t*)(buf));
}
void* player_rfid_data_t_to_buf(player_rfid_data_t* msg)
{
  return((void*)(msg));
}
size_t player_rfid_cmd_t_sizeof(void)
{
  return(sizeof(player_rfid_cmd_t));
}
player_rfid_cmd_t* buf_to_player_rfid_cmd_t(void* buf)
{
  return((player_rfid_cmd_t*)(buf));
}
void* player_rfid_cmd_t_to_buf(player_rfid_cmd_t* msg)
{
  return((void*)(msg));
}
size_t player_wsn_node_data_t_sizeof(void)
{
  return(sizeof(player_wsn_node_data_t));
}
player_wsn_node_data_t* buf_to_player_wsn_node_data_t(void* buf)
{
  return((player_wsn_node_data_t*)(buf));
}
void* player_wsn_node_data_t_to_buf(player_wsn_node_data_t* msg)
{
  return((void*)(msg));
}
size_t player_wsn_data_t_sizeof(void)
{
  return(sizeof(player_wsn_data_t));
}
player_wsn_data_t* buf_to_player_wsn_data_t(void* buf)
{
  return((player_wsn_data_t*)(buf));
}
void* player_wsn_data_t_to_buf(player_wsn_data_t* msg)
{
  return((void*)(msg));
}
size_t player_wsn_cmd_t_sizeof(void)
{
  return(sizeof(player_wsn_cmd_t));
}
player_wsn_cmd_t* buf_to_player_wsn_cmd_t(void* buf)
{
  return((player_wsn_cmd_t*)(buf));
}
void* player_wsn_cmd_t_to_buf(player_wsn_cmd_t* msg)
{
  return((void*)(msg));
}
size_t player_wsn_power_config_t_sizeof(void)
{
  return(sizeof(player_wsn_power_config_t));
}
player_wsn_power_config_t* buf_to_player_wsn_power_config_t(void* buf)
{
  return((player_wsn_power_config_t*)(buf));
}
void* player_wsn_power_config_t_to_buf(player_wsn_power_config_t* msg)
{
  return((void*)(msg));
}
size_t player_wsn_datatype_config_t_sizeof(void)
{
  return(sizeof(player_wsn_datatype_config_t));
}
player_wsn_datatype_config_t* buf_to_player_wsn_datatype_config_t(void* buf)
{
  return((player_wsn_datatype_config_t*)(buf));
}
void* player_wsn_datatype_config_t_to_buf(player_wsn_datatype_config_t* msg)
{
  return((void*)(msg));
}
size_t player_wsn_datafreq_config_t_sizeof(void)
{
  return(sizeof(player_wsn_datafreq_config_t));
}
player_wsn_datafreq_config_t* buf_to_player_wsn_datafreq_config_t(void* buf)
{
  return((player_wsn_datafreq_config_t*)(buf));
}
void* player_wsn_datafreq_config_t_to_buf(player_wsn_datafreq_config_t* msg)
{
  return((void*)(msg));
}

%}
