#include <libplayerxdr/playerxdr.h>

int
xdr_player_devaddr_t(XDR* xdrs, player_devaddr_t* msg)
{
  if(xdr_u_int(xdrs,&msg->host) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->robot) != 1)
    return(0);
  if(xdr_u_short(xdrs,&msg->interf) != 1)
    return(0);
  if(xdr_u_short(xdrs,&msg->index) != 1)
    return(0);
  return(1);
}

int
player_devaddr_pack(void* buf, size_t buflen, player_devaddr_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->host) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->robot) != 1)
    return(-1);
  if(xdr_u_short(&xdrs,&msg->interf) != 1)
    return(-1);
  if(xdr_u_short(&xdrs,&msg->index) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_devaddr_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_msghdr_t(XDR* xdrs, player_msghdr_t* msg)
{
  if(xdr_player_devaddr_t(xdrs,&msg->addr) != 1)
    return(0);
  if(xdr_u_char(xdrs,&msg->type) != 1)
    return(0);
  if(xdr_u_char(xdrs,&msg->subtype) != 1)
    return(0);
  if(xdr_double(xdrs,&msg->timestamp) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->seq) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->size) != 1)
    return(0);
  return(1);
}

int
player_msghdr_pack(void* buf, size_t buflen, player_msghdr_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_player_devaddr_t(&xdrs,&msg->addr) != 1)
    return(-1);
  if(xdr_u_char(&xdrs,&msg->type) != 1)
    return(-1);
  if(xdr_u_char(&xdrs,&msg->subtype) != 1)
    return(-1);
  if(xdr_double(&xdrs,&msg->timestamp) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->seq) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->size) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_msghdr_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_point_2d_t(XDR* xdrs, player_point_2d_t* msg)
{
  if(xdr_float(xdrs,&msg->px) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->py) != 1)
    return(0);
  return(1);
}

int
player_point_2d_pack(void* buf, size_t buflen, player_point_2d_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_float(&xdrs,&msg->px) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->py) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_point_2d_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_point_3d_t(XDR* xdrs, player_point_3d_t* msg)
{
  if(xdr_float(xdrs,&msg->px) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->py) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->pz) != 1)
    return(0);
  return(1);
}

int
player_point_3d_pack(void* buf, size_t buflen, player_point_3d_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_float(&xdrs,&msg->px) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->py) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->pz) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_point_3d_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_pose_t(XDR* xdrs, player_pose_t* msg)
{
  if(xdr_float(xdrs,&msg->px) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->py) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->pa) != 1)
    return(0);
  return(1);
}

int
player_pose_pack(void* buf, size_t buflen, player_pose_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_float(&xdrs,&msg->px) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->py) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->pa) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_pose_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_pose3d_t(XDR* xdrs, player_pose3d_t* msg)
{
  if(xdr_float(xdrs,&msg->px) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->py) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->pz) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->proll) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->ppitch) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->pyaw) != 1)
    return(0);
  return(1);
}

int
player_pose3d_pack(void* buf, size_t buflen, player_pose3d_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_float(&xdrs,&msg->px) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->py) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->pz) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->proll) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->ppitch) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->pyaw) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_pose3d_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_bbox_t(XDR* xdrs, player_bbox_t* msg)
{
  if(xdr_float(xdrs,&msg->sw) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->sl) != 1)
    return(0);
  return(1);
}

int
player_bbox_pack(void* buf, size_t buflen, player_bbox_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_float(&xdrs,&msg->sw) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->sl) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_bbox_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_bbox3d_t(XDR* xdrs, player_bbox3d_t* msg)
{
  if(xdr_float(xdrs,&msg->sw) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->sl) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->sh) != 1)
    return(0);
  return(1);
}

int
player_bbox3d_pack(void* buf, size_t buflen, player_bbox3d_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_float(&xdrs,&msg->sw) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->sl) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->sh) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_bbox3d_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_segment_t(XDR* xdrs, player_segment_t* msg)
{
  if(xdr_float(xdrs,&msg->x0) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->y0) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->x1) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->y1) != 1)
    return(0);
  return(1);
}

int
player_segment_pack(void* buf, size_t buflen, player_segment_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_float(&xdrs,&msg->x0) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->y0) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->x1) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->y1) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_segment_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_color_t(XDR* xdrs, player_color_t* msg)
{
  if(xdr_u_char(xdrs,&msg->alpha) != 1)
    return(0);
  if(xdr_u_char(xdrs,&msg->red) != 1)
    return(0);
  if(xdr_u_char(xdrs,&msg->green) != 1)
    return(0);
  if(xdr_u_char(xdrs,&msg->blue) != 1)
    return(0);
  return(1);
}

int
player_color_pack(void* buf, size_t buflen, player_color_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_char(&xdrs,&msg->alpha) != 1)
    return(-1);
  if(xdr_u_char(&xdrs,&msg->red) != 1)
    return(-1);
  if(xdr_u_char(&xdrs,&msg->green) != 1)
    return(-1);
  if(xdr_u_char(&xdrs,&msg->blue) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_color_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_bool_t(XDR* xdrs, player_bool_t* msg)
{
  if(xdr_u_char(xdrs,&msg->state) != 1)
    return(0);
  return(1);
}

int
player_bool_pack(void* buf, size_t buflen, player_bool_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_char(&xdrs,&msg->state) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_bool_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_actarray_actuator_t(XDR* xdrs, player_actarray_actuator_t* msg)
{
  if(xdr_float(xdrs,&msg->position) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->speed) != 1)
    return(0);
  if(xdr_u_char(xdrs,&msg->state) != 1)
    return(0);
  return(1);
}

int
player_actarray_actuator_pack(void* buf, size_t buflen, player_actarray_actuator_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_float(&xdrs,&msg->position) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->speed) != 1)
    return(-1);
  if(xdr_u_char(&xdrs,&msg->state) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_actarray_actuator_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_actarray_data_t(XDR* xdrs, player_actarray_data_t* msg)
{
  if(xdr_u_int(xdrs,&msg->actuators_count) != 1)
    return(0);
  {
    player_actarray_actuator_t* actuators_p = msg->actuators;
    if(xdr_array(xdrs, (char**)&actuators_p, &msg->actuators_count, PLAYER_ACTARRAY_NUM_ACTUATORS, sizeof(player_actarray_actuator_t), (xdrproc_t)xdr_player_actarray_actuator_t) != 1)
      return(0);
  }
  return(1);
}

int
player_actarray_data_pack(void* buf, size_t buflen, player_actarray_data_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->actuators_count) != 1)
    return(-1);
  {
    player_actarray_actuator_t* actuators_p = msg->actuators;
    if(xdr_array(&xdrs, (char**)&actuators_p, &msg->actuators_count, PLAYER_ACTARRAY_NUM_ACTUATORS, sizeof(player_actarray_actuator_t), (xdrproc_t)xdr_player_actarray_actuator_t) != 1)
      return(-1);
  }
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_actarray_data_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_actarray_actuatorgeom_t(XDR* xdrs, player_actarray_actuatorgeom_t* msg)
{
  if(xdr_u_char(xdrs,&msg->type) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->min) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->centre) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->max) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->home) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->config_speed) != 1)
    return(0);
  if(xdr_u_char(xdrs,&msg->hasbrakes) != 1)
    return(0);
  return(1);
}

int
player_actarray_actuatorgeom_pack(void* buf, size_t buflen, player_actarray_actuatorgeom_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_char(&xdrs,&msg->type) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->min) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->centre) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->max) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->home) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->config_speed) != 1)
    return(-1);
  if(xdr_u_char(&xdrs,&msg->hasbrakes) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_actarray_actuatorgeom_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_actarray_geom_t(XDR* xdrs, player_actarray_geom_t* msg)
{
  if(xdr_u_int(xdrs,&msg->actuators_count) != 1)
    return(0);
  {
    player_actarray_actuatorgeom_t* actuators_p = msg->actuators;
    if(xdr_array(xdrs, (char**)&actuators_p, &msg->actuators_count, PLAYER_ACTARRAY_NUM_ACTUATORS, sizeof(player_actarray_actuatorgeom_t), (xdrproc_t)xdr_player_actarray_actuatorgeom_t) != 1)
      return(0);
  }
  return(1);
}

int
player_actarray_geom_pack(void* buf, size_t buflen, player_actarray_geom_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->actuators_count) != 1)
    return(-1);
  {
    player_actarray_actuatorgeom_t* actuators_p = msg->actuators;
    if(xdr_array(&xdrs, (char**)&actuators_p, &msg->actuators_count, PLAYER_ACTARRAY_NUM_ACTUATORS, sizeof(player_actarray_actuatorgeom_t), (xdrproc_t)xdr_player_actarray_actuatorgeom_t) != 1)
      return(-1);
  }
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_actarray_geom_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_actarray_position_cmd_t(XDR* xdrs, player_actarray_position_cmd_t* msg)
{
  if(xdr_int(xdrs,&msg->joint) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->position) != 1)
    return(0);
  return(1);
}

int
player_actarray_position_cmd_pack(void* buf, size_t buflen, player_actarray_position_cmd_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_int(&xdrs,&msg->joint) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->position) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_actarray_position_cmd_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_actarray_speed_cmd_t(XDR* xdrs, player_actarray_speed_cmd_t* msg)
{
  if(xdr_int(xdrs,&msg->joint) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->speed) != 1)
    return(0);
  return(1);
}

int
player_actarray_speed_cmd_pack(void* buf, size_t buflen, player_actarray_speed_cmd_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_int(&xdrs,&msg->joint) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->speed) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_actarray_speed_cmd_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_actarray_home_cmd_t(XDR* xdrs, player_actarray_home_cmd_t* msg)
{
  if(xdr_int(xdrs,&msg->joint) != 1)
    return(0);
  return(1);
}

int
player_actarray_home_cmd_pack(void* buf, size_t buflen, player_actarray_home_cmd_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_int(&xdrs,&msg->joint) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_actarray_home_cmd_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_actarray_power_config_t(XDR* xdrs, player_actarray_power_config_t* msg)
{
  if(xdr_u_char(xdrs,&msg->value) != 1)
    return(0);
  return(1);
}

int
player_actarray_power_config_pack(void* buf, size_t buflen, player_actarray_power_config_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_char(&xdrs,&msg->value) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_actarray_power_config_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_actarray_brakes_config_t(XDR* xdrs, player_actarray_brakes_config_t* msg)
{
  if(xdr_u_char(xdrs,&msg->value) != 1)
    return(0);
  return(1);
}

int
player_actarray_brakes_config_pack(void* buf, size_t buflen, player_actarray_brakes_config_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_char(&xdrs,&msg->value) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_actarray_brakes_config_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_actarray_speed_config_t(XDR* xdrs, player_actarray_speed_config_t* msg)
{
  if(xdr_int(xdrs,&msg->joint) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->speed) != 1)
    return(0);
  return(1);
}

int
player_actarray_speed_config_pack(void* buf, size_t buflen, player_actarray_speed_config_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_int(&xdrs,&msg->joint) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->speed) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_actarray_speed_config_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_aio_data_t(XDR* xdrs, player_aio_data_t* msg)
{
  if(xdr_u_int(xdrs,&msg->voltages_count) != 1)
    return(0);
  {
    float* voltages_p = msg->voltages;
    if(xdr_array(xdrs, (char**)&voltages_p, &msg->voltages_count, PLAYER_AIO_MAX_INPUTS, sizeof(float), (xdrproc_t)xdr_float) != 1)
      return(0);
  }
  return(1);
}

int
player_aio_data_pack(void* buf, size_t buflen, player_aio_data_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->voltages_count) != 1)
    return(-1);
  {
    float* voltages_p = msg->voltages;
    if(xdr_array(&xdrs, (char**)&voltages_p, &msg->voltages_count, PLAYER_AIO_MAX_INPUTS, sizeof(float), (xdrproc_t)xdr_float) != 1)
      return(-1);
  }
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_aio_data_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_aio_cmd_t(XDR* xdrs, player_aio_cmd_t* msg)
{
  if(xdr_u_int(xdrs,&msg->id) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->voltage) != 1)
    return(0);
  return(1);
}

int
player_aio_cmd_pack(void* buf, size_t buflen, player_aio_cmd_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->id) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->voltage) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_aio_cmd_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_audio_data_t(XDR* xdrs, player_audio_data_t* msg)
{
  if(xdr_u_int(xdrs,&msg->frequency_count) != 1)
    return(0);
  {
    float* frequency_p = msg->frequency;
    if(xdr_array(xdrs, (char**)&frequency_p, &msg->frequency_count, PLAYER_AUDIO_PAIRS, sizeof(float), (xdrproc_t)xdr_float) != 1)
      return(0);
  }
  if(xdr_u_int(xdrs,&msg->amplitude_count) != 1)
    return(0);
  {
    float* amplitude_p = msg->amplitude;
    if(xdr_array(xdrs, (char**)&amplitude_p, &msg->amplitude_count, PLAYER_AUDIO_PAIRS, sizeof(float), (xdrproc_t)xdr_float) != 1)
      return(0);
  }
  return(1);
}

int
player_audio_data_pack(void* buf, size_t buflen, player_audio_data_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->frequency_count) != 1)
    return(-1);
  {
    float* frequency_p = msg->frequency;
    if(xdr_array(&xdrs, (char**)&frequency_p, &msg->frequency_count, PLAYER_AUDIO_PAIRS, sizeof(float), (xdrproc_t)xdr_float) != 1)
      return(-1);
  }
  if(xdr_u_int(&xdrs,&msg->amplitude_count) != 1)
    return(-1);
  {
    float* amplitude_p = msg->amplitude;
    if(xdr_array(&xdrs, (char**)&amplitude_p, &msg->amplitude_count, PLAYER_AUDIO_PAIRS, sizeof(float), (xdrproc_t)xdr_float) != 1)
      return(-1);
  }
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_audio_data_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_audio_cmd_t(XDR* xdrs, player_audio_cmd_t* msg)
{
  if(xdr_float(xdrs,&msg->frequency) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->amplitude) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->duration) != 1)
    return(0);
  return(1);
}

int
player_audio_cmd_pack(void* buf, size_t buflen, player_audio_cmd_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_float(&xdrs,&msg->frequency) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->amplitude) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->duration) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_audio_cmd_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_audiodsp_data_t(XDR* xdrs, player_audiodsp_data_t* msg)
{
  if(xdr_u_int(xdrs,&msg->frequency_count) != 1)
    return(0);
  {
    float* frequency_p = msg->frequency;
    if(xdr_array(xdrs, (char**)&frequency_p, &msg->frequency_count, PLAYER_AUDIODSP_MAX_FREQS, sizeof(float), (xdrproc_t)xdr_float) != 1)
      return(0);
  }
  if(xdr_u_int(xdrs,&msg->amplitude_count) != 1)
    return(0);
  {
    float* amplitude_p = msg->amplitude;
    if(xdr_array(xdrs, (char**)&amplitude_p, &msg->amplitude_count, PLAYER_AUDIODSP_MAX_FREQS, sizeof(float), (xdrproc_t)xdr_float) != 1)
      return(0);
  }
  return(1);
}

int
player_audiodsp_data_pack(void* buf, size_t buflen, player_audiodsp_data_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->frequency_count) != 1)
    return(-1);
  {
    float* frequency_p = msg->frequency;
    if(xdr_array(&xdrs, (char**)&frequency_p, &msg->frequency_count, PLAYER_AUDIODSP_MAX_FREQS, sizeof(float), (xdrproc_t)xdr_float) != 1)
      return(-1);
  }
  if(xdr_u_int(&xdrs,&msg->amplitude_count) != 1)
    return(-1);
  {
    float* amplitude_p = msg->amplitude;
    if(xdr_array(&xdrs, (char**)&amplitude_p, &msg->amplitude_count, PLAYER_AUDIODSP_MAX_FREQS, sizeof(float), (xdrproc_t)xdr_float) != 1)
      return(-1);
  }
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_audiodsp_data_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_audiodsp_cmd_t(XDR* xdrs, player_audiodsp_cmd_t* msg)
{
  if(xdr_float(xdrs,&msg->frequency) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->amplitude) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->duration) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->bit_string_count) != 1)
    return(0);
  {
    uint8_t* bit_string_p = msg->bit_string;
    if(xdr_bytes(xdrs, (char**)&bit_string_p, &msg->bit_string_count, PLAYER_AUDIODSP_MAX_BITSTRING_LEN) != 1)
      return(0);
  }
  return(1);
}

int
player_audiodsp_cmd_pack(void* buf, size_t buflen, player_audiodsp_cmd_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_float(&xdrs,&msg->frequency) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->amplitude) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->duration) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->bit_string_count) != 1)
    return(-1);
  {
    uint8_t* bit_string_p = msg->bit_string;
    if(xdr_bytes(&xdrs, (char**)&bit_string_p, &msg->bit_string_count, PLAYER_AUDIODSP_MAX_BITSTRING_LEN) != 1)
      return(-1);
  }
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_audiodsp_cmd_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_audiodsp_config_t(XDR* xdrs, player_audiodsp_config_t* msg)
{
  if(xdr_int(xdrs,&msg->format) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->frequency) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->channels) != 1)
    return(0);
  return(1);
}

int
player_audiodsp_config_pack(void* buf, size_t buflen, player_audiodsp_config_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_int(&xdrs,&msg->format) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->frequency) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->channels) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_audiodsp_config_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_audiomixer_cmd_t(XDR* xdrs, player_audiomixer_cmd_t* msg)
{
  if(xdr_u_int(xdrs,&msg->left) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->right) != 1)
    return(0);
  return(1);
}

int
player_audiomixer_cmd_pack(void* buf, size_t buflen, player_audiomixer_cmd_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->left) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->right) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_audiomixer_cmd_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_audiomixer_config_t(XDR* xdrs, player_audiomixer_config_t* msg)
{
  if(xdr_u_int(xdrs,&msg->master_left) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->master_right) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->pcm_left) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->pcm_right) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->line_left) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->line_right) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->mic_left) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->mic_right) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->i_gain) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->o_gain) != 1)
    return(0);
  return(1);
}

int
player_audiomixer_config_pack(void* buf, size_t buflen, player_audiomixer_config_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->master_left) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->master_right) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->pcm_left) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->pcm_right) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->line_left) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->line_right) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->mic_left) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->mic_right) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->i_gain) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->o_gain) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_audiomixer_config_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_blinkenlight_data_t(XDR* xdrs, player_blinkenlight_data_t* msg)
{
  if(xdr_u_char(xdrs,&msg->enable) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->period) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->dutycycle) != 1)
    return(0);
  if(xdr_player_color_t(xdrs,&msg->color) != 1)
    return(0);
  return(1);
}

int
player_blinkenlight_data_pack(void* buf, size_t buflen, player_blinkenlight_data_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_char(&xdrs,&msg->enable) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->period) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->dutycycle) != 1)
    return(-1);
  if(xdr_player_color_t(&xdrs,&msg->color) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_blinkenlight_data_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_blinkenlight_cmd_t(XDR* xdrs, player_blinkenlight_cmd_t* msg)
{
  if(xdr_u_char(xdrs,&msg->enable) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->period) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->dutycycle) != 1)
    return(0);
  if(xdr_player_color_t(xdrs,&msg->color) != 1)
    return(0);
  return(1);
}

int
player_blinkenlight_cmd_pack(void* buf, size_t buflen, player_blinkenlight_cmd_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_char(&xdrs,&msg->enable) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->period) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->dutycycle) != 1)
    return(-1);
  if(xdr_player_color_t(&xdrs,&msg->color) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_blinkenlight_cmd_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_blinkenlight_cmd_power_t(XDR* xdrs, player_blinkenlight_cmd_power_t* msg)
{
  if(xdr_u_char(xdrs,&msg->enable) != 1)
    return(0);
  return(1);
}

int
player_blinkenlight_cmd_power_pack(void* buf, size_t buflen, player_blinkenlight_cmd_power_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_char(&xdrs,&msg->enable) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_blinkenlight_cmd_power_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_blinkenlight_cmd_color_t(XDR* xdrs, player_blinkenlight_cmd_color_t* msg)
{
  if(xdr_player_color_t(xdrs,&msg->color) != 1)
    return(0);
  return(1);
}

int
player_blinkenlight_cmd_color_pack(void* buf, size_t buflen, player_blinkenlight_cmd_color_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_player_color_t(&xdrs,&msg->color) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_blinkenlight_cmd_color_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_blinkenlight_cmd_period_t(XDR* xdrs, player_blinkenlight_cmd_period_t* msg)
{
  if(xdr_float(xdrs,&msg->period) != 1)
    return(0);
  return(1);
}

int
player_blinkenlight_cmd_period_pack(void* buf, size_t buflen, player_blinkenlight_cmd_period_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_float(&xdrs,&msg->period) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_blinkenlight_cmd_period_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_blinkenlight_cmd_dutycycle_t(XDR* xdrs, player_blinkenlight_cmd_dutycycle_t* msg)
{
  if(xdr_float(xdrs,&msg->dutycycle) != 1)
    return(0);
  return(1);
}

int
player_blinkenlight_cmd_dutycycle_pack(void* buf, size_t buflen, player_blinkenlight_cmd_dutycycle_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_float(&xdrs,&msg->dutycycle) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_blinkenlight_cmd_dutycycle_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_blobfinder_blob_t(XDR* xdrs, player_blobfinder_blob_t* msg)
{
  if(xdr_u_int(xdrs,&msg->id) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->color) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->area) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->x) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->y) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->left) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->right) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->top) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->bottom) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->range) != 1)
    return(0);
  return(1);
}

int
player_blobfinder_blob_pack(void* buf, size_t buflen, player_blobfinder_blob_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->id) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->color) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->area) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->x) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->y) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->left) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->right) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->top) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->bottom) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->range) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_blobfinder_blob_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_blobfinder_data_t(XDR* xdrs, player_blobfinder_data_t* msg)
{
  if(xdr_u_int(xdrs,&msg->width) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->height) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->blobs_count) != 1)
    return(0);
  {
    player_blobfinder_blob_t* blobs_p = msg->blobs;
    if(xdr_array(xdrs, (char**)&blobs_p, &msg->blobs_count, PLAYER_BLOBFINDER_MAX_BLOBS, sizeof(player_blobfinder_blob_t), (xdrproc_t)xdr_player_blobfinder_blob_t) != 1)
      return(0);
  }
  return(1);
}

int
player_blobfinder_data_pack(void* buf, size_t buflen, player_blobfinder_data_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->width) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->height) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->blobs_count) != 1)
    return(-1);
  {
    player_blobfinder_blob_t* blobs_p = msg->blobs;
    if(xdr_array(&xdrs, (char**)&blobs_p, &msg->blobs_count, PLAYER_BLOBFINDER_MAX_BLOBS, sizeof(player_blobfinder_blob_t), (xdrproc_t)xdr_player_blobfinder_blob_t) != 1)
      return(-1);
  }
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_blobfinder_data_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_blobfinder_color_config_t(XDR* xdrs, player_blobfinder_color_config_t* msg)
{
  if(xdr_u_int(xdrs,&msg->rmin) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->rmax) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->gmin) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->gmax) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->bmin) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->bmax) != 1)
    return(0);
  return(1);
}

int
player_blobfinder_color_config_pack(void* buf, size_t buflen, player_blobfinder_color_config_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->rmin) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->rmax) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->gmin) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->gmax) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->bmin) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->bmax) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_blobfinder_color_config_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_blobfinder_imager_config_t(XDR* xdrs, player_blobfinder_imager_config_t* msg)
{
  if(xdr_int(xdrs,&msg->brightness) != 1)
    return(0);
  if(xdr_int(xdrs,&msg->contrast) != 1)
    return(0);
  if(xdr_int(xdrs,&msg->colormode) != 1)
    return(0);
  if(xdr_int(xdrs,&msg->autogain) != 1)
    return(0);
  return(1);
}

int
player_blobfinder_imager_config_pack(void* buf, size_t buflen, player_blobfinder_imager_config_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_int(&xdrs,&msg->brightness) != 1)
    return(-1);
  if(xdr_int(&xdrs,&msg->contrast) != 1)
    return(-1);
  if(xdr_int(&xdrs,&msg->colormode) != 1)
    return(-1);
  if(xdr_int(&xdrs,&msg->autogain) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_blobfinder_imager_config_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_bumper_data_t(XDR* xdrs, player_bumper_data_t* msg)
{
  if(xdr_u_int(xdrs,&msg->bumpers_count) != 1)
    return(0);
  {
    uint8_t* bumpers_p = msg->bumpers;
    if(xdr_bytes(xdrs, (char**)&bumpers_p, &msg->bumpers_count, PLAYER_BUMPER_MAX_SAMPLES) != 1)
      return(0);
  }
  return(1);
}

int
player_bumper_data_pack(void* buf, size_t buflen, player_bumper_data_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->bumpers_count) != 1)
    return(-1);
  {
    uint8_t* bumpers_p = msg->bumpers;
    if(xdr_bytes(&xdrs, (char**)&bumpers_p, &msg->bumpers_count, PLAYER_BUMPER_MAX_SAMPLES) != 1)
      return(-1);
  }
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_bumper_data_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_bumper_define_t(XDR* xdrs, player_bumper_define_t* msg)
{
  if(xdr_player_pose_t(xdrs,&msg->pose) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->length) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->radius) != 1)
    return(0);
  return(1);
}

int
player_bumper_define_pack(void* buf, size_t buflen, player_bumper_define_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_player_pose_t(&xdrs,&msg->pose) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->length) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->radius) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_bumper_define_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_bumper_geom_t(XDR* xdrs, player_bumper_geom_t* msg)
{
  if(xdr_u_int(xdrs,&msg->bumper_def_count) != 1)
    return(0);
  {
    player_bumper_define_t* bumper_def_p = msg->bumper_def;
    if(xdr_array(xdrs, (char**)&bumper_def_p, &msg->bumper_def_count, PLAYER_BUMPER_MAX_SAMPLES, sizeof(player_bumper_define_t), (xdrproc_t)xdr_player_bumper_define_t) != 1)
      return(0);
  }
  return(1);
}

int
player_bumper_geom_pack(void* buf, size_t buflen, player_bumper_geom_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->bumper_def_count) != 1)
    return(-1);
  {
    player_bumper_define_t* bumper_def_p = msg->bumper_def;
    if(xdr_array(&xdrs, (char**)&bumper_def_p, &msg->bumper_def_count, PLAYER_BUMPER_MAX_SAMPLES, sizeof(player_bumper_define_t), (xdrproc_t)xdr_player_bumper_define_t) != 1)
      return(-1);
  }
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_bumper_geom_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_camera_data_t(XDR* xdrs, player_camera_data_t* msg)
{
  if(xdr_u_int(xdrs,&msg->width) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->height) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->bpp) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->format) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->fdiv) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->compression) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->image_count) != 1)
    return(0);
  {
    uint8_t* image_p = msg->image;
    if(xdr_bytes(xdrs, (char**)&image_p, &msg->image_count, PLAYER_CAMERA_IMAGE_SIZE) != 1)
      return(0);
  }
  return(1);
}

int
player_camera_data_pack(void* buf, size_t buflen, player_camera_data_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->width) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->height) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->bpp) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->format) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->fdiv) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->compression) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->image_count) != 1)
    return(-1);
  {
    uint8_t* image_p = msg->image;
    if(xdr_bytes(&xdrs, (char**)&image_p, &msg->image_count, PLAYER_CAMERA_IMAGE_SIZE) != 1)
      return(-1);
  }
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_camera_data_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_dio_data_t(XDR* xdrs, player_dio_data_t* msg)
{
  if(xdr_u_int(xdrs,&msg->count) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->digin) != 1)
    return(0);
  return(1);
}

int
player_dio_data_pack(void* buf, size_t buflen, player_dio_data_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->count) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->digin) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_dio_data_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_dio_cmd_t(XDR* xdrs, player_dio_cmd_t* msg)
{
  if(xdr_u_int(xdrs,&msg->count) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->digout) != 1)
    return(0);
  return(1);
}

int
player_dio_cmd_pack(void* buf, size_t buflen, player_dio_cmd_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->count) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->digout) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_dio_cmd_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_energy_data_t(XDR* xdrs, player_energy_data_t* msg)
{
  if(xdr_float(xdrs,&msg->joules) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->watts) != 1)
    return(0);
  if(xdr_int(xdrs,&msg->charging) != 1)
    return(0);
  return(1);
}

int
player_energy_data_pack(void* buf, size_t buflen, player_energy_data_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_float(&xdrs,&msg->joules) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->watts) != 1)
    return(-1);
  if(xdr_int(&xdrs,&msg->charging) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_energy_data_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_energy_chargepolicy_config_t(XDR* xdrs, player_energy_chargepolicy_config_t* msg)
{
  if(xdr_u_char(xdrs,&msg->enable_input) != 1)
    return(0);
  if(xdr_u_char(xdrs,&msg->enable_output) != 1)
    return(0);
  return(1);
}

int
player_energy_chargepolicy_config_pack(void* buf, size_t buflen, player_energy_chargepolicy_config_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_char(&xdrs,&msg->enable_input) != 1)
    return(-1);
  if(xdr_u_char(&xdrs,&msg->enable_output) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_energy_chargepolicy_config_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_fiducial_item_t(XDR* xdrs, player_fiducial_item_t* msg)
{
  if(xdr_int(xdrs,&msg->id) != 1)
    return(0);
  if(xdr_player_pose3d_t(xdrs,&msg->pose) != 1)
    return(0);
  if(xdr_player_pose3d_t(xdrs,&msg->upose) != 1)
    return(0);
  return(1);
}

int
player_fiducial_item_pack(void* buf, size_t buflen, player_fiducial_item_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_int(&xdrs,&msg->id) != 1)
    return(-1);
  if(xdr_player_pose3d_t(&xdrs,&msg->pose) != 1)
    return(-1);
  if(xdr_player_pose3d_t(&xdrs,&msg->upose) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_fiducial_item_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_fiducial_data_t(XDR* xdrs, player_fiducial_data_t* msg)
{
  if(xdr_u_int(xdrs,&msg->fiducials_count) != 1)
    return(0);
  {
    player_fiducial_item_t* fiducials_p = msg->fiducials;
    if(xdr_array(xdrs, (char**)&fiducials_p, &msg->fiducials_count, PLAYER_FIDUCIAL_MAX_SAMPLES, sizeof(player_fiducial_item_t), (xdrproc_t)xdr_player_fiducial_item_t) != 1)
      return(0);
  }
  return(1);
}

int
player_fiducial_data_pack(void* buf, size_t buflen, player_fiducial_data_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->fiducials_count) != 1)
    return(-1);
  {
    player_fiducial_item_t* fiducials_p = msg->fiducials;
    if(xdr_array(&xdrs, (char**)&fiducials_p, &msg->fiducials_count, PLAYER_FIDUCIAL_MAX_SAMPLES, sizeof(player_fiducial_item_t), (xdrproc_t)xdr_player_fiducial_item_t) != 1)
      return(-1);
  }
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_fiducial_data_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_fiducial_geom_t(XDR* xdrs, player_fiducial_geom_t* msg)
{
  if(xdr_player_pose_t(xdrs,&msg->pose) != 1)
    return(0);
  if(xdr_player_bbox_t(xdrs,&msg->size) != 1)
    return(0);
  if(xdr_player_bbox_t(xdrs,&msg->fiducial_size) != 1)
    return(0);
  return(1);
}

int
player_fiducial_geom_pack(void* buf, size_t buflen, player_fiducial_geom_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_player_pose_t(&xdrs,&msg->pose) != 1)
    return(-1);
  if(xdr_player_bbox_t(&xdrs,&msg->size) != 1)
    return(-1);
  if(xdr_player_bbox_t(&xdrs,&msg->fiducial_size) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_fiducial_geom_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_fiducial_fov_t(XDR* xdrs, player_fiducial_fov_t* msg)
{
  if(xdr_float(xdrs,&msg->min_range) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->max_range) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->view_angle) != 1)
    return(0);
  return(1);
}

int
player_fiducial_fov_pack(void* buf, size_t buflen, player_fiducial_fov_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_float(&xdrs,&msg->min_range) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->max_range) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->view_angle) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_fiducial_fov_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_fiducial_id_t(XDR* xdrs, player_fiducial_id_t* msg)
{
  if(xdr_u_int(xdrs,&msg->id) != 1)
    return(0);
  return(1);
}

int
player_fiducial_id_pack(void* buf, size_t buflen, player_fiducial_id_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->id) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_fiducial_id_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_gps_data_t(XDR* xdrs, player_gps_data_t* msg)
{
  if(xdr_u_int(xdrs,&msg->time_sec) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->time_usec) != 1)
    return(0);
  if(xdr_int(xdrs,&msg->latitude) != 1)
    return(0);
  if(xdr_int(xdrs,&msg->longitude) != 1)
    return(0);
  if(xdr_int(xdrs,&msg->altitude) != 1)
    return(0);
  if(xdr_double(xdrs,&msg->utm_e) != 1)
    return(0);
  if(xdr_double(xdrs,&msg->utm_n) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->quality) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->num_sats) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->hdop) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->vdop) != 1)
    return(0);
  if(xdr_double(xdrs,&msg->err_horz) != 1)
    return(0);
  if(xdr_double(xdrs,&msg->err_vert) != 1)
    return(0);
  return(1);
}

int
player_gps_data_pack(void* buf, size_t buflen, player_gps_data_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->time_sec) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->time_usec) != 1)
    return(-1);
  if(xdr_int(&xdrs,&msg->latitude) != 1)
    return(-1);
  if(xdr_int(&xdrs,&msg->longitude) != 1)
    return(-1);
  if(xdr_int(&xdrs,&msg->altitude) != 1)
    return(-1);
  if(xdr_double(&xdrs,&msg->utm_e) != 1)
    return(-1);
  if(xdr_double(&xdrs,&msg->utm_n) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->quality) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->num_sats) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->hdop) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->vdop) != 1)
    return(-1);
  if(xdr_double(&xdrs,&msg->err_horz) != 1)
    return(-1);
  if(xdr_double(&xdrs,&msg->err_vert) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_gps_data_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_graphics2d_cmd_points_t(XDR* xdrs, player_graphics2d_cmd_points_t* msg)
{
  if(xdr_u_short(xdrs,&msg->count) != 1)
    return(0);
  if(xdr_vector(xdrs, (char*)&msg->points, PLAYER_GRAPHICS2D_MAX_POINTS, sizeof(player_point_2d_t), (xdrproc_t)xdr_player_point_2d_t) != 1)
    return(0);
  if(xdr_player_color_t(xdrs,&msg->color) != 1)
    return(0);
  return(1);
}

int
player_graphics2d_cmd_points_pack(void* buf, size_t buflen, player_graphics2d_cmd_points_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_short(&xdrs,&msg->count) != 1)
    return(-1);
  if(xdr_vector(&xdrs, (char*)&msg->points, PLAYER_GRAPHICS2D_MAX_POINTS, sizeof(player_point_2d_t), (xdrproc_t)xdr_player_point_2d_t) != 1)
    return(-1);
  if(xdr_player_color_t(&xdrs,&msg->color) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_graphics2d_cmd_points_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_graphics2d_cmd_polyline_t(XDR* xdrs, player_graphics2d_cmd_polyline_t* msg)
{
  if(xdr_u_short(xdrs,&msg->count) != 1)
    return(0);
  if(xdr_vector(xdrs, (char*)&msg->points, PLAYER_GRAPHICS2D_MAX_POINTS, sizeof(player_point_2d_t), (xdrproc_t)xdr_player_point_2d_t) != 1)
    return(0);
  if(xdr_player_color_t(xdrs,&msg->color) != 1)
    return(0);
  return(1);
}

int
player_graphics2d_cmd_polyline_pack(void* buf, size_t buflen, player_graphics2d_cmd_polyline_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_short(&xdrs,&msg->count) != 1)
    return(-1);
  if(xdr_vector(&xdrs, (char*)&msg->points, PLAYER_GRAPHICS2D_MAX_POINTS, sizeof(player_point_2d_t), (xdrproc_t)xdr_player_point_2d_t) != 1)
    return(-1);
  if(xdr_player_color_t(&xdrs,&msg->color) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_graphics2d_cmd_polyline_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_graphics2d_cmd_polygon_t(XDR* xdrs, player_graphics2d_cmd_polygon_t* msg)
{
  if(xdr_u_short(xdrs,&msg->count) != 1)
    return(0);
  if(xdr_vector(xdrs, (char*)&msg->points, PLAYER_GRAPHICS2D_MAX_POINTS, sizeof(player_point_2d_t), (xdrproc_t)xdr_player_point_2d_t) != 1)
    return(0);
  if(xdr_player_color_t(xdrs,&msg->color) != 1)
    return(0);
  if(xdr_player_color_t(xdrs,&msg->fill_color) != 1)
    return(0);
  if(xdr_u_char(xdrs,&msg->filled) != 1)
    return(0);
  return(1);
}

int
player_graphics2d_cmd_polygon_pack(void* buf, size_t buflen, player_graphics2d_cmd_polygon_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_short(&xdrs,&msg->count) != 1)
    return(-1);
  if(xdr_vector(&xdrs, (char*)&msg->points, PLAYER_GRAPHICS2D_MAX_POINTS, sizeof(player_point_2d_t), (xdrproc_t)xdr_player_point_2d_t) != 1)
    return(-1);
  if(xdr_player_color_t(&xdrs,&msg->color) != 1)
    return(-1);
  if(xdr_player_color_t(&xdrs,&msg->fill_color) != 1)
    return(-1);
  if(xdr_u_char(&xdrs,&msg->filled) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_graphics2d_cmd_polygon_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_graphics3d_cmd_draw_t(XDR* xdrs, player_graphics3d_cmd_draw_t* msg)
{
  if(xdr_u_int(xdrs,&msg->draw_mode) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->points_count) != 1)
    return(0);
  {
    player_point_3d_t* points_p = msg->points;
    if(xdr_array(xdrs, (char**)&points_p, &msg->points_count, PLAYER_GRAPHICS3D_MAX_POINTS, sizeof(player_point_3d_t), (xdrproc_t)xdr_player_point_3d_t) != 1)
      return(0);
  }
  if(xdr_player_color_t(xdrs,&msg->color) != 1)
    return(0);
  return(1);
}

int
player_graphics3d_cmd_draw_pack(void* buf, size_t buflen, player_graphics3d_cmd_draw_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->draw_mode) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->points_count) != 1)
    return(-1);
  {
    player_point_3d_t* points_p = msg->points;
    if(xdr_array(&xdrs, (char**)&points_p, &msg->points_count, PLAYER_GRAPHICS3D_MAX_POINTS, sizeof(player_point_3d_t), (xdrproc_t)xdr_player_point_3d_t) != 1)
      return(-1);
  }
  if(xdr_player_color_t(&xdrs,&msg->color) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_graphics3d_cmd_draw_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_gripper_data_t(XDR* xdrs, player_gripper_data_t* msg)
{
  if(xdr_u_int(xdrs,&msg->state) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->beams) != 1)
    return(0);
  return(1);
}

int
player_gripper_data_pack(void* buf, size_t buflen, player_gripper_data_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->state) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->beams) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_gripper_data_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_gripper_cmd_t(XDR* xdrs, player_gripper_cmd_t* msg)
{
  if(xdr_u_int(xdrs,&msg->cmd) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->arg) != 1)
    return(0);
  return(1);
}

int
player_gripper_cmd_pack(void* buf, size_t buflen, player_gripper_cmd_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->cmd) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->arg) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_gripper_cmd_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_gripper_geom_t(XDR* xdrs, player_gripper_geom_t* msg)
{
  if(xdr_player_pose_t(xdrs,&msg->pose) != 1)
    return(0);
  if(xdr_player_bbox_t(xdrs,&msg->size) != 1)
    return(0);
  return(1);
}

int
player_gripper_geom_pack(void* buf, size_t buflen, player_gripper_geom_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_player_pose_t(&xdrs,&msg->pose) != 1)
    return(-1);
  if(xdr_player_bbox_t(&xdrs,&msg->size) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_gripper_geom_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_ir_data_t(XDR* xdrs, player_ir_data_t* msg)
{
  if(xdr_u_int(xdrs,&msg->voltages_count) != 1)
    return(0);
  {
    float* voltages_p = msg->voltages;
    if(xdr_array(xdrs, (char**)&voltages_p, &msg->voltages_count, PLAYER_IR_MAX_SAMPLES, sizeof(float), (xdrproc_t)xdr_float) != 1)
      return(0);
  }
  if(xdr_u_int(xdrs,&msg->ranges_count) != 1)
    return(0);
  {
    float* ranges_p = msg->ranges;
    if(xdr_array(xdrs, (char**)&ranges_p, &msg->ranges_count, PLAYER_IR_MAX_SAMPLES, sizeof(float), (xdrproc_t)xdr_float) != 1)
      return(0);
  }
  return(1);
}

int
player_ir_data_pack(void* buf, size_t buflen, player_ir_data_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->voltages_count) != 1)
    return(-1);
  {
    float* voltages_p = msg->voltages;
    if(xdr_array(&xdrs, (char**)&voltages_p, &msg->voltages_count, PLAYER_IR_MAX_SAMPLES, sizeof(float), (xdrproc_t)xdr_float) != 1)
      return(-1);
  }
  if(xdr_u_int(&xdrs,&msg->ranges_count) != 1)
    return(-1);
  {
    float* ranges_p = msg->ranges;
    if(xdr_array(&xdrs, (char**)&ranges_p, &msg->ranges_count, PLAYER_IR_MAX_SAMPLES, sizeof(float), (xdrproc_t)xdr_float) != 1)
      return(-1);
  }
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_ir_data_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_ir_pose_t(XDR* xdrs, player_ir_pose_t* msg)
{
  if(xdr_u_int(xdrs,&msg->poses_count) != 1)
    return(0);
  {
    player_pose_t* poses_p = msg->poses;
    if(xdr_array(xdrs, (char**)&poses_p, &msg->poses_count, PLAYER_IR_MAX_SAMPLES, sizeof(player_pose_t), (xdrproc_t)xdr_player_pose_t) != 1)
      return(0);
  }
  return(1);
}

int
player_ir_pose_pack(void* buf, size_t buflen, player_ir_pose_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->poses_count) != 1)
    return(-1);
  {
    player_pose_t* poses_p = msg->poses;
    if(xdr_array(&xdrs, (char**)&poses_p, &msg->poses_count, PLAYER_IR_MAX_SAMPLES, sizeof(player_pose_t), (xdrproc_t)xdr_player_pose_t) != 1)
      return(-1);
  }
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_ir_pose_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_ir_power_req_t(XDR* xdrs, player_ir_power_req_t* msg)
{
  if(xdr_u_char(xdrs,&msg->state) != 1)
    return(0);
  return(1);
}

int
player_ir_power_req_pack(void* buf, size_t buflen, player_ir_power_req_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_char(&xdrs,&msg->state) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_ir_power_req_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_joystick_data_t(XDR* xdrs, player_joystick_data_t* msg)
{
  if(xdr_int(xdrs,&msg->xpos) != 1)
    return(0);
  if(xdr_int(xdrs,&msg->ypos) != 1)
    return(0);
  if(xdr_int(xdrs,&msg->xscale) != 1)
    return(0);
  if(xdr_int(xdrs,&msg->yscale) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->buttons) != 1)
    return(0);
  return(1);
}

int
player_joystick_data_pack(void* buf, size_t buflen, player_joystick_data_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_int(&xdrs,&msg->xpos) != 1)
    return(-1);
  if(xdr_int(&xdrs,&msg->ypos) != 1)
    return(-1);
  if(xdr_int(&xdrs,&msg->xscale) != 1)
    return(-1);
  if(xdr_int(&xdrs,&msg->yscale) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->buttons) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_joystick_data_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_laser_data_t(XDR* xdrs, player_laser_data_t* msg)
{
  if(xdr_float(xdrs,&msg->min_angle) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->max_angle) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->resolution) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->max_range) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->ranges_count) != 1)
    return(0);
  {
    float* ranges_p = msg->ranges;
    if(xdr_array(xdrs, (char**)&ranges_p, &msg->ranges_count, PLAYER_LASER_MAX_SAMPLES, sizeof(float), (xdrproc_t)xdr_float) != 1)
      return(0);
  }
  if(xdr_u_int(xdrs,&msg->intensity_count) != 1)
    return(0);
  {
    uint8_t* intensity_p = msg->intensity;
    if(xdr_bytes(xdrs, (char**)&intensity_p, &msg->intensity_count, PLAYER_LASER_MAX_SAMPLES) != 1)
      return(0);
  }
  if(xdr_u_int(xdrs,&msg->id) != 1)
    return(0);
  return(1);
}

int
player_laser_data_pack(void* buf, size_t buflen, player_laser_data_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_float(&xdrs,&msg->min_angle) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->max_angle) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->resolution) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->max_range) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->ranges_count) != 1)
    return(-1);
  {
    float* ranges_p = msg->ranges;
    if(xdr_array(&xdrs, (char**)&ranges_p, &msg->ranges_count, PLAYER_LASER_MAX_SAMPLES, sizeof(float), (xdrproc_t)xdr_float) != 1)
      return(-1);
  }
  if(xdr_u_int(&xdrs,&msg->intensity_count) != 1)
    return(-1);
  {
    uint8_t* intensity_p = msg->intensity;
    if(xdr_bytes(&xdrs, (char**)&intensity_p, &msg->intensity_count, PLAYER_LASER_MAX_SAMPLES) != 1)
      return(-1);
  }
  if(xdr_u_int(&xdrs,&msg->id) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_laser_data_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_laser_data_scanpose_t(XDR* xdrs, player_laser_data_scanpose_t* msg)
{
  if(xdr_player_laser_data_t(xdrs,&msg->scan) != 1)
    return(0);
  if(xdr_player_pose_t(xdrs,&msg->pose) != 1)
    return(0);
  return(1);
}

int
player_laser_data_scanpose_pack(void* buf, size_t buflen, player_laser_data_scanpose_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_player_laser_data_t(&xdrs,&msg->scan) != 1)
    return(-1);
  if(xdr_player_pose_t(&xdrs,&msg->pose) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_laser_data_scanpose_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_laser_geom_t(XDR* xdrs, player_laser_geom_t* msg)
{
  if(xdr_player_pose_t(xdrs,&msg->pose) != 1)
    return(0);
  if(xdr_player_bbox_t(xdrs,&msg->size) != 1)
    return(0);
  return(1);
}

int
player_laser_geom_pack(void* buf, size_t buflen, player_laser_geom_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_player_pose_t(&xdrs,&msg->pose) != 1)
    return(-1);
  if(xdr_player_bbox_t(&xdrs,&msg->size) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_laser_geom_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_laser_config_t(XDR* xdrs, player_laser_config_t* msg)
{
  if(xdr_float(xdrs,&msg->min_angle) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->max_angle) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->resolution) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->max_range) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->range_res) != 1)
    return(0);
  if(xdr_u_char(xdrs,&msg->intensity) != 1)
    return(0);
  return(1);
}

int
player_laser_config_pack(void* buf, size_t buflen, player_laser_config_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_float(&xdrs,&msg->min_angle) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->max_angle) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->resolution) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->max_range) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->range_res) != 1)
    return(-1);
  if(xdr_u_char(&xdrs,&msg->intensity) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_laser_config_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_laser_power_config_t(XDR* xdrs, player_laser_power_config_t* msg)
{
  if(xdr_u_char(xdrs,&msg->state) != 1)
    return(0);
  return(1);
}

int
player_laser_power_config_pack(void* buf, size_t buflen, player_laser_power_config_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_char(&xdrs,&msg->state) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_laser_power_config_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_limb_data_t(XDR* xdrs, player_limb_data_t* msg)
{
  if(xdr_player_point_3d_t(xdrs,&msg->position) != 1)
    return(0);
  if(xdr_player_point_3d_t(xdrs,&msg->approach) != 1)
    return(0);
  if(xdr_player_point_3d_t(xdrs,&msg->orientation) != 1)
    return(0);
  if(xdr_u_char(xdrs,&msg->state) != 1)
    return(0);
  return(1);
}

int
player_limb_data_pack(void* buf, size_t buflen, player_limb_data_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_player_point_3d_t(&xdrs,&msg->position) != 1)
    return(-1);
  if(xdr_player_point_3d_t(&xdrs,&msg->approach) != 1)
    return(-1);
  if(xdr_player_point_3d_t(&xdrs,&msg->orientation) != 1)
    return(-1);
  if(xdr_u_char(&xdrs,&msg->state) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_limb_data_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_limb_home_cmd_t(XDR* xdrs, player_limb_home_cmd_t* msg)
{
  return(1);
}

int
player_limb_home_cmd_pack(void* buf, size_t buflen, player_limb_home_cmd_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_limb_home_cmd_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_limb_stop_cmd_t(XDR* xdrs, player_limb_stop_cmd_t* msg)
{
  return(1);
}

int
player_limb_stop_cmd_pack(void* buf, size_t buflen, player_limb_stop_cmd_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_limb_stop_cmd_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_limb_setpose_cmd_t(XDR* xdrs, player_limb_setpose_cmd_t* msg)
{
  if(xdr_player_point_3d_t(xdrs,&msg->position) != 1)
    return(0);
  if(xdr_player_point_3d_t(xdrs,&msg->approach) != 1)
    return(0);
  if(xdr_player_point_3d_t(xdrs,&msg->orientation) != 1)
    return(0);
  return(1);
}

int
player_limb_setpose_cmd_pack(void* buf, size_t buflen, player_limb_setpose_cmd_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_player_point_3d_t(&xdrs,&msg->position) != 1)
    return(-1);
  if(xdr_player_point_3d_t(&xdrs,&msg->approach) != 1)
    return(-1);
  if(xdr_player_point_3d_t(&xdrs,&msg->orientation) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_limb_setpose_cmd_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_limb_setposition_cmd_t(XDR* xdrs, player_limb_setposition_cmd_t* msg)
{
  if(xdr_player_point_3d_t(xdrs,&msg->position) != 1)
    return(0);
  return(1);
}

int
player_limb_setposition_cmd_pack(void* buf, size_t buflen, player_limb_setposition_cmd_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_player_point_3d_t(&xdrs,&msg->position) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_limb_setposition_cmd_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_limb_vecmove_cmd_t(XDR* xdrs, player_limb_vecmove_cmd_t* msg)
{
  if(xdr_player_point_3d_t(xdrs,&msg->direction) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->length) != 1)
    return(0);
  return(1);
}

int
player_limb_vecmove_cmd_pack(void* buf, size_t buflen, player_limb_vecmove_cmd_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_player_point_3d_t(&xdrs,&msg->direction) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->length) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_limb_vecmove_cmd_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_limb_power_req_t(XDR* xdrs, player_limb_power_req_t* msg)
{
  if(xdr_u_char(xdrs,&msg->value) != 1)
    return(0);
  return(1);
}

int
player_limb_power_req_pack(void* buf, size_t buflen, player_limb_power_req_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_char(&xdrs,&msg->value) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_limb_power_req_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_limb_brakes_req_t(XDR* xdrs, player_limb_brakes_req_t* msg)
{
  if(xdr_u_char(xdrs,&msg->value) != 1)
    return(0);
  return(1);
}

int
player_limb_brakes_req_pack(void* buf, size_t buflen, player_limb_brakes_req_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_char(&xdrs,&msg->value) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_limb_brakes_req_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_limb_geom_req_t(XDR* xdrs, player_limb_geom_req_t* msg)
{
  if(xdr_player_point_3d_t(xdrs,&msg->basePos) != 1)
    return(0);
  return(1);
}

int
player_limb_geom_req_pack(void* buf, size_t buflen, player_limb_geom_req_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_player_point_3d_t(&xdrs,&msg->basePos) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_limb_geom_req_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_limb_speed_req_t(XDR* xdrs, player_limb_speed_req_t* msg)
{
  if(xdr_float(xdrs,&msg->speed) != 1)
    return(0);
  return(1);
}

int
player_limb_speed_req_pack(void* buf, size_t buflen, player_limb_speed_req_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_float(&xdrs,&msg->speed) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_limb_speed_req_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_localize_hypoth_t(XDR* xdrs, player_localize_hypoth_t* msg)
{
  if(xdr_player_pose_t(xdrs,&msg->mean) != 1)
    return(0);
  if(xdr_vector(xdrs, (char*)&msg->cov, 3, sizeof(double), (xdrproc_t)xdr_double) != 1)
    return(0);
  if(xdr_double(xdrs,&msg->alpha) != 1)
    return(0);
  return(1);
}

int
player_localize_hypoth_pack(void* buf, size_t buflen, player_localize_hypoth_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_player_pose_t(&xdrs,&msg->mean) != 1)
    return(-1);
  if(xdr_vector(&xdrs, (char*)&msg->cov, 3, sizeof(double), (xdrproc_t)xdr_double) != 1)
    return(-1);
  if(xdr_double(&xdrs,&msg->alpha) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_localize_hypoth_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_localize_data_t(XDR* xdrs, player_localize_data_t* msg)
{
  if(xdr_u_int(xdrs,&msg->pending_count) != 1)
    return(0);
  if(xdr_double(xdrs,&msg->pending_time) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->hypoths_count) != 1)
    return(0);
  {
    player_localize_hypoth_t* hypoths_p = msg->hypoths;
    if(xdr_array(xdrs, (char**)&hypoths_p, &msg->hypoths_count, PLAYER_LOCALIZE_MAX_HYPOTHS, sizeof(player_localize_hypoth_t), (xdrproc_t)xdr_player_localize_hypoth_t) != 1)
      return(0);
  }
  return(1);
}

int
player_localize_data_pack(void* buf, size_t buflen, player_localize_data_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->pending_count) != 1)
    return(-1);
  if(xdr_double(&xdrs,&msg->pending_time) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->hypoths_count) != 1)
    return(-1);
  {
    player_localize_hypoth_t* hypoths_p = msg->hypoths;
    if(xdr_array(&xdrs, (char**)&hypoths_p, &msg->hypoths_count, PLAYER_LOCALIZE_MAX_HYPOTHS, sizeof(player_localize_hypoth_t), (xdrproc_t)xdr_player_localize_hypoth_t) != 1)
      return(-1);
  }
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_localize_data_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_localize_set_pose_t(XDR* xdrs, player_localize_set_pose_t* msg)
{
  if(xdr_player_pose_t(xdrs,&msg->mean) != 1)
    return(0);
  if(xdr_vector(xdrs, (char*)&msg->cov, 3, sizeof(double), (xdrproc_t)xdr_double) != 1)
    return(0);
  return(1);
}

int
player_localize_set_pose_pack(void* buf, size_t buflen, player_localize_set_pose_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_player_pose_t(&xdrs,&msg->mean) != 1)
    return(-1);
  if(xdr_vector(&xdrs, (char*)&msg->cov, 3, sizeof(double), (xdrproc_t)xdr_double) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_localize_set_pose_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_localize_particle_t(XDR* xdrs, player_localize_particle_t* msg)
{
  if(xdr_player_pose_t(xdrs,&msg->pose) != 1)
    return(0);
  if(xdr_double(xdrs,&msg->alpha) != 1)
    return(0);
  return(1);
}

int
player_localize_particle_pack(void* buf, size_t buflen, player_localize_particle_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_player_pose_t(&xdrs,&msg->pose) != 1)
    return(-1);
  if(xdr_double(&xdrs,&msg->alpha) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_localize_particle_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_localize_get_particles_t(XDR* xdrs, player_localize_get_particles_t* msg)
{
  if(xdr_player_pose_t(xdrs,&msg->mean) != 1)
    return(0);
  if(xdr_double(xdrs,&msg->variance) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->particles_count) != 1)
    return(0);
  {
    player_localize_particle_t* particles_p = msg->particles;
    if(xdr_array(xdrs, (char**)&particles_p, &msg->particles_count, PLAYER_LOCALIZE_PARTICLES_MAX, sizeof(player_localize_particle_t), (xdrproc_t)xdr_player_localize_particle_t) != 1)
      return(0);
  }
  return(1);
}

int
player_localize_get_particles_pack(void* buf, size_t buflen, player_localize_get_particles_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_player_pose_t(&xdrs,&msg->mean) != 1)
    return(-1);
  if(xdr_double(&xdrs,&msg->variance) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->particles_count) != 1)
    return(-1);
  {
    player_localize_particle_t* particles_p = msg->particles;
    if(xdr_array(&xdrs, (char**)&particles_p, &msg->particles_count, PLAYER_LOCALIZE_PARTICLES_MAX, sizeof(player_localize_particle_t), (xdrproc_t)xdr_player_localize_particle_t) != 1)
      return(-1);
  }
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_localize_get_particles_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_log_set_write_state_t(XDR* xdrs, player_log_set_write_state_t* msg)
{
  if(xdr_u_char(xdrs,&msg->state) != 1)
    return(0);
  return(1);
}

int
player_log_set_write_state_pack(void* buf, size_t buflen, player_log_set_write_state_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_char(&xdrs,&msg->state) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_log_set_write_state_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_log_set_read_state_t(XDR* xdrs, player_log_set_read_state_t* msg)
{
  if(xdr_u_char(xdrs,&msg->state) != 1)
    return(0);
  return(1);
}

int
player_log_set_read_state_pack(void* buf, size_t buflen, player_log_set_read_state_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_char(&xdrs,&msg->state) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_log_set_read_state_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_log_set_read_rewind_t(XDR* xdrs, player_log_set_read_rewind_t* msg)
{
  return(1);
}

int
player_log_set_read_rewind_pack(void* buf, size_t buflen, player_log_set_read_rewind_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_log_set_read_rewind_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_log_get_state_t(XDR* xdrs, player_log_get_state_t* msg)
{
  if(xdr_u_char(xdrs,&msg->type) != 1)
    return(0);
  if(xdr_u_char(xdrs,&msg->state) != 1)
    return(0);
  return(1);
}

int
player_log_get_state_pack(void* buf, size_t buflen, player_log_get_state_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_char(&xdrs,&msg->type) != 1)
    return(-1);
  if(xdr_u_char(&xdrs,&msg->state) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_log_get_state_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_log_set_filename_t(XDR* xdrs, player_log_set_filename_t* msg)
{
  if(xdr_u_int(xdrs,&msg->filename_count) != 1)
    return(0);
  {
    char* filename_p = msg->filename;
    if(xdr_bytes(xdrs, (char**)&filename_p, &msg->filename_count, 256) != 1)
      return(0);
  }
  return(1);
}

int
player_log_set_filename_pack(void* buf, size_t buflen, player_log_set_filename_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->filename_count) != 1)
    return(-1);
  {
    char* filename_p = msg->filename;
    if(xdr_bytes(&xdrs, (char**)&filename_p, &msg->filename_count, 256) != 1)
      return(-1);
  }
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_log_set_filename_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_map_info_t(XDR* xdrs, player_map_info_t* msg)
{
  if(xdr_float(xdrs,&msg->scale) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->width) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->height) != 1)
    return(0);
  if(xdr_player_pose_t(xdrs,&msg->origin) != 1)
    return(0);
  return(1);
}

int
player_map_info_pack(void* buf, size_t buflen, player_map_info_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_float(&xdrs,&msg->scale) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->width) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->height) != 1)
    return(-1);
  if(xdr_player_pose_t(&xdrs,&msg->origin) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_map_info_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_map_data_t(XDR* xdrs, player_map_data_t* msg)
{
  if(xdr_u_int(xdrs,&msg->col) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->row) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->width) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->height) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->data_count) != 1)
    return(0);
  {
    int8_t* data_p = msg->data;
    if(xdr_bytes(xdrs, (char**)&data_p, &msg->data_count, PLAYER_MAP_MAX_TILE_SIZE) != 1)
      return(0);
  }
  return(1);
}

int
player_map_data_pack(void* buf, size_t buflen, player_map_data_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->col) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->row) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->width) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->height) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->data_count) != 1)
    return(-1);
  {
    int8_t* data_p = msg->data;
    if(xdr_bytes(&xdrs, (char**)&data_p, &msg->data_count, PLAYER_MAP_MAX_TILE_SIZE) != 1)
      return(-1);
  }
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_map_data_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_map_data_vector_t(XDR* xdrs, player_map_data_vector_t* msg)
{
  if(xdr_float(xdrs,&msg->minx) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->maxx) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->miny) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->maxy) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->segments_count) != 1)
    return(0);
  {
    player_segment_t* segments_p = msg->segments;
    if(xdr_array(xdrs, (char**)&segments_p, &msg->segments_count, PLAYER_MAP_MAX_SEGMENTS, sizeof(player_segment_t), (xdrproc_t)xdr_player_segment_t) != 1)
      return(0);
  }
  return(1);
}

int
player_map_data_vector_pack(void* buf, size_t buflen, player_map_data_vector_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_float(&xdrs,&msg->minx) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->maxx) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->miny) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->maxy) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->segments_count) != 1)
    return(-1);
  {
    player_segment_t* segments_p = msg->segments;
    if(xdr_array(&xdrs, (char**)&segments_p, &msg->segments_count, PLAYER_MAP_MAX_SEGMENTS, sizeof(player_segment_t), (xdrproc_t)xdr_player_segment_t) != 1)
      return(-1);
  }
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_map_data_vector_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_mcom_data_t(XDR* xdrs, player_mcom_data_t* msg)
{
  if(xdr_char(xdrs,&msg->full) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->data_count) != 1)
    return(0);
  {
    char* data_p = msg->data;
    if(xdr_bytes(xdrs, (char**)&data_p, &msg->data_count, MCOM_DATA_LEN) != 1)
      return(0);
  }
  return(1);
}

int
player_mcom_data_pack(void* buf, size_t buflen, player_mcom_data_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_char(&xdrs,&msg->full) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->data_count) != 1)
    return(-1);
  {
    char* data_p = msg->data;
    if(xdr_bytes(&xdrs, (char**)&data_p, &msg->data_count, MCOM_DATA_LEN) != 1)
      return(-1);
  }
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_mcom_data_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_mcom_config_t(XDR* xdrs, player_mcom_config_t* msg)
{
  if(xdr_u_int(xdrs,&msg->command) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->type) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->channel_count) != 1)
    return(0);
  {
    char* channel_p = msg->channel;
    if(xdr_bytes(xdrs, (char**)&channel_p, &msg->channel_count, MCOM_CHANNEL_LEN) != 1)
      return(0);
  }
  if(xdr_player_mcom_data_t(xdrs,&msg->data) != 1)
    return(0);
  return(1);
}

int
player_mcom_config_pack(void* buf, size_t buflen, player_mcom_config_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->command) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->type) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->channel_count) != 1)
    return(-1);
  {
    char* channel_p = msg->channel;
    if(xdr_bytes(&xdrs, (char**)&channel_p, &msg->channel_count, MCOM_CHANNEL_LEN) != 1)
      return(-1);
  }
  if(xdr_player_mcom_data_t(&xdrs,&msg->data) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_mcom_config_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_mcom_return_t(XDR* xdrs, player_mcom_return_t* msg)
{
  if(xdr_u_int(xdrs,&msg->type) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->channel_count) != 1)
    return(0);
  {
    char* channel_p = msg->channel;
    if(xdr_bytes(xdrs, (char**)&channel_p, &msg->channel_count, MCOM_CHANNEL_LEN) != 1)
      return(0);
  }
  if(xdr_player_mcom_data_t(xdrs,&msg->data) != 1)
    return(0);
  return(1);
}

int
player_mcom_return_pack(void* buf, size_t buflen, player_mcom_return_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->type) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->channel_count) != 1)
    return(-1);
  {
    char* channel_p = msg->channel;
    if(xdr_bytes(&xdrs, (char**)&channel_p, &msg->channel_count, MCOM_CHANNEL_LEN) != 1)
      return(-1);
  }
  if(xdr_player_mcom_data_t(&xdrs,&msg->data) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_mcom_return_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_opaque_data_t(XDR* xdrs, player_opaque_data_t* msg)
{
  if(xdr_u_int(xdrs,&msg->data_count) != 1)
    return(0);
  {
    uint8_t* data_p = msg->data;
    if(xdr_bytes(xdrs, (char**)&data_p, &msg->data_count, PLAYER_OPAQUE_MAX_SIZE) != 1)
      return(0);
  }
  return(1);
}

int
player_opaque_data_pack(void* buf, size_t buflen, player_opaque_data_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->data_count) != 1)
    return(-1);
  {
    uint8_t* data_p = msg->data;
    if(xdr_bytes(&xdrs, (char**)&data_p, &msg->data_count, PLAYER_OPAQUE_MAX_SIZE) != 1)
      return(-1);
  }
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_opaque_data_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_planner_data_t(XDR* xdrs, player_planner_data_t* msg)
{
  if(xdr_u_char(xdrs,&msg->valid) != 1)
    return(0);
  if(xdr_u_char(xdrs,&msg->done) != 1)
    return(0);
  if(xdr_player_pose_t(xdrs,&msg->pos) != 1)
    return(0);
  if(xdr_player_pose_t(xdrs,&msg->goal) != 1)
    return(0);
  if(xdr_player_pose_t(xdrs,&msg->waypoint) != 1)
    return(0);
  if(xdr_int(xdrs,&msg->waypoint_idx) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->waypoints_count) != 1)
    return(0);
  return(1);
}

int
player_planner_data_pack(void* buf, size_t buflen, player_planner_data_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_char(&xdrs,&msg->valid) != 1)
    return(-1);
  if(xdr_u_char(&xdrs,&msg->done) != 1)
    return(-1);
  if(xdr_player_pose_t(&xdrs,&msg->pos) != 1)
    return(-1);
  if(xdr_player_pose_t(&xdrs,&msg->goal) != 1)
    return(-1);
  if(xdr_player_pose_t(&xdrs,&msg->waypoint) != 1)
    return(-1);
  if(xdr_int(&xdrs,&msg->waypoint_idx) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->waypoints_count) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_planner_data_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_planner_cmd_t(XDR* xdrs, player_planner_cmd_t* msg)
{
  if(xdr_player_pose_t(xdrs,&msg->goal) != 1)
    return(0);
  return(1);
}

int
player_planner_cmd_pack(void* buf, size_t buflen, player_planner_cmd_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_player_pose_t(&xdrs,&msg->goal) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_planner_cmd_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_planner_waypoints_req_t(XDR* xdrs, player_planner_waypoints_req_t* msg)
{
  if(xdr_u_int(xdrs,&msg->waypoints_count) != 1)
    return(0);
  {
    player_pose_t* waypoints_p = msg->waypoints;
    if(xdr_array(xdrs, (char**)&waypoints_p, &msg->waypoints_count, PLAYER_PLANNER_MAX_WAYPOINTS, sizeof(player_pose_t), (xdrproc_t)xdr_player_pose_t) != 1)
      return(0);
  }
  return(1);
}

int
player_planner_waypoints_req_pack(void* buf, size_t buflen, player_planner_waypoints_req_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->waypoints_count) != 1)
    return(-1);
  {
    player_pose_t* waypoints_p = msg->waypoints;
    if(xdr_array(&xdrs, (char**)&waypoints_p, &msg->waypoints_count, PLAYER_PLANNER_MAX_WAYPOINTS, sizeof(player_pose_t), (xdrproc_t)xdr_player_pose_t) != 1)
      return(-1);
  }
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_planner_waypoints_req_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_planner_enable_req_t(XDR* xdrs, player_planner_enable_req_t* msg)
{
  if(xdr_u_char(xdrs,&msg->state) != 1)
    return(0);
  return(1);
}

int
player_planner_enable_req_pack(void* buf, size_t buflen, player_planner_enable_req_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_char(&xdrs,&msg->state) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_planner_enable_req_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_device_devlist_t(XDR* xdrs, player_device_devlist_t* msg)
{
  if(xdr_u_int(xdrs,&msg->devices_count) != 1)
    return(0);
  {
    player_devaddr_t* devices_p = msg->devices;
    if(xdr_array(xdrs, (char**)&devices_p, &msg->devices_count, PLAYER_MAX_DEVICES, sizeof(player_devaddr_t), (xdrproc_t)xdr_player_devaddr_t) != 1)
      return(0);
  }
  return(1);
}

int
player_device_devlist_pack(void* buf, size_t buflen, player_device_devlist_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->devices_count) != 1)
    return(-1);
  {
    player_devaddr_t* devices_p = msg->devices;
    if(xdr_array(&xdrs, (char**)&devices_p, &msg->devices_count, PLAYER_MAX_DEVICES, sizeof(player_devaddr_t), (xdrproc_t)xdr_player_devaddr_t) != 1)
      return(-1);
  }
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_device_devlist_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_device_driverinfo_t(XDR* xdrs, player_device_driverinfo_t* msg)
{
  if(xdr_player_devaddr_t(xdrs,&msg->addr) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->driver_name_count) != 1)
    return(0);
  {
    char* driver_name_p = msg->driver_name;
    if(xdr_bytes(xdrs, (char**)&driver_name_p, &msg->driver_name_count, PLAYER_MAX_DRIVER_STRING_LEN) != 1)
      return(0);
  }
  return(1);
}

int
player_device_driverinfo_pack(void* buf, size_t buflen, player_device_driverinfo_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_player_devaddr_t(&xdrs,&msg->addr) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->driver_name_count) != 1)
    return(-1);
  {
    char* driver_name_p = msg->driver_name;
    if(xdr_bytes(&xdrs, (char**)&driver_name_p, &msg->driver_name_count, PLAYER_MAX_DRIVER_STRING_LEN) != 1)
      return(-1);
  }
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_device_driverinfo_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_device_req_t(XDR* xdrs, player_device_req_t* msg)
{
  if(xdr_player_devaddr_t(xdrs,&msg->addr) != 1)
    return(0);
  if(xdr_u_char(xdrs,&msg->access) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->driver_name_count) != 1)
    return(0);
  {
    char* driver_name_p = msg->driver_name;
    if(xdr_bytes(xdrs, (char**)&driver_name_p, &msg->driver_name_count, PLAYER_MAX_DRIVER_STRING_LEN) != 1)
      return(0);
  }
  return(1);
}

int
player_device_req_pack(void* buf, size_t buflen, player_device_req_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_player_devaddr_t(&xdrs,&msg->addr) != 1)
    return(-1);
  if(xdr_u_char(&xdrs,&msg->access) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->driver_name_count) != 1)
    return(-1);
  {
    char* driver_name_p = msg->driver_name;
    if(xdr_bytes(&xdrs, (char**)&driver_name_p, &msg->driver_name_count, PLAYER_MAX_DRIVER_STRING_LEN) != 1)
      return(-1);
  }
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_device_req_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_device_data_req_t(XDR* xdrs, player_device_data_req_t* msg)
{
  return(1);
}

int
player_device_data_req_pack(void* buf, size_t buflen, player_device_data_req_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_device_data_req_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_device_datamode_req_t(XDR* xdrs, player_device_datamode_req_t* msg)
{
  if(xdr_u_char(xdrs,&msg->mode) != 1)
    return(0);
  return(1);
}

int
player_device_datamode_req_pack(void* buf, size_t buflen, player_device_datamode_req_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_char(&xdrs,&msg->mode) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_device_datamode_req_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_device_auth_req_t(XDR* xdrs, player_device_auth_req_t* msg)
{
  if(xdr_u_int(xdrs,&msg->auth_key_count) != 1)
    return(0);
  {
    uint8_t* auth_key_p = msg->auth_key;
    if(xdr_bytes(xdrs, (char**)&auth_key_p, &msg->auth_key_count, PLAYER_KEYLEN) != 1)
      return(0);
  }
  return(1);
}

int
player_device_auth_req_pack(void* buf, size_t buflen, player_device_auth_req_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->auth_key_count) != 1)
    return(-1);
  {
    uint8_t* auth_key_p = msg->auth_key;
    if(xdr_bytes(&xdrs, (char**)&auth_key_p, &msg->auth_key_count, PLAYER_KEYLEN) != 1)
      return(-1);
  }
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_device_auth_req_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_device_nameservice_req_t(XDR* xdrs, player_device_nameservice_req_t* msg)
{
  if(xdr_u_int(xdrs,&msg->name_count) != 1)
    return(0);
  {
    uint8_t* name_p = msg->name;
    if(xdr_bytes(xdrs, (char**)&name_p, &msg->name_count, PLAYER_MAX_DRIVER_STRING_LEN) != 1)
      return(0);
  }
  if(xdr_u_short(xdrs,&msg->port) != 1)
    return(0);
  return(1);
}

int
player_device_nameservice_req_pack(void* buf, size_t buflen, player_device_nameservice_req_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->name_count) != 1)
    return(-1);
  {
    uint8_t* name_p = msg->name;
    if(xdr_bytes(&xdrs, (char**)&name_p, &msg->name_count, PLAYER_MAX_DRIVER_STRING_LEN) != 1)
      return(-1);
  }
  if(xdr_u_short(&xdrs,&msg->port) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_device_nameservice_req_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_add_replace_rule_req_t(XDR* xdrs, player_add_replace_rule_req_t* msg)
{
  if(xdr_int(xdrs,&msg->interf) != 1)
    return(0);
  if(xdr_int(xdrs,&msg->index) != 1)
    return(0);
  if(xdr_int(xdrs,&msg->type) != 1)
    return(0);
  if(xdr_int(xdrs,&msg->subtype) != 1)
    return(0);
  if(xdr_int(xdrs,&msg->replace) != 1)
    return(0);
  return(1);
}

int
player_add_replace_rule_req_pack(void* buf, size_t buflen, player_add_replace_rule_req_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_int(&xdrs,&msg->interf) != 1)
    return(-1);
  if(xdr_int(&xdrs,&msg->index) != 1)
    return(-1);
  if(xdr_int(&xdrs,&msg->type) != 1)
    return(-1);
  if(xdr_int(&xdrs,&msg->subtype) != 1)
    return(-1);
  if(xdr_int(&xdrs,&msg->replace) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_add_replace_rule_req_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_position1d_data_t(XDR* xdrs, player_position1d_data_t* msg)
{
  if(xdr_float(xdrs,&msg->pos) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->vel) != 1)
    return(0);
  if(xdr_u_char(xdrs,&msg->stall) != 1)
    return(0);
  if(xdr_u_char(xdrs,&msg->status) != 1)
    return(0);
  return(1);
}

int
player_position1d_data_pack(void* buf, size_t buflen, player_position1d_data_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_float(&xdrs,&msg->pos) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->vel) != 1)
    return(-1);
  if(xdr_u_char(&xdrs,&msg->stall) != 1)
    return(-1);
  if(xdr_u_char(&xdrs,&msg->status) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_position1d_data_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_position1d_cmd_vel_t(XDR* xdrs, player_position1d_cmd_vel_t* msg)
{
  if(xdr_float(xdrs,&msg->vel) != 1)
    return(0);
  if(xdr_u_char(xdrs,&msg->state) != 1)
    return(0);
  return(1);
}

int
player_position1d_cmd_vel_pack(void* buf, size_t buflen, player_position1d_cmd_vel_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_float(&xdrs,&msg->vel) != 1)
    return(-1);
  if(xdr_u_char(&xdrs,&msg->state) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_position1d_cmd_vel_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_position1d_cmd_pos_t(XDR* xdrs, player_position1d_cmd_pos_t* msg)
{
  if(xdr_float(xdrs,&msg->pos) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->vel) != 1)
    return(0);
  if(xdr_u_char(xdrs,&msg->state) != 1)
    return(0);
  return(1);
}

int
player_position1d_cmd_pos_pack(void* buf, size_t buflen, player_position1d_cmd_pos_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_float(&xdrs,&msg->pos) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->vel) != 1)
    return(-1);
  if(xdr_u_char(&xdrs,&msg->state) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_position1d_cmd_pos_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_position1d_geom_t(XDR* xdrs, player_position1d_geom_t* msg)
{
  if(xdr_player_pose_t(xdrs,&msg->pose) != 1)
    return(0);
  if(xdr_player_bbox_t(xdrs,&msg->size) != 1)
    return(0);
  return(1);
}

int
player_position1d_geom_pack(void* buf, size_t buflen, player_position1d_geom_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_player_pose_t(&xdrs,&msg->pose) != 1)
    return(-1);
  if(xdr_player_bbox_t(&xdrs,&msg->size) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_position1d_geom_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_position1d_power_config_t(XDR* xdrs, player_position1d_power_config_t* msg)
{
  if(xdr_u_char(xdrs,&msg->state) != 1)
    return(0);
  return(1);
}

int
player_position1d_power_config_pack(void* buf, size_t buflen, player_position1d_power_config_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_char(&xdrs,&msg->state) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_position1d_power_config_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_position1d_velocity_mode_config_t(XDR* xdrs, player_position1d_velocity_mode_config_t* msg)
{
  if(xdr_u_int(xdrs,&msg->value) != 1)
    return(0);
  return(1);
}

int
player_position1d_velocity_mode_config_pack(void* buf, size_t buflen, player_position1d_velocity_mode_config_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->value) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_position1d_velocity_mode_config_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_position1d_reset_odom_config_t(XDR* xdrs, player_position1d_reset_odom_config_t* msg)
{
  if(xdr_u_int(xdrs,&msg->value) != 1)
    return(0);
  return(1);
}

int
player_position1d_reset_odom_config_pack(void* buf, size_t buflen, player_position1d_reset_odom_config_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->value) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_position1d_reset_odom_config_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_position1d_position_mode_req_t(XDR* xdrs, player_position1d_position_mode_req_t* msg)
{
  if(xdr_u_int(xdrs,&msg->state) != 1)
    return(0);
  return(1);
}

int
player_position1d_position_mode_pack(void* buf, size_t buflen, player_position1d_position_mode_req_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->state) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_position1d_position_mode_req_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_position1d_set_odom_req_t(XDR* xdrs, player_position1d_set_odom_req_t* msg)
{
  if(xdr_float(xdrs,&msg->pos) != 1)
    return(0);
  return(1);
}

int
player_position1d_set_odom_pack(void* buf, size_t buflen, player_position1d_set_odom_req_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_float(&xdrs,&msg->pos) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_position1d_set_odom_req_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_position1d_speed_pid_req_t(XDR* xdrs, player_position1d_speed_pid_req_t* msg)
{
  if(xdr_float(xdrs,&msg->kp) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->ki) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->kd) != 1)
    return(0);
  return(1);
}

int
player_position1d_speed_pid_pack(void* buf, size_t buflen, player_position1d_speed_pid_req_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_float(&xdrs,&msg->kp) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->ki) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->kd) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_position1d_speed_pid_req_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_position1d_position_pid_req_t(XDR* xdrs, player_position1d_position_pid_req_t* msg)
{
  if(xdr_float(xdrs,&msg->kp) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->ki) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->kd) != 1)
    return(0);
  return(1);
}

int
player_position1d_position_pid_pack(void* buf, size_t buflen, player_position1d_position_pid_req_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_float(&xdrs,&msg->kp) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->ki) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->kd) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_position1d_position_pid_req_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_position1d_speed_prof_req_t(XDR* xdrs, player_position1d_speed_prof_req_t* msg)
{
  if(xdr_float(xdrs,&msg->speed) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->acc) != 1)
    return(0);
  return(1);
}

int
player_position1d_speed_prof_pack(void* buf, size_t buflen, player_position1d_speed_prof_req_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_float(&xdrs,&msg->speed) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->acc) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_position1d_speed_prof_req_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_position2d_data_t(XDR* xdrs, player_position2d_data_t* msg)
{
  if(xdr_player_pose_t(xdrs,&msg->pos) != 1)
    return(0);
  if(xdr_player_pose_t(xdrs,&msg->vel) != 1)
    return(0);
  if(xdr_u_char(xdrs,&msg->stall) != 1)
    return(0);
  return(1);
}

int
player_position2d_data_pack(void* buf, size_t buflen, player_position2d_data_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_player_pose_t(&xdrs,&msg->pos) != 1)
    return(-1);
  if(xdr_player_pose_t(&xdrs,&msg->vel) != 1)
    return(-1);
  if(xdr_u_char(&xdrs,&msg->stall) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_position2d_data_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_position2d_cmd_vel_t(XDR* xdrs, player_position2d_cmd_vel_t* msg)
{
  if(xdr_player_pose_t(xdrs,&msg->vel) != 1)
    return(0);
  if(xdr_u_char(xdrs,&msg->state) != 1)
    return(0);
  return(1);
}

int
player_position2d_cmd_vel_pack(void* buf, size_t buflen, player_position2d_cmd_vel_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_player_pose_t(&xdrs,&msg->vel) != 1)
    return(-1);
  if(xdr_u_char(&xdrs,&msg->state) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_position2d_cmd_vel_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_position2d_cmd_pos_t(XDR* xdrs, player_position2d_cmd_pos_t* msg)
{
  if(xdr_player_pose_t(xdrs,&msg->pos) != 1)
    return(0);
  if(xdr_player_pose_t(xdrs,&msg->vel) != 1)
    return(0);
  if(xdr_u_char(xdrs,&msg->state) != 1)
    return(0);
  return(1);
}

int
player_position2d_cmd_pos_pack(void* buf, size_t buflen, player_position2d_cmd_pos_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_player_pose_t(&xdrs,&msg->pos) != 1)
    return(-1);
  if(xdr_player_pose_t(&xdrs,&msg->vel) != 1)
    return(-1);
  if(xdr_u_char(&xdrs,&msg->state) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_position2d_cmd_pos_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_position2d_cmd_car_t(XDR* xdrs, player_position2d_cmd_car_t* msg)
{
  if(xdr_double(xdrs,&msg->velocity) != 1)
    return(0);
  if(xdr_double(xdrs,&msg->angle) != 1)
    return(0);
  return(1);
}

int
player_position2d_cmd_car_pack(void* buf, size_t buflen, player_position2d_cmd_car_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_double(&xdrs,&msg->velocity) != 1)
    return(-1);
  if(xdr_double(&xdrs,&msg->angle) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_position2d_cmd_car_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_position2d_geom_t(XDR* xdrs, player_position2d_geom_t* msg)
{
  if(xdr_player_pose_t(xdrs,&msg->pose) != 1)
    return(0);
  if(xdr_player_bbox_t(xdrs,&msg->size) != 1)
    return(0);
  return(1);
}

int
player_position2d_geom_pack(void* buf, size_t buflen, player_position2d_geom_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_player_pose_t(&xdrs,&msg->pose) != 1)
    return(-1);
  if(xdr_player_bbox_t(&xdrs,&msg->size) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_position2d_geom_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_position2d_power_config_t(XDR* xdrs, player_position2d_power_config_t* msg)
{
  if(xdr_u_char(xdrs,&msg->state) != 1)
    return(0);
  return(1);
}

int
player_position2d_power_config_pack(void* buf, size_t buflen, player_position2d_power_config_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_char(&xdrs,&msg->state) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_position2d_power_config_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_position2d_velocity_mode_config_t(XDR* xdrs, player_position2d_velocity_mode_config_t* msg)
{
  if(xdr_u_int(xdrs,&msg->value) != 1)
    return(0);
  return(1);
}

int
player_position2d_velocity_mode_config_pack(void* buf, size_t buflen, player_position2d_velocity_mode_config_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->value) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_position2d_velocity_mode_config_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_position2d_reset_odom_config_t(XDR* xdrs, player_position2d_reset_odom_config_t* msg)
{
  return(1);
}

int
player_position2d_reset_odom_config_pack(void* buf, size_t buflen, player_position2d_reset_odom_config_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_position2d_reset_odom_config_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_position2d_position_mode_req_t(XDR* xdrs, player_position2d_position_mode_req_t* msg)
{
  if(xdr_u_int(xdrs,&msg->state) != 1)
    return(0);
  return(1);
}

int
player_position2d_position_mode_req_pack(void* buf, size_t buflen, player_position2d_position_mode_req_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->state) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_position2d_position_mode_req_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_position2d_set_odom_req_t(XDR* xdrs, player_position2d_set_odom_req_t* msg)
{
  if(xdr_player_pose_t(xdrs,&msg->pose) != 1)
    return(0);
  return(1);
}

int
player_position2d_set_odom_req_pack(void* buf, size_t buflen, player_position2d_set_odom_req_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_player_pose_t(&xdrs,&msg->pose) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_position2d_set_odom_req_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_position2d_speed_pid_req_t(XDR* xdrs, player_position2d_speed_pid_req_t* msg)
{
  if(xdr_float(xdrs,&msg->kp) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->ki) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->kd) != 1)
    return(0);
  return(1);
}

int
player_position2d_speed_pid_req_pack(void* buf, size_t buflen, player_position2d_speed_pid_req_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_float(&xdrs,&msg->kp) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->ki) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->kd) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_position2d_speed_pid_req_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_position2d_position_pid_req_t(XDR* xdrs, player_position2d_position_pid_req_t* msg)
{
  if(xdr_float(xdrs,&msg->kp) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->ki) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->kd) != 1)
    return(0);
  return(1);
}

int
player_position2d_position_pid_req_pack(void* buf, size_t buflen, player_position2d_position_pid_req_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_float(&xdrs,&msg->kp) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->ki) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->kd) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_position2d_position_pid_req_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_position2d_speed_prof_req_t(XDR* xdrs, player_position2d_speed_prof_req_t* msg)
{
  if(xdr_float(xdrs,&msg->speed) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->acc) != 1)
    return(0);
  return(1);
}

int
player_position2d_speed_prof_req_pack(void* buf, size_t buflen, player_position2d_speed_prof_req_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_float(&xdrs,&msg->speed) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->acc) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_position2d_speed_prof_req_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_position3d_data_t(XDR* xdrs, player_position3d_data_t* msg)
{
  if(xdr_player_pose3d_t(xdrs,&msg->pos) != 1)
    return(0);
  if(xdr_player_pose3d_t(xdrs,&msg->vel) != 1)
    return(0);
  if(xdr_u_char(xdrs,&msg->stall) != 1)
    return(0);
  return(1);
}

int
player_position3d_data_pack(void* buf, size_t buflen, player_position3d_data_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_player_pose3d_t(&xdrs,&msg->pos) != 1)
    return(-1);
  if(xdr_player_pose3d_t(&xdrs,&msg->vel) != 1)
    return(-1);
  if(xdr_u_char(&xdrs,&msg->stall) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_position3d_data_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_position3d_cmd_pos_t(XDR* xdrs, player_position3d_cmd_pos_t* msg)
{
  if(xdr_player_pose3d_t(xdrs,&msg->pos) != 1)
    return(0);
  if(xdr_player_pose3d_t(xdrs,&msg->vel) != 1)
    return(0);
  if(xdr_u_char(xdrs,&msg->state) != 1)
    return(0);
  return(1);
}

int
player_position3d_cmd_pos_pack(void* buf, size_t buflen, player_position3d_cmd_pos_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_player_pose3d_t(&xdrs,&msg->pos) != 1)
    return(-1);
  if(xdr_player_pose3d_t(&xdrs,&msg->vel) != 1)
    return(-1);
  if(xdr_u_char(&xdrs,&msg->state) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_position3d_cmd_pos_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_position3d_cmd_vel_t(XDR* xdrs, player_position3d_cmd_vel_t* msg)
{
  if(xdr_player_pose3d_t(xdrs,&msg->vel) != 1)
    return(0);
  if(xdr_u_char(xdrs,&msg->state) != 1)
    return(0);
  return(1);
}

int
player_position3d_cmd_vel_pack(void* buf, size_t buflen, player_position3d_cmd_vel_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_player_pose3d_t(&xdrs,&msg->vel) != 1)
    return(-1);
  if(xdr_u_char(&xdrs,&msg->state) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_position3d_cmd_vel_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_position3d_geom_t(XDR* xdrs, player_position3d_geom_t* msg)
{
  if(xdr_player_pose3d_t(xdrs,&msg->pose) != 1)
    return(0);
  if(xdr_player_bbox3d_t(xdrs,&msg->size) != 1)
    return(0);
  return(1);
}

int
player_position3d_geom_pack(void* buf, size_t buflen, player_position3d_geom_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_player_pose3d_t(&xdrs,&msg->pose) != 1)
    return(-1);
  if(xdr_player_bbox3d_t(&xdrs,&msg->size) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_position3d_geom_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_position3d_power_config_t(XDR* xdrs, player_position3d_power_config_t* msg)
{
  if(xdr_u_char(xdrs,&msg->state) != 1)
    return(0);
  return(1);
}

int
player_position3d_power_config_pack(void* buf, size_t buflen, player_position3d_power_config_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_char(&xdrs,&msg->state) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_position3d_power_config_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_position3d_position_mode_req_t(XDR* xdrs, player_position3d_position_mode_req_t* msg)
{
  if(xdr_u_int(xdrs,&msg->value) != 1)
    return(0);
  return(1);
}

int
player_position3d_position_mode_req_pack(void* buf, size_t buflen, player_position3d_position_mode_req_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->value) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_position3d_position_mode_req_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_position3d_velocity_mode_config_t(XDR* xdrs, player_position3d_velocity_mode_config_t* msg)
{
  if(xdr_u_int(xdrs,&msg->value) != 1)
    return(0);
  return(1);
}

int
player_position3d_velocity_mode_config_pack(void* buf, size_t buflen, player_position3d_velocity_mode_config_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->value) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_position3d_velocity_mode_config_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_position3d_set_odom_req_t(XDR* xdrs, player_position3d_set_odom_req_t* msg)
{
  if(xdr_player_pose3d_t(xdrs,&msg->pos) != 1)
    return(0);
  return(1);
}

int
player_position3d_set_odom_req_pack(void* buf, size_t buflen, player_position3d_set_odom_req_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_player_pose3d_t(&xdrs,&msg->pos) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_position3d_set_odom_req_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_position3d_reset_odom_config_t(XDR* xdrs, player_position3d_reset_odom_config_t* msg)
{
  return(1);
}

int
player_position3d_reset_odom_config_pack(void* buf, size_t buflen, player_position3d_reset_odom_config_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_position3d_reset_odom_config_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_position3d_speed_pid_req_t(XDR* xdrs, player_position3d_speed_pid_req_t* msg)
{
  if(xdr_float(xdrs,&msg->kp) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->ki) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->kd) != 1)
    return(0);
  return(1);
}

int
player_position3d_speed_pid_req_pack(void* buf, size_t buflen, player_position3d_speed_pid_req_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_float(&xdrs,&msg->kp) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->ki) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->kd) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_position3d_speed_pid_req_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_position3d_position_pid_req_t(XDR* xdrs, player_position3d_position_pid_req_t* msg)
{
  if(xdr_float(xdrs,&msg->kp) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->ki) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->kd) != 1)
    return(0);
  return(1);
}

int
player_position3d_position_pid_req_pack(void* buf, size_t buflen, player_position3d_position_pid_req_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_float(&xdrs,&msg->kp) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->ki) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->kd) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_position3d_position_pid_req_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_position3d_speed_prof_req_t(XDR* xdrs, player_position3d_speed_prof_req_t* msg)
{
  if(xdr_float(xdrs,&msg->speed) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->acc) != 1)
    return(0);
  return(1);
}

int
player_position3d_speed_prof_req_pack(void* buf, size_t buflen, player_position3d_speed_prof_req_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_float(&xdrs,&msg->speed) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->acc) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_position3d_speed_prof_req_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_power_data_t(XDR* xdrs, player_power_data_t* msg)
{
  if(xdr_u_int(xdrs,&msg->valid) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->volts) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->percent) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->joules) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->watts) != 1)
    return(0);
  if(xdr_int(xdrs,&msg->charging) != 1)
    return(0);
  return(1);
}

int
player_power_data_pack(void* buf, size_t buflen, player_power_data_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->valid) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->volts) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->percent) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->joules) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->watts) != 1)
    return(-1);
  if(xdr_int(&xdrs,&msg->charging) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_power_data_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_power_chargepolicy_config_t(XDR* xdrs, player_power_chargepolicy_config_t* msg)
{
  if(xdr_u_char(xdrs,&msg->enable_input) != 1)
    return(0);
  if(xdr_u_char(xdrs,&msg->enable_output) != 1)
    return(0);
  return(1);
}

int
player_power_chargepolicy_config_pack(void* buf, size_t buflen, player_power_chargepolicy_config_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_char(&xdrs,&msg->enable_input) != 1)
    return(-1);
  if(xdr_u_char(&xdrs,&msg->enable_output) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_power_chargepolicy_config_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_ptz_data_t(XDR* xdrs, player_ptz_data_t* msg)
{
  if(xdr_float(xdrs,&msg->pan) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->tilt) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->zoom) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->panspeed) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->tiltspeed) != 1)
    return(0);
  return(1);
}

int
player_ptz_data_pack(void* buf, size_t buflen, player_ptz_data_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_float(&xdrs,&msg->pan) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->tilt) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->zoom) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->panspeed) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->tiltspeed) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_ptz_data_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_ptz_cmd_t(XDR* xdrs, player_ptz_cmd_t* msg)
{
  if(xdr_float(xdrs,&msg->pan) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->tilt) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->zoom) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->panspeed) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->tiltspeed) != 1)
    return(0);
  return(1);
}

int
player_ptz_cmd_pack(void* buf, size_t buflen, player_ptz_cmd_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_float(&xdrs,&msg->pan) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->tilt) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->zoom) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->panspeed) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->tiltspeed) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_ptz_cmd_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_ptz_geom_t(XDR* xdrs, player_ptz_geom_t* msg)
{
  if(xdr_player_pose3d_t(xdrs,&msg->pos) != 1)
    return(0);
  if(xdr_player_bbox3d_t(xdrs,&msg->size) != 1)
    return(0);
  return(1);
}

int
player_ptz_geom_pack(void* buf, size_t buflen, player_ptz_geom_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_player_pose3d_t(&xdrs,&msg->pos) != 1)
    return(-1);
  if(xdr_player_bbox3d_t(&xdrs,&msg->size) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_ptz_geom_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_ptz_req_generic_t(XDR* xdrs, player_ptz_req_generic_t* msg)
{
  if(xdr_u_int(xdrs,&msg->config_count) != 1)
    return(0);
  {
    uint32_t* config_p = msg->config;
    if(xdr_array(xdrs, (char**)&config_p, &msg->config_count, PLAYER_PTZ_MAX_CONFIG_LEN, sizeof(uint32_t), (xdrproc_t)xdr_u_int) != 1)
      return(0);
  }
  return(1);
}

int
player_ptz_req_generic_pack(void* buf, size_t buflen, player_ptz_req_generic_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->config_count) != 1)
    return(-1);
  {
    uint32_t* config_p = msg->config;
    if(xdr_array(&xdrs, (char**)&config_p, &msg->config_count, PLAYER_PTZ_MAX_CONFIG_LEN, sizeof(uint32_t), (xdrproc_t)xdr_u_int) != 1)
      return(-1);
  }
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_ptz_req_generic_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_ptz_req_control_mode_t(XDR* xdrs, player_ptz_req_control_mode_t* msg)
{
  if(xdr_u_int(xdrs,&msg->mode) != 1)
    return(0);
  return(1);
}

int
player_ptz_req_control_mode_pack(void* buf, size_t buflen, player_ptz_req_control_mode_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->mode) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_ptz_req_control_mode_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_simulation_data_t(XDR* xdrs, player_simulation_data_t* msg)
{
  if(xdr_u_char(xdrs,&msg->data) != 1)
    return(0);
  return(1);
}

int
player_simulation_data_pack(void* buf, size_t buflen, player_simulation_data_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_char(&xdrs,&msg->data) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_simulation_data_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_simulation_cmd_t(XDR* xdrs, player_simulation_cmd_t* msg)
{
  if(xdr_u_char(xdrs,&msg->cmd) != 1)
    return(0);
  return(1);
}

int
player_simulation_cmd_pack(void* buf, size_t buflen, player_simulation_cmd_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_char(&xdrs,&msg->cmd) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_simulation_cmd_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_simulation_pose2d_req_t(XDR* xdrs, player_simulation_pose2d_req_t* msg)
{
  if(xdr_u_int(xdrs,&msg->name_count) != 1)
    return(0);
  {
    char* name_p = msg->name;
    if(xdr_bytes(xdrs, (char**)&name_p, &msg->name_count, PLAYER_SIMULATION_IDENTIFIER_MAXLEN) != 1)
      return(0);
  }
  if(xdr_player_pose_t(xdrs,&msg->pose) != 1)
    return(0);
  return(1);
}

int
player_simulation_pose2d_req_pack(void* buf, size_t buflen, player_simulation_pose2d_req_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->name_count) != 1)
    return(-1);
  {
    char* name_p = msg->name;
    if(xdr_bytes(&xdrs, (char**)&name_p, &msg->name_count, PLAYER_SIMULATION_IDENTIFIER_MAXLEN) != 1)
      return(-1);
  }
  if(xdr_player_pose_t(&xdrs,&msg->pose) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_simulation_pose2d_req_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_simulation_property_int_req_t(XDR* xdrs, player_simulation_property_int_req_t* msg)
{
  if(xdr_u_int(xdrs,&msg->name_count) != 1)
    return(0);
  {
    char* name_p = msg->name;
    if(xdr_bytes(xdrs, (char**)&name_p, &msg->name_count, PLAYER_SIMULATION_IDENTIFIER_MAXLEN) != 1)
      return(0);
  }
  if(xdr_u_int(xdrs,&msg->prop_count) != 1)
    return(0);
  {
    char* prop_p = msg->prop;
    if(xdr_bytes(xdrs, (char**)&prop_p, &msg->prop_count, PLAYER_SIMULATION_IDENTIFIER_MAXLEN) != 1)
      return(0);
  }
  if(xdr_int(xdrs,&msg->value) != 1)
    return(0);
  return(1);
}

int
player_simulation_property_int_req_pack(void* buf, size_t buflen, player_simulation_property_int_req_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->name_count) != 1)
    return(-1);
  {
    char* name_p = msg->name;
    if(xdr_bytes(&xdrs, (char**)&name_p, &msg->name_count, PLAYER_SIMULATION_IDENTIFIER_MAXLEN) != 1)
      return(-1);
  }
  if(xdr_u_int(&xdrs,&msg->prop_count) != 1)
    return(-1);
  {
    char* prop_p = msg->prop;
    if(xdr_bytes(&xdrs, (char**)&prop_p, &msg->prop_count, PLAYER_SIMULATION_IDENTIFIER_MAXLEN) != 1)
      return(-1);
  }
  if(xdr_int(&xdrs,&msg->value) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_simulation_property_int_req_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_simulation_property_float_req_t(XDR* xdrs, player_simulation_property_float_req_t* msg)
{
  if(xdr_u_int(xdrs,&msg->name_count) != 1)
    return(0);
  {
    char* name_p = msg->name;
    if(xdr_bytes(xdrs, (char**)&name_p, &msg->name_count, PLAYER_SIMULATION_IDENTIFIER_MAXLEN) != 1)
      return(0);
  }
  if(xdr_u_int(xdrs,&msg->prop_count) != 1)
    return(0);
  {
    char* prop_p = msg->prop;
    if(xdr_bytes(xdrs, (char**)&prop_p, &msg->prop_count, PLAYER_SIMULATION_IDENTIFIER_MAXLEN) != 1)
      return(0);
  }
  if(xdr_double(xdrs,&msg->value) != 1)
    return(0);
  return(1);
}

int
player_simulation_property_float_req_pack(void* buf, size_t buflen, player_simulation_property_float_req_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->name_count) != 1)
    return(-1);
  {
    char* name_p = msg->name;
    if(xdr_bytes(&xdrs, (char**)&name_p, &msg->name_count, PLAYER_SIMULATION_IDENTIFIER_MAXLEN) != 1)
      return(-1);
  }
  if(xdr_u_int(&xdrs,&msg->prop_count) != 1)
    return(-1);
  {
    char* prop_p = msg->prop;
    if(xdr_bytes(&xdrs, (char**)&prop_p, &msg->prop_count, PLAYER_SIMULATION_IDENTIFIER_MAXLEN) != 1)
      return(-1);
  }
  if(xdr_double(&xdrs,&msg->value) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_simulation_property_float_req_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_simulation_property_string_req_t(XDR* xdrs, player_simulation_property_string_req_t* msg)
{
  if(xdr_u_int(xdrs,&msg->name_count) != 1)
    return(0);
  {
    char* name_p = msg->name;
    if(xdr_bytes(xdrs, (char**)&name_p, &msg->name_count, PLAYER_SIMULATION_IDENTIFIER_MAXLEN) != 1)
      return(0);
  }
  if(xdr_u_int(xdrs,&msg->prop_count) != 1)
    return(0);
  {
    char* prop_p = msg->prop;
    if(xdr_bytes(xdrs, (char**)&prop_p, &msg->prop_count, PLAYER_SIMULATION_IDENTIFIER_MAXLEN) != 1)
      return(0);
  }
  if(xdr_u_int(xdrs,&msg->value_count) != 1)
    return(0);
  {
    char* value_p = msg->value;
    if(xdr_bytes(xdrs, (char**)&value_p, &msg->value_count, PLAYER_SIMULATION_IDENTIFIER_MAXLEN) != 1)
      return(0);
  }
  return(1);
}

int
player_simulation_property_string_req_pack(void* buf, size_t buflen, player_simulation_property_string_req_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->name_count) != 1)
    return(-1);
  {
    char* name_p = msg->name;
    if(xdr_bytes(&xdrs, (char**)&name_p, &msg->name_count, PLAYER_SIMULATION_IDENTIFIER_MAXLEN) != 1)
      return(-1);
  }
  if(xdr_u_int(&xdrs,&msg->prop_count) != 1)
    return(-1);
  {
    char* prop_p = msg->prop;
    if(xdr_bytes(&xdrs, (char**)&prop_p, &msg->prop_count, PLAYER_SIMULATION_IDENTIFIER_MAXLEN) != 1)
      return(-1);
  }
  if(xdr_u_int(&xdrs,&msg->value_count) != 1)
    return(-1);
  {
    char* value_p = msg->value;
    if(xdr_bytes(&xdrs, (char**)&value_p, &msg->value_count, PLAYER_SIMULATION_IDENTIFIER_MAXLEN) != 1)
      return(-1);
  }
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_simulation_property_string_req_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_sonar_data_t(XDR* xdrs, player_sonar_data_t* msg)
{
  if(xdr_u_int(xdrs,&msg->ranges_count) != 1)
    return(0);
  {
    float* ranges_p = msg->ranges;
    if(xdr_array(xdrs, (char**)&ranges_p, &msg->ranges_count, PLAYER_SONAR_MAX_SAMPLES, sizeof(float), (xdrproc_t)xdr_float) != 1)
      return(0);
  }
  return(1);
}

int
player_sonar_data_pack(void* buf, size_t buflen, player_sonar_data_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->ranges_count) != 1)
    return(-1);
  {
    float* ranges_p = msg->ranges;
    if(xdr_array(&xdrs, (char**)&ranges_p, &msg->ranges_count, PLAYER_SONAR_MAX_SAMPLES, sizeof(float), (xdrproc_t)xdr_float) != 1)
      return(-1);
  }
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_sonar_data_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_sonar_geom_t(XDR* xdrs, player_sonar_geom_t* msg)
{
  if(xdr_u_int(xdrs,&msg->poses_count) != 1)
    return(0);
  {
    player_pose_t* poses_p = msg->poses;
    if(xdr_array(xdrs, (char**)&poses_p, &msg->poses_count, PLAYER_SONAR_MAX_SAMPLES, sizeof(player_pose_t), (xdrproc_t)xdr_player_pose_t) != 1)
      return(0);
  }
  return(1);
}

int
player_sonar_geom_pack(void* buf, size_t buflen, player_sonar_geom_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->poses_count) != 1)
    return(-1);
  {
    player_pose_t* poses_p = msg->poses;
    if(xdr_array(&xdrs, (char**)&poses_p, &msg->poses_count, PLAYER_SONAR_MAX_SAMPLES, sizeof(player_pose_t), (xdrproc_t)xdr_player_pose_t) != 1)
      return(-1);
  }
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_sonar_geom_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_sonar_power_config_t(XDR* xdrs, player_sonar_power_config_t* msg)
{
  if(xdr_u_char(xdrs,&msg->state) != 1)
    return(0);
  return(1);
}

int
player_sonar_power_config_pack(void* buf, size_t buflen, player_sonar_power_config_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_char(&xdrs,&msg->state) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_sonar_power_config_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_sound_cmd_t(XDR* xdrs, player_sound_cmd_t* msg)
{
  if(xdr_u_int(xdrs,&msg->index) != 1)
    return(0);
  return(1);
}

int
player_sound_cmd_pack(void* buf, size_t buflen, player_sound_cmd_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->index) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_sound_cmd_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_speech_cmd_t(XDR* xdrs, player_speech_cmd_t* msg)
{
  if(xdr_u_int(xdrs,&msg->string_count) != 1)
    return(0);
  {
    char* string_p = msg->string;
    if(xdr_bytes(xdrs, (char**)&string_p, &msg->string_count, PLAYER_SPEECH_MAX_STRING_LEN) != 1)
      return(0);
  }
  return(1);
}

int
player_speech_cmd_pack(void* buf, size_t buflen, player_speech_cmd_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->string_count) != 1)
    return(-1);
  {
    char* string_p = msg->string;
    if(xdr_bytes(&xdrs, (char**)&string_p, &msg->string_count, PLAYER_SPEECH_MAX_STRING_LEN) != 1)
      return(-1);
  }
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_speech_cmd_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_speech_recognition_data_t(XDR* xdrs, player_speech_recognition_data_t* msg)
{
  if(xdr_u_int(xdrs,&msg->text_count) != 1)
    return(0);
  {
    char* text_p = msg->text;
    if(xdr_bytes(xdrs, (char**)&text_p, &msg->text_count, PLAYER_SPEECH_RECOGNITION_TEXT_LEN) != 1)
      return(0);
  }
  return(1);
}

int
player_speech_recognition_data_pack(void* buf, size_t buflen, player_speech_recognition_data_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->text_count) != 1)
    return(-1);
  {
    char* text_p = msg->text;
    if(xdr_bytes(&xdrs, (char**)&text_p, &msg->text_count, PLAYER_SPEECH_RECOGNITION_TEXT_LEN) != 1)
      return(-1);
  }
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_speech_recognition_data_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_truth_pose_t(XDR* xdrs, player_truth_pose_t* msg)
{
  if(xdr_player_pose3d_t(xdrs,&msg->pose) != 1)
    return(0);
  return(1);
}

int
player_truth_pose_pack(void* buf, size_t buflen, player_truth_pose_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_player_pose3d_t(&xdrs,&msg->pose) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_truth_pose_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_truth_fiducial_id_t(XDR* xdrs, player_truth_fiducial_id_t* msg)
{
  if(xdr_u_char(xdrs,&msg->subtype) != 1)
    return(0);
  if(xdr_short(xdrs,&msg->id) != 1)
    return(0);
  return(1);
}

int
player_truth_fiducial_id_pack(void* buf, size_t buflen, player_truth_fiducial_id_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_char(&xdrs,&msg->subtype) != 1)
    return(-1);
  if(xdr_short(&xdrs,&msg->id) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_truth_fiducial_id_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_waveform_data_t(XDR* xdrs, player_waveform_data_t* msg)
{
  if(xdr_u_int(xdrs,&msg->rate) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->depth) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->data_count) != 1)
    return(0);
  {
    uint8_t* data_p = msg->data;
    if(xdr_bytes(xdrs, (char**)&data_p, &msg->data_count, PLAYER_WAVEFORM_DATA_MAX) != 1)
      return(0);
  }
  return(1);
}

int
player_waveform_data_pack(void* buf, size_t buflen, player_waveform_data_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->rate) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->depth) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->data_count) != 1)
    return(-1);
  {
    uint8_t* data_p = msg->data;
    if(xdr_bytes(&xdrs, (char**)&data_p, &msg->data_count, PLAYER_WAVEFORM_DATA_MAX) != 1)
      return(-1);
  }
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_waveform_data_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_wifi_link_t(XDR* xdrs, player_wifi_link_t* msg)
{
  if(xdr_u_int(xdrs,&msg->mac_count) != 1)
    return(0);
  {
    uint8_t* mac_p = msg->mac;
    if(xdr_bytes(xdrs, (char**)&mac_p, &msg->mac_count, 32) != 1)
      return(0);
  }
  if(xdr_u_int(xdrs,&msg->ip_count) != 1)
    return(0);
  {
    uint8_t* ip_p = msg->ip;
    if(xdr_bytes(xdrs, (char**)&ip_p, &msg->ip_count, 32) != 1)
      return(0);
  }
  if(xdr_u_int(xdrs,&msg->essid_count) != 1)
    return(0);
  {
    uint8_t* essid_p = msg->essid;
    if(xdr_bytes(xdrs, (char**)&essid_p, &msg->essid_count, 32) != 1)
      return(0);
  }
  if(xdr_u_int(xdrs,&msg->mode) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->freq) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->encrypt) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->qual) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->level) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->noise) != 1)
    return(0);
  return(1);
}

int
player_wifi_link_pack(void* buf, size_t buflen, player_wifi_link_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->mac_count) != 1)
    return(-1);
  {
    uint8_t* mac_p = msg->mac;
    if(xdr_bytes(&xdrs, (char**)&mac_p, &msg->mac_count, 32) != 1)
      return(-1);
  }
  if(xdr_u_int(&xdrs,&msg->ip_count) != 1)
    return(-1);
  {
    uint8_t* ip_p = msg->ip;
    if(xdr_bytes(&xdrs, (char**)&ip_p, &msg->ip_count, 32) != 1)
      return(-1);
  }
  if(xdr_u_int(&xdrs,&msg->essid_count) != 1)
    return(-1);
  {
    uint8_t* essid_p = msg->essid;
    if(xdr_bytes(&xdrs, (char**)&essid_p, &msg->essid_count, 32) != 1)
      return(-1);
  }
  if(xdr_u_int(&xdrs,&msg->mode) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->freq) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->encrypt) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->qual) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->level) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->noise) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_wifi_link_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_wifi_data_t(XDR* xdrs, player_wifi_data_t* msg)
{
  if(xdr_u_int(xdrs,&msg->links_count) != 1)
    return(0);
  {
    player_wifi_link_t* links_p = msg->links;
    if(xdr_array(xdrs, (char**)&links_p, &msg->links_count, PLAYER_WIFI_MAX_LINKS, sizeof(player_wifi_link_t), (xdrproc_t)xdr_player_wifi_link_t) != 1)
      return(0);
  }
  if(xdr_u_int(xdrs,&msg->throughput) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->bitrate) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->mode) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->qual_type) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->maxqual) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->maxlevel) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->maxnoise) != 1)
    return(0);
  if(xdr_opaque(xdrs, (char*)&msg->ap, 32) != 1)
    return(0);
  return(1);
}

int
player_wifi_data_pack(void* buf, size_t buflen, player_wifi_data_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->links_count) != 1)
    return(-1);
  {
    player_wifi_link_t* links_p = msg->links;
    if(xdr_array(&xdrs, (char**)&links_p, &msg->links_count, PLAYER_WIFI_MAX_LINKS, sizeof(player_wifi_link_t), (xdrproc_t)xdr_player_wifi_link_t) != 1)
      return(-1);
  }
  if(xdr_u_int(&xdrs,&msg->throughput) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->bitrate) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->mode) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->qual_type) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->maxqual) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->maxlevel) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->maxnoise) != 1)
    return(-1);
  if(xdr_opaque(&xdrs, (char*)&msg->ap, 32) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_wifi_data_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_wifi_mac_req_t(XDR* xdrs, player_wifi_mac_req_t* msg)
{
  if(xdr_u_int(xdrs,&msg->mac_count) != 1)
    return(0);
  {
    uint8_t* mac_p = msg->mac;
    if(xdr_bytes(xdrs, (char**)&mac_p, &msg->mac_count, 32) != 1)
      return(0);
  }
  return(1);
}

int
player_wifi_mac_req_pack(void* buf, size_t buflen, player_wifi_mac_req_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->mac_count) != 1)
    return(-1);
  {
    uint8_t* mac_p = msg->mac;
    if(xdr_bytes(&xdrs, (char**)&mac_p, &msg->mac_count, 32) != 1)
      return(-1);
  }
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_wifi_mac_req_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_wifi_iwspy_addr_req_t(XDR* xdrs, player_wifi_iwspy_addr_req_t* msg)
{
  if(xdr_opaque(xdrs, (char*)&msg->address, 32) != 1)
    return(0);
  return(1);
}

int
player_wifi_iwspy_addr_req_pack(void* buf, size_t buflen, player_wifi_iwspy_addr_req_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_opaque(&xdrs, (char*)&msg->address, 32) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_wifi_iwspy_addr_req_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_rfid_tag_t(XDR* xdrs, player_rfid_tag_t* msg)
{
  if(xdr_u_int(xdrs,&msg->type) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->guid_count) != 1)
    return(0);
  {
    char* guid_p = msg->guid;
    if(xdr_bytes(xdrs, (char**)&guid_p, &msg->guid_count, PLAYER_RFID_MAX_GUID) != 1)
      return(0);
  }
  return(1);
}

int
player_rfid_tag_pack(void* buf, size_t buflen, player_rfid_tag_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->type) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->guid_count) != 1)
    return(-1);
  {
    char* guid_p = msg->guid;
    if(xdr_bytes(&xdrs, (char**)&guid_p, &msg->guid_count, PLAYER_RFID_MAX_GUID) != 1)
      return(-1);
  }
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_rfid_tag_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_rfid_data_t(XDR* xdrs, player_rfid_data_t* msg)
{
  if(xdr_u_int(xdrs,&msg->tags_count) != 1)
    return(0);
  {
    player_rfid_tag_t* tags_p = msg->tags;
    if(xdr_array(xdrs, (char**)&tags_p, &msg->tags_count, PLAYER_RFID_MAX_TAGS, sizeof(player_rfid_tag_t), (xdrproc_t)xdr_player_rfid_tag_t) != 1)
      return(0);
  }
  return(1);
}

int
player_rfid_data_pack(void* buf, size_t buflen, player_rfid_data_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->tags_count) != 1)
    return(-1);
  {
    player_rfid_tag_t* tags_p = msg->tags;
    if(xdr_array(&xdrs, (char**)&tags_p, &msg->tags_count, PLAYER_RFID_MAX_TAGS, sizeof(player_rfid_tag_t), (xdrproc_t)xdr_player_rfid_tag_t) != 1)
      return(-1);
  }
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_rfid_data_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_rfid_cmd_t(XDR* xdrs, player_rfid_cmd_t* msg)
{
  if(xdr_u_char(xdrs,&msg->temp) != 1)
    return(0);
  return(1);
}

int
player_rfid_cmd_pack(void* buf, size_t buflen, player_rfid_cmd_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_char(&xdrs,&msg->temp) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_rfid_cmd_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_wsn_node_data_t(XDR* xdrs, player_wsn_node_data_t* msg)
{
  if(xdr_float(xdrs,&msg->light) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->mic) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->accel_x) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->accel_y) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->accel_z) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->magn_x) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->magn_y) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->magn_z) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->temperature) != 1)
    return(0);
  if(xdr_float(xdrs,&msg->battery) != 1)
    return(0);
  return(1);
}

int
player_wsn_node_data_pack(void* buf, size_t buflen, player_wsn_node_data_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_float(&xdrs,&msg->light) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->mic) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->accel_x) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->accel_y) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->accel_z) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->magn_x) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->magn_y) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->magn_z) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->temperature) != 1)
    return(-1);
  if(xdr_float(&xdrs,&msg->battery) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_wsn_node_data_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_wsn_data_t(XDR* xdrs, player_wsn_data_t* msg)
{
  if(xdr_u_int(xdrs,&msg->node_type) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->node_id) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->node_parent_id) != 1)
    return(0);
  if(xdr_player_wsn_node_data_t(xdrs,&msg->data_packet) != 1)
    return(0);
  return(1);
}

int
player_wsn_data_pack(void* buf, size_t buflen, player_wsn_data_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_int(&xdrs,&msg->node_type) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->node_id) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->node_parent_id) != 1)
    return(-1);
  if(xdr_player_wsn_node_data_t(&xdrs,&msg->data_packet) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_wsn_data_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_wsn_cmd_t(XDR* xdrs, player_wsn_cmd_t* msg)
{
  if(xdr_int(xdrs,&msg->node_id) != 1)
    return(0);
  if(xdr_int(xdrs,&msg->group_id) != 1)
    return(0);
  if(xdr_u_int(xdrs,&msg->device) != 1)
    return(0);
  if(xdr_u_char(xdrs,&msg->enable) != 1)
    return(0);
  return(1);
}

int
player_wsn_cmd_pack(void* buf, size_t buflen, player_wsn_cmd_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_int(&xdrs,&msg->node_id) != 1)
    return(-1);
  if(xdr_int(&xdrs,&msg->group_id) != 1)
    return(-1);
  if(xdr_u_int(&xdrs,&msg->device) != 1)
    return(-1);
  if(xdr_u_char(&xdrs,&msg->enable) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_wsn_cmd_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_wsn_power_config_t(XDR* xdrs, player_wsn_power_config_t* msg)
{
  if(xdr_int(xdrs,&msg->node_id) != 1)
    return(0);
  if(xdr_int(xdrs,&msg->group_id) != 1)
    return(0);
  if(xdr_u_char(xdrs,&msg->value) != 1)
    return(0);
  return(1);
}

int
player_wsn_power_config_pack(void* buf, size_t buflen, player_wsn_power_config_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_int(&xdrs,&msg->node_id) != 1)
    return(-1);
  if(xdr_int(&xdrs,&msg->group_id) != 1)
    return(-1);
  if(xdr_u_char(&xdrs,&msg->value) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_wsn_power_config_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_wsn_datatype_config_t(XDR* xdrs, player_wsn_datatype_config_t* msg)
{
  if(xdr_u_char(xdrs,&msg->value) != 1)
    return(0);
  return(1);
}

int
player_wsn_datatype_config_pack(void* buf, size_t buflen, player_wsn_datatype_config_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_u_char(&xdrs,&msg->value) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_wsn_datatype_config_t);
  xdr_destroy(&xdrs);
  return(len);
}

int
xdr_player_wsn_datafreq_config_t(XDR* xdrs, player_wsn_datafreq_config_t* msg)
{
  if(xdr_int(xdrs,&msg->node_id) != 1)
    return(0);
  if(xdr_int(xdrs,&msg->group_id) != 1)
    return(0);
  if(xdr_double(xdrs,&msg->frequency) != 1)
    return(0);
  return(1);
}

int
player_wsn_datafreq_config_pack(void* buf, size_t buflen, player_wsn_datafreq_config_t* msg, int op)
{
  XDR xdrs;
  int len;
  if(!buflen)
    return(0);
  xdrmem_create(&xdrs, buf, buflen, op);
  if(xdr_int(&xdrs,&msg->node_id) != 1)
    return(-1);
  if(xdr_int(&xdrs,&msg->group_id) != 1)
    return(-1);
  if(xdr_double(&xdrs,&msg->frequency) != 1)
    return(-1);
  if(op == PLAYERXDR_ENCODE)
    len = xdr_getpos(&xdrs);
  else
    len = sizeof(player_wsn_datafreq_config_t);
  xdr_destroy(&xdrs);
  return(len);
}

