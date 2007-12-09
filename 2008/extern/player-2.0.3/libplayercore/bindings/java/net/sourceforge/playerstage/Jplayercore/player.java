package net.sourceforge.playerstage.Jplayercore;
public class player {

  public static Jplayer_devaddr_t buf_to_Jplayer_devaddr_t(SWIGTYPE_p_void buf) {
    player_devaddr_t data = playercore_java.buf_to_player_devaddr_t(buf);
    return(player_devaddr_t_to_Jplayer_devaddr_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_devaddr_t_to_buf(Jplayer_devaddr_t Jdata) {
    player_devaddr_t data = Jplayer_devaddr_t_to_player_devaddr_t(Jdata);
    return(playercore_java.player_devaddr_t_to_buf(data));
  }

  public static Jplayer_devaddr_t player_devaddr_t_to_Jplayer_devaddr_t(player_devaddr_t data) {
    Jplayer_devaddr_t Jdata = new Jplayer_devaddr_t();
    Jdata.host = data.getHost();
    Jdata.robot = data.getRobot();
    Jdata.interf = data.getInterf();
    Jdata.index = data.getIndex();
    return(Jdata);
  }

  public static player_devaddr_t Jplayer_devaddr_t_to_player_devaddr_t(Jplayer_devaddr_t Jdata) {
    player_devaddr_t data = new player_devaddr_t();
    data.setHost(Jdata.host);
    data.setRobot(Jdata.robot);
    data.setInterf(Jdata.interf);
    data.setIndex(Jdata.index);
    return(data);
  }

  public static Jplayer_msghdr_t buf_to_Jplayer_msghdr_t(SWIGTYPE_p_void buf) {
    player_msghdr_t data = playercore_java.buf_to_player_msghdr_t(buf);
    return(player_msghdr_t_to_Jplayer_msghdr_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_msghdr_t_to_buf(Jplayer_msghdr_t Jdata) {
    player_msghdr_t data = Jplayer_msghdr_t_to_player_msghdr_t(Jdata);
    return(playercore_java.player_msghdr_t_to_buf(data));
  }

  public static Jplayer_msghdr_t player_msghdr_t_to_Jplayer_msghdr_t(player_msghdr_t data) {
    Jplayer_msghdr_t Jdata = new Jplayer_msghdr_t();
    Jdata.addr = player_devaddr_t_to_Jplayer_devaddr_t(data.getAddr());
    Jdata.type = data.getType();
    Jdata.subtype = data.getSubtype();
    Jdata.timestamp = data.getTimestamp();
    Jdata.seq = data.getSeq();
    Jdata.size = data.getSize();
    return(Jdata);
  }

  public static player_msghdr_t Jplayer_msghdr_t_to_player_msghdr_t(Jplayer_msghdr_t Jdata) {
    player_msghdr_t data = new player_msghdr_t();
    data.setAddr(Jplayer_devaddr_t_to_player_devaddr_t(Jdata.addr));
    data.setType(Jdata.type);
    data.setSubtype(Jdata.subtype);
    data.setTimestamp(Jdata.timestamp);
    data.setSeq(Jdata.seq);
    data.setSize(Jdata.size);
    return(data);
  }

  public static Jplayer_point_2d_t buf_to_Jplayer_point_2d_t(SWIGTYPE_p_void buf) {
    player_point_2d_t data = playercore_java.buf_to_player_point_2d_t(buf);
    return(player_point_2d_t_to_Jplayer_point_2d_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_point_2d_t_to_buf(Jplayer_point_2d_t Jdata) {
    player_point_2d_t data = Jplayer_point_2d_t_to_player_point_2d_t(Jdata);
    return(playercore_java.player_point_2d_t_to_buf(data));
  }

  public static Jplayer_point_2d_t player_point_2d_t_to_Jplayer_point_2d_t(player_point_2d_t data) {
    Jplayer_point_2d_t Jdata = new Jplayer_point_2d_t();
    Jdata.px = data.getPx();
    Jdata.py = data.getPy();
    return(Jdata);
  }

  public static player_point_2d_t Jplayer_point_2d_t_to_player_point_2d_t(Jplayer_point_2d_t Jdata) {
    player_point_2d_t data = new player_point_2d_t();
    data.setPx(Jdata.px);
    data.setPy(Jdata.py);
    return(data);
  }

  public static Jplayer_point_3d_t buf_to_Jplayer_point_3d_t(SWIGTYPE_p_void buf) {
    player_point_3d_t data = playercore_java.buf_to_player_point_3d_t(buf);
    return(player_point_3d_t_to_Jplayer_point_3d_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_point_3d_t_to_buf(Jplayer_point_3d_t Jdata) {
    player_point_3d_t data = Jplayer_point_3d_t_to_player_point_3d_t(Jdata);
    return(playercore_java.player_point_3d_t_to_buf(data));
  }

  public static Jplayer_point_3d_t player_point_3d_t_to_Jplayer_point_3d_t(player_point_3d_t data) {
    Jplayer_point_3d_t Jdata = new Jplayer_point_3d_t();
    Jdata.px = data.getPx();
    Jdata.py = data.getPy();
    Jdata.pz = data.getPz();
    return(Jdata);
  }

  public static player_point_3d_t Jplayer_point_3d_t_to_player_point_3d_t(Jplayer_point_3d_t Jdata) {
    player_point_3d_t data = new player_point_3d_t();
    data.setPx(Jdata.px);
    data.setPy(Jdata.py);
    data.setPz(Jdata.pz);
    return(data);
  }

  public static Jplayer_pose_t buf_to_Jplayer_pose_t(SWIGTYPE_p_void buf) {
    player_pose_t data = playercore_java.buf_to_player_pose_t(buf);
    return(player_pose_t_to_Jplayer_pose_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_pose_t_to_buf(Jplayer_pose_t Jdata) {
    player_pose_t data = Jplayer_pose_t_to_player_pose_t(Jdata);
    return(playercore_java.player_pose_t_to_buf(data));
  }

  public static Jplayer_pose_t player_pose_t_to_Jplayer_pose_t(player_pose_t data) {
    Jplayer_pose_t Jdata = new Jplayer_pose_t();
    Jdata.px = data.getPx();
    Jdata.py = data.getPy();
    Jdata.pa = data.getPa();
    return(Jdata);
  }

  public static player_pose_t Jplayer_pose_t_to_player_pose_t(Jplayer_pose_t Jdata) {
    player_pose_t data = new player_pose_t();
    data.setPx(Jdata.px);
    data.setPy(Jdata.py);
    data.setPa(Jdata.pa);
    return(data);
  }

  public static Jplayer_pose3d_t buf_to_Jplayer_pose3d_t(SWIGTYPE_p_void buf) {
    player_pose3d_t data = playercore_java.buf_to_player_pose3d_t(buf);
    return(player_pose3d_t_to_Jplayer_pose3d_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_pose3d_t_to_buf(Jplayer_pose3d_t Jdata) {
    player_pose3d_t data = Jplayer_pose3d_t_to_player_pose3d_t(Jdata);
    return(playercore_java.player_pose3d_t_to_buf(data));
  }

  public static Jplayer_pose3d_t player_pose3d_t_to_Jplayer_pose3d_t(player_pose3d_t data) {
    Jplayer_pose3d_t Jdata = new Jplayer_pose3d_t();
    Jdata.px = data.getPx();
    Jdata.py = data.getPy();
    Jdata.pz = data.getPz();
    Jdata.proll = data.getProll();
    Jdata.ppitch = data.getPpitch();
    Jdata.pyaw = data.getPyaw();
    return(Jdata);
  }

  public static player_pose3d_t Jplayer_pose3d_t_to_player_pose3d_t(Jplayer_pose3d_t Jdata) {
    player_pose3d_t data = new player_pose3d_t();
    data.setPx(Jdata.px);
    data.setPy(Jdata.py);
    data.setPz(Jdata.pz);
    data.setProll(Jdata.proll);
    data.setPpitch(Jdata.ppitch);
    data.setPyaw(Jdata.pyaw);
    return(data);
  }

  public static Jplayer_bbox_t buf_to_Jplayer_bbox_t(SWIGTYPE_p_void buf) {
    player_bbox_t data = playercore_java.buf_to_player_bbox_t(buf);
    return(player_bbox_t_to_Jplayer_bbox_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_bbox_t_to_buf(Jplayer_bbox_t Jdata) {
    player_bbox_t data = Jplayer_bbox_t_to_player_bbox_t(Jdata);
    return(playercore_java.player_bbox_t_to_buf(data));
  }

  public static Jplayer_bbox_t player_bbox_t_to_Jplayer_bbox_t(player_bbox_t data) {
    Jplayer_bbox_t Jdata = new Jplayer_bbox_t();
    Jdata.sw = data.getSw();
    Jdata.sl = data.getSl();
    return(Jdata);
  }

  public static player_bbox_t Jplayer_bbox_t_to_player_bbox_t(Jplayer_bbox_t Jdata) {
    player_bbox_t data = new player_bbox_t();
    data.setSw(Jdata.sw);
    data.setSl(Jdata.sl);
    return(data);
  }

  public static Jplayer_bbox3d_t buf_to_Jplayer_bbox3d_t(SWIGTYPE_p_void buf) {
    player_bbox3d_t data = playercore_java.buf_to_player_bbox3d_t(buf);
    return(player_bbox3d_t_to_Jplayer_bbox3d_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_bbox3d_t_to_buf(Jplayer_bbox3d_t Jdata) {
    player_bbox3d_t data = Jplayer_bbox3d_t_to_player_bbox3d_t(Jdata);
    return(playercore_java.player_bbox3d_t_to_buf(data));
  }

  public static Jplayer_bbox3d_t player_bbox3d_t_to_Jplayer_bbox3d_t(player_bbox3d_t data) {
    Jplayer_bbox3d_t Jdata = new Jplayer_bbox3d_t();
    Jdata.sw = data.getSw();
    Jdata.sl = data.getSl();
    Jdata.sh = data.getSh();
    return(Jdata);
  }

  public static player_bbox3d_t Jplayer_bbox3d_t_to_player_bbox3d_t(Jplayer_bbox3d_t Jdata) {
    player_bbox3d_t data = new player_bbox3d_t();
    data.setSw(Jdata.sw);
    data.setSl(Jdata.sl);
    data.setSh(Jdata.sh);
    return(data);
  }

  public static Jplayer_segment_t buf_to_Jplayer_segment_t(SWIGTYPE_p_void buf) {
    player_segment_t data = playercore_java.buf_to_player_segment_t(buf);
    return(player_segment_t_to_Jplayer_segment_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_segment_t_to_buf(Jplayer_segment_t Jdata) {
    player_segment_t data = Jplayer_segment_t_to_player_segment_t(Jdata);
    return(playercore_java.player_segment_t_to_buf(data));
  }

  public static Jplayer_segment_t player_segment_t_to_Jplayer_segment_t(player_segment_t data) {
    Jplayer_segment_t Jdata = new Jplayer_segment_t();
    Jdata.x0 = data.getX0();
    Jdata.y0 = data.getY0();
    Jdata.x1 = data.getX1();
    Jdata.y1 = data.getY1();
    return(Jdata);
  }

  public static player_segment_t Jplayer_segment_t_to_player_segment_t(Jplayer_segment_t Jdata) {
    player_segment_t data = new player_segment_t();
    data.setX0(Jdata.x0);
    data.setY0(Jdata.y0);
    data.setX1(Jdata.x1);
    data.setY1(Jdata.y1);
    return(data);
  }

  public static Jplayer_color_t buf_to_Jplayer_color_t(SWIGTYPE_p_void buf) {
    player_color_t data = playercore_java.buf_to_player_color_t(buf);
    return(player_color_t_to_Jplayer_color_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_color_t_to_buf(Jplayer_color_t Jdata) {
    player_color_t data = Jplayer_color_t_to_player_color_t(Jdata);
    return(playercore_java.player_color_t_to_buf(data));
  }

  public static Jplayer_color_t player_color_t_to_Jplayer_color_t(player_color_t data) {
    Jplayer_color_t Jdata = new Jplayer_color_t();
    Jdata.alpha = data.getAlpha();
    Jdata.red = data.getRed();
    Jdata.green = data.getGreen();
    Jdata.blue = data.getBlue();
    return(Jdata);
  }

  public static player_color_t Jplayer_color_t_to_player_color_t(Jplayer_color_t Jdata) {
    player_color_t data = new player_color_t();
    data.setAlpha(Jdata.alpha);
    data.setRed(Jdata.red);
    data.setGreen(Jdata.green);
    data.setBlue(Jdata.blue);
    return(data);
  }

  public static Jplayer_bool_t buf_to_Jplayer_bool_t(SWIGTYPE_p_void buf) {
    player_bool_t data = playercore_java.buf_to_player_bool_t(buf);
    return(player_bool_t_to_Jplayer_bool_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_bool_t_to_buf(Jplayer_bool_t Jdata) {
    player_bool_t data = Jplayer_bool_t_to_player_bool_t(Jdata);
    return(playercore_java.player_bool_t_to_buf(data));
  }

  public static Jplayer_bool_t player_bool_t_to_Jplayer_bool_t(player_bool_t data) {
    Jplayer_bool_t Jdata = new Jplayer_bool_t();
    Jdata.state = data.getState();
    return(Jdata);
  }

  public static player_bool_t Jplayer_bool_t_to_player_bool_t(Jplayer_bool_t Jdata) {
    player_bool_t data = new player_bool_t();
    data.setState(Jdata.state);
    return(data);
  }

  public static Jplayer_actarray_actuator_t buf_to_Jplayer_actarray_actuator_t(SWIGTYPE_p_void buf) {
    player_actarray_actuator_t data = playercore_java.buf_to_player_actarray_actuator_t(buf);
    return(player_actarray_actuator_t_to_Jplayer_actarray_actuator_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_actarray_actuator_t_to_buf(Jplayer_actarray_actuator_t Jdata) {
    player_actarray_actuator_t data = Jplayer_actarray_actuator_t_to_player_actarray_actuator_t(Jdata);
    return(playercore_java.player_actarray_actuator_t_to_buf(data));
  }

  public static Jplayer_actarray_actuator_t player_actarray_actuator_t_to_Jplayer_actarray_actuator_t(player_actarray_actuator_t data) {
    Jplayer_actarray_actuator_t Jdata = new Jplayer_actarray_actuator_t();
    Jdata.position = data.getPosition();
    Jdata.speed = data.getSpeed();
    Jdata.state = data.getState();
    return(Jdata);
  }

  public static player_actarray_actuator_t Jplayer_actarray_actuator_t_to_player_actarray_actuator_t(Jplayer_actarray_actuator_t Jdata) {
    player_actarray_actuator_t data = new player_actarray_actuator_t();
    data.setPosition(Jdata.position);
    data.setSpeed(Jdata.speed);
    data.setState(Jdata.state);
    return(data);
  }

  public static Jplayer_actarray_data_t buf_to_Jplayer_actarray_data_t(SWIGTYPE_p_void buf) {
    player_actarray_data_t data = playercore_java.buf_to_player_actarray_data_t(buf);
    return(player_actarray_data_t_to_Jplayer_actarray_data_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_actarray_data_t_to_buf(Jplayer_actarray_data_t Jdata) {
    player_actarray_data_t data = Jplayer_actarray_data_t_to_player_actarray_data_t(Jdata);
    return(playercore_java.player_actarray_data_t_to_buf(data));
  }

  public static Jplayer_actarray_data_t player_actarray_data_t_to_Jplayer_actarray_data_t(player_actarray_data_t data) {
    Jplayer_actarray_data_t Jdata = new Jplayer_actarray_data_t();
    Jdata.actuators_count = data.getActuators_count();
    {
      player_actarray_actuator_t foo[] = data.getActuators();
      for(int i=0;i<playercore_javaConstants.PLAYER_ACTARRAY_NUM_ACTUATORS;i++)
        Jdata.actuators[i] = player_actarray_actuator_t_to_Jplayer_actarray_actuator_t(foo[i]);
    }
    return(Jdata);
  }

  public static player_actarray_data_t Jplayer_actarray_data_t_to_player_actarray_data_t(Jplayer_actarray_data_t Jdata) {
    player_actarray_data_t data = new player_actarray_data_t();
    data.setActuators_count(Jdata.actuators_count);
    {
      player_actarray_actuator_t foo[] = new player_actarray_actuator_t[playercore_javaConstants.PLAYER_ACTARRAY_NUM_ACTUATORS];
      for(int i=0;i<playercore_javaConstants.PLAYER_ACTARRAY_NUM_ACTUATORS;i++)
        foo[i] = Jplayer_actarray_actuator_t_to_player_actarray_actuator_t(Jdata.actuators[i]);
      data.setActuators(foo);
    }
    return(data);
  }

  public static Jplayer_actarray_actuatorgeom_t buf_to_Jplayer_actarray_actuatorgeom_t(SWIGTYPE_p_void buf) {
    player_actarray_actuatorgeom_t data = playercore_java.buf_to_player_actarray_actuatorgeom_t(buf);
    return(player_actarray_actuatorgeom_t_to_Jplayer_actarray_actuatorgeom_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_actarray_actuatorgeom_t_to_buf(Jplayer_actarray_actuatorgeom_t Jdata) {
    player_actarray_actuatorgeom_t data = Jplayer_actarray_actuatorgeom_t_to_player_actarray_actuatorgeom_t(Jdata);
    return(playercore_java.player_actarray_actuatorgeom_t_to_buf(data));
  }

  public static Jplayer_actarray_actuatorgeom_t player_actarray_actuatorgeom_t_to_Jplayer_actarray_actuatorgeom_t(player_actarray_actuatorgeom_t data) {
    Jplayer_actarray_actuatorgeom_t Jdata = new Jplayer_actarray_actuatorgeom_t();
    Jdata.type = data.getType();
    Jdata.min = data.getMin();
    Jdata.centre = data.getCentre();
    Jdata.max = data.getMax();
    Jdata.home = data.getHome();
    Jdata.config_speed = data.getConfig_speed();
    Jdata.hasbrakes = data.getHasbrakes();
    return(Jdata);
  }

  public static player_actarray_actuatorgeom_t Jplayer_actarray_actuatorgeom_t_to_player_actarray_actuatorgeom_t(Jplayer_actarray_actuatorgeom_t Jdata) {
    player_actarray_actuatorgeom_t data = new player_actarray_actuatorgeom_t();
    data.setType(Jdata.type);
    data.setMin(Jdata.min);
    data.setCentre(Jdata.centre);
    data.setMax(Jdata.max);
    data.setHome(Jdata.home);
    data.setConfig_speed(Jdata.config_speed);
    data.setHasbrakes(Jdata.hasbrakes);
    return(data);
  }

  public static Jplayer_actarray_geom_t buf_to_Jplayer_actarray_geom_t(SWIGTYPE_p_void buf) {
    player_actarray_geom_t data = playercore_java.buf_to_player_actarray_geom_t(buf);
    return(player_actarray_geom_t_to_Jplayer_actarray_geom_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_actarray_geom_t_to_buf(Jplayer_actarray_geom_t Jdata) {
    player_actarray_geom_t data = Jplayer_actarray_geom_t_to_player_actarray_geom_t(Jdata);
    return(playercore_java.player_actarray_geom_t_to_buf(data));
  }

  public static Jplayer_actarray_geom_t player_actarray_geom_t_to_Jplayer_actarray_geom_t(player_actarray_geom_t data) {
    Jplayer_actarray_geom_t Jdata = new Jplayer_actarray_geom_t();
    Jdata.actuators_count = data.getActuators_count();
    {
      player_actarray_actuatorgeom_t foo[] = data.getActuators();
      for(int i=0;i<playercore_javaConstants.PLAYER_ACTARRAY_NUM_ACTUATORS;i++)
        Jdata.actuators[i] = player_actarray_actuatorgeom_t_to_Jplayer_actarray_actuatorgeom_t(foo[i]);
    }
    return(Jdata);
  }

  public static player_actarray_geom_t Jplayer_actarray_geom_t_to_player_actarray_geom_t(Jplayer_actarray_geom_t Jdata) {
    player_actarray_geom_t data = new player_actarray_geom_t();
    data.setActuators_count(Jdata.actuators_count);
    {
      player_actarray_actuatorgeom_t foo[] = new player_actarray_actuatorgeom_t[playercore_javaConstants.PLAYER_ACTARRAY_NUM_ACTUATORS];
      for(int i=0;i<playercore_javaConstants.PLAYER_ACTARRAY_NUM_ACTUATORS;i++)
        foo[i] = Jplayer_actarray_actuatorgeom_t_to_player_actarray_actuatorgeom_t(Jdata.actuators[i]);
      data.setActuators(foo);
    }
    return(data);
  }

  public static Jplayer_actarray_position_cmd_t buf_to_Jplayer_actarray_position_cmd_t(SWIGTYPE_p_void buf) {
    player_actarray_position_cmd_t data = playercore_java.buf_to_player_actarray_position_cmd_t(buf);
    return(player_actarray_position_cmd_t_to_Jplayer_actarray_position_cmd_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_actarray_position_cmd_t_to_buf(Jplayer_actarray_position_cmd_t Jdata) {
    player_actarray_position_cmd_t data = Jplayer_actarray_position_cmd_t_to_player_actarray_position_cmd_t(Jdata);
    return(playercore_java.player_actarray_position_cmd_t_to_buf(data));
  }

  public static Jplayer_actarray_position_cmd_t player_actarray_position_cmd_t_to_Jplayer_actarray_position_cmd_t(player_actarray_position_cmd_t data) {
    Jplayer_actarray_position_cmd_t Jdata = new Jplayer_actarray_position_cmd_t();
    Jdata.joint = data.getJoint();
    Jdata.position = data.getPosition();
    return(Jdata);
  }

  public static player_actarray_position_cmd_t Jplayer_actarray_position_cmd_t_to_player_actarray_position_cmd_t(Jplayer_actarray_position_cmd_t Jdata) {
    player_actarray_position_cmd_t data = new player_actarray_position_cmd_t();
    data.setJoint(Jdata.joint);
    data.setPosition(Jdata.position);
    return(data);
  }

  public static Jplayer_actarray_speed_cmd_t buf_to_Jplayer_actarray_speed_cmd_t(SWIGTYPE_p_void buf) {
    player_actarray_speed_cmd_t data = playercore_java.buf_to_player_actarray_speed_cmd_t(buf);
    return(player_actarray_speed_cmd_t_to_Jplayer_actarray_speed_cmd_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_actarray_speed_cmd_t_to_buf(Jplayer_actarray_speed_cmd_t Jdata) {
    player_actarray_speed_cmd_t data = Jplayer_actarray_speed_cmd_t_to_player_actarray_speed_cmd_t(Jdata);
    return(playercore_java.player_actarray_speed_cmd_t_to_buf(data));
  }

  public static Jplayer_actarray_speed_cmd_t player_actarray_speed_cmd_t_to_Jplayer_actarray_speed_cmd_t(player_actarray_speed_cmd_t data) {
    Jplayer_actarray_speed_cmd_t Jdata = new Jplayer_actarray_speed_cmd_t();
    Jdata.joint = data.getJoint();
    Jdata.speed = data.getSpeed();
    return(Jdata);
  }

  public static player_actarray_speed_cmd_t Jplayer_actarray_speed_cmd_t_to_player_actarray_speed_cmd_t(Jplayer_actarray_speed_cmd_t Jdata) {
    player_actarray_speed_cmd_t data = new player_actarray_speed_cmd_t();
    data.setJoint(Jdata.joint);
    data.setSpeed(Jdata.speed);
    return(data);
  }

  public static Jplayer_actarray_home_cmd_t buf_to_Jplayer_actarray_home_cmd_t(SWIGTYPE_p_void buf) {
    player_actarray_home_cmd_t data = playercore_java.buf_to_player_actarray_home_cmd_t(buf);
    return(player_actarray_home_cmd_t_to_Jplayer_actarray_home_cmd_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_actarray_home_cmd_t_to_buf(Jplayer_actarray_home_cmd_t Jdata) {
    player_actarray_home_cmd_t data = Jplayer_actarray_home_cmd_t_to_player_actarray_home_cmd_t(Jdata);
    return(playercore_java.player_actarray_home_cmd_t_to_buf(data));
  }

  public static Jplayer_actarray_home_cmd_t player_actarray_home_cmd_t_to_Jplayer_actarray_home_cmd_t(player_actarray_home_cmd_t data) {
    Jplayer_actarray_home_cmd_t Jdata = new Jplayer_actarray_home_cmd_t();
    Jdata.joint = data.getJoint();
    return(Jdata);
  }

  public static player_actarray_home_cmd_t Jplayer_actarray_home_cmd_t_to_player_actarray_home_cmd_t(Jplayer_actarray_home_cmd_t Jdata) {
    player_actarray_home_cmd_t data = new player_actarray_home_cmd_t();
    data.setJoint(Jdata.joint);
    return(data);
  }

  public static Jplayer_actarray_power_config_t buf_to_Jplayer_actarray_power_config_t(SWIGTYPE_p_void buf) {
    player_actarray_power_config_t data = playercore_java.buf_to_player_actarray_power_config_t(buf);
    return(player_actarray_power_config_t_to_Jplayer_actarray_power_config_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_actarray_power_config_t_to_buf(Jplayer_actarray_power_config_t Jdata) {
    player_actarray_power_config_t data = Jplayer_actarray_power_config_t_to_player_actarray_power_config_t(Jdata);
    return(playercore_java.player_actarray_power_config_t_to_buf(data));
  }

  public static Jplayer_actarray_power_config_t player_actarray_power_config_t_to_Jplayer_actarray_power_config_t(player_actarray_power_config_t data) {
    Jplayer_actarray_power_config_t Jdata = new Jplayer_actarray_power_config_t();
    Jdata.value = data.getValue();
    return(Jdata);
  }

  public static player_actarray_power_config_t Jplayer_actarray_power_config_t_to_player_actarray_power_config_t(Jplayer_actarray_power_config_t Jdata) {
    player_actarray_power_config_t data = new player_actarray_power_config_t();
    data.setValue(Jdata.value);
    return(data);
  }

  public static Jplayer_actarray_brakes_config_t buf_to_Jplayer_actarray_brakes_config_t(SWIGTYPE_p_void buf) {
    player_actarray_brakes_config_t data = playercore_java.buf_to_player_actarray_brakes_config_t(buf);
    return(player_actarray_brakes_config_t_to_Jplayer_actarray_brakes_config_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_actarray_brakes_config_t_to_buf(Jplayer_actarray_brakes_config_t Jdata) {
    player_actarray_brakes_config_t data = Jplayer_actarray_brakes_config_t_to_player_actarray_brakes_config_t(Jdata);
    return(playercore_java.player_actarray_brakes_config_t_to_buf(data));
  }

  public static Jplayer_actarray_brakes_config_t player_actarray_brakes_config_t_to_Jplayer_actarray_brakes_config_t(player_actarray_brakes_config_t data) {
    Jplayer_actarray_brakes_config_t Jdata = new Jplayer_actarray_brakes_config_t();
    Jdata.value = data.getValue();
    return(Jdata);
  }

  public static player_actarray_brakes_config_t Jplayer_actarray_brakes_config_t_to_player_actarray_brakes_config_t(Jplayer_actarray_brakes_config_t Jdata) {
    player_actarray_brakes_config_t data = new player_actarray_brakes_config_t();
    data.setValue(Jdata.value);
    return(data);
  }

  public static Jplayer_actarray_speed_config_t buf_to_Jplayer_actarray_speed_config_t(SWIGTYPE_p_void buf) {
    player_actarray_speed_config_t data = playercore_java.buf_to_player_actarray_speed_config_t(buf);
    return(player_actarray_speed_config_t_to_Jplayer_actarray_speed_config_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_actarray_speed_config_t_to_buf(Jplayer_actarray_speed_config_t Jdata) {
    player_actarray_speed_config_t data = Jplayer_actarray_speed_config_t_to_player_actarray_speed_config_t(Jdata);
    return(playercore_java.player_actarray_speed_config_t_to_buf(data));
  }

  public static Jplayer_actarray_speed_config_t player_actarray_speed_config_t_to_Jplayer_actarray_speed_config_t(player_actarray_speed_config_t data) {
    Jplayer_actarray_speed_config_t Jdata = new Jplayer_actarray_speed_config_t();
    Jdata.joint = data.getJoint();
    Jdata.speed = data.getSpeed();
    return(Jdata);
  }

  public static player_actarray_speed_config_t Jplayer_actarray_speed_config_t_to_player_actarray_speed_config_t(Jplayer_actarray_speed_config_t Jdata) {
    player_actarray_speed_config_t data = new player_actarray_speed_config_t();
    data.setJoint(Jdata.joint);
    data.setSpeed(Jdata.speed);
    return(data);
  }

  public static Jplayer_aio_data_t buf_to_Jplayer_aio_data_t(SWIGTYPE_p_void buf) {
    player_aio_data_t data = playercore_java.buf_to_player_aio_data_t(buf);
    return(player_aio_data_t_to_Jplayer_aio_data_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_aio_data_t_to_buf(Jplayer_aio_data_t Jdata) {
    player_aio_data_t data = Jplayer_aio_data_t_to_player_aio_data_t(Jdata);
    return(playercore_java.player_aio_data_t_to_buf(data));
  }

  public static Jplayer_aio_data_t player_aio_data_t_to_Jplayer_aio_data_t(player_aio_data_t data) {
    Jplayer_aio_data_t Jdata = new Jplayer_aio_data_t();
    Jdata.voltages_count = data.getVoltages_count();
    Jdata.voltages = data.getVoltages();
    return(Jdata);
  }

  public static player_aio_data_t Jplayer_aio_data_t_to_player_aio_data_t(Jplayer_aio_data_t Jdata) {
    player_aio_data_t data = new player_aio_data_t();
    data.setVoltages_count(Jdata.voltages_count);
    data.setVoltages(Jdata.voltages);
    return(data);
  }

  public static Jplayer_aio_cmd_t buf_to_Jplayer_aio_cmd_t(SWIGTYPE_p_void buf) {
    player_aio_cmd_t data = playercore_java.buf_to_player_aio_cmd_t(buf);
    return(player_aio_cmd_t_to_Jplayer_aio_cmd_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_aio_cmd_t_to_buf(Jplayer_aio_cmd_t Jdata) {
    player_aio_cmd_t data = Jplayer_aio_cmd_t_to_player_aio_cmd_t(Jdata);
    return(playercore_java.player_aio_cmd_t_to_buf(data));
  }

  public static Jplayer_aio_cmd_t player_aio_cmd_t_to_Jplayer_aio_cmd_t(player_aio_cmd_t data) {
    Jplayer_aio_cmd_t Jdata = new Jplayer_aio_cmd_t();
    Jdata.id = data.getId();
    Jdata.voltage = data.getVoltage();
    return(Jdata);
  }

  public static player_aio_cmd_t Jplayer_aio_cmd_t_to_player_aio_cmd_t(Jplayer_aio_cmd_t Jdata) {
    player_aio_cmd_t data = new player_aio_cmd_t();
    data.setId(Jdata.id);
    data.setVoltage(Jdata.voltage);
    return(data);
  }

  public static Jplayer_audio_data_t buf_to_Jplayer_audio_data_t(SWIGTYPE_p_void buf) {
    player_audio_data_t data = playercore_java.buf_to_player_audio_data_t(buf);
    return(player_audio_data_t_to_Jplayer_audio_data_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_audio_data_t_to_buf(Jplayer_audio_data_t Jdata) {
    player_audio_data_t data = Jplayer_audio_data_t_to_player_audio_data_t(Jdata);
    return(playercore_java.player_audio_data_t_to_buf(data));
  }

  public static Jplayer_audio_data_t player_audio_data_t_to_Jplayer_audio_data_t(player_audio_data_t data) {
    Jplayer_audio_data_t Jdata = new Jplayer_audio_data_t();
    Jdata.frequency_count = data.getFrequency_count();
    Jdata.frequency = data.getFrequency();
    Jdata.amplitude_count = data.getAmplitude_count();
    Jdata.amplitude = data.getAmplitude();
    return(Jdata);
  }

  public static player_audio_data_t Jplayer_audio_data_t_to_player_audio_data_t(Jplayer_audio_data_t Jdata) {
    player_audio_data_t data = new player_audio_data_t();
    data.setFrequency_count(Jdata.frequency_count);
    data.setFrequency(Jdata.frequency);
    data.setAmplitude_count(Jdata.amplitude_count);
    data.setAmplitude(Jdata.amplitude);
    return(data);
  }

  public static Jplayer_audio_cmd_t buf_to_Jplayer_audio_cmd_t(SWIGTYPE_p_void buf) {
    player_audio_cmd_t data = playercore_java.buf_to_player_audio_cmd_t(buf);
    return(player_audio_cmd_t_to_Jplayer_audio_cmd_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_audio_cmd_t_to_buf(Jplayer_audio_cmd_t Jdata) {
    player_audio_cmd_t data = Jplayer_audio_cmd_t_to_player_audio_cmd_t(Jdata);
    return(playercore_java.player_audio_cmd_t_to_buf(data));
  }

  public static Jplayer_audio_cmd_t player_audio_cmd_t_to_Jplayer_audio_cmd_t(player_audio_cmd_t data) {
    Jplayer_audio_cmd_t Jdata = new Jplayer_audio_cmd_t();
    Jdata.frequency = data.getFrequency();
    Jdata.amplitude = data.getAmplitude();
    Jdata.duration = data.getDuration();
    return(Jdata);
  }

  public static player_audio_cmd_t Jplayer_audio_cmd_t_to_player_audio_cmd_t(Jplayer_audio_cmd_t Jdata) {
    player_audio_cmd_t data = new player_audio_cmd_t();
    data.setFrequency(Jdata.frequency);
    data.setAmplitude(Jdata.amplitude);
    data.setDuration(Jdata.duration);
    return(data);
  }

  public static Jplayer_audiodsp_data_t buf_to_Jplayer_audiodsp_data_t(SWIGTYPE_p_void buf) {
    player_audiodsp_data_t data = playercore_java.buf_to_player_audiodsp_data_t(buf);
    return(player_audiodsp_data_t_to_Jplayer_audiodsp_data_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_audiodsp_data_t_to_buf(Jplayer_audiodsp_data_t Jdata) {
    player_audiodsp_data_t data = Jplayer_audiodsp_data_t_to_player_audiodsp_data_t(Jdata);
    return(playercore_java.player_audiodsp_data_t_to_buf(data));
  }

  public static Jplayer_audiodsp_data_t player_audiodsp_data_t_to_Jplayer_audiodsp_data_t(player_audiodsp_data_t data) {
    Jplayer_audiodsp_data_t Jdata = new Jplayer_audiodsp_data_t();
    Jdata.frequency_count = data.getFrequency_count();
    Jdata.frequency = data.getFrequency();
    Jdata.amplitude_count = data.getAmplitude_count();
    Jdata.amplitude = data.getAmplitude();
    return(Jdata);
  }

  public static player_audiodsp_data_t Jplayer_audiodsp_data_t_to_player_audiodsp_data_t(Jplayer_audiodsp_data_t Jdata) {
    player_audiodsp_data_t data = new player_audiodsp_data_t();
    data.setFrequency_count(Jdata.frequency_count);
    data.setFrequency(Jdata.frequency);
    data.setAmplitude_count(Jdata.amplitude_count);
    data.setAmplitude(Jdata.amplitude);
    return(data);
  }

  public static Jplayer_audiodsp_cmd_t buf_to_Jplayer_audiodsp_cmd_t(SWIGTYPE_p_void buf) {
    player_audiodsp_cmd_t data = playercore_java.buf_to_player_audiodsp_cmd_t(buf);
    return(player_audiodsp_cmd_t_to_Jplayer_audiodsp_cmd_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_audiodsp_cmd_t_to_buf(Jplayer_audiodsp_cmd_t Jdata) {
    player_audiodsp_cmd_t data = Jplayer_audiodsp_cmd_t_to_player_audiodsp_cmd_t(Jdata);
    return(playercore_java.player_audiodsp_cmd_t_to_buf(data));
  }

  public static Jplayer_audiodsp_cmd_t player_audiodsp_cmd_t_to_Jplayer_audiodsp_cmd_t(player_audiodsp_cmd_t data) {
    Jplayer_audiodsp_cmd_t Jdata = new Jplayer_audiodsp_cmd_t();
    Jdata.frequency = data.getFrequency();
    Jdata.amplitude = data.getAmplitude();
    Jdata.duration = data.getDuration();
    Jdata.bit_string_count = data.getBit_string_count();
    Jdata.bit_string = data.getBit_string();
    return(Jdata);
  }

  public static player_audiodsp_cmd_t Jplayer_audiodsp_cmd_t_to_player_audiodsp_cmd_t(Jplayer_audiodsp_cmd_t Jdata) {
    player_audiodsp_cmd_t data = new player_audiodsp_cmd_t();
    data.setFrequency(Jdata.frequency);
    data.setAmplitude(Jdata.amplitude);
    data.setDuration(Jdata.duration);
    data.setBit_string_count(Jdata.bit_string_count);
    data.setBit_string(Jdata.bit_string);
    return(data);
  }

  public static Jplayer_audiodsp_config_t buf_to_Jplayer_audiodsp_config_t(SWIGTYPE_p_void buf) {
    player_audiodsp_config_t data = playercore_java.buf_to_player_audiodsp_config_t(buf);
    return(player_audiodsp_config_t_to_Jplayer_audiodsp_config_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_audiodsp_config_t_to_buf(Jplayer_audiodsp_config_t Jdata) {
    player_audiodsp_config_t data = Jplayer_audiodsp_config_t_to_player_audiodsp_config_t(Jdata);
    return(playercore_java.player_audiodsp_config_t_to_buf(data));
  }

  public static Jplayer_audiodsp_config_t player_audiodsp_config_t_to_Jplayer_audiodsp_config_t(player_audiodsp_config_t data) {
    Jplayer_audiodsp_config_t Jdata = new Jplayer_audiodsp_config_t();
    Jdata.format = data.getFormat();
    Jdata.frequency = data.getFrequency();
    Jdata.channels = data.getChannels();
    return(Jdata);
  }

  public static player_audiodsp_config_t Jplayer_audiodsp_config_t_to_player_audiodsp_config_t(Jplayer_audiodsp_config_t Jdata) {
    player_audiodsp_config_t data = new player_audiodsp_config_t();
    data.setFormat(Jdata.format);
    data.setFrequency(Jdata.frequency);
    data.setChannels(Jdata.channels);
    return(data);
  }

  public static Jplayer_audiomixer_cmd_t buf_to_Jplayer_audiomixer_cmd_t(SWIGTYPE_p_void buf) {
    player_audiomixer_cmd_t data = playercore_java.buf_to_player_audiomixer_cmd_t(buf);
    return(player_audiomixer_cmd_t_to_Jplayer_audiomixer_cmd_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_audiomixer_cmd_t_to_buf(Jplayer_audiomixer_cmd_t Jdata) {
    player_audiomixer_cmd_t data = Jplayer_audiomixer_cmd_t_to_player_audiomixer_cmd_t(Jdata);
    return(playercore_java.player_audiomixer_cmd_t_to_buf(data));
  }

  public static Jplayer_audiomixer_cmd_t player_audiomixer_cmd_t_to_Jplayer_audiomixer_cmd_t(player_audiomixer_cmd_t data) {
    Jplayer_audiomixer_cmd_t Jdata = new Jplayer_audiomixer_cmd_t();
    Jdata.left = data.getLeft();
    Jdata.right = data.getRight();
    return(Jdata);
  }

  public static player_audiomixer_cmd_t Jplayer_audiomixer_cmd_t_to_player_audiomixer_cmd_t(Jplayer_audiomixer_cmd_t Jdata) {
    player_audiomixer_cmd_t data = new player_audiomixer_cmd_t();
    data.setLeft(Jdata.left);
    data.setRight(Jdata.right);
    return(data);
  }

  public static Jplayer_audiomixer_config_t buf_to_Jplayer_audiomixer_config_t(SWIGTYPE_p_void buf) {
    player_audiomixer_config_t data = playercore_java.buf_to_player_audiomixer_config_t(buf);
    return(player_audiomixer_config_t_to_Jplayer_audiomixer_config_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_audiomixer_config_t_to_buf(Jplayer_audiomixer_config_t Jdata) {
    player_audiomixer_config_t data = Jplayer_audiomixer_config_t_to_player_audiomixer_config_t(Jdata);
    return(playercore_java.player_audiomixer_config_t_to_buf(data));
  }

  public static Jplayer_audiomixer_config_t player_audiomixer_config_t_to_Jplayer_audiomixer_config_t(player_audiomixer_config_t data) {
    Jplayer_audiomixer_config_t Jdata = new Jplayer_audiomixer_config_t();
    Jdata.master_left = data.getMaster_left();
    Jdata.master_right = data.getMaster_right();
    Jdata.pcm_left = data.getPcm_left();
    Jdata.pcm_right = data.getPcm_right();
    Jdata.line_left = data.getLine_left();
    Jdata.line_right = data.getLine_right();
    Jdata.mic_left = data.getMic_left();
    Jdata.mic_right = data.getMic_right();
    Jdata.i_gain = data.getI_gain();
    Jdata.o_gain = data.getO_gain();
    return(Jdata);
  }

  public static player_audiomixer_config_t Jplayer_audiomixer_config_t_to_player_audiomixer_config_t(Jplayer_audiomixer_config_t Jdata) {
    player_audiomixer_config_t data = new player_audiomixer_config_t();
    data.setMaster_left(Jdata.master_left);
    data.setMaster_right(Jdata.master_right);
    data.setPcm_left(Jdata.pcm_left);
    data.setPcm_right(Jdata.pcm_right);
    data.setLine_left(Jdata.line_left);
    data.setLine_right(Jdata.line_right);
    data.setMic_left(Jdata.mic_left);
    data.setMic_right(Jdata.mic_right);
    data.setI_gain(Jdata.i_gain);
    data.setO_gain(Jdata.o_gain);
    return(data);
  }

  public static Jplayer_blinkenlight_data_t buf_to_Jplayer_blinkenlight_data_t(SWIGTYPE_p_void buf) {
    player_blinkenlight_data_t data = playercore_java.buf_to_player_blinkenlight_data_t(buf);
    return(player_blinkenlight_data_t_to_Jplayer_blinkenlight_data_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_blinkenlight_data_t_to_buf(Jplayer_blinkenlight_data_t Jdata) {
    player_blinkenlight_data_t data = Jplayer_blinkenlight_data_t_to_player_blinkenlight_data_t(Jdata);
    return(playercore_java.player_blinkenlight_data_t_to_buf(data));
  }

  public static Jplayer_blinkenlight_data_t player_blinkenlight_data_t_to_Jplayer_blinkenlight_data_t(player_blinkenlight_data_t data) {
    Jplayer_blinkenlight_data_t Jdata = new Jplayer_blinkenlight_data_t();
    Jdata.enable = data.getEnable();
    Jdata.period = data.getPeriod();
    Jdata.dutycycle = data.getDutycycle();
    Jdata.color = player_color_t_to_Jplayer_color_t(data.getColor());
    return(Jdata);
  }

  public static player_blinkenlight_data_t Jplayer_blinkenlight_data_t_to_player_blinkenlight_data_t(Jplayer_blinkenlight_data_t Jdata) {
    player_blinkenlight_data_t data = new player_blinkenlight_data_t();
    data.setEnable(Jdata.enable);
    data.setPeriod(Jdata.period);
    data.setDutycycle(Jdata.dutycycle);
    data.setColor(Jplayer_color_t_to_player_color_t(Jdata.color));
    return(data);
  }

  public static Jplayer_blinkenlight_cmd_t buf_to_Jplayer_blinkenlight_cmd_t(SWIGTYPE_p_void buf) {
    player_blinkenlight_cmd_t data = playercore_java.buf_to_player_blinkenlight_cmd_t(buf);
    return(player_blinkenlight_cmd_t_to_Jplayer_blinkenlight_cmd_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_blinkenlight_cmd_t_to_buf(Jplayer_blinkenlight_cmd_t Jdata) {
    player_blinkenlight_cmd_t data = Jplayer_blinkenlight_cmd_t_to_player_blinkenlight_cmd_t(Jdata);
    return(playercore_java.player_blinkenlight_cmd_t_to_buf(data));
  }

  public static Jplayer_blinkenlight_cmd_t player_blinkenlight_cmd_t_to_Jplayer_blinkenlight_cmd_t(player_blinkenlight_cmd_t data) {
    Jplayer_blinkenlight_cmd_t Jdata = new Jplayer_blinkenlight_cmd_t();
    Jdata.enable = data.getEnable();
    Jdata.period = data.getPeriod();
    Jdata.dutycycle = data.getDutycycle();
    Jdata.color = player_color_t_to_Jplayer_color_t(data.getColor());
    return(Jdata);
  }

  public static player_blinkenlight_cmd_t Jplayer_blinkenlight_cmd_t_to_player_blinkenlight_cmd_t(Jplayer_blinkenlight_cmd_t Jdata) {
    player_blinkenlight_cmd_t data = new player_blinkenlight_cmd_t();
    data.setEnable(Jdata.enable);
    data.setPeriod(Jdata.period);
    data.setDutycycle(Jdata.dutycycle);
    data.setColor(Jplayer_color_t_to_player_color_t(Jdata.color));
    return(data);
  }

  public static Jplayer_blinkenlight_cmd_power_t buf_to_Jplayer_blinkenlight_cmd_power_t(SWIGTYPE_p_void buf) {
    player_blinkenlight_cmd_power_t data = playercore_java.buf_to_player_blinkenlight_cmd_power_t(buf);
    return(player_blinkenlight_cmd_power_t_to_Jplayer_blinkenlight_cmd_power_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_blinkenlight_cmd_power_t_to_buf(Jplayer_blinkenlight_cmd_power_t Jdata) {
    player_blinkenlight_cmd_power_t data = Jplayer_blinkenlight_cmd_power_t_to_player_blinkenlight_cmd_power_t(Jdata);
    return(playercore_java.player_blinkenlight_cmd_power_t_to_buf(data));
  }

  public static Jplayer_blinkenlight_cmd_power_t player_blinkenlight_cmd_power_t_to_Jplayer_blinkenlight_cmd_power_t(player_blinkenlight_cmd_power_t data) {
    Jplayer_blinkenlight_cmd_power_t Jdata = new Jplayer_blinkenlight_cmd_power_t();
    Jdata.enable = data.getEnable();
    return(Jdata);
  }

  public static player_blinkenlight_cmd_power_t Jplayer_blinkenlight_cmd_power_t_to_player_blinkenlight_cmd_power_t(Jplayer_blinkenlight_cmd_power_t Jdata) {
    player_blinkenlight_cmd_power_t data = new player_blinkenlight_cmd_power_t();
    data.setEnable(Jdata.enable);
    return(data);
  }

  public static Jplayer_blinkenlight_cmd_color_t buf_to_Jplayer_blinkenlight_cmd_color_t(SWIGTYPE_p_void buf) {
    player_blinkenlight_cmd_color_t data = playercore_java.buf_to_player_blinkenlight_cmd_color_t(buf);
    return(player_blinkenlight_cmd_color_t_to_Jplayer_blinkenlight_cmd_color_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_blinkenlight_cmd_color_t_to_buf(Jplayer_blinkenlight_cmd_color_t Jdata) {
    player_blinkenlight_cmd_color_t data = Jplayer_blinkenlight_cmd_color_t_to_player_blinkenlight_cmd_color_t(Jdata);
    return(playercore_java.player_blinkenlight_cmd_color_t_to_buf(data));
  }

  public static Jplayer_blinkenlight_cmd_color_t player_blinkenlight_cmd_color_t_to_Jplayer_blinkenlight_cmd_color_t(player_blinkenlight_cmd_color_t data) {
    Jplayer_blinkenlight_cmd_color_t Jdata = new Jplayer_blinkenlight_cmd_color_t();
    Jdata.color = player_color_t_to_Jplayer_color_t(data.getColor());
    return(Jdata);
  }

  public static player_blinkenlight_cmd_color_t Jplayer_blinkenlight_cmd_color_t_to_player_blinkenlight_cmd_color_t(Jplayer_blinkenlight_cmd_color_t Jdata) {
    player_blinkenlight_cmd_color_t data = new player_blinkenlight_cmd_color_t();
    data.setColor(Jplayer_color_t_to_player_color_t(Jdata.color));
    return(data);
  }

  public static Jplayer_blinkenlight_cmd_period_t buf_to_Jplayer_blinkenlight_cmd_period_t(SWIGTYPE_p_void buf) {
    player_blinkenlight_cmd_period_t data = playercore_java.buf_to_player_blinkenlight_cmd_period_t(buf);
    return(player_blinkenlight_cmd_period_t_to_Jplayer_blinkenlight_cmd_period_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_blinkenlight_cmd_period_t_to_buf(Jplayer_blinkenlight_cmd_period_t Jdata) {
    player_blinkenlight_cmd_period_t data = Jplayer_blinkenlight_cmd_period_t_to_player_blinkenlight_cmd_period_t(Jdata);
    return(playercore_java.player_blinkenlight_cmd_period_t_to_buf(data));
  }

  public static Jplayer_blinkenlight_cmd_period_t player_blinkenlight_cmd_period_t_to_Jplayer_blinkenlight_cmd_period_t(player_blinkenlight_cmd_period_t data) {
    Jplayer_blinkenlight_cmd_period_t Jdata = new Jplayer_blinkenlight_cmd_period_t();
    Jdata.period = data.getPeriod();
    return(Jdata);
  }

  public static player_blinkenlight_cmd_period_t Jplayer_blinkenlight_cmd_period_t_to_player_blinkenlight_cmd_period_t(Jplayer_blinkenlight_cmd_period_t Jdata) {
    player_blinkenlight_cmd_period_t data = new player_blinkenlight_cmd_period_t();
    data.setPeriod(Jdata.period);
    return(data);
  }

  public static Jplayer_blinkenlight_cmd_dutycycle_t buf_to_Jplayer_blinkenlight_cmd_dutycycle_t(SWIGTYPE_p_void buf) {
    player_blinkenlight_cmd_dutycycle_t data = playercore_java.buf_to_player_blinkenlight_cmd_dutycycle_t(buf);
    return(player_blinkenlight_cmd_dutycycle_t_to_Jplayer_blinkenlight_cmd_dutycycle_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_blinkenlight_cmd_dutycycle_t_to_buf(Jplayer_blinkenlight_cmd_dutycycle_t Jdata) {
    player_blinkenlight_cmd_dutycycle_t data = Jplayer_blinkenlight_cmd_dutycycle_t_to_player_blinkenlight_cmd_dutycycle_t(Jdata);
    return(playercore_java.player_blinkenlight_cmd_dutycycle_t_to_buf(data));
  }

  public static Jplayer_blinkenlight_cmd_dutycycle_t player_blinkenlight_cmd_dutycycle_t_to_Jplayer_blinkenlight_cmd_dutycycle_t(player_blinkenlight_cmd_dutycycle_t data) {
    Jplayer_blinkenlight_cmd_dutycycle_t Jdata = new Jplayer_blinkenlight_cmd_dutycycle_t();
    Jdata.dutycycle = data.getDutycycle();
    return(Jdata);
  }

  public static player_blinkenlight_cmd_dutycycle_t Jplayer_blinkenlight_cmd_dutycycle_t_to_player_blinkenlight_cmd_dutycycle_t(Jplayer_blinkenlight_cmd_dutycycle_t Jdata) {
    player_blinkenlight_cmd_dutycycle_t data = new player_blinkenlight_cmd_dutycycle_t();
    data.setDutycycle(Jdata.dutycycle);
    return(data);
  }

  public static Jplayer_blobfinder_blob_t buf_to_Jplayer_blobfinder_blob_t(SWIGTYPE_p_void buf) {
    player_blobfinder_blob_t data = playercore_java.buf_to_player_blobfinder_blob_t(buf);
    return(player_blobfinder_blob_t_to_Jplayer_blobfinder_blob_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_blobfinder_blob_t_to_buf(Jplayer_blobfinder_blob_t Jdata) {
    player_blobfinder_blob_t data = Jplayer_blobfinder_blob_t_to_player_blobfinder_blob_t(Jdata);
    return(playercore_java.player_blobfinder_blob_t_to_buf(data));
  }

  public static Jplayer_blobfinder_blob_t player_blobfinder_blob_t_to_Jplayer_blobfinder_blob_t(player_blobfinder_blob_t data) {
    Jplayer_blobfinder_blob_t Jdata = new Jplayer_blobfinder_blob_t();
    Jdata.id = data.getId();
    Jdata.color = data.getColor();
    Jdata.area = data.getArea();
    Jdata.x = data.getX();
    Jdata.y = data.getY();
    Jdata.left = data.getLeft();
    Jdata.right = data.getRight();
    Jdata.top = data.getTop();
    Jdata.bottom = data.getBottom();
    Jdata.range = data.getRange();
    return(Jdata);
  }

  public static player_blobfinder_blob_t Jplayer_blobfinder_blob_t_to_player_blobfinder_blob_t(Jplayer_blobfinder_blob_t Jdata) {
    player_blobfinder_blob_t data = new player_blobfinder_blob_t();
    data.setId(Jdata.id);
    data.setColor(Jdata.color);
    data.setArea(Jdata.area);
    data.setX(Jdata.x);
    data.setY(Jdata.y);
    data.setLeft(Jdata.left);
    data.setRight(Jdata.right);
    data.setTop(Jdata.top);
    data.setBottom(Jdata.bottom);
    data.setRange(Jdata.range);
    return(data);
  }

  public static Jplayer_blobfinder_data_t buf_to_Jplayer_blobfinder_data_t(SWIGTYPE_p_void buf) {
    player_blobfinder_data_t data = playercore_java.buf_to_player_blobfinder_data_t(buf);
    return(player_blobfinder_data_t_to_Jplayer_blobfinder_data_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_blobfinder_data_t_to_buf(Jplayer_blobfinder_data_t Jdata) {
    player_blobfinder_data_t data = Jplayer_blobfinder_data_t_to_player_blobfinder_data_t(Jdata);
    return(playercore_java.player_blobfinder_data_t_to_buf(data));
  }

  public static Jplayer_blobfinder_data_t player_blobfinder_data_t_to_Jplayer_blobfinder_data_t(player_blobfinder_data_t data) {
    Jplayer_blobfinder_data_t Jdata = new Jplayer_blobfinder_data_t();
    Jdata.width = data.getWidth();
    Jdata.height = data.getHeight();
    Jdata.blobs_count = data.getBlobs_count();
    {
      player_blobfinder_blob_t foo[] = data.getBlobs();
      for(int i=0;i<playercore_javaConstants.PLAYER_BLOBFINDER_MAX_BLOBS;i++)
        Jdata.blobs[i] = player_blobfinder_blob_t_to_Jplayer_blobfinder_blob_t(foo[i]);
    }
    return(Jdata);
  }

  public static player_blobfinder_data_t Jplayer_blobfinder_data_t_to_player_blobfinder_data_t(Jplayer_blobfinder_data_t Jdata) {
    player_blobfinder_data_t data = new player_blobfinder_data_t();
    data.setWidth(Jdata.width);
    data.setHeight(Jdata.height);
    data.setBlobs_count(Jdata.blobs_count);
    {
      player_blobfinder_blob_t foo[] = new player_blobfinder_blob_t[playercore_javaConstants.PLAYER_BLOBFINDER_MAX_BLOBS];
      for(int i=0;i<playercore_javaConstants.PLAYER_BLOBFINDER_MAX_BLOBS;i++)
        foo[i] = Jplayer_blobfinder_blob_t_to_player_blobfinder_blob_t(Jdata.blobs[i]);
      data.setBlobs(foo);
    }
    return(data);
  }

  public static Jplayer_blobfinder_color_config_t buf_to_Jplayer_blobfinder_color_config_t(SWIGTYPE_p_void buf) {
    player_blobfinder_color_config_t data = playercore_java.buf_to_player_blobfinder_color_config_t(buf);
    return(player_blobfinder_color_config_t_to_Jplayer_blobfinder_color_config_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_blobfinder_color_config_t_to_buf(Jplayer_blobfinder_color_config_t Jdata) {
    player_blobfinder_color_config_t data = Jplayer_blobfinder_color_config_t_to_player_blobfinder_color_config_t(Jdata);
    return(playercore_java.player_blobfinder_color_config_t_to_buf(data));
  }

  public static Jplayer_blobfinder_color_config_t player_blobfinder_color_config_t_to_Jplayer_blobfinder_color_config_t(player_blobfinder_color_config_t data) {
    Jplayer_blobfinder_color_config_t Jdata = new Jplayer_blobfinder_color_config_t();
    Jdata.rmin = data.getRmin();
    Jdata.rmax = data.getRmax();
    Jdata.gmin = data.getGmin();
    Jdata.gmax = data.getGmax();
    Jdata.bmin = data.getBmin();
    Jdata.bmax = data.getBmax();
    return(Jdata);
  }

  public static player_blobfinder_color_config_t Jplayer_blobfinder_color_config_t_to_player_blobfinder_color_config_t(Jplayer_blobfinder_color_config_t Jdata) {
    player_blobfinder_color_config_t data = new player_blobfinder_color_config_t();
    data.setRmin(Jdata.rmin);
    data.setRmax(Jdata.rmax);
    data.setGmin(Jdata.gmin);
    data.setGmax(Jdata.gmax);
    data.setBmin(Jdata.bmin);
    data.setBmax(Jdata.bmax);
    return(data);
  }

  public static Jplayer_blobfinder_imager_config_t buf_to_Jplayer_blobfinder_imager_config_t(SWIGTYPE_p_void buf) {
    player_blobfinder_imager_config_t data = playercore_java.buf_to_player_blobfinder_imager_config_t(buf);
    return(player_blobfinder_imager_config_t_to_Jplayer_blobfinder_imager_config_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_blobfinder_imager_config_t_to_buf(Jplayer_blobfinder_imager_config_t Jdata) {
    player_blobfinder_imager_config_t data = Jplayer_blobfinder_imager_config_t_to_player_blobfinder_imager_config_t(Jdata);
    return(playercore_java.player_blobfinder_imager_config_t_to_buf(data));
  }

  public static Jplayer_blobfinder_imager_config_t player_blobfinder_imager_config_t_to_Jplayer_blobfinder_imager_config_t(player_blobfinder_imager_config_t data) {
    Jplayer_blobfinder_imager_config_t Jdata = new Jplayer_blobfinder_imager_config_t();
    Jdata.brightness = data.getBrightness();
    Jdata.contrast = data.getContrast();
    Jdata.colormode = data.getColormode();
    Jdata.autogain = data.getAutogain();
    return(Jdata);
  }

  public static player_blobfinder_imager_config_t Jplayer_blobfinder_imager_config_t_to_player_blobfinder_imager_config_t(Jplayer_blobfinder_imager_config_t Jdata) {
    player_blobfinder_imager_config_t data = new player_blobfinder_imager_config_t();
    data.setBrightness(Jdata.brightness);
    data.setContrast(Jdata.contrast);
    data.setColormode(Jdata.colormode);
    data.setAutogain(Jdata.autogain);
    return(data);
  }

  public static Jplayer_bumper_data_t buf_to_Jplayer_bumper_data_t(SWIGTYPE_p_void buf) {
    player_bumper_data_t data = playercore_java.buf_to_player_bumper_data_t(buf);
    return(player_bumper_data_t_to_Jplayer_bumper_data_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_bumper_data_t_to_buf(Jplayer_bumper_data_t Jdata) {
    player_bumper_data_t data = Jplayer_bumper_data_t_to_player_bumper_data_t(Jdata);
    return(playercore_java.player_bumper_data_t_to_buf(data));
  }

  public static Jplayer_bumper_data_t player_bumper_data_t_to_Jplayer_bumper_data_t(player_bumper_data_t data) {
    Jplayer_bumper_data_t Jdata = new Jplayer_bumper_data_t();
    Jdata.bumpers_count = data.getBumpers_count();
    Jdata.bumpers = data.getBumpers();
    return(Jdata);
  }

  public static player_bumper_data_t Jplayer_bumper_data_t_to_player_bumper_data_t(Jplayer_bumper_data_t Jdata) {
    player_bumper_data_t data = new player_bumper_data_t();
    data.setBumpers_count(Jdata.bumpers_count);
    data.setBumpers(Jdata.bumpers);
    return(data);
  }

  public static Jplayer_bumper_define_t buf_to_Jplayer_bumper_define_t(SWIGTYPE_p_void buf) {
    player_bumper_define_t data = playercore_java.buf_to_player_bumper_define_t(buf);
    return(player_bumper_define_t_to_Jplayer_bumper_define_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_bumper_define_t_to_buf(Jplayer_bumper_define_t Jdata) {
    player_bumper_define_t data = Jplayer_bumper_define_t_to_player_bumper_define_t(Jdata);
    return(playercore_java.player_bumper_define_t_to_buf(data));
  }

  public static Jplayer_bumper_define_t player_bumper_define_t_to_Jplayer_bumper_define_t(player_bumper_define_t data) {
    Jplayer_bumper_define_t Jdata = new Jplayer_bumper_define_t();
    Jdata.pose = player_pose_t_to_Jplayer_pose_t(data.getPose());
    Jdata.length = data.getLength();
    Jdata.radius = data.getRadius();
    return(Jdata);
  }

  public static player_bumper_define_t Jplayer_bumper_define_t_to_player_bumper_define_t(Jplayer_bumper_define_t Jdata) {
    player_bumper_define_t data = new player_bumper_define_t();
    data.setPose(Jplayer_pose_t_to_player_pose_t(Jdata.pose));
    data.setLength(Jdata.length);
    data.setRadius(Jdata.radius);
    return(data);
  }

  public static Jplayer_bumper_geom_t buf_to_Jplayer_bumper_geom_t(SWIGTYPE_p_void buf) {
    player_bumper_geom_t data = playercore_java.buf_to_player_bumper_geom_t(buf);
    return(player_bumper_geom_t_to_Jplayer_bumper_geom_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_bumper_geom_t_to_buf(Jplayer_bumper_geom_t Jdata) {
    player_bumper_geom_t data = Jplayer_bumper_geom_t_to_player_bumper_geom_t(Jdata);
    return(playercore_java.player_bumper_geom_t_to_buf(data));
  }

  public static Jplayer_bumper_geom_t player_bumper_geom_t_to_Jplayer_bumper_geom_t(player_bumper_geom_t data) {
    Jplayer_bumper_geom_t Jdata = new Jplayer_bumper_geom_t();
    Jdata.bumper_def_count = data.getBumper_def_count();
    {
      player_bumper_define_t foo[] = data.getBumper_def();
      for(int i=0;i<playercore_javaConstants.PLAYER_BUMPER_MAX_SAMPLES;i++)
        Jdata.bumper_def[i] = player_bumper_define_t_to_Jplayer_bumper_define_t(foo[i]);
    }
    return(Jdata);
  }

  public static player_bumper_geom_t Jplayer_bumper_geom_t_to_player_bumper_geom_t(Jplayer_bumper_geom_t Jdata) {
    player_bumper_geom_t data = new player_bumper_geom_t();
    data.setBumper_def_count(Jdata.bumper_def_count);
    {
      player_bumper_define_t foo[] = new player_bumper_define_t[playercore_javaConstants.PLAYER_BUMPER_MAX_SAMPLES];
      for(int i=0;i<playercore_javaConstants.PLAYER_BUMPER_MAX_SAMPLES;i++)
        foo[i] = Jplayer_bumper_define_t_to_player_bumper_define_t(Jdata.bumper_def[i]);
      data.setBumper_def(foo);
    }
    return(data);
  }

  public static Jplayer_camera_data_t buf_to_Jplayer_camera_data_t(SWIGTYPE_p_void buf) {
    player_camera_data_t data = playercore_java.buf_to_player_camera_data_t(buf);
    return(player_camera_data_t_to_Jplayer_camera_data_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_camera_data_t_to_buf(Jplayer_camera_data_t Jdata) {
    player_camera_data_t data = Jplayer_camera_data_t_to_player_camera_data_t(Jdata);
    return(playercore_java.player_camera_data_t_to_buf(data));
  }

  public static Jplayer_camera_data_t player_camera_data_t_to_Jplayer_camera_data_t(player_camera_data_t data) {
    Jplayer_camera_data_t Jdata = new Jplayer_camera_data_t();
    Jdata.width = data.getWidth();
    Jdata.height = data.getHeight();
    Jdata.bpp = data.getBpp();
    Jdata.format = data.getFormat();
    Jdata.fdiv = data.getFdiv();
    Jdata.compression = data.getCompression();
    Jdata.image_count = data.getImage_count();
    Jdata.image = data.getImage();
    return(Jdata);
  }

  public static player_camera_data_t Jplayer_camera_data_t_to_player_camera_data_t(Jplayer_camera_data_t Jdata) {
    player_camera_data_t data = new player_camera_data_t();
    data.setWidth(Jdata.width);
    data.setHeight(Jdata.height);
    data.setBpp(Jdata.bpp);
    data.setFormat(Jdata.format);
    data.setFdiv(Jdata.fdiv);
    data.setCompression(Jdata.compression);
    data.setImage_count(Jdata.image_count);
    data.setImage(Jdata.image);
    return(data);
  }

  public static Jplayer_dio_data_t buf_to_Jplayer_dio_data_t(SWIGTYPE_p_void buf) {
    player_dio_data_t data = playercore_java.buf_to_player_dio_data_t(buf);
    return(player_dio_data_t_to_Jplayer_dio_data_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_dio_data_t_to_buf(Jplayer_dio_data_t Jdata) {
    player_dio_data_t data = Jplayer_dio_data_t_to_player_dio_data_t(Jdata);
    return(playercore_java.player_dio_data_t_to_buf(data));
  }

  public static Jplayer_dio_data_t player_dio_data_t_to_Jplayer_dio_data_t(player_dio_data_t data) {
    Jplayer_dio_data_t Jdata = new Jplayer_dio_data_t();
    Jdata.count = data.getCount();
    Jdata.digin = data.getDigin();
    return(Jdata);
  }

  public static player_dio_data_t Jplayer_dio_data_t_to_player_dio_data_t(Jplayer_dio_data_t Jdata) {
    player_dio_data_t data = new player_dio_data_t();
    data.setCount(Jdata.count);
    data.setDigin(Jdata.digin);
    return(data);
  }

  public static Jplayer_dio_cmd_t buf_to_Jplayer_dio_cmd_t(SWIGTYPE_p_void buf) {
    player_dio_cmd_t data = playercore_java.buf_to_player_dio_cmd_t(buf);
    return(player_dio_cmd_t_to_Jplayer_dio_cmd_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_dio_cmd_t_to_buf(Jplayer_dio_cmd_t Jdata) {
    player_dio_cmd_t data = Jplayer_dio_cmd_t_to_player_dio_cmd_t(Jdata);
    return(playercore_java.player_dio_cmd_t_to_buf(data));
  }

  public static Jplayer_dio_cmd_t player_dio_cmd_t_to_Jplayer_dio_cmd_t(player_dio_cmd_t data) {
    Jplayer_dio_cmd_t Jdata = new Jplayer_dio_cmd_t();
    Jdata.count = data.getCount();
    Jdata.digout = data.getDigout();
    return(Jdata);
  }

  public static player_dio_cmd_t Jplayer_dio_cmd_t_to_player_dio_cmd_t(Jplayer_dio_cmd_t Jdata) {
    player_dio_cmd_t data = new player_dio_cmd_t();
    data.setCount(Jdata.count);
    data.setDigout(Jdata.digout);
    return(data);
  }

  public static Jplayer_energy_data_t buf_to_Jplayer_energy_data_t(SWIGTYPE_p_void buf) {
    player_energy_data_t data = playercore_java.buf_to_player_energy_data_t(buf);
    return(player_energy_data_t_to_Jplayer_energy_data_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_energy_data_t_to_buf(Jplayer_energy_data_t Jdata) {
    player_energy_data_t data = Jplayer_energy_data_t_to_player_energy_data_t(Jdata);
    return(playercore_java.player_energy_data_t_to_buf(data));
  }

  public static Jplayer_energy_data_t player_energy_data_t_to_Jplayer_energy_data_t(player_energy_data_t data) {
    Jplayer_energy_data_t Jdata = new Jplayer_energy_data_t();
    Jdata.joules = data.getJoules();
    Jdata.watts = data.getWatts();
    Jdata.charging = data.getCharging();
    return(Jdata);
  }

  public static player_energy_data_t Jplayer_energy_data_t_to_player_energy_data_t(Jplayer_energy_data_t Jdata) {
    player_energy_data_t data = new player_energy_data_t();
    data.setJoules(Jdata.joules);
    data.setWatts(Jdata.watts);
    data.setCharging(Jdata.charging);
    return(data);
  }

  public static Jplayer_energy_chargepolicy_config_t buf_to_Jplayer_energy_chargepolicy_config_t(SWIGTYPE_p_void buf) {
    player_energy_chargepolicy_config_t data = playercore_java.buf_to_player_energy_chargepolicy_config_t(buf);
    return(player_energy_chargepolicy_config_t_to_Jplayer_energy_chargepolicy_config_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_energy_chargepolicy_config_t_to_buf(Jplayer_energy_chargepolicy_config_t Jdata) {
    player_energy_chargepolicy_config_t data = Jplayer_energy_chargepolicy_config_t_to_player_energy_chargepolicy_config_t(Jdata);
    return(playercore_java.player_energy_chargepolicy_config_t_to_buf(data));
  }

  public static Jplayer_energy_chargepolicy_config_t player_energy_chargepolicy_config_t_to_Jplayer_energy_chargepolicy_config_t(player_energy_chargepolicy_config_t data) {
    Jplayer_energy_chargepolicy_config_t Jdata = new Jplayer_energy_chargepolicy_config_t();
    Jdata.enable_input = data.getEnable_input();
    Jdata.enable_output = data.getEnable_output();
    return(Jdata);
  }

  public static player_energy_chargepolicy_config_t Jplayer_energy_chargepolicy_config_t_to_player_energy_chargepolicy_config_t(Jplayer_energy_chargepolicy_config_t Jdata) {
    player_energy_chargepolicy_config_t data = new player_energy_chargepolicy_config_t();
    data.setEnable_input(Jdata.enable_input);
    data.setEnable_output(Jdata.enable_output);
    return(data);
  }

  public static Jplayer_fiducial_item_t buf_to_Jplayer_fiducial_item_t(SWIGTYPE_p_void buf) {
    player_fiducial_item_t data = playercore_java.buf_to_player_fiducial_item_t(buf);
    return(player_fiducial_item_t_to_Jplayer_fiducial_item_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_fiducial_item_t_to_buf(Jplayer_fiducial_item_t Jdata) {
    player_fiducial_item_t data = Jplayer_fiducial_item_t_to_player_fiducial_item_t(Jdata);
    return(playercore_java.player_fiducial_item_t_to_buf(data));
  }

  public static Jplayer_fiducial_item_t player_fiducial_item_t_to_Jplayer_fiducial_item_t(player_fiducial_item_t data) {
    Jplayer_fiducial_item_t Jdata = new Jplayer_fiducial_item_t();
    Jdata.id = data.getId();
    Jdata.pose = player_pose3d_t_to_Jplayer_pose3d_t(data.getPose());
    Jdata.upose = player_pose3d_t_to_Jplayer_pose3d_t(data.getUpose());
    return(Jdata);
  }

  public static player_fiducial_item_t Jplayer_fiducial_item_t_to_player_fiducial_item_t(Jplayer_fiducial_item_t Jdata) {
    player_fiducial_item_t data = new player_fiducial_item_t();
    data.setId(Jdata.id);
    data.setPose(Jplayer_pose3d_t_to_player_pose3d_t(Jdata.pose));
    data.setUpose(Jplayer_pose3d_t_to_player_pose3d_t(Jdata.upose));
    return(data);
  }

  public static Jplayer_fiducial_data_t buf_to_Jplayer_fiducial_data_t(SWIGTYPE_p_void buf) {
    player_fiducial_data_t data = playercore_java.buf_to_player_fiducial_data_t(buf);
    return(player_fiducial_data_t_to_Jplayer_fiducial_data_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_fiducial_data_t_to_buf(Jplayer_fiducial_data_t Jdata) {
    player_fiducial_data_t data = Jplayer_fiducial_data_t_to_player_fiducial_data_t(Jdata);
    return(playercore_java.player_fiducial_data_t_to_buf(data));
  }

  public static Jplayer_fiducial_data_t player_fiducial_data_t_to_Jplayer_fiducial_data_t(player_fiducial_data_t data) {
    Jplayer_fiducial_data_t Jdata = new Jplayer_fiducial_data_t();
    Jdata.fiducials_count = data.getFiducials_count();
    {
      player_fiducial_item_t foo[] = data.getFiducials();
      for(int i=0;i<playercore_javaConstants.PLAYER_FIDUCIAL_MAX_SAMPLES;i++)
        Jdata.fiducials[i] = player_fiducial_item_t_to_Jplayer_fiducial_item_t(foo[i]);
    }
    return(Jdata);
  }

  public static player_fiducial_data_t Jplayer_fiducial_data_t_to_player_fiducial_data_t(Jplayer_fiducial_data_t Jdata) {
    player_fiducial_data_t data = new player_fiducial_data_t();
    data.setFiducials_count(Jdata.fiducials_count);
    {
      player_fiducial_item_t foo[] = new player_fiducial_item_t[playercore_javaConstants.PLAYER_FIDUCIAL_MAX_SAMPLES];
      for(int i=0;i<playercore_javaConstants.PLAYER_FIDUCIAL_MAX_SAMPLES;i++)
        foo[i] = Jplayer_fiducial_item_t_to_player_fiducial_item_t(Jdata.fiducials[i]);
      data.setFiducials(foo);
    }
    return(data);
  }

  public static Jplayer_fiducial_geom_t buf_to_Jplayer_fiducial_geom_t(SWIGTYPE_p_void buf) {
    player_fiducial_geom_t data = playercore_java.buf_to_player_fiducial_geom_t(buf);
    return(player_fiducial_geom_t_to_Jplayer_fiducial_geom_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_fiducial_geom_t_to_buf(Jplayer_fiducial_geom_t Jdata) {
    player_fiducial_geom_t data = Jplayer_fiducial_geom_t_to_player_fiducial_geom_t(Jdata);
    return(playercore_java.player_fiducial_geom_t_to_buf(data));
  }

  public static Jplayer_fiducial_geom_t player_fiducial_geom_t_to_Jplayer_fiducial_geom_t(player_fiducial_geom_t data) {
    Jplayer_fiducial_geom_t Jdata = new Jplayer_fiducial_geom_t();
    Jdata.pose = player_pose_t_to_Jplayer_pose_t(data.getPose());
    Jdata.size = player_bbox_t_to_Jplayer_bbox_t(data.getSize());
    Jdata.fiducial_size = player_bbox_t_to_Jplayer_bbox_t(data.getFiducial_size());
    return(Jdata);
  }

  public static player_fiducial_geom_t Jplayer_fiducial_geom_t_to_player_fiducial_geom_t(Jplayer_fiducial_geom_t Jdata) {
    player_fiducial_geom_t data = new player_fiducial_geom_t();
    data.setPose(Jplayer_pose_t_to_player_pose_t(Jdata.pose));
    data.setSize(Jplayer_bbox_t_to_player_bbox_t(Jdata.size));
    data.setFiducial_size(Jplayer_bbox_t_to_player_bbox_t(Jdata.fiducial_size));
    return(data);
  }

  public static Jplayer_fiducial_fov_t buf_to_Jplayer_fiducial_fov_t(SWIGTYPE_p_void buf) {
    player_fiducial_fov_t data = playercore_java.buf_to_player_fiducial_fov_t(buf);
    return(player_fiducial_fov_t_to_Jplayer_fiducial_fov_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_fiducial_fov_t_to_buf(Jplayer_fiducial_fov_t Jdata) {
    player_fiducial_fov_t data = Jplayer_fiducial_fov_t_to_player_fiducial_fov_t(Jdata);
    return(playercore_java.player_fiducial_fov_t_to_buf(data));
  }

  public static Jplayer_fiducial_fov_t player_fiducial_fov_t_to_Jplayer_fiducial_fov_t(player_fiducial_fov_t data) {
    Jplayer_fiducial_fov_t Jdata = new Jplayer_fiducial_fov_t();
    Jdata.min_range = data.getMin_range();
    Jdata.max_range = data.getMax_range();
    Jdata.view_angle = data.getView_angle();
    return(Jdata);
  }

  public static player_fiducial_fov_t Jplayer_fiducial_fov_t_to_player_fiducial_fov_t(Jplayer_fiducial_fov_t Jdata) {
    player_fiducial_fov_t data = new player_fiducial_fov_t();
    data.setMin_range(Jdata.min_range);
    data.setMax_range(Jdata.max_range);
    data.setView_angle(Jdata.view_angle);
    return(data);
  }

  public static Jplayer_fiducial_id_t buf_to_Jplayer_fiducial_id_t(SWIGTYPE_p_void buf) {
    player_fiducial_id_t data = playercore_java.buf_to_player_fiducial_id_t(buf);
    return(player_fiducial_id_t_to_Jplayer_fiducial_id_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_fiducial_id_t_to_buf(Jplayer_fiducial_id_t Jdata) {
    player_fiducial_id_t data = Jplayer_fiducial_id_t_to_player_fiducial_id_t(Jdata);
    return(playercore_java.player_fiducial_id_t_to_buf(data));
  }

  public static Jplayer_fiducial_id_t player_fiducial_id_t_to_Jplayer_fiducial_id_t(player_fiducial_id_t data) {
    Jplayer_fiducial_id_t Jdata = new Jplayer_fiducial_id_t();
    Jdata.id = data.getId();
    return(Jdata);
  }

  public static player_fiducial_id_t Jplayer_fiducial_id_t_to_player_fiducial_id_t(Jplayer_fiducial_id_t Jdata) {
    player_fiducial_id_t data = new player_fiducial_id_t();
    data.setId(Jdata.id);
    return(data);
  }

  public static Jplayer_gps_data_t buf_to_Jplayer_gps_data_t(SWIGTYPE_p_void buf) {
    player_gps_data_t data = playercore_java.buf_to_player_gps_data_t(buf);
    return(player_gps_data_t_to_Jplayer_gps_data_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_gps_data_t_to_buf(Jplayer_gps_data_t Jdata) {
    player_gps_data_t data = Jplayer_gps_data_t_to_player_gps_data_t(Jdata);
    return(playercore_java.player_gps_data_t_to_buf(data));
  }

  public static Jplayer_gps_data_t player_gps_data_t_to_Jplayer_gps_data_t(player_gps_data_t data) {
    Jplayer_gps_data_t Jdata = new Jplayer_gps_data_t();
    Jdata.time_sec = data.getTime_sec();
    Jdata.time_usec = data.getTime_usec();
    Jdata.latitude = data.getLatitude();
    Jdata.longitude = data.getLongitude();
    Jdata.altitude = data.getAltitude();
    Jdata.utm_e = data.getUtm_e();
    Jdata.utm_n = data.getUtm_n();
    Jdata.quality = data.getQuality();
    Jdata.num_sats = data.getNum_sats();
    Jdata.hdop = data.getHdop();
    Jdata.vdop = data.getVdop();
    Jdata.err_horz = data.getErr_horz();
    Jdata.err_vert = data.getErr_vert();
    return(Jdata);
  }

  public static player_gps_data_t Jplayer_gps_data_t_to_player_gps_data_t(Jplayer_gps_data_t Jdata) {
    player_gps_data_t data = new player_gps_data_t();
    data.setTime_sec(Jdata.time_sec);
    data.setTime_usec(Jdata.time_usec);
    data.setLatitude(Jdata.latitude);
    data.setLongitude(Jdata.longitude);
    data.setAltitude(Jdata.altitude);
    data.setUtm_e(Jdata.utm_e);
    data.setUtm_n(Jdata.utm_n);
    data.setQuality(Jdata.quality);
    data.setNum_sats(Jdata.num_sats);
    data.setHdop(Jdata.hdop);
    data.setVdop(Jdata.vdop);
    data.setErr_horz(Jdata.err_horz);
    data.setErr_vert(Jdata.err_vert);
    return(data);
  }

  public static Jplayer_graphics2d_cmd_points_t buf_to_Jplayer_graphics2d_cmd_points_t(SWIGTYPE_p_void buf) {
    player_graphics2d_cmd_points_t data = playercore_java.buf_to_player_graphics2d_cmd_points_t(buf);
    return(player_graphics2d_cmd_points_t_to_Jplayer_graphics2d_cmd_points_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_graphics2d_cmd_points_t_to_buf(Jplayer_graphics2d_cmd_points_t Jdata) {
    player_graphics2d_cmd_points_t data = Jplayer_graphics2d_cmd_points_t_to_player_graphics2d_cmd_points_t(Jdata);
    return(playercore_java.player_graphics2d_cmd_points_t_to_buf(data));
  }

  public static Jplayer_graphics2d_cmd_points_t player_graphics2d_cmd_points_t_to_Jplayer_graphics2d_cmd_points_t(player_graphics2d_cmd_points_t data) {
    Jplayer_graphics2d_cmd_points_t Jdata = new Jplayer_graphics2d_cmd_points_t();
    Jdata.count = data.getCount();
    {
      player_point_2d_t foo[] = data.getPoints();
      for(int i=0;i<playercore_javaConstants.PLAYER_GRAPHICS2D_MAX_POINTS;i++)
        Jdata.points[i] = player_point_2d_t_to_Jplayer_point_2d_t(foo[i]);
    }
    Jdata.color = player_color_t_to_Jplayer_color_t(data.getColor());
    return(Jdata);
  }

  public static player_graphics2d_cmd_points_t Jplayer_graphics2d_cmd_points_t_to_player_graphics2d_cmd_points_t(Jplayer_graphics2d_cmd_points_t Jdata) {
    player_graphics2d_cmd_points_t data = new player_graphics2d_cmd_points_t();
    data.setCount(Jdata.count);
    {
      player_point_2d_t foo[] = new player_point_2d_t[playercore_javaConstants.PLAYER_GRAPHICS2D_MAX_POINTS];
      for(int i=0;i<playercore_javaConstants.PLAYER_GRAPHICS2D_MAX_POINTS;i++)
        foo[i] = Jplayer_point_2d_t_to_player_point_2d_t(Jdata.points[i]);
      data.setPoints(foo);
    }
    data.setColor(Jplayer_color_t_to_player_color_t(Jdata.color));
    return(data);
  }

  public static Jplayer_graphics2d_cmd_polyline_t buf_to_Jplayer_graphics2d_cmd_polyline_t(SWIGTYPE_p_void buf) {
    player_graphics2d_cmd_polyline_t data = playercore_java.buf_to_player_graphics2d_cmd_polyline_t(buf);
    return(player_graphics2d_cmd_polyline_t_to_Jplayer_graphics2d_cmd_polyline_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_graphics2d_cmd_polyline_t_to_buf(Jplayer_graphics2d_cmd_polyline_t Jdata) {
    player_graphics2d_cmd_polyline_t data = Jplayer_graphics2d_cmd_polyline_t_to_player_graphics2d_cmd_polyline_t(Jdata);
    return(playercore_java.player_graphics2d_cmd_polyline_t_to_buf(data));
  }

  public static Jplayer_graphics2d_cmd_polyline_t player_graphics2d_cmd_polyline_t_to_Jplayer_graphics2d_cmd_polyline_t(player_graphics2d_cmd_polyline_t data) {
    Jplayer_graphics2d_cmd_polyline_t Jdata = new Jplayer_graphics2d_cmd_polyline_t();
    Jdata.count = data.getCount();
    {
      player_point_2d_t foo[] = data.getPoints();
      for(int i=0;i<playercore_javaConstants.PLAYER_GRAPHICS2D_MAX_POINTS;i++)
        Jdata.points[i] = player_point_2d_t_to_Jplayer_point_2d_t(foo[i]);
    }
    Jdata.color = player_color_t_to_Jplayer_color_t(data.getColor());
    return(Jdata);
  }

  public static player_graphics2d_cmd_polyline_t Jplayer_graphics2d_cmd_polyline_t_to_player_graphics2d_cmd_polyline_t(Jplayer_graphics2d_cmd_polyline_t Jdata) {
    player_graphics2d_cmd_polyline_t data = new player_graphics2d_cmd_polyline_t();
    data.setCount(Jdata.count);
    {
      player_point_2d_t foo[] = new player_point_2d_t[playercore_javaConstants.PLAYER_GRAPHICS2D_MAX_POINTS];
      for(int i=0;i<playercore_javaConstants.PLAYER_GRAPHICS2D_MAX_POINTS;i++)
        foo[i] = Jplayer_point_2d_t_to_player_point_2d_t(Jdata.points[i]);
      data.setPoints(foo);
    }
    data.setColor(Jplayer_color_t_to_player_color_t(Jdata.color));
    return(data);
  }

  public static Jplayer_graphics2d_cmd_polygon_t buf_to_Jplayer_graphics2d_cmd_polygon_t(SWIGTYPE_p_void buf) {
    player_graphics2d_cmd_polygon_t data = playercore_java.buf_to_player_graphics2d_cmd_polygon_t(buf);
    return(player_graphics2d_cmd_polygon_t_to_Jplayer_graphics2d_cmd_polygon_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_graphics2d_cmd_polygon_t_to_buf(Jplayer_graphics2d_cmd_polygon_t Jdata) {
    player_graphics2d_cmd_polygon_t data = Jplayer_graphics2d_cmd_polygon_t_to_player_graphics2d_cmd_polygon_t(Jdata);
    return(playercore_java.player_graphics2d_cmd_polygon_t_to_buf(data));
  }

  public static Jplayer_graphics2d_cmd_polygon_t player_graphics2d_cmd_polygon_t_to_Jplayer_graphics2d_cmd_polygon_t(player_graphics2d_cmd_polygon_t data) {
    Jplayer_graphics2d_cmd_polygon_t Jdata = new Jplayer_graphics2d_cmd_polygon_t();
    Jdata.count = data.getCount();
    {
      player_point_2d_t foo[] = data.getPoints();
      for(int i=0;i<playercore_javaConstants.PLAYER_GRAPHICS2D_MAX_POINTS;i++)
        Jdata.points[i] = player_point_2d_t_to_Jplayer_point_2d_t(foo[i]);
    }
    Jdata.color = player_color_t_to_Jplayer_color_t(data.getColor());
    Jdata.fill_color = player_color_t_to_Jplayer_color_t(data.getFill_color());
    Jdata.filled = data.getFilled();
    return(Jdata);
  }

  public static player_graphics2d_cmd_polygon_t Jplayer_graphics2d_cmd_polygon_t_to_player_graphics2d_cmd_polygon_t(Jplayer_graphics2d_cmd_polygon_t Jdata) {
    player_graphics2d_cmd_polygon_t data = new player_graphics2d_cmd_polygon_t();
    data.setCount(Jdata.count);
    {
      player_point_2d_t foo[] = new player_point_2d_t[playercore_javaConstants.PLAYER_GRAPHICS2D_MAX_POINTS];
      for(int i=0;i<playercore_javaConstants.PLAYER_GRAPHICS2D_MAX_POINTS;i++)
        foo[i] = Jplayer_point_2d_t_to_player_point_2d_t(Jdata.points[i]);
      data.setPoints(foo);
    }
    data.setColor(Jplayer_color_t_to_player_color_t(Jdata.color));
    data.setFill_color(Jplayer_color_t_to_player_color_t(Jdata.fill_color));
    data.setFilled(Jdata.filled);
    return(data);
  }

  public static Jplayer_graphics3d_cmd_draw_t buf_to_Jplayer_graphics3d_cmd_draw_t(SWIGTYPE_p_void buf) {
    player_graphics3d_cmd_draw_t data = playercore_java.buf_to_player_graphics3d_cmd_draw_t(buf);
    return(player_graphics3d_cmd_draw_t_to_Jplayer_graphics3d_cmd_draw_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_graphics3d_cmd_draw_t_to_buf(Jplayer_graphics3d_cmd_draw_t Jdata) {
    player_graphics3d_cmd_draw_t data = Jplayer_graphics3d_cmd_draw_t_to_player_graphics3d_cmd_draw_t(Jdata);
    return(playercore_java.player_graphics3d_cmd_draw_t_to_buf(data));
  }

  public static Jplayer_graphics3d_cmd_draw_t player_graphics3d_cmd_draw_t_to_Jplayer_graphics3d_cmd_draw_t(player_graphics3d_cmd_draw_t data) {
    Jplayer_graphics3d_cmd_draw_t Jdata = new Jplayer_graphics3d_cmd_draw_t();
    Jdata.draw_mode = data.getDraw_mode();
    Jdata.points_count = data.getPoints_count();
    {
      player_point_3d_t foo[] = data.getPoints();
      for(int i=0;i<playercore_javaConstants.PLAYER_GRAPHICS3D_MAX_POINTS;i++)
        Jdata.points[i] = player_point_3d_t_to_Jplayer_point_3d_t(foo[i]);
    }
    Jdata.color = player_color_t_to_Jplayer_color_t(data.getColor());
    return(Jdata);
  }

  public static player_graphics3d_cmd_draw_t Jplayer_graphics3d_cmd_draw_t_to_player_graphics3d_cmd_draw_t(Jplayer_graphics3d_cmd_draw_t Jdata) {
    player_graphics3d_cmd_draw_t data = new player_graphics3d_cmd_draw_t();
    data.setDraw_mode(Jdata.draw_mode);
    data.setPoints_count(Jdata.points_count);
    {
      player_point_3d_t foo[] = new player_point_3d_t[playercore_javaConstants.PLAYER_GRAPHICS3D_MAX_POINTS];
      for(int i=0;i<playercore_javaConstants.PLAYER_GRAPHICS3D_MAX_POINTS;i++)
        foo[i] = Jplayer_point_3d_t_to_player_point_3d_t(Jdata.points[i]);
      data.setPoints(foo);
    }
    data.setColor(Jplayer_color_t_to_player_color_t(Jdata.color));
    return(data);
  }

  public static Jplayer_gripper_data_t buf_to_Jplayer_gripper_data_t(SWIGTYPE_p_void buf) {
    player_gripper_data_t data = playercore_java.buf_to_player_gripper_data_t(buf);
    return(player_gripper_data_t_to_Jplayer_gripper_data_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_gripper_data_t_to_buf(Jplayer_gripper_data_t Jdata) {
    player_gripper_data_t data = Jplayer_gripper_data_t_to_player_gripper_data_t(Jdata);
    return(playercore_java.player_gripper_data_t_to_buf(data));
  }

  public static Jplayer_gripper_data_t player_gripper_data_t_to_Jplayer_gripper_data_t(player_gripper_data_t data) {
    Jplayer_gripper_data_t Jdata = new Jplayer_gripper_data_t();
    Jdata.state = data.getState();
    Jdata.beams = data.getBeams();
    return(Jdata);
  }

  public static player_gripper_data_t Jplayer_gripper_data_t_to_player_gripper_data_t(Jplayer_gripper_data_t Jdata) {
    player_gripper_data_t data = new player_gripper_data_t();
    data.setState(Jdata.state);
    data.setBeams(Jdata.beams);
    return(data);
  }

  public static Jplayer_gripper_cmd_t buf_to_Jplayer_gripper_cmd_t(SWIGTYPE_p_void buf) {
    player_gripper_cmd_t data = playercore_java.buf_to_player_gripper_cmd_t(buf);
    return(player_gripper_cmd_t_to_Jplayer_gripper_cmd_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_gripper_cmd_t_to_buf(Jplayer_gripper_cmd_t Jdata) {
    player_gripper_cmd_t data = Jplayer_gripper_cmd_t_to_player_gripper_cmd_t(Jdata);
    return(playercore_java.player_gripper_cmd_t_to_buf(data));
  }

  public static Jplayer_gripper_cmd_t player_gripper_cmd_t_to_Jplayer_gripper_cmd_t(player_gripper_cmd_t data) {
    Jplayer_gripper_cmd_t Jdata = new Jplayer_gripper_cmd_t();
    Jdata.cmd = data.getCmd();
    Jdata.arg = data.getArg();
    return(Jdata);
  }

  public static player_gripper_cmd_t Jplayer_gripper_cmd_t_to_player_gripper_cmd_t(Jplayer_gripper_cmd_t Jdata) {
    player_gripper_cmd_t data = new player_gripper_cmd_t();
    data.setCmd(Jdata.cmd);
    data.setArg(Jdata.arg);
    return(data);
  }

  public static Jplayer_gripper_geom_t buf_to_Jplayer_gripper_geom_t(SWIGTYPE_p_void buf) {
    player_gripper_geom_t data = playercore_java.buf_to_player_gripper_geom_t(buf);
    return(player_gripper_geom_t_to_Jplayer_gripper_geom_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_gripper_geom_t_to_buf(Jplayer_gripper_geom_t Jdata) {
    player_gripper_geom_t data = Jplayer_gripper_geom_t_to_player_gripper_geom_t(Jdata);
    return(playercore_java.player_gripper_geom_t_to_buf(data));
  }

  public static Jplayer_gripper_geom_t player_gripper_geom_t_to_Jplayer_gripper_geom_t(player_gripper_geom_t data) {
    Jplayer_gripper_geom_t Jdata = new Jplayer_gripper_geom_t();
    Jdata.pose = player_pose_t_to_Jplayer_pose_t(data.getPose());
    Jdata.size = player_bbox_t_to_Jplayer_bbox_t(data.getSize());
    return(Jdata);
  }

  public static player_gripper_geom_t Jplayer_gripper_geom_t_to_player_gripper_geom_t(Jplayer_gripper_geom_t Jdata) {
    player_gripper_geom_t data = new player_gripper_geom_t();
    data.setPose(Jplayer_pose_t_to_player_pose_t(Jdata.pose));
    data.setSize(Jplayer_bbox_t_to_player_bbox_t(Jdata.size));
    return(data);
  }

  public static Jplayer_ir_data_t buf_to_Jplayer_ir_data_t(SWIGTYPE_p_void buf) {
    player_ir_data_t data = playercore_java.buf_to_player_ir_data_t(buf);
    return(player_ir_data_t_to_Jplayer_ir_data_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_ir_data_t_to_buf(Jplayer_ir_data_t Jdata) {
    player_ir_data_t data = Jplayer_ir_data_t_to_player_ir_data_t(Jdata);
    return(playercore_java.player_ir_data_t_to_buf(data));
  }

  public static Jplayer_ir_data_t player_ir_data_t_to_Jplayer_ir_data_t(player_ir_data_t data) {
    Jplayer_ir_data_t Jdata = new Jplayer_ir_data_t();
    Jdata.voltages_count = data.getVoltages_count();
    Jdata.voltages = data.getVoltages();
    Jdata.ranges_count = data.getRanges_count();
    Jdata.ranges = data.getRanges();
    return(Jdata);
  }

  public static player_ir_data_t Jplayer_ir_data_t_to_player_ir_data_t(Jplayer_ir_data_t Jdata) {
    player_ir_data_t data = new player_ir_data_t();
    data.setVoltages_count(Jdata.voltages_count);
    data.setVoltages(Jdata.voltages);
    data.setRanges_count(Jdata.ranges_count);
    data.setRanges(Jdata.ranges);
    return(data);
  }

  public static Jplayer_ir_pose_t buf_to_Jplayer_ir_pose_t(SWIGTYPE_p_void buf) {
    player_ir_pose_t data = playercore_java.buf_to_player_ir_pose_t(buf);
    return(player_ir_pose_t_to_Jplayer_ir_pose_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_ir_pose_t_to_buf(Jplayer_ir_pose_t Jdata) {
    player_ir_pose_t data = Jplayer_ir_pose_t_to_player_ir_pose_t(Jdata);
    return(playercore_java.player_ir_pose_t_to_buf(data));
  }

  public static Jplayer_ir_pose_t player_ir_pose_t_to_Jplayer_ir_pose_t(player_ir_pose_t data) {
    Jplayer_ir_pose_t Jdata = new Jplayer_ir_pose_t();
    Jdata.poses_count = data.getPoses_count();
    {
      player_pose_t foo[] = data.getPoses();
      for(int i=0;i<playercore_javaConstants.PLAYER_IR_MAX_SAMPLES;i++)
        Jdata.poses[i] = player_pose_t_to_Jplayer_pose_t(foo[i]);
    }
    return(Jdata);
  }

  public static player_ir_pose_t Jplayer_ir_pose_t_to_player_ir_pose_t(Jplayer_ir_pose_t Jdata) {
    player_ir_pose_t data = new player_ir_pose_t();
    data.setPoses_count(Jdata.poses_count);
    {
      player_pose_t foo[] = new player_pose_t[playercore_javaConstants.PLAYER_IR_MAX_SAMPLES];
      for(int i=0;i<playercore_javaConstants.PLAYER_IR_MAX_SAMPLES;i++)
        foo[i] = Jplayer_pose_t_to_player_pose_t(Jdata.poses[i]);
      data.setPoses(foo);
    }
    return(data);
  }

  public static Jplayer_ir_power_req_t buf_to_Jplayer_ir_power_req_t(SWIGTYPE_p_void buf) {
    player_ir_power_req_t data = playercore_java.buf_to_player_ir_power_req_t(buf);
    return(player_ir_power_req_t_to_Jplayer_ir_power_req_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_ir_power_req_t_to_buf(Jplayer_ir_power_req_t Jdata) {
    player_ir_power_req_t data = Jplayer_ir_power_req_t_to_player_ir_power_req_t(Jdata);
    return(playercore_java.player_ir_power_req_t_to_buf(data));
  }

  public static Jplayer_ir_power_req_t player_ir_power_req_t_to_Jplayer_ir_power_req_t(player_ir_power_req_t data) {
    Jplayer_ir_power_req_t Jdata = new Jplayer_ir_power_req_t();
    Jdata.state = data.getState();
    return(Jdata);
  }

  public static player_ir_power_req_t Jplayer_ir_power_req_t_to_player_ir_power_req_t(Jplayer_ir_power_req_t Jdata) {
    player_ir_power_req_t data = new player_ir_power_req_t();
    data.setState(Jdata.state);
    return(data);
  }

  public static Jplayer_joystick_data_t buf_to_Jplayer_joystick_data_t(SWIGTYPE_p_void buf) {
    player_joystick_data_t data = playercore_java.buf_to_player_joystick_data_t(buf);
    return(player_joystick_data_t_to_Jplayer_joystick_data_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_joystick_data_t_to_buf(Jplayer_joystick_data_t Jdata) {
    player_joystick_data_t data = Jplayer_joystick_data_t_to_player_joystick_data_t(Jdata);
    return(playercore_java.player_joystick_data_t_to_buf(data));
  }

  public static Jplayer_joystick_data_t player_joystick_data_t_to_Jplayer_joystick_data_t(player_joystick_data_t data) {
    Jplayer_joystick_data_t Jdata = new Jplayer_joystick_data_t();
    Jdata.xpos = data.getXpos();
    Jdata.ypos = data.getYpos();
    Jdata.xscale = data.getXscale();
    Jdata.yscale = data.getYscale();
    Jdata.buttons = data.getButtons();
    return(Jdata);
  }

  public static player_joystick_data_t Jplayer_joystick_data_t_to_player_joystick_data_t(Jplayer_joystick_data_t Jdata) {
    player_joystick_data_t data = new player_joystick_data_t();
    data.setXpos(Jdata.xpos);
    data.setYpos(Jdata.ypos);
    data.setXscale(Jdata.xscale);
    data.setYscale(Jdata.yscale);
    data.setButtons(Jdata.buttons);
    return(data);
  }

  public static Jplayer_laser_data_t buf_to_Jplayer_laser_data_t(SWIGTYPE_p_void buf) {
    player_laser_data_t data = playercore_java.buf_to_player_laser_data_t(buf);
    return(player_laser_data_t_to_Jplayer_laser_data_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_laser_data_t_to_buf(Jplayer_laser_data_t Jdata) {
    player_laser_data_t data = Jplayer_laser_data_t_to_player_laser_data_t(Jdata);
    return(playercore_java.player_laser_data_t_to_buf(data));
  }

  public static Jplayer_laser_data_t player_laser_data_t_to_Jplayer_laser_data_t(player_laser_data_t data) {
    Jplayer_laser_data_t Jdata = new Jplayer_laser_data_t();
    Jdata.min_angle = data.getMin_angle();
    Jdata.max_angle = data.getMax_angle();
    Jdata.resolution = data.getResolution();
    Jdata.max_range = data.getMax_range();
    Jdata.ranges_count = data.getRanges_count();
    Jdata.ranges = data.getRanges();
    Jdata.intensity_count = data.getIntensity_count();
    Jdata.intensity = data.getIntensity();
    Jdata.id = data.getId();
    return(Jdata);
  }

  public static player_laser_data_t Jplayer_laser_data_t_to_player_laser_data_t(Jplayer_laser_data_t Jdata) {
    player_laser_data_t data = new player_laser_data_t();
    data.setMin_angle(Jdata.min_angle);
    data.setMax_angle(Jdata.max_angle);
    data.setResolution(Jdata.resolution);
    data.setMax_range(Jdata.max_range);
    data.setRanges_count(Jdata.ranges_count);
    data.setRanges(Jdata.ranges);
    data.setIntensity_count(Jdata.intensity_count);
    data.setIntensity(Jdata.intensity);
    data.setId(Jdata.id);
    return(data);
  }

  public static Jplayer_laser_data_scanpose_t buf_to_Jplayer_laser_data_scanpose_t(SWIGTYPE_p_void buf) {
    player_laser_data_scanpose_t data = playercore_java.buf_to_player_laser_data_scanpose_t(buf);
    return(player_laser_data_scanpose_t_to_Jplayer_laser_data_scanpose_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_laser_data_scanpose_t_to_buf(Jplayer_laser_data_scanpose_t Jdata) {
    player_laser_data_scanpose_t data = Jplayer_laser_data_scanpose_t_to_player_laser_data_scanpose_t(Jdata);
    return(playercore_java.player_laser_data_scanpose_t_to_buf(data));
  }

  public static Jplayer_laser_data_scanpose_t player_laser_data_scanpose_t_to_Jplayer_laser_data_scanpose_t(player_laser_data_scanpose_t data) {
    Jplayer_laser_data_scanpose_t Jdata = new Jplayer_laser_data_scanpose_t();
    Jdata.scan = player_laser_data_t_to_Jplayer_laser_data_t(data.getScan());
    Jdata.pose = player_pose_t_to_Jplayer_pose_t(data.getPose());
    return(Jdata);
  }

  public static player_laser_data_scanpose_t Jplayer_laser_data_scanpose_t_to_player_laser_data_scanpose_t(Jplayer_laser_data_scanpose_t Jdata) {
    player_laser_data_scanpose_t data = new player_laser_data_scanpose_t();
    data.setScan(Jplayer_laser_data_t_to_player_laser_data_t(Jdata.scan));
    data.setPose(Jplayer_pose_t_to_player_pose_t(Jdata.pose));
    return(data);
  }

  public static Jplayer_laser_geom_t buf_to_Jplayer_laser_geom_t(SWIGTYPE_p_void buf) {
    player_laser_geom_t data = playercore_java.buf_to_player_laser_geom_t(buf);
    return(player_laser_geom_t_to_Jplayer_laser_geom_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_laser_geom_t_to_buf(Jplayer_laser_geom_t Jdata) {
    player_laser_geom_t data = Jplayer_laser_geom_t_to_player_laser_geom_t(Jdata);
    return(playercore_java.player_laser_geom_t_to_buf(data));
  }

  public static Jplayer_laser_geom_t player_laser_geom_t_to_Jplayer_laser_geom_t(player_laser_geom_t data) {
    Jplayer_laser_geom_t Jdata = new Jplayer_laser_geom_t();
    Jdata.pose = player_pose_t_to_Jplayer_pose_t(data.getPose());
    Jdata.size = player_bbox_t_to_Jplayer_bbox_t(data.getSize());
    return(Jdata);
  }

  public static player_laser_geom_t Jplayer_laser_geom_t_to_player_laser_geom_t(Jplayer_laser_geom_t Jdata) {
    player_laser_geom_t data = new player_laser_geom_t();
    data.setPose(Jplayer_pose_t_to_player_pose_t(Jdata.pose));
    data.setSize(Jplayer_bbox_t_to_player_bbox_t(Jdata.size));
    return(data);
  }

  public static Jplayer_laser_config_t buf_to_Jplayer_laser_config_t(SWIGTYPE_p_void buf) {
    player_laser_config_t data = playercore_java.buf_to_player_laser_config_t(buf);
    return(player_laser_config_t_to_Jplayer_laser_config_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_laser_config_t_to_buf(Jplayer_laser_config_t Jdata) {
    player_laser_config_t data = Jplayer_laser_config_t_to_player_laser_config_t(Jdata);
    return(playercore_java.player_laser_config_t_to_buf(data));
  }

  public static Jplayer_laser_config_t player_laser_config_t_to_Jplayer_laser_config_t(player_laser_config_t data) {
    Jplayer_laser_config_t Jdata = new Jplayer_laser_config_t();
    Jdata.min_angle = data.getMin_angle();
    Jdata.max_angle = data.getMax_angle();
    Jdata.resolution = data.getResolution();
    Jdata.max_range = data.getMax_range();
    Jdata.range_res = data.getRange_res();
    Jdata.intensity = data.getIntensity();
    return(Jdata);
  }

  public static player_laser_config_t Jplayer_laser_config_t_to_player_laser_config_t(Jplayer_laser_config_t Jdata) {
    player_laser_config_t data = new player_laser_config_t();
    data.setMin_angle(Jdata.min_angle);
    data.setMax_angle(Jdata.max_angle);
    data.setResolution(Jdata.resolution);
    data.setMax_range(Jdata.max_range);
    data.setRange_res(Jdata.range_res);
    data.setIntensity(Jdata.intensity);
    return(data);
  }

  public static Jplayer_laser_power_config_t buf_to_Jplayer_laser_power_config_t(SWIGTYPE_p_void buf) {
    player_laser_power_config_t data = playercore_java.buf_to_player_laser_power_config_t(buf);
    return(player_laser_power_config_t_to_Jplayer_laser_power_config_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_laser_power_config_t_to_buf(Jplayer_laser_power_config_t Jdata) {
    player_laser_power_config_t data = Jplayer_laser_power_config_t_to_player_laser_power_config_t(Jdata);
    return(playercore_java.player_laser_power_config_t_to_buf(data));
  }

  public static Jplayer_laser_power_config_t player_laser_power_config_t_to_Jplayer_laser_power_config_t(player_laser_power_config_t data) {
    Jplayer_laser_power_config_t Jdata = new Jplayer_laser_power_config_t();
    Jdata.state = data.getState();
    return(Jdata);
  }

  public static player_laser_power_config_t Jplayer_laser_power_config_t_to_player_laser_power_config_t(Jplayer_laser_power_config_t Jdata) {
    player_laser_power_config_t data = new player_laser_power_config_t();
    data.setState(Jdata.state);
    return(data);
  }

  public static Jplayer_limb_data_t buf_to_Jplayer_limb_data_t(SWIGTYPE_p_void buf) {
    player_limb_data_t data = playercore_java.buf_to_player_limb_data_t(buf);
    return(player_limb_data_t_to_Jplayer_limb_data_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_limb_data_t_to_buf(Jplayer_limb_data_t Jdata) {
    player_limb_data_t data = Jplayer_limb_data_t_to_player_limb_data_t(Jdata);
    return(playercore_java.player_limb_data_t_to_buf(data));
  }

  public static Jplayer_limb_data_t player_limb_data_t_to_Jplayer_limb_data_t(player_limb_data_t data) {
    Jplayer_limb_data_t Jdata = new Jplayer_limb_data_t();
    Jdata.position = player_point_3d_t_to_Jplayer_point_3d_t(data.getPosition());
    Jdata.approach = player_point_3d_t_to_Jplayer_point_3d_t(data.getApproach());
    Jdata.orientation = player_point_3d_t_to_Jplayer_point_3d_t(data.getOrientation());
    Jdata.state = data.getState();
    return(Jdata);
  }

  public static player_limb_data_t Jplayer_limb_data_t_to_player_limb_data_t(Jplayer_limb_data_t Jdata) {
    player_limb_data_t data = new player_limb_data_t();
    data.setPosition(Jplayer_point_3d_t_to_player_point_3d_t(Jdata.position));
    data.setApproach(Jplayer_point_3d_t_to_player_point_3d_t(Jdata.approach));
    data.setOrientation(Jplayer_point_3d_t_to_player_point_3d_t(Jdata.orientation));
    data.setState(Jdata.state);
    return(data);
  }

  public static Jplayer_limb_home_cmd_t buf_to_Jplayer_limb_home_cmd_t(SWIGTYPE_p_void buf) {
    player_limb_home_cmd_t data = playercore_java.buf_to_player_limb_home_cmd_t(buf);
    return(player_limb_home_cmd_t_to_Jplayer_limb_home_cmd_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_limb_home_cmd_t_to_buf(Jplayer_limb_home_cmd_t Jdata) {
    player_limb_home_cmd_t data = Jplayer_limb_home_cmd_t_to_player_limb_home_cmd_t(Jdata);
    return(playercore_java.player_limb_home_cmd_t_to_buf(data));
  }

  public static Jplayer_limb_home_cmd_t player_limb_home_cmd_t_to_Jplayer_limb_home_cmd_t(player_limb_home_cmd_t data) {
    Jplayer_limb_home_cmd_t Jdata = new Jplayer_limb_home_cmd_t();
    return(Jdata);
  }

  public static player_limb_home_cmd_t Jplayer_limb_home_cmd_t_to_player_limb_home_cmd_t(Jplayer_limb_home_cmd_t Jdata) {
    player_limb_home_cmd_t data = new player_limb_home_cmd_t();
    return(data);
  }

  public static Jplayer_limb_stop_cmd_t buf_to_Jplayer_limb_stop_cmd_t(SWIGTYPE_p_void buf) {
    player_limb_stop_cmd_t data = playercore_java.buf_to_player_limb_stop_cmd_t(buf);
    return(player_limb_stop_cmd_t_to_Jplayer_limb_stop_cmd_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_limb_stop_cmd_t_to_buf(Jplayer_limb_stop_cmd_t Jdata) {
    player_limb_stop_cmd_t data = Jplayer_limb_stop_cmd_t_to_player_limb_stop_cmd_t(Jdata);
    return(playercore_java.player_limb_stop_cmd_t_to_buf(data));
  }

  public static Jplayer_limb_stop_cmd_t player_limb_stop_cmd_t_to_Jplayer_limb_stop_cmd_t(player_limb_stop_cmd_t data) {
    Jplayer_limb_stop_cmd_t Jdata = new Jplayer_limb_stop_cmd_t();
    return(Jdata);
  }

  public static player_limb_stop_cmd_t Jplayer_limb_stop_cmd_t_to_player_limb_stop_cmd_t(Jplayer_limb_stop_cmd_t Jdata) {
    player_limb_stop_cmd_t data = new player_limb_stop_cmd_t();
    return(data);
  }

  public static Jplayer_limb_setpose_cmd_t buf_to_Jplayer_limb_setpose_cmd_t(SWIGTYPE_p_void buf) {
    player_limb_setpose_cmd_t data = playercore_java.buf_to_player_limb_setpose_cmd_t(buf);
    return(player_limb_setpose_cmd_t_to_Jplayer_limb_setpose_cmd_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_limb_setpose_cmd_t_to_buf(Jplayer_limb_setpose_cmd_t Jdata) {
    player_limb_setpose_cmd_t data = Jplayer_limb_setpose_cmd_t_to_player_limb_setpose_cmd_t(Jdata);
    return(playercore_java.player_limb_setpose_cmd_t_to_buf(data));
  }

  public static Jplayer_limb_setpose_cmd_t player_limb_setpose_cmd_t_to_Jplayer_limb_setpose_cmd_t(player_limb_setpose_cmd_t data) {
    Jplayer_limb_setpose_cmd_t Jdata = new Jplayer_limb_setpose_cmd_t();
    Jdata.position = player_point_3d_t_to_Jplayer_point_3d_t(data.getPosition());
    Jdata.approach = player_point_3d_t_to_Jplayer_point_3d_t(data.getApproach());
    Jdata.orientation = player_point_3d_t_to_Jplayer_point_3d_t(data.getOrientation());
    return(Jdata);
  }

  public static player_limb_setpose_cmd_t Jplayer_limb_setpose_cmd_t_to_player_limb_setpose_cmd_t(Jplayer_limb_setpose_cmd_t Jdata) {
    player_limb_setpose_cmd_t data = new player_limb_setpose_cmd_t();
    data.setPosition(Jplayer_point_3d_t_to_player_point_3d_t(Jdata.position));
    data.setApproach(Jplayer_point_3d_t_to_player_point_3d_t(Jdata.approach));
    data.setOrientation(Jplayer_point_3d_t_to_player_point_3d_t(Jdata.orientation));
    return(data);
  }

  public static Jplayer_limb_setposition_cmd_t buf_to_Jplayer_limb_setposition_cmd_t(SWIGTYPE_p_void buf) {
    player_limb_setposition_cmd_t data = playercore_java.buf_to_player_limb_setposition_cmd_t(buf);
    return(player_limb_setposition_cmd_t_to_Jplayer_limb_setposition_cmd_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_limb_setposition_cmd_t_to_buf(Jplayer_limb_setposition_cmd_t Jdata) {
    player_limb_setposition_cmd_t data = Jplayer_limb_setposition_cmd_t_to_player_limb_setposition_cmd_t(Jdata);
    return(playercore_java.player_limb_setposition_cmd_t_to_buf(data));
  }

  public static Jplayer_limb_setposition_cmd_t player_limb_setposition_cmd_t_to_Jplayer_limb_setposition_cmd_t(player_limb_setposition_cmd_t data) {
    Jplayer_limb_setposition_cmd_t Jdata = new Jplayer_limb_setposition_cmd_t();
    Jdata.position = player_point_3d_t_to_Jplayer_point_3d_t(data.getPosition());
    return(Jdata);
  }

  public static player_limb_setposition_cmd_t Jplayer_limb_setposition_cmd_t_to_player_limb_setposition_cmd_t(Jplayer_limb_setposition_cmd_t Jdata) {
    player_limb_setposition_cmd_t data = new player_limb_setposition_cmd_t();
    data.setPosition(Jplayer_point_3d_t_to_player_point_3d_t(Jdata.position));
    return(data);
  }

  public static Jplayer_limb_vecmove_cmd_t buf_to_Jplayer_limb_vecmove_cmd_t(SWIGTYPE_p_void buf) {
    player_limb_vecmove_cmd_t data = playercore_java.buf_to_player_limb_vecmove_cmd_t(buf);
    return(player_limb_vecmove_cmd_t_to_Jplayer_limb_vecmove_cmd_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_limb_vecmove_cmd_t_to_buf(Jplayer_limb_vecmove_cmd_t Jdata) {
    player_limb_vecmove_cmd_t data = Jplayer_limb_vecmove_cmd_t_to_player_limb_vecmove_cmd_t(Jdata);
    return(playercore_java.player_limb_vecmove_cmd_t_to_buf(data));
  }

  public static Jplayer_limb_vecmove_cmd_t player_limb_vecmove_cmd_t_to_Jplayer_limb_vecmove_cmd_t(player_limb_vecmove_cmd_t data) {
    Jplayer_limb_vecmove_cmd_t Jdata = new Jplayer_limb_vecmove_cmd_t();
    Jdata.direction = player_point_3d_t_to_Jplayer_point_3d_t(data.getDirection());
    Jdata.length = data.getLength();
    return(Jdata);
  }

  public static player_limb_vecmove_cmd_t Jplayer_limb_vecmove_cmd_t_to_player_limb_vecmove_cmd_t(Jplayer_limb_vecmove_cmd_t Jdata) {
    player_limb_vecmove_cmd_t data = new player_limb_vecmove_cmd_t();
    data.setDirection(Jplayer_point_3d_t_to_player_point_3d_t(Jdata.direction));
    data.setLength(Jdata.length);
    return(data);
  }

  public static Jplayer_limb_power_req_t buf_to_Jplayer_limb_power_req_t(SWIGTYPE_p_void buf) {
    player_limb_power_req_t data = playercore_java.buf_to_player_limb_power_req_t(buf);
    return(player_limb_power_req_t_to_Jplayer_limb_power_req_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_limb_power_req_t_to_buf(Jplayer_limb_power_req_t Jdata) {
    player_limb_power_req_t data = Jplayer_limb_power_req_t_to_player_limb_power_req_t(Jdata);
    return(playercore_java.player_limb_power_req_t_to_buf(data));
  }

  public static Jplayer_limb_power_req_t player_limb_power_req_t_to_Jplayer_limb_power_req_t(player_limb_power_req_t data) {
    Jplayer_limb_power_req_t Jdata = new Jplayer_limb_power_req_t();
    Jdata.value = data.getValue();
    return(Jdata);
  }

  public static player_limb_power_req_t Jplayer_limb_power_req_t_to_player_limb_power_req_t(Jplayer_limb_power_req_t Jdata) {
    player_limb_power_req_t data = new player_limb_power_req_t();
    data.setValue(Jdata.value);
    return(data);
  }

  public static Jplayer_limb_brakes_req_t buf_to_Jplayer_limb_brakes_req_t(SWIGTYPE_p_void buf) {
    player_limb_brakes_req_t data = playercore_java.buf_to_player_limb_brakes_req_t(buf);
    return(player_limb_brakes_req_t_to_Jplayer_limb_brakes_req_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_limb_brakes_req_t_to_buf(Jplayer_limb_brakes_req_t Jdata) {
    player_limb_brakes_req_t data = Jplayer_limb_brakes_req_t_to_player_limb_brakes_req_t(Jdata);
    return(playercore_java.player_limb_brakes_req_t_to_buf(data));
  }

  public static Jplayer_limb_brakes_req_t player_limb_brakes_req_t_to_Jplayer_limb_brakes_req_t(player_limb_brakes_req_t data) {
    Jplayer_limb_brakes_req_t Jdata = new Jplayer_limb_brakes_req_t();
    Jdata.value = data.getValue();
    return(Jdata);
  }

  public static player_limb_brakes_req_t Jplayer_limb_brakes_req_t_to_player_limb_brakes_req_t(Jplayer_limb_brakes_req_t Jdata) {
    player_limb_brakes_req_t data = new player_limb_brakes_req_t();
    data.setValue(Jdata.value);
    return(data);
  }

  public static Jplayer_limb_geom_req_t buf_to_Jplayer_limb_geom_req_t(SWIGTYPE_p_void buf) {
    player_limb_geom_req_t data = playercore_java.buf_to_player_limb_geom_req_t(buf);
    return(player_limb_geom_req_t_to_Jplayer_limb_geom_req_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_limb_geom_req_t_to_buf(Jplayer_limb_geom_req_t Jdata) {
    player_limb_geom_req_t data = Jplayer_limb_geom_req_t_to_player_limb_geom_req_t(Jdata);
    return(playercore_java.player_limb_geom_req_t_to_buf(data));
  }

  public static Jplayer_limb_geom_req_t player_limb_geom_req_t_to_Jplayer_limb_geom_req_t(player_limb_geom_req_t data) {
    Jplayer_limb_geom_req_t Jdata = new Jplayer_limb_geom_req_t();
    Jdata.basePos = player_point_3d_t_to_Jplayer_point_3d_t(data.getBasePos());
    return(Jdata);
  }

  public static player_limb_geom_req_t Jplayer_limb_geom_req_t_to_player_limb_geom_req_t(Jplayer_limb_geom_req_t Jdata) {
    player_limb_geom_req_t data = new player_limb_geom_req_t();
    data.setBasePos(Jplayer_point_3d_t_to_player_point_3d_t(Jdata.basePos));
    return(data);
  }

  public static Jplayer_limb_speed_req_t buf_to_Jplayer_limb_speed_req_t(SWIGTYPE_p_void buf) {
    player_limb_speed_req_t data = playercore_java.buf_to_player_limb_speed_req_t(buf);
    return(player_limb_speed_req_t_to_Jplayer_limb_speed_req_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_limb_speed_req_t_to_buf(Jplayer_limb_speed_req_t Jdata) {
    player_limb_speed_req_t data = Jplayer_limb_speed_req_t_to_player_limb_speed_req_t(Jdata);
    return(playercore_java.player_limb_speed_req_t_to_buf(data));
  }

  public static Jplayer_limb_speed_req_t player_limb_speed_req_t_to_Jplayer_limb_speed_req_t(player_limb_speed_req_t data) {
    Jplayer_limb_speed_req_t Jdata = new Jplayer_limb_speed_req_t();
    Jdata.speed = data.getSpeed();
    return(Jdata);
  }

  public static player_limb_speed_req_t Jplayer_limb_speed_req_t_to_player_limb_speed_req_t(Jplayer_limb_speed_req_t Jdata) {
    player_limb_speed_req_t data = new player_limb_speed_req_t();
    data.setSpeed(Jdata.speed);
    return(data);
  }

  public static Jplayer_localize_hypoth_t buf_to_Jplayer_localize_hypoth_t(SWIGTYPE_p_void buf) {
    player_localize_hypoth_t data = playercore_java.buf_to_player_localize_hypoth_t(buf);
    return(player_localize_hypoth_t_to_Jplayer_localize_hypoth_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_localize_hypoth_t_to_buf(Jplayer_localize_hypoth_t Jdata) {
    player_localize_hypoth_t data = Jplayer_localize_hypoth_t_to_player_localize_hypoth_t(Jdata);
    return(playercore_java.player_localize_hypoth_t_to_buf(data));
  }

  public static Jplayer_localize_hypoth_t player_localize_hypoth_t_to_Jplayer_localize_hypoth_t(player_localize_hypoth_t data) {
    Jplayer_localize_hypoth_t Jdata = new Jplayer_localize_hypoth_t();
    Jdata.mean = player_pose_t_to_Jplayer_pose_t(data.getMean());
    Jdata.cov = data.getCov();
    Jdata.alpha = data.getAlpha();
    return(Jdata);
  }

  public static player_localize_hypoth_t Jplayer_localize_hypoth_t_to_player_localize_hypoth_t(Jplayer_localize_hypoth_t Jdata) {
    player_localize_hypoth_t data = new player_localize_hypoth_t();
    data.setMean(Jplayer_pose_t_to_player_pose_t(Jdata.mean));
    data.setCov(Jdata.cov);
    data.setAlpha(Jdata.alpha);
    return(data);
  }

  public static Jplayer_localize_data_t buf_to_Jplayer_localize_data_t(SWIGTYPE_p_void buf) {
    player_localize_data_t data = playercore_java.buf_to_player_localize_data_t(buf);
    return(player_localize_data_t_to_Jplayer_localize_data_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_localize_data_t_to_buf(Jplayer_localize_data_t Jdata) {
    player_localize_data_t data = Jplayer_localize_data_t_to_player_localize_data_t(Jdata);
    return(playercore_java.player_localize_data_t_to_buf(data));
  }

  public static Jplayer_localize_data_t player_localize_data_t_to_Jplayer_localize_data_t(player_localize_data_t data) {
    Jplayer_localize_data_t Jdata = new Jplayer_localize_data_t();
    Jdata.pending_count = data.getPending_count();
    Jdata.pending_time = data.getPending_time();
    Jdata.hypoths_count = data.getHypoths_count();
    {
      player_localize_hypoth_t foo[] = data.getHypoths();
      for(int i=0;i<playercore_javaConstants.PLAYER_LOCALIZE_MAX_HYPOTHS;i++)
        Jdata.hypoths[i] = player_localize_hypoth_t_to_Jplayer_localize_hypoth_t(foo[i]);
    }
    return(Jdata);
  }

  public static player_localize_data_t Jplayer_localize_data_t_to_player_localize_data_t(Jplayer_localize_data_t Jdata) {
    player_localize_data_t data = new player_localize_data_t();
    data.setPending_count(Jdata.pending_count);
    data.setPending_time(Jdata.pending_time);
    data.setHypoths_count(Jdata.hypoths_count);
    {
      player_localize_hypoth_t foo[] = new player_localize_hypoth_t[playercore_javaConstants.PLAYER_LOCALIZE_MAX_HYPOTHS];
      for(int i=0;i<playercore_javaConstants.PLAYER_LOCALIZE_MAX_HYPOTHS;i++)
        foo[i] = Jplayer_localize_hypoth_t_to_player_localize_hypoth_t(Jdata.hypoths[i]);
      data.setHypoths(foo);
    }
    return(data);
  }

  public static Jplayer_localize_set_pose_t buf_to_Jplayer_localize_set_pose_t(SWIGTYPE_p_void buf) {
    player_localize_set_pose_t data = playercore_java.buf_to_player_localize_set_pose_t(buf);
    return(player_localize_set_pose_t_to_Jplayer_localize_set_pose_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_localize_set_pose_t_to_buf(Jplayer_localize_set_pose_t Jdata) {
    player_localize_set_pose_t data = Jplayer_localize_set_pose_t_to_player_localize_set_pose_t(Jdata);
    return(playercore_java.player_localize_set_pose_t_to_buf(data));
  }

  public static Jplayer_localize_set_pose_t player_localize_set_pose_t_to_Jplayer_localize_set_pose_t(player_localize_set_pose_t data) {
    Jplayer_localize_set_pose_t Jdata = new Jplayer_localize_set_pose_t();
    Jdata.mean = player_pose_t_to_Jplayer_pose_t(data.getMean());
    Jdata.cov = data.getCov();
    return(Jdata);
  }

  public static player_localize_set_pose_t Jplayer_localize_set_pose_t_to_player_localize_set_pose_t(Jplayer_localize_set_pose_t Jdata) {
    player_localize_set_pose_t data = new player_localize_set_pose_t();
    data.setMean(Jplayer_pose_t_to_player_pose_t(Jdata.mean));
    data.setCov(Jdata.cov);
    return(data);
  }

  public static Jplayer_localize_particle_t buf_to_Jplayer_localize_particle_t(SWIGTYPE_p_void buf) {
    player_localize_particle_t data = playercore_java.buf_to_player_localize_particle_t(buf);
    return(player_localize_particle_t_to_Jplayer_localize_particle_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_localize_particle_t_to_buf(Jplayer_localize_particle_t Jdata) {
    player_localize_particle_t data = Jplayer_localize_particle_t_to_player_localize_particle_t(Jdata);
    return(playercore_java.player_localize_particle_t_to_buf(data));
  }

  public static Jplayer_localize_particle_t player_localize_particle_t_to_Jplayer_localize_particle_t(player_localize_particle_t data) {
    Jplayer_localize_particle_t Jdata = new Jplayer_localize_particle_t();
    Jdata.pose = player_pose_t_to_Jplayer_pose_t(data.getPose());
    Jdata.alpha = data.getAlpha();
    return(Jdata);
  }

  public static player_localize_particle_t Jplayer_localize_particle_t_to_player_localize_particle_t(Jplayer_localize_particle_t Jdata) {
    player_localize_particle_t data = new player_localize_particle_t();
    data.setPose(Jplayer_pose_t_to_player_pose_t(Jdata.pose));
    data.setAlpha(Jdata.alpha);
    return(data);
  }

  public static Jplayer_localize_get_particles_t buf_to_Jplayer_localize_get_particles_t(SWIGTYPE_p_void buf) {
    player_localize_get_particles_t data = playercore_java.buf_to_player_localize_get_particles_t(buf);
    return(player_localize_get_particles_t_to_Jplayer_localize_get_particles_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_localize_get_particles_t_to_buf(Jplayer_localize_get_particles_t Jdata) {
    player_localize_get_particles_t data = Jplayer_localize_get_particles_t_to_player_localize_get_particles_t(Jdata);
    return(playercore_java.player_localize_get_particles_t_to_buf(data));
  }

  public static Jplayer_localize_get_particles_t player_localize_get_particles_t_to_Jplayer_localize_get_particles_t(player_localize_get_particles_t data) {
    Jplayer_localize_get_particles_t Jdata = new Jplayer_localize_get_particles_t();
    Jdata.mean = player_pose_t_to_Jplayer_pose_t(data.getMean());
    Jdata.variance = data.getVariance();
    Jdata.particles_count = data.getParticles_count();
    {
      player_localize_particle_t foo[] = data.getParticles();
      for(int i=0;i<playercore_javaConstants.PLAYER_LOCALIZE_PARTICLES_MAX;i++)
        Jdata.particles[i] = player_localize_particle_t_to_Jplayer_localize_particle_t(foo[i]);
    }
    return(Jdata);
  }

  public static player_localize_get_particles_t Jplayer_localize_get_particles_t_to_player_localize_get_particles_t(Jplayer_localize_get_particles_t Jdata) {
    player_localize_get_particles_t data = new player_localize_get_particles_t();
    data.setMean(Jplayer_pose_t_to_player_pose_t(Jdata.mean));
    data.setVariance(Jdata.variance);
    data.setParticles_count(Jdata.particles_count);
    {
      player_localize_particle_t foo[] = new player_localize_particle_t[playercore_javaConstants.PLAYER_LOCALIZE_PARTICLES_MAX];
      for(int i=0;i<playercore_javaConstants.PLAYER_LOCALIZE_PARTICLES_MAX;i++)
        foo[i] = Jplayer_localize_particle_t_to_player_localize_particle_t(Jdata.particles[i]);
      data.setParticles(foo);
    }
    return(data);
  }

  public static Jplayer_log_set_write_state_t buf_to_Jplayer_log_set_write_state_t(SWIGTYPE_p_void buf) {
    player_log_set_write_state_t data = playercore_java.buf_to_player_log_set_write_state_t(buf);
    return(player_log_set_write_state_t_to_Jplayer_log_set_write_state_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_log_set_write_state_t_to_buf(Jplayer_log_set_write_state_t Jdata) {
    player_log_set_write_state_t data = Jplayer_log_set_write_state_t_to_player_log_set_write_state_t(Jdata);
    return(playercore_java.player_log_set_write_state_t_to_buf(data));
  }

  public static Jplayer_log_set_write_state_t player_log_set_write_state_t_to_Jplayer_log_set_write_state_t(player_log_set_write_state_t data) {
    Jplayer_log_set_write_state_t Jdata = new Jplayer_log_set_write_state_t();
    Jdata.state = data.getState();
    return(Jdata);
  }

  public static player_log_set_write_state_t Jplayer_log_set_write_state_t_to_player_log_set_write_state_t(Jplayer_log_set_write_state_t Jdata) {
    player_log_set_write_state_t data = new player_log_set_write_state_t();
    data.setState(Jdata.state);
    return(data);
  }

  public static Jplayer_log_set_read_state_t buf_to_Jplayer_log_set_read_state_t(SWIGTYPE_p_void buf) {
    player_log_set_read_state_t data = playercore_java.buf_to_player_log_set_read_state_t(buf);
    return(player_log_set_read_state_t_to_Jplayer_log_set_read_state_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_log_set_read_state_t_to_buf(Jplayer_log_set_read_state_t Jdata) {
    player_log_set_read_state_t data = Jplayer_log_set_read_state_t_to_player_log_set_read_state_t(Jdata);
    return(playercore_java.player_log_set_read_state_t_to_buf(data));
  }

  public static Jplayer_log_set_read_state_t player_log_set_read_state_t_to_Jplayer_log_set_read_state_t(player_log_set_read_state_t data) {
    Jplayer_log_set_read_state_t Jdata = new Jplayer_log_set_read_state_t();
    Jdata.state = data.getState();
    return(Jdata);
  }

  public static player_log_set_read_state_t Jplayer_log_set_read_state_t_to_player_log_set_read_state_t(Jplayer_log_set_read_state_t Jdata) {
    player_log_set_read_state_t data = new player_log_set_read_state_t();
    data.setState(Jdata.state);
    return(data);
  }

  public static Jplayer_log_set_read_rewind_t buf_to_Jplayer_log_set_read_rewind_t(SWIGTYPE_p_void buf) {
    player_log_set_read_rewind_t data = playercore_java.buf_to_player_log_set_read_rewind_t(buf);
    return(player_log_set_read_rewind_t_to_Jplayer_log_set_read_rewind_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_log_set_read_rewind_t_to_buf(Jplayer_log_set_read_rewind_t Jdata) {
    player_log_set_read_rewind_t data = Jplayer_log_set_read_rewind_t_to_player_log_set_read_rewind_t(Jdata);
    return(playercore_java.player_log_set_read_rewind_t_to_buf(data));
  }

  public static Jplayer_log_set_read_rewind_t player_log_set_read_rewind_t_to_Jplayer_log_set_read_rewind_t(player_log_set_read_rewind_t data) {
    Jplayer_log_set_read_rewind_t Jdata = new Jplayer_log_set_read_rewind_t();
    return(Jdata);
  }

  public static player_log_set_read_rewind_t Jplayer_log_set_read_rewind_t_to_player_log_set_read_rewind_t(Jplayer_log_set_read_rewind_t Jdata) {
    player_log_set_read_rewind_t data = new player_log_set_read_rewind_t();
    return(data);
  }

  public static Jplayer_log_get_state_t buf_to_Jplayer_log_get_state_t(SWIGTYPE_p_void buf) {
    player_log_get_state_t data = playercore_java.buf_to_player_log_get_state_t(buf);
    return(player_log_get_state_t_to_Jplayer_log_get_state_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_log_get_state_t_to_buf(Jplayer_log_get_state_t Jdata) {
    player_log_get_state_t data = Jplayer_log_get_state_t_to_player_log_get_state_t(Jdata);
    return(playercore_java.player_log_get_state_t_to_buf(data));
  }

  public static Jplayer_log_get_state_t player_log_get_state_t_to_Jplayer_log_get_state_t(player_log_get_state_t data) {
    Jplayer_log_get_state_t Jdata = new Jplayer_log_get_state_t();
    Jdata.type = data.getType();
    Jdata.state = data.getState();
    return(Jdata);
  }

  public static player_log_get_state_t Jplayer_log_get_state_t_to_player_log_get_state_t(Jplayer_log_get_state_t Jdata) {
    player_log_get_state_t data = new player_log_get_state_t();
    data.setType(Jdata.type);
    data.setState(Jdata.state);
    return(data);
  }

  public static Jplayer_log_set_filename_t buf_to_Jplayer_log_set_filename_t(SWIGTYPE_p_void buf) {
    player_log_set_filename_t data = playercore_java.buf_to_player_log_set_filename_t(buf);
    return(player_log_set_filename_t_to_Jplayer_log_set_filename_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_log_set_filename_t_to_buf(Jplayer_log_set_filename_t Jdata) {
    player_log_set_filename_t data = Jplayer_log_set_filename_t_to_player_log_set_filename_t(Jdata);
    return(playercore_java.player_log_set_filename_t_to_buf(data));
  }

  public static Jplayer_log_set_filename_t player_log_set_filename_t_to_Jplayer_log_set_filename_t(player_log_set_filename_t data) {
    Jplayer_log_set_filename_t Jdata = new Jplayer_log_set_filename_t();
    Jdata.filename_count = data.getFilename_count();
    Jdata.filename = data.getFilename();
    return(Jdata);
  }

  public static player_log_set_filename_t Jplayer_log_set_filename_t_to_player_log_set_filename_t(Jplayer_log_set_filename_t Jdata) {
    player_log_set_filename_t data = new player_log_set_filename_t();
    data.setFilename_count(Jdata.filename_count);
    data.setFilename(Jdata.filename);
    return(data);
  }

  public static Jplayer_map_info_t buf_to_Jplayer_map_info_t(SWIGTYPE_p_void buf) {
    player_map_info_t data = playercore_java.buf_to_player_map_info_t(buf);
    return(player_map_info_t_to_Jplayer_map_info_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_map_info_t_to_buf(Jplayer_map_info_t Jdata) {
    player_map_info_t data = Jplayer_map_info_t_to_player_map_info_t(Jdata);
    return(playercore_java.player_map_info_t_to_buf(data));
  }

  public static Jplayer_map_info_t player_map_info_t_to_Jplayer_map_info_t(player_map_info_t data) {
    Jplayer_map_info_t Jdata = new Jplayer_map_info_t();
    Jdata.scale = data.getScale();
    Jdata.width = data.getWidth();
    Jdata.height = data.getHeight();
    Jdata.origin = player_pose_t_to_Jplayer_pose_t(data.getOrigin());
    return(Jdata);
  }

  public static player_map_info_t Jplayer_map_info_t_to_player_map_info_t(Jplayer_map_info_t Jdata) {
    player_map_info_t data = new player_map_info_t();
    data.setScale(Jdata.scale);
    data.setWidth(Jdata.width);
    data.setHeight(Jdata.height);
    data.setOrigin(Jplayer_pose_t_to_player_pose_t(Jdata.origin));
    return(data);
  }

  public static Jplayer_map_data_t buf_to_Jplayer_map_data_t(SWIGTYPE_p_void buf) {
    player_map_data_t data = playercore_java.buf_to_player_map_data_t(buf);
    return(player_map_data_t_to_Jplayer_map_data_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_map_data_t_to_buf(Jplayer_map_data_t Jdata) {
    player_map_data_t data = Jplayer_map_data_t_to_player_map_data_t(Jdata);
    return(playercore_java.player_map_data_t_to_buf(data));
  }

  public static Jplayer_map_data_t player_map_data_t_to_Jplayer_map_data_t(player_map_data_t data) {
    Jplayer_map_data_t Jdata = new Jplayer_map_data_t();
    Jdata.col = data.getCol();
    Jdata.row = data.getRow();
    Jdata.width = data.getWidth();
    Jdata.height = data.getHeight();
    Jdata.data_count = data.getData_count();
    Jdata.data = data.getData();
    return(Jdata);
  }

  public static player_map_data_t Jplayer_map_data_t_to_player_map_data_t(Jplayer_map_data_t Jdata) {
    player_map_data_t data = new player_map_data_t();
    data.setCol(Jdata.col);
    data.setRow(Jdata.row);
    data.setWidth(Jdata.width);
    data.setHeight(Jdata.height);
    data.setData_count(Jdata.data_count);
    data.setData(Jdata.data);
    return(data);
  }

  public static Jplayer_map_data_vector_t buf_to_Jplayer_map_data_vector_t(SWIGTYPE_p_void buf) {
    player_map_data_vector_t data = playercore_java.buf_to_player_map_data_vector_t(buf);
    return(player_map_data_vector_t_to_Jplayer_map_data_vector_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_map_data_vector_t_to_buf(Jplayer_map_data_vector_t Jdata) {
    player_map_data_vector_t data = Jplayer_map_data_vector_t_to_player_map_data_vector_t(Jdata);
    return(playercore_java.player_map_data_vector_t_to_buf(data));
  }

  public static Jplayer_map_data_vector_t player_map_data_vector_t_to_Jplayer_map_data_vector_t(player_map_data_vector_t data) {
    Jplayer_map_data_vector_t Jdata = new Jplayer_map_data_vector_t();
    Jdata.minx = data.getMinx();
    Jdata.maxx = data.getMaxx();
    Jdata.miny = data.getMiny();
    Jdata.maxy = data.getMaxy();
    Jdata.segments_count = data.getSegments_count();
    {
      player_segment_t foo[] = data.getSegments();
      for(int i=0;i<playercore_javaConstants.PLAYER_MAP_MAX_SEGMENTS;i++)
        Jdata.segments[i] = player_segment_t_to_Jplayer_segment_t(foo[i]);
    }
    return(Jdata);
  }

  public static player_map_data_vector_t Jplayer_map_data_vector_t_to_player_map_data_vector_t(Jplayer_map_data_vector_t Jdata) {
    player_map_data_vector_t data = new player_map_data_vector_t();
    data.setMinx(Jdata.minx);
    data.setMaxx(Jdata.maxx);
    data.setMiny(Jdata.miny);
    data.setMaxy(Jdata.maxy);
    data.setSegments_count(Jdata.segments_count);
    {
      player_segment_t foo[] = new player_segment_t[playercore_javaConstants.PLAYER_MAP_MAX_SEGMENTS];
      for(int i=0;i<playercore_javaConstants.PLAYER_MAP_MAX_SEGMENTS;i++)
        foo[i] = Jplayer_segment_t_to_player_segment_t(Jdata.segments[i]);
      data.setSegments(foo);
    }
    return(data);
  }

  public static Jplayer_mcom_data_t buf_to_Jplayer_mcom_data_t(SWIGTYPE_p_void buf) {
    player_mcom_data_t data = playercore_java.buf_to_player_mcom_data_t(buf);
    return(player_mcom_data_t_to_Jplayer_mcom_data_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_mcom_data_t_to_buf(Jplayer_mcom_data_t Jdata) {
    player_mcom_data_t data = Jplayer_mcom_data_t_to_player_mcom_data_t(Jdata);
    return(playercore_java.player_mcom_data_t_to_buf(data));
  }

  public static Jplayer_mcom_data_t player_mcom_data_t_to_Jplayer_mcom_data_t(player_mcom_data_t data) {
    Jplayer_mcom_data_t Jdata = new Jplayer_mcom_data_t();
    Jdata.full = data.getFull();
    Jdata.data_count = data.getData_count();
    Jdata.data = data.getData();
    return(Jdata);
  }

  public static player_mcom_data_t Jplayer_mcom_data_t_to_player_mcom_data_t(Jplayer_mcom_data_t Jdata) {
    player_mcom_data_t data = new player_mcom_data_t();
    data.setFull(Jdata.full);
    data.setData_count(Jdata.data_count);
    data.setData(Jdata.data);
    return(data);
  }

  public static Jplayer_mcom_config_t buf_to_Jplayer_mcom_config_t(SWIGTYPE_p_void buf) {
    player_mcom_config_t data = playercore_java.buf_to_player_mcom_config_t(buf);
    return(player_mcom_config_t_to_Jplayer_mcom_config_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_mcom_config_t_to_buf(Jplayer_mcom_config_t Jdata) {
    player_mcom_config_t data = Jplayer_mcom_config_t_to_player_mcom_config_t(Jdata);
    return(playercore_java.player_mcom_config_t_to_buf(data));
  }

  public static Jplayer_mcom_config_t player_mcom_config_t_to_Jplayer_mcom_config_t(player_mcom_config_t data) {
    Jplayer_mcom_config_t Jdata = new Jplayer_mcom_config_t();
    Jdata.command = data.getCommand();
    Jdata.type = data.getType();
    Jdata.channel_count = data.getChannel_count();
    Jdata.channel = data.getChannel();
    Jdata.data = player_mcom_data_t_to_Jplayer_mcom_data_t(data.getData());
    return(Jdata);
  }

  public static player_mcom_config_t Jplayer_mcom_config_t_to_player_mcom_config_t(Jplayer_mcom_config_t Jdata) {
    player_mcom_config_t data = new player_mcom_config_t();
    data.setCommand(Jdata.command);
    data.setType(Jdata.type);
    data.setChannel_count(Jdata.channel_count);
    data.setChannel(Jdata.channel);
    data.setData(Jplayer_mcom_data_t_to_player_mcom_data_t(Jdata.data));
    return(data);
  }

  public static Jplayer_mcom_return_t buf_to_Jplayer_mcom_return_t(SWIGTYPE_p_void buf) {
    player_mcom_return_t data = playercore_java.buf_to_player_mcom_return_t(buf);
    return(player_mcom_return_t_to_Jplayer_mcom_return_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_mcom_return_t_to_buf(Jplayer_mcom_return_t Jdata) {
    player_mcom_return_t data = Jplayer_mcom_return_t_to_player_mcom_return_t(Jdata);
    return(playercore_java.player_mcom_return_t_to_buf(data));
  }

  public static Jplayer_mcom_return_t player_mcom_return_t_to_Jplayer_mcom_return_t(player_mcom_return_t data) {
    Jplayer_mcom_return_t Jdata = new Jplayer_mcom_return_t();
    Jdata.type = data.getType();
    Jdata.channel_count = data.getChannel_count();
    Jdata.channel = data.getChannel();
    Jdata.data = player_mcom_data_t_to_Jplayer_mcom_data_t(data.getData());
    return(Jdata);
  }

  public static player_mcom_return_t Jplayer_mcom_return_t_to_player_mcom_return_t(Jplayer_mcom_return_t Jdata) {
    player_mcom_return_t data = new player_mcom_return_t();
    data.setType(Jdata.type);
    data.setChannel_count(Jdata.channel_count);
    data.setChannel(Jdata.channel);
    data.setData(Jplayer_mcom_data_t_to_player_mcom_data_t(Jdata.data));
    return(data);
  }

  public static Jplayer_opaque_data_t buf_to_Jplayer_opaque_data_t(SWIGTYPE_p_void buf) {
    player_opaque_data_t data = playercore_java.buf_to_player_opaque_data_t(buf);
    return(player_opaque_data_t_to_Jplayer_opaque_data_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_opaque_data_t_to_buf(Jplayer_opaque_data_t Jdata) {
    player_opaque_data_t data = Jplayer_opaque_data_t_to_player_opaque_data_t(Jdata);
    return(playercore_java.player_opaque_data_t_to_buf(data));
  }

  public static Jplayer_opaque_data_t player_opaque_data_t_to_Jplayer_opaque_data_t(player_opaque_data_t data) {
    Jplayer_opaque_data_t Jdata = new Jplayer_opaque_data_t();
    Jdata.data_count = data.getData_count();
    Jdata.data = data.getData();
    return(Jdata);
  }

  public static player_opaque_data_t Jplayer_opaque_data_t_to_player_opaque_data_t(Jplayer_opaque_data_t Jdata) {
    player_opaque_data_t data = new player_opaque_data_t();
    data.setData_count(Jdata.data_count);
    data.setData(Jdata.data);
    return(data);
  }

  public static Jplayer_planner_data_t buf_to_Jplayer_planner_data_t(SWIGTYPE_p_void buf) {
    player_planner_data_t data = playercore_java.buf_to_player_planner_data_t(buf);
    return(player_planner_data_t_to_Jplayer_planner_data_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_planner_data_t_to_buf(Jplayer_planner_data_t Jdata) {
    player_planner_data_t data = Jplayer_planner_data_t_to_player_planner_data_t(Jdata);
    return(playercore_java.player_planner_data_t_to_buf(data));
  }

  public static Jplayer_planner_data_t player_planner_data_t_to_Jplayer_planner_data_t(player_planner_data_t data) {
    Jplayer_planner_data_t Jdata = new Jplayer_planner_data_t();
    Jdata.valid = data.getValid();
    Jdata.done = data.getDone();
    Jdata.pos = player_pose_t_to_Jplayer_pose_t(data.getPos());
    Jdata.goal = player_pose_t_to_Jplayer_pose_t(data.getGoal());
    Jdata.waypoint = player_pose_t_to_Jplayer_pose_t(data.getWaypoint());
    Jdata.waypoint_idx = data.getWaypoint_idx();
    Jdata.waypoints_count = data.getWaypoints_count();
    return(Jdata);
  }

  public static player_planner_data_t Jplayer_planner_data_t_to_player_planner_data_t(Jplayer_planner_data_t Jdata) {
    player_planner_data_t data = new player_planner_data_t();
    data.setValid(Jdata.valid);
    data.setDone(Jdata.done);
    data.setPos(Jplayer_pose_t_to_player_pose_t(Jdata.pos));
    data.setGoal(Jplayer_pose_t_to_player_pose_t(Jdata.goal));
    data.setWaypoint(Jplayer_pose_t_to_player_pose_t(Jdata.waypoint));
    data.setWaypoint_idx(Jdata.waypoint_idx);
    data.setWaypoints_count(Jdata.waypoints_count);
    return(data);
  }

  public static Jplayer_planner_cmd_t buf_to_Jplayer_planner_cmd_t(SWIGTYPE_p_void buf) {
    player_planner_cmd_t data = playercore_java.buf_to_player_planner_cmd_t(buf);
    return(player_planner_cmd_t_to_Jplayer_planner_cmd_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_planner_cmd_t_to_buf(Jplayer_planner_cmd_t Jdata) {
    player_planner_cmd_t data = Jplayer_planner_cmd_t_to_player_planner_cmd_t(Jdata);
    return(playercore_java.player_planner_cmd_t_to_buf(data));
  }

  public static Jplayer_planner_cmd_t player_planner_cmd_t_to_Jplayer_planner_cmd_t(player_planner_cmd_t data) {
    Jplayer_planner_cmd_t Jdata = new Jplayer_planner_cmd_t();
    Jdata.goal = player_pose_t_to_Jplayer_pose_t(data.getGoal());
    return(Jdata);
  }

  public static player_planner_cmd_t Jplayer_planner_cmd_t_to_player_planner_cmd_t(Jplayer_planner_cmd_t Jdata) {
    player_planner_cmd_t data = new player_planner_cmd_t();
    data.setGoal(Jplayer_pose_t_to_player_pose_t(Jdata.goal));
    return(data);
  }

  public static Jplayer_planner_waypoints_req_t buf_to_Jplayer_planner_waypoints_req_t(SWIGTYPE_p_void buf) {
    player_planner_waypoints_req_t data = playercore_java.buf_to_player_planner_waypoints_req_t(buf);
    return(player_planner_waypoints_req_t_to_Jplayer_planner_waypoints_req_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_planner_waypoints_req_t_to_buf(Jplayer_planner_waypoints_req_t Jdata) {
    player_planner_waypoints_req_t data = Jplayer_planner_waypoints_req_t_to_player_planner_waypoints_req_t(Jdata);
    return(playercore_java.player_planner_waypoints_req_t_to_buf(data));
  }

  public static Jplayer_planner_waypoints_req_t player_planner_waypoints_req_t_to_Jplayer_planner_waypoints_req_t(player_planner_waypoints_req_t data) {
    Jplayer_planner_waypoints_req_t Jdata = new Jplayer_planner_waypoints_req_t();
    Jdata.waypoints_count = data.getWaypoints_count();
    {
      player_pose_t foo[] = data.getWaypoints();
      for(int i=0;i<playercore_javaConstants.PLAYER_PLANNER_MAX_WAYPOINTS;i++)
        Jdata.waypoints[i] = player_pose_t_to_Jplayer_pose_t(foo[i]);
    }
    return(Jdata);
  }

  public static player_planner_waypoints_req_t Jplayer_planner_waypoints_req_t_to_player_planner_waypoints_req_t(Jplayer_planner_waypoints_req_t Jdata) {
    player_planner_waypoints_req_t data = new player_planner_waypoints_req_t();
    data.setWaypoints_count(Jdata.waypoints_count);
    {
      player_pose_t foo[] = new player_pose_t[playercore_javaConstants.PLAYER_PLANNER_MAX_WAYPOINTS];
      for(int i=0;i<playercore_javaConstants.PLAYER_PLANNER_MAX_WAYPOINTS;i++)
        foo[i] = Jplayer_pose_t_to_player_pose_t(Jdata.waypoints[i]);
      data.setWaypoints(foo);
    }
    return(data);
  }

  public static Jplayer_planner_enable_req_t buf_to_Jplayer_planner_enable_req_t(SWIGTYPE_p_void buf) {
    player_planner_enable_req_t data = playercore_java.buf_to_player_planner_enable_req_t(buf);
    return(player_planner_enable_req_t_to_Jplayer_planner_enable_req_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_planner_enable_req_t_to_buf(Jplayer_planner_enable_req_t Jdata) {
    player_planner_enable_req_t data = Jplayer_planner_enable_req_t_to_player_planner_enable_req_t(Jdata);
    return(playercore_java.player_planner_enable_req_t_to_buf(data));
  }

  public static Jplayer_planner_enable_req_t player_planner_enable_req_t_to_Jplayer_planner_enable_req_t(player_planner_enable_req_t data) {
    Jplayer_planner_enable_req_t Jdata = new Jplayer_planner_enable_req_t();
    Jdata.state = data.getState();
    return(Jdata);
  }

  public static player_planner_enable_req_t Jplayer_planner_enable_req_t_to_player_planner_enable_req_t(Jplayer_planner_enable_req_t Jdata) {
    player_planner_enable_req_t data = new player_planner_enable_req_t();
    data.setState(Jdata.state);
    return(data);
  }

  public static Jplayer_device_devlist_t buf_to_Jplayer_device_devlist_t(SWIGTYPE_p_void buf) {
    player_device_devlist_t data = playercore_java.buf_to_player_device_devlist_t(buf);
    return(player_device_devlist_t_to_Jplayer_device_devlist_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_device_devlist_t_to_buf(Jplayer_device_devlist_t Jdata) {
    player_device_devlist_t data = Jplayer_device_devlist_t_to_player_device_devlist_t(Jdata);
    return(playercore_java.player_device_devlist_t_to_buf(data));
  }

  public static Jplayer_device_devlist_t player_device_devlist_t_to_Jplayer_device_devlist_t(player_device_devlist_t data) {
    Jplayer_device_devlist_t Jdata = new Jplayer_device_devlist_t();
    Jdata.devices_count = data.getDevices_count();
    {
      player_devaddr_t foo[] = data.getDevices();
      for(int i=0;i<playercore_javaConstants.PLAYER_MAX_DEVICES;i++)
        Jdata.devices[i] = player_devaddr_t_to_Jplayer_devaddr_t(foo[i]);
    }
    return(Jdata);
  }

  public static player_device_devlist_t Jplayer_device_devlist_t_to_player_device_devlist_t(Jplayer_device_devlist_t Jdata) {
    player_device_devlist_t data = new player_device_devlist_t();
    data.setDevices_count(Jdata.devices_count);
    {
      player_devaddr_t foo[] = new player_devaddr_t[playercore_javaConstants.PLAYER_MAX_DEVICES];
      for(int i=0;i<playercore_javaConstants.PLAYER_MAX_DEVICES;i++)
        foo[i] = Jplayer_devaddr_t_to_player_devaddr_t(Jdata.devices[i]);
      data.setDevices(foo);
    }
    return(data);
  }

  public static Jplayer_device_driverinfo_t buf_to_Jplayer_device_driverinfo_t(SWIGTYPE_p_void buf) {
    player_device_driverinfo_t data = playercore_java.buf_to_player_device_driverinfo_t(buf);
    return(player_device_driverinfo_t_to_Jplayer_device_driverinfo_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_device_driverinfo_t_to_buf(Jplayer_device_driverinfo_t Jdata) {
    player_device_driverinfo_t data = Jplayer_device_driverinfo_t_to_player_device_driverinfo_t(Jdata);
    return(playercore_java.player_device_driverinfo_t_to_buf(data));
  }

  public static Jplayer_device_driverinfo_t player_device_driverinfo_t_to_Jplayer_device_driverinfo_t(player_device_driverinfo_t data) {
    Jplayer_device_driverinfo_t Jdata = new Jplayer_device_driverinfo_t();
    Jdata.addr = player_devaddr_t_to_Jplayer_devaddr_t(data.getAddr());
    Jdata.driver_name_count = data.getDriver_name_count();
    Jdata.driver_name = data.getDriver_name();
    return(Jdata);
  }

  public static player_device_driverinfo_t Jplayer_device_driverinfo_t_to_player_device_driverinfo_t(Jplayer_device_driverinfo_t Jdata) {
    player_device_driverinfo_t data = new player_device_driverinfo_t();
    data.setAddr(Jplayer_devaddr_t_to_player_devaddr_t(Jdata.addr));
    data.setDriver_name_count(Jdata.driver_name_count);
    data.setDriver_name(Jdata.driver_name);
    return(data);
  }

  public static Jplayer_device_req_t buf_to_Jplayer_device_req_t(SWIGTYPE_p_void buf) {
    player_device_req_t data = playercore_java.buf_to_player_device_req_t(buf);
    return(player_device_req_t_to_Jplayer_device_req_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_device_req_t_to_buf(Jplayer_device_req_t Jdata) {
    player_device_req_t data = Jplayer_device_req_t_to_player_device_req_t(Jdata);
    return(playercore_java.player_device_req_t_to_buf(data));
  }

  public static Jplayer_device_req_t player_device_req_t_to_Jplayer_device_req_t(player_device_req_t data) {
    Jplayer_device_req_t Jdata = new Jplayer_device_req_t();
    Jdata.addr = player_devaddr_t_to_Jplayer_devaddr_t(data.getAddr());
    Jdata.access = data.getAccess();
    Jdata.driver_name_count = data.getDriver_name_count();
    Jdata.driver_name = data.getDriver_name();
    return(Jdata);
  }

  public static player_device_req_t Jplayer_device_req_t_to_player_device_req_t(Jplayer_device_req_t Jdata) {
    player_device_req_t data = new player_device_req_t();
    data.setAddr(Jplayer_devaddr_t_to_player_devaddr_t(Jdata.addr));
    data.setAccess(Jdata.access);
    data.setDriver_name_count(Jdata.driver_name_count);
    data.setDriver_name(Jdata.driver_name);
    return(data);
  }

  public static Jplayer_device_data_req_t buf_to_Jplayer_device_data_req_t(SWIGTYPE_p_void buf) {
    player_device_data_req_t data = playercore_java.buf_to_player_device_data_req_t(buf);
    return(player_device_data_req_t_to_Jplayer_device_data_req_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_device_data_req_t_to_buf(Jplayer_device_data_req_t Jdata) {
    player_device_data_req_t data = Jplayer_device_data_req_t_to_player_device_data_req_t(Jdata);
    return(playercore_java.player_device_data_req_t_to_buf(data));
  }

  public static Jplayer_device_data_req_t player_device_data_req_t_to_Jplayer_device_data_req_t(player_device_data_req_t data) {
    Jplayer_device_data_req_t Jdata = new Jplayer_device_data_req_t();
    return(Jdata);
  }

  public static player_device_data_req_t Jplayer_device_data_req_t_to_player_device_data_req_t(Jplayer_device_data_req_t Jdata) {
    player_device_data_req_t data = new player_device_data_req_t();
    return(data);
  }

  public static Jplayer_device_datamode_req_t buf_to_Jplayer_device_datamode_req_t(SWIGTYPE_p_void buf) {
    player_device_datamode_req_t data = playercore_java.buf_to_player_device_datamode_req_t(buf);
    return(player_device_datamode_req_t_to_Jplayer_device_datamode_req_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_device_datamode_req_t_to_buf(Jplayer_device_datamode_req_t Jdata) {
    player_device_datamode_req_t data = Jplayer_device_datamode_req_t_to_player_device_datamode_req_t(Jdata);
    return(playercore_java.player_device_datamode_req_t_to_buf(data));
  }

  public static Jplayer_device_datamode_req_t player_device_datamode_req_t_to_Jplayer_device_datamode_req_t(player_device_datamode_req_t data) {
    Jplayer_device_datamode_req_t Jdata = new Jplayer_device_datamode_req_t();
    Jdata.mode = data.getMode();
    return(Jdata);
  }

  public static player_device_datamode_req_t Jplayer_device_datamode_req_t_to_player_device_datamode_req_t(Jplayer_device_datamode_req_t Jdata) {
    player_device_datamode_req_t data = new player_device_datamode_req_t();
    data.setMode(Jdata.mode);
    return(data);
  }

  public static Jplayer_device_auth_req_t buf_to_Jplayer_device_auth_req_t(SWIGTYPE_p_void buf) {
    player_device_auth_req_t data = playercore_java.buf_to_player_device_auth_req_t(buf);
    return(player_device_auth_req_t_to_Jplayer_device_auth_req_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_device_auth_req_t_to_buf(Jplayer_device_auth_req_t Jdata) {
    player_device_auth_req_t data = Jplayer_device_auth_req_t_to_player_device_auth_req_t(Jdata);
    return(playercore_java.player_device_auth_req_t_to_buf(data));
  }

  public static Jplayer_device_auth_req_t player_device_auth_req_t_to_Jplayer_device_auth_req_t(player_device_auth_req_t data) {
    Jplayer_device_auth_req_t Jdata = new Jplayer_device_auth_req_t();
    Jdata.auth_key_count = data.getAuth_key_count();
    Jdata.auth_key = data.getAuth_key();
    return(Jdata);
  }

  public static player_device_auth_req_t Jplayer_device_auth_req_t_to_player_device_auth_req_t(Jplayer_device_auth_req_t Jdata) {
    player_device_auth_req_t data = new player_device_auth_req_t();
    data.setAuth_key_count(Jdata.auth_key_count);
    data.setAuth_key(Jdata.auth_key);
    return(data);
  }

  public static Jplayer_device_nameservice_req_t buf_to_Jplayer_device_nameservice_req_t(SWIGTYPE_p_void buf) {
    player_device_nameservice_req_t data = playercore_java.buf_to_player_device_nameservice_req_t(buf);
    return(player_device_nameservice_req_t_to_Jplayer_device_nameservice_req_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_device_nameservice_req_t_to_buf(Jplayer_device_nameservice_req_t Jdata) {
    player_device_nameservice_req_t data = Jplayer_device_nameservice_req_t_to_player_device_nameservice_req_t(Jdata);
    return(playercore_java.player_device_nameservice_req_t_to_buf(data));
  }

  public static Jplayer_device_nameservice_req_t player_device_nameservice_req_t_to_Jplayer_device_nameservice_req_t(player_device_nameservice_req_t data) {
    Jplayer_device_nameservice_req_t Jdata = new Jplayer_device_nameservice_req_t();
    Jdata.name_count = data.getName_count();
    Jdata.name = data.getName();
    Jdata.port = data.getPort();
    return(Jdata);
  }

  public static player_device_nameservice_req_t Jplayer_device_nameservice_req_t_to_player_device_nameservice_req_t(Jplayer_device_nameservice_req_t Jdata) {
    player_device_nameservice_req_t data = new player_device_nameservice_req_t();
    data.setName_count(Jdata.name_count);
    data.setName(Jdata.name);
    data.setPort(Jdata.port);
    return(data);
  }

  public static Jplayer_add_replace_rule_req_t buf_to_Jplayer_add_replace_rule_req_t(SWIGTYPE_p_void buf) {
    player_add_replace_rule_req_t data = playercore_java.buf_to_player_add_replace_rule_req_t(buf);
    return(player_add_replace_rule_req_t_to_Jplayer_add_replace_rule_req_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_add_replace_rule_req_t_to_buf(Jplayer_add_replace_rule_req_t Jdata) {
    player_add_replace_rule_req_t data = Jplayer_add_replace_rule_req_t_to_player_add_replace_rule_req_t(Jdata);
    return(playercore_java.player_add_replace_rule_req_t_to_buf(data));
  }

  public static Jplayer_add_replace_rule_req_t player_add_replace_rule_req_t_to_Jplayer_add_replace_rule_req_t(player_add_replace_rule_req_t data) {
    Jplayer_add_replace_rule_req_t Jdata = new Jplayer_add_replace_rule_req_t();
    Jdata.interf = data.getInterf();
    Jdata.index = data.getIndex();
    Jdata.type = data.getType();
    Jdata.subtype = data.getSubtype();
    Jdata.replace = data.getReplace();
    return(Jdata);
  }

  public static player_add_replace_rule_req_t Jplayer_add_replace_rule_req_t_to_player_add_replace_rule_req_t(Jplayer_add_replace_rule_req_t Jdata) {
    player_add_replace_rule_req_t data = new player_add_replace_rule_req_t();
    data.setInterf(Jdata.interf);
    data.setIndex(Jdata.index);
    data.setType(Jdata.type);
    data.setSubtype(Jdata.subtype);
    data.setReplace(Jdata.replace);
    return(data);
  }

  public static Jplayer_position1d_data_t buf_to_Jplayer_position1d_data_t(SWIGTYPE_p_void buf) {
    player_position1d_data_t data = playercore_java.buf_to_player_position1d_data_t(buf);
    return(player_position1d_data_t_to_Jplayer_position1d_data_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_position1d_data_t_to_buf(Jplayer_position1d_data_t Jdata) {
    player_position1d_data_t data = Jplayer_position1d_data_t_to_player_position1d_data_t(Jdata);
    return(playercore_java.player_position1d_data_t_to_buf(data));
  }

  public static Jplayer_position1d_data_t player_position1d_data_t_to_Jplayer_position1d_data_t(player_position1d_data_t data) {
    Jplayer_position1d_data_t Jdata = new Jplayer_position1d_data_t();
    Jdata.pos = data.getPos();
    Jdata.vel = data.getVel();
    Jdata.stall = data.getStall();
    Jdata.status = data.getStatus();
    return(Jdata);
  }

  public static player_position1d_data_t Jplayer_position1d_data_t_to_player_position1d_data_t(Jplayer_position1d_data_t Jdata) {
    player_position1d_data_t data = new player_position1d_data_t();
    data.setPos(Jdata.pos);
    data.setVel(Jdata.vel);
    data.setStall(Jdata.stall);
    data.setStatus(Jdata.status);
    return(data);
  }

  public static Jplayer_position1d_cmd_vel_t buf_to_Jplayer_position1d_cmd_vel_t(SWIGTYPE_p_void buf) {
    player_position1d_cmd_vel_t data = playercore_java.buf_to_player_position1d_cmd_vel_t(buf);
    return(player_position1d_cmd_vel_t_to_Jplayer_position1d_cmd_vel_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_position1d_cmd_vel_t_to_buf(Jplayer_position1d_cmd_vel_t Jdata) {
    player_position1d_cmd_vel_t data = Jplayer_position1d_cmd_vel_t_to_player_position1d_cmd_vel_t(Jdata);
    return(playercore_java.player_position1d_cmd_vel_t_to_buf(data));
  }

  public static Jplayer_position1d_cmd_vel_t player_position1d_cmd_vel_t_to_Jplayer_position1d_cmd_vel_t(player_position1d_cmd_vel_t data) {
    Jplayer_position1d_cmd_vel_t Jdata = new Jplayer_position1d_cmd_vel_t();
    Jdata.vel = data.getVel();
    Jdata.state = data.getState();
    return(Jdata);
  }

  public static player_position1d_cmd_vel_t Jplayer_position1d_cmd_vel_t_to_player_position1d_cmd_vel_t(Jplayer_position1d_cmd_vel_t Jdata) {
    player_position1d_cmd_vel_t data = new player_position1d_cmd_vel_t();
    data.setVel(Jdata.vel);
    data.setState(Jdata.state);
    return(data);
  }

  public static Jplayer_position1d_cmd_pos_t buf_to_Jplayer_position1d_cmd_pos_t(SWIGTYPE_p_void buf) {
    player_position1d_cmd_pos_t data = playercore_java.buf_to_player_position1d_cmd_pos_t(buf);
    return(player_position1d_cmd_pos_t_to_Jplayer_position1d_cmd_pos_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_position1d_cmd_pos_t_to_buf(Jplayer_position1d_cmd_pos_t Jdata) {
    player_position1d_cmd_pos_t data = Jplayer_position1d_cmd_pos_t_to_player_position1d_cmd_pos_t(Jdata);
    return(playercore_java.player_position1d_cmd_pos_t_to_buf(data));
  }

  public static Jplayer_position1d_cmd_pos_t player_position1d_cmd_pos_t_to_Jplayer_position1d_cmd_pos_t(player_position1d_cmd_pos_t data) {
    Jplayer_position1d_cmd_pos_t Jdata = new Jplayer_position1d_cmd_pos_t();
    Jdata.pos = data.getPos();
    Jdata.vel = data.getVel();
    Jdata.state = data.getState();
    return(Jdata);
  }

  public static player_position1d_cmd_pos_t Jplayer_position1d_cmd_pos_t_to_player_position1d_cmd_pos_t(Jplayer_position1d_cmd_pos_t Jdata) {
    player_position1d_cmd_pos_t data = new player_position1d_cmd_pos_t();
    data.setPos(Jdata.pos);
    data.setVel(Jdata.vel);
    data.setState(Jdata.state);
    return(data);
  }

  public static Jplayer_position1d_geom_t buf_to_Jplayer_position1d_geom_t(SWIGTYPE_p_void buf) {
    player_position1d_geom_t data = playercore_java.buf_to_player_position1d_geom_t(buf);
    return(player_position1d_geom_t_to_Jplayer_position1d_geom_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_position1d_geom_t_to_buf(Jplayer_position1d_geom_t Jdata) {
    player_position1d_geom_t data = Jplayer_position1d_geom_t_to_player_position1d_geom_t(Jdata);
    return(playercore_java.player_position1d_geom_t_to_buf(data));
  }

  public static Jplayer_position1d_geom_t player_position1d_geom_t_to_Jplayer_position1d_geom_t(player_position1d_geom_t data) {
    Jplayer_position1d_geom_t Jdata = new Jplayer_position1d_geom_t();
    Jdata.pose = player_pose_t_to_Jplayer_pose_t(data.getPose());
    Jdata.size = player_bbox_t_to_Jplayer_bbox_t(data.getSize());
    return(Jdata);
  }

  public static player_position1d_geom_t Jplayer_position1d_geom_t_to_player_position1d_geom_t(Jplayer_position1d_geom_t Jdata) {
    player_position1d_geom_t data = new player_position1d_geom_t();
    data.setPose(Jplayer_pose_t_to_player_pose_t(Jdata.pose));
    data.setSize(Jplayer_bbox_t_to_player_bbox_t(Jdata.size));
    return(data);
  }

  public static Jplayer_position1d_power_config_t buf_to_Jplayer_position1d_power_config_t(SWIGTYPE_p_void buf) {
    player_position1d_power_config_t data = playercore_java.buf_to_player_position1d_power_config_t(buf);
    return(player_position1d_power_config_t_to_Jplayer_position1d_power_config_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_position1d_power_config_t_to_buf(Jplayer_position1d_power_config_t Jdata) {
    player_position1d_power_config_t data = Jplayer_position1d_power_config_t_to_player_position1d_power_config_t(Jdata);
    return(playercore_java.player_position1d_power_config_t_to_buf(data));
  }

  public static Jplayer_position1d_power_config_t player_position1d_power_config_t_to_Jplayer_position1d_power_config_t(player_position1d_power_config_t data) {
    Jplayer_position1d_power_config_t Jdata = new Jplayer_position1d_power_config_t();
    Jdata.state = data.getState();
    return(Jdata);
  }

  public static player_position1d_power_config_t Jplayer_position1d_power_config_t_to_player_position1d_power_config_t(Jplayer_position1d_power_config_t Jdata) {
    player_position1d_power_config_t data = new player_position1d_power_config_t();
    data.setState(Jdata.state);
    return(data);
  }

  public static Jplayer_position1d_velocity_mode_config_t buf_to_Jplayer_position1d_velocity_mode_config_t(SWIGTYPE_p_void buf) {
    player_position1d_velocity_mode_config_t data = playercore_java.buf_to_player_position1d_velocity_mode_config_t(buf);
    return(player_position1d_velocity_mode_config_t_to_Jplayer_position1d_velocity_mode_config_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_position1d_velocity_mode_config_t_to_buf(Jplayer_position1d_velocity_mode_config_t Jdata) {
    player_position1d_velocity_mode_config_t data = Jplayer_position1d_velocity_mode_config_t_to_player_position1d_velocity_mode_config_t(Jdata);
    return(playercore_java.player_position1d_velocity_mode_config_t_to_buf(data));
  }

  public static Jplayer_position1d_velocity_mode_config_t player_position1d_velocity_mode_config_t_to_Jplayer_position1d_velocity_mode_config_t(player_position1d_velocity_mode_config_t data) {
    Jplayer_position1d_velocity_mode_config_t Jdata = new Jplayer_position1d_velocity_mode_config_t();
    Jdata.value = data.getValue();
    return(Jdata);
  }

  public static player_position1d_velocity_mode_config_t Jplayer_position1d_velocity_mode_config_t_to_player_position1d_velocity_mode_config_t(Jplayer_position1d_velocity_mode_config_t Jdata) {
    player_position1d_velocity_mode_config_t data = new player_position1d_velocity_mode_config_t();
    data.setValue(Jdata.value);
    return(data);
  }

  public static Jplayer_position1d_reset_odom_config_t buf_to_Jplayer_position1d_reset_odom_config_t(SWIGTYPE_p_void buf) {
    player_position1d_reset_odom_config_t data = playercore_java.buf_to_player_position1d_reset_odom_config_t(buf);
    return(player_position1d_reset_odom_config_t_to_Jplayer_position1d_reset_odom_config_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_position1d_reset_odom_config_t_to_buf(Jplayer_position1d_reset_odom_config_t Jdata) {
    player_position1d_reset_odom_config_t data = Jplayer_position1d_reset_odom_config_t_to_player_position1d_reset_odom_config_t(Jdata);
    return(playercore_java.player_position1d_reset_odom_config_t_to_buf(data));
  }

  public static Jplayer_position1d_reset_odom_config_t player_position1d_reset_odom_config_t_to_Jplayer_position1d_reset_odom_config_t(player_position1d_reset_odom_config_t data) {
    Jplayer_position1d_reset_odom_config_t Jdata = new Jplayer_position1d_reset_odom_config_t();
    Jdata.value = data.getValue();
    return(Jdata);
  }

  public static player_position1d_reset_odom_config_t Jplayer_position1d_reset_odom_config_t_to_player_position1d_reset_odom_config_t(Jplayer_position1d_reset_odom_config_t Jdata) {
    player_position1d_reset_odom_config_t data = new player_position1d_reset_odom_config_t();
    data.setValue(Jdata.value);
    return(data);
  }

  public static Jplayer_position1d_position_mode_req_t buf_to_Jplayer_position1d_position_mode_req_t(SWIGTYPE_p_void buf) {
    player_position1d_position_mode_req_t data = playercore_java.buf_to_player_position1d_position_mode_req_t(buf);
    return(player_position1d_position_mode_req_t_to_Jplayer_position1d_position_mode_req_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_position1d_position_mode_req_t_to_buf(Jplayer_position1d_position_mode_req_t Jdata) {
    player_position1d_position_mode_req_t data = Jplayer_position1d_position_mode_req_t_to_player_position1d_position_mode_req_t(Jdata);
    return(playercore_java.player_position1d_position_mode_req_t_to_buf(data));
  }

  public static Jplayer_position1d_position_mode_req_t player_position1d_position_mode_req_t_to_Jplayer_position1d_position_mode_req_t(player_position1d_position_mode_req_t data) {
    Jplayer_position1d_position_mode_req_t Jdata = new Jplayer_position1d_position_mode_req_t();
    Jdata.state = data.getState();
    return(Jdata);
  }

  public static player_position1d_position_mode_req_t Jplayer_position1d_position_mode_req_t_to_player_position1d_position_mode_req_t(Jplayer_position1d_position_mode_req_t Jdata) {
    player_position1d_position_mode_req_t data = new player_position1d_position_mode_req_t();
    data.setState(Jdata.state);
    return(data);
  }

  public static Jplayer_position1d_set_odom_req_t buf_to_Jplayer_position1d_set_odom_req_t(SWIGTYPE_p_void buf) {
    player_position1d_set_odom_req_t data = playercore_java.buf_to_player_position1d_set_odom_req_t(buf);
    return(player_position1d_set_odom_req_t_to_Jplayer_position1d_set_odom_req_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_position1d_set_odom_req_t_to_buf(Jplayer_position1d_set_odom_req_t Jdata) {
    player_position1d_set_odom_req_t data = Jplayer_position1d_set_odom_req_t_to_player_position1d_set_odom_req_t(Jdata);
    return(playercore_java.player_position1d_set_odom_req_t_to_buf(data));
  }

  public static Jplayer_position1d_set_odom_req_t player_position1d_set_odom_req_t_to_Jplayer_position1d_set_odom_req_t(player_position1d_set_odom_req_t data) {
    Jplayer_position1d_set_odom_req_t Jdata = new Jplayer_position1d_set_odom_req_t();
    Jdata.pos = data.getPos();
    return(Jdata);
  }

  public static player_position1d_set_odom_req_t Jplayer_position1d_set_odom_req_t_to_player_position1d_set_odom_req_t(Jplayer_position1d_set_odom_req_t Jdata) {
    player_position1d_set_odom_req_t data = new player_position1d_set_odom_req_t();
    data.setPos(Jdata.pos);
    return(data);
  }

  public static Jplayer_position1d_speed_pid_req_t buf_to_Jplayer_position1d_speed_pid_req_t(SWIGTYPE_p_void buf) {
    player_position1d_speed_pid_req_t data = playercore_java.buf_to_player_position1d_speed_pid_req_t(buf);
    return(player_position1d_speed_pid_req_t_to_Jplayer_position1d_speed_pid_req_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_position1d_speed_pid_req_t_to_buf(Jplayer_position1d_speed_pid_req_t Jdata) {
    player_position1d_speed_pid_req_t data = Jplayer_position1d_speed_pid_req_t_to_player_position1d_speed_pid_req_t(Jdata);
    return(playercore_java.player_position1d_speed_pid_req_t_to_buf(data));
  }

  public static Jplayer_position1d_speed_pid_req_t player_position1d_speed_pid_req_t_to_Jplayer_position1d_speed_pid_req_t(player_position1d_speed_pid_req_t data) {
    Jplayer_position1d_speed_pid_req_t Jdata = new Jplayer_position1d_speed_pid_req_t();
    Jdata.kp = data.getKp();
    Jdata.ki = data.getKi();
    Jdata.kd = data.getKd();
    return(Jdata);
  }

  public static player_position1d_speed_pid_req_t Jplayer_position1d_speed_pid_req_t_to_player_position1d_speed_pid_req_t(Jplayer_position1d_speed_pid_req_t Jdata) {
    player_position1d_speed_pid_req_t data = new player_position1d_speed_pid_req_t();
    data.setKp(Jdata.kp);
    data.setKi(Jdata.ki);
    data.setKd(Jdata.kd);
    return(data);
  }

  public static Jplayer_position1d_position_pid_req_t buf_to_Jplayer_position1d_position_pid_req_t(SWIGTYPE_p_void buf) {
    player_position1d_position_pid_req_t data = playercore_java.buf_to_player_position1d_position_pid_req_t(buf);
    return(player_position1d_position_pid_req_t_to_Jplayer_position1d_position_pid_req_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_position1d_position_pid_req_t_to_buf(Jplayer_position1d_position_pid_req_t Jdata) {
    player_position1d_position_pid_req_t data = Jplayer_position1d_position_pid_req_t_to_player_position1d_position_pid_req_t(Jdata);
    return(playercore_java.player_position1d_position_pid_req_t_to_buf(data));
  }

  public static Jplayer_position1d_position_pid_req_t player_position1d_position_pid_req_t_to_Jplayer_position1d_position_pid_req_t(player_position1d_position_pid_req_t data) {
    Jplayer_position1d_position_pid_req_t Jdata = new Jplayer_position1d_position_pid_req_t();
    Jdata.kp = data.getKp();
    Jdata.ki = data.getKi();
    Jdata.kd = data.getKd();
    return(Jdata);
  }

  public static player_position1d_position_pid_req_t Jplayer_position1d_position_pid_req_t_to_player_position1d_position_pid_req_t(Jplayer_position1d_position_pid_req_t Jdata) {
    player_position1d_position_pid_req_t data = new player_position1d_position_pid_req_t();
    data.setKp(Jdata.kp);
    data.setKi(Jdata.ki);
    data.setKd(Jdata.kd);
    return(data);
  }

  public static Jplayer_position1d_speed_prof_req_t buf_to_Jplayer_position1d_speed_prof_req_t(SWIGTYPE_p_void buf) {
    player_position1d_speed_prof_req_t data = playercore_java.buf_to_player_position1d_speed_prof_req_t(buf);
    return(player_position1d_speed_prof_req_t_to_Jplayer_position1d_speed_prof_req_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_position1d_speed_prof_req_t_to_buf(Jplayer_position1d_speed_prof_req_t Jdata) {
    player_position1d_speed_prof_req_t data = Jplayer_position1d_speed_prof_req_t_to_player_position1d_speed_prof_req_t(Jdata);
    return(playercore_java.player_position1d_speed_prof_req_t_to_buf(data));
  }

  public static Jplayer_position1d_speed_prof_req_t player_position1d_speed_prof_req_t_to_Jplayer_position1d_speed_prof_req_t(player_position1d_speed_prof_req_t data) {
    Jplayer_position1d_speed_prof_req_t Jdata = new Jplayer_position1d_speed_prof_req_t();
    Jdata.speed = data.getSpeed();
    Jdata.acc = data.getAcc();
    return(Jdata);
  }

  public static player_position1d_speed_prof_req_t Jplayer_position1d_speed_prof_req_t_to_player_position1d_speed_prof_req_t(Jplayer_position1d_speed_prof_req_t Jdata) {
    player_position1d_speed_prof_req_t data = new player_position1d_speed_prof_req_t();
    data.setSpeed(Jdata.speed);
    data.setAcc(Jdata.acc);
    return(data);
  }

  public static Jplayer_position2d_data_t buf_to_Jplayer_position2d_data_t(SWIGTYPE_p_void buf) {
    player_position2d_data_t data = playercore_java.buf_to_player_position2d_data_t(buf);
    return(player_position2d_data_t_to_Jplayer_position2d_data_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_position2d_data_t_to_buf(Jplayer_position2d_data_t Jdata) {
    player_position2d_data_t data = Jplayer_position2d_data_t_to_player_position2d_data_t(Jdata);
    return(playercore_java.player_position2d_data_t_to_buf(data));
  }

  public static Jplayer_position2d_data_t player_position2d_data_t_to_Jplayer_position2d_data_t(player_position2d_data_t data) {
    Jplayer_position2d_data_t Jdata = new Jplayer_position2d_data_t();
    Jdata.pos = player_pose_t_to_Jplayer_pose_t(data.getPos());
    Jdata.vel = player_pose_t_to_Jplayer_pose_t(data.getVel());
    Jdata.stall = data.getStall();
    return(Jdata);
  }

  public static player_position2d_data_t Jplayer_position2d_data_t_to_player_position2d_data_t(Jplayer_position2d_data_t Jdata) {
    player_position2d_data_t data = new player_position2d_data_t();
    data.setPos(Jplayer_pose_t_to_player_pose_t(Jdata.pos));
    data.setVel(Jplayer_pose_t_to_player_pose_t(Jdata.vel));
    data.setStall(Jdata.stall);
    return(data);
  }

  public static Jplayer_position2d_cmd_vel_t buf_to_Jplayer_position2d_cmd_vel_t(SWIGTYPE_p_void buf) {
    player_position2d_cmd_vel_t data = playercore_java.buf_to_player_position2d_cmd_vel_t(buf);
    return(player_position2d_cmd_vel_t_to_Jplayer_position2d_cmd_vel_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_position2d_cmd_vel_t_to_buf(Jplayer_position2d_cmd_vel_t Jdata) {
    player_position2d_cmd_vel_t data = Jplayer_position2d_cmd_vel_t_to_player_position2d_cmd_vel_t(Jdata);
    return(playercore_java.player_position2d_cmd_vel_t_to_buf(data));
  }

  public static Jplayer_position2d_cmd_vel_t player_position2d_cmd_vel_t_to_Jplayer_position2d_cmd_vel_t(player_position2d_cmd_vel_t data) {
    Jplayer_position2d_cmd_vel_t Jdata = new Jplayer_position2d_cmd_vel_t();
    Jdata.vel = player_pose_t_to_Jplayer_pose_t(data.getVel());
    Jdata.state = data.getState();
    return(Jdata);
  }

  public static player_position2d_cmd_vel_t Jplayer_position2d_cmd_vel_t_to_player_position2d_cmd_vel_t(Jplayer_position2d_cmd_vel_t Jdata) {
    player_position2d_cmd_vel_t data = new player_position2d_cmd_vel_t();
    data.setVel(Jplayer_pose_t_to_player_pose_t(Jdata.vel));
    data.setState(Jdata.state);
    return(data);
  }

  public static Jplayer_position2d_cmd_pos_t buf_to_Jplayer_position2d_cmd_pos_t(SWIGTYPE_p_void buf) {
    player_position2d_cmd_pos_t data = playercore_java.buf_to_player_position2d_cmd_pos_t(buf);
    return(player_position2d_cmd_pos_t_to_Jplayer_position2d_cmd_pos_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_position2d_cmd_pos_t_to_buf(Jplayer_position2d_cmd_pos_t Jdata) {
    player_position2d_cmd_pos_t data = Jplayer_position2d_cmd_pos_t_to_player_position2d_cmd_pos_t(Jdata);
    return(playercore_java.player_position2d_cmd_pos_t_to_buf(data));
  }

  public static Jplayer_position2d_cmd_pos_t player_position2d_cmd_pos_t_to_Jplayer_position2d_cmd_pos_t(player_position2d_cmd_pos_t data) {
    Jplayer_position2d_cmd_pos_t Jdata = new Jplayer_position2d_cmd_pos_t();
    Jdata.pos = player_pose_t_to_Jplayer_pose_t(data.getPos());
    Jdata.vel = player_pose_t_to_Jplayer_pose_t(data.getVel());
    Jdata.state = data.getState();
    return(Jdata);
  }

  public static player_position2d_cmd_pos_t Jplayer_position2d_cmd_pos_t_to_player_position2d_cmd_pos_t(Jplayer_position2d_cmd_pos_t Jdata) {
    player_position2d_cmd_pos_t data = new player_position2d_cmd_pos_t();
    data.setPos(Jplayer_pose_t_to_player_pose_t(Jdata.pos));
    data.setVel(Jplayer_pose_t_to_player_pose_t(Jdata.vel));
    data.setState(Jdata.state);
    return(data);
  }

  public static Jplayer_position2d_cmd_car_t buf_to_Jplayer_position2d_cmd_car_t(SWIGTYPE_p_void buf) {
    player_position2d_cmd_car_t data = playercore_java.buf_to_player_position2d_cmd_car_t(buf);
    return(player_position2d_cmd_car_t_to_Jplayer_position2d_cmd_car_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_position2d_cmd_car_t_to_buf(Jplayer_position2d_cmd_car_t Jdata) {
    player_position2d_cmd_car_t data = Jplayer_position2d_cmd_car_t_to_player_position2d_cmd_car_t(Jdata);
    return(playercore_java.player_position2d_cmd_car_t_to_buf(data));
  }

  public static Jplayer_position2d_cmd_car_t player_position2d_cmd_car_t_to_Jplayer_position2d_cmd_car_t(player_position2d_cmd_car_t data) {
    Jplayer_position2d_cmd_car_t Jdata = new Jplayer_position2d_cmd_car_t();
    Jdata.velocity = data.getVelocity();
    Jdata.angle = data.getAngle();
    return(Jdata);
  }

  public static player_position2d_cmd_car_t Jplayer_position2d_cmd_car_t_to_player_position2d_cmd_car_t(Jplayer_position2d_cmd_car_t Jdata) {
    player_position2d_cmd_car_t data = new player_position2d_cmd_car_t();
    data.setVelocity(Jdata.velocity);
    data.setAngle(Jdata.angle);
    return(data);
  }

  public static Jplayer_position2d_geom_t buf_to_Jplayer_position2d_geom_t(SWIGTYPE_p_void buf) {
    player_position2d_geom_t data = playercore_java.buf_to_player_position2d_geom_t(buf);
    return(player_position2d_geom_t_to_Jplayer_position2d_geom_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_position2d_geom_t_to_buf(Jplayer_position2d_geom_t Jdata) {
    player_position2d_geom_t data = Jplayer_position2d_geom_t_to_player_position2d_geom_t(Jdata);
    return(playercore_java.player_position2d_geom_t_to_buf(data));
  }

  public static Jplayer_position2d_geom_t player_position2d_geom_t_to_Jplayer_position2d_geom_t(player_position2d_geom_t data) {
    Jplayer_position2d_geom_t Jdata = new Jplayer_position2d_geom_t();
    Jdata.pose = player_pose_t_to_Jplayer_pose_t(data.getPose());
    Jdata.size = player_bbox_t_to_Jplayer_bbox_t(data.getSize());
    return(Jdata);
  }

  public static player_position2d_geom_t Jplayer_position2d_geom_t_to_player_position2d_geom_t(Jplayer_position2d_geom_t Jdata) {
    player_position2d_geom_t data = new player_position2d_geom_t();
    data.setPose(Jplayer_pose_t_to_player_pose_t(Jdata.pose));
    data.setSize(Jplayer_bbox_t_to_player_bbox_t(Jdata.size));
    return(data);
  }

  public static Jplayer_position2d_power_config_t buf_to_Jplayer_position2d_power_config_t(SWIGTYPE_p_void buf) {
    player_position2d_power_config_t data = playercore_java.buf_to_player_position2d_power_config_t(buf);
    return(player_position2d_power_config_t_to_Jplayer_position2d_power_config_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_position2d_power_config_t_to_buf(Jplayer_position2d_power_config_t Jdata) {
    player_position2d_power_config_t data = Jplayer_position2d_power_config_t_to_player_position2d_power_config_t(Jdata);
    return(playercore_java.player_position2d_power_config_t_to_buf(data));
  }

  public static Jplayer_position2d_power_config_t player_position2d_power_config_t_to_Jplayer_position2d_power_config_t(player_position2d_power_config_t data) {
    Jplayer_position2d_power_config_t Jdata = new Jplayer_position2d_power_config_t();
    Jdata.state = data.getState();
    return(Jdata);
  }

  public static player_position2d_power_config_t Jplayer_position2d_power_config_t_to_player_position2d_power_config_t(Jplayer_position2d_power_config_t Jdata) {
    player_position2d_power_config_t data = new player_position2d_power_config_t();
    data.setState(Jdata.state);
    return(data);
  }

  public static Jplayer_position2d_velocity_mode_config_t buf_to_Jplayer_position2d_velocity_mode_config_t(SWIGTYPE_p_void buf) {
    player_position2d_velocity_mode_config_t data = playercore_java.buf_to_player_position2d_velocity_mode_config_t(buf);
    return(player_position2d_velocity_mode_config_t_to_Jplayer_position2d_velocity_mode_config_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_position2d_velocity_mode_config_t_to_buf(Jplayer_position2d_velocity_mode_config_t Jdata) {
    player_position2d_velocity_mode_config_t data = Jplayer_position2d_velocity_mode_config_t_to_player_position2d_velocity_mode_config_t(Jdata);
    return(playercore_java.player_position2d_velocity_mode_config_t_to_buf(data));
  }

  public static Jplayer_position2d_velocity_mode_config_t player_position2d_velocity_mode_config_t_to_Jplayer_position2d_velocity_mode_config_t(player_position2d_velocity_mode_config_t data) {
    Jplayer_position2d_velocity_mode_config_t Jdata = new Jplayer_position2d_velocity_mode_config_t();
    Jdata.value = data.getValue();
    return(Jdata);
  }

  public static player_position2d_velocity_mode_config_t Jplayer_position2d_velocity_mode_config_t_to_player_position2d_velocity_mode_config_t(Jplayer_position2d_velocity_mode_config_t Jdata) {
    player_position2d_velocity_mode_config_t data = new player_position2d_velocity_mode_config_t();
    data.setValue(Jdata.value);
    return(data);
  }

  public static Jplayer_position2d_reset_odom_config_t buf_to_Jplayer_position2d_reset_odom_config_t(SWIGTYPE_p_void buf) {
    player_position2d_reset_odom_config_t data = playercore_java.buf_to_player_position2d_reset_odom_config_t(buf);
    return(player_position2d_reset_odom_config_t_to_Jplayer_position2d_reset_odom_config_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_position2d_reset_odom_config_t_to_buf(Jplayer_position2d_reset_odom_config_t Jdata) {
    player_position2d_reset_odom_config_t data = Jplayer_position2d_reset_odom_config_t_to_player_position2d_reset_odom_config_t(Jdata);
    return(playercore_java.player_position2d_reset_odom_config_t_to_buf(data));
  }

  public static Jplayer_position2d_reset_odom_config_t player_position2d_reset_odom_config_t_to_Jplayer_position2d_reset_odom_config_t(player_position2d_reset_odom_config_t data) {
    Jplayer_position2d_reset_odom_config_t Jdata = new Jplayer_position2d_reset_odom_config_t();
    return(Jdata);
  }

  public static player_position2d_reset_odom_config_t Jplayer_position2d_reset_odom_config_t_to_player_position2d_reset_odom_config_t(Jplayer_position2d_reset_odom_config_t Jdata) {
    player_position2d_reset_odom_config_t data = new player_position2d_reset_odom_config_t();
    return(data);
  }

  public static Jplayer_position2d_position_mode_req_t buf_to_Jplayer_position2d_position_mode_req_t(SWIGTYPE_p_void buf) {
    player_position2d_position_mode_req_t data = playercore_java.buf_to_player_position2d_position_mode_req_t(buf);
    return(player_position2d_position_mode_req_t_to_Jplayer_position2d_position_mode_req_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_position2d_position_mode_req_t_to_buf(Jplayer_position2d_position_mode_req_t Jdata) {
    player_position2d_position_mode_req_t data = Jplayer_position2d_position_mode_req_t_to_player_position2d_position_mode_req_t(Jdata);
    return(playercore_java.player_position2d_position_mode_req_t_to_buf(data));
  }

  public static Jplayer_position2d_position_mode_req_t player_position2d_position_mode_req_t_to_Jplayer_position2d_position_mode_req_t(player_position2d_position_mode_req_t data) {
    Jplayer_position2d_position_mode_req_t Jdata = new Jplayer_position2d_position_mode_req_t();
    Jdata.state = data.getState();
    return(Jdata);
  }

  public static player_position2d_position_mode_req_t Jplayer_position2d_position_mode_req_t_to_player_position2d_position_mode_req_t(Jplayer_position2d_position_mode_req_t Jdata) {
    player_position2d_position_mode_req_t data = new player_position2d_position_mode_req_t();
    data.setState(Jdata.state);
    return(data);
  }

  public static Jplayer_position2d_set_odom_req_t buf_to_Jplayer_position2d_set_odom_req_t(SWIGTYPE_p_void buf) {
    player_position2d_set_odom_req_t data = playercore_java.buf_to_player_position2d_set_odom_req_t(buf);
    return(player_position2d_set_odom_req_t_to_Jplayer_position2d_set_odom_req_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_position2d_set_odom_req_t_to_buf(Jplayer_position2d_set_odom_req_t Jdata) {
    player_position2d_set_odom_req_t data = Jplayer_position2d_set_odom_req_t_to_player_position2d_set_odom_req_t(Jdata);
    return(playercore_java.player_position2d_set_odom_req_t_to_buf(data));
  }

  public static Jplayer_position2d_set_odom_req_t player_position2d_set_odom_req_t_to_Jplayer_position2d_set_odom_req_t(player_position2d_set_odom_req_t data) {
    Jplayer_position2d_set_odom_req_t Jdata = new Jplayer_position2d_set_odom_req_t();
    Jdata.pose = player_pose_t_to_Jplayer_pose_t(data.getPose());
    return(Jdata);
  }

  public static player_position2d_set_odom_req_t Jplayer_position2d_set_odom_req_t_to_player_position2d_set_odom_req_t(Jplayer_position2d_set_odom_req_t Jdata) {
    player_position2d_set_odom_req_t data = new player_position2d_set_odom_req_t();
    data.setPose(Jplayer_pose_t_to_player_pose_t(Jdata.pose));
    return(data);
  }

  public static Jplayer_position2d_speed_pid_req_t buf_to_Jplayer_position2d_speed_pid_req_t(SWIGTYPE_p_void buf) {
    player_position2d_speed_pid_req_t data = playercore_java.buf_to_player_position2d_speed_pid_req_t(buf);
    return(player_position2d_speed_pid_req_t_to_Jplayer_position2d_speed_pid_req_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_position2d_speed_pid_req_t_to_buf(Jplayer_position2d_speed_pid_req_t Jdata) {
    player_position2d_speed_pid_req_t data = Jplayer_position2d_speed_pid_req_t_to_player_position2d_speed_pid_req_t(Jdata);
    return(playercore_java.player_position2d_speed_pid_req_t_to_buf(data));
  }

  public static Jplayer_position2d_speed_pid_req_t player_position2d_speed_pid_req_t_to_Jplayer_position2d_speed_pid_req_t(player_position2d_speed_pid_req_t data) {
    Jplayer_position2d_speed_pid_req_t Jdata = new Jplayer_position2d_speed_pid_req_t();
    Jdata.kp = data.getKp();
    Jdata.ki = data.getKi();
    Jdata.kd = data.getKd();
    return(Jdata);
  }

  public static player_position2d_speed_pid_req_t Jplayer_position2d_speed_pid_req_t_to_player_position2d_speed_pid_req_t(Jplayer_position2d_speed_pid_req_t Jdata) {
    player_position2d_speed_pid_req_t data = new player_position2d_speed_pid_req_t();
    data.setKp(Jdata.kp);
    data.setKi(Jdata.ki);
    data.setKd(Jdata.kd);
    return(data);
  }

  public static Jplayer_position2d_position_pid_req_t buf_to_Jplayer_position2d_position_pid_req_t(SWIGTYPE_p_void buf) {
    player_position2d_position_pid_req_t data = playercore_java.buf_to_player_position2d_position_pid_req_t(buf);
    return(player_position2d_position_pid_req_t_to_Jplayer_position2d_position_pid_req_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_position2d_position_pid_req_t_to_buf(Jplayer_position2d_position_pid_req_t Jdata) {
    player_position2d_position_pid_req_t data = Jplayer_position2d_position_pid_req_t_to_player_position2d_position_pid_req_t(Jdata);
    return(playercore_java.player_position2d_position_pid_req_t_to_buf(data));
  }

  public static Jplayer_position2d_position_pid_req_t player_position2d_position_pid_req_t_to_Jplayer_position2d_position_pid_req_t(player_position2d_position_pid_req_t data) {
    Jplayer_position2d_position_pid_req_t Jdata = new Jplayer_position2d_position_pid_req_t();
    Jdata.kp = data.getKp();
    Jdata.ki = data.getKi();
    Jdata.kd = data.getKd();
    return(Jdata);
  }

  public static player_position2d_position_pid_req_t Jplayer_position2d_position_pid_req_t_to_player_position2d_position_pid_req_t(Jplayer_position2d_position_pid_req_t Jdata) {
    player_position2d_position_pid_req_t data = new player_position2d_position_pid_req_t();
    data.setKp(Jdata.kp);
    data.setKi(Jdata.ki);
    data.setKd(Jdata.kd);
    return(data);
  }

  public static Jplayer_position2d_speed_prof_req_t buf_to_Jplayer_position2d_speed_prof_req_t(SWIGTYPE_p_void buf) {
    player_position2d_speed_prof_req_t data = playercore_java.buf_to_player_position2d_speed_prof_req_t(buf);
    return(player_position2d_speed_prof_req_t_to_Jplayer_position2d_speed_prof_req_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_position2d_speed_prof_req_t_to_buf(Jplayer_position2d_speed_prof_req_t Jdata) {
    player_position2d_speed_prof_req_t data = Jplayer_position2d_speed_prof_req_t_to_player_position2d_speed_prof_req_t(Jdata);
    return(playercore_java.player_position2d_speed_prof_req_t_to_buf(data));
  }

  public static Jplayer_position2d_speed_prof_req_t player_position2d_speed_prof_req_t_to_Jplayer_position2d_speed_prof_req_t(player_position2d_speed_prof_req_t data) {
    Jplayer_position2d_speed_prof_req_t Jdata = new Jplayer_position2d_speed_prof_req_t();
    Jdata.speed = data.getSpeed();
    Jdata.acc = data.getAcc();
    return(Jdata);
  }

  public static player_position2d_speed_prof_req_t Jplayer_position2d_speed_prof_req_t_to_player_position2d_speed_prof_req_t(Jplayer_position2d_speed_prof_req_t Jdata) {
    player_position2d_speed_prof_req_t data = new player_position2d_speed_prof_req_t();
    data.setSpeed(Jdata.speed);
    data.setAcc(Jdata.acc);
    return(data);
  }

  public static Jplayer_position3d_data_t buf_to_Jplayer_position3d_data_t(SWIGTYPE_p_void buf) {
    player_position3d_data_t data = playercore_java.buf_to_player_position3d_data_t(buf);
    return(player_position3d_data_t_to_Jplayer_position3d_data_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_position3d_data_t_to_buf(Jplayer_position3d_data_t Jdata) {
    player_position3d_data_t data = Jplayer_position3d_data_t_to_player_position3d_data_t(Jdata);
    return(playercore_java.player_position3d_data_t_to_buf(data));
  }

  public static Jplayer_position3d_data_t player_position3d_data_t_to_Jplayer_position3d_data_t(player_position3d_data_t data) {
    Jplayer_position3d_data_t Jdata = new Jplayer_position3d_data_t();
    Jdata.pos = player_pose3d_t_to_Jplayer_pose3d_t(data.getPos());
    Jdata.vel = player_pose3d_t_to_Jplayer_pose3d_t(data.getVel());
    Jdata.stall = data.getStall();
    return(Jdata);
  }

  public static player_position3d_data_t Jplayer_position3d_data_t_to_player_position3d_data_t(Jplayer_position3d_data_t Jdata) {
    player_position3d_data_t data = new player_position3d_data_t();
    data.setPos(Jplayer_pose3d_t_to_player_pose3d_t(Jdata.pos));
    data.setVel(Jplayer_pose3d_t_to_player_pose3d_t(Jdata.vel));
    data.setStall(Jdata.stall);
    return(data);
  }

  public static Jplayer_position3d_cmd_pos_t buf_to_Jplayer_position3d_cmd_pos_t(SWIGTYPE_p_void buf) {
    player_position3d_cmd_pos_t data = playercore_java.buf_to_player_position3d_cmd_pos_t(buf);
    return(player_position3d_cmd_pos_t_to_Jplayer_position3d_cmd_pos_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_position3d_cmd_pos_t_to_buf(Jplayer_position3d_cmd_pos_t Jdata) {
    player_position3d_cmd_pos_t data = Jplayer_position3d_cmd_pos_t_to_player_position3d_cmd_pos_t(Jdata);
    return(playercore_java.player_position3d_cmd_pos_t_to_buf(data));
  }

  public static Jplayer_position3d_cmd_pos_t player_position3d_cmd_pos_t_to_Jplayer_position3d_cmd_pos_t(player_position3d_cmd_pos_t data) {
    Jplayer_position3d_cmd_pos_t Jdata = new Jplayer_position3d_cmd_pos_t();
    Jdata.pos = player_pose3d_t_to_Jplayer_pose3d_t(data.getPos());
    Jdata.vel = player_pose3d_t_to_Jplayer_pose3d_t(data.getVel());
    Jdata.state = data.getState();
    return(Jdata);
  }

  public static player_position3d_cmd_pos_t Jplayer_position3d_cmd_pos_t_to_player_position3d_cmd_pos_t(Jplayer_position3d_cmd_pos_t Jdata) {
    player_position3d_cmd_pos_t data = new player_position3d_cmd_pos_t();
    data.setPos(Jplayer_pose3d_t_to_player_pose3d_t(Jdata.pos));
    data.setVel(Jplayer_pose3d_t_to_player_pose3d_t(Jdata.vel));
    data.setState(Jdata.state);
    return(data);
  }

  public static Jplayer_position3d_cmd_vel_t buf_to_Jplayer_position3d_cmd_vel_t(SWIGTYPE_p_void buf) {
    player_position3d_cmd_vel_t data = playercore_java.buf_to_player_position3d_cmd_vel_t(buf);
    return(player_position3d_cmd_vel_t_to_Jplayer_position3d_cmd_vel_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_position3d_cmd_vel_t_to_buf(Jplayer_position3d_cmd_vel_t Jdata) {
    player_position3d_cmd_vel_t data = Jplayer_position3d_cmd_vel_t_to_player_position3d_cmd_vel_t(Jdata);
    return(playercore_java.player_position3d_cmd_vel_t_to_buf(data));
  }

  public static Jplayer_position3d_cmd_vel_t player_position3d_cmd_vel_t_to_Jplayer_position3d_cmd_vel_t(player_position3d_cmd_vel_t data) {
    Jplayer_position3d_cmd_vel_t Jdata = new Jplayer_position3d_cmd_vel_t();
    Jdata.vel = player_pose3d_t_to_Jplayer_pose3d_t(data.getVel());
    Jdata.state = data.getState();
    return(Jdata);
  }

  public static player_position3d_cmd_vel_t Jplayer_position3d_cmd_vel_t_to_player_position3d_cmd_vel_t(Jplayer_position3d_cmd_vel_t Jdata) {
    player_position3d_cmd_vel_t data = new player_position3d_cmd_vel_t();
    data.setVel(Jplayer_pose3d_t_to_player_pose3d_t(Jdata.vel));
    data.setState(Jdata.state);
    return(data);
  }

  public static Jplayer_position3d_geom_t buf_to_Jplayer_position3d_geom_t(SWIGTYPE_p_void buf) {
    player_position3d_geom_t data = playercore_java.buf_to_player_position3d_geom_t(buf);
    return(player_position3d_geom_t_to_Jplayer_position3d_geom_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_position3d_geom_t_to_buf(Jplayer_position3d_geom_t Jdata) {
    player_position3d_geom_t data = Jplayer_position3d_geom_t_to_player_position3d_geom_t(Jdata);
    return(playercore_java.player_position3d_geom_t_to_buf(data));
  }

  public static Jplayer_position3d_geom_t player_position3d_geom_t_to_Jplayer_position3d_geom_t(player_position3d_geom_t data) {
    Jplayer_position3d_geom_t Jdata = new Jplayer_position3d_geom_t();
    Jdata.pose = player_pose3d_t_to_Jplayer_pose3d_t(data.getPose());
    Jdata.size = player_bbox3d_t_to_Jplayer_bbox3d_t(data.getSize());
    return(Jdata);
  }

  public static player_position3d_geom_t Jplayer_position3d_geom_t_to_player_position3d_geom_t(Jplayer_position3d_geom_t Jdata) {
    player_position3d_geom_t data = new player_position3d_geom_t();
    data.setPose(Jplayer_pose3d_t_to_player_pose3d_t(Jdata.pose));
    data.setSize(Jplayer_bbox3d_t_to_player_bbox3d_t(Jdata.size));
    return(data);
  }

  public static Jplayer_position3d_power_config_t buf_to_Jplayer_position3d_power_config_t(SWIGTYPE_p_void buf) {
    player_position3d_power_config_t data = playercore_java.buf_to_player_position3d_power_config_t(buf);
    return(player_position3d_power_config_t_to_Jplayer_position3d_power_config_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_position3d_power_config_t_to_buf(Jplayer_position3d_power_config_t Jdata) {
    player_position3d_power_config_t data = Jplayer_position3d_power_config_t_to_player_position3d_power_config_t(Jdata);
    return(playercore_java.player_position3d_power_config_t_to_buf(data));
  }

  public static Jplayer_position3d_power_config_t player_position3d_power_config_t_to_Jplayer_position3d_power_config_t(player_position3d_power_config_t data) {
    Jplayer_position3d_power_config_t Jdata = new Jplayer_position3d_power_config_t();
    Jdata.state = data.getState();
    return(Jdata);
  }

  public static player_position3d_power_config_t Jplayer_position3d_power_config_t_to_player_position3d_power_config_t(Jplayer_position3d_power_config_t Jdata) {
    player_position3d_power_config_t data = new player_position3d_power_config_t();
    data.setState(Jdata.state);
    return(data);
  }

  public static Jplayer_position3d_position_mode_req_t buf_to_Jplayer_position3d_position_mode_req_t(SWIGTYPE_p_void buf) {
    player_position3d_position_mode_req_t data = playercore_java.buf_to_player_position3d_position_mode_req_t(buf);
    return(player_position3d_position_mode_req_t_to_Jplayer_position3d_position_mode_req_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_position3d_position_mode_req_t_to_buf(Jplayer_position3d_position_mode_req_t Jdata) {
    player_position3d_position_mode_req_t data = Jplayer_position3d_position_mode_req_t_to_player_position3d_position_mode_req_t(Jdata);
    return(playercore_java.player_position3d_position_mode_req_t_to_buf(data));
  }

  public static Jplayer_position3d_position_mode_req_t player_position3d_position_mode_req_t_to_Jplayer_position3d_position_mode_req_t(player_position3d_position_mode_req_t data) {
    Jplayer_position3d_position_mode_req_t Jdata = new Jplayer_position3d_position_mode_req_t();
    Jdata.value = data.getValue();
    return(Jdata);
  }

  public static player_position3d_position_mode_req_t Jplayer_position3d_position_mode_req_t_to_player_position3d_position_mode_req_t(Jplayer_position3d_position_mode_req_t Jdata) {
    player_position3d_position_mode_req_t data = new player_position3d_position_mode_req_t();
    data.setValue(Jdata.value);
    return(data);
  }

  public static Jplayer_position3d_velocity_mode_config_t buf_to_Jplayer_position3d_velocity_mode_config_t(SWIGTYPE_p_void buf) {
    player_position3d_velocity_mode_config_t data = playercore_java.buf_to_player_position3d_velocity_mode_config_t(buf);
    return(player_position3d_velocity_mode_config_t_to_Jplayer_position3d_velocity_mode_config_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_position3d_velocity_mode_config_t_to_buf(Jplayer_position3d_velocity_mode_config_t Jdata) {
    player_position3d_velocity_mode_config_t data = Jplayer_position3d_velocity_mode_config_t_to_player_position3d_velocity_mode_config_t(Jdata);
    return(playercore_java.player_position3d_velocity_mode_config_t_to_buf(data));
  }

  public static Jplayer_position3d_velocity_mode_config_t player_position3d_velocity_mode_config_t_to_Jplayer_position3d_velocity_mode_config_t(player_position3d_velocity_mode_config_t data) {
    Jplayer_position3d_velocity_mode_config_t Jdata = new Jplayer_position3d_velocity_mode_config_t();
    Jdata.value = data.getValue();
    return(Jdata);
  }

  public static player_position3d_velocity_mode_config_t Jplayer_position3d_velocity_mode_config_t_to_player_position3d_velocity_mode_config_t(Jplayer_position3d_velocity_mode_config_t Jdata) {
    player_position3d_velocity_mode_config_t data = new player_position3d_velocity_mode_config_t();
    data.setValue(Jdata.value);
    return(data);
  }

  public static Jplayer_position3d_set_odom_req_t buf_to_Jplayer_position3d_set_odom_req_t(SWIGTYPE_p_void buf) {
    player_position3d_set_odom_req_t data = playercore_java.buf_to_player_position3d_set_odom_req_t(buf);
    return(player_position3d_set_odom_req_t_to_Jplayer_position3d_set_odom_req_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_position3d_set_odom_req_t_to_buf(Jplayer_position3d_set_odom_req_t Jdata) {
    player_position3d_set_odom_req_t data = Jplayer_position3d_set_odom_req_t_to_player_position3d_set_odom_req_t(Jdata);
    return(playercore_java.player_position3d_set_odom_req_t_to_buf(data));
  }

  public static Jplayer_position3d_set_odom_req_t player_position3d_set_odom_req_t_to_Jplayer_position3d_set_odom_req_t(player_position3d_set_odom_req_t data) {
    Jplayer_position3d_set_odom_req_t Jdata = new Jplayer_position3d_set_odom_req_t();
    Jdata.pos = player_pose3d_t_to_Jplayer_pose3d_t(data.getPos());
    return(Jdata);
  }

  public static player_position3d_set_odom_req_t Jplayer_position3d_set_odom_req_t_to_player_position3d_set_odom_req_t(Jplayer_position3d_set_odom_req_t Jdata) {
    player_position3d_set_odom_req_t data = new player_position3d_set_odom_req_t();
    data.setPos(Jplayer_pose3d_t_to_player_pose3d_t(Jdata.pos));
    return(data);
  }

  public static Jplayer_position3d_reset_odom_config_t buf_to_Jplayer_position3d_reset_odom_config_t(SWIGTYPE_p_void buf) {
    player_position3d_reset_odom_config_t data = playercore_java.buf_to_player_position3d_reset_odom_config_t(buf);
    return(player_position3d_reset_odom_config_t_to_Jplayer_position3d_reset_odom_config_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_position3d_reset_odom_config_t_to_buf(Jplayer_position3d_reset_odom_config_t Jdata) {
    player_position3d_reset_odom_config_t data = Jplayer_position3d_reset_odom_config_t_to_player_position3d_reset_odom_config_t(Jdata);
    return(playercore_java.player_position3d_reset_odom_config_t_to_buf(data));
  }

  public static Jplayer_position3d_reset_odom_config_t player_position3d_reset_odom_config_t_to_Jplayer_position3d_reset_odom_config_t(player_position3d_reset_odom_config_t data) {
    Jplayer_position3d_reset_odom_config_t Jdata = new Jplayer_position3d_reset_odom_config_t();
    return(Jdata);
  }

  public static player_position3d_reset_odom_config_t Jplayer_position3d_reset_odom_config_t_to_player_position3d_reset_odom_config_t(Jplayer_position3d_reset_odom_config_t Jdata) {
    player_position3d_reset_odom_config_t data = new player_position3d_reset_odom_config_t();
    return(data);
  }

  public static Jplayer_position3d_speed_pid_req_t buf_to_Jplayer_position3d_speed_pid_req_t(SWIGTYPE_p_void buf) {
    player_position3d_speed_pid_req_t data = playercore_java.buf_to_player_position3d_speed_pid_req_t(buf);
    return(player_position3d_speed_pid_req_t_to_Jplayer_position3d_speed_pid_req_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_position3d_speed_pid_req_t_to_buf(Jplayer_position3d_speed_pid_req_t Jdata) {
    player_position3d_speed_pid_req_t data = Jplayer_position3d_speed_pid_req_t_to_player_position3d_speed_pid_req_t(Jdata);
    return(playercore_java.player_position3d_speed_pid_req_t_to_buf(data));
  }

  public static Jplayer_position3d_speed_pid_req_t player_position3d_speed_pid_req_t_to_Jplayer_position3d_speed_pid_req_t(player_position3d_speed_pid_req_t data) {
    Jplayer_position3d_speed_pid_req_t Jdata = new Jplayer_position3d_speed_pid_req_t();
    Jdata.kp = data.getKp();
    Jdata.ki = data.getKi();
    Jdata.kd = data.getKd();
    return(Jdata);
  }

  public static player_position3d_speed_pid_req_t Jplayer_position3d_speed_pid_req_t_to_player_position3d_speed_pid_req_t(Jplayer_position3d_speed_pid_req_t Jdata) {
    player_position3d_speed_pid_req_t data = new player_position3d_speed_pid_req_t();
    data.setKp(Jdata.kp);
    data.setKi(Jdata.ki);
    data.setKd(Jdata.kd);
    return(data);
  }

  public static Jplayer_position3d_position_pid_req_t buf_to_Jplayer_position3d_position_pid_req_t(SWIGTYPE_p_void buf) {
    player_position3d_position_pid_req_t data = playercore_java.buf_to_player_position3d_position_pid_req_t(buf);
    return(player_position3d_position_pid_req_t_to_Jplayer_position3d_position_pid_req_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_position3d_position_pid_req_t_to_buf(Jplayer_position3d_position_pid_req_t Jdata) {
    player_position3d_position_pid_req_t data = Jplayer_position3d_position_pid_req_t_to_player_position3d_position_pid_req_t(Jdata);
    return(playercore_java.player_position3d_position_pid_req_t_to_buf(data));
  }

  public static Jplayer_position3d_position_pid_req_t player_position3d_position_pid_req_t_to_Jplayer_position3d_position_pid_req_t(player_position3d_position_pid_req_t data) {
    Jplayer_position3d_position_pid_req_t Jdata = new Jplayer_position3d_position_pid_req_t();
    Jdata.kp = data.getKp();
    Jdata.ki = data.getKi();
    Jdata.kd = data.getKd();
    return(Jdata);
  }

  public static player_position3d_position_pid_req_t Jplayer_position3d_position_pid_req_t_to_player_position3d_position_pid_req_t(Jplayer_position3d_position_pid_req_t Jdata) {
    player_position3d_position_pid_req_t data = new player_position3d_position_pid_req_t();
    data.setKp(Jdata.kp);
    data.setKi(Jdata.ki);
    data.setKd(Jdata.kd);
    return(data);
  }

  public static Jplayer_position3d_speed_prof_req_t buf_to_Jplayer_position3d_speed_prof_req_t(SWIGTYPE_p_void buf) {
    player_position3d_speed_prof_req_t data = playercore_java.buf_to_player_position3d_speed_prof_req_t(buf);
    return(player_position3d_speed_prof_req_t_to_Jplayer_position3d_speed_prof_req_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_position3d_speed_prof_req_t_to_buf(Jplayer_position3d_speed_prof_req_t Jdata) {
    player_position3d_speed_prof_req_t data = Jplayer_position3d_speed_prof_req_t_to_player_position3d_speed_prof_req_t(Jdata);
    return(playercore_java.player_position3d_speed_prof_req_t_to_buf(data));
  }

  public static Jplayer_position3d_speed_prof_req_t player_position3d_speed_prof_req_t_to_Jplayer_position3d_speed_prof_req_t(player_position3d_speed_prof_req_t data) {
    Jplayer_position3d_speed_prof_req_t Jdata = new Jplayer_position3d_speed_prof_req_t();
    Jdata.speed = data.getSpeed();
    Jdata.acc = data.getAcc();
    return(Jdata);
  }

  public static player_position3d_speed_prof_req_t Jplayer_position3d_speed_prof_req_t_to_player_position3d_speed_prof_req_t(Jplayer_position3d_speed_prof_req_t Jdata) {
    player_position3d_speed_prof_req_t data = new player_position3d_speed_prof_req_t();
    data.setSpeed(Jdata.speed);
    data.setAcc(Jdata.acc);
    return(data);
  }

  public static Jplayer_power_data_t buf_to_Jplayer_power_data_t(SWIGTYPE_p_void buf) {
    player_power_data_t data = playercore_java.buf_to_player_power_data_t(buf);
    return(player_power_data_t_to_Jplayer_power_data_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_power_data_t_to_buf(Jplayer_power_data_t Jdata) {
    player_power_data_t data = Jplayer_power_data_t_to_player_power_data_t(Jdata);
    return(playercore_java.player_power_data_t_to_buf(data));
  }

  public static Jplayer_power_data_t player_power_data_t_to_Jplayer_power_data_t(player_power_data_t data) {
    Jplayer_power_data_t Jdata = new Jplayer_power_data_t();
    Jdata.valid = data.getValid();
    Jdata.volts = data.getVolts();
    Jdata.percent = data.getPercent();
    Jdata.joules = data.getJoules();
    Jdata.watts = data.getWatts();
    Jdata.charging = data.getCharging();
    return(Jdata);
  }

  public static player_power_data_t Jplayer_power_data_t_to_player_power_data_t(Jplayer_power_data_t Jdata) {
    player_power_data_t data = new player_power_data_t();
    data.setValid(Jdata.valid);
    data.setVolts(Jdata.volts);
    data.setPercent(Jdata.percent);
    data.setJoules(Jdata.joules);
    data.setWatts(Jdata.watts);
    data.setCharging(Jdata.charging);
    return(data);
  }

  public static Jplayer_power_chargepolicy_config_t buf_to_Jplayer_power_chargepolicy_config_t(SWIGTYPE_p_void buf) {
    player_power_chargepolicy_config_t data = playercore_java.buf_to_player_power_chargepolicy_config_t(buf);
    return(player_power_chargepolicy_config_t_to_Jplayer_power_chargepolicy_config_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_power_chargepolicy_config_t_to_buf(Jplayer_power_chargepolicy_config_t Jdata) {
    player_power_chargepolicy_config_t data = Jplayer_power_chargepolicy_config_t_to_player_power_chargepolicy_config_t(Jdata);
    return(playercore_java.player_power_chargepolicy_config_t_to_buf(data));
  }

  public static Jplayer_power_chargepolicy_config_t player_power_chargepolicy_config_t_to_Jplayer_power_chargepolicy_config_t(player_power_chargepolicy_config_t data) {
    Jplayer_power_chargepolicy_config_t Jdata = new Jplayer_power_chargepolicy_config_t();
    Jdata.enable_input = data.getEnable_input();
    Jdata.enable_output = data.getEnable_output();
    return(Jdata);
  }

  public static player_power_chargepolicy_config_t Jplayer_power_chargepolicy_config_t_to_player_power_chargepolicy_config_t(Jplayer_power_chargepolicy_config_t Jdata) {
    player_power_chargepolicy_config_t data = new player_power_chargepolicy_config_t();
    data.setEnable_input(Jdata.enable_input);
    data.setEnable_output(Jdata.enable_output);
    return(data);
  }

  public static Jplayer_ptz_data_t buf_to_Jplayer_ptz_data_t(SWIGTYPE_p_void buf) {
    player_ptz_data_t data = playercore_java.buf_to_player_ptz_data_t(buf);
    return(player_ptz_data_t_to_Jplayer_ptz_data_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_ptz_data_t_to_buf(Jplayer_ptz_data_t Jdata) {
    player_ptz_data_t data = Jplayer_ptz_data_t_to_player_ptz_data_t(Jdata);
    return(playercore_java.player_ptz_data_t_to_buf(data));
  }

  public static Jplayer_ptz_data_t player_ptz_data_t_to_Jplayer_ptz_data_t(player_ptz_data_t data) {
    Jplayer_ptz_data_t Jdata = new Jplayer_ptz_data_t();
    Jdata.pan = data.getPan();
    Jdata.tilt = data.getTilt();
    Jdata.zoom = data.getZoom();
    Jdata.panspeed = data.getPanspeed();
    Jdata.tiltspeed = data.getTiltspeed();
    return(Jdata);
  }

  public static player_ptz_data_t Jplayer_ptz_data_t_to_player_ptz_data_t(Jplayer_ptz_data_t Jdata) {
    player_ptz_data_t data = new player_ptz_data_t();
    data.setPan(Jdata.pan);
    data.setTilt(Jdata.tilt);
    data.setZoom(Jdata.zoom);
    data.setPanspeed(Jdata.panspeed);
    data.setTiltspeed(Jdata.tiltspeed);
    return(data);
  }

  public static Jplayer_ptz_cmd_t buf_to_Jplayer_ptz_cmd_t(SWIGTYPE_p_void buf) {
    player_ptz_cmd_t data = playercore_java.buf_to_player_ptz_cmd_t(buf);
    return(player_ptz_cmd_t_to_Jplayer_ptz_cmd_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_ptz_cmd_t_to_buf(Jplayer_ptz_cmd_t Jdata) {
    player_ptz_cmd_t data = Jplayer_ptz_cmd_t_to_player_ptz_cmd_t(Jdata);
    return(playercore_java.player_ptz_cmd_t_to_buf(data));
  }

  public static Jplayer_ptz_cmd_t player_ptz_cmd_t_to_Jplayer_ptz_cmd_t(player_ptz_cmd_t data) {
    Jplayer_ptz_cmd_t Jdata = new Jplayer_ptz_cmd_t();
    Jdata.pan = data.getPan();
    Jdata.tilt = data.getTilt();
    Jdata.zoom = data.getZoom();
    Jdata.panspeed = data.getPanspeed();
    Jdata.tiltspeed = data.getTiltspeed();
    return(Jdata);
  }

  public static player_ptz_cmd_t Jplayer_ptz_cmd_t_to_player_ptz_cmd_t(Jplayer_ptz_cmd_t Jdata) {
    player_ptz_cmd_t data = new player_ptz_cmd_t();
    data.setPan(Jdata.pan);
    data.setTilt(Jdata.tilt);
    data.setZoom(Jdata.zoom);
    data.setPanspeed(Jdata.panspeed);
    data.setTiltspeed(Jdata.tiltspeed);
    return(data);
  }

  public static Jplayer_ptz_geom_t buf_to_Jplayer_ptz_geom_t(SWIGTYPE_p_void buf) {
    player_ptz_geom_t data = playercore_java.buf_to_player_ptz_geom_t(buf);
    return(player_ptz_geom_t_to_Jplayer_ptz_geom_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_ptz_geom_t_to_buf(Jplayer_ptz_geom_t Jdata) {
    player_ptz_geom_t data = Jplayer_ptz_geom_t_to_player_ptz_geom_t(Jdata);
    return(playercore_java.player_ptz_geom_t_to_buf(data));
  }

  public static Jplayer_ptz_geom_t player_ptz_geom_t_to_Jplayer_ptz_geom_t(player_ptz_geom_t data) {
    Jplayer_ptz_geom_t Jdata = new Jplayer_ptz_geom_t();
    Jdata.pos = player_pose3d_t_to_Jplayer_pose3d_t(data.getPos());
    Jdata.size = player_bbox3d_t_to_Jplayer_bbox3d_t(data.getSize());
    return(Jdata);
  }

  public static player_ptz_geom_t Jplayer_ptz_geom_t_to_player_ptz_geom_t(Jplayer_ptz_geom_t Jdata) {
    player_ptz_geom_t data = new player_ptz_geom_t();
    data.setPos(Jplayer_pose3d_t_to_player_pose3d_t(Jdata.pos));
    data.setSize(Jplayer_bbox3d_t_to_player_bbox3d_t(Jdata.size));
    return(data);
  }

  public static Jplayer_ptz_req_generic_t buf_to_Jplayer_ptz_req_generic_t(SWIGTYPE_p_void buf) {
    player_ptz_req_generic_t data = playercore_java.buf_to_player_ptz_req_generic_t(buf);
    return(player_ptz_req_generic_t_to_Jplayer_ptz_req_generic_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_ptz_req_generic_t_to_buf(Jplayer_ptz_req_generic_t Jdata) {
    player_ptz_req_generic_t data = Jplayer_ptz_req_generic_t_to_player_ptz_req_generic_t(Jdata);
    return(playercore_java.player_ptz_req_generic_t_to_buf(data));
  }

  public static Jplayer_ptz_req_generic_t player_ptz_req_generic_t_to_Jplayer_ptz_req_generic_t(player_ptz_req_generic_t data) {
    Jplayer_ptz_req_generic_t Jdata = new Jplayer_ptz_req_generic_t();
    Jdata.config_count = data.getConfig_count();
    Jdata.config = data.getConfig();
    return(Jdata);
  }

  public static player_ptz_req_generic_t Jplayer_ptz_req_generic_t_to_player_ptz_req_generic_t(Jplayer_ptz_req_generic_t Jdata) {
    player_ptz_req_generic_t data = new player_ptz_req_generic_t();
    data.setConfig_count(Jdata.config_count);
    data.setConfig(Jdata.config);
    return(data);
  }

  public static Jplayer_ptz_req_control_mode_t buf_to_Jplayer_ptz_req_control_mode_t(SWIGTYPE_p_void buf) {
    player_ptz_req_control_mode_t data = playercore_java.buf_to_player_ptz_req_control_mode_t(buf);
    return(player_ptz_req_control_mode_t_to_Jplayer_ptz_req_control_mode_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_ptz_req_control_mode_t_to_buf(Jplayer_ptz_req_control_mode_t Jdata) {
    player_ptz_req_control_mode_t data = Jplayer_ptz_req_control_mode_t_to_player_ptz_req_control_mode_t(Jdata);
    return(playercore_java.player_ptz_req_control_mode_t_to_buf(data));
  }

  public static Jplayer_ptz_req_control_mode_t player_ptz_req_control_mode_t_to_Jplayer_ptz_req_control_mode_t(player_ptz_req_control_mode_t data) {
    Jplayer_ptz_req_control_mode_t Jdata = new Jplayer_ptz_req_control_mode_t();
    Jdata.mode = data.getMode();
    return(Jdata);
  }

  public static player_ptz_req_control_mode_t Jplayer_ptz_req_control_mode_t_to_player_ptz_req_control_mode_t(Jplayer_ptz_req_control_mode_t Jdata) {
    player_ptz_req_control_mode_t data = new player_ptz_req_control_mode_t();
    data.setMode(Jdata.mode);
    return(data);
  }

  public static Jplayer_simulation_data_t buf_to_Jplayer_simulation_data_t(SWIGTYPE_p_void buf) {
    player_simulation_data_t data = playercore_java.buf_to_player_simulation_data_t(buf);
    return(player_simulation_data_t_to_Jplayer_simulation_data_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_simulation_data_t_to_buf(Jplayer_simulation_data_t Jdata) {
    player_simulation_data_t data = Jplayer_simulation_data_t_to_player_simulation_data_t(Jdata);
    return(playercore_java.player_simulation_data_t_to_buf(data));
  }

  public static Jplayer_simulation_data_t player_simulation_data_t_to_Jplayer_simulation_data_t(player_simulation_data_t data) {
    Jplayer_simulation_data_t Jdata = new Jplayer_simulation_data_t();
    Jdata.data = data.getData();
    return(Jdata);
  }

  public static player_simulation_data_t Jplayer_simulation_data_t_to_player_simulation_data_t(Jplayer_simulation_data_t Jdata) {
    player_simulation_data_t data = new player_simulation_data_t();
    data.setData(Jdata.data);
    return(data);
  }

  public static Jplayer_simulation_cmd_t buf_to_Jplayer_simulation_cmd_t(SWIGTYPE_p_void buf) {
    player_simulation_cmd_t data = playercore_java.buf_to_player_simulation_cmd_t(buf);
    return(player_simulation_cmd_t_to_Jplayer_simulation_cmd_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_simulation_cmd_t_to_buf(Jplayer_simulation_cmd_t Jdata) {
    player_simulation_cmd_t data = Jplayer_simulation_cmd_t_to_player_simulation_cmd_t(Jdata);
    return(playercore_java.player_simulation_cmd_t_to_buf(data));
  }

  public static Jplayer_simulation_cmd_t player_simulation_cmd_t_to_Jplayer_simulation_cmd_t(player_simulation_cmd_t data) {
    Jplayer_simulation_cmd_t Jdata = new Jplayer_simulation_cmd_t();
    Jdata.cmd = data.getCmd();
    return(Jdata);
  }

  public static player_simulation_cmd_t Jplayer_simulation_cmd_t_to_player_simulation_cmd_t(Jplayer_simulation_cmd_t Jdata) {
    player_simulation_cmd_t data = new player_simulation_cmd_t();
    data.setCmd(Jdata.cmd);
    return(data);
  }

  public static Jplayer_simulation_pose2d_req_t buf_to_Jplayer_simulation_pose2d_req_t(SWIGTYPE_p_void buf) {
    player_simulation_pose2d_req_t data = playercore_java.buf_to_player_simulation_pose2d_req_t(buf);
    return(player_simulation_pose2d_req_t_to_Jplayer_simulation_pose2d_req_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_simulation_pose2d_req_t_to_buf(Jplayer_simulation_pose2d_req_t Jdata) {
    player_simulation_pose2d_req_t data = Jplayer_simulation_pose2d_req_t_to_player_simulation_pose2d_req_t(Jdata);
    return(playercore_java.player_simulation_pose2d_req_t_to_buf(data));
  }

  public static Jplayer_simulation_pose2d_req_t player_simulation_pose2d_req_t_to_Jplayer_simulation_pose2d_req_t(player_simulation_pose2d_req_t data) {
    Jplayer_simulation_pose2d_req_t Jdata = new Jplayer_simulation_pose2d_req_t();
    Jdata.name_count = data.getName_count();
    Jdata.name = data.getName();
    Jdata.pose = player_pose_t_to_Jplayer_pose_t(data.getPose());
    return(Jdata);
  }

  public static player_simulation_pose2d_req_t Jplayer_simulation_pose2d_req_t_to_player_simulation_pose2d_req_t(Jplayer_simulation_pose2d_req_t Jdata) {
    player_simulation_pose2d_req_t data = new player_simulation_pose2d_req_t();
    data.setName_count(Jdata.name_count);
    data.setName(Jdata.name);
    data.setPose(Jplayer_pose_t_to_player_pose_t(Jdata.pose));
    return(data);
  }

  public static Jplayer_simulation_property_int_req_t buf_to_Jplayer_simulation_property_int_req_t(SWIGTYPE_p_void buf) {
    player_simulation_property_int_req_t data = playercore_java.buf_to_player_simulation_property_int_req_t(buf);
    return(player_simulation_property_int_req_t_to_Jplayer_simulation_property_int_req_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_simulation_property_int_req_t_to_buf(Jplayer_simulation_property_int_req_t Jdata) {
    player_simulation_property_int_req_t data = Jplayer_simulation_property_int_req_t_to_player_simulation_property_int_req_t(Jdata);
    return(playercore_java.player_simulation_property_int_req_t_to_buf(data));
  }

  public static Jplayer_simulation_property_int_req_t player_simulation_property_int_req_t_to_Jplayer_simulation_property_int_req_t(player_simulation_property_int_req_t data) {
    Jplayer_simulation_property_int_req_t Jdata = new Jplayer_simulation_property_int_req_t();
    Jdata.name_count = data.getName_count();
    Jdata.name = data.getName();
    Jdata.prop_count = data.getProp_count();
    Jdata.prop = data.getProp();
    Jdata.value = data.getValue();
    return(Jdata);
  }

  public static player_simulation_property_int_req_t Jplayer_simulation_property_int_req_t_to_player_simulation_property_int_req_t(Jplayer_simulation_property_int_req_t Jdata) {
    player_simulation_property_int_req_t data = new player_simulation_property_int_req_t();
    data.setName_count(Jdata.name_count);
    data.setName(Jdata.name);
    data.setProp_count(Jdata.prop_count);
    data.setProp(Jdata.prop);
    data.setValue(Jdata.value);
    return(data);
  }

  public static Jplayer_simulation_property_float_req_t buf_to_Jplayer_simulation_property_float_req_t(SWIGTYPE_p_void buf) {
    player_simulation_property_float_req_t data = playercore_java.buf_to_player_simulation_property_float_req_t(buf);
    return(player_simulation_property_float_req_t_to_Jplayer_simulation_property_float_req_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_simulation_property_float_req_t_to_buf(Jplayer_simulation_property_float_req_t Jdata) {
    player_simulation_property_float_req_t data = Jplayer_simulation_property_float_req_t_to_player_simulation_property_float_req_t(Jdata);
    return(playercore_java.player_simulation_property_float_req_t_to_buf(data));
  }

  public static Jplayer_simulation_property_float_req_t player_simulation_property_float_req_t_to_Jplayer_simulation_property_float_req_t(player_simulation_property_float_req_t data) {
    Jplayer_simulation_property_float_req_t Jdata = new Jplayer_simulation_property_float_req_t();
    Jdata.name_count = data.getName_count();
    Jdata.name = data.getName();
    Jdata.prop_count = data.getProp_count();
    Jdata.prop = data.getProp();
    Jdata.value = data.getValue();
    return(Jdata);
  }

  public static player_simulation_property_float_req_t Jplayer_simulation_property_float_req_t_to_player_simulation_property_float_req_t(Jplayer_simulation_property_float_req_t Jdata) {
    player_simulation_property_float_req_t data = new player_simulation_property_float_req_t();
    data.setName_count(Jdata.name_count);
    data.setName(Jdata.name);
    data.setProp_count(Jdata.prop_count);
    data.setProp(Jdata.prop);
    data.setValue(Jdata.value);
    return(data);
  }

  public static Jplayer_simulation_property_string_req_t buf_to_Jplayer_simulation_property_string_req_t(SWIGTYPE_p_void buf) {
    player_simulation_property_string_req_t data = playercore_java.buf_to_player_simulation_property_string_req_t(buf);
    return(player_simulation_property_string_req_t_to_Jplayer_simulation_property_string_req_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_simulation_property_string_req_t_to_buf(Jplayer_simulation_property_string_req_t Jdata) {
    player_simulation_property_string_req_t data = Jplayer_simulation_property_string_req_t_to_player_simulation_property_string_req_t(Jdata);
    return(playercore_java.player_simulation_property_string_req_t_to_buf(data));
  }

  public static Jplayer_simulation_property_string_req_t player_simulation_property_string_req_t_to_Jplayer_simulation_property_string_req_t(player_simulation_property_string_req_t data) {
    Jplayer_simulation_property_string_req_t Jdata = new Jplayer_simulation_property_string_req_t();
    Jdata.name_count = data.getName_count();
    Jdata.name = data.getName();
    Jdata.prop_count = data.getProp_count();
    Jdata.prop = data.getProp();
    Jdata.value_count = data.getValue_count();
    Jdata.value = data.getValue();
    return(Jdata);
  }

  public static player_simulation_property_string_req_t Jplayer_simulation_property_string_req_t_to_player_simulation_property_string_req_t(Jplayer_simulation_property_string_req_t Jdata) {
    player_simulation_property_string_req_t data = new player_simulation_property_string_req_t();
    data.setName_count(Jdata.name_count);
    data.setName(Jdata.name);
    data.setProp_count(Jdata.prop_count);
    data.setProp(Jdata.prop);
    data.setValue_count(Jdata.value_count);
    data.setValue(Jdata.value);
    return(data);
  }

  public static Jplayer_sonar_data_t buf_to_Jplayer_sonar_data_t(SWIGTYPE_p_void buf) {
    player_sonar_data_t data = playercore_java.buf_to_player_sonar_data_t(buf);
    return(player_sonar_data_t_to_Jplayer_sonar_data_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_sonar_data_t_to_buf(Jplayer_sonar_data_t Jdata) {
    player_sonar_data_t data = Jplayer_sonar_data_t_to_player_sonar_data_t(Jdata);
    return(playercore_java.player_sonar_data_t_to_buf(data));
  }

  public static Jplayer_sonar_data_t player_sonar_data_t_to_Jplayer_sonar_data_t(player_sonar_data_t data) {
    Jplayer_sonar_data_t Jdata = new Jplayer_sonar_data_t();
    Jdata.ranges_count = data.getRanges_count();
    Jdata.ranges = data.getRanges();
    return(Jdata);
  }

  public static player_sonar_data_t Jplayer_sonar_data_t_to_player_sonar_data_t(Jplayer_sonar_data_t Jdata) {
    player_sonar_data_t data = new player_sonar_data_t();
    data.setRanges_count(Jdata.ranges_count);
    data.setRanges(Jdata.ranges);
    return(data);
  }

  public static Jplayer_sonar_geom_t buf_to_Jplayer_sonar_geom_t(SWIGTYPE_p_void buf) {
    player_sonar_geom_t data = playercore_java.buf_to_player_sonar_geom_t(buf);
    return(player_sonar_geom_t_to_Jplayer_sonar_geom_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_sonar_geom_t_to_buf(Jplayer_sonar_geom_t Jdata) {
    player_sonar_geom_t data = Jplayer_sonar_geom_t_to_player_sonar_geom_t(Jdata);
    return(playercore_java.player_sonar_geom_t_to_buf(data));
  }

  public static Jplayer_sonar_geom_t player_sonar_geom_t_to_Jplayer_sonar_geom_t(player_sonar_geom_t data) {
    Jplayer_sonar_geom_t Jdata = new Jplayer_sonar_geom_t();
    Jdata.poses_count = data.getPoses_count();
    {
      player_pose_t foo[] = data.getPoses();
      for(int i=0;i<playercore_javaConstants.PLAYER_SONAR_MAX_SAMPLES;i++)
        Jdata.poses[i] = player_pose_t_to_Jplayer_pose_t(foo[i]);
    }
    return(Jdata);
  }

  public static player_sonar_geom_t Jplayer_sonar_geom_t_to_player_sonar_geom_t(Jplayer_sonar_geom_t Jdata) {
    player_sonar_geom_t data = new player_sonar_geom_t();
    data.setPoses_count(Jdata.poses_count);
    {
      player_pose_t foo[] = new player_pose_t[playercore_javaConstants.PLAYER_SONAR_MAX_SAMPLES];
      for(int i=0;i<playercore_javaConstants.PLAYER_SONAR_MAX_SAMPLES;i++)
        foo[i] = Jplayer_pose_t_to_player_pose_t(Jdata.poses[i]);
      data.setPoses(foo);
    }
    return(data);
  }

  public static Jplayer_sonar_power_config_t buf_to_Jplayer_sonar_power_config_t(SWIGTYPE_p_void buf) {
    player_sonar_power_config_t data = playercore_java.buf_to_player_sonar_power_config_t(buf);
    return(player_sonar_power_config_t_to_Jplayer_sonar_power_config_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_sonar_power_config_t_to_buf(Jplayer_sonar_power_config_t Jdata) {
    player_sonar_power_config_t data = Jplayer_sonar_power_config_t_to_player_sonar_power_config_t(Jdata);
    return(playercore_java.player_sonar_power_config_t_to_buf(data));
  }

  public static Jplayer_sonar_power_config_t player_sonar_power_config_t_to_Jplayer_sonar_power_config_t(player_sonar_power_config_t data) {
    Jplayer_sonar_power_config_t Jdata = new Jplayer_sonar_power_config_t();
    Jdata.state = data.getState();
    return(Jdata);
  }

  public static player_sonar_power_config_t Jplayer_sonar_power_config_t_to_player_sonar_power_config_t(Jplayer_sonar_power_config_t Jdata) {
    player_sonar_power_config_t data = new player_sonar_power_config_t();
    data.setState(Jdata.state);
    return(data);
  }

  public static Jplayer_sound_cmd_t buf_to_Jplayer_sound_cmd_t(SWIGTYPE_p_void buf) {
    player_sound_cmd_t data = playercore_java.buf_to_player_sound_cmd_t(buf);
    return(player_sound_cmd_t_to_Jplayer_sound_cmd_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_sound_cmd_t_to_buf(Jplayer_sound_cmd_t Jdata) {
    player_sound_cmd_t data = Jplayer_sound_cmd_t_to_player_sound_cmd_t(Jdata);
    return(playercore_java.player_sound_cmd_t_to_buf(data));
  }

  public static Jplayer_sound_cmd_t player_sound_cmd_t_to_Jplayer_sound_cmd_t(player_sound_cmd_t data) {
    Jplayer_sound_cmd_t Jdata = new Jplayer_sound_cmd_t();
    Jdata.index = data.getIndex();
    return(Jdata);
  }

  public static player_sound_cmd_t Jplayer_sound_cmd_t_to_player_sound_cmd_t(Jplayer_sound_cmd_t Jdata) {
    player_sound_cmd_t data = new player_sound_cmd_t();
    data.setIndex(Jdata.index);
    return(data);
  }

  public static Jplayer_speech_cmd_t buf_to_Jplayer_speech_cmd_t(SWIGTYPE_p_void buf) {
    player_speech_cmd_t data = playercore_java.buf_to_player_speech_cmd_t(buf);
    return(player_speech_cmd_t_to_Jplayer_speech_cmd_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_speech_cmd_t_to_buf(Jplayer_speech_cmd_t Jdata) {
    player_speech_cmd_t data = Jplayer_speech_cmd_t_to_player_speech_cmd_t(Jdata);
    return(playercore_java.player_speech_cmd_t_to_buf(data));
  }

  public static Jplayer_speech_cmd_t player_speech_cmd_t_to_Jplayer_speech_cmd_t(player_speech_cmd_t data) {
    Jplayer_speech_cmd_t Jdata = new Jplayer_speech_cmd_t();
    Jdata.string_count = data.getString_count();
    Jdata.string = data.getString();
    return(Jdata);
  }

  public static player_speech_cmd_t Jplayer_speech_cmd_t_to_player_speech_cmd_t(Jplayer_speech_cmd_t Jdata) {
    player_speech_cmd_t data = new player_speech_cmd_t();
    data.setString_count(Jdata.string_count);
    data.setString(Jdata.string);
    return(data);
  }

  public static Jplayer_speech_recognition_data_t buf_to_Jplayer_speech_recognition_data_t(SWIGTYPE_p_void buf) {
    player_speech_recognition_data_t data = playercore_java.buf_to_player_speech_recognition_data_t(buf);
    return(player_speech_recognition_data_t_to_Jplayer_speech_recognition_data_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_speech_recognition_data_t_to_buf(Jplayer_speech_recognition_data_t Jdata) {
    player_speech_recognition_data_t data = Jplayer_speech_recognition_data_t_to_player_speech_recognition_data_t(Jdata);
    return(playercore_java.player_speech_recognition_data_t_to_buf(data));
  }

  public static Jplayer_speech_recognition_data_t player_speech_recognition_data_t_to_Jplayer_speech_recognition_data_t(player_speech_recognition_data_t data) {
    Jplayer_speech_recognition_data_t Jdata = new Jplayer_speech_recognition_data_t();
    Jdata.text_count = data.getText_count();
    Jdata.text = data.getText();
    return(Jdata);
  }

  public static player_speech_recognition_data_t Jplayer_speech_recognition_data_t_to_player_speech_recognition_data_t(Jplayer_speech_recognition_data_t Jdata) {
    player_speech_recognition_data_t data = new player_speech_recognition_data_t();
    data.setText_count(Jdata.text_count);
    data.setText(Jdata.text);
    return(data);
  }

  public static Jplayer_truth_pose_t buf_to_Jplayer_truth_pose_t(SWIGTYPE_p_void buf) {
    player_truth_pose_t data = playercore_java.buf_to_player_truth_pose_t(buf);
    return(player_truth_pose_t_to_Jplayer_truth_pose_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_truth_pose_t_to_buf(Jplayer_truth_pose_t Jdata) {
    player_truth_pose_t data = Jplayer_truth_pose_t_to_player_truth_pose_t(Jdata);
    return(playercore_java.player_truth_pose_t_to_buf(data));
  }

  public static Jplayer_truth_pose_t player_truth_pose_t_to_Jplayer_truth_pose_t(player_truth_pose_t data) {
    Jplayer_truth_pose_t Jdata = new Jplayer_truth_pose_t();
    Jdata.pose = player_pose3d_t_to_Jplayer_pose3d_t(data.getPose());
    return(Jdata);
  }

  public static player_truth_pose_t Jplayer_truth_pose_t_to_player_truth_pose_t(Jplayer_truth_pose_t Jdata) {
    player_truth_pose_t data = new player_truth_pose_t();
    data.setPose(Jplayer_pose3d_t_to_player_pose3d_t(Jdata.pose));
    return(data);
  }

  public static Jplayer_truth_fiducial_id_t buf_to_Jplayer_truth_fiducial_id_t(SWIGTYPE_p_void buf) {
    player_truth_fiducial_id_t data = playercore_java.buf_to_player_truth_fiducial_id_t(buf);
    return(player_truth_fiducial_id_t_to_Jplayer_truth_fiducial_id_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_truth_fiducial_id_t_to_buf(Jplayer_truth_fiducial_id_t Jdata) {
    player_truth_fiducial_id_t data = Jplayer_truth_fiducial_id_t_to_player_truth_fiducial_id_t(Jdata);
    return(playercore_java.player_truth_fiducial_id_t_to_buf(data));
  }

  public static Jplayer_truth_fiducial_id_t player_truth_fiducial_id_t_to_Jplayer_truth_fiducial_id_t(player_truth_fiducial_id_t data) {
    Jplayer_truth_fiducial_id_t Jdata = new Jplayer_truth_fiducial_id_t();
    Jdata.subtype = data.getSubtype();
    Jdata.id = data.getId();
    return(Jdata);
  }

  public static player_truth_fiducial_id_t Jplayer_truth_fiducial_id_t_to_player_truth_fiducial_id_t(Jplayer_truth_fiducial_id_t Jdata) {
    player_truth_fiducial_id_t data = new player_truth_fiducial_id_t();
    data.setSubtype(Jdata.subtype);
    data.setId(Jdata.id);
    return(data);
  }

  public static Jplayer_waveform_data_t buf_to_Jplayer_waveform_data_t(SWIGTYPE_p_void buf) {
    player_waveform_data_t data = playercore_java.buf_to_player_waveform_data_t(buf);
    return(player_waveform_data_t_to_Jplayer_waveform_data_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_waveform_data_t_to_buf(Jplayer_waveform_data_t Jdata) {
    player_waveform_data_t data = Jplayer_waveform_data_t_to_player_waveform_data_t(Jdata);
    return(playercore_java.player_waveform_data_t_to_buf(data));
  }

  public static Jplayer_waveform_data_t player_waveform_data_t_to_Jplayer_waveform_data_t(player_waveform_data_t data) {
    Jplayer_waveform_data_t Jdata = new Jplayer_waveform_data_t();
    Jdata.rate = data.getRate();
    Jdata.depth = data.getDepth();
    Jdata.data_count = data.getData_count();
    Jdata.data = data.getData();
    return(Jdata);
  }

  public static player_waveform_data_t Jplayer_waveform_data_t_to_player_waveform_data_t(Jplayer_waveform_data_t Jdata) {
    player_waveform_data_t data = new player_waveform_data_t();
    data.setRate(Jdata.rate);
    data.setDepth(Jdata.depth);
    data.setData_count(Jdata.data_count);
    data.setData(Jdata.data);
    return(data);
  }

  public static Jplayer_wifi_link_t buf_to_Jplayer_wifi_link_t(SWIGTYPE_p_void buf) {
    player_wifi_link_t data = playercore_java.buf_to_player_wifi_link_t(buf);
    return(player_wifi_link_t_to_Jplayer_wifi_link_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_wifi_link_t_to_buf(Jplayer_wifi_link_t Jdata) {
    player_wifi_link_t data = Jplayer_wifi_link_t_to_player_wifi_link_t(Jdata);
    return(playercore_java.player_wifi_link_t_to_buf(data));
  }

  public static Jplayer_wifi_link_t player_wifi_link_t_to_Jplayer_wifi_link_t(player_wifi_link_t data) {
    Jplayer_wifi_link_t Jdata = new Jplayer_wifi_link_t();
    Jdata.mac_count = data.getMac_count();
    Jdata.mac = data.getMac();
    Jdata.ip_count = data.getIp_count();
    Jdata.ip = data.getIp();
    Jdata.essid_count = data.getEssid_count();
    Jdata.essid = data.getEssid();
    Jdata.mode = data.getMode();
    Jdata.freq = data.getFreq();
    Jdata.encrypt = data.getEncrypt();
    Jdata.qual = data.getQual();
    Jdata.level = data.getLevel();
    Jdata.noise = data.getNoise();
    return(Jdata);
  }

  public static player_wifi_link_t Jplayer_wifi_link_t_to_player_wifi_link_t(Jplayer_wifi_link_t Jdata) {
    player_wifi_link_t data = new player_wifi_link_t();
    data.setMac_count(Jdata.mac_count);
    data.setMac(Jdata.mac);
    data.setIp_count(Jdata.ip_count);
    data.setIp(Jdata.ip);
    data.setEssid_count(Jdata.essid_count);
    data.setEssid(Jdata.essid);
    data.setMode(Jdata.mode);
    data.setFreq(Jdata.freq);
    data.setEncrypt(Jdata.encrypt);
    data.setQual(Jdata.qual);
    data.setLevel(Jdata.level);
    data.setNoise(Jdata.noise);
    return(data);
  }

  public static Jplayer_wifi_data_t buf_to_Jplayer_wifi_data_t(SWIGTYPE_p_void buf) {
    player_wifi_data_t data = playercore_java.buf_to_player_wifi_data_t(buf);
    return(player_wifi_data_t_to_Jplayer_wifi_data_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_wifi_data_t_to_buf(Jplayer_wifi_data_t Jdata) {
    player_wifi_data_t data = Jplayer_wifi_data_t_to_player_wifi_data_t(Jdata);
    return(playercore_java.player_wifi_data_t_to_buf(data));
  }

  public static Jplayer_wifi_data_t player_wifi_data_t_to_Jplayer_wifi_data_t(player_wifi_data_t data) {
    Jplayer_wifi_data_t Jdata = new Jplayer_wifi_data_t();
    Jdata.links_count = data.getLinks_count();
    {
      player_wifi_link_t foo[] = data.getLinks();
      for(int i=0;i<playercore_javaConstants.PLAYER_WIFI_MAX_LINKS;i++)
        Jdata.links[i] = player_wifi_link_t_to_Jplayer_wifi_link_t(foo[i]);
    }
    Jdata.throughput = data.getThroughput();
    Jdata.bitrate = data.getBitrate();
    Jdata.mode = data.getMode();
    Jdata.qual_type = data.getQual_type();
    Jdata.maxqual = data.getMaxqual();
    Jdata.maxlevel = data.getMaxlevel();
    Jdata.maxnoise = data.getMaxnoise();
    Jdata.ap = data.getAp();
    return(Jdata);
  }

  public static player_wifi_data_t Jplayer_wifi_data_t_to_player_wifi_data_t(Jplayer_wifi_data_t Jdata) {
    player_wifi_data_t data = new player_wifi_data_t();
    data.setLinks_count(Jdata.links_count);
    {
      player_wifi_link_t foo[] = new player_wifi_link_t[playercore_javaConstants.PLAYER_WIFI_MAX_LINKS];
      for(int i=0;i<playercore_javaConstants.PLAYER_WIFI_MAX_LINKS;i++)
        foo[i] = Jplayer_wifi_link_t_to_player_wifi_link_t(Jdata.links[i]);
      data.setLinks(foo);
    }
    data.setThroughput(Jdata.throughput);
    data.setBitrate(Jdata.bitrate);
    data.setMode(Jdata.mode);
    data.setQual_type(Jdata.qual_type);
    data.setMaxqual(Jdata.maxqual);
    data.setMaxlevel(Jdata.maxlevel);
    data.setMaxnoise(Jdata.maxnoise);
    data.setAp(Jdata.ap);
    return(data);
  }

  public static Jplayer_wifi_mac_req_t buf_to_Jplayer_wifi_mac_req_t(SWIGTYPE_p_void buf) {
    player_wifi_mac_req_t data = playercore_java.buf_to_player_wifi_mac_req_t(buf);
    return(player_wifi_mac_req_t_to_Jplayer_wifi_mac_req_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_wifi_mac_req_t_to_buf(Jplayer_wifi_mac_req_t Jdata) {
    player_wifi_mac_req_t data = Jplayer_wifi_mac_req_t_to_player_wifi_mac_req_t(Jdata);
    return(playercore_java.player_wifi_mac_req_t_to_buf(data));
  }

  public static Jplayer_wifi_mac_req_t player_wifi_mac_req_t_to_Jplayer_wifi_mac_req_t(player_wifi_mac_req_t data) {
    Jplayer_wifi_mac_req_t Jdata = new Jplayer_wifi_mac_req_t();
    Jdata.mac_count = data.getMac_count();
    Jdata.mac = data.getMac();
    return(Jdata);
  }

  public static player_wifi_mac_req_t Jplayer_wifi_mac_req_t_to_player_wifi_mac_req_t(Jplayer_wifi_mac_req_t Jdata) {
    player_wifi_mac_req_t data = new player_wifi_mac_req_t();
    data.setMac_count(Jdata.mac_count);
    data.setMac(Jdata.mac);
    return(data);
  }

  public static Jplayer_wifi_iwspy_addr_req_t buf_to_Jplayer_wifi_iwspy_addr_req_t(SWIGTYPE_p_void buf) {
    player_wifi_iwspy_addr_req_t data = playercore_java.buf_to_player_wifi_iwspy_addr_req_t(buf);
    return(player_wifi_iwspy_addr_req_t_to_Jplayer_wifi_iwspy_addr_req_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_wifi_iwspy_addr_req_t_to_buf(Jplayer_wifi_iwspy_addr_req_t Jdata) {
    player_wifi_iwspy_addr_req_t data = Jplayer_wifi_iwspy_addr_req_t_to_player_wifi_iwspy_addr_req_t(Jdata);
    return(playercore_java.player_wifi_iwspy_addr_req_t_to_buf(data));
  }

  public static Jplayer_wifi_iwspy_addr_req_t player_wifi_iwspy_addr_req_t_to_Jplayer_wifi_iwspy_addr_req_t(player_wifi_iwspy_addr_req_t data) {
    Jplayer_wifi_iwspy_addr_req_t Jdata = new Jplayer_wifi_iwspy_addr_req_t();
    Jdata.address = data.getAddress();
    return(Jdata);
  }

  public static player_wifi_iwspy_addr_req_t Jplayer_wifi_iwspy_addr_req_t_to_player_wifi_iwspy_addr_req_t(Jplayer_wifi_iwspy_addr_req_t Jdata) {
    player_wifi_iwspy_addr_req_t data = new player_wifi_iwspy_addr_req_t();
    data.setAddress(Jdata.address);
    return(data);
  }

  public static Jplayer_rfid_tag_t buf_to_Jplayer_rfid_tag_t(SWIGTYPE_p_void buf) {
    player_rfid_tag_t data = playercore_java.buf_to_player_rfid_tag_t(buf);
    return(player_rfid_tag_t_to_Jplayer_rfid_tag_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_rfid_tag_t_to_buf(Jplayer_rfid_tag_t Jdata) {
    player_rfid_tag_t data = Jplayer_rfid_tag_t_to_player_rfid_tag_t(Jdata);
    return(playercore_java.player_rfid_tag_t_to_buf(data));
  }

  public static Jplayer_rfid_tag_t player_rfid_tag_t_to_Jplayer_rfid_tag_t(player_rfid_tag_t data) {
    Jplayer_rfid_tag_t Jdata = new Jplayer_rfid_tag_t();
    Jdata.type = data.getType();
    Jdata.guid_count = data.getGuid_count();
    Jdata.guid = data.getGuid();
    return(Jdata);
  }

  public static player_rfid_tag_t Jplayer_rfid_tag_t_to_player_rfid_tag_t(Jplayer_rfid_tag_t Jdata) {
    player_rfid_tag_t data = new player_rfid_tag_t();
    data.setType(Jdata.type);
    data.setGuid_count(Jdata.guid_count);
    data.setGuid(Jdata.guid);
    return(data);
  }

  public static Jplayer_rfid_data_t buf_to_Jplayer_rfid_data_t(SWIGTYPE_p_void buf) {
    player_rfid_data_t data = playercore_java.buf_to_player_rfid_data_t(buf);
    return(player_rfid_data_t_to_Jplayer_rfid_data_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_rfid_data_t_to_buf(Jplayer_rfid_data_t Jdata) {
    player_rfid_data_t data = Jplayer_rfid_data_t_to_player_rfid_data_t(Jdata);
    return(playercore_java.player_rfid_data_t_to_buf(data));
  }

  public static Jplayer_rfid_data_t player_rfid_data_t_to_Jplayer_rfid_data_t(player_rfid_data_t data) {
    Jplayer_rfid_data_t Jdata = new Jplayer_rfid_data_t();
    Jdata.tags_count = data.getTags_count();
    {
      player_rfid_tag_t foo[] = data.getTags();
      for(int i=0;i<playercore_javaConstants.PLAYER_RFID_MAX_TAGS;i++)
        Jdata.tags[i] = player_rfid_tag_t_to_Jplayer_rfid_tag_t(foo[i]);
    }
    return(Jdata);
  }

  public static player_rfid_data_t Jplayer_rfid_data_t_to_player_rfid_data_t(Jplayer_rfid_data_t Jdata) {
    player_rfid_data_t data = new player_rfid_data_t();
    data.setTags_count(Jdata.tags_count);
    {
      player_rfid_tag_t foo[] = new player_rfid_tag_t[playercore_javaConstants.PLAYER_RFID_MAX_TAGS];
      for(int i=0;i<playercore_javaConstants.PLAYER_RFID_MAX_TAGS;i++)
        foo[i] = Jplayer_rfid_tag_t_to_player_rfid_tag_t(Jdata.tags[i]);
      data.setTags(foo);
    }
    return(data);
  }

  public static Jplayer_rfid_cmd_t buf_to_Jplayer_rfid_cmd_t(SWIGTYPE_p_void buf) {
    player_rfid_cmd_t data = playercore_java.buf_to_player_rfid_cmd_t(buf);
    return(player_rfid_cmd_t_to_Jplayer_rfid_cmd_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_rfid_cmd_t_to_buf(Jplayer_rfid_cmd_t Jdata) {
    player_rfid_cmd_t data = Jplayer_rfid_cmd_t_to_player_rfid_cmd_t(Jdata);
    return(playercore_java.player_rfid_cmd_t_to_buf(data));
  }

  public static Jplayer_rfid_cmd_t player_rfid_cmd_t_to_Jplayer_rfid_cmd_t(player_rfid_cmd_t data) {
    Jplayer_rfid_cmd_t Jdata = new Jplayer_rfid_cmd_t();
    Jdata.temp = data.getTemp();
    return(Jdata);
  }

  public static player_rfid_cmd_t Jplayer_rfid_cmd_t_to_player_rfid_cmd_t(Jplayer_rfid_cmd_t Jdata) {
    player_rfid_cmd_t data = new player_rfid_cmd_t();
    data.setTemp(Jdata.temp);
    return(data);
  }

  public static Jplayer_wsn_node_data_t buf_to_Jplayer_wsn_node_data_t(SWIGTYPE_p_void buf) {
    player_wsn_node_data_t data = playercore_java.buf_to_player_wsn_node_data_t(buf);
    return(player_wsn_node_data_t_to_Jplayer_wsn_node_data_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_wsn_node_data_t_to_buf(Jplayer_wsn_node_data_t Jdata) {
    player_wsn_node_data_t data = Jplayer_wsn_node_data_t_to_player_wsn_node_data_t(Jdata);
    return(playercore_java.player_wsn_node_data_t_to_buf(data));
  }

  public static Jplayer_wsn_node_data_t player_wsn_node_data_t_to_Jplayer_wsn_node_data_t(player_wsn_node_data_t data) {
    Jplayer_wsn_node_data_t Jdata = new Jplayer_wsn_node_data_t();
    Jdata.light = data.getLight();
    Jdata.mic = data.getMic();
    Jdata.accel_x = data.getAccel_x();
    Jdata.accel_y = data.getAccel_y();
    Jdata.accel_z = data.getAccel_z();
    Jdata.magn_x = data.getMagn_x();
    Jdata.magn_y = data.getMagn_y();
    Jdata.magn_z = data.getMagn_z();
    Jdata.temperature = data.getTemperature();
    Jdata.battery = data.getBattery();
    return(Jdata);
  }

  public static player_wsn_node_data_t Jplayer_wsn_node_data_t_to_player_wsn_node_data_t(Jplayer_wsn_node_data_t Jdata) {
    player_wsn_node_data_t data = new player_wsn_node_data_t();
    data.setLight(Jdata.light);
    data.setMic(Jdata.mic);
    data.setAccel_x(Jdata.accel_x);
    data.setAccel_y(Jdata.accel_y);
    data.setAccel_z(Jdata.accel_z);
    data.setMagn_x(Jdata.magn_x);
    data.setMagn_y(Jdata.magn_y);
    data.setMagn_z(Jdata.magn_z);
    data.setTemperature(Jdata.temperature);
    data.setBattery(Jdata.battery);
    return(data);
  }

  public static Jplayer_wsn_data_t buf_to_Jplayer_wsn_data_t(SWIGTYPE_p_void buf) {
    player_wsn_data_t data = playercore_java.buf_to_player_wsn_data_t(buf);
    return(player_wsn_data_t_to_Jplayer_wsn_data_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_wsn_data_t_to_buf(Jplayer_wsn_data_t Jdata) {
    player_wsn_data_t data = Jplayer_wsn_data_t_to_player_wsn_data_t(Jdata);
    return(playercore_java.player_wsn_data_t_to_buf(data));
  }

  public static Jplayer_wsn_data_t player_wsn_data_t_to_Jplayer_wsn_data_t(player_wsn_data_t data) {
    Jplayer_wsn_data_t Jdata = new Jplayer_wsn_data_t();
    Jdata.node_type = data.getNode_type();
    Jdata.node_id = data.getNode_id();
    Jdata.node_parent_id = data.getNode_parent_id();
    Jdata.data_packet = player_wsn_node_data_t_to_Jplayer_wsn_node_data_t(data.getData_packet());
    return(Jdata);
  }

  public static player_wsn_data_t Jplayer_wsn_data_t_to_player_wsn_data_t(Jplayer_wsn_data_t Jdata) {
    player_wsn_data_t data = new player_wsn_data_t();
    data.setNode_type(Jdata.node_type);
    data.setNode_id(Jdata.node_id);
    data.setNode_parent_id(Jdata.node_parent_id);
    data.setData_packet(Jplayer_wsn_node_data_t_to_player_wsn_node_data_t(Jdata.data_packet));
    return(data);
  }

  public static Jplayer_wsn_cmd_t buf_to_Jplayer_wsn_cmd_t(SWIGTYPE_p_void buf) {
    player_wsn_cmd_t data = playercore_java.buf_to_player_wsn_cmd_t(buf);
    return(player_wsn_cmd_t_to_Jplayer_wsn_cmd_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_wsn_cmd_t_to_buf(Jplayer_wsn_cmd_t Jdata) {
    player_wsn_cmd_t data = Jplayer_wsn_cmd_t_to_player_wsn_cmd_t(Jdata);
    return(playercore_java.player_wsn_cmd_t_to_buf(data));
  }

  public static Jplayer_wsn_cmd_t player_wsn_cmd_t_to_Jplayer_wsn_cmd_t(player_wsn_cmd_t data) {
    Jplayer_wsn_cmd_t Jdata = new Jplayer_wsn_cmd_t();
    Jdata.node_id = data.getNode_id();
    Jdata.group_id = data.getGroup_id();
    Jdata.device = data.getDevice();
    Jdata.enable = data.getEnable();
    return(Jdata);
  }

  public static player_wsn_cmd_t Jplayer_wsn_cmd_t_to_player_wsn_cmd_t(Jplayer_wsn_cmd_t Jdata) {
    player_wsn_cmd_t data = new player_wsn_cmd_t();
    data.setNode_id(Jdata.node_id);
    data.setGroup_id(Jdata.group_id);
    data.setDevice(Jdata.device);
    data.setEnable(Jdata.enable);
    return(data);
  }

  public static Jplayer_wsn_power_config_t buf_to_Jplayer_wsn_power_config_t(SWIGTYPE_p_void buf) {
    player_wsn_power_config_t data = playercore_java.buf_to_player_wsn_power_config_t(buf);
    return(player_wsn_power_config_t_to_Jplayer_wsn_power_config_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_wsn_power_config_t_to_buf(Jplayer_wsn_power_config_t Jdata) {
    player_wsn_power_config_t data = Jplayer_wsn_power_config_t_to_player_wsn_power_config_t(Jdata);
    return(playercore_java.player_wsn_power_config_t_to_buf(data));
  }

  public static Jplayer_wsn_power_config_t player_wsn_power_config_t_to_Jplayer_wsn_power_config_t(player_wsn_power_config_t data) {
    Jplayer_wsn_power_config_t Jdata = new Jplayer_wsn_power_config_t();
    Jdata.node_id = data.getNode_id();
    Jdata.group_id = data.getGroup_id();
    Jdata.value = data.getValue();
    return(Jdata);
  }

  public static player_wsn_power_config_t Jplayer_wsn_power_config_t_to_player_wsn_power_config_t(Jplayer_wsn_power_config_t Jdata) {
    player_wsn_power_config_t data = new player_wsn_power_config_t();
    data.setNode_id(Jdata.node_id);
    data.setGroup_id(Jdata.group_id);
    data.setValue(Jdata.value);
    return(data);
  }

  public static Jplayer_wsn_datatype_config_t buf_to_Jplayer_wsn_datatype_config_t(SWIGTYPE_p_void buf) {
    player_wsn_datatype_config_t data = playercore_java.buf_to_player_wsn_datatype_config_t(buf);
    return(player_wsn_datatype_config_t_to_Jplayer_wsn_datatype_config_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_wsn_datatype_config_t_to_buf(Jplayer_wsn_datatype_config_t Jdata) {
    player_wsn_datatype_config_t data = Jplayer_wsn_datatype_config_t_to_player_wsn_datatype_config_t(Jdata);
    return(playercore_java.player_wsn_datatype_config_t_to_buf(data));
  }

  public static Jplayer_wsn_datatype_config_t player_wsn_datatype_config_t_to_Jplayer_wsn_datatype_config_t(player_wsn_datatype_config_t data) {
    Jplayer_wsn_datatype_config_t Jdata = new Jplayer_wsn_datatype_config_t();
    Jdata.value = data.getValue();
    return(Jdata);
  }

  public static player_wsn_datatype_config_t Jplayer_wsn_datatype_config_t_to_player_wsn_datatype_config_t(Jplayer_wsn_datatype_config_t Jdata) {
    player_wsn_datatype_config_t data = new player_wsn_datatype_config_t();
    data.setValue(Jdata.value);
    return(data);
  }

  public static Jplayer_wsn_datafreq_config_t buf_to_Jplayer_wsn_datafreq_config_t(SWIGTYPE_p_void buf) {
    player_wsn_datafreq_config_t data = playercore_java.buf_to_player_wsn_datafreq_config_t(buf);
    return(player_wsn_datafreq_config_t_to_Jplayer_wsn_datafreq_config_t(data));
  }

  public static SWIGTYPE_p_void Jplayer_wsn_datafreq_config_t_to_buf(Jplayer_wsn_datafreq_config_t Jdata) {
    player_wsn_datafreq_config_t data = Jplayer_wsn_datafreq_config_t_to_player_wsn_datafreq_config_t(Jdata);
    return(playercore_java.player_wsn_datafreq_config_t_to_buf(data));
  }

  public static Jplayer_wsn_datafreq_config_t player_wsn_datafreq_config_t_to_Jplayer_wsn_datafreq_config_t(player_wsn_datafreq_config_t data) {
    Jplayer_wsn_datafreq_config_t Jdata = new Jplayer_wsn_datafreq_config_t();
    Jdata.node_id = data.getNode_id();
    Jdata.group_id = data.getGroup_id();
    Jdata.frequency = data.getFrequency();
    return(Jdata);
  }

  public static player_wsn_datafreq_config_t Jplayer_wsn_datafreq_config_t_to_player_wsn_datafreq_config_t(Jplayer_wsn_datafreq_config_t Jdata) {
    player_wsn_datafreq_config_t data = new player_wsn_datafreq_config_t();
    data.setNode_id(Jdata.node_id);
    data.setGroup_id(Jdata.group_id);
    data.setFrequency(Jdata.frequency);
    return(data);
  }


}
