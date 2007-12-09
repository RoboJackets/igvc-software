package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_laser_data_scanpose_t implements Serializable {
  public final static long serialVersionUID = -5089181856442203960L;
  public Jplayer_laser_data_t scan;
  public Jplayer_pose_t pose;
  public Jplayer_laser_data_scanpose_t() {
    scan = new Jplayer_laser_data_t();
    pose = new Jplayer_pose_t();
  }
}
