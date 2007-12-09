package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_position2d_set_odom_req_t implements Serializable {
  public final static long serialVersionUID = 3241444100461226037L;
  public Jplayer_pose_t pose;
  public Jplayer_position2d_set_odom_req_t() {
    pose = new Jplayer_pose_t();
  }
}
