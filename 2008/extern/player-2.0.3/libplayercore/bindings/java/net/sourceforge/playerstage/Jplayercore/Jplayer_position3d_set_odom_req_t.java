package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_position3d_set_odom_req_t implements Serializable {
  public final static long serialVersionUID = 1938780359631584044L;
  public Jplayer_pose3d_t pos;
  public Jplayer_position3d_set_odom_req_t() {
    pos = new Jplayer_pose3d_t();
  }
}
