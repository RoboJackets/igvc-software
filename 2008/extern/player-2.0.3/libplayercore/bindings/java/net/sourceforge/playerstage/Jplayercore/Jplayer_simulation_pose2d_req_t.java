package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_simulation_pose2d_req_t implements Serializable {
  public final static long serialVersionUID = -152922728427465415L;
  public long name_count;
  public String name;
  public Jplayer_pose_t pose;
  public Jplayer_simulation_pose2d_req_t() {
    pose = new Jplayer_pose_t();
  }
}
