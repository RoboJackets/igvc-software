package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_planner_cmd_t implements Serializable {
  public final static long serialVersionUID = 1797933160250639863L;
  public Jplayer_pose_t goal;
  public Jplayer_planner_cmd_t() {
    goal = new Jplayer_pose_t();
  }
}
