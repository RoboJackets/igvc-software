package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_planner_data_t implements Serializable {
  public final static long serialVersionUID = -5434445389184760666L;
  public short valid;
  public short done;
  public Jplayer_pose_t pos;
  public Jplayer_pose_t goal;
  public Jplayer_pose_t waypoint;
  public int waypoint_idx;
  public long waypoints_count;
  public Jplayer_planner_data_t() {
    pos = new Jplayer_pose_t();
    goal = new Jplayer_pose_t();
    waypoint = new Jplayer_pose_t();
  }
}
