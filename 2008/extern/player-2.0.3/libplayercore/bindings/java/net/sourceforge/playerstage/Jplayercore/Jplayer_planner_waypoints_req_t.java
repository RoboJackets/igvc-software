package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_planner_waypoints_req_t implements Serializable {
  public final static long serialVersionUID = -3739526779765080314L;
  public long waypoints_count;
  public Jplayer_pose_t[] waypoints;
  public Jplayer_planner_waypoints_req_t() {
    waypoints = new Jplayer_pose_t[playercore_javaConstants.PLAYER_PLANNER_MAX_WAYPOINTS];
  }
}
