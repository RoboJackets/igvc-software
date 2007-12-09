package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_sonar_geom_t implements Serializable {
  public final static long serialVersionUID = -4741091138404178884L;
  public long poses_count;
  public Jplayer_pose_t[] poses;
  public Jplayer_sonar_geom_t() {
    poses = new Jplayer_pose_t[playercore_javaConstants.PLAYER_SONAR_MAX_SAMPLES];
  }
}
