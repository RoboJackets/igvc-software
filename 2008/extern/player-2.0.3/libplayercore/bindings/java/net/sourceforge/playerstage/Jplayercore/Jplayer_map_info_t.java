package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_map_info_t implements Serializable {
  public final static long serialVersionUID = -2432263717789376722L;
  public float scale;
  public long width;
  public long height;
  public Jplayer_pose_t origin;
  public Jplayer_map_info_t() {
    origin = new Jplayer_pose_t();
  }
}
