package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_position3d_geom_t implements Serializable {
  public final static long serialVersionUID = 4950701408388418741L;
  public Jplayer_pose3d_t pose;
  public Jplayer_bbox3d_t size;
  public Jplayer_position3d_geom_t() {
    pose = new Jplayer_pose3d_t();
    size = new Jplayer_bbox3d_t();
  }
}
