package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_ptz_geom_t implements Serializable {
  public final static long serialVersionUID = -1745889328328455589L;
  public Jplayer_pose3d_t pos;
  public Jplayer_bbox3d_t size;
  public Jplayer_ptz_geom_t() {
    pos = new Jplayer_pose3d_t();
    size = new Jplayer_bbox3d_t();
  }
}
