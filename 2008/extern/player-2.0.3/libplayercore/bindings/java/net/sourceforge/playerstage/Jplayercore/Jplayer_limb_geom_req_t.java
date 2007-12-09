package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_limb_geom_req_t implements Serializable {
  public final static long serialVersionUID = 6615706895180678633L;
  public Jplayer_point_3d_t basePos;
  public Jplayer_limb_geom_req_t() {
    basePos = new Jplayer_point_3d_t();
  }
}
