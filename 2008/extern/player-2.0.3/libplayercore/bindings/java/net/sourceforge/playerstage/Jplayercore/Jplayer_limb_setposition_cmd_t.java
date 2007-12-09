package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_limb_setposition_cmd_t implements Serializable {
  public final static long serialVersionUID = -5381711282017263476L;
  public Jplayer_point_3d_t position;
  public Jplayer_limb_setposition_cmd_t() {
    position = new Jplayer_point_3d_t();
  }
}
