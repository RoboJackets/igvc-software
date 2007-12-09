package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_limb_setpose_cmd_t implements Serializable {
  public final static long serialVersionUID = -7627336640665723475L;
  public Jplayer_point_3d_t position;
  public Jplayer_point_3d_t approach;
  public Jplayer_point_3d_t orientation;
  public Jplayer_limb_setpose_cmd_t() {
    position = new Jplayer_point_3d_t();
    approach = new Jplayer_point_3d_t();
    orientation = new Jplayer_point_3d_t();
  }
}
