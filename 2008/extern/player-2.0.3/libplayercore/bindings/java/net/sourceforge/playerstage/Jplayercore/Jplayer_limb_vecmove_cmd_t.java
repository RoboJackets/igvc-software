package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_limb_vecmove_cmd_t implements Serializable {
  public final static long serialVersionUID = 5390216937498196338L;
  public Jplayer_point_3d_t direction;
  public float length;
  public Jplayer_limb_vecmove_cmd_t() {
    direction = new Jplayer_point_3d_t();
  }
}
