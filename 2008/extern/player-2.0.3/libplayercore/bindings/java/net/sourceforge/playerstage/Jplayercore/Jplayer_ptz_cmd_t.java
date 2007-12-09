package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_ptz_cmd_t implements Serializable {
  public final static long serialVersionUID = -4015699126092083649L;
  public float pan;
  public float tilt;
  public float zoom;
  public float panspeed;
  public float tiltspeed;
  public Jplayer_ptz_cmd_t() {
  }
}
