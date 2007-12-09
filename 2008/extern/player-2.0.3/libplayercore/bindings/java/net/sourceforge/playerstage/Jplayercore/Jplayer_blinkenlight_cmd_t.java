package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_blinkenlight_cmd_t implements Serializable {
  public final static long serialVersionUID = 5567105422680416354L;
  public short enable;
  public float period;
  public float dutycycle;
  public Jplayer_color_t color;
  public Jplayer_blinkenlight_cmd_t() {
    color = new Jplayer_color_t();
  }
}
