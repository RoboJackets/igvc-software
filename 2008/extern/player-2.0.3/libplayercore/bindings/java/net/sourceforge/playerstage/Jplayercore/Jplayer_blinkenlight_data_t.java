package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_blinkenlight_data_t implements Serializable {
  public final static long serialVersionUID = -7919147919642511168L;
  public short enable;
  public float period;
  public float dutycycle;
  public Jplayer_color_t color;
  public Jplayer_blinkenlight_data_t() {
    color = new Jplayer_color_t();
  }
}
