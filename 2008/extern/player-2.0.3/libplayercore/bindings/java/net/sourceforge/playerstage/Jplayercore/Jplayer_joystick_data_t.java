package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_joystick_data_t implements Serializable {
  public final static long serialVersionUID = 7214210102564716533L;
  public int xpos;
  public int ypos;
  public int xscale;
  public int yscale;
  public long buttons;
  public Jplayer_joystick_data_t() {
  }
}
