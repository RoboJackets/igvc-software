package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_graphics2d_cmd_polyline_t implements Serializable {
  public final static long serialVersionUID = 2046684828270892729L;
  public int count;
  public Jplayer_point_2d_t[] points;
  public Jplayer_color_t color;
  public Jplayer_graphics2d_cmd_polyline_t() {
    points = new Jplayer_point_2d_t[playercore_javaConstants.PLAYER_GRAPHICS2D_MAX_POINTS];
    color = new Jplayer_color_t();
  }
}
