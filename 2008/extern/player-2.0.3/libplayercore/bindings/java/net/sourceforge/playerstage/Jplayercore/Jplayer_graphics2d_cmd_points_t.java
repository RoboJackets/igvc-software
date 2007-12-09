package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_graphics2d_cmd_points_t implements Serializable {
  public final static long serialVersionUID = -2862922403100799681L;
  public int count;
  public Jplayer_point_2d_t[] points;
  public Jplayer_color_t color;
  public Jplayer_graphics2d_cmd_points_t() {
    points = new Jplayer_point_2d_t[playercore_javaConstants.PLAYER_GRAPHICS2D_MAX_POINTS];
    color = new Jplayer_color_t();
  }
}
