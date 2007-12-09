package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_graphics2d_cmd_polygon_t implements Serializable {
  public final static long serialVersionUID = 8757804046372823192L;
  public int count;
  public Jplayer_point_2d_t[] points;
  public Jplayer_color_t color;
  public Jplayer_color_t fill_color;
  public short filled;
  public Jplayer_graphics2d_cmd_polygon_t() {
    points = new Jplayer_point_2d_t[playercore_javaConstants.PLAYER_GRAPHICS2D_MAX_POINTS];
    color = new Jplayer_color_t();
    fill_color = new Jplayer_color_t();
  }
}
