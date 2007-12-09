package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_graphics3d_cmd_draw_t implements Serializable {
  public final static long serialVersionUID = 6489578772930683571L;
  public long draw_mode;
  public long points_count;
  public Jplayer_point_3d_t[] points;
  public Jplayer_color_t color;
  public Jplayer_graphics3d_cmd_draw_t() {
    points = new Jplayer_point_3d_t[playercore_javaConstants.PLAYER_GRAPHICS3D_MAX_POINTS];
    color = new Jplayer_color_t();
  }
}
