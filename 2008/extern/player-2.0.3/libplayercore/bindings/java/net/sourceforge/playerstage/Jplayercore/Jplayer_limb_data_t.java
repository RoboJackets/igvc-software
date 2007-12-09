package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_limb_data_t implements Serializable {
  public final static long serialVersionUID = 1524772526837984291L;
  public Jplayer_point_3d_t position;
  public Jplayer_point_3d_t approach;
  public Jplayer_point_3d_t orientation;
  public short state;
  public Jplayer_limb_data_t() {
    position = new Jplayer_point_3d_t();
    approach = new Jplayer_point_3d_t();
    orientation = new Jplayer_point_3d_t();
  }
}
