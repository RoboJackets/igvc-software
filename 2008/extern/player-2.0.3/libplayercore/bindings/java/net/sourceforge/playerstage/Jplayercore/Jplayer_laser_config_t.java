package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_laser_config_t implements Serializable {
  public final static long serialVersionUID = -6098597951619470141L;
  public float min_angle;
  public float max_angle;
  public float resolution;
  public float max_range;
  public float range_res;
  public short intensity;
  public Jplayer_laser_config_t() {
  }
}
