package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_laser_data_t implements Serializable {
  public final static long serialVersionUID = -6285310959446000668L;
  public float min_angle;
  public float max_angle;
  public float resolution;
  public float max_range;
  public long ranges_count;
  public float[] ranges;
  public long intensity_count;
  public short[] intensity;
  public long id;
  public Jplayer_laser_data_t() {
    ranges = new float[playercore_javaConstants.PLAYER_LASER_MAX_SAMPLES];
    intensity = new short[playercore_javaConstants.PLAYER_LASER_MAX_SAMPLES];
  }
}
