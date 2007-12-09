package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_sonar_data_t implements Serializable {
  public final static long serialVersionUID = 8573423395259471018L;
  public long ranges_count;
  public float[] ranges;
  public Jplayer_sonar_data_t() {
    ranges = new float[playercore_javaConstants.PLAYER_SONAR_MAX_SAMPLES];
  }
}
