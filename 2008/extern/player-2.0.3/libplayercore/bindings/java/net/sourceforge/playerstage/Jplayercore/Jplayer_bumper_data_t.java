package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_bumper_data_t implements Serializable {
  public final static long serialVersionUID = -3232251317751291422L;
  public long bumpers_count;
  public short[] bumpers;
  public Jplayer_bumper_data_t() {
    bumpers = new short[playercore_javaConstants.PLAYER_BUMPER_MAX_SAMPLES];
  }
}
