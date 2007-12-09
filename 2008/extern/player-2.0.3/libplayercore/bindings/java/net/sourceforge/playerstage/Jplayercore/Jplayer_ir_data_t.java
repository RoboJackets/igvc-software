package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_ir_data_t implements Serializable {
  public final static long serialVersionUID = -172414141783621279L;
  public long voltages_count;
  public float[] voltages;
  public long ranges_count;
  public float[] ranges;
  public Jplayer_ir_data_t() {
    voltages = new float[playercore_javaConstants.PLAYER_IR_MAX_SAMPLES];
    ranges = new float[playercore_javaConstants.PLAYER_IR_MAX_SAMPLES];
  }
}
