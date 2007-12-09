package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_aio_data_t implements Serializable {
  public final static long serialVersionUID = 2903105377576264967L;
  public long voltages_count;
  public float[] voltages;
  public Jplayer_aio_data_t() {
    voltages = new float[playercore_javaConstants.PLAYER_AIO_MAX_INPUTS];
  }
}
