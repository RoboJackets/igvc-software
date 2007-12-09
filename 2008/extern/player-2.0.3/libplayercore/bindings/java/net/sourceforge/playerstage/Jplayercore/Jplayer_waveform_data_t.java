package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_waveform_data_t implements Serializable {
  public final static long serialVersionUID = -3922146846109904868L;
  public long rate;
  public long depth;
  public long data_count;
  public short[] data;
  public Jplayer_waveform_data_t() {
    data = new short[playercore_javaConstants.PLAYER_WAVEFORM_DATA_MAX];
  }
}
