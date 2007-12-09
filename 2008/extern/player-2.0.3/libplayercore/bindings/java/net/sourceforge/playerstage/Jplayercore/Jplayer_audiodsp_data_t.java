package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_audiodsp_data_t implements Serializable {
  public final static long serialVersionUID = 5279831977042972339L;
  public long frequency_count;
  public float[] frequency;
  public long amplitude_count;
  public float[] amplitude;
  public Jplayer_audiodsp_data_t() {
    frequency = new float[playercore_javaConstants.PLAYER_AUDIODSP_MAX_FREQS];
    amplitude = new float[playercore_javaConstants.PLAYER_AUDIODSP_MAX_FREQS];
  }
}
