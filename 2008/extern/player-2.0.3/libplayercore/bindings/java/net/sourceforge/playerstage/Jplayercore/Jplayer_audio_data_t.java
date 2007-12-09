package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_audio_data_t implements Serializable {
  public final static long serialVersionUID = 3440514618793873375L;
  public long frequency_count;
  public float[] frequency;
  public long amplitude_count;
  public float[] amplitude;
  public Jplayer_audio_data_t() {
    frequency = new float[playercore_javaConstants.PLAYER_AUDIO_PAIRS];
    amplitude = new float[playercore_javaConstants.PLAYER_AUDIO_PAIRS];
  }
}
