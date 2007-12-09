package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_audiomixer_config_t implements Serializable {
  public final static long serialVersionUID = 1369929818460224299L;
  public long master_left;
  public long master_right;
  public long pcm_left;
  public long pcm_right;
  public long line_left;
  public long line_right;
  public long mic_left;
  public long mic_right;
  public long i_gain;
  public long o_gain;
  public Jplayer_audiomixer_config_t() {
  }
}
