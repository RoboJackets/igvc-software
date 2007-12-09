package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_localize_data_t implements Serializable {
  public final static long serialVersionUID = 8256725450095600071L;
  public long pending_count;
  public double pending_time;
  public long hypoths_count;
  public Jplayer_localize_hypoth_t[] hypoths;
  public Jplayer_localize_data_t() {
    hypoths = new Jplayer_localize_hypoth_t[playercore_javaConstants.PLAYER_LOCALIZE_MAX_HYPOTHS];
  }
}
