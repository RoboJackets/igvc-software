package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_bumper_geom_t implements Serializable {
  public final static long serialVersionUID = -8594336951190197038L;
  public long bumper_def_count;
  public Jplayer_bumper_define_t[] bumper_def;
  public Jplayer_bumper_geom_t() {
    bumper_def = new Jplayer_bumper_define_t[playercore_javaConstants.PLAYER_BUMPER_MAX_SAMPLES];
  }
}
