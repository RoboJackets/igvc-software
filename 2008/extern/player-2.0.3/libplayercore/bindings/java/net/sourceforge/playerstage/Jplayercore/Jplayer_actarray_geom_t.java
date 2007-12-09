package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_actarray_geom_t implements Serializable {
  public final static long serialVersionUID = 8020528305241934173L;
  public long actuators_count;
  public Jplayer_actarray_actuatorgeom_t[] actuators;
  public Jplayer_actarray_geom_t() {
    actuators = new Jplayer_actarray_actuatorgeom_t[playercore_javaConstants.PLAYER_ACTARRAY_NUM_ACTUATORS];
  }
}
