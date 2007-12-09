package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_actarray_data_t implements Serializable {
  public final static long serialVersionUID = -3526064040668594711L;
  public long actuators_count;
  public Jplayer_actarray_actuator_t[] actuators;
  public Jplayer_actarray_data_t() {
    actuators = new Jplayer_actarray_actuator_t[playercore_javaConstants.PLAYER_ACTARRAY_NUM_ACTUATORS];
  }
}
