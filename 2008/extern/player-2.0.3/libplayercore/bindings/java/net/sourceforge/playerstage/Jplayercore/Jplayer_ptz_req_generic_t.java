package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_ptz_req_generic_t implements Serializable {
  public final static long serialVersionUID = 6166216320939151950L;
  public long config_count;
  public long[] config;
  public Jplayer_ptz_req_generic_t() {
    config = new long[playercore_javaConstants.PLAYER_PTZ_MAX_CONFIG_LEN];
  }
}
