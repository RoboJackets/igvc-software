package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_fiducial_data_t implements Serializable {
  public final static long serialVersionUID = -2708081753478439210L;
  public long fiducials_count;
  public Jplayer_fiducial_item_t[] fiducials;
  public Jplayer_fiducial_data_t() {
    fiducials = new Jplayer_fiducial_item_t[playercore_javaConstants.PLAYER_FIDUCIAL_MAX_SAMPLES];
  }
}
