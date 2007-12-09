package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_device_devlist_t implements Serializable {
  public final static long serialVersionUID = 4765986890604472318L;
  public long devices_count;
  public Jplayer_devaddr_t[] devices;
  public Jplayer_device_devlist_t() {
    devices = new Jplayer_devaddr_t[playercore_javaConstants.PLAYER_MAX_DEVICES];
  }
}
