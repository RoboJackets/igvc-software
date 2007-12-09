package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_device_nameservice_req_t implements Serializable {
  public final static long serialVersionUID = 6896456571724211803L;
  public long name_count;
  public short[] name;
  public int port;
  public Jplayer_device_nameservice_req_t() {
    name = new short[playercore_javaConstants.PLAYER_MAX_DRIVER_STRING_LEN];
  }
}
