package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_device_auth_req_t implements Serializable {
  public final static long serialVersionUID = 3763057870427485308L;
  public long auth_key_count;
  public short[] auth_key;
  public Jplayer_device_auth_req_t() {
    auth_key = new short[playercore_javaConstants.PLAYER_KEYLEN];
  }
}
