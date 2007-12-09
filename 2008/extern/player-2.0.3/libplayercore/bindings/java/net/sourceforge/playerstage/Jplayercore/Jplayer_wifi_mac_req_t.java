package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_wifi_mac_req_t implements Serializable {
  public final static long serialVersionUID = -4815158481838517622L;
  public long mac_count;
  public short[] mac;
  public Jplayer_wifi_mac_req_t() {
    mac = new short[32];
  }
}
