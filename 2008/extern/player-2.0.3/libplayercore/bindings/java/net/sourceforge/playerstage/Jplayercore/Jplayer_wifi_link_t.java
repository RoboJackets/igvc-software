package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_wifi_link_t implements Serializable {
  public final static long serialVersionUID = -7356790569087240944L;
  public long mac_count;
  public short[] mac;
  public long ip_count;
  public short[] ip;
  public long essid_count;
  public short[] essid;
  public long mode;
  public long freq;
  public long encrypt;
  public long qual;
  public long level;
  public long noise;
  public Jplayer_wifi_link_t() {
    mac = new short[32];
    ip = new short[32];
    essid = new short[32];
  }
}
