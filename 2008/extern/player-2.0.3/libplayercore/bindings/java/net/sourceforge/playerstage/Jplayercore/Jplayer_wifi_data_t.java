package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_wifi_data_t implements Serializable {
  public final static long serialVersionUID = 5033768358350744583L;
  public long links_count;
  public Jplayer_wifi_link_t[] links;
  public long throughput;
  public long bitrate;
  public long mode;
  public long qual_type;
  public long maxqual;
  public long maxlevel;
  public long maxnoise;
  public String ap;
  public Jplayer_wifi_data_t() {
    links = new Jplayer_wifi_link_t[playercore_javaConstants.PLAYER_WIFI_MAX_LINKS];
  }
}
