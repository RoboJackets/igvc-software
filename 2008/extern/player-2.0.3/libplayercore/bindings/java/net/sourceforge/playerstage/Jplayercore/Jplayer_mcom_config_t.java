package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_mcom_config_t implements Serializable {
  public final static long serialVersionUID = -2673390218259668801L;
  public long command;
  public long type;
  public long channel_count;
  public String channel;
  public Jplayer_mcom_data_t data;
  public Jplayer_mcom_config_t() {
    data = new Jplayer_mcom_data_t();
  }
}
