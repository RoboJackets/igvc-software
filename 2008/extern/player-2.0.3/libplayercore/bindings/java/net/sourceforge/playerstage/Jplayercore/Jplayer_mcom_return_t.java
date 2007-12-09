package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_mcom_return_t implements Serializable {
  public final static long serialVersionUID = -2077217215783527391L;
  public long type;
  public long channel_count;
  public String channel;
  public Jplayer_mcom_data_t data;
  public Jplayer_mcom_return_t() {
    data = new Jplayer_mcom_data_t();
  }
}
