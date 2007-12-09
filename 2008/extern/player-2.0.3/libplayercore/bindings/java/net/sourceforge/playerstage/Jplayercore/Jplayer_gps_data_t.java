package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_gps_data_t implements Serializable {
  public final static long serialVersionUID = 7361094404381128852L;
  public long time_sec;
  public long time_usec;
  public int latitude;
  public int longitude;
  public int altitude;
  public double utm_e;
  public double utm_n;
  public long quality;
  public long num_sats;
  public long hdop;
  public long vdop;
  public double err_horz;
  public double err_vert;
  public Jplayer_gps_data_t() {
  }
}
