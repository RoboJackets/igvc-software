package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_power_data_t implements Serializable {
  public final static long serialVersionUID = -4737010291226811029L;
  public long valid;
  public float volts;
  public float percent;
  public float joules;
  public float watts;
  public int charging;
  public Jplayer_power_data_t() {
  }
}
