package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_energy_data_t implements Serializable {
  public final static long serialVersionUID = 6446325389249309038L;
  public float joules;
  public float watts;
  public int charging;
  public Jplayer_energy_data_t() {
  }
}
