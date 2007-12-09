package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_device_driverinfo_t implements Serializable {
  public final static long serialVersionUID = -5912655578407101062L;
  public Jplayer_devaddr_t addr;
  public long driver_name_count;
  public String driver_name;
  public Jplayer_device_driverinfo_t() {
    addr = new Jplayer_devaddr_t();
  }
}
