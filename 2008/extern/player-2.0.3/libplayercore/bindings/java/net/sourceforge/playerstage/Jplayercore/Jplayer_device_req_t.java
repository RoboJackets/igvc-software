package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_device_req_t implements Serializable {
  public final static long serialVersionUID = 9065955501459268346L;
  public Jplayer_devaddr_t addr;
  public short access;
  public long driver_name_count;
  public String driver_name;
  public Jplayer_device_req_t() {
    addr = new Jplayer_devaddr_t();
  }
}
