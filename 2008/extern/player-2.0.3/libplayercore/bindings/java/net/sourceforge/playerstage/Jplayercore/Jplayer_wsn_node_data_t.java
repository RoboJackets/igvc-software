package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_wsn_node_data_t implements Serializable {
  public final static long serialVersionUID = -7554927025633191633L;
  public float light;
  public float mic;
  public float accel_x;
  public float accel_y;
  public float accel_z;
  public float magn_x;
  public float magn_y;
  public float magn_z;
  public float temperature;
  public float battery;
  public Jplayer_wsn_node_data_t() {
  }
}
