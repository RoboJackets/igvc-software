package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_position3d_data_t implements Serializable {
  public final static long serialVersionUID = 4373685322170572145L;
  public Jplayer_pose3d_t pos;
  public Jplayer_pose3d_t vel;
  public short stall;
  public Jplayer_position3d_data_t() {
    pos = new Jplayer_pose3d_t();
    vel = new Jplayer_pose3d_t();
  }
}
