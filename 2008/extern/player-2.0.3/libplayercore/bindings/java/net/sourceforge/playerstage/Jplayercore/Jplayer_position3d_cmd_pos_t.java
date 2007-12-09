package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_position3d_cmd_pos_t implements Serializable {
  public final static long serialVersionUID = -1393121575932884218L;
  public Jplayer_pose3d_t pos;
  public Jplayer_pose3d_t vel;
  public short state;
  public Jplayer_position3d_cmd_pos_t() {
    pos = new Jplayer_pose3d_t();
    vel = new Jplayer_pose3d_t();
  }
}
