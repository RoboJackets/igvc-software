package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_position3d_cmd_vel_t implements Serializable {
  public final static long serialVersionUID = -2847068163257635285L;
  public Jplayer_pose3d_t vel;
  public short state;
  public Jplayer_position3d_cmd_vel_t() {
    vel = new Jplayer_pose3d_t();
  }
}
