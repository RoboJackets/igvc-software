package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_position2d_cmd_vel_t implements Serializable {
  public final static long serialVersionUID = -5066842356070648748L;
  public Jplayer_pose_t vel;
  public short state;
  public Jplayer_position2d_cmd_vel_t() {
    vel = new Jplayer_pose_t();
  }
}
