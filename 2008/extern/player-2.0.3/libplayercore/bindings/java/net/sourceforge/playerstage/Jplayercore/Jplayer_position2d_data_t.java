package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_position2d_data_t implements Serializable {
  public final static long serialVersionUID = -4537529536897007341L;
  public Jplayer_pose_t pos;
  public Jplayer_pose_t vel;
  public short stall;
  public Jplayer_position2d_data_t() {
    pos = new Jplayer_pose_t();
    vel = new Jplayer_pose_t();
  }
}
