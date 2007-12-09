package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_position2d_cmd_pos_t implements Serializable {
  public final static long serialVersionUID = -3377445525447841006L;
  public Jplayer_pose_t pos;
  public Jplayer_pose_t vel;
  public short state;
  public Jplayer_position2d_cmd_pos_t() {
    pos = new Jplayer_pose_t();
    vel = new Jplayer_pose_t();
  }
}
