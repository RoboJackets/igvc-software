package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_truth_pose_t implements Serializable {
  public final static long serialVersionUID = -574079768195846317L;
  public Jplayer_pose3d_t pose;
  public Jplayer_truth_pose_t() {
    pose = new Jplayer_pose3d_t();
  }
}
