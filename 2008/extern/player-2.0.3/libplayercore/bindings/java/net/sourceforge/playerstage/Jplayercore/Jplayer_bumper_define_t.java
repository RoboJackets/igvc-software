package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_bumper_define_t implements Serializable {
  public final static long serialVersionUID = -8131240577839595961L;
  public Jplayer_pose_t pose;
  public float length;
  public float radius;
  public Jplayer_bumper_define_t() {
    pose = new Jplayer_pose_t();
  }
}
