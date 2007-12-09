package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_localize_particle_t implements Serializable {
  public final static long serialVersionUID = 7905669201850241971L;
  public Jplayer_pose_t pose;
  public double alpha;
  public Jplayer_localize_particle_t() {
    pose = new Jplayer_pose_t();
  }
}
