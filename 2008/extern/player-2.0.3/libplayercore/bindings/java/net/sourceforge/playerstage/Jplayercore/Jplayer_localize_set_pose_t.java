package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_localize_set_pose_t implements Serializable {
  public final static long serialVersionUID = 1686310758135058489L;
  public Jplayer_pose_t mean;
  public double[] cov;
  public Jplayer_localize_set_pose_t() {
    mean = new Jplayer_pose_t();
    cov = new double[3];
  }
}
