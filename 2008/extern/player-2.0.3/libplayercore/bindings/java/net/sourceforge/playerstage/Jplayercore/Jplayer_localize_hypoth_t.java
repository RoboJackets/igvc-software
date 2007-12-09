package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_localize_hypoth_t implements Serializable {
  public final static long serialVersionUID = 8151210908990626013L;
  public Jplayer_pose_t mean;
  public double[] cov;
  public double alpha;
  public Jplayer_localize_hypoth_t() {
    mean = new Jplayer_pose_t();
    cov = new double[3];
  }
}
