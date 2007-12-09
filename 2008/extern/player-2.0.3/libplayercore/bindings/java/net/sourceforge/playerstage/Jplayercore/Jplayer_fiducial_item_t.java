package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_fiducial_item_t implements Serializable {
  public final static long serialVersionUID = 2500978655668242076L;
  public int id;
  public Jplayer_pose3d_t pose;
  public Jplayer_pose3d_t upose;
  public Jplayer_fiducial_item_t() {
    pose = new Jplayer_pose3d_t();
    upose = new Jplayer_pose3d_t();
  }
}
