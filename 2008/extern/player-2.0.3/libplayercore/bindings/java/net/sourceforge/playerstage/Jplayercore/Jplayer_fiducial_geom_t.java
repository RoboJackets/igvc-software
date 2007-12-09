package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_fiducial_geom_t implements Serializable {
  public final static long serialVersionUID = 2541338227086772460L;
  public Jplayer_pose_t pose;
  public Jplayer_bbox_t size;
  public Jplayer_bbox_t fiducial_size;
  public Jplayer_fiducial_geom_t() {
    pose = new Jplayer_pose_t();
    size = new Jplayer_bbox_t();
    fiducial_size = new Jplayer_bbox_t();
  }
}
