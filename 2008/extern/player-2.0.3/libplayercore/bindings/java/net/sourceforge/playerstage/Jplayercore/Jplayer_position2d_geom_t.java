package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_position2d_geom_t implements Serializable {
  public final static long serialVersionUID = -7688829896765390277L;
  public Jplayer_pose_t pose;
  public Jplayer_bbox_t size;
  public Jplayer_position2d_geom_t() {
    pose = new Jplayer_pose_t();
    size = new Jplayer_bbox_t();
  }
}
