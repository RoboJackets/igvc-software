package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_gripper_geom_t implements Serializable {
  public final static long serialVersionUID = 3622334531306130665L;
  public Jplayer_pose_t pose;
  public Jplayer_bbox_t size;
  public Jplayer_gripper_geom_t() {
    pose = new Jplayer_pose_t();
    size = new Jplayer_bbox_t();
  }
}
