package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_ir_pose_t implements Serializable {
  public final static long serialVersionUID = 4412408605960421277L;
  public long poses_count;
  public Jplayer_pose_t[] poses;
  public Jplayer_ir_pose_t() {
    poses = new Jplayer_pose_t[playercore_javaConstants.PLAYER_IR_MAX_SAMPLES];
  }
}
