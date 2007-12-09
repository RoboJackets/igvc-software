package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_wsn_cmd_t implements Serializable {
  public final static long serialVersionUID = 3865215245957029976L;
  public int node_id;
  public int group_id;
  public long device;
  public short enable;
  public Jplayer_wsn_cmd_t() {
  }
}
