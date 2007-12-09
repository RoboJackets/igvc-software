package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_wsn_data_t implements Serializable {
  public final static long serialVersionUID = 6829988651223212394L;
  public long node_type;
  public long node_id;
  public long node_parent_id;
  public Jplayer_wsn_node_data_t data_packet;
  public Jplayer_wsn_data_t() {
    data_packet = new Jplayer_wsn_node_data_t();
  }
}
