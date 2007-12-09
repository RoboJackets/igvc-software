package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_map_data_vector_t implements Serializable {
  public final static long serialVersionUID = -4792601412616288382L;
  public float minx;
  public float maxx;
  public float miny;
  public float maxy;
  public long segments_count;
  public Jplayer_segment_t[] segments;
  public Jplayer_map_data_vector_t() {
    segments = new Jplayer_segment_t[playercore_javaConstants.PLAYER_MAP_MAX_SEGMENTS];
  }
}
