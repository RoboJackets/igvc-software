package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_map_data_t implements Serializable {
  public final static long serialVersionUID = 7663820204992652127L;
  public long col;
  public long row;
  public long width;
  public long height;
  public long data_count;
  public byte[] data;
  public Jplayer_map_data_t() {
    data = new byte[playercore_javaConstants.PLAYER_MAP_MAX_TILE_SIZE];
  }
}
