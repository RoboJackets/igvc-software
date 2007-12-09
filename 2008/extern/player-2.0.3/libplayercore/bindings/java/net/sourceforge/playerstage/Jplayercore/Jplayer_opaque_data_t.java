package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_opaque_data_t implements Serializable {
  public final static long serialVersionUID = -4095612089775298467L;
  public long data_count;
  public short[] data;
  public Jplayer_opaque_data_t() {
    data = new short[playercore_javaConstants.PLAYER_OPAQUE_MAX_SIZE];
  }
}
