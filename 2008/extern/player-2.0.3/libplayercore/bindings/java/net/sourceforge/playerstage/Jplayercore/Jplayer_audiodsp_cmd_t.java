package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_audiodsp_cmd_t implements Serializable {
  public final static long serialVersionUID = -5548773113883664433L;
  public float frequency;
  public float amplitude;
  public float duration;
  public long bit_string_count;
  public short[] bit_string;
  public Jplayer_audiodsp_cmd_t() {
    bit_string = new short[playercore_javaConstants.PLAYER_AUDIODSP_MAX_BITSTRING_LEN];
  }
}
