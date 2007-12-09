package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_rfid_data_t implements Serializable {
  public final static long serialVersionUID = -8133723520652156821L;
  public long tags_count;
  public Jplayer_rfid_tag_t[] tags;
  public Jplayer_rfid_data_t() {
    tags = new Jplayer_rfid_tag_t[playercore_javaConstants.PLAYER_RFID_MAX_TAGS];
  }
}
