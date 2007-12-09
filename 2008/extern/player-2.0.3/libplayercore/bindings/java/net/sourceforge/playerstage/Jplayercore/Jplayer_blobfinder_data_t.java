package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_blobfinder_data_t implements Serializable {
  public final static long serialVersionUID = 5105673824375817067L;
  public long width;
  public long height;
  public long blobs_count;
  public Jplayer_blobfinder_blob_t[] blobs;
  public Jplayer_blobfinder_data_t() {
    blobs = new Jplayer_blobfinder_blob_t[playercore_javaConstants.PLAYER_BLOBFINDER_MAX_BLOBS];
  }
}
