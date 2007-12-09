package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_camera_data_t implements Serializable {
  public final static long serialVersionUID = 7178024706226744136L;
  public long width;
  public long height;
  public long bpp;
  public long format;
  public long fdiv;
  public long compression;
  public long image_count;
  public short[] image;
  public Jplayer_camera_data_t() {
    image = new short[playercore_javaConstants.PLAYER_CAMERA_IMAGE_SIZE];
  }
}
