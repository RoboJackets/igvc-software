package net.sourceforge.playerstage.Jplayercore;
import java.io.Serializable;
public class Jplayer_localize_get_particles_t implements Serializable {
  public final static long serialVersionUID = 2391130062819225307L;
  public Jplayer_pose_t mean;
  public double variance;
  public long particles_count;
  public Jplayer_localize_particle_t[] particles;
  public Jplayer_localize_get_particles_t() {
    mean = new Jplayer_pose_t();
    particles = new Jplayer_localize_particle_t[playercore_javaConstants.PLAYER_LOCALIZE_PARTICLES_MAX];
  }
}
