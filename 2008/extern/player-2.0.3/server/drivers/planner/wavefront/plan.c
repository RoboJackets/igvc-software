
/**************************************************************************
 * Desc: Path planning
 * Author: Andrew Howard
 * Date: 10 Oct 2002
 * CVS: $Id: plan.c,v 1.21 2006/03/28 19:00:15 gerkey Exp $
**************************************************************************/

#if HAVE_CONFIG_H
  #include <config.h>
#endif

// This header MUST come before <openssl/md5.h>
#include <sys/types.h>

#if HAVE_OPENSSL_MD5_H && HAVE_LIBCRYPTO
  #include <openssl/md5.h>
#endif

#include <assert.h>
#include <math.h>
#include <float.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <libplayercore/playercommon.h>
#include <libplayercore/error.h>

#include "plan.h"
//#include "heap.h"

#if HAVE_OPENSSL_MD5_H && HAVE_LIBCRYPTO
// length of the hash, in unsigned ints
#define HASH_LEN (MD5_DIGEST_LENGTH / sizeof(unsigned int))
#endif

// Create a planner
plan_t *plan_alloc(double abs_min_radius, double des_min_radius,
                   double max_radius, double dist_penalty)
{
  plan_t *plan;

  plan = calloc(1, sizeof(plan_t));

  plan->size_x = 0;
  plan->size_y = 0;
  plan->scale = 0.0;

  plan->min_x = 0;
  plan->min_y = 0;
  plan->max_x = 0;
  plan->max_y = 0;

  plan->abs_min_radius = abs_min_radius;
  plan->des_min_radius = des_min_radius;

  plan->max_radius = max_radius;
  plan->dist_penalty = dist_penalty;
  
  plan->queue_start = 0;
  plan->queue_len = 0;
  plan->queue_size = 400000; // HACK: FIX
  plan->queue = calloc(plan->queue_size, sizeof(plan->queue[0]));

  plan->waypoint_count = 0;
  plan->waypoint_size = 100;
  plan->waypoints = calloc(plan->waypoint_size, sizeof(plan->waypoints[0]));
  
  return plan;
}


// Destroy a planner
void plan_free(plan_t *plan)
{
  if (plan->cells)
    free(plan->cells);
  free(plan->queue);
  free(plan->waypoints);
  free(plan);

  return;
}


// Reset the plan
void plan_reset(plan_t *plan)
{
  int i, j;
  plan_cell_t *cell;

  for (j = 0; j < plan->size_y; j++)
  {
    for (i = 0; i < plan->size_x; i++)
    {
      cell = plan->cells + PLAN_INDEX(plan, i, j);
      cell->ci = i;
      cell->cj = j;
      cell->occ_state = 0;
      cell->occ_dist = plan->max_radius;
      cell->plan_cost = 1e12;
      cell->plan_next = NULL;
    }
  }
  plan->waypoint_count = 0;
  return;
}

void
plan_set_bounds(plan_t* plan, int min_x, int min_y, int max_x, int max_y)
{
  assert(min_x < plan->size_x);
  assert(min_y < plan->size_y);
  assert(max_x < plan->size_x);
  assert(max_y < plan->size_y);
  assert(min_x <= max_x);
  assert(min_y <= max_y);

  plan->min_x = min_x;
  plan->min_y = min_y;
  plan->max_x = max_x;
  plan->max_y = max_y;
}

int
plan_check_inbounds(plan_t* plan, double x, double y)
{
  int gx, gy;

  gx = PLAN_GXWX(plan, x);
  gy = PLAN_GYWY(plan, y);

  if((gx >= plan->min_x) && (gx <= plan->max_x) &&
     (gy >= plan->min_y) && (gy <= plan->max_y))
    return(1);
  else
    return(0);
}

void
plan_set_bbox(plan_t* plan, double padding, double min_size,
              double x0, double y0, double x1, double y1)
{
  int gx0, gy0, gx1, gy1;
  int min_x, min_y, max_x, max_y;
  int sx, sy;
  int dx, dy;
  int gmin_size;
  int gpadding;

  gx0 = PLAN_GXWX(plan, x0);
  gy0 = PLAN_GYWY(plan, y0);
  gx1 = PLAN_GXWX(plan, x1);
  gy1 = PLAN_GYWY(plan, y1);

  // Make a bounding box to include both points.
  min_x = MIN(gx0, gx1);
  min_y = MIN(gy0, gy1);
  max_x = MAX(gx0, gx1);
  max_y = MAX(gy0, gy1);

  // Make sure the min_size is achievable
  gmin_size = (int)ceil(min_size / plan->scale);
  gmin_size = MIN(gmin_size, MIN(plan->size_x-1, plan->size_y-1));

  // Add padding
  gpadding = (int)ceil(padding / plan->scale);
  min_x -= gpadding / 2;
  min_x = MAX(min_x, 0);
  max_x += gpadding / 2;
  max_x = MIN(max_x, plan->size_x - 1);
  min_y -= gpadding / 2;
  min_y = MAX(min_y, 0);
  max_y += gpadding / 2;
  max_y = MIN(max_y, plan->size_y - 1);

  // Grow the box if necessary to achieve the min_size
  sx = max_x - min_x;
  while(sx < gmin_size)
  {
    dx = gmin_size - sx;
    min_x -= (int)ceil(dx / 2.0);
    max_x += (int)ceil(dx / 2.0);

    min_x = MAX(min_x, 0);
    max_x = MIN(max_x, plan->size_x-1);

    sx = max_x - min_x;
  }
  sy = max_y - min_y;
  while(sy < gmin_size)
  {
    dy = gmin_size - sy;
    min_y -= (int)ceil(dy / 2.0);
    max_y += (int)ceil(dy / 2.0);

    min_y = MAX(min_y, 0);
    max_y = MIN(max_y, plan->size_y-1);

    sy = max_y - min_y;
  }

  plan_set_bounds(plan, min_x, min_y, max_x, max_y);
}

void
plan_update_cspace_naive(plan_t* plan)
{
  int i, j;
  int di, dj, dn;
  double r;
  plan_cell_t *cell, *ncell;

  PLAYER_MSG0(2,"Generating C-space....");
          
  dn = (int) ceil(plan->max_radius / plan->scale);

  for (j = plan->min_y; j <= plan->max_y; j++)
  {
    for (i = plan->min_x; i <= plan->max_x; i++)
    {
      cell = plan->cells + PLAN_INDEX(plan, i, j);
      if (cell->occ_state < 0)
        continue;

      cell->occ_dist = FLT_MAX;

      for (dj = -dn; dj <= +dn; dj++)
      {
        for (di = -dn; di <= +dn; di++)
        {
          if (!PLAN_VALID_BOUNDS(plan, i + di, j + dj))            
            continue;
          ncell = plan->cells + PLAN_INDEX(plan, i + di, j + dj);
          
          r = plan->scale * sqrt(di * di + dj * dj);
          if (r < ncell->occ_dist)
            ncell->occ_dist = r;
        }
      }
    }
  }
}

#if 0
void
plan_update_cspace_dp(plan_t* plan)
{
  int i, j;
  int di, dj, dn;
  double r;
  plan_cell_t *cell, *ncell;
  //heap_t* Q;
  plan_cell_t** Q;
  int qhead, qtail;

  PLAYER_MSG0(2,"Generating C-space....");

  // We'll look this far away from obstacles when updating cell costs
  dn = (int) ceil(plan->max_radius / plan->scale);

  // We'll need space for, at most, dn^2 cells in our queue (which is a
  // binary heap).
  //Q = heap_alloc(dn*dn, NULL);
  Q = (plan_cell_t**)malloc(sizeof(plan_cell_t*)*dn*dn);
  assert(Q);

  // Make space for the marks that we'll use.
  if(plan->marks_size != plan->size_x*plan->size_y)
  {
    plan->marks_size = plan->size_x*plan->size_y;
    plan->marks = (unsigned char*)realloc(plan->marks,
                                          sizeof(unsigned char*) * 
                                          plan->marks_size);
  }
  assert(plan->marks);
  
  // For each obstacle (or unknown cell), spread a wave out in all
  // directions, updating occupancy distances (inverse costs) on the way.  
  // Don't ever raise the occupancy distance of a cell.
  for (j = 0; j < plan->size_y; j++)
  {
    for (i = 0; i < plan->size_x; i++)
    {
      cell = plan->cells + PLAN_INDEX(plan, i, j);
      if (cell->occ_state < 0)
        continue;

      //cell->occ_dist = plan->max_radius;
      cell->occ_dist = 0.0;

      // clear the marks
      memset(plan->marks,0,sizeof(unsigned char)*plan->size_x*plan->size_y);

      // Start with the current cell
      //heap_reset(Q);
      //heap_insert(Q, cell->occ_dist, cell);
      qhead = 0;
      Q[qhead] = cell;
      qtail = 1;

      //while(!heap_empty(Q))
      while(qtail != qhead)
      {
        //ncell = heap_extract_max(Q);
        ncell = Q[qhead++];

        // Mark it, so we don't come back
        plan->marks[PLAN_INDEX(plan, ncell->ci, ncell->cj)] = 1;

        // Is this cell an obstacle or unknown cell (and not the initial
        // cell?  If so, don't bother updating its cost here.
        if((ncell != cell) && (ncell->occ_state >= 0))
          continue;
        
        // How far are we from the obstacle cell we started with?
        r = plan->scale * hypot(ncell->ci - cell->ci,
                                ncell->cj - cell->cj);

        // Are we past the distance at which we care?
        if(r > plan->max_radius)
          continue;

        // Update the occupancy distance if we're closer
        if(r < ncell->occ_dist)
        {
          ncell->occ_dist = r;
          // Also put its neighbors on the queue for processing
          for(dj = -1; dj <= 1; dj+= 2)
          {
            for(di = -1; di <= 1; di+= 2)
            {
              if (!PLAN_VALID(plan, ncell->ci + di, ncell->cj + dj))            
                continue;
              // Have we already seen this cell?
              if(plan->marks[PLAN_INDEX(plan, ncell->ci + di, ncell->cj + dj)])
                continue;
              // Add it to the queue
              /*
              heap_insert(Q, ncell->occ_dist,
                          plan->cells + PLAN_INDEX(plan,
                                                   ncell->ci+di,
                                                   ncell->cj+dj));
                                                   */
              Q[qtail++] = plan->cells + PLAN_INDEX(plan,
                                                    ncell->ci+di,
                                                    ncell->cj+dj);
            }
          }
        }
      }
    }
  }
  //heap_free(Q);
}
#endif

// Construct the configuration space from the occupancy grid.
// This treats both occupied and unknown cells as bad.
// 
// If cachefile is non-NULL, then we try to read the c-space from that
// file.  If that fails, then we construct the c-space as per normal and
// then write it out to cachefile.
void 
plan_update_cspace(plan_t *plan, const char* cachefile)
{
#if HAVE_OPENSSL_MD5_H && HAVE_LIBCRYPTO
  unsigned int hash[HASH_LEN];
  plan_md5(hash, plan);
  if(cachefile && strlen(cachefile))
  {
    PLAYER_MSG1(2,"Trying to read c-space from file %s", cachefile);
    if(plan_read_cspace(plan,cachefile,hash) == 0)
    {
      // Reading from the cache file worked; we're done here.
      PLAYER_MSG1(2,"Successfully read c-space from file %s", cachefile);
      return;
    }
    PLAYER_MSG1(2, "Failed to read c-space from file %s", cachefile);
  }
#endif

  //plan_update_cspace_dp(plan);
  plan_update_cspace_naive(plan);

#if HAVE_OPENSSL_MD5_H && HAVE_LIBCRYPTO
  if(cachefile)
    plan_write_cspace(plan,cachefile, (unsigned int*)hash);
#endif

  PLAYER_MSG0(2,"Done.");
}

#if HAVE_OPENSSL_MD5_H && HAVE_LIBCRYPTO
// Write the cspace occupancy distance values to a file, one per line.
// Read them back in with plan_read_cspace().
// Returns non-zero on error.
int 
plan_write_cspace(plan_t *plan, const char* fname, unsigned int* hash)
{
  plan_cell_t* cell;
  int i,j;
  FILE* fp;

  if(!(fp = fopen(fname,"w+")))
  {
    PLAYER_MSG2(2,"Failed to open file %s to write c-space: %s",
                fname,strerror(errno));
    return(-1);
  }

  fprintf(fp,"%d\n%d\n", plan->size_x, plan->size_y);
  fprintf(fp,"%.3lf\n%.3lf\n", plan->origin_x, plan->origin_y);
  fprintf(fp,"%.3lf\n%.3lf\n", plan->scale,plan->max_radius);
  for(i=0;i<HASH_LEN;i++)
    fprintf(fp,"%08X", hash[i]);
  fprintf(fp,"\n");

  for(j = 0; j < plan->size_y; j++)
  {
    for(i = 0; i < plan->size_x; i++)
    {
      cell = plan->cells + PLAN_INDEX(plan, i, j);
      fprintf(fp,"%.3f\n", cell->occ_dist);
    }
  }

  fclose(fp);
  return(0);
}

// Read the cspace occupancy distance values from a file, one per line.
// Write them in first with plan_read_cspace().
// Returns non-zero on error.
int 
plan_read_cspace(plan_t *plan, const char* fname, unsigned int* hash)
{
  plan_cell_t* cell;
  int i,j;
  FILE* fp;
  int size_x, size_y;
  double origin_x, origin_y;
  double scale, max_radius;
  unsigned int cached_hash[HASH_LEN];

  if(!(fp = fopen(fname,"r")))
  {
    PLAYER_MSG1(2,"Failed to open file %s", fname);
    return(-1);
  }
  
  /* Read out the metadata */
  if((fscanf(fp,"%d", &size_x) < 1) ||
     (fscanf(fp,"%d", &size_y) < 1) ||
     (fscanf(fp,"%lf", &origin_x) < 1) ||
     (fscanf(fp,"%lf", &origin_y) < 1) ||
     (fscanf(fp,"%lf", &scale) < 1) ||
     (fscanf(fp,"%lf", &max_radius) < 1))
  {
    PLAYER_MSG1(2,"Failed to read c-space metadata from file %s", fname);
    fclose(fp);
    return(-1);
  }

  for(i=0;i<HASH_LEN;i++)
  {
    if(fscanf(fp,"%08X", cached_hash+i) < 1)
    {
      PLAYER_MSG1(2,"Failed to read c-space metadata from file %s", fname);
      fclose(fp);
      return(-1);
    }
  }

  /* Verify that metadata matches */
  if((size_x != plan->size_x) ||
     (size_y != plan->size_y) ||
     (fabs(origin_x - plan->origin_x) > 1e-3) ||
     (fabs(origin_y - plan->origin_y) > 1e-3) ||
     (fabs(scale - plan->scale) > 1e-3) ||
     (fabs(max_radius - plan->max_radius) > 1e-3) ||
     memcmp(cached_hash, hash, sizeof(unsigned int) * HASH_LEN))
  {
    PLAYER_MSG1(2,"Mismatch in c-space metadata read from file %s", fname);
    fclose(fp);
    return(-1);
  }

  for(j = 0; j < plan->size_y; j++)
  {
    for(i = 0; i < plan->size_x; i++)
    {
      cell = plan->cells + PLAN_INDEX(plan, i, j);
      if(fscanf(fp,"%f", &(cell->occ_dist)) < 1)
      {
        PLAYER_MSG3(2,"Failed to read c-space data for cell (%d,%d) from file %s",
                     i,j,fname);
        fclose(fp);
        return(-1);
      }
    }
  }

  fclose(fp);
  return(0);
}

// Compute the 16-byte MD5 hash of the map data in the given plan
// object.
void
plan_md5(unsigned int* digest, plan_t* plan)
{
  MD5_CTX c;

  MD5_Init(&c);

  MD5_Update(&c,(const unsigned char*)plan->cells,
             (plan->size_x*plan->size_y)*sizeof(plan_cell_t));

  MD5_Final((unsigned char*)digest,&c);
}
#endif // HAVE_OPENSSL_MD5_H && HAVE_LIBCRYPTO
