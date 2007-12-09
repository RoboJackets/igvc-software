
/**************************************************************************
 * Desc: Path planner: plan generation
 * Author: Andrew Howard
 * Date: 10 Oct 2002
 * CVS: $Id: plan_plan.c,v 1.2 2005/10/07 00:54:33 gerkey Exp $
**************************************************************************/

#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <limits.h>

#include "plan.h"

// Plan queue stuff
void plan_push(plan_t *plan, plan_cell_t *cell);
plan_cell_t *plan_pop(plan_t *plan);


// Generate the plan
void plan_update_plan(plan_t *plan, double gx, double gy)
{
  int oi, oj, di, dj, ni, nj;
  float cost;
  plan_cell_t *cell, *ncell;

  // Reset the grid
  for (nj = 0; nj < plan->size_y; nj++)
  {
    for (ni = 0; ni < plan->size_x; ni++)
    {
      cell = plan->cells + PLAN_INDEX(plan, ni, nj);
      cell->plan_cost = 1e6;
      cell->plan_next = NULL;
    }
  }
  
  // Reset the queue
  plan->queue_start = 0;
  plan->queue_len = 0;

  // Initialize the goal cell
  ni = PLAN_GXWX(plan, gx);
  nj = PLAN_GYWY(plan, gy);
  
  if (!PLAN_VALID_BOUNDS(plan, ni, nj))
    return;

  cell = plan->cells + PLAN_INDEX(plan, ni, nj);
  cell->plan_cost = 0;
  plan_push(plan, cell);

  while (1)
  {
    cell = plan_pop(plan);
    if (cell == NULL)
      break;

    oi = cell->ci;
    oj = cell->cj;

    //printf("pop %d %d %f\n", cell->ci, cell->cj, cell->plan_cost);

    for (dj = -1; dj <= +1; dj++)
    {
      for (di = -1; di <= +1; di++)
      {
        if (di == 0 && dj == 0)
          continue;
        
        ni = oi + di;
        nj = oj + dj;

        if (!PLAN_VALID_BOUNDS(plan, ni, nj))
          continue;
        ncell = plan->cells + PLAN_INDEX(plan, ni, nj);

        if (ncell->occ_dist < plan->abs_min_radius)
          continue;

        cost = cell->plan_cost + plan->scale;

        if (ncell->occ_dist < plan->max_radius)
          cost += plan->dist_penalty * (plan->max_radius - ncell->occ_dist);

        if (cost < ncell->plan_cost)
        {
          ncell->plan_cost = cost;
          ncell->plan_next = cell;
          plan_push(plan, ncell);
        }
      }
    }
  }

  return;
}


// Push a plan location onto the queue
void plan_push(plan_t *plan, plan_cell_t *cell)
{
  int i;

  // HACK: should resize the queue dynamically (tricky for circular queue)
  assert(plan->queue_len < plan->queue_size);

  i = (plan->queue_start + plan->queue_len) % plan->queue_size;
  plan->queue[i] = cell;
  plan->queue_len++;

  return;
}


// Pop a plan location from the queue
plan_cell_t *plan_pop(plan_t *plan)
{
  int i;
  plan_cell_t *cell;
  
  if (plan->queue_len == 0)
    return NULL;

  i = plan->queue_start % plan->queue_size;
  cell = plan->queue[i];
  plan->queue_start++;
  plan->queue_len--;

  return cell;
}
