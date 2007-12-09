/*
  Desc: Logfile reader
  Author: Andrew Howard
  Date: 19 Nov 2004
  CVS: $Id: logfile.cpp,v 1.1.2.2 2006/06/07 16:12:58 gerkey Exp $
 */

#include <assert.h>
#include <ctype.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include "logfile.h"


// Create a logfile reader
logfile_t *logfile_alloc(const char *filename)
{
  logfile_t *self;

  self = new logfile_t;
  
  // Open logfile
  self->file = fopen(filename, "r");
  if (!self->file)
  {
    printf("unable to open log file [%s]: %s\n", filename, strerror(errno));
    return NULL;
  }

  self->eof = 0;
  
  // Allocate space for lines
  self->line_size = 2 * 1024 * 1024;
  self->line = new char[self->line_size];
  
  return self;
}


// Read a line from the log file
int logfile_read(logfile_t *self)
{
  int i, len;
  
  // Read in a line
  self->line = fgets(self->line, self->line_size, self->file);
  if (!self->line)
  {
    self->eof = 1;
    return -1;
  }
  
  self->linenum++;
  
  // Tokenize the line using whitespace separators
  self->token_count = 0;
  len = strlen(self->line);
  for (i = 0; i < len; i++)
  {
    if (isspace(self->line[i]))
      self->line[i] = 0;
    else if (i == 0)
    {
      assert(self->token_count < (int) (sizeof(self->tokens) / sizeof(self->tokens[i])));
      self->tokens[self->token_count++] = self->line + i;
    }
    else if (self->line[i - 1] == 0)
    {
      assert(self->token_count < (int) (sizeof(self->tokens) / sizeof(self->tokens[i])));
      self->tokens[self->token_count++] = self->line + i;
    }
  }

  self->interface = "";
  
  // Skip blank lines
  if (self->token_count == 0)
    return 0;
    
  // Discard comments
  if (strcmp(self->tokens[0], "#") == 0 || strcmp(self->tokens[0], "##") == 0)
    return 0;

  assert(self->token_count >= 3);
  self->interface = self->tokens[3];
  
  // Skip sync packets
  if (strcmp(self->interface, "sync") == 0)
    return 0;
  
  assert(self->token_count >= 5);
  self->index = atoi(self->tokens[4]);
  self->dtime = atof(self->tokens[0]);

  if (strcmp(self->interface, "position3d") == 0)
  {
    // HACK
    assert(self->token_count >= 12);
    self->position_pose[0] = atof(self->tokens[6]);
    self->position_pose[1] = atof(self->tokens[7]);
    self->position_pose[2] = atof(self->tokens[11]);
  }
  else if (strcmp(self->interface, "position2d") == 0)
  {
    assert(self->token_count >= 9);
    self->position_pose[0] = atof(self->tokens[7]);
    self->position_pose[1] = atof(self->tokens[8]);
    self->position_pose[2] = atof(self->tokens[9]);
  }
  else if (strcmp(self->interface, "gps") == 0)
  {
    assert(self->token_count >= 12);
    self->gps_pos[0] = atof(self->tokens[10]);
    self->gps_pos[1] = atof(self->tokens[11]);
  }
  else if (strcmp(self->interface, "laser") == 0)
  {
    assert(self->token_count >= 13);    
    self->laser_range_count = atoi(self->tokens[12]);
    assert(self->token_count >= 13 + self->laser_range_count * 2);    

    for (i = 0; i < self->laser_range_count; i++)
      self->laser_ranges[i] = atof(self->tokens[13 + i * 2]);
  }
  
  return 0;
}

