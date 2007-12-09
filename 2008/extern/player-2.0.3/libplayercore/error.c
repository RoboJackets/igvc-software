
/***************************************************************************
 * Desc: Error handling macros
 * Author: Andrew Howard
 * Date: 13 May 2002
 * CVS: $Id: error.c,v 1.5 2006/01/19 19:26:12 gerkey Exp $
 **************************************************************************/

#include <stdarg.h>

#include <libplayercore/error.h>

// User-selected msg level: 0 for the most important messages (always
// printed); 9 for the least important.
static int msgLevel;

// File for logging messages
static FILE *msgFile;


// Initialize error logging
void 
ErrorInit(int _msgLevel)
{
  msgLevel = _msgLevel;
  msgFile = fopen(".player", "a+");
}


// Function for printing and logging errors.
void ErrorPrint(int msgType, int level, const char *file, int line, const char *fmt, ...)
{
  va_list ap;

  if (level <= msgLevel)
  {
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
  }
  if (msgFile)
  {
    fprintf(msgFile, "%s:%d ", file, line);
    va_start(ap, fmt);
    vfprintf(msgFile, fmt, ap);
    va_end(ap);
  }
  
  return;
}
