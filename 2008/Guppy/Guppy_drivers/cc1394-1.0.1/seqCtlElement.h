/*
  seqCtlElement.h
  
  'struct'-definition of the SequenceControl Elements
  
  12.06.05 G.Glock
*/

#ifndef SEQCTLELEMENT_H
#define SEQCTLELEMENT_H

#define MAX_SEQ_LENGTH 256 // 8 bit

typedef struct seqElementStruct {
    bool gainAvailable;
    unsigned int gainLevel;
    bool shutterAvailable;
    unsigned int shutterValue;
    bool f7Available;
    unsigned int f7Width;
    unsigned int f7Height;
    unsigned int f7X;
    unsigned int f7Y;
} seqElement;


#endif // SEQCTLELEMENT_H
