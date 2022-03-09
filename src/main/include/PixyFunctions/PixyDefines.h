#ifndef PIXY_DEFINES_H
#define PIXY_DEFINES_H

#include <stdint.h>

// PIXY data block struct array
typedef struct 
{
  uint16_t  sig;   // Signature Number
  uint16_t  x;   // x center of object
  uint16_t  y;   // y center of object
  uint16_t  w;   // width of object
  uint16_t  h;   // height of object
} PIXY_BLOCK_TYPE;

#define MAX_NR_OF_BLOCKS 30

#endif // PIXY_DEFINES_H