#include "Si114x_functions.h"
#include "Si114x_debug.h"

void PT_replaceSample( u8 nonused, u8 channel, u8 index, s32 value)
{
   switch(index)
   {
      case 0:  Si114xsamples.vis = value;
               break;
      case 1:  Si114xsamples.ir  = value;
               break;
      case 2:  Si114xsamples.ps1 = value;
               break;
      case 3:  Si114xsamples.ps2 = value;
               break;
      case 4:  Si114xsamples.ps3 = value;
               break;
      case 5:  Si114xsamples.aux       = value;
               break;
   }

int printf(char *fmt, ...)
{
	return 0;
}