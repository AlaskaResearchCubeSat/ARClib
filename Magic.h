#ifndef __MAGIC_H
  #define __MAGIC_H

//Two magic values:
//  RESET_MAGIC_PRE, set by reset function just before causing WDT violation
//  RESET_MAGIC_POST set by startup code if magic is RESET_MAGIC_PRE
#define     RESET_MAGIC_PRE   0xAA54
#define     RESET_MAGIC_POST  0xA5A6

//used if no magic value found
#define     RESET_MAGIC_EMPTY 0

#endif
  