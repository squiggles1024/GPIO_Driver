#include "common.h"

void assert_failed(uint8_t* file, uint32_t line) {
      //printf("Assert fail at File %s Line %d", file, (int)line);
      while(1); 
}