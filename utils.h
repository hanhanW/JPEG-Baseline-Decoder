#ifndef __UTILS_H_
#define __UTILS_H_
#include <stdint.h>

uint16_t econv16(uint16_t n);
void zigzag(const int16_t zz[64], int16_t mat[8][8]);
void idct8x8(double mat[8][8]);

#endif
