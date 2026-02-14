#ifndef ROM_H
#define ROM_H

#include <stdint.h>

//PC will hold this, a program that writes "HELLO" over uart at port 0xF001, and toggles LED at
// 0x8000 in a loop
extern const uint8_t rom[4096];

#endif

