#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
inline void delay(unsigned long) {}
inline void delayMicroseconds(unsigned int) {}
class Print { public: virtual ~Print() {} virtual size_t write(uint8_t) = 0;
  size_t print(const char *s) { size_t n = 0; while (*s) n += write((uint8_t)*s++); return n; } };
