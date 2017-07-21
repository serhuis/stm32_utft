#ifndef PIN_SHIELD_8_H_
#define PIN_SHIELD_8_H_

// just provide macros for the 8-bit data bus
// i.e. write_8(), read_8(), setWriteDir(), setReadDir()


#define STM32F103xB


#include <STM32F10X.h>

#include <STM32F1XX.h>

// configure macros for the data pins. -00=10.06, -O1=7.85, -O1t=7.21, -O2=7.87, -O3=7.45, -O3t=7.03
  #define write_8(d) { \
   GPIOA->BSRR = 0x0700 << 16; \
   GPIOB->BSRR = 0x0438 << 16; \
   GPIOC->BSRR = 0x0080 << 16; \
   GPIOA->BSRR = (((d) & (1<<0)) << 9) \
               | (((d) & (1<<2)) << 8) \
               | (((d) & (1<<7)) << 1); \
   GPIOB->BSRR = (((d) & (1<<3)) << 0) \
               | (((d) & (1<<4)) << 1) \
               | (((d) & (1<<5)) >> 1) \
               | (((d) & (1<<6)) << 4); \
   GPIOC->BSRR = (((d) & (1<<1)) << 6); \
    }
  #define read_8() (          (((GPIOA->IDR & (1<<9)) >> 9) \
                             | ((GPIOC->IDR & (1<<7)) >> 6) \
                             | ((GPIOA->IDR & (1<<10)) >> 8) \
                             | ((GPIOB->IDR & (1<<3)) >> 0) \
                             | ((GPIOB->IDR & (1<<5)) >> 1) \
                             | ((GPIOB->IDR & (1<<4)) << 1) \
                             | ((GPIOB->IDR & (1<<10)) >> 4) \
                             | ((GPIOA->IDR & (1<<8))  >> 1)))
// be wise to clear both MODER bits properly.
#if defined(STM32F103xB)
#define GROUP_MODE(port, reg, mask, val)  {port->reg = (port->reg & ~(mask)) | ((mask)&(val)); }
#define GP_OUT(port, reg, mask)           GROUP_MODE(port, reg, mask, 0x33333333)
#define GP_INP(port, reg, mask)           GROUP_MODE(port, reg, mask, 0x44444444)
//                                 PA10,PA9,PA8                       PB10                   PB5,PB4,PB3                             PC7
#define setWriteDir() {GP_OUT(GPIOA, CRH, 0xFFF); GP_OUT(GPIOB, CRH, 0xF00); GP_OUT(GPIOB, CRL, 0xFFF000); GP_OUT(GPIOC, CRL, 0xF0000000); }
#define setReadDir()  {GP_INP(GPIOA, CRH, 0xFFF); GP_INP(GPIOB, CRH, 0xF00); GP_INP(GPIOB, CRL, 0xFFF000); GP_INP(GPIOC, CRL, 0xF0000000); }


