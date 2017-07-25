/*
#ifndef MCUFRIEND_KEIL_H_
#define MCUFRIEND_KEIL_H_

#if defined(USE_SERIAL)
#include "mcufriend_keil_spi.h"
#else
#include "pin_shield_1.h"     //shield pin macros e.g. A2_PORT, PIN_OUTPUT()
#include "pin_shield_8.h"     //macros for write_8(), read_8(), setWriteDir(), ...

// control pins as used in MCUFRIEND shields 
  #define RD_PORT A0_PORT
  #define RD_PIN  A0_PIN
  #define WR_PORT A1_PORT
  #define WR_PIN  A1_PIN
  #define CD_PORT A2_PORT
  #define CD_PIN  A2_PIN
  #define CS_PORT A3_PORT
  #define CS_PIN  A3_PIN
  #define RESET_PORT A4_PORT
  #define RESET_PIN  A4_PIN

// general purpose pin macros
 #define RD_ACTIVE  PIN_LOW(RD_PORT, RD_PIN)
 #define RD_IDLE    PIN_HIGH(RD_PORT, RD_PIN)
 #define RD_OUTPUT  PIN_OUTPUT(RD_PORT, RD_PIN)
 #define WR_ACTIVE  PIN_LOW(WR_PORT, WR_PIN)
 #define WR_IDLE    PIN_HIGH(WR_PORT, WR_PIN)
 #define WR_OUTPUT  PIN_OUTPUT(WR_PORT, WR_PIN)
 #define CD_COMMAND PIN_LOW(CD_PORT, CD_PIN)
 #define CD_DATA    PIN_HIGH(CD_PORT, CD_PIN)
 #define CD_OUTPUT  PIN_OUTPUT(CD_PORT, CD_PIN)
 #define CS_ACTIVE  PIN_LOW(CS_PORT, CS_PIN)
 #define CS_IDLE    PIN_HIGH(CS_PORT, CS_PIN)
 #define CS_OUTPUT  PIN_OUTPUT(CS_PORT, CS_PIN)
 #define RESET_ACTIVE  PIN_LOW(RESET_PORT, RESET_PIN)
 #define RESET_IDLE    PIN_HIGH(RESET_PORT, RESET_PIN)
 #define RESET_OUTPUT  PIN_OUTPUT(RESET_PORT, RESET_PIN)

// General macros.   IOCLR registers are 1 cycle when optimised.
#define WR_STROBE { WR_ACTIVE; WR_IDLE; }         //PWLW=TWRL=50ns
#define RD_STROBE RD_IDLE, RD_ACTIVE, RD_ACTIVE, RD_ACTIVE   //PWLR=TRDL=150ns
#if defined(TEENSY) || defined(__ARM_ARCH_7EM__) // || defined(STM32L476xx)
#define write8(d) { write_8(d); WR_ACTIVE; WR_ACTIVE; WR_STROBE; WR_IDLE; } // STROBEs are defined later
// read 250ns after RD_ACTIVE goes low
#define read8() ( RD_STROBE, RD_ACTIVE, RD_ACTIVE, RD_ACTIVE, RD_ACTIVE, RD_ACTIVE, RD_ACTIVE, read_8() )
#else
#define write8(d) { write_8(d); WR_STROBE; } // STROBEs are defined later
// read 250ns after RD_ACTIVE goes low
#define read8() ( RD_STROBE, read_8() )
#endif

#define write16(x)    { uint8_t h = (x)>>8, l = x; write8(h); write8(l); }
#define READ_8(dst)   { dst = read8(); RD_IDLE; }
#define READ_16(dst)  { dst = read8(); dst = (dst<<8) | read8(); RD_IDLE; }

#define CTL_INIT()   { RD_OUTPUT; WR_OUTPUT; CD_OUTPUT; CS_OUTPUT; RESET_OUTPUT; }
#define WriteCmd(x)  { CD_COMMAND; write16(x); }
#define WriteData(x) { CD_DATA; write16(x); }

#endif   //!USE_SERIAL
#endif   //MCUFRIEND_KEIL_H_

*/
