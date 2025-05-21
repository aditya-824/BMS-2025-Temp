#ifndef pinModes_h
#define pinModes_h

#include <Arduino.h>
#include <stdint.h>

#define AMS_FAULT_SDC 34 // AMS Fault SDC pin number
#define AMS_FAULT_PIN 7  // AMS Fault pin number
#define AIRP 33          // AIR Positive pin number
#define AIRN 31          // AIR Negative pin number
#define PCRDONE 35       // PCR Done
#define PCR5V 15         // PCR 5 V
#define VSBAT 39
#define TCHECK 26
#define K2 37
#define K1 36
#define SSFINAL 32
#define GRNIN 30
#define GRNOUT 1
#define AUXP 29
#define AUXN 27
#define PCRAUXIN 25
#define CARTID 9
#define CHARGERAUX 6
#define DIGITALPIN4 48
#define CSOUT A5
#define VS_HV_IN A0
#define VS_BAT_IN A1

#define ISOSPIA 16
#define ISOSPIB 53
#define VCHECK 28

void pinModes(void);
void lvdatainit(void);

#endif
