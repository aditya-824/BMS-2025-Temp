#include "pinModes.h"

void pinModes(void)
{
    pinMode(AMS_FAULT_PIN, OUTPUT);
    pinMode(AMS_FAULT_SDC, OUTPUT); // CHANGES
    pinMode(CHARGERAUX, INPUT);
    pinMode(DIGITALPIN4, INPUT);
    pinMode(PCRDONE, INPUT);   // pcrdone
    pinMode(AIRN, INPUT);      // airn
    pinMode(PCR5V, INPUT);     // pcr5v
    pinMode(VSBAT, INPUT);     // vsbat
    pinMode(TCHECK, INPUT);    // tcheck
    pinMode(K1, INPUT);        // k2
    pinMode(K2, INPUT);        // k1
    pinMode(AIRP, INPUT);      // airp
    pinMode(SSFINAL, INPUT);   // ssfinal
    pinMode(GRNIN, INPUT);     // grnin
    pinMode(GRNOUT, INPUT);    // grnout
    pinMode(AUXP, INPUT);      // auxp
    pinMode(AUXN, INPUT);      // auxn
    pinMode(PCRAUXIN, INPUT);  // pcrauxin
    pinMode(VS_HV_IN, INPUT);  // VS_HV_IN
    pinMode(VS_BAT_IN, INPUT); // VS_BAT_IN
    pinMode(CSOUT, INPUT);     // CSOUT
    pinMode(CARTID, INPUT);    // Cart ID
    // pinMode(ISOSPIA, OUTPUT);  // isoSPI port B
    // pinMode(ISOSPIB, OUTPUT);  // isoSPI port A
}
