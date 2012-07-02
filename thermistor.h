#ifndef _THERMISTOR_H
#define _THERMISTOR_H

extern int bone_thermistor_100k( int adc, double* celsius);
extern int bone_epcos_b5760g104f( int adc, double* celsius);
extern int bone_bed_thermistor_330k( int adc, double* celsius);

#endif
