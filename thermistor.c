
#include <unistd.h>

#include "beaglebone.h"
#include "thermistor.h"


int bone_thermistor_100k( int adc, double* celsius)
{
  // TODO: implement short to GND and short to +12V detection
  if (celsius != NULL) {
    if (adc > 3000) {
      *celsius = 5.0;
    } else if (adc > 1000) {
      *celsius = 185.0 - (adc - 1000) * 95.0 / 2000.0;
    } else {
      *celsius = 300.0;
    }
    return 0;
  }
  return -1;
}

