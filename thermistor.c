
#include <unistd.h>

#include "beaglebone.h"
#include "thermistor.h"

struct conversion_entry
{
	unsigned int	adc_value;
	double		celsius;
};

// {AIN, C} for beta=4066
static const struct conversion_entry thermistor_100k[] = {
  { 3617,   0.0 },
  { 3612,   5.0 },
  { 3604,  10.0 },
  { 3595,  15.0 },
  { 3584,  20.0 },
  { 3571,  25.0 },
  { 3555,  30.0 },
  { 3536,  35.0 },
  { 3513,  40.0 },
  { 3486,  45.0 },
  { 3455,  50.0 },
  { 3419,  55.0 },
  { 3377,  60.0 },
  { 3330,  65.0 },
  { 3276,  70.0 },
  { 3216,  75.0 },
  { 3149,  80.0 },
  { 3075,  85.0 },
  { 2994,  90.0 },
  { 2907,  95.0 },
  { 2814, 100.0 },
  { 2715, 105.0 },
  { 2612, 110.0 },
  { 2504, 115.0 },
  { 2393, 120.0 },
  { 2279, 125.0 },
  { 2164, 130.0 },
  { 2049, 135.0 },
  { 1935, 140.0 },
  { 1822, 145.0 },
  { 1711, 150.0 },
  { 1604, 155.0 },
  { 1500, 160.0 },
  { 1400, 165.0 },
  { 1305, 170.0 },
  { 1215, 175.0 },
  { 1130, 180.0 },
  { 1049, 185.0 },
  {  974, 190.0 },
  {  904, 195.0 },
  {  838, 200.0 },
  {  777, 205.0 },
  {  720, 210.0 },
  {  668, 215.0 },
  {  619, 220.0 },
  {  574, 225.0 },
  {  532, 230.0 },
  {  494, 235.0 },
  {  459, 240.0 },
  {  396, 250.0 },
  {  369, 255.0 },
  {  343, 260.0 },
  {  298, 270.0 },
  {  278, 275.0 },
  {  260, 280.0 },
  {  243, 285.0 },
  {  227, 290.0 },
  {  212, 295.0 },
  {  199, 300.0 },
};

/*
 * Calibration table derived from calculated transfer function
 * with values from the EPCOS B57560G104F thermistor datasheet
 * and the BeBoPr Cape schematics.
 */
static const struct conversion_entry epcos_b5760g104f[] = {
  { 3635, -20.0 },
  { 3630, -10.0 },
  { 3622,   0.0 },
  { 3609,  10.0 },
  { 3590,  20.0 },
  { 3561,  30.0 },
  { 3520,  40.0 },
  { 3463,  50.0 },
  { 3340,  60.0 },
  { 3287,  70.0 },
  { 3089,  85.0 },
  { 2832, 100.0 },
  { 2072, 135.0 },
  { 1235, 175.0 },
  {  920, 195.0 },
  {  681, 215.0 },
  {  543, 230.0 },
  {  435, 245.0 },
  {  351, 260.0 },
  {  305, 270.0 },
  {  265, 280.0 },
  {  232, 290.0 },
  {  203, 300.0 },
};

/*
 * Calibration table derived from calculated transfer function
 * with measured values from a recycled 330kOhm oven thermistor
 * and the BeBoPr Cape schematics.
 */
static const struct conversion_entry my_330k_bed_thermistor[] = {
{ 3650,  0.0 },	// forged value to prevent out of range result
{ 3626, 21.0 },
{ 3601, 35.0 },
{ 3597, 37.0 },
{ 3571, 47.0 },
{ 3546, 54.0 },
{ 3534, 57.0 },
{ 3519, 60.0 },
{ 3485, 66.0 },
{ 3452, 71.0 },
{ 3412, 76.0 },
{ 3393, 78.4 },
{ 3375, 80.4 },
{ 3357, 82.1 },
{ 3342, 83.7 },
{ 3328, 85.2 },
// the following values are not measured but extrapolated from an exponential fit
{ 3240, 90.0 },
{ 3157, 95.0 },
{ 3061, 100.0 },
{ 2949, 105.0 },
{ 2822, 110.0 },
{ 2678, 115.0 },
{ 2520, 120.0 },
{ 2348, 125.0 },
{ 2165, 130.0 },
{ 1975, 135.0 },
{ 1782, 140.0 },
{ 1589, 145.0 },
{ 1401, 150.0 },
};

static int convert_using_table( const struct conversion_entry* table, int entries, int adc, double* celsius)
{
  // TODO: implement short to GND and short to +12V detection
  if (celsius != NULL) {
    int ix;
    for (ix = 0 ; ix < entries ; ++ix) {
      int delta_adc = adc - table[ ix].adc_value;
      if (delta_adc > 0) {
        if (ix > 0) {
          double celsius_ix      = table[ ix].celsius;
	  unsigned int adc_ix	 = table[ ix].adc_value;
	  double delta_celsius   = celsius_ix - table[ ix - 1].celsius;
	  unsigned int delta_adc = table[ ix - 1].adc_value - adc_ix;
	  double rc              = delta_celsius / delta_adc;
	  *celsius = celsius_ix - (adc - adc_ix) * rc;
	} else {
         /*
          *  Before first entry, assume open circuit (sensor fault)
          *  Return very high temperature to turn heaters off
          */
          *celsius = 777.0;     // lll (low)
        }
        break;
      } else if (delta_adc == 0) {
        *celsius = table[ ix].celsius;
        break;
      } else if (ix == entries) {
       /*
        *  After last entry, assume short circuit (sensor fault)
        *  Return very high temperature to turn heaters off
        */
        *celsius = 444.0;       // hhh (high)
      }
    }
    return 0;
  }
  return -1;
}


int bone_thermistor_100k( int adc, double* celsius)
{
  return convert_using_table( thermistor_100k, NR_ITEMS( thermistor_100k), adc, celsius);
}

int bone_epcos_b5760g104f( int adc, double* celsius)
{
  return convert_using_table( epcos_b5760g104f, NR_ITEMS( epcos_b5760g104f), adc, celsius);
}

int bone_bed_thermistor_330k( int adc, double* celsius)
{
  return convert_using_table( my_330k_bed_thermistor, NR_ITEMS( my_330k_bed_thermistor), adc, celsius);
}
