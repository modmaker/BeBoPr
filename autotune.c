#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include "timestamp.h"
#include "heater.h"
#include "autotune.h"

#define max(a, b) ((a) > (b)) ? (a) : (b)
#define min(a, b) ((a) < (b)) ? (a) : (b)
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

typedef int bool;
#define true 1
#define false 0

#define PID_MAX 100 // The max signal your hotend can recieve. In the case of BeBoPr, this is 100 (100%)

uint64_t millis() {
  return timestamp_get() * 1000.0;
}

/* Invoke with
   PID_autotune(150.0, 0, 8); */

void PID_autotune(float temp, int extruder, int ncycles)
{
  double input = 0.0;
  int cycles = 0;
  bool heating = true;

  unsigned long temp_millis = millis(), t1 = temp_millis, t2 = temp_millis;
  long t_high = 0, t_low = 0;

  long bias, d;
  float Ku, Tu;
  float Kp, Ki, Kd;
  float max = 0, min = 10000;

  int p;

  #if HAS_AUTO_FAN
        unsigned long extruder_autofan_last_check = temp_millis;
  #endif

  // Assuming only one extruder
  fprintf(stderr, "Begin PID tuning..\n");

  /* 
  disable_heater(); // switch off all heaters.
  */

  // CONTROL PWM OUTPUT
  bias = d = PID_MAX / 2;
  setPWM(extruder, p = bias);

  // PID Tuning loop
  for (;;) {

    unsigned long ms = millis();

    // GET TEMPERATURE
    input = heater_get_temperature(extruder);

    max = max(max, input);
    min = min(min, input);

    if (heating == true && input > temp) {
      if (ms - t2 > 5000) {
        heating = false;
        // CONTROL PWM OUTPUT
        setPWM(extruder, p = (bias - d)>> 1);
        t1 = ms;
        t_high = t1 - t2;
        max = temp;
      }
    }
    if (heating == false && input < temp) {
      if (ms - t1 > 5000) {
        heating = true;
        t2 = ms;
        t_low = t2 - t1;
        if (cycles > 0) {
          long max_pow = PID_MAX;
          bias += (d*(t_high - t_low))/(t_low + t_high); //center the oscillation around your setpoint
          bias = constrain(bias, 20, max_pow - 20); //make sure we don't go over max power
          d = (bias > max_pow / 2) ? max_pow - 1 - bias : bias;

          fprintf(stderr,"Values .. Bias: %ld, D: %ld, T Min: %f, T Max: %f\n",
            bias, d, min, max);
          /* SERIAL_PROTOCOLPGM(MSG_BIAS); SERIAL_PROTOCOL(bias);
          SERIAL_PROTOCOLPGM(MSG_D);    SERIAL_PROTOCOL(d);
          SERIAL_PROTOCOLPGM(MSG_T_MIN);  SERIAL_PROTOCOL(min);
          SERIAL_PROTOCOLPGM(MSG_T_MAX);  SERIAL_PROTOCOLLN(max); */
          if (cycles > 2) {
            Ku = (4.0 * d) / (3.14159265 * (max - min) / 2.0); //I don't know where these magic numbers come from...
            Tu = ((float)(t_low + t_high) / 1000.0);
            /* SERIAL_PROTOCOLPGM(MSG_KU); SERIAL_PROTOCOL(Ku);
            SERIAL_PROTOCOLPGM(MSG_TU); SERIAL_PROTOCOLLN(Tu); */
            Kp = 0.6 * Ku; //Zlieger-Nichols tuning constants
            Ki = 2 * Kp / Tu;
            Kd = -Kp * Tu / 8;  //For some reason the autotune values in bebopr only work with the negative Kd
            fprintf(stderr,"...... .. Ku: %f, Tu: %f, Kp: %f, Ki: %f, Kd: %f\n",
              Ku, Tu, Kp, Ki, Kd);
            /* SERIAL_PROTOCOLLNPGM(MSG_CLASSIC_PID);
            SERIAL_PROTOCOLPGM(MSG_KP); SERIAL_PROTOCOLLN(Kp);
            SERIAL_PROTOCOLPGM(MSG_KI); SERIAL_PROTOCOLLN(Ki);
            SERIAL_PROTOCOLPGM(MSG_KD); SERIAL_PROTOCOLLN(Kd); */
            /*
            Kp = 0.33*Ku;
            Ki = Kp/Tu;
            Kd = Kp*Tu/3;
            SERIAL_PROTOCOLLNPGM(" Some overshoot ");
            SERIAL_PROTOCOLPGM(" Kp: "); SERIAL_PROTOCOLLN(Kp);
            SERIAL_PROTOCOLPGM(" Ki: "); SERIAL_PROTOCOLLN(Ki);
            SERIAL_PROTOCOLPGM(" Kd: "); SERIAL_PROTOCOLLN(Kd);
            Kp = 0.2*Ku;
            Ki = 2*Kp/Tu;
            Kd = Kp*Tu/3;
            SERIAL_PROTOCOLLNPGM(" No overshoot ");
            SERIAL_PROTOCOLPGM(" Kp: "); SERIAL_PROTOCOLLN(Kp);
            SERIAL_PROTOCOLPGM(" Ki: "); SERIAL_PROTOCOLLN(Ki);
            SERIAL_PROTOCOLPGM(" Kd: "); SERIAL_PROTOCOLLN(Kd);
            */
          }
        }
        // CONTROL PWM OUTPUT
        setPWM(extruder, p = (bias + d) >> 1);
        cycles++;
        min = temp;
      }
    } 
    if (input > temp + 20) {
      fprintf(stderr,"Temperature too high\n");
      return;
    }
    // Every 2 seconds...
    if (ms > temp_millis + 2000) {
      // CONTROL PWM OUTPUT
      fprintf(stderr, "At %d , %f\n", p, input);

      /* SERIAL_PROTOCOL(input);
      SERIAL_PROTOCOLPGM(MSG_AT);
      SERIAL_PROTOCOLLN(p); */

      temp_millis = ms;
    } // every 2 seconds
    // Over 2 minutes?
    if (((ms - t1) + (ms - t2)) > (10L*60L*1000L*2L)) {
      // PID tuning timed out.
      fprintf(stderr,"PID tuning timed out\n");
      return;
    }
    if (cycles > ncycles) {
      fprintf(stderr,"PID tuning complete!\n");
      return;
    }
  }
}

