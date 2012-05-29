BeBoPr
======

3D printer controller software for BeagleBone with BeBoPr Cape.

Currently this is a very preliminary version, that allows some interactive
experiments with the stepper driver. For this one needs the binary 'stepper.bin'
that's not in this repository and only comes with the BeBoPr Cape.

Compilation:
------------
Change setenv to reflect the compilation environment.
Source setenv before running 'make'.
After compilation, copy mendel.elf together with stepper.bin and
bebopr_test.sh to the beaglebone.
The Linux kernel used needs several features to be able to run this code:
The uio_pruss module should be available and the adc, pwm and gpio
interfaces should be present in the /sys filesystem.
The kernel should also contain the Cape EEPROM parsing code, otherwise all
settings must be made manually.
If all these conditions are met, run mendel.elf as root and play with
the interactive gcode interpreter.

