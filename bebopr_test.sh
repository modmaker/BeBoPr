#!/bin/sh

grep uio_pruss /proc/modules 2>&1 >/dev/null
if test $? -ne 0 ; then
	echo "Loading UIO_PRUSS driver module" ;
	modprobe uio_pruss ;
fi

grep debugfs /proc/mounts 2>&1 >/dev/null
if test $? -ne 0 ; then
	echo "Mounting debugfs filesystem" ;
	/bin/mount -t debugfs none /sys/kernel/debug ;
fi

export VER=`uname -r`
if test "$VER" = "3.2.0sjl+" ; then
	export KERNEL=ok
elif test "$VER" = "3.2.16+" ; then
	export KERNEL=ok
elif test "$VER" = "3.2.16sjl+" ; then
	export KERNEL=ok
fi

if test "$KERNEL" = "ok" ; then
	echo "Configuring PWM outputs"
	for i in ehrpwm.2\:0 ehrpwm.2\:1 ehrpwm.1\:0 ; do
		echo 1 > /sys/class/pwm/$i/request ;
		echo 1 > /sys/class/pwm/$i/period_freq ;
		echo 50 > /sys/class/pwm/$i/duty_percent ;
		echo 1 > /sys/class/pwm/$i/polarity ;
		echo 1 > /sys/class/pwm/$i/run ;
	done ;
	echo 1 > /sys/class/pwm/ehrpwm.2\:0/period_freq ;
	echo 2 > /sys/class/pwm/ehrpwm.1\:0/period_freq ;
	echo 50 > /sys/class/pwm/ehrpwm.1\:0/duty_percent ;
	echo 33 > /sys/class/pwm/ehrpwm.2\:1/duty_percent ;
	echo 66 > /sys/class/pwm/ehrpwm.2\:0/duty_percent ;

	echo "Turning BEBOPR I/O power on" ;
	# IO_PWR_ON  = R9 / GPIO1[6] / gpio38 /  gpmc_ad6
	# !IO_PWR_ON = R8 / GPIO1[2] / gpio34 /  gpmc_ad2
	for i in 38 34 ; do
		echo "$i" > /sys/class/gpio/export ;
		echo "out" > /sys/class/gpio/gpio$i/direction ;
		echo "0" > /sys/class/gpio/gpio$i/value ;
	done ;
	echo "1" > /sys/class/gpio/gpio38/value ;
fi
