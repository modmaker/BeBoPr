BeBoPr
======

> 2014-02-24 - The [BeBoPr++](https://github.com/modmaker/BeBoPr/wiki/The-BeBoPr-plus-plus-Cape) is available for immediate shipping. See [here](https://github.com/modmaker/BeBoPr/wiki/The-BeBoPr-plus-plus-Cape) for details.

> 2014-02-09 - **Announcement** of a **new BeBoPr**. The new [**BeBoPr++**](https://github.com/modmaker/BeBoPr/wiki/The-BeBoPr-plus-plus-Cape) integrates the BeBoPr and Bridge and replaces the BeBoPr+.
> 
> ![](http://imagizer.imageshack.us/v2/640x480q90/36/9jdd.jpg)

> 2013-09-30 - **Announcement of a [forum](http://forum.bebopr.info) at http://forum.bebopr.info** for BeBoPr related topics and q&a.

> 2013-08-30 - **Announcement of a new board in the BeBoPr series**. The new [**BeBoPr+**](http://circuitco.com/support/index.php?title=BeBoPr-Plus) *(BeBoPr-plus)* integrates the BeBoPr and Bridge. Initially the board is available as BeBoPr with the Bridge soldered on as shown on the picture below. Once this stock depletes, the combination will be replaced by the redesigned (functional equivalent) **BeBoPr++** *(BeBoPr plus-plus)* board as entry-level solution.
> 
> ![](http://img541.imageshack.us/img541/9715/4l1w.jpg)
> 
> The BeBoPr+ (and later the BeBoPr++) will be compatible with the current BeBoPr and Bridge combination. It requires no soldering or assembly otherwise. The cape is 100% BeagleBone and BeagleBone Black compatible and requires no fixes / hacks / patches. The BeBoPr++ will also mount the BeagleBone elevated above the board to provide easy access to all the connectors (serial, HDMI, USB and SD-card) once stacked on the BeBoPr. 
>   
> 2013-08-12 - The new [Bridge PCBs](https://github.com/modmaker/BeBoPr/wiki/BeBoPr-Bridge) are available for shipping. See [here](https://github.com/modmaker/BeBoPr/wiki/BeBoPr-Bridge) for details.


This repository contains the **(Open Source)** software to create 3D printer with a [BeagleBone](http://beagleboard.org/) and a [BeBoPr Cape](http://circuitco.com/support/index.php?title=BeBoPr_Cape). It also hosts the [**wiki**](https://github.com/modmaker/BeBoPr/wiki/Home-of-the-BeBoPr) pages with information about the BeBoPr Cape and software.

The software controlling a home built Prusa 3D-printer can be seen [here](http://www.youtube.com/watch?v=yfPLskLrslA&feature=youtu.be) and [here](http://www.youtube.com/watch?v=zzGiLhBEtcs&feature=youtu.be). The **step pulse generation**, including **acceleration** and **decelleration**, is done entirely in the **PRUSS coprocessor** that is part of the AM3359 ARM SoC on the BeagleBone.

### Bones and Capes
The [BeBoPr](https://github.com/modmaker/BeBoPr/wiki/The-BeBoPr-Cape) is a so called BeagleBone **'cape'**. It is the interface between the 3D printer hardware and the [BeagleBone](http://beagleboard.org/) processor module.

Originally the BeBoPr was designed for the **BeagleBone (classic/white)**, but there are [two ways](https://github.com/modmaker/BeBoPr/wiki/BeBoPr-does-Black) to use a **BeagleBone Black** with the [BeBoPr Cape](http://circuitco.com/support/index.php?title=BeBoPr_Cape). The first needs some changes to the BeagleBone [boot configuration](https://github.com/modmaker/BeBoPr/wiki/BeBoPr-goes-Black) and a [hardware patch](https://github.com/modmaker/BeBoPr/wiki/BeBoPr-Enable-Workaround) to the BeBoPr board. The second solution uses the BeagleBone Black without changes by adding a small [**Bridge** adapter](https://github.com/modmaker/BeBoPr/wiki/BeBoPr-Bridge) to the BeBoPr that makes it **100% BeagleBone Black compatible**. 

### LinuxCNC

Alternative software that can be used with the BeBoPr to create for example a CNC **lathe**, **mill**, **router** or **laser cutter** is [LinuxCNC](http://www.linuxcnc.org/) (formerly known as EMC2). It uses a **xenomai real-time kernel** on top of Debian 7 (wheezy) and a **PRUSS based step pulse generator**. More information and a complete SD-card image can be found [here](http://bb-lcnc.blogspot.nl/p/machinekit_16.html).

