Other tools included with MAME
==============================


ledutil.exe/ledutil.sh
----------------------

On Microsoft Windows, ledutil.exe can take control of your keyboard LEDs to mirror those that were present on some early arcade games (e.g. Asteroids)

Start **ledutil.exe** from the command line to enable LED handling. Run **ledutil.exe -kill** to stop the handler.

On SDLMAME platforms such as Mac OS X and Linux, **ledutil.sh** can be used. Use **ledutil.sh -a** to have it automatically close when you exit SDLMAME.


Developer-focused tools included with MAME
==========================================


pngcmp
------

This tool is used in regression testing to compare PNG screenshot results with the runtest.cmd script found in the source archive. This script works only on Microsoft Windows.


nltool
------

Discrete component conversion tool. Most users will not need to work with this.

nlwav
-----

Discrete component conversion and testing tool. Most users will not need to work with this.


jedutil
-------

PAL/PLA/PLD/GAL dump handling tool. It can convert between the industry-standard JED format and MAME's proprietary packed binary format and it can show logic equations for the types of devices it knows the internal logic of. Most users will not need to work with this.


ldresample
----------

This tool recompresses video data for laserdisc and VHS dumps. Most users will not need to work with this.


ldverify
--------

This tool is used for comparing laserdisc or VHS CHD images with the source AVI. Most users will not need to work with this.

unidasm
-------

Universal disassembler for many of the architectures supported in MAME. Most users will not need to work with this.
