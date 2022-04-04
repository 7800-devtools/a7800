# **A7800 - the Atari 7800 Emulator** #

What is A7800?
==============
A7800 is a fork of the MAME Atari 7800 driver, with several enhancements added.

* Support for emulation of Proline Joysticks, VCS Joysticks, Lightguns, Paddles, Driving Controllers, Keypads, Trak-Balls, Amiga Mice, and ST Mice.
* Maria DMA timing has been improved further, with the addition of accurate DMA hole penalties.
* A bug in the existing RIOT emulation has been fixed.
* POKEY sound emulation improvements.
* SALLY (CPU) and MARIA (Graphics chip) performance adjustments.
* Selectable and improved palettes with enhanced screen options.
* Audio indication of no ROM loaded silenced.
* BIOS file(s) no longer required and made optional.
* Streamlined UI including menu options to have an Atari 7800 system focus.
* Implementation of XM control registers updated.
* Graphical register updates made mid-scanline are now displayed mid-scanline.
* Bankset bankswitching support added.
* POKEY@800 added for non-banked, supergame, and bankset formats.
* Machine targets a7800dev and a7800pdev added, which display DMA usage per-scanline.

MAME compatibility and syntax has been maintained, to allow for the reuse of MAME configuration files and front-ends.
