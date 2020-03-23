// license:BSD-3-Clause
// copyright-holders:Wilbert Pol, Miodrag Milanovic
/***************************************************************************

    drivers/pc.c

Driver file for IBM PC, IBM PC XT, and related machines.

    PC-XT memory map

    00000-9FFFF   RAM
    A0000-AFFFF   NOP       or videoram EGA/VGA
    B0000-B7FFF   videoram  MDA, page #0
    B8000-BFFFF   videoram  CGA and/or MDA page #1, T1T mapped RAM
    C0000-C7FFF   NOP       or ROM EGA/VGA
    C8000-C9FFF   ROM       XT HDC #1
    CA000-CBFFF   ROM       XT HDC #2
    D0000-EFFFF   NOP       or 'adapter RAM'
    F0000-FDFFF   NOP       or ROM Basic + other Extensions
    FE000-FFFFF   ROM

Data General One / DG-1
=======================
Links: http://www.1000bit.it/ad/bro/datageneral/DG-ONE-PersonalSystem.pdf , http://www.1000bit.it/ad/bro/datageneral/DG-ONE-Interduction-PR.pdf , http://www.oldcomputers.net/data-general-one.html , http://forums.bannister.org/ubbthreads.php?ubb=showflat&Number=30897&page=all
Info: According to the discussion in the thread, the ROM we have is from the original version. Specs for later permutations can be found on oldcomputers.net
Form Factor: Laptop
CPU: 80C88 @ 4 MHz
RAM: 128K - 256K - 384K - 512K internally
Bus: no internal slots
Video: On board, Text mode 80x25 with 8x8 or 8x10 characters, CGA
Display: non-backlit LCD 640x256 pixels
Mass storage: 1/2x Floppy 3.5" 720K
On board Ports: Floppy, RTC, 1x RS232C + 1x RS232C/RS422 via 8251, speaker
Options: ext. 5.25" Floppy, int. Bell 103A 300 Baud Modem, 8087 FPU
Expansion: Expansion box, with 5 ISA slots and space for a 5.25" drive and a harddisk; specifically mentioned are the 5.25" drive, color graphics and memory expansion via ISA cards


Bondwell BW230 (Pro28 series)
=============
Links: http://gallery.fdd5-25.net/details.php?image_id=3463&sessionid=1eaeb42abdf2758a020b16204a2a8e5a ; http://www.zonadepruebas.com/viewtopic.php?t=3696 ; ftp://ftp.whtech.com/emulators/mess/old/Complete%20MESS%20Geneve%20emulation/mess/sysinfo/bondwell.htm
Info:   Info is hard to come by. A BW230 is nowhere to be found, the links about the Pro28 series suggest an XT compatible built around a passive backplane and a slot CPU. This is confirmed by the old MESS info.
Form Factor: Desktop
CPU: 8088 @ 4.77 MHz (MESS sysinfo: 3.75)/ 8 MHz
RAM: 512K / 640K
Bus: at least 2x ISA:   1)  CPU, RAM, Floppy controller
                        2)  Graphics, Game, Parallel
Video: Hercules/CGA
Mass storage: 1x 5.25" 360K and 20/30MB Harddisk.


Commodore PC-1
=============
Links: http://www.amiga-stuff.com/hardware/pc-i.html , http://www.zimmers.net/cbmpics/cpci.html
Form Factor: Desktop
CPU: 8088 @ 4.77 MHz
RAM: 512K / 640K
Bus: Proprietary expansion slot, carrying almost all ISA signals
Video: On board, MDA/Hercules/CGA
Mass storage: 1x 5.25" 360K
On board ports: Floppy, floppy expansion (for Amiga A1010/1011 (720 KB, 3.5") or A1020 (360 KB, 5.25" drives), speaker (but no speaker fitted), mouse,
Options: 8087 FPU
Expansion: Expansion box: 2x ISA


Commodore PC10 / PC20 / PC30
Links: http://www.zimmers.net/cbmpics/cpcs.html , https://de.wikipedia.org/wiki/Commodore_PC-10_bis_PC-60 , http://mingos-commodorepage.tumblr.com/post/123656301482/commodore-pc-20-beim-pc-20-handelt-es-sich-um
http://www.richardlagendijk.nl/cip/computer/item/pc20ii/de
Form Factor: Desktop
CPU: 8088 @ 4.77 MHz
RAM: 256K / 512K / 640K
BUS: 5x ISA
Video: MDA
Mass storage: PC10: 1 or 2x 5.25" 360K , PC20: 1x 360K + 10MB HD, PC30: 1x 360K + 20MB HD
On board ports: Floppy, serial, parallel, speaker
Options: 8087 FPU


Commodore PC-10 III
=============
Links: http://dostalgie.de/downloads/pc10III-20III/PC10III_OM_COMMODORE_EN_DE.pdf ; ftp://ftp.zimmers.net/pub/cbm-pc/documents/PC-8088-Information.txt
Info: PC10-III and PC20-III are the same machines - PC10 has two floppies, PC20 one floppy and one harddisk
Form Factor: Desktop
CPU: 8088 @ 4.77 MHz / 7.16 MHz / 9.54 MHz
RAM: 640K
Bus: 3x ISA
Video: On board: MDA/CGA/Hercules/Plantronics
Mass storage: 1x Floppy 5.25" 360K and (PC10) another 360K or (PC20) 3.5" harddisk
On board ports: Floppy, XT-IDE Harddisk, Mouse, serial, parallel, RTC, Speaker
Options: 8087 FPU


Zenith SuperSport
=======================
Links:  http://www.focushacks.com/zenith/myzenith.html , http://retro-computing.blogspot.de/2006/08/zds-supersport-laptop.html , http://www.minuszerodegrees.net/manuals/Zenith%20Data%20Systems/ZDS%20SupersPort%20-%20Service%20Manual.pdf
        http://www.minuszerodegrees.net/manuals/Zenith%20Data%20Systems/ZDS%20SupersPort%20-%20User%20and%20Technical%20Manual.pdf
Info: ZWL-184 to distinguish it from the later 80286 based models
Form Factor: Laptop
CPU: 80C88 @ 4.77 MHz or 8 MHz
RAM: 640 KB
Bus: no internal slots
Video: CGA
Display: The second link has a picture of a working SuperSport. This shows the LCD display with a green background and blue text/graphics.
Mass storage: 1x 3.5" 720K floppy and 1x720K floppy or 20MB harddisk
On board ports: serial, parallel, ext. keyboard, ext. CGA video, ext. floppy
Options: 2400 Baud Modem, 8087 FPU


Siemens Sicomp PC16-05
=======================
Links: http://www.computerwoche.de/a/siemens-erweitert-pc-16-programm,1169752 , http://www.phantom.sannata.ru/museum/siemens_pc_16_05.shtml
Info: Multitech PC/700 mainboard
Form Factor: Desktop
CPU: 8088 @ 4.77MHz / 8 MHz
RAM: 640KB
Bus: 6x ISA:    1) MDA/Hercules/CGA and parallel port
                2) Floppy, RTC and serial port
                3) (optional) MFM harddisk controller
Video: MDA/Hercules, exchangable via ISA-slot
Mass storage: 1x 5.25" 360K floppy and 1x 5.25" 360K floppy or MFM hard drive (10MB or 20MB)
On board ports: parallel, serial, beeper
Options: 8087 FPU


NCR PC4i
========
Links: http://www.minuszerodegrees.net/manuals/NCR/NCR%20PC4i%20-%20Technical%20Reference%20Manual%20-%20January%201986.pdf
Info:   The earlier PC4 is not quite IBM compatible, the "i" in PC4i indicates full IBM compatibility.
        The NCR Graphics card supports a special 640x400 video mode
Form Factor: All-in-one desktop
CPU: 8088 @ 4.77 MHz
RAM: 256K, expandable to 640K
Bus: 7x ISA:    1)  (optional) RAM expansion board
                2)  empty
                3)  32K Video/Graphics board (64K option)
                4)  (optional) Alpha board
                5)  empty
                6)  (optional) MFM harddisk controller
                7)  empty
Video: K510: 4KB Alpha for internal monitor; K511: 32KB Graphics for internal monitor; K512: 32KB upgrade for K512; K140: 16KB Graphics for external monitor; K141: 4KB Alpha for external monitor
Display: Mono or color CRT 640x400 pixel
Mass storage: 1x 5.25" 360K floppy and 1x 5.25" 360K floppy or 10 MB harddisk
On board ports: parallel, serial, speaker, floppy
Options: 8087 FPU, K101 memory upgrade in 64K steps, 1.2MB floppy and controller board


Olivetti M15
============
Links:  http://www.1000bit.it/ad/bro/olivetti/olivettiM15.pdf , http://electrickery.xs4all.nl/comp/m15/ , http://electrickery.xs4all.nl/comp/m15/doc/M15_InstallationAndOperationsGuide.pdf
        http://www.museotecnologicamente.it/olivetti-m-15-1987/ , http://www.museotecnologicamente.it/wp-content/uploads/M15_Depliant_inglese.pdf
Info: The info brochure has a picture of a working M15. This shows the LCD display with a green background and blue text/graphics.
Form Factor: Laptop
CPU: 80C88 @ 4.77 MHz
RAM: 256K / 512K
Bus: no internal slots
Video: 80x25 text mode, CGA
Display: LCD
Mass storage: 2x 3.5" 720K drives
Ports: serial, parallel, ext. floppy, RTC
Expansion: External 5.25" 360K floppy drive


IBM5550
=======
Information can be found at http://homepage3.nifty.com/ibm5550/index-e.html
It's a heavily modified IBM PC-XT machine, with a completely different
video HW too.


Sharp PC-7000
=============
Links: http://oldcomputers.net/sharp-pc7000.html , http://curtamania.com/curta/database/brand/sharp/Sharp%20PC-7000/index.html , http://pcmuseum.de/pc7000.html
Form Factor: Luggable
CPU: 8086 @ 4.77 MHz or 7.37 MHz
RAM: 320K / 704K
Bus: no internal slots
Video: 80x24 text, 600x200 pixel graphics
Display: electroluminescent mono backlit (blue) LCD
Mass storage: 2x 5.25" 360K floppies
On board ports: serial, parallel
Options: Modem, color video output


Sanyo MBC-16
============
Links:
Info: In the MBC-16 I had, the graphics card had a Sanyo sticker on it, so I assume that was the original graphics card for the machine.
Form Factor: Desktop
CPU: 8088 @ 8MHz
RAM: 640KB
Bus: 3x ISA:    1)  ATI Graphics Solution SR https://sites.google.com/site/atiwonderseriesdatabase/
Video: MDA/CGA/Plantronics
Mass storage: 1 or 2 5.25" 360K floppies, MFM harddisk on hardcard or via seperate controller
On board ports: serial, parallel, floppy


Atari PC1
=========
Links: http://www.ataripc.net/pc1-8088/ ; http://krap.pl/mirrorz/atari/www.atari-computermuseum.de/pc.htm ; http://www.atari-computermuseum.de/pc1.htm
Form Factor: Desktop
CPU: 8088 @ 4.77 MHz / 8 MHz
RAM: 512K / 640KB
Bus: ISA signals available on board, no slot
Video: Hercules/CGA/EGA
Mass storage: 1 5.25" 360K floppy
On board ports: floppy, graphics, parallel, serial, mouse, external floppy
Options: 8087 FPU
Expansion: Up to two external floppy drives: PCF554, SF314 or SF354


Atari PC2
Links: http://www.binarydinosaurs.co.uk/Museum/atari/pc2.php ; http://www.ataripc.net/pc2-8088/ ; http://www.ataripc.net/components/
Info: The Atari PC2 mainboard has only one ISA slot, but is expanded via a four slot riser card. BIOS is identical to later PC1 and PC3
CPU: 8088 @ 4.77 MHz / 8 MHz
RAM: 512K / 640KB
Bus: 4x ISA
Video: Hercules/CGA/EGA
Mass storage: 1 5.25" 360K floppy and 1 5.25" 360K floppy or 20MB hard drive
On board ports: floppy, graphics, parallel, serial, mouse
Expansion: 8087 FPU

Atari PC3
=========
Links: http://www.atari-computermuseum.de/pc1.htm , http://trelohra.blogspot.de/2015/06/atari-pc3.html , http://www.ataripc.net/pc3-8088/
Form Factor: Desktop
CPU: 8088 @ 4.77 MHz or 8 MHz
RAM: 640K
Bus: 5x ISA:    1) Adaptec ACB-2072 RLL Controller Card
Video: On-board MDA/CGA/Hercules/EGA
Mass storage: 1x 5.25" 360K floppy + 30MB RLL harddisk
On board ports: floppy, parallel, serial, mouse, speaker
Options: 8087 FPU


Eagle 1600
==========
Links: https://archive.org/details/bitsavers_eagleCompu00Brochure_9975235 , http://www.vcfed.org/forum/showthread.php?49510-Eagle-Computer-model-list , http://bitsavers.trailing-edge.com/pdf/eagleComputer/1600/1600_Series_Training_Notes.pdf
Info:   Eagle 1620 - 8086/128K, 2 Quad density floppy drives, 4 Expansion slots available, ~1983, Eagle 1630 - 8086/128K, 1 Quad density floppy drive, 10MB HD, 3 Expansion Slots available (Same as 1620 with hard drive), ~1983Eagle 1640 - 8086/512K, 1 Quad density floppy drive, 32MB HD, 3 Expansion Slots available, ~1984
        The native floppy format is 780K, 2 sides, 80 tracks/side, 1024 bytes/sector, 5 sectors per track. Standard 360K disks can be read
        Holding "T" and resetting starts a system diagnostics test
Form Factor: Desktop
CPU: 8086 @ 8 MHz
RAM: 128K / 512K
Bus: 8xISA:     1) SASI board, connects to a XEBEC Sl410 SASI => MFM bridge board
                2) Floppy controller
                3) empty
                4) Video/graphics controller board
                5) empty
                6) empty
                7) Serial board: 2x serial, one sync/async, one async only
                8) Parallel board
Video: 80x25 text mode, 720x352 pixel graphics mode
Mass storage: 1x 5.25" QD 780K floppy and 1x 5.25" QD 820K floppy or 10/30MB MFM harddisk
Options: 8087 FPU, EagleNet File server, EightPort serial card, High Resolution color board and video, Video Cassette Adapter board for 80MB backup on video cassette

VTech Laser Turbo XT
=======================
Links: http://minuszerodegrees.net/manuals.htm#VTech , http://minuszerodegrees.net/manuals/VTech/VTech%20-%20Laser%20Turbo%20XT%20-%20Brochure.pdf
Form Factor: Desktop
CPU: 8088 @ 4.77 MHz or 10 MHz
RAM: 512K / 640K, additionally 512K or 1M EMS on board
Bus: 8xISA:     1) Monochrome graphics/color graphics card
                2) Multi I/O Card (Floppy, 2x serial, parallel, game, RTC)
                3) (optional) hard disk controller
Video: MDA/CGA/Hercules
Mass storage: 2x 5.25" 360K floppies and 1 or 2 harddisks (20MB / 30MB / 40MB)
On board ports: speaker
Options: 8087 FPU

VTech Laser XT/3
=======================
Links: http://minuszerodegrees.net/manuals.htm#VTech , http://th99.classic-computing.de/src/v/U-Z/52547.htm
Form Factor: Desktop
CPU: 8088 @ 4.77MHz or 10 MHz
RAM: 512K / 640K, additionally 512K or 1M EMS on board
Bus: 8x ISA:    1) Monochrome graphics/color graphics card http://th99.classic-computing.de/src/v/U-Z/52547.htm , alternatively an EGA card
                2) Multi I/O Card (Floppy, 2x serial, 1x parallel, game, RTC) http://th99.classic-computing.de/src/i/U-Z/52519.htm
                3) (optional) hard disk controller
Video: MDA/Hercules/CGA
Mass storage: 2x 5.25" 360K or 1x 5.25" 360K and 1x 3.5" 720K, additional harddisk optional
On board ports: speaker
Options: 8087 FPU

***************************************************************************/


#include "emu.h"
#include "machine/genpc.h"
#include "cpu/i86/i86.h"
#include "bus/isa/isa.h"
#include "bus/isa/isa_cards.h"
#include "bus/pc_kbd/keyboards.h"
#include "softlist.h"

class pc_state : public driver_device
{
public:
	pc_state(const machine_config &mconfig, device_type type, const char *tag) :
		driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu")
	{ }

	required_device<cpu_device> m_maincpu;

	DECLARE_READ8_MEMBER(unk_r);

	DECLARE_DRIVER_INIT(bondwell);

	DECLARE_INPUT_CHANGED_MEMBER(pc_turbo_callback);

	double m_turbo_off_speed;
};

static ADDRESS_MAP_START( pc8_map, AS_PROGRAM, 8, pc_state )
	ADDRESS_MAP_UNMAP_HIGH
	AM_RANGE(0xf0000, 0xfffff) AM_ROM AM_REGION("bios", 0)
ADDRESS_MAP_END

static ADDRESS_MAP_START( zenith_map, AS_PROGRAM, 8, pc_state )
	ADDRESS_MAP_UNMAP_HIGH
	AM_RANGE(0xf0000, 0xf7fff) AM_RAM
	AM_RANGE(0xf8000, 0xfffff) AM_ROM AM_REGION("bios", 0x8000)
ADDRESS_MAP_END

static ADDRESS_MAP_START( pc16_map, AS_PROGRAM, 16, pc_state )
	ADDRESS_MAP_UNMAP_HIGH
	AM_RANGE(0xf0000, 0xfffff) AM_ROM AM_REGION("bios", 0)
ADDRESS_MAP_END

static ADDRESS_MAP_START(pc8_io, AS_IO, 8, pc_state )
	ADDRESS_MAP_UNMAP_HIGH
	AM_RANGE(0x0000, 0x00ff) AM_DEVICE("mb", ibm5160_mb_device, map)
ADDRESS_MAP_END

static ADDRESS_MAP_START(pc16_io, AS_IO, 16, pc_state )
	ADDRESS_MAP_UNMAP_HIGH
	AM_RANGE(0x0070, 0x007f) AM_RAM // needed for Poisk-2
	AM_RANGE(0x0000, 0x00ff) AM_DEVICE8("mb", ibm5160_mb_device, map, 0xffff)
ADDRESS_MAP_END

READ8_MEMBER(pc_state::unk_r)
{
	return 0;
}

static ADDRESS_MAP_START(ibm5550_io, AS_IO, 16, pc_state )
	ADDRESS_MAP_UNMAP_HIGH
	AM_RANGE(0x00a0, 0x00a1) AM_READ8(unk_r, 0x00ff )
	AM_RANGE(0x0000, 0x00ff) AM_DEVICE8("mb", ibm5160_mb_device, map, 0xffff)
ADDRESS_MAP_END

INPUT_CHANGED_MEMBER(pc_state::pc_turbo_callback)
{
	m_maincpu->set_clock_scale((newval & 2) ? 1 : m_turbo_off_speed);
}

DRIVER_INIT_MEMBER(pc_state,bondwell)
{
	m_turbo_off_speed = 4.77/12;
}

static INPUT_PORTS_START( pccga )
	PORT_START("DSW1") /* IN2 */
	PORT_DIPNAME( 0x80, 0x80, "COM1: enable")
	PORT_DIPSETTING(    0x00, DEF_STR( No ) )
	PORT_DIPSETTING(    0x80, DEF_STR( Yes ) )
	PORT_DIPNAME( 0x40, 0x40, "COM2: enable")
	PORT_DIPSETTING(    0x00, DEF_STR( No ) )
	PORT_DIPSETTING(    0x40, DEF_STR( Yes ) )
	PORT_DIPNAME( 0x20, 0x00, "COM3: enable")
	PORT_DIPSETTING(    0x00, DEF_STR( No ) )
	PORT_DIPSETTING(    0x20, DEF_STR( Yes ) )
	PORT_DIPNAME( 0x10, 0x00, "COM4: enable")
	PORT_DIPSETTING(    0x00, DEF_STR( No ) )
	PORT_DIPSETTING(    0x10, DEF_STR( Yes ) )
	PORT_DIPNAME( 0x08, 0x08, "LPT1: enable")
	PORT_DIPSETTING(    0x00, DEF_STR( No ) )
	PORT_DIPSETTING(    0x08, DEF_STR( Yes ) )
	PORT_DIPNAME( 0x04, 0x00, "LPT2: enable")
	PORT_DIPSETTING(    0x00, DEF_STR( No ) )
	PORT_DIPSETTING(    0x04, DEF_STR( Yes ) )
	PORT_DIPNAME( 0x02, 0x00, "LPT3: enable")
	PORT_DIPSETTING(    0x00, DEF_STR( No ) )
	PORT_DIPSETTING(    0x02, DEF_STR( Yes ) )
	PORT_DIPNAME( 0x01, 0x00, "Game port enable")
	PORT_DIPSETTING(    0x00, DEF_STR( No ) )
	PORT_DIPSETTING(    0x01, DEF_STR( Yes ) )

	PORT_START("DSW2") /* IN3 */
	PORT_DIPNAME( 0x08, 0x08, "HDC1 (C800:0 port 320-323)")
	PORT_DIPSETTING(    0x00, DEF_STR( No ) )
	PORT_DIPSETTING(    0x08, DEF_STR( Yes ) )
	PORT_DIPNAME( 0x04, 0x04, "HDC2 (CA00:0 port 324-327)")
	PORT_DIPSETTING(    0x00, DEF_STR( No ) )
	PORT_DIPSETTING(    0x04, DEF_STR( Yes ) )
	PORT_BIT( 0x02, 0x02,   IPT_UNUSED ) /* no turbo switch */
	PORT_BIT( 0x01, 0x01,   IPT_UNUSED )
INPUT_PORTS_END


static DEVICE_INPUT_DEFAULTS_START( pccga )
	DEVICE_INPUT_DEFAULTS("DSW0", 0x30, 0x20)
DEVICE_INPUT_DEFAULTS_END


#define MCFG_CPU_PC(mem, port, type, clock) \
	MCFG_CPU_ADD("maincpu", type, clock)                \
	MCFG_CPU_PROGRAM_MAP(mem##_map) \
	MCFG_CPU_IO_MAP(port##_io) \
	MCFG_CPU_IRQ_ACKNOWLEDGE_DEVICE("mb:pic8259", pic8259_device, inta_cb)


static MACHINE_CONFIG_START( pccga )
	/* basic machine hardware */
	MCFG_CPU_PC(pc8, pc8, I8088, 4772720)   /* 4,77 MHz */

	MCFG_IBM5160_MOTHERBOARD_ADD("mb", "maincpu")
	MCFG_DEVICE_INPUT_DEFAULTS(pccga)

	MCFG_ISA8_SLOT_ADD("mb:isa", "isa1", pc_isa8_cards, "cga", false)
	MCFG_ISA8_SLOT_ADD("mb:isa", "isa2", pc_isa8_cards, "fdc_xt", false)
	MCFG_ISA8_SLOT_ADD("mb:isa", "isa3", pc_isa8_cards, "lpt", false)
	MCFG_ISA8_SLOT_ADD("mb:isa", "isa4", pc_isa8_cards, "com", false)
	MCFG_ISA8_SLOT_ADD("mb:isa", "isa5", pc_isa8_cards, nullptr, false)

	/* keyboard */
	MCFG_PC_KBDC_SLOT_ADD("mb:pc_kbdc", "kbd", pc_xt_keyboards, STR_KBD_IBM_PC_XT_83)
	/* internal ram */
	MCFG_RAM_ADD(RAM_TAG)
	MCFG_RAM_DEFAULT_SIZE("640K")
	MCFG_RAM_EXTRA_OPTIONS("64K, 128K, 256K, 512K")

	/* software lists */
	MCFG_SOFTWARE_LIST_ADD("disk_list","ibm5150")
MACHINE_CONFIG_END


static MACHINE_CONFIG_START( cfg_dual_720K )
	MCFG_DEVICE_MODIFY("fdc:0")
	MCFG_SLOT_DEFAULT_OPTION("35dd")
	MCFG_DEVICE_MODIFY("fdc:1")
	MCFG_SLOT_DEFAULT_OPTION("35dd")
MACHINE_CONFIG_END

static MACHINE_CONFIG_START( cfg_single_360K )
	MCFG_DEVICE_MODIFY("fdc:0")
	MCFG_SLOT_DEFAULT_OPTION("525dd")
	MCFG_SLOT_FIXED(true)
	MCFG_DEVICE_REMOVE("fdc:1")
MACHINE_CONFIG_END

//Data General One
static MACHINE_CONFIG_DERIVED( dgone, pccga )
	MCFG_DEVICE_MODIFY("isa2")
	MCFG_SLOT_OPTION_MACHINE_CONFIG("fdc_xt", cfg_dual_720K)
MACHINE_CONFIG_END


// Bondwell BW230
static INPUT_PORTS_START( bondwell )
	PORT_INCLUDE(pccga)

	PORT_MODIFY("DSW2") /* IN3 */
	PORT_DIPNAME( 0x02, 0x02, "Turbo Switch" ) PORT_CHANGED_MEMBER(DEVICE_SELF, pc_state, pc_turbo_callback, 0)
	PORT_DIPSETTING(    0x00, "Off (4.77 MHz)" )
	PORT_DIPSETTING(    0x02, "On (12 MHz)" )
INPUT_PORTS_END

static MACHINE_CONFIG_DERIVED(bondwell, pccga)
	MCFG_DEVICE_REMOVE("maincpu")
	MCFG_CPU_PC(pc8, pc8, I8088, 4772720) // turbo?
MACHINE_CONFIG_END


// Schetmash Iskra-3104
static DEVICE_INPUT_DEFAULTS_START( iskr3104 )
	DEVICE_INPUT_DEFAULTS("DSW0", 0x30, 0x00)
DEVICE_INPUT_DEFAULTS_END

static MACHINE_CONFIG_START( iskr3104 )
	/* basic machine hardware */
	MCFG_CPU_PC(pc16, pc16, I8086, 4772720)

	MCFG_IBM5160_MOTHERBOARD_ADD("mb", "maincpu")
	MCFG_DEVICE_INPUT_DEFAULTS(iskr3104)

	MCFG_ISA8_SLOT_ADD("mb:isa", "isa1", pc_isa8_cards, "ega", false)
	MCFG_SLOT_OPTION_DEFAULT_BIOS("ega", "iskr3104")

	MCFG_ISA8_SLOT_ADD("mb:isa", "isa2", pc_isa8_cards, "fdc_xt", false)
	MCFG_ISA8_SLOT_ADD("mb:isa", "isa3", pc_isa8_cards, "lpt", false)
	MCFG_ISA8_SLOT_ADD("mb:isa", "isa4", pc_isa8_cards, "com", false)

	/* keyboard */
	MCFG_PC_KBDC_SLOT_ADD("mb:pc_kbdc", "kbd", pc_xt_keyboards, STR_KBD_IBM_PC_XT_83)
	/* internal ram */
	MCFG_RAM_ADD(RAM_TAG)
	MCFG_RAM_DEFAULT_SIZE("640K")
	MCFG_RAM_EXTRA_OPTIONS("64K, 128K, 256K, 512K")
MACHINE_CONFIG_END


//Poisk-2
static MACHINE_CONFIG_START( poisk2 )
	/* basic machine hardware */
	MCFG_CPU_PC(pc16, pc16, I8086, 4772720)

	MCFG_IBM5160_MOTHERBOARD_ADD("mb", "maincpu")
	MCFG_DEVICE_INPUT_DEFAULTS(pccga)

	MCFG_ISA8_SLOT_ADD("mb:isa", "isa1", pc_isa8_cards, "cga_poisk2", false)
	MCFG_ISA8_SLOT_ADD("mb:isa", "isa2", pc_isa8_cards, "fdc_xt", false)
	MCFG_ISA8_SLOT_ADD("mb:isa", "isa3", pc_isa8_cards, "lpt", false)
	MCFG_ISA8_SLOT_ADD("mb:isa", "isa4", pc_isa8_cards, "com", false)

	/* keyboard */
	MCFG_PC_KBDC_SLOT_ADD("mb:pc_kbdc", "kbd", pc_xt_keyboards, STR_KBD_IBM_PC_XT_83)
	/* internal ram */
	MCFG_RAM_ADD(RAM_TAG)
	MCFG_RAM_DEFAULT_SIZE("640K")
	MCFG_RAM_EXTRA_OPTIONS("64K, 128K, 256K, 512K")
MACHINE_CONFIG_END


//MK-88
static MACHINE_CONFIG_DERIVED(mk88, poisk2)
	MCFG_DEVICE_MODIFY("isa1")
	MCFG_SLOT_DEFAULT_OPTION("cga_ec1841")
MACHINE_CONFIG_END


// Zenith SuperSport
static MACHINE_CONFIG_START( zenith )
	/* basic machine hardware */
	MCFG_CPU_PC(zenith, pc8, I8088, XTAL_14_31818MHz/3) /* 4,77 MHz */

	MCFG_IBM5150_MOTHERBOARD_ADD("mb", "maincpu")
	MCFG_DEVICE_INPUT_DEFAULTS(pccga)

	MCFG_ISA8_SLOT_ADD("mb:isa", "isa1", pc_isa8_cards, "cga", false)
	MCFG_ISA8_SLOT_ADD("mb:isa", "isa2", pc_isa8_cards, "fdc_xt", false)
	MCFG_SLOT_OPTION_MACHINE_CONFIG("fdc_xt", cfg_dual_720K)
	MCFG_ISA8_SLOT_ADD("mb:isa", "isa3", pc_isa8_cards, "lpt", false)
	MCFG_ISA8_SLOT_ADD("mb:isa", "isa4", pc_isa8_cards, "com", false)

	/* keyboard */
	MCFG_PC_KBDC_SLOT_ADD("mb:pc_kbdc", "kbd", pc_xt_keyboards, STR_KBD_IBM_PC_XT_83)
	/* internal ram */
	MCFG_RAM_ADD(RAM_TAG)
	MCFG_RAM_DEFAULT_SIZE("640K")
	MCFG_RAM_EXTRA_OPTIONS("128K, 256K, 512K")

	/* software lists */
	MCFG_SOFTWARE_LIST_ADD("disk_list","ibm5150")
MACHINE_CONFIG_END


//NCR PC4i
static MACHINE_CONFIG_DERIVED ( ncrpc4i, pccga )
	//MCFG_DEVICE_MODIFY("mb:isa")
	MCFG_ISA8_SLOT_ADD("mb:isa", "isa6", pc_isa8_cards, nullptr, false)
	MCFG_ISA8_SLOT_ADD("mb:isa", "isa7", pc_isa8_cards, nullptr, false)

	MCFG_DEVICE_MODIFY(RAM_TAG)
	MCFG_RAM_DEFAULT_SIZE("640K")
	MCFG_RAM_EXTRA_OPTIONS("64K, 128K, 256K, 512K")
MACHINE_CONFIG_END


// Siemens Sicomp PC16-05
static DEVICE_INPUT_DEFAULTS_START( siemens )
	DEVICE_INPUT_DEFAULTS("DSW0", 0x30, 0x30)
DEVICE_INPUT_DEFAULTS_END

static MACHINE_CONFIG_START( siemens )
	/* basic machine hardware */
	MCFG_CPU_PC(pc8, pc8, I8088, XTAL_14_31818MHz/3) /* 4,77 MHz */

	MCFG_IBM5150_MOTHERBOARD_ADD("mb", "maincpu")
	MCFG_DEVICE_INPUT_DEFAULTS(siemens)

	MCFG_ISA8_SLOT_ADD("mb:isa", "isa1", pc_isa8_cards, "hercules", false)
	MCFG_ISA8_SLOT_ADD("mb:isa", "isa2", pc_isa8_cards, "fdc_xt", false)
	MCFG_ISA8_SLOT_ADD("mb:isa", "isa3", pc_isa8_cards, "lpt", false)
	MCFG_ISA8_SLOT_ADD("mb:isa", "isa4", pc_isa8_cards, "com", false)
	MCFG_ISA8_SLOT_ADD("mb:isa", "isa5", pc_isa8_cards, "hdc", false)
	MCFG_ISA8_SLOT_ADD("mb:isa", "isa6", pc_isa8_cards, nullptr, false)

	/* keyboard */
	MCFG_PC_KBDC_SLOT_ADD("mb:pc_kbdc", "kbd", pc_xt_keyboards, STR_KBD_IBM_PC_XT_83)
	/* internal ram */
	MCFG_RAM_ADD(RAM_TAG)
	MCFG_RAM_DEFAULT_SIZE("640K")
	MCFG_RAM_EXTRA_OPTIONS("64K, 128K, 256K, 512K")

MACHINE_CONFIG_END


// IBM 5550
static MACHINE_CONFIG_START( ibm5550 )
	/* basic machine hardware */
	MCFG_CPU_PC(pc16, ibm5550, I8086, 8000000)

	MCFG_IBM5160_MOTHERBOARD_ADD("mb", "maincpu")
	MCFG_DEVICE_INPUT_DEFAULTS(pccga)

	MCFG_ISA8_SLOT_ADD("mb:isa", "isa1", pc_isa8_cards, "cga", false)
	MCFG_ISA8_SLOT_ADD("mb:isa", "isa2", pc_isa8_cards, "fdc_xt", false)
	MCFG_ISA8_SLOT_ADD("mb:isa", "isa3", pc_isa8_cards, "lpt", false)
	MCFG_ISA8_SLOT_ADD("mb:isa", "isa4", pc_isa8_cards, "com", false)

	/* keyboard */
	MCFG_PC_KBDC_SLOT_ADD("mb:pc_kbdc", "kbd", pc_xt_keyboards, STR_KBD_IBM_PC_XT_83)
	/* internal ram */
	MCFG_RAM_ADD(RAM_TAG)
	MCFG_RAM_DEFAULT_SIZE("640K")
	MCFG_RAM_EXTRA_OPTIONS("64K, 128K, 256K, 512K")
MACHINE_CONFIG_END


// Olivetti M15
static DEVICE_INPUT_DEFAULTS_START( m15 )
	DEVICE_INPUT_DEFAULTS("DSW0", 0x30, 0x20) // TODO: document correct dip settings
	DEVICE_INPUT_DEFAULTS("DSW0", 0x01, 0x00)
DEVICE_INPUT_DEFAULTS_END

static MACHINE_CONFIG_DERIVED(m15, pccga)
	MCFG_DEVICE_MODIFY("mb")
	MCFG_DEVICE_INPUT_DEFAULTS(m15)
	MCFG_DEVICE_MODIFY("isa2")
	MCFG_SLOT_OPTION_MACHINE_CONFIG("fdc_xt", cfg_dual_720K)
	MCFG_DEVICE_MODIFY(RAM_TAG)
	MCFG_RAM_DEFAULT_SIZE("448K")
	MCFG_RAM_EXTRA_OPTIONS("16K, 160K, 304K")
MACHINE_CONFIG_END


// Atari PC1
static MACHINE_CONFIG_DERIVED(ataripc1, pccga)
	MCFG_DEVICE_MODIFY("isa1")
	MCFG_SLOT_DEFAULT_OPTION("ega")
	MCFG_DEVICE_MODIFY("isa2")
	MCFG_SLOT_OPTION_MACHINE_CONFIG("fdc_xt", cfg_single_360K)
MACHINE_CONFIG_END


//Eagle 1600
static MACHINE_CONFIG_DERIVED(eagle1600, pccga)
	MCFG_DEVICE_REMOVE("maincpu")
	MCFG_CPU_PC(pc16, pc16, I8086, 8000000)
MACHINE_CONFIG_END


//Laser XT/3
static MACHINE_CONFIG_START( laser_xt3 )
	MCFG_CPU_PC(pc8, pc8, I8088, XTAL_14_31818MHz/3) /* 4,77 MHz */

	MCFG_IBM5160_MOTHERBOARD_ADD("mb","maincpu")
	MCFG_DEVICE_INPUT_DEFAULTS(pccga)

	MCFG_ISA8_SLOT_ADD("mb:isa", "isa1", pc_isa8_cards, "cga", false)
	MCFG_ISA8_SLOT_ADD("mb:isa", "isa2", pc_isa8_cards, "com", false) // Multi I/O card (includes FDC)
	MCFG_ISA8_SLOT_ADD("mb:isa", "isa3", pc_isa8_cards, "fdc_xt", false) // floppy drive A is 5.25" 360K and B is 3.5" 720K
	MCFG_ISA8_SLOT_ADD("mb:isa", "isa4", pc_isa8_cards, nullptr, false)
	MCFG_ISA8_SLOT_ADD("mb:isa", "isa5", pc_isa8_cards, nullptr, false)
	MCFG_ISA8_SLOT_ADD("mb:isa", "isa6", pc_isa8_cards, nullptr, false)
	MCFG_ISA8_SLOT_ADD("mb:isa", "isa7", pc_isa8_cards, nullptr, false)
	MCFG_ISA8_SLOT_ADD("mb:isa", "isa8", pc_isa8_cards, nullptr, false)

	/* keyboard */
	MCFG_PC_KBDC_SLOT_ADD("mb:pc_kbdc", "kbd", pc_xt_keyboards, STR_KBD_IBM_PC_XT_83)

	/* internal ram */
	MCFG_RAM_ADD(RAM_TAG)
	MCFG_RAM_DEFAULT_SIZE("640K")
	MCFG_RAM_EXTRA_OPTIONS("512K,1024K,1536K,1664K")

	/* software lists */
	MCFG_SOFTWARE_LIST_ADD("disk_list","ibm5150")
MACHINE_CONFIG_END


//Laser Turbo XT
static MACHINE_CONFIG_START( laser_turbo_xt )
	MCFG_CPU_PC(pc8, pc8, I8088, XTAL_14_31818MHz/3) /* 4,77 MHz */

	MCFG_IBM5160_MOTHERBOARD_ADD("mb","maincpu")
	MCFG_DEVICE_INPUT_DEFAULTS(pccga)

	MCFG_ISA8_SLOT_ADD("mb:isa", "isa1", pc_isa8_cards, "cga", false)
	MCFG_ISA8_SLOT_ADD("mb:isa", "isa2", pc_isa8_cards, "com", false) // Multi I/O card (includes FDC)
	MCFG_ISA8_SLOT_ADD("mb:isa", "isa3", pc_isa8_cards, "fdc_xt", false)
	MCFG_ISA8_SLOT_ADD("mb:isa", "isa4", pc_isa8_cards, nullptr, false)
	MCFG_ISA8_SLOT_ADD("mb:isa", "isa5", pc_isa8_cards, nullptr, false)
	MCFG_ISA8_SLOT_ADD("mb:isa", "isa6", pc_isa8_cards, nullptr, false)
	MCFG_ISA8_SLOT_ADD("mb:isa", "isa7", pc_isa8_cards, nullptr, false)
	MCFG_ISA8_SLOT_ADD("mb:isa", "isa8", pc_isa8_cards, nullptr, false)

	/* keyboard */
	MCFG_PC_KBDC_SLOT_ADD("mb:pc_kbdc", "kbd", pc_xt_keyboards, STR_KBD_IBM_PC_XT_83)

	/* internal ram */
	MCFG_RAM_ADD(RAM_TAG)
	MCFG_RAM_DEFAULT_SIZE("640K")
	MCFG_RAM_EXTRA_OPTIONS("512K,768K,896K,1024K,1408K,1536K,1664K")

	/* software lists */
	MCFG_SOFTWARE_LIST_ADD("disk_list","ibm5150")
MACHINE_CONFIG_END

//**************************************************************************
//  ROM DEFINITIONS
//**************************************************************************

ROM_START( dgone )
	ROM_REGION(0x10000,"bios", 0)
	ROM_LOAD( "dgone.bin",  0x8000, 0x08000, CRC(2c38c86e) SHA1(c0f85a000d1d13cd354965689e925d677822549e))
ROM_END

ROM_START( bw230 )
	ROM_REGION(0x10000,"bios", 0)
	ROM_LOAD("bondwell.bin", 0xe000, 0x2000, CRC(d435a405) SHA1(a57c705d1144c7b61940b6f5c05d785c272fc9bb))
ROM_END

ROM_START( compc1 )
	ROM_REGION(0x10000, "bios", 0)
	ROM_LOAD("pc1_bios.bin", 0xc000, 0x4000, CRC(e37367c8) SHA1(9aac9c38b4ebdb9a740e393199c2eff75a0bde03))
	ROM_REGION(0x8000, "gfx1", 0)
	ROM_LOAD("pc1_char.bin", 0x0000, 0x4000, CRC(ee6c27f0) SHA1(e769cc3a49a1d708bd74eb4ac85bb6ea67220d38))
ROM_END

ROM_START( iskr3104 )
	ROM_REGION16_LE(0x10000,"bios", 0)
	ROMX_LOAD( "198.bin", 0xc000, 0x2000, CRC(bcfd8e41) SHA1(e21ddf78839aa51fa5feb23f511ff5e2da31b433),ROM_SKIP(1))
	ROMX_LOAD( "199.bin", 0xc001, 0x2000, CRC(2da5fe79) SHA1(14d5dccc141a0b3367f7f8a7188306fdf03c2b6c),ROM_SKIP(1))
	// EGA card from Iskra-3104
	//ROMX_LOAD( "143-03.bin", 0xc0001, 0x2000, CRC(d0706345) SHA1(e04bb40d944426a4ae2e3a614d3f4953d7132ede),ROM_SKIP(1))
	//ROMX_LOAD( "143-02.bin", 0xc0000, 0x2000, CRC(c8c18ebb) SHA1(fd6dac76d43ab8b582e70f1d5cc931d679036fb9),ROM_SKIP(1))
ROM_END

ROM_START( mk88 )
	ROM_REGION16_LE(0x10000,"bios", 0)
	ROM_DEFAULT_BIOS("v392")
	ROM_SYSTEM_BIOS(0, "v290", "v2.90")
	ROMX_LOAD( "mk88m.bin", 0xc000, 0x2000, CRC(09c9da3b) SHA1(d1e7ad23b5f5b3576ad128c1198294129754f39f), ROM_BIOS(1))
	ROMX_LOAD( "mk88b.bin", 0xe000, 0x2000, CRC(8a922476) SHA1(c19c3644ab92fd12e13f32b410cd26e3c844a03b), ROM_BIOS(1))
	ROM_SYSTEM_BIOS(1, "v391", "v3.91")
	ROMX_LOAD( "mkm.bin", 0xc000, 0x2000, CRC(65f979e8) SHA1(13e85be9bc8ceb5ab9e559e7d0089e26fbbb84fc), ROM_BIOS(2))
	ROMX_LOAD( "mkb.bin", 0xe000, 0x2000, CRC(830a0447) SHA1(11bc200fdbcfbbe335f4c282020750c0b5ca4167), ROM_BIOS(2))
	ROM_SYSTEM_BIOS(2, "v392", "v3.92")
	ROMX_LOAD( "m88.bin", 0xc000, 0x2000, CRC(fe1b4e36) SHA1(fcb420af0ff09a7d43fcb9b7d0b0233a2071c159), ROM_BIOS(3))
	ROMX_LOAD( "b88.bin", 0xe000, 0x2000, CRC(58a418df) SHA1(216398d4e4302ee7efcc2c8f9ff9d8a1161229ea), ROM_BIOS(3))
ROM_END

ROM_START( poisk2 )
	ROM_REGION16_LE(0x10000,"bios", 0)
	ROM_SYSTEM_BIOS(0, "v20", "v2.0")
	ROMX_LOAD( "b_p2_20h.rf4", 0xc001, 0x2000, CRC(d53189b7) SHA1(ace40f1a40642b51fe5d2874acef81e48768b23b), ROM_SKIP(1) | ROM_BIOS(1))
	ROMX_LOAD( "b_p2_20l.rf4", 0xc000, 0x2000, CRC(2d61fcc9) SHA1(11873c8741ba37d6c2fe1f482296aece514b7618), ROM_SKIP(1) | ROM_BIOS(1))
	ROM_SYSTEM_BIOS(1, "v21", "v2.1")
	ROMX_LOAD( "b_p2_21h.rf4", 0xc001, 0x2000, CRC(22197297) SHA1(506c7e63027f734d62ef537f484024548546011f), ROM_SKIP(1) | ROM_BIOS(2))
	ROMX_LOAD( "b_p2_21l.rf4", 0xc000, 0x2000, CRC(0eb2ea7f) SHA1(67bb5fec53ebfa2a5cad2a3d3d595678d6023024), ROM_SKIP(1) | ROM_BIOS(2))
	ROM_SYSTEM_BIOS(2, "v24", "v2.4")
	ROMX_LOAD( "b_p2_24h.rf4", 0xc001, 0x2000, CRC(ea842c9e) SHA1(dcdbf27374149dae0ef76d410cc6c615d9b99372), ROM_SKIP(1) | ROM_BIOS(3))
	ROMX_LOAD( "b_p2_24l.rf4", 0xc000, 0x2000, CRC(02f21250) SHA1(f0b133fb4470bddf2f7bf59688cf68198ed8ce55), ROM_SKIP(1) | ROM_BIOS(3))
	ROM_SYSTEM_BIOS(3, "v21d", "v2.1d")
	ROMX_LOAD( "opp2_1h.rf4", 0xc001, 0x2000, CRC(b7cd7f4f) SHA1(ac473822fb44d7b898d628732cf0a27fcb4d26d6), ROM_SKIP(1) | ROM_BIOS(4))
	ROMX_LOAD( "opp2_1l.rf4", 0xc000, 0x2000, CRC(1971dca3) SHA1(ecd61cc7952af834d8abc11db372c3e70775489d), ROM_SKIP(1) | ROM_BIOS(4))
	ROM_SYSTEM_BIOS(4, "v22d", "v2.2d")
	ROMX_LOAD( "opp2_2h.rf4", 0xc001, 0x2000, CRC(b9e3a5cc) SHA1(0a28afbff612471ee81d69a98789e75253c57a30), ROM_SKIP(1) | ROM_BIOS(5))
	ROMX_LOAD( "opp2_2l.rf4", 0xc000, 0x2000, CRC(6877aad6) SHA1(1d0031d044beb4f9f321e3c8fdedf57467958900), ROM_SKIP(1) | ROM_BIOS(5))
	ROM_SYSTEM_BIOS(5, "v23d", "v2.3d")
	ROMX_LOAD( "opp2_3h.rf4", 0xc001, 0x2000, CRC(ac7d4f06) SHA1(858d6e084a38814280b3e29fb54971f4f532e484), ROM_SKIP(1) | ROM_BIOS(6))
	ROMX_LOAD( "opp2_3l.rf4", 0xc000, 0x2000, CRC(3c877ea1) SHA1(0753168659653538311c0ad1df851cbbdba426f4), ROM_SKIP(1) | ROM_BIOS(6))
ROM_END

ROM_START( mc1702 )
	ROM_REGION16_LE(0x10000,"bios", 0)
	ROM_LOAD16_BYTE( "2764_2_(573rf4).rom", 0xc000,  0x2000, CRC(34a0c8fb) SHA1(88dc247f2e417c2848a2fd3e9b52258ad22a2c07))
	ROM_LOAD16_BYTE( "2764_3_(573rf4).rom", 0xc001, 0x2000, CRC(68ab212b) SHA1(f3313f77392877d28ce290ffa3432f0a32fc4619))
	ROM_LOAD( "ba1m_(573rf5).rom", 0x0000, 0x0800, CRC(08d938e8) SHA1(957b6c691dbef75c1c735e8e4e81669d056971e4))
ROM_END

ROM_START( zdsupers )
	ROM_REGION(0x10000,"bios", 0)
	ROM_SYSTEM_BIOS( 0, "v31d", "v3.1d" )
	ROMX_LOAD( "z184m v3.1d.10d", 0x8000, 0x8000, CRC(44012c3b) SHA1(f2f28979798874386ca8ba3dd3ead24ae7c2aeb4), ROM_BIOS(1))
	ROM_SYSTEM_BIOS( 1, "v29e", "v2.9e" )
	ROMX_LOAD( "z184m v2.9e.10d", 0x8000, 0x8000, CRC(de2f200b) SHA1(ad5ce601669a82351e412fc6c1c70c47779a1e55), ROM_BIOS(2))
ROM_END

ROM_START( sicpc1605 )
	ROM_REGION(0x10000,"bios", 0)
	ROM_LOAD("multitech pc-700 3.1.bin", 0xe000, 0x2000, CRC(0ac7a2e1) SHA1(b9c8504e21213d81a068dde9f51f9c973d726e7b))
ROM_END

ROM_START( ncrpc4i )
	ROM_REGION(0x10000,"bios", 0)
	ROM_LOAD( "ncr_pc4i_biosrom_1985.bin",0xc000, 0x4000, CRC(b9732648) SHA1(0d5d96fbc36089ca4d893b0db84faffa8043a5e4))
ROM_END

ROM_START( olivm15 )
	ROM_REGION(0x10000,"bios", 0)
	ROM_LOAD( "oliv_m15.bin",0xc000, 0x04000, CRC(bf2ef795) SHA1(02d497131f5ca2c78f2accd38ab0eab6813e3ebf))
ROM_END

ROM_START( ibm5550 )
	ROM_REGION16_LE(0x10000,"bios", 0)
	ROM_LOAD( "ipl5550.rom", 0xc000, 0x4000, CRC(40cf34c9) SHA1(d41f77fdfa787b0e97ed311e1c084b8699a5b197))
ROM_END

ROM_START( pc7000 )
	ROM_REGION16_LE(0x10000,"bios", 0)
	ROMX_LOAD( "mitsubishi-m5l27128k-1.bin", 0x8000, 0x4000, CRC(9683957f) SHA1(4569eab6d88eb1bba0d553d1358e593c326978aa), ROM_SKIP(1))
	ROMX_LOAD( "mitsubishi-m5l27128k-2.bin", 0x8001, 0x4000, CRC(99b229a4) SHA1(5800c8bafed26873d8cfcc79a05f93a780a31c91), ROM_SKIP(1))
ROM_END

ROM_START( sx16 )
	ROM_REGION(0x10000,"bios", 0)
	ROM_LOAD( "tmm27128ad.bin",0xc000, 0x4000, CRC(f8543362) SHA1(fef625e260ca89ba02174584bdc12db609f0780e))
ROM_END

ROM_START( mbc16 )
	ROM_REGION(0x10000,"bios", 0)
	ROM_LOAD( "mbc16.bin", 0xc000, 0x4000, CRC(f3e0934a) SHA1(e4b91c3d395be0414e20f23ad4919b8ac52639b2))
	ROM_REGION(0x2000,"gfx1", 0)
	//ATI Graphics Solution SR (graphics card, need to make it ISA card)
	ROM_LOAD( "atigssr.bin", 0x0000, 0x2000, CRC(aca81498) SHA1(0d84c89487ee7a6ac4c9e73fdb30c5fd8aa595f8))
ROM_END

ROM_START ( ataripc1 )
	ROM_REGION(0x10000,"bios", 0)
	ROM_SYSTEM_BIOS( 0, "v3.06", "v3.06" )
	ROMX_LOAD( "award_atari_pc_bios_3.06.bin", 0x8000, 0x8000, CRC(256427ce) SHA1(999f6af64b79f88c1d3492f386d9bee08efb50e7), ROM_BIOS(1))
	ROM_SYSTEM_BIOS( 1, "v3.08", "v3.08" )
	ROMX_LOAD( "award_atari_pc_bios_3.08.bin", 0x8000, 0x8000, CRC(929a2443) SHA1(8e98f3c9180c55b1f5521727779c016083d27960), ROM_BIOS(2)) //same as on Atari PC3, also used on Atari PC2
ROM_END

ROM_START( ataripc3 )
	ROM_REGION(0x10000,"bios", 0)
	ROM_LOAD( "c101701-004 308.u61",0x8000, 0x8000, CRC(929a2443) SHA1(8e98f3c9180c55b1f5521727779c016083d27960))

	ROM_REGION(0x8000,"gfx1", 0)
	ROM_LOAD("5788005.u33", 0x00000, 0x2000, BAD_DUMP CRC(0bf56d70) SHA1(c2a8b10808bf51a3c123ba3eb1e9dd608231916f)) // not the real character ROM

	ROM_REGION(0x8000,"plds", 0)
	ROM_LOAD( "c101681 6ffb.u60",0x000, 0x100, NO_DUMP ) // PAL20L10NC
ROM_END

ROM_START( ssam88s )
	ROM_REGION(0x10000,"bios", 0)
	ROM_LOAD( "samsung_samtron_88s_vers_2.0a.bin",  0x8000, 0x08000, CRC(d1252a91) SHA1(469d15b6ecd7b70234975dc12c6bda4212a66652))
ROM_END

ROM_START( eagle1600 )
	ROM_REGION(0x10000,"bios", 0)
	ROMX_LOAD( "eagle 1600 62-2732-001 rev e u403.bin",0xe000, 0x1000, CRC(3da1e96a) SHA1(77861ba5ebd056da1daf048f5abd459e0528666d), ROM_SKIP(1))
	ROMX_LOAD( "eagle 1600 62-2732-002 rev e u404.bin",0xe001, 0x1000, CRC(be6492d4) SHA1(ef25faf33e8336121d030e38e177be39be8afb7a), ROM_SKIP(1))

	ROM_REGION(0x8000,"gfx1", 0)
	ROM_LOAD("eagle 1600 video char gen u301.bin", 0x00000, 0x1000, CRC(1a7e552f) SHA1(749058783eec9d96a70dc5fdbfccb56196f889dc))
ROM_END

ROM_START( laser_turbo_xt )
	ROM_REGION(0x10000, "bios", 0)
	ROM_LOAD("laser_turbo_xt.bin", 0x0e000, 0x02000, CRC(0a6121d3) SHA1(59b1f8dd6fe981ef9a7700adebf6e1adda7cee90)) // version 1.11 - 27c64d
ROM_END

ROM_START( laser_xt3 )
	ROM_REGION(0x10000, "bios", 0)
	ROM_LOAD("laser_xt3.bin", 0x0e000, 0x02000, CRC(b45a7dd3) SHA1(62f17c408be0036d00a182e94c5c88b83d46b625)) // version 1.26 - 27c64
ROM_END


/***************************************************************************

  Game driver(s)

***************************************************************************/

//    YEAR    NAME              PARENT      COMPAT      MACHINE         INPUT     STATE     INIT      COMPANY                            FULLNAME                FLAGS
COMP( 1984,   dgone,            ibm5150,    0,          dgone,          pccga,    pc_state, 0,        "Data General",                    "Data General/One" ,    MACHINE_NOT_WORKING ) // CGA, 2x 3.5" disk drives
COMP( 1985,   bw230,            ibm5150,    0,          bondwell,       bondwell, pc_state, bondwell, "Bondwell Holding",                "BW230 (PRO28 Series)", 0 )
COMP( 1984,   compc1,           ibm5150,    0,          pccga,          pccga,    pc_state, 0,        "Commodore Business Machines",     "Commodore PC-1" ,      MACHINE_NOT_WORKING )
COMP( 1992,   iskr3104,         ibm5150,    0,          iskr3104,       pccga,    pc_state, 0,        "Schetmash",                       "Iskra 3104",           MACHINE_NOT_WORKING )
COMP( 1989,   mk88,             ibm5150,    0,          mk88,           pccga,    pc_state, 0,        "<unknown>",                       "MK-88",                MACHINE_NOT_WORKING )
COMP( 1991,   poisk2,           ibm5150,    0,          poisk2,         pccga,    pc_state, 0,        "<unknown>",                       "Poisk-2",              MACHINE_NOT_WORKING )
COMP( 1990,   mc1702,           ibm5150,    0,          pccga,          pccga,    pc_state, 0,        "<unknown>",                       "Elektronika MC-1702",  MACHINE_NOT_WORKING )
COMP( 1987,   zdsupers,         ibm5150,    0,          zenith,         pccga,    pc_state, 0,        "Zenith Data Systems",             "SuperSport",           0 )
COMP( 1985,   sicpc1605,        ibm5150,    0,          siemens,        pccga,    pc_state, 0,        "Siemens",                         "Sicomp PC16-05",       MACHINE_NOT_WORKING )
COMP( 1985,   ncrpc4i,          ibm5150,    0,          ncrpc4i,        pccga,    pc_state, 0,        "NCR",                             "PC4i",                 MACHINE_NOT_WORKING )
COMP( 198?,   olivm15,          ibm5150,    0,          m15,            pccga,    pc_state, 0,        "Olivetti",                        "M15",                  0 )
COMP( 1983,   ibm5550,          ibm5150,    0,          ibm5550,        pccga,    pc_state, 0,        "International Business Machines", "IBM 5550",             MACHINE_NOT_WORKING )
COMP( 1985,   pc7000,           ibm5150,    0,          pccga,          pccga,    pc_state, 0,        "Sharp",                           "PC-7000",              MACHINE_NOT_WORKING )
COMP( 1988,   sx16,             ibm5150,    0,          pccga,          pccga,    pc_state, 0,        "Sanyo",                           "SX-16",                MACHINE_NOT_WORKING )
COMP( 198?,   mbc16,            ibm5150,    0,          pccga,          pccga,    pc_state, 0,        "Sanyo",                           "MBC-16",               MACHINE_NOT_WORKING )
COMP( 1987,   ataripc1,         ibm5150,    0,          ataripc1,       pccga,    pc_state, 0,        "Atari",                           "PC1" ,                 0 )
COMP( 1988,   ataripc3,         ibm5150,    0,          pccga,          pccga,    pc_state, 0,        "Atari",                           "PC3" ,                 0 )
COMP( 1989,   ssam88s,          ibm5150,    0,          pccga,          pccga,    pc_state, 0,        "Samsung",                         "Samtron 88S" ,         MACHINE_NOT_WORKING )
COMP( 1983,   eagle1600,        ibm5150,    0,          eagle1600,      pccga,    pc_state, 0,        "Eagle",                           "1600" ,                MACHINE_NOT_WORKING )
COMP( 1988,   laser_turbo_xt,   ibm5150,    0,          laser_turbo_xt, 0,        pc_state, 0,        "VTech",                           "Laser Turbo XT",       0 )
COMP( 1989,   laser_xt3,        ibm5150,    0,          laser_xt3,      0,        pc_state, 0,        "VTech",                           "Laser XT/3",           0 )
