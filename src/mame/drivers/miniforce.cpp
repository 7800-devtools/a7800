// license:BSD-3-Clause
// copyright-holders:Joakim Larsson Edstrom
/***************************************************************************
 *
 *  Force miniForce 2P/32 driver
 *
 *  24/12/2016
 *
 * Thanks to Al Kossow and his site http://www.bitsavers.org/ I got the information
 * required to start the work with this driver.
 *
 * +=============================================================================================================================+
 * |CPU  |SRAM |     |     |     |     |     |     |WFC-1|     |  SYS68K/PWR-09A | SYS68K/WFMOD-50                               |
 * |-21  | -22 |     |     |     |     |     |     |     |     |                 |                                               |
 * | RST |     |     |     |     |     |     |     |O RUN|     |                 |                                               |
 * | ABT |     |     |     |     |     |     |     | R/L |     |O +5v            |+---------------------------------------------+|
 * |     |     |     |     |     |     |     |     |O LOC|     |O +12v           ||                                             ||
 * |O RUN|O RUN|     |     |     |     |     |     |O ERR|     |O -12v           ||                                             ||
 * |O HLT|     |     |     |     |     |     |     |O BSY|     |O ON             ||                                             ||
 * |O BM |     |     |     |     |     |     |     |     |     |                 ||                                             ||
 * |     |     |     |     |     |     |     |     |     |     |                 |+---------------------------------------------+|
 * |O FLM|O SL0|     |     |     |     |     |     |     |     |                 || FDD                                         ||
 * |O EPR|O SL1|     |     |     |     |     |     |     |     |                 ||                                             ||
 * |O 2WS|     |     |     |     |     |     |     |     |     |    +-------+    ||                                             ||
 * |O 4WS|     |     |     |     |     |     |     |     |     |    |   o   |PWR ||                                             ||
 * |O 6WS|     |     |     |     |     |     |     |     |     |    |       |    |+---------------------------------------------+|
 * |O 8WS|     |     |     |     |     |     |     |     |     |    +-------+    |                                               |
 * |O12WS|     |     |     |     |     |     |     |     |     |                 |                                               |
 * |O14WS|     |     |     |     |     |     |     |     |     |                 |+---------------------------------------------+|
 * |     |     |     |     |     |     |     |     |     |     |                 || HDD                                         ||
 * | CSH |     |     |     |     |     |     |     |     |     |                 ||                                             ||
 * | R/M |     |     |     |     |     |     |     |     |     |                 ||                                             ||
 * |     |     |     |     |     |     |     |     |     |     |                 ||                                             ||
 * |  o  |     |     |     |     |     |     |     |     |     |                 |+---------------------------------------------+|
 * |  o  |     |     |     |     |     |     |     |     |     |                 ||                                             ||
 * |  o  |     |     |     |     |     |     |     |     |     |                 ||                                             ||
 * |  o  |     |     |     |     |     |     |     |     |     |                 ||                                             ||
 * | RS232/422 |     |     |     |     |     |     |     |     |                 ||                                             ||
 * | P4  | P3  |     |     |     |     |     |     |     |     |                 |+---------------------------------------------+|
 * |     |     |     |     |     |     |     |     |     |     |                 |                                               |
 * |SLOT1|SLOT2|SLOT3|SLOT4|SLOT5|SLOT6|SLOT7|SLOT7|SLOT9|     |                 |                                               |
 * +=============================================================================================================================+
 *
 * History of Force Computers
 *---------------------------
 * See fccpu30.cpp
 *
 * Misc links about Force Computes and this board:
 *------------------------------------------------
 * http://bitsavers.org/pdf/forceComputers/
 *
 * Description, from datasheets etc
 * --------------------------------
 * - Desktop station for 32 bit VMEbus environments
 * - Two 9 slot motherboards for A32/D32 wide VMEbus (Pl,P2)
 * - 280W power supply to drive VMEbus and mass storage memory
 * - 7HE 19 inch metal chassis including modules for drives,
 *   power supply and connectors (344mm x 520mm x 400mm).
 * - High modularity(!)
 * - One 1HE fan module including 3 fans for optimal cooling
 * - Flexible mounting and very little time expenditure for
 *   repairs through modularity
 * - Status indicators and switches of the VME boards are
 *   directly accessible on the front of the system
 * - One 5 1/4" full height space for the floppy drive
 * - One 5 1/4" full height space for the winchester drive
 * - Up to 6 free slots for system expansion
 *
 * Features per version
 * --------------------------------------------------------------------------
 *  Description             miniFORCE 2P21A  miniFORCE 2P21   miniFORCE 2P21S
 * --------------------------------------------------------------------------
 *  CPU 68020                20 MHz           16.7 MHz         12.5 MHz
 *  FPU 68881                20 MHz           16.7 MHz         12.5 MHz
 *  Memory SRAM              512KB            512KB            512KB
 *  Serial 68561 MPSC        2 RS232 ports    2 RS232 ports    2 RS232 ports
 *  Winchester HDD           51 MB            51 MB            20 MB
 *  Floppy                   1 MB             1 MB             1 MB
 *  Timer 68230 PIT          1                1                1
 *  RTOS                     PDOS             PDOS             PDOS
 * --------------------------------------------------------------------------
 *
 * Address Map from CPU-21 board perspective
 * --------------------------------------------------------------------------
 *  Range                   Decscription
 * --------------------------------------------------------------------------
 * 00000000-0007FFFF        Local 512KB SRAM CPU-21 CPU board
 * 00080000-000FFFFF        VME A32 512KB SRAM CPU-22 SRAM board (optional)
 * 00080000-FAFFFFFF        VME A32 Memory if no CPU-22 installed
 * 00100000-FAFFFFFF        VME A32 Memory if CPU-22 installed
 * FCB00000-FCB001FF        VME A24 First SIO-1 card (optional)
 * FCB01000-FCB0100F        VME A24 WFC-l card
 * FCB02000-FCB022FF        VME A24 ASCU-l/2 card (optional)
 * FF000000-FF07FFFF        EPROM Area 1
 * FF080000-FFFFFFFF        Local I/O devices
 * --------------------------------------------------------------------------
 */
#include "emu.h"
#include "bus/vme/vme.h"
#include "bus/vme/vme_fccpu20.h"
#include "bus/vme/vme_fcisio.h"
#include "bus/vme/vme_fcscsi.h"
#include "bus/vme/vme_mzr8300.h"
#include "machine/clock.h"

#define LOG_GENERAL 0x01
#define LOG_SETUP   0x02
#define LOG_PRINTF  0x04

#define VERBOSE 0 // (LOG_PRINTF | LOG_SETUP  | LOG_GENERAL)

#define LOGMASK(mask, ...)   do { if (VERBOSE & mask) logerror(__VA_ARGS__); } while (0)
#define LOGLEVEL(mask, level, ...) do { if ((VERBOSE & mask) >= level) logerror(__VA_ARGS__); } while (0)

#define LOG(...)      LOGMASK(LOG_GENERAL, __VA_ARGS__)
#define LOGSETUP(...) LOGMASK(LOG_SETUP,   __VA_ARGS__)

#if VERBOSE & LOG_PRINTF
#define logerror printf
#endif

#ifdef _MSC_VER
#define FUNCNAME __func__
#else
#define FUNCNAME __PRETTY_FUNCTION__
#endif

class miniforce_state : public driver_device
{
public:
miniforce_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device (mconfig, type, tag)
	{
	}
	virtual void machine_start () override;
	virtual void machine_reset () override;
};

#if 0
static ADDRESS_MAP_START (miniforce_mem, AS_PROGRAM, 32, miniforce_state)
	ADDRESS_MAP_UNMAP_HIGH
/* The ROMs contains an OS9 bootloader. It is position independent but reset vector suggests that it sits flat on adress 0 (zero) */
//  AM_RANGE (0x000000, 0x003fff) AM_ROM AM_REGION("roms", 0x000000) /* System EPROM Area 16Kb OS9 DEBUG - not verified     */
//  AM_RANGE (0x004000, 0x01ffff) AM_ROM AM_REGION("roms", 0x004000)/* System EPROM Area 112Kb for System ROM - not verified    */
//  AM_RANGE (0x020000, 0x03ffff) AM_RAM /* Not verified */
//  AM_RANGE (0x100000, 0xfeffff)  AM_READWRITE(vme_a24_r, vme_a24_w) /* VMEbus Rev B addresses (24 bits) - not verified */
//  AM_RANGE (0xff0000, 0xffffff)  AM_READWRITE(vme_a16_r, vme_a16_w) /* VMEbus Rev B addresses (16 bits) - not verified */
ADDRESS_MAP_END
#endif

/* Start it up */
void miniforce_state::machine_start()
{
	LOG("%s\n", FUNCNAME);
}

/* Start it up */
void miniforce_state::machine_reset()
{
	LOG("%s\n", FUNCNAME);
}

/* Input ports */
static INPUT_PORTS_START (miniforce)
INPUT_PORTS_END

static SLOT_INTERFACE_START(miniforce_vme_cards)
	SLOT_INTERFACE("fccpu21", VME_FCCPU21)
	SLOT_INTERFACE("fcisio", VME_FCISIO1)
	SLOT_INTERFACE("fcscsi", VME_FCSCSI1)
SLOT_INTERFACE_END

/*
 * Machine configuration
 */
MACHINE_CONFIG_START (miniforce)
//  MCFG_CPU_PROGRAM_MAP (miniforce_mem)
	MCFG_VME_DEVICE_ADD("vme")
	MCFG_VME_SLOT_ADD ("vme", 1, miniforce_vme_cards, "fccpu21")
	MCFG_VME_SLOT_ADD ("vme", 2, miniforce_vme_cards, nullptr)
	MCFG_VME_SLOT_ADD ("vme", 3, miniforce_vme_cards, nullptr)
	MCFG_VME_SLOT_ADD ("vme", 4, miniforce_vme_cards, nullptr)
	MCFG_VME_SLOT_ADD ("vme", 5, miniforce_vme_cards, nullptr)
	MCFG_VME_SLOT_ADD ("vme", 6, miniforce_vme_cards, nullptr)
	MCFG_VME_SLOT_ADD ("vme", 7, miniforce_vme_cards, nullptr)
	MCFG_VME_SLOT_ADD ("vme", 8, miniforce_vme_cards, nullptr)
	MCFG_VME_SLOT_ADD ("vme", 9, miniforce_vme_cards, nullptr)
MACHINE_CONFIG_END

ROM_START(miniforce)
ROM_END

/* Drivers TODO: setup distinct miniforce machine configurations */
/*    YEAR  NAME        PARENT  COMPAT  MACHINE     INPUT       CLASS             INIT  COMPANY             FULLNAME      FLAGS */
COMP (1987, miniforce,  0,      0,      miniforce,  miniforce,  miniforce_state,  0,    "Force Computers",  "miniFORCE",  MACHINE_NOT_WORKING | MACHINE_NO_SOUND_HW | MACHINE_TYPE_COMPUTER)
