// license:BSD-3-Clause
// copyright-holders:Joakim Larsson Edstrom
/***************************************************************************
 * Interrupt scheme and dmac hookup shamelessly based on esq5505.cpp
 *
 *  11/04/2016
 *  Force SYS68K ISCSI-1 driver - This driver will be converted into a slot device once the VME bus driver exists.
 *  The ISCSI-1 board is a VME slave board that reads command and returns results through dual ported RAM to the VME bus.
 *
 * http://bitsavers.trailing-edge.com/pdf/forceComputers/800114_Force_Introduction_to_the_SYS68K_ISCSI-1_Oct86.pdf
 *
 *       ||
 * ||    ||
 * ||||--||
 * ||||--||
 * ||    ||__________________________________________________________    ___
 *       ||                                                          |_|   |
 *       ||                                                          | |   |
 *       ||                                                          | |   |
 *       ||                                                          | |   |
 *       ||                                                          | |   |
 *       ||                                                          | |   |
 *       ||                                                          | |VME|
 *       ||                                                          | |   |
 *       ||                                                          | |P1 |
 *       ||                                                          | |   |
 *       ||                                                          | |   |
 *       ||                                                          | |   |
 *       ||                                                          | |   |
 *       ||                                                          | |   |
 *       ||                                                          | |   |
 *       ||                                                          |_|   |
 *       ||                                                            |___|
 *       ||                                                            |
 *       ||                                                            |
 *       ||                                                            |
 *       ||                                                            |
 *       ||                                                            |
 *       ||                                                            |
 *       ||                                                            |
 *       ||                                                            |
 *       ||                                                            |___
 *       ||                                                           _|   |
 *       ||                                                          | |   |
 *       ||                                                          | |   |
 *       ||                                                          | |   |
 *       ||                                                          | |   |
 *       ||                                                          | |VME|
 *       ||                                                          | |   |
 *       ||                                                          | |P2 |
 *       ||                                                          | |   |
 *       ||                                                          | |   |
 *       ||                                                          | |   |
 *       ||                                                          | |   |
 *       ||                                                          | |   |
 *       ||                                                          | |   |
 *       ||                                                          | |   |
 *       ||                                                          | |   |
 *       ||                                                          |_|   |
 *       ||                                                            |___|
 * ||    ||------------------------------------------------------------+-+
 * ||||--||
 * ||||--||
 * ||
 *
 * History of Force Computers
 *------------------------------------------------------------------------
 *  See fccpu30.cpp
 *
 * Description from datasheet etc
 * ------------------------------
 * - 68010 CPU for local control (10MHz)
 * - 68450 DMA Controller for local transfers (10MHz)
 * - Dual Ported l28Kbyte 0 wait state static RAM between the VMEbus and the local CPU
 * - SCSlbus interface built with the NCR 5386S SCSlbus controller.
 *   o Programmable as an initiator or target
 *   o Transfer rate up to 1.5Mbyte/s
 * - SHUGART compatible floppy interface with the WD1772 FDC. Up to 4 floppy drives can be
 *   controlled independent of the SCSlbus
 * - All I/O signals available on P2 connector 4 different interrupt request signals to the VMEbus. Each
 *   channel contains a software programmable IRQ level (1 to 7) and vector
 * - Local parallel interface for controlling and monitoring board functions
 * - VMEbus Rev.C/IEEE P10l4 compatible interface A24:D16, D8, SYSFAIL (jumper)
 * - Watchdog timer controlling correct functions of on-board hard and software
 * - Status and control LEDs for monitoring local activities
 * - High level handling firmware for communication, self test, data caching/hashing and control
 *
 * Address Map
 * ----------------------------------------------------------
 * Address Range     Description
 * ----------------------------------------------------------
 * 000000 - 000007 Initialisation vectors from system EPROM
 * 000008 - 001FFF Local SRAM
 * 002000 - 01FFFF Dynamic Dual Port RAM VME bus accessable
 * C40000 - C4001F SCSIbus controller
 * C80000 - C800FF DMAC
 * CC0000 - CC0007 FDC
 * CC0009 - CC0009 Control Register
 * D00000 - D0003F PI/T
 * E00000 - E70000 EPROMs
 * ----------------------------------------------------------
 *
 * VME side A24 address map - Dual ported RAM
 * ----------------------------------------------------------
 * Default Range     Description
 * ----------------------------------------------------------
 * A00000 - A00000  Status word Bits 8:RESET 9:HALT 10:WD
 * A00001 - A0000F  BIM - See below
 * A01001 - A01001  Read generates local interrupt
 * A01801 - A01801  Read generates local reset
 * A02000 - A1FFFF  Dynamic Dual Port RAM
 * ----------------------------------------------------------
 *
 * BIM  Ctrl  Vect Req  Toggle PI/T  Status PI/T
 *  ch   adr    adr      pin write    pin rea
 * ----------------------------------------------------------
 *  0  A00001 A00009     PA0          PA4
 *  1  A00003 A0000B     PA1          PA5
 *  2  A00005 A0000D     PA2          PA6
 *  3  A00007 A0000F     PA3          PA7
 * ----------------------------------------------------------
 *
 * Interrupt sources
 * ----------------------------------------------------------
 * Description                  Device  Lvl  IRQ    VME board
 *                           /Board      Vector  Address
 * ----------------------------------------------------------
 * On board Sources
 * ABORT                        P3 p13  1    AV1
 * DMA controller               68450   2    DMAC/AV2
 * SCSI bus controller       NCR 5386S  3    AV3
 * Floppy Disk controller    WD 1772    4    AV4
 * Parallel interface and timer 68230   5    PIT timer
 *                              ---     6    ---
 * Parallel interface and timer 68230   7    PIT port
 * ----------------------------------------------------------
 *
 *  TODO:
 *  - Write and attach a NCR5386S SCSI device
 *  - Find a floppy image and present it to the WD1772 floppy controller
 *  - Add VME bus driver
 *  - Let a controller CPU board (eg CPU-1 or CPU-30) boot from floppy or SCSI disk
 *
 ****************************************************************************/
#include "emu.h"
#include "vme_fcscsi.h"

#include "cpu/m68000/m68000.h"
#include "machine/68230pit.h"
#include "machine/wd_fdc.h"
#include "machine/hd63450.h" // compatible with MC68450
#include "machine/clock.h"
#include "formats/pc_dsk.h"

#define LOG_GENERAL 0x01
#define LOG_SETUP   0x02
#define LOG_PRINTF  0x04

#define VERBOSE 0 //(LOG_PRINTF | LOG_SETUP  | LOG_GENERAL)

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

#define TODO "Driver for SCSI NCR5386s device needed\n"

//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

DEFINE_DEVICE_TYPE(VME_FCSCSI1, vme_fcscsi1_card_device, "fcscsi1", "Force Computer SYS68K/ISCSI-1 Intelligent Mass Storage Controller Board")

#define CPU_CRYSTAL XTAL_20MHz /* Jauch */
#define PIT_CRYSTAL XTAL_16MHz /* Jauch */

static ADDRESS_MAP_START (fcscsi1_mem, AS_PROGRAM, 16, vme_fcscsi1_card_device)
	ADDRESS_MAP_UNMAP_HIGH
	AM_RANGE (0x000000, 0x000007) AM_ROM AM_READ (bootvect_r)       /* Vectors mapped from System EPROM */
	AM_RANGE (0x000008, 0x001fff) AM_RAM /* SRAM */
	AM_RANGE (0x002000, 0x01ffff) AM_RAM /* Dual Ported RAM */
	AM_RANGE (0xe00000, 0xe7ffff) AM_ROM /* System EPROM Area 32Kb DEBUGGER supplied */
	AM_RANGE (0xd00000, 0xd0003f) AM_DEVREADWRITE8 ("pit", pit68230_device, read, write, 0x00ff)
//  AM_RANGE (0xc40000, 0xc4001f) AM_DEVREADWRITE8("scsi", ncr5386_device, read, write, 0x00ff) /* SCSI Controller interface - device support not yet available*/
	AM_RANGE (0xc40000, 0xc4001f) AM_READWRITE8 (scsi_r, scsi_w, 0x00ff)
	AM_RANGE (0xc80000, 0xc800ff) AM_DEVREADWRITE("mc68450", hd63450_device, read, write)  /* DMA Controller interface */
	AM_RANGE (0xcc0000, 0xcc0007) AM_DEVREADWRITE8("fdc", wd1772_device, read, write, 0x00ff)      /* Floppy Controller interface */
	AM_RANGE (0xcc0008, 0xcc0009) AM_READWRITE8 (tcr_r, tcr_w, 0x00ff) /* The Control Register, SCSI ID and FD drive select bits */
ADDRESS_MAP_END


FLOPPY_FORMATS_MEMBER( vme_fcscsi1_card_device::floppy_formats )
	FLOPPY_PC_FORMAT
FLOPPY_FORMATS_END

static SLOT_INTERFACE_START( fcscsi_floppies )
	SLOT_INTERFACE( "525qd", FLOPPY_525_QD )
SLOT_INTERFACE_END


/* ROM definitions */
ROM_START (fcscsi1)
	ROM_REGION (0x1000000, "maincpu", 0)

	/* Besta ROM:s - apparantly patched Force ROM:s */
	ROM_SYSTEM_BIOS(0, "Besta 88", "Besta 88")
	ROMX_LOAD ("besta88_scsi_lower.ROM", 0xe00001, 0x4000, CRC (fb3ab364) SHA1 (d79112100f1c4beaf358e006efd4dde5e300b0ba), ROM_SKIP(1) | ROM_BIOS(1))
	ROMX_LOAD ("besta88_scsi_upper.ROM", 0xe00000, 0x4000, CRC (41f9cdf4) SHA1 (66b998bbf9459f0a613718260e05e97749532073), ROM_SKIP(1) | ROM_BIOS(1))

	/* Force ROM:s  */
	ROM_SYSTEM_BIOS(1, "ISCSI-1 v3.7", "Force Computer SYS68K/ISCSI-1 firmware v3.7")
	ROMX_LOAD ("ISCSI-1_V3.7_L.BIN", 0xe00001, 0x4000, CRC (83d95ab7) SHA1 (bf249910bcb6cb0b04dda2a95a38a0f90b553352), ROM_SKIP(1) | ROM_BIOS(2))
	ROMX_LOAD ("ISCSI-1_V3.7_U.BIN", 0xe00000, 0x4000, CRC (58815831) SHA1 (074085ef96e1fe2a551938bdeee6a9cab40ff09c), ROM_SKIP(1) | ROM_BIOS(2))

ROM_END


MACHINE_CONFIG_MEMBER(vme_fcscsi1_card_device::device_add_mconfig)
	/* basic machine hardware */
	MCFG_CPU_ADD ("maincpu", M68010, CPU_CRYSTAL / 2) /* 7474 based frequency divide by 2 */
	MCFG_CPU_PROGRAM_MAP (fcscsi1_mem)
	MCFG_CPU_IRQ_ACKNOWLEDGE_DRIVER(vme_fcscsi1_card_device, maincpu_irq_acknowledge_callback)

	/* FDC  */
	MCFG_WD1772_ADD("fdc", PIT_CRYSTAL / 2)
	MCFG_WD_FDC_INTRQ_CALLBACK(WRITE8(vme_fcscsi1_card_device, fdc_irq))
	MCFG_WD_FDC_DRQ_CALLBACK(DEVWRITELINE("mc68450", hd63450_device, drq1_w))
	MCFG_FLOPPY_DRIVE_ADD("fdc:0", fcscsi_floppies, "525qd", vme_fcscsi1_card_device::floppy_formats)
	MCFG_FLOPPY_DRIVE_ADD("fdc:1", fcscsi_floppies, "525qd", vme_fcscsi1_card_device::floppy_formats)
	MCFG_FLOPPY_DRIVE_ADD("fdc:2", fcscsi_floppies, "525qd", vme_fcscsi1_card_device::floppy_formats)
	MCFG_FLOPPY_DRIVE_ADD("fdc:3", fcscsi_floppies, "525qd", vme_fcscsi1_card_device::floppy_formats)

	/* PIT Parallel Interface and Timer device */
	MCFG_DEVICE_ADD ("pit", PIT68230, PIT_CRYSTAL / 2) /* 7474 based frequency divide by 2 */
	MCFG_PIT68230_PB_OUTPUT_CB(WRITE8(vme_fcscsi1_card_device, led_w))

	/* DMAC it is really a M68450 but the HD63850 is upwards compatible */
	MCFG_DEVICE_ADD("mc68450", HD63450, 0)   // MC68450 compatible
	MCFG_HD63450_CPU("maincpu") // CPU - 68010
	MCFG_HD63450_CLOCKS(attotime::from_usec(32), attotime::from_nsec(450), attotime::from_usec(4), attotime::from_hz(15625/2))
	MCFG_HD63450_BURST_CLOCKS(attotime::from_usec(32), attotime::from_nsec(450), attotime::from_nsec(50), attotime::from_nsec(50))
	MCFG_HD63450_DMA_END_CB(WRITE8(vme_fcscsi1_card_device, dma_end))
	MCFG_HD63450_DMA_ERROR_CB(WRITE8(vme_fcscsi1_card_device, dma_error))
	//MCFG_HD63450_DMA_READ_0_CB(READ8(vme_fcscsi1_card_device, scsi_read_byte))  // ch 0 = SCSI
	//MCFG_HD63450_DMA_WRITE_0_CB(WRITE8(vme_fcscsi1_card_device, scsi_write_byte))
	MCFG_HD63450_DMA_READ_1_CB(READ8(vme_fcscsi1_card_device, fdc_read_byte))  // ch 1 = fdc
	MCFG_HD63450_DMA_WRITE_1_CB(WRITE8(vme_fcscsi1_card_device, fdc_write_byte))
MACHINE_CONFIG_END

const tiny_rom_entry *vme_fcscsi1_card_device::device_rom_region() const
{
	LOG("%s\n", FUNCNAME);
	return ROM_NAME( fcscsi1 );
}

//**************************************************************************
//  LIVE DEVICE
//**************************************************************************
vme_fcscsi1_card_device::vme_fcscsi1_card_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, type, tag, owner, clock)
	, device_vme_card_interface(mconfig, *this)
	, m_maincpu(*this, "maincpu")
	, m_fdc(*this, "fdc")
	, m_pit(*this, "pit")
	, m_dmac(*this, "mc68450")
	, m_tcr(0)
{
	LOG("%s\n", FUNCNAME);
}

vme_fcscsi1_card_device::vme_fcscsi1_card_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: vme_fcscsi1_card_device(mconfig, VME_FCSCSI1, tag, owner, clock)
{
}

/* Start it up */
void vme_fcscsi1_card_device::device_start()
{
	LOG("%s\n", FUNCNAME);
	set_vme_device();

	/* Setup pointer to bootvector in ROM for bootvector handler bootvect_r */
	m_sysrom = (uint16_t*)(memregion ("maincpu")->base () + 0xe00000);

#if 0 // TODO: Setup VME access handlers for shared memory area
	uint32_t base = 0x00A00000;
	m_vme->install_device(base + 0, base + 1, // Channel B - Data
							 read8_delegate(FUNC(z80sio_device::db_r),  subdevice<z80sio_device>("pit")), write8_delegate(FUNC(z80sio_device::db_w), subdevice<z80sio_device>("pit")), 0x00ff);
	m_vme->install_device(base + 2, base + 3, // Channel B - Control
							 read8_delegate(FUNC(z80sio_device::cb_r),  subdevice<z80sio_device>("pit")), write8_delegate(FUNC(z80sio_device::cb_w), subdevice<z80sio_device>("pit")), 0x00ff);
#endif

}

void vme_fcscsi1_card_device::device_reset()
{
	LOG("%s\n", FUNCNAME);
}

/* Boot vector handler, the PCB hardwires the first 8 bytes from 0x80000 to 0x0 */
READ16_MEMBER (vme_fcscsi1_card_device::bootvect_r){
	return m_sysrom [offset];
}

/* The Control Register - discretely implemented on the PCB
Bit #: 7 6 5 4 3 2 1 0
       \ \ \ \ \ \ \ \ Floppy Disk Side Select
        \ \ \ \ \ \ \ Floppy Disk Drive Select 0
         \ \ \ \ \ \ Floppy Disk Drive Select 1
          \ \ \ \ \ Floppy Disk Drive Select 2
           \ \ \ \ Floppy Disk Drive Select 3
            \ \ \ ISCSI-l I.D. Bit #0
             \ \ ISCSI-l I.D. Bit #1
              \ ISCSI-l 1.D. Bit #2
*/

READ8_MEMBER (vme_fcscsi1_card_device::tcr_r){
	LOG("%s\n", FUNCNAME);
	return (uint8_t) m_tcr;
}

WRITE8_MEMBER (vme_fcscsi1_card_device::tcr_w){
	floppy_image_device *floppy0 = m_fdc->subdevice<floppy_connector>("0")->get_device();
	floppy_image_device *floppy1 = m_fdc->subdevice<floppy_connector>("1")->get_device();
	floppy_image_device *floppy2 = m_fdc->subdevice<floppy_connector>("2")->get_device();
	floppy_image_device *floppy3 = m_fdc->subdevice<floppy_connector>("3")->get_device();
	floppy_image_device *floppy = nullptr;

	if (!BIT(data, 1)) floppy = floppy0;
	else
	if (!BIT(data, 2)) floppy = floppy1;
	else
	if (!BIT(data, 3)) floppy = floppy2;
	else
	if (!BIT(data, 4)) floppy = floppy3;

	if (floppy) {
		m_fdc->set_floppy(floppy);
		floppy->ss_w(!BIT(data, 0));
		floppy->mon_w(0);
	} else {
		floppy0->mon_w(1);
		floppy1->mon_w(1);
		floppy2->mon_w(1);
		floppy3->mon_w(1);
	}

	LOG("%s [%02x]\n", FUNCNAME, data);
	m_tcr = data;
	return;
}

WRITE8_MEMBER (vme_fcscsi1_card_device::led_w){
	LOG("%s [%02x]\n", FUNCNAME, data);

	m_fdc->dden_w(BIT(data, 7));

	return;
}

WRITE8_MEMBER(vme_fcscsi1_card_device::dma_end)
{
	if (data != 0)
	{
		dmac_irq_state = 1;
		dmac_irq_vector = m_dmac->get_vector(offset);
	}
	else
	{
		dmac_irq_state = 0;
	}

	update_irq_to_maincpu();
}

WRITE8_MEMBER(vme_fcscsi1_card_device::dma_error)
{
	if(data != 0)
	{
		logerror("DMAC error, vector = %x\n", m_dmac->get_error_vector(offset));
		dmac_irq_state = 1;
		dmac_irq_vector = m_dmac->get_vector(offset);
	}
	else
	{
		dmac_irq_state = 0;
	}

	update_irq_to_maincpu();
}

WRITE8_MEMBER(vme_fcscsi1_card_device::fdc_irq)
{
	if (data != 0)
	{
		fdc_irq_state = 1;
	}
	else
	{
		fdc_irq_state = 0;
	}
	update_irq_to_maincpu();
}

READ8_MEMBER(vme_fcscsi1_card_device::fdc_read_byte)
{
	return m_fdc->data_r();
}

WRITE8_MEMBER(vme_fcscsi1_card_device::fdc_write_byte)
{
	m_fdc->data_w(data & 0xff);
}

READ8_MEMBER(vme_fcscsi1_card_device::scsi_r)
{
	uint8_t data = 0;

	// fake diag status
	if (offset == 9)
		data = 0x80;

	LOG("scsi R %02x == %02x\n", offset, data);

	return data;
}

WRITE8_MEMBER(vme_fcscsi1_card_device::scsi_w)
{
	LOG("scsi W %02x <- %02x\n", offset, data);
}

READ8_MEMBER (vme_fcscsi1_card_device::not_implemented_r){
	static int been_here = 0;
	if (!been_here++){
		logerror(TODO);
		printf(TODO);
	}
	return (uint8_t) 0;
}

WRITE8_MEMBER (vme_fcscsi1_card_device::not_implemented_w){
	static int been_here = 0;
	if (!been_here++){
		logerror(TODO);
		printf(TODO);
	}
	return;
}

/*
----------------------------------------------------
 IRQ  IRQ
Level Source       B4l inserted     B4l removed (Def)
-----------------------------------------------------
 1     P3 Pin #13   AV1 Autovector   AV1 Autovector
 2     DMAC         DMAC             AV2 Autovector
 3     SCSIBC       AV3 Autovector   AV3 Autovector
 4     FDC          AV4 Autovector   AV4 Autovector
 5     PI/T Timer   PI/T Timer Vect  PI/T Timer Vect
 6     --           --               --
 7     PI/T Port    PI/T Port Vect   PI/T Port Vect
------------------------------------------------------
Default configuration: B41 jumper removed

The PI/T port interrupt can be used under software control to
cause non-maskable (Level 7) interrupts if the watchdog timer
elapses and/or if the VMEbus interrupt trigger call occurs.
*/

/* TODO: Add configurable B41 jumper */
#define B41 0

void vme_fcscsi1_card_device::update_irq_to_maincpu() {
	if (fdc_irq_state) {
		m_maincpu->set_input_line(M68K_IRQ_3, ASSERT_LINE);
		m_maincpu->set_input_line(M68K_IRQ_2, CLEAR_LINE);
		m_maincpu->set_input_line(M68K_IRQ_1, CLEAR_LINE);
	} else if (dmac_irq_state) {
		m_maincpu->set_input_line(M68K_IRQ_3, CLEAR_LINE);
		m_maincpu->set_input_line(M68K_IRQ_1, CLEAR_LINE);
#if B41 == 1
		m_maincpu->set_input_line_and_vector(M68K_IRQ_2, ASSERT_LINE, dmac_irq_vector);
#else
		m_maincpu->set_input_line(M68K_IRQ_2, ASSERT_LINE);
#endif
	} else {
		m_maincpu->set_input_line(M68K_IRQ_3, CLEAR_LINE);
		m_maincpu->set_input_line(M68K_IRQ_2, CLEAR_LINE);
		m_maincpu->set_input_line(M68K_IRQ_1, CLEAR_LINE);
	}
}

IRQ_CALLBACK_MEMBER(vme_fcscsi1_card_device::maincpu_irq_acknowledge_callback)
{
	// We immediately update the interrupt presented to the CPU, so that it doesn't
	// end up retrying the same interrupt over and over. We then return the appropriate vector.
	int vector = 0;
	switch(irqline) {
	case 2:
		dmac_irq_state = 0;
		vector = dmac_irq_vector;
		break;
	default:
		logerror("\nUnexpected IRQ ACK Callback: IRQ %d\n", irqline);
		return 0;
	}
	update_irq_to_maincpu();
	return vector;
}

// This info isn't kept in a card driver atm so storing it as a comment for later use
//      YEAR  NAME           PARENT  COMPAT  MACHINE       INPUT    CLASS             INIT COMPANY                  FULLNAME           FLAGS
//COMP( 1986, fcscsi1,       0,      0,      fcscsi1,      fcscsi1, driver_device,     0,  "Force Computers Gmbh",  "SYS68K/SCSI-1",   MACHINE_IS_SKELETON )
