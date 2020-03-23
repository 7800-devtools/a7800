// license:BSD-3-Clause
// copyright-holders:Sergey Svishchev
/******************************************************************************

Integral Personal Computer (HP9807A)
Hewlett-Packard, 1985

Driver to-do list
=================

- softlist: merge dumps from coho.org and classiccmp.org
- keyboard: NMI generation, autorepeat
- HP-HIL mouse
- RTC chip: proper month, day
- switchable graphics resolution ("_desktop" mode uses 640x400)
- HP-IB chip
- CS/80, SS/80 storage protocol(s) and drives
- HP-IL printer
- sound (needs dump of COP452)

QA
- diagnstc.td0: display test [cannot execute]
- diagnstc.td0: complete keyboard test [keyboard stops responding]
- diagnstc.td0: speaker test
- diagnstc.td0: printer test
+ diagnstc.td0: auto: floppy disc test
+ diagnstc.td0: auto: ram test
- diagnstc.td0: auto: rtc test [cannot execute]
- diagnstc.td0: auto: short keyboard test [cannot execute + keyboard stops responding]

maybe
- drive AP line of MLC from a timer
- RTC standby interrupt?
- what does _desktop do except setting 640x400 mode?
- non-HLE keyboard and mouse? (need dumps of COP4xx)

slot devices
- 82915A -- 300/1200 bps modem; http://www.hpmuseum.net/display_item.php?hw=920
- 82919A -- serial; http://www.hpmuseum.net/display_item.php?hw=445
- 82920A -- current loop; http://www.hpmuseum.net/display_item.php?hw=975
- 82922A -- BCD interface; http://www.hpmuseum.net/display_item.php?hw=921
- 82923A -- GPIO; http://www.hpmuseum.net/display_item.php?hw=976
- 82924A -- HP-IL; http://www.hpmuseum.net/display_item.php?hw=922
- 82968A -- up to 256 KB of ROM on top of operating system PCA
- 82971A -- up to 1 MB of EPROM or 2 MB or masked ROM
- 82998A -- HP-IB; http://www.hpmuseum.net/display_item.php?hw=933
- 98071A -- 640x400 composite Video + Serial; http://www.hpmuseum.net/display_item.php?hw=935


This is a portable mains-powered UNIX workstation computer system produced by Hewlett-Packard and launched in 1985
Basic hardware specs are....
- 68000 CPU at 7.96MHz
- 9" amber electro-luminescent display with a resolution of 255*512 pixels or up to 85 characters x 31 lines (default=80*24) with
  dedicated 32Kb display memory
- Internal 3.5" floppy disk drive with the following specification....
  Encoding: Double Density HP MFM Format
  Rotational speed: 600 RPM
  Transfer speed: 62.5kB/second
  Capacity: 709Kb (709632 bytes)
  Bytes per sector: 512
  Sectors per track: 9
  Total tracks per surface: 80, Available: 77, Spare: 2, Wear: 1
  Surfaces: 2
  Interleave: 1
- HP ThinkJet ink-jet printer integrated into the top of the case. This is a modified HP2225B Thinkjet printer
- 90-key detachable keyboard
- ROM: up to 512Kb standard and an additional 512Kb of option ROM
- RAM: 512Kb, arranged as 256Kbx16. RAM is expandable externally to 7Mb
- Real Time Clock
- Speaker
- External I/O bus with two I/O ports for interfaces and memory modules. Expandable to maximum 10 ports with two 5-port Bus Expander Modules
- HP-IB (IEEE-488) bus
- Runs the HP-UX Operating System III or System V (in ROM)


PCB Layouts
===========

CPU/Memory Board (LOGIC A PCA)
----------------

HP Part# 00095-60953
                     |---------------------------------------------------------------------------------|
                     |  15.92MHz     J1        J2                                                      |
                     |LS74                                                                             |
                     |     |---|                                                                       |
                     |     |T  |                                                                       |
                     |     |M  |                                                                       |
                     |     |S  |                                                                       |
                     |     |4  |         |-------------------------|                                   |
|--------------------|     |5  |         |          68000          |                                   |
|                          |0  |         |                         |                                   |
| MB81256   MB81256        |0  |         |-------------------------|                                   |
|                          |A  |                                                                       |
| MB81256   MB81256        |---|                                                                       |
|                      |----------------J3-----------------|                                           |
| MB81256   MB81256    |                                   |                                           |
|                      |                                   |                                           |
| MB81256   MB81256    |                                   |                                           |
|                      |                                   |                                           |
| MB81256   MB81256    |                                   |                                           |
|                      |                                   |                                           |
| MB81256   MB81256    |                                   |                                           |
|                      |----------------J4-----------------|                                           |
| MB81256   MB81256                                                                                    |
|                                                                                U58            U60    |
| MB81256   MB81256                                                                      555           |
|                                                                                                      |
|                                                                                                      |
|                                                                                                      |
|      J5                                                           J6                         J7      |
|------------------------------------------------------------------------------------------------------|
Notes:
        68000 - 68000 CPU at U7. Clock input 7.96MHz [15.92/2] (DIP64)
         LS74 - 74LS74 at U2 provides system clock dividers /4 /2
      MB81256 - Fujitsu MB81256 256Kx1 DRAM. Total RAM 512Kx8/256Kx16. HP part# 1818-3308. U20-U35 (DIP16)
      TMS4500 - Texas Instruments TMS4500A DRAM Controller at U4. Clock input 3.98MHz [15.92/4] (DIP40)
          U58 - 1RD2-6001, HP-HIL Master Link Controller 'Cerberus'. Clock input on pin 24 is unknown (DIP24)
          U60 - Unknown IC used as an interrupt encoder. Possibly a logic chip? (DIP20)
          555 - 555 Timer
        J1/J2 - Connectors joining to LOGIC B PCA (logic A to logic B bus)
        J3/J4 - Connectors for ROM board
           J5 - Power input connector from power supply PCB
           J6 - 64-pin external I/O bus. This is connected to the I/O backplane PCB mounted at the rear of the case using a ribbon cable
           J7 - 10 pin connector. Labelled on schematics as LOOP0 In/Out and LOOP1 In/Out. Connected to 'Cerberus' IC. Possibly for more input devices?


Interface & Control Board (LOGIC B PCA)
-------------------------

HP Part# 00095-60952
00095-60107 BD REV A
00095-60149 ASSY LOGIC B
|---------------------------------------------------------------------------------|
|SPEAKER                        NS58167A               J11       J10              |
|            LM358   32.768kHz                                                    |
|                                                                                 |
|                                                      16Kx4  16Kx4  16Kx4  16Kx4 |
|                 ROM                                                             |
|                                                                                 |
|   BT1                       HP-IL(2)       GPU                                  |
|                                                                                 |--------------------|
|                 1Kb                                                                                  |
|  COP452                                                                                              |
|                                                                                                      |
|                                                                                                      |
|                                                                              WD2797                  |
| LM393          HP-IL(1)                                                                              |
|                                                                                                      |
|                                           LS193                                                      |
|                                                                                                      |
|                                                                                                      |
|                                                                                                      |
|                                                  LS74    24MHz      LS161                            |
|                                                                                                    J2|
|                                                                                                      |
|                                                            TMS9914                                   |
|                                                                                                      |
|                                                                                                      |
|                                                              75162   75160                           |
|                                                                                                      |
|  J14  J6     J5     J4     J3                  J7             J1         J8        J9        J12     |
|------------------------------------------------------------------------------------------------------|
Notes:
         GPU - 1LL3-0005 GPU (Graphics Processor) at U1. Clock input 3MHz [24/8]. VSync on pin 45, HSync on pin 46 (DIP48)
       16Kx4 - 16Kx4 DRAM, organised as 32Kx8bit/16Kx16bit at U3, U4, U5 & U6. Chip type unknown, likely Fujitsu MB81416 (DIP18)
      WD2797 - Western Digital WD2797 Floppy Disk Controller at U18. Clock input 2MHz [24/2/3/2] (DIP40)
    HP-IL(1) - HP-IL 1LJ7-0015 'Saturn' Thinkjet Printer Controller IC at U28. Clock input 3MHz on pin 9 [24/8] (DIP48)
    HP-IL(2) - HP-IL 1LB3 Interface IC at U31. Clock input 2MHz on pin 22 [24/2/3/2] (DIP28)
         1Kb - 1Kb RAM at U27 for printer buffer (DIP28, type unknown, very old, with only DATA0,1,2,3, C/D and DIN,DOUT)
         ROM - 16Kb (32Kx4) Some kind of very early DIP28 PROM/ROM? Same pinout as 1Kb RAM above. Holds the character font table for the printer
               Four versions of this ROM exist, one each for Japan/Arabic/Hebrew and one for all other regions
    NS58167A - National Semiconductor NS58167A Clock Controller RTC at U44. Clock input 32.768kHz (DIP24)
       LM358 - National Semiconductor LM358 Operational Amplifier at U40 (DIP8)
       LM393 - Texas Instruments LM393 Dual Comparator at U34 (DIP8)
         BT1 - 3v lithium battery
      COP452 - National Semiconductor COP452 Speaker Controller at U39. Clock input 2MHz [24/2/3/2]. This IC provides programmable square wave output from 100Hz to 5000Hz (DIP14)
     TMS9914 - Texas Instruments TMS9914 General Purpose Interface Bus Adapter at U41. Clock input 4MHz [24/2/3] (DIP40)
       75160 - National Semiconductor DS75160A IEEE-488 General Purpose Interface Bus Transceiver at U42 (DIP20)
       75162 - National Semiconductor DS75162A IEEE-488 General Purpose Interface Bus Transceiver at U43 (DIP22)
       LS193 - 74LS193 at U9 provides the main system clock dividers /8 /4 /2 (24MHz -> 12MHz, 6MHz, 3MHz). This is also the source of the 6MHz video clock on connector J1
       LS161 - 74LS161 at U46. Takes 12MHz clock input on pin 2 and outputs 4MHz on pin 13 (i.e. /3). This is the clock source for the TMS9914
        LS74 - LS74 at U16. Takes output of 4MHz from LS161 at U46 and outputs 2MHz on pin 5 (i.e. /2). This is the source of the 2MHz clocks
          J1 - Connector joining to video display assembly
          J2 - Connector joining to floppy drive
          J3 - Connector joining to printer front switch panel assembly (ADV/CONT/FF/ATTN)
          J4 - Connector joining to printer out of paper switch connected to ATTN
          J5 - Connector joining to printer carriage motor
          J6 - Connector joining to printer mechanism assembly including paper motor
          J7 - Connector joining to printer printhead
          J8 - Connector joining to HO?E switch (? ~ M, text unreadable on schems)
          J9 - Connector for ?
     J10/J11 - Connectors joining to LOGIC A PCA (logic A to logic B bus)
         J12 - Power input connector from power supply PCB
         J14 - Fan connector


ROM board (Operating System ROM PCA. Assembly# HP82991A or HP82995A)
---------

|----------------J3-----------------|
|                                   |
|                                   |
|                                   |
|J1  U1      U2      U3     U4    J2|
|                                   |
|                                   |
|                                   |
|----------------J4-----------------|
Notes:
      J1/J2 - 20 pin connector joining to 'Option ROM PCA'
      J3/J4 - 20 pin connector joining to 'LOGIC A PCA'
      U1-U4 - 28 pin EPROM/MASKROM 0L/1L/0H/1H (note 1Mbit: 128Kx8 28 pin)


ROM board (Option ROM PCA)
---------
Note this PCB plugs in upside-down on top of the Operating System ROM PCB

|----------------J4-----------------|
|                                   |
|                                   |
|                                   |
|J1  U1      U2      U3     U4    J2|
|                                   |
|                                   |
|                                   |
|----------------J3-----------------|
Notes:
      J1/J2 - 20 pin connector joining to 'Operating System ROM PCA'
      J3/J4 - 20 pin connector joining to 'LOGIC A PCA'
      U1-U4 - 28 pin EPROM/MASKROM 0L/1L/0H/1H (note 1Mbit: 128Kx8 28 pin)


Physical Memory Map
===================
000000-07FFFF - Internal ROM (Operating System PCA 512Kb)
080000-0FFFFF - Internal ROM (Option ROM PCA 512Kb)
100000-4FFFFF - External ROM modules (up to 4Mb)
500000-5FFFFF - Reserved 1Mb
600000-6FFFFF - Internal I/O 1Mb

                Address        Port  Use
                -------------------------
                600000-60FFFF   0    MMU
                610000-61FFFF   1    Disk Drive
                620000-62FFFF   2    Display
                630000-63FFFF   3    HP-IB
                640000-64FFFF   4    RTC
                650000-65FFFF   5    Printer
                660000-66FFFF   6    Keyboard
                670000-67FFFF   7    Speaker
                680000-68FFFF   8    Reserved
                690000-69FFFF   9    Reserved
                6A0000-6AFFFF   10   Reserved
                6B0000-6BFFFF   11   Reserved
                6C0000-6CFFFF   12   Reserved
                6D0000-6DFFFF   13   Reserved
                6E0000-6EFFFF   14   Reserved
                6F0000-6FFFFF   15   Reserved

700000-7FFFFF - External I/O 1Mb

                Address        Port  Use
                -------------------------
                700000-70FFFF   16   Mainframe Port A
                710000-71FFFF   17   Mainframe Port B
                720000-72FFFF   18   Bus Expander Port A1
                730000-73FFFF   19   Bus Expander Port A2
                740000-74FFFF   20   Bus Expander Port A3
                750000-75FFFF   21   Bus Expander Port A4
                760000-76FFFF   22   Bus Expander Port A5
                770000-77FFFF   23   Reserved
                780000-78FFFF   24   Reserved
                790000-79FFFF   25   Reserved
                7A0000-7AFFFF   26   Bus Expander Port B1
                7B0000-7BFFFF   27   Bus Expander Port B2
                7C0000-7CFFFF   28   Bus Expander Port B3
                7D0000-7DFFFF   29   Bus Expander Port B4
                7E0000-7EFFFF   30   Bus Expander Port B5
                7F0000-7FFFFF   31   Reserved

800000-EFFFFF - External RAM modules (up to 7Mb)
F00000-F7FFFF - Internal RAM 512Kb
F80000-FFFFFF - Reserved 512Kb

Access to 800000-FFFFFF can be remapped by the MMU registers.


Interrupts (all autovectored)
----------
High Priority 7 - Soft reset from keyboard (NMI)
     /\       6 - RTC or NBIR3 (external I/O)
     ||       5 - Disc Drive or NBIR2 (external I/O)
     ||       4 - GPU or NBIR1 (external I/O)
     ||       3 - HP-IB, printer or NBIR0 (external I/O)
     \/       2 - HP-HIL devices (keyboard/mouse)
Low Priority  1 - RTC

Note external interrupt lines NBIR0 to NBIR3 can be asserted by an interface connected to the external I/O port


Useful links etc.
-----------------

bitsavers://pdf/hp/integral/00095-90126_Integral_Personal_Computer_Service_Jan86.pdf

bitsavers://pdf/hp/hp-hil/45918A-90001_HP-HIL_Technical_Reference_Manual_Jan86.pdf
    HP-HIL MLC, SLC datasheets

bitsavers://pdf/sony/floppy/Sony_OA-D32_Microfloppy_Service_Nov83.pdf
    OA-D32W

http://www.hpl.hp.com/hpjournal/pdfs/IssuePDFs/1983-01.pdf
    HP-IL issue

http://www.hpl.hp.com/hpjournal/pdfs/IssuePDFs/1985-10.pdf
    IPC issue

http://www.hpl.hp.com/hpjournal/pdfs/IssuePDFs/1987-06.pdf
    HP-HIL article

http://www.hpmuseum.net/pdf/ComputerNews_1985_Jan15_37pages_OCR.pdf
    introducing the IPC

http://www.hpmuseum.net/pdf/ComputerFocus_1985_Nov_25pages_OCR.pdf
    SysV upgrade

http://www.hpmuseum.net/pdf/InformationSystemsAndManufacturingNews_81pages_Jun1-86_OCR.pdf
    EPROM/ROM modules

http://www.hpmuseum.net/pdf/HPChannels_1986_11_37pages_Nov86_OCR.pdf
    SW Eng ROM and serial option for it

http://www.coho.org/~pete/downloads/IPC/burst/Freeware/IPC_Driver_Writers_Disc/hp-ux.5.0.0
    kernel namelist

http://www.hpmuseum.net/display_item.php?hw=122
    overview, manuals, software

http://www.ambry.com/hp-computer-model/9807A.html
    replacement parts

http://www.brouhaha.com/~eric/hpcalc/chips/
    chip part numbers


Software to look for
--------------------

00095-60978 "Service ROM - Used in trobleshooting the integral PC" via ambry
00095-60925 "Service ROM" via service manual
00095-60969 "Service Diagnostic Disc" via service manual
00095-60950 "I/O Component-Level Diagnostic Disc" via serial interface service manual

00095-60006 (original System III-based HP-UX 1.0 ROM)
82995A (same bits as 82991A; the latter is an upgrade kit, former was pre-installed)
82989J Technical Basic ROM (Jan'1986)
82987A Software Engineering ROM (Nov'1986) (possibly can be rebuilt from floppy images on coho?)

******************************************************************************/

#include "emu.h"

#include "bus/hp_hil/hp_hil.h"
#include "bus/hp_hil/hil_devices.h"
#include "cpu/m68000/m68000.h"
#include "formats/hp_ipc_dsk.h"
#include "machine/bankdev.h"
#include "machine/mm58167.h"
#include "machine/ram.h"
#include "machine/wd_fdc.h"
#include "video/hp1ll3.h"

#include "rendlay.h"

#include "screen.h"


class hp_ipc_state : public driver_device
{
public:
	hp_ipc_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_bankdev(*this, "bankdev")
		, m_fdc(*this, "fdc")
		, m_ram(*this, RAM_TAG)
		, m_screen(*this, "screen")
	{ }

	virtual void machine_start() override;
	virtual void machine_reset() override;

	DECLARE_READ16_MEMBER(mem_r);
	DECLARE_WRITE16_MEMBER(mem_w);
	DECLARE_READ16_MEMBER(mmu_r);
	DECLARE_WRITE16_MEMBER(mmu_w);
	DECLARE_READ16_MEMBER(ram_r);
	DECLARE_WRITE16_MEMBER(ram_w);
	DECLARE_READ16_MEMBER(trap_r);
	DECLARE_WRITE16_MEMBER(trap_w);

	DECLARE_READ8_MEMBER(floppy_id_r);
	DECLARE_WRITE8_MEMBER(floppy_id_w);
	DECLARE_FLOPPY_FORMATS(floppy_formats);

	DECLARE_WRITE_LINE_MEMBER(irq_1);
	DECLARE_WRITE_LINE_MEMBER(irq_2);
	DECLARE_WRITE_LINE_MEMBER(irq_3);
	DECLARE_WRITE_LINE_MEMBER(irq_4);
	DECLARE_WRITE_LINE_MEMBER(irq_5);
	DECLARE_WRITE_LINE_MEMBER(irq_6);
	DECLARE_WRITE_LINE_MEMBER(irq_7);

	emu_timer *m_bus_error_timer;

private:
	required_device<m68000_device> m_maincpu;
	required_device<address_map_bank_device> m_bankdev;
	required_device<wd2797_device> m_fdc;
	required_device<ram_device> m_ram;
	required_device<screen_device> m_screen;

	uint32_t m_mmu[4], m_lowest_ram_addr;
	uint16_t *m_internal_ram;
	int m_fc;

	floppy_image_device *m_floppy;

	inline uint32_t get_ram_address(offs_t offset)
	{
		return (m_mmu[(m_maincpu->get_fc() >> 1) & 3] + offset) & 0x3FFFFF;
	}

protected:
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;
	void set_bus_error(uint32_t address, bool write, uint16_t mem_mask);
	bool m_bus_error;
};


void hp_ipc_state::device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr)
{
	m_bus_error = false;
}

void hp_ipc_state::set_bus_error(uint32_t address, bool write, uint16_t mem_mask)
{
	if (m_bus_error)
	{
		return;
	}
	if (!ACCESSING_BITS_8_15)
	{
		address++;
	}
	m_bus_error = true;
	m_maincpu->set_buserror_details(address, write, m_maincpu->get_fc());
	m_maincpu->set_input_line(M68K_LINE_BUSERROR, ASSERT_LINE);
	m_bus_error_timer->adjust(m_maincpu->cycles_to_attotime(16)); // let rmw cycles complete
}

static ADDRESS_MAP_START(hp_ipc_mem_outer, AS_PROGRAM, 16, hp_ipc_state)
	AM_RANGE(0x000000, 0xFFFFFF) AM_READWRITE(mem_r, mem_w)
ADDRESS_MAP_END

static ADDRESS_MAP_START(hp_ipc_mem_inner, AS_PROGRAM, 16, hp_ipc_state)
// user mode
	AM_RANGE(0x1000000, 0x17FFFFF) AM_READWRITE(ram_r, ram_w)
	AM_RANGE(0x1800000, 0x187FFFF) AM_ROM AM_REGION("maincpu", 0)
	AM_RANGE(0x1E20000, 0x1E2000F) AM_DEVREADWRITE8("gpu", hp1ll3_device, read, write, 0x00ff)
	AM_RANGE(0x1E40000, 0x1E4002F) AM_DEVREADWRITE8("rtc", mm58167_device, read, write, 0x00ff)

// supervisor mode
	AM_RANGE(0x0000000, 0x007FFFF) AM_ROM AM_REGION("maincpu", 0)       // Internal ROM (operating system PCA)
	AM_RANGE(0x0080000, 0x00FFFFF) AM_UNMAP     // Internal ROM (option ROM PCA)
	AM_RANGE(0x0100000, 0x04FFFFF) AM_UNMAP     // External ROM modules
	AM_RANGE(0x0600000, 0x060FFFF) AM_READWRITE(mmu_r, mmu_w)
	AM_RANGE(0x0610000, 0x0610007) AM_READWRITE8(floppy_id_r, floppy_id_w, 0x00ff)
	AM_RANGE(0x0610008, 0x061000F) AM_DEVREADWRITE8("fdc", wd2797_device, read, write, 0x00ff)
	AM_RANGE(0x0620000, 0x062000F) AM_DEVREADWRITE8("gpu", hp1ll3_device, read, write, 0x00ff)
	AM_RANGE(0x0630000, 0x063FFFF) AM_NOP       // AM_DEVREADWRITE8(TMS9914_TAG, tms9914_device, read, write, 0x00ff)
	AM_RANGE(0x0640000, 0x064002F) AM_DEVREADWRITE8("rtc", mm58167_device, read, write, 0x00ff)
	AM_RANGE(0x0650000, 0x065FFFF) AM_NOP       // HP-IL Printer (optional; ROM sets _desktop to 0 if not mapped) -- sys/lpint.h
	AM_RANGE(0x0660000, 0x06600FF) AM_DEVREADWRITE8("mlc", hp_hil_mlc_device, read, write, 0x00ff)  // 'caravan', scrn/caravan.h
	AM_RANGE(0x0670000, 0x067FFFF) AM_NOP       // Speaker (NatSemi COP 452)
	AM_RANGE(0x0680000, 0x068FFFF) AM_NOP       // 'SIMON (98628) fast HP-IB card' -- sys/simon.h
	AM_RANGE(0x0700000, 0x07FFFFF) AM_UNMAP     // External I/O
	AM_RANGE(0x0800000, 0x0FFFFFF) AM_READWRITE(ram_r, ram_w)

// bus error handler
	AM_RANGE(0x0000000, 0x1FFFFFF) AM_READWRITE(trap_r, trap_w)
ADDRESS_MAP_END

static INPUT_PORTS_START(hp_ipc)
INPUT_PORTS_END


READ16_MEMBER(hp_ipc_state::mmu_r)
{
	uint16_t data = (m_mmu[offset & 3] >> 10);

	return data;
}

WRITE16_MEMBER(hp_ipc_state::mmu_w)
{
	m_mmu[offset & 3] = (data & 0xFFF) << 10;
}

READ16_MEMBER(hp_ipc_state::mem_r)
{
	int fc = m_maincpu->get_fc() & 4;

	if (fc != m_fc)
	{
		m_fc = fc;
		m_bankdev->set_bank(m_fc ? 0 : 1);
	}

	return m_bankdev->read16(space, offset, mem_mask);
}

WRITE16_MEMBER(hp_ipc_state::mem_w)
{
	int fc = m_maincpu->get_fc() & 4;

	if (fc != m_fc)
	{
		m_fc = fc;
		m_bankdev->set_bank(m_fc ? 0 : 1);
	}

	m_bankdev->write16(space, offset, data, mem_mask);
}

READ16_MEMBER(hp_ipc_state::trap_r)
{
	if (!machine().side_effect_disabled()) set_bus_error((offset << 1) & 0xFFFFFF, 0, mem_mask);

	return 0xffff;
}

WRITE16_MEMBER(hp_ipc_state::trap_w)
{
	if (!machine().side_effect_disabled()) set_bus_error((offset << 1) & 0xFFFFFF, 1, mem_mask);
}


READ16_MEMBER(hp_ipc_state::ram_r)
{
	uint32_t ram_address = get_ram_address(offset);
	uint16_t data = 0xffff;

	if (ram_address < m_lowest_ram_addr)
	{
		if (!machine().side_effect_disabled()) set_bus_error((offset << 1) + 0x800000, 0, mem_mask);
	}
	else if (ram_address < 0x3c0000)
	{
		// Internal RAM
		data = m_internal_ram[ram_address - m_lowest_ram_addr];
	}

	return data;
}

WRITE16_MEMBER(hp_ipc_state::ram_w)
{
	uint32_t ram_address = get_ram_address(offset);

	if (ram_address < m_lowest_ram_addr)
	{
		if (!machine().side_effect_disabled()) set_bus_error((offset << 1) + 0x800000, 1, mem_mask);
	}
	else if (ram_address < 0x3c0000)
	{
		// Internal RAM
		COMBINE_DATA(&m_internal_ram[ram_address - m_lowest_ram_addr]);
	}
}


/*
 * bit 6 -- INTRQ
 * bit 1 -- disk changed (from drive)
 * bit 0 -- write protect (from drive)
 */
READ8_MEMBER(hp_ipc_state::floppy_id_r)
{
	uint8_t data = 0;

	data = (m_fdc->intrq_r() << 6);

	if (m_floppy)
	{
		data |= (m_fdc->intrq_r() << 6) | (m_floppy->dskchg_r() << 1) | m_floppy->wpt_r();
	}

	return data;
}

/*
 * bit 7 -- 1: motor on (via inverter to drive's /MTorOn)
 * bit 3 -- 1: head 0, 0: head 1 (via inverter to drive's /SSO)
 * bit 1 -- 1: drive select (via inverter to drive's /DRIVE SEL 1)
 * bit 0 -- 1: reset disc_changed (via inverter to drive's /DSKRST)
 */
WRITE8_MEMBER(hp_ipc_state::floppy_id_w)
{
	floppy_image_device *floppy0 = m_fdc->subdevice<floppy_connector>("0")->get_device();

	if (!BIT(data, 1))
	{
		if (m_floppy == nullptr)
		{
			m_floppy = floppy0;
			m_fdc->set_floppy(m_floppy);
		}
	}
	else
	{
		m_floppy = nullptr;
	}

	if (m_floppy)
	{
		m_floppy->ss_w(BIT(data, 3));
		m_floppy->mon_w(!BIT(data, 7));
		if (BIT(data, 0))
			m_floppy->dskchg_w(0);
	}
	else
	{
		floppy0->mon_w(1);
	}
}


WRITE_LINE_MEMBER(hp_ipc_state::irq_1)
{
	m_maincpu->set_input_line_and_vector(M68K_IRQ_1, state, M68K_INT_ACK_AUTOVECTOR);
}

WRITE_LINE_MEMBER(hp_ipc_state::irq_2)
{
	m_maincpu->set_input_line_and_vector(M68K_IRQ_2, state, M68K_INT_ACK_AUTOVECTOR);
}

WRITE_LINE_MEMBER(hp_ipc_state::irq_3)
{
	m_maincpu->set_input_line_and_vector(M68K_IRQ_3, state, M68K_INT_ACK_AUTOVECTOR);
}

WRITE_LINE_MEMBER(hp_ipc_state::irq_4)
{
	m_maincpu->set_input_line_and_vector(M68K_IRQ_4, state, M68K_INT_ACK_AUTOVECTOR);
}

WRITE_LINE_MEMBER(hp_ipc_state::irq_5)
{
	m_maincpu->set_input_line_and_vector(M68K_IRQ_5, state, M68K_INT_ACK_AUTOVECTOR);
}

WRITE_LINE_MEMBER(hp_ipc_state::irq_6)
{
	m_maincpu->set_input_line_and_vector(M68K_IRQ_6, state, M68K_INT_ACK_AUTOVECTOR);
}

WRITE_LINE_MEMBER(hp_ipc_state::irq_7)
{
	m_maincpu->set_input_line_and_vector(M68K_IRQ_7, state, M68K_INT_ACK_AUTOVECTOR);
}


void hp_ipc_state::machine_start()
{
	m_bus_error_timer = timer_alloc(0);

	m_bankdev->set_bank(1);

	m_lowest_ram_addr = 0x3c0000 - (m_ram->size() >> 1);
	m_internal_ram = (uint16_t *) m_ram->pointer();
}

void hp_ipc_state::machine_reset()
{
	m_floppy = nullptr;
}


FLOPPY_FORMATS_MEMBER( hp_ipc_state::floppy_formats )
	FLOPPY_HP_IPC_FORMAT
FLOPPY_FORMATS_END

static SLOT_INTERFACE_START( hp_ipc_floppies )
	SLOT_INTERFACE( "35dd", SONY_OA_D32W )
SLOT_INTERFACE_END

/*
 *  IRQ levels (page 5-4)
 *
 *  7   Soft reset from keyboard, non-maskable
 *  6   Real-time clock or NBIR3 (ext. I/O)
 *  5   Disc Drive or NBIR2
 *  4   GPU or NBIR1
 *  3   HP-IB, printer, or NBIR0
 *  2   HP-HIL devices (keyboard, mouse)
 *  1   Real-time clock
 */
static MACHINE_CONFIG_START(hp_ipc)
	MCFG_CPU_ADD("maincpu", M68000, XTAL_15_92MHz / 2)
	MCFG_CPU_PROGRAM_MAP(hp_ipc_mem_outer)

	MCFG_DEVICE_ADD("bankdev", ADDRESS_MAP_BANK, 0)
	MCFG_DEVICE_PROGRAM_MAP(hp_ipc_mem_inner)
	MCFG_ADDRESS_MAP_BANK_ENDIANNESS(ENDIANNESS_BIG)
	MCFG_ADDRESS_MAP_BANK_ADDRBUS_WIDTH(25)
	MCFG_ADDRESS_MAP_BANK_DATABUS_WIDTH(16)
	MCFG_ADDRESS_MAP_BANK_STRIDE(0x1000000)

	// horizontal time = 60 us (min)
	// ver.refresh period = ~300 us
	// ver.period = 16.7ms (~60 hz)
	MCFG_SCREEN_ADD_MONOCHROME("screen", RASTER, rgb_t::amber())
	MCFG_SCREEN_UPDATE_DEVICE("gpu", hp1ll3_device, screen_update)
	MCFG_SCREEN_RAW_PARAMS(XTAL_6MHz * 2, 720, 0, 512, 278, 0, 256)
//  when _desktop == 0:
//  MCFG_SCREEN_RAW_PARAMS(XTAL_6MHz * 2, 720, 0, 640, 480, 0, 400)
	MCFG_SCREEN_VBLANK_CALLBACK(DEVWRITELINE("mlc", hp_hil_mlc_device, ap_w)) // XXX actually it's driven by 555 (U59)
	MCFG_DEFAULT_LAYOUT(layout_lcd)

	MCFG_SCREEN_PALETTE("palette")
	MCFG_PALETTE_ADD_MONOCHROME("palette")

	MCFG_HP1LL3_ADD("gpu")
//  MCFG_HP1LL3_IRQ_CALLBACK(WRITELINE(hp_ipc_state, irq_4))
	MCFG_VIDEO_SET_SCREEN("screen")

	// XXX actual clock is 1MHz; remove this workaround (and change 2000 to 100 in hp_ipc_dsk.cpp)
	// XXX when floppy code correctly handles 600 rpm drives.
	MCFG_WD2797_ADD("fdc", XTAL_2MHz)
	MCFG_WD_FDC_INTRQ_CALLBACK(WRITELINE(hp_ipc_state, irq_5))
	MCFG_FLOPPY_DRIVE_ADD("fdc:0", hp_ipc_floppies, "35dd", hp_ipc_state::floppy_formats)

	MCFG_SOFTWARE_LIST_ADD("flop_list","hp_ipc")

	MCFG_DEVICE_ADD("rtc", MM58167, XTAL_32_768kHz)
	MCFG_MM58167_IRQ_CALLBACK(WRITELINE(hp_ipc_state, irq_1))
//  MCFG_MM58167_STANDBY_IRQ_CALLBACK(WRITELINE(hp_ipc_state, irq_6))

	MCFG_DEVICE_ADD("mlc", HP_HIL_MLC, XTAL_15_92MHz/2)
	MCFG_HP_HIL_INT_CALLBACK(WRITELINE(hp_ipc_state, irq_2))
	MCFG_HP_HIL_NMI_CALLBACK(WRITELINE(hp_ipc_state, irq_7))
	MCFG_HP_HIL_SLOT_ADD("mlc", "hil1", hp_hil_devices, "hp_ipc_kbd")

	MCFG_RAM_ADD(RAM_TAG)
	MCFG_RAM_DEFAULT_SIZE("512K")
	MCFG_RAM_EXTRA_OPTIONS("768K,1M,1576K,2M,3M,4M,5M,6M,7M,7680K")
MACHINE_CONFIG_END


ROM_START(hp_ipc)
	ROM_REGION(0x100000, "maincpu" , 0)
	ROM_LOAD("hp ipc os 82991A.bin", 0x00000, 0x80000, BAD_DUMP CRC(df45a37b) SHA1(476af9923bca0d2d0f40aeb81be5145ca76fddf5)) // Should be spread across 4 x 128K ROMs
ROM_END


COMP(1985, hp_ipc, 0, 0, hp_ipc, hp_ipc, hp_ipc_state, 0, "HP", "Integral Personal Computer", MACHINE_NO_SOUND | MACHINE_IMPERFECT_GRAPHICS | MACHINE_IMPERFECT_KEYBOARD)
