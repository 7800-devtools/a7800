// license:LGPL-2.1+
// copyright-holders:Angelo Salese, R. Belmont
/*

    dc.c - Sega Dreamcast hardware

    MESS (DC home console) hardware overrides (GD-ROM drive etc)

    c230048 - 5 is written, want 6
    c0d9d9e - where bad happens, from routine @ c0da260

    c0d9d8e - R0 on return is the value to put in

    cfffee0 - stack location when bad happens

    TODO:
    - gdrom_alt_status is identical to normal status except that "but it does not clear DMA status information when it is accessed"

*/

#include "emu.h"
#include "cdrom.h"
#include "debugger.h"
#include "includes/dc.h"
#include "cpu/sh4/sh4.h"
#include "sound/aica.h"
#include "includes/dccons.h"

#define ATAPI_CYCLES_PER_SECTOR (5000)  // TBD for Dreamcast

WRITE_LINE_MEMBER(dc_cons_state::ata_interrupt)
{
	if (state)
		dc_sysctrl_regs[SB_ISTEXT] |= IST_EXT_GDROM;
	else
		dc_sysctrl_regs[SB_ISTEXT] &= ~IST_EXT_GDROM;

	dc_update_interrupt_status();
}

TIMER_CALLBACK_MEMBER(dc_cons_state::atapi_xfer_end )
{
	uint8_t sector_buffer[ 4096 ];

	atapi_timer->adjust(attotime::never);

	printf("atapi_xfer_end atapi_xferlen = %d\n", atapi_xferlen );

	//osd_printf_debug("ATAPI: xfer_end.  xferlen = %d\n", atapi_xferlen);

	m_ata->write_dmack(1);

	while (atapi_xferlen > 0 )
	{
		struct sh4_ddt_dma ddtdata;

		// get a sector from the SCSI device
		for (int i = 0; i < 2048/2; i++)
		{
			int d = m_ata->read_dma();
			sector_buffer[ i*2 ] = d & 0xff;
			sector_buffer[ (i*2)+1 ] = d >> 8;
		}

		atapi_xferlen -= 2048;

		// perform the DMA
		ddtdata.destination = atapi_xferbase;   // destination address
		ddtdata.length = 2048/4;
		ddtdata.size = 4;
		ddtdata.buffer = sector_buffer;
		ddtdata.direction=1;    // 0 source to buffer, 1 buffer to destination
		ddtdata.channel= 0;
		ddtdata.mode= -1;       // copy from/to buffer
		printf("ATAPI: DMA one sector to %x, %x remaining\n", atapi_xferbase, atapi_xferlen);
		m_maincpu->sh4_dma_ddt(&ddtdata);

		atapi_xferbase += 2048;
	}

	m_ata->write_dmack(0);

	g1bus_regs[SB_GDST]=0;
	dc_sysctrl_regs[SB_ISTNRM] |= IST_DMA_GDROM;
	dc_update_interrupt_status();
}

void dc_cons_state::dreamcast_atapi_init()
{
	atapi_timer = machine().scheduler().timer_alloc(timer_expired_delegate(FUNC(dc_cons_state::atapi_xfer_end),this));
	atapi_timer->adjust(attotime::never);
	save_item(NAME(atapi_xferlen));
	save_item(NAME(atapi_xferbase));
}

/*

 GDROM regsters:

 5f7018: alternate status/device control
 5f7080: data
 5f7084: error/features
 5f7088: interrupt reason/sector count
 5f708c: sector number
 5f7090: byte control low
 5f7094: byte control high
 5f7098: drive select
 5f709c: status/command

c002910 - ATAPI packet writes
c002796 - aux status read after that
c000776 - DMA triggered to c008000

*/

READ32_MEMBER(dc_cons_state::dc_mess_g1_ctrl_r )
{
	switch(offset)
	{
		case SB_GDSTARD:
			printf("G1CTRL: GDSTARD %08x\n", atapi_xferbase); // Hello Kitty reads here
			machine().debug_break();
			return atapi_xferbase;
		case SB_GDST:
			break;
		case SB_GDLEND:
			//machine().debug_break();
			return atapi_xferlen; // TODO: check me
		case SB_SECUR_EADR:     // always read 0xFF on hardware
			return 0x000000ff;
		case SB_SECUR_STATE:    // state of BIOS checksum security system (R/O):
								// 3 - check passed OK, G1 ATA (5F70xx) registers area accessible
								// 2 - check failed, G1 ATA area blocked (read FFFFFFFFh)
								// 0 - check in progress, BIOS data summed, G1 ATA area blocked (read FFFFFFFFh)
			return 3;
		default:
			printf("G1CTRL:  Unmapped read %08x\n", 0x5f7400+offset*4);
			machine().debug_break();
			break;
	}
	return g1bus_regs[offset];
}

WRITE32_MEMBER(dc_cons_state::dc_mess_g1_ctrl_w )
{
	g1bus_regs[offset] = data; // 5f7400+reg*4=dat
//  osd_printf_verbose("%s",string_format("G1CTRL: [%08x=%x] write %I64x to %x, mask %I64x\n", 0x5f7400+reg*4, dat, data, offset, mem_mask).c_str());
	switch (offset)
	{
	case SB_GDST:
		if (data & 1 && g1bus_regs[SB_GDEN] == 1) // 0 -> 1
		{
			if (g1bus_regs[SB_GDDIR] == 0)
			{
				printf("G1CTRL: unsupported transfer\n");
				return;
			}

			atapi_xferbase = g1bus_regs[SB_GDSTAR];
			//atapi_timer->adjust(m_maincpu->cycles_to_attotime((ATAPI_CYCLES_PER_SECTOR * (atapi_xferlen/2048))));
			/* 12x * 75 Hz = 0,00(1) secs per sector */
			/* TODO: make DMA to be single step */
			atapi_timer->adjust(attotime::from_usec(1111*atapi_xferlen/2048));
//          atapi_regs[ATAPI_REG_SAMTAG] = GDROM_PAUSE_STATE | 0x80;
		}
		break;

	case SB_GDLEN:
		atapi_xferlen = data;
		break;

/*
    The following register is involved in BIOS checksum protection system.
    current understanding of its functioning based on several hardware tests:

    after power on security system is in state 0 (check in progress):
    - access to G1 ATA register area (5F70XX) is blocked, ie GD-ROM in Dreamcast or cartridge/DIMM in arcade systems is not accessible;
    - *any* data readed via G1 data bus (i.e. BIOS) is summed internally by chipset;
    - write to SB_SECUR_EADR register set last address of checksummed area;

    then read address will match SB_SECUR_EADR - calculated summ compared with some hardcoded value
    if values match - security system becomes in state 3 (check OK):
    - G1 ATA registers area unlocked;
    - can be switched back to state 0 by write to SB_SECUR_EADR register, Dreamcast BIOS write 42FEh before jump into Mil-CD executables

    if values doesn't match - security system switch to state 2 (check fail):
    - G1 ATA locked
    - can be switched to state 0 by write to SB_SECUR_EADR register, however passing valid data block through security system set it back to state 2
    - the only exit from this state - power off/on or reset;

    current state can be read from SB_SECUR_STATE register
    actual checksum algorithm is unknown, but its supposed to be simple and weak,
    known few modded BIOSes which succesfully passes this CRC check, because of good luck

    all described above works the same way in all HOLLY/CLX2-based systems - Dreamcast, Naomi 1/2, Atomiswave, SystemSP
*/
	case SB_SECUR_EADR:
		if (data==0 || data==0x001fffff || data==0x42fe)
		{
//          atapi_regs[ATAPI_REG_SAMTAG] = GDROM_PAUSE_STATE | 0x80;
			printf("Unlocking GD-ROM! %x\n", data);
		}
		break;
	}
}
