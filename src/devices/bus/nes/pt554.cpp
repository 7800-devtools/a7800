// license:BSD-3-Clause
// copyright-holders:Fabio Priuli
/***********************************************************************************************************


 NES/Famicom cartridge emulation for Bandai PT-554 PCBs


 Here we emulate the following Bandai PT-554 PCB (a CNROM PCB + LPC / PARCOR speech synthesis chip)

 TODO:
 - emulate the mat controller

 ***********************************************************************************************************/


#include "emu.h"
#include "pt554.h"
#include "speaker.h"


#ifdef NES_PCB_DEBUG
#define VERBOSE 1
#else
#define VERBOSE 0
#endif

#define LOG_MMC(x) do { if (VERBOSE) logerror x; } while (0)


//-------------------------------------------------
//  constructor
//-------------------------------------------------

DEFINE_DEVICE_TYPE(NES_BANDAI_PT554, nes_bandai_pt554_device, "nes_bandai_pt554", "NES Cart Bandai BT-554 PCB")


nes_bandai_pt554_device::nes_bandai_pt554_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: nes_cnrom_device(mconfig, NES_BANDAI_PT554, tag, owner, clock)
	, m_samples(*this, "samples")
{
}



/*-------------------------------------------------
 mapper specific handlers
 -------------------------------------------------*/

/*-------------------------------------------------

 Bandai PT-554 board emulation

 This is used by Aerobics Studio. It is basically a CNROM board
 with an Mitsubishi M50805 LPC / PARCOR speech synthesis chip
 with internal tables stored in ROM which have not yet been dumped.

 iNES: mapper 3?

 -------------------------------------------------*/

WRITE8_MEMBER(nes_bandai_pt554_device::write_m)
{
	LOG_MMC(("Bandai PT-554 Sound write, data: %02x\n", data));

	if (!BIT(data, 6))
		m_samples->start(data & 0x07, data & 0x07);
	else
		m_samples->stop(data & 0x07);
}

static const char *const pt554_sample_names[] =
{
	"*ftaerobi",
	"00",
	"01",
	"02",
	"03",
	"04",
	"05",
	"06",
	"07",
	nullptr
};


//-------------------------------------------------
//  device_add_mconfig - add device configuration
//-------------------------------------------------

MACHINE_CONFIG_MEMBER( nes_bandai_pt554_device::device_add_mconfig )

	// additional sound hardware
	MCFG_SPEAKER_STANDARD_MONO("addon")

	MCFG_SOUND_ADD("samples", SAMPLES, 0)
	MCFG_SAMPLES_CHANNELS(8)
	MCFG_SAMPLES_NAMES(pt554_sample_names)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "addon", 0.50)
MACHINE_CONFIG_END
