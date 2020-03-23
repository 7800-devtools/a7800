// license:GPL-2.0+
// copyright-holders:Dirk Best
/***************************************************************************

    CR-511-B CD-ROM drive

    CD-ROM drive with a custom MKE/Panasonic interface as used in the
    Commodore CDTV and early SoundBlaster cards.

***************************************************************************/

#include "emu.h"
#include "cr511b.h"


//**************************************************************************
//  DEVICE DEFINITIONS
//**************************************************************************

DEFINE_DEVICE_TYPE(CR511B, cr511b_device, "cr511b", "CR-511-B CD-ROM drive")

//-------------------------------------------------
//  device_add_mconfig - add device configuration
//-------------------------------------------------

MACHINE_CONFIG_MEMBER( cr511b_device::device_add_mconfig )
	MCFG_CDROM_ADD("cdrom")
	MCFG_CDROM_INTERFACE("cdrom")
	MCFG_SOUND_ADD("cdda", CDDA, 0)
	MCFG_SOUND_ROUTE(0, ":lspeaker", 1.0)
	MCFG_SOUND_ROUTE(1, ":rspeaker", 1.0)
MACHINE_CONFIG_END


//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  cr511b_device - constructor
//-------------------------------------------------

cr511b_device::cr511b_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, CR511B, tag, owner, clock),
	m_cdrom(*this, "cdrom"),
	m_cdda(*this, "cdda"),
	m_stch_handler(*this),
	m_sten_handler(*this),
	m_drq_handler(*this),
	m_dten_handler(*this),
	m_scor_handler(*this),
	m_xaen_handler(*this),
	m_frame_timer(nullptr),
	//m_motor(false),
	m_enabled(-1),
	m_cmd(-1)
{
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void cr511b_device::device_start()
{
	// resolve callbacks
	m_stch_handler.resolve_safe();
	m_sten_handler.resolve_safe();
	m_drq_handler.resolve_safe();
	m_dten_handler.resolve_safe();
	m_scor_handler.resolve_safe();
	m_xaen_handler.resolve_safe();

	m_frame_timer = timer_alloc(0, nullptr);
	m_frame_timer->adjust(attotime::never);
}

//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void cr511b_device::device_reset()
{
}

//-------------------------------------------------
//  device_timer - device-specific timer events
//-------------------------------------------------

void cr511b_device::device_timer(emu_timer &timer, device_timer_id tid, int param, void *ptr)
{
}


//**************************************************************************
//  IMPLEMENTATION
//**************************************************************************

READ8_MEMBER( cr511b_device::read )
{
	return 0xff;
}

WRITE8_MEMBER ( cr511b_device::write )
{
}

WRITE_LINE_MEMBER( cr511b_device::enable_w )
{
	m_enabled = state;
}

WRITE_LINE_MEMBER( cr511b_device::cmd_w )
{
	m_cmd = state;
}
