// license:BSD-3-Clause
// copyright-holders:Nathan Woods, Curt Coder
/**********************************************************************

    MOS 6581/8580 Sound Interface Device emulation

**********************************************************************/

#include "emu.h"
#include "mos6581.h"
#include "sid.h"



//**************************************************************************
//  MACROS / CONSTANTS
//**************************************************************************

#define LOG 0



//**************************************************************************
//  DEVICE DEFINITIONS
//**************************************************************************

// device type definition
DEFINE_DEVICE_TYPE(MOS6581, mos6581_device, "mos6581", "MOS 6581 SID")
DEFINE_DEVICE_TYPE(MOS8580, mos8580_device, "mos8580", "MOS 8580 SID")



//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  mos6581_device - constructor
//-------------------------------------------------

mos6581_device::mos6581_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock, uint32_t variant)
	: device_t(mconfig, type, tag, owner, clock)
	, device_sound_interface(mconfig, *this)
	, m_read_potx(*this)
	, m_read_poty(*this)
	, m_stream(nullptr)
	, m_variant(variant)
	, m_token(make_unique_clear<SID6581_t>())

{
}

mos6581_device::mos6581_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: mos6581_device(mconfig, MOS6581, tag, owner, clock, TYPE_6581)
{
}

mos6581_device::~mos6581_device()
{
}

//-------------------------------------------------
//  mos8580_device - constructor
//-------------------------------------------------

mos8580_device::mos8580_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: mos6581_device(mconfig, MOS8580, tag, owner, clock, TYPE_8580)
{
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void mos6581_device::device_start()
{
	// resolve callbacks
	m_read_potx.resolve_safe(0xff);
	m_read_poty.resolve_safe(0xff);

	// create sound stream
	m_stream = machine().sound().stream_alloc(*this, 0, 1, machine().sample_rate());

	// initialize SID engine
	m_token->device = this;
	m_token->mixer_channel = m_stream;
	m_token->PCMfreq = machine().sample_rate();
	m_token->clock = clock();
	m_token->type = m_variant;

	m_token->init();
	sidInitWaveformTables(m_variant);
}


//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void mos6581_device::device_reset()
{
	m_token->reset();
}


//-------------------------------------------------
//  sound_stream_update - handle update requests for
//  our sound stream
//-------------------------------------------------

void mos6581_device::sound_stream_update(sound_stream &stream, stream_sample_t **inputs, stream_sample_t **outputs, int samples)
{
	m_token->fill_buffer(outputs[0], samples);
}


//-------------------------------------------------
//  read -
//-------------------------------------------------

READ8_MEMBER( mos6581_device::read )
{
	uint8_t data;

	switch (offset & 0x1f)
	{
	case 0x19:
		data = m_read_potx(0);
		break;

	case 0x1a:
		data = m_read_poty(0);
		break;

	default:
		data = m_token->port_r(machine(), offset);
		break;
	}

	return data;
}


//-------------------------------------------------
//  write -
//-------------------------------------------------

WRITE8_MEMBER( mos6581_device::write )
{
	m_token->port_w(offset, data);
}
