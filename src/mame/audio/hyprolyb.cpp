// license:BSD-3-Clause
// copyright-holders:Chris Hardy
#include "emu.h"
#include "cpu/m6800/m6800.h"
#include "audio/hyprolyb.h"

DEFINE_DEVICE_TYPE(HYPROLYB_ADPCM, hyprolyb_adpcm_device, "hyprolyb_adpcm", "Hyper Olympics Audio")

hyprolyb_adpcm_device::hyprolyb_adpcm_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, HYPROLYB_ADPCM, tag, owner, clock)
	, device_sound_interface(mconfig, *this)
	, m_soundlatch2(*this, ":soundlatch2")
	, m_adpcm_ready(0)
	, m_adpcm_busy(0)
	, m_vck_ready(0)
{
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void hyprolyb_adpcm_device::device_start()
{
	m_space = &machine().device("audiocpu")->memory().space(AS_PROGRAM);
	m_msm = machine().device<msm5205_device>("msm");
	save_item(NAME(m_adpcm_ready));  // only bootlegs
	save_item(NAME(m_adpcm_busy));
	save_item(NAME(m_vck_ready));
}

//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void hyprolyb_adpcm_device::device_reset()
{
	m_adpcm_ready = 0;
	m_adpcm_busy = 0;
	m_vck_ready = 0;
}

WRITE8_MEMBER( hyprolyb_adpcm_device::write )
{
	m_soundlatch2->write(*m_space, offset, data);
	m_adpcm_ready = 0x80;
}

READ8_MEMBER( hyprolyb_adpcm_device::busy_r )
{
	return m_adpcm_busy ? 0x10 : 0x00;
}

WRITE8_MEMBER( hyprolyb_adpcm_device::msm_data_w )
{
	m_msm->data_w(data);
	m_adpcm_busy = ~data & 0x80;
}

READ8_MEMBER( hyprolyb_adpcm_device::msm_vck_r )
{
	uint8_t old = m_vck_ready;
	m_vck_ready = 0x00;
	return old;
}

READ8_MEMBER( hyprolyb_adpcm_device::ready_r )
{
	return m_adpcm_ready;
}

READ8_MEMBER( hyprolyb_adpcm_device::data_r )
{
	m_adpcm_ready = 0x00;
	return m_soundlatch2->read(*m_space, offset);
}

static ADDRESS_MAP_START( hyprolyb_adpcm_map, AS_PROGRAM, 8, hyprolyb_adpcm_device )
	AM_RANGE(0x0000, 0x007f) AM_RAM
	AM_RANGE(0x1000, 0x1000) AM_DEVREAD("hyprolyb_adpcm", hyprolyb_adpcm_device, data_r)
	AM_RANGE(0x1001, 0x1001) AM_DEVREAD("hyprolyb_adpcm", hyprolyb_adpcm_device, ready_r)
	AM_RANGE(0x1002, 0x1002) AM_DEVWRITE("hyprolyb_adpcm", hyprolyb_adpcm_device, msm_data_w)
	AM_RANGE(0x1003, 0x1003) AM_DEVREAD("hyprolyb_adpcm", hyprolyb_adpcm_device, msm_vck_r)
		// on init:
		//    $1003 = $00
		//    $1002 = $FF
		//    $1003 = $34
		//    $1001 = $36
		//    $1002 = $80
		// loops while ($1003) & 0x80 == 0
		// 1002 = ADPCM data written (low 4 bits)
		//
		// $1003 & $80 (in) = 5205 DRQ
		// $1002 & $0f (out) = 5205 data
		// $1001 & $80 (in) = sound latch request
		// $1000 (in) = sound latch data
	AM_RANGE(0x8000, 0xffff) AM_ROM
ADDRESS_MAP_END

void hyprolyb_adpcm_device::vck_callback( int st )
{
	m_vck_ready = 0x80;
}

MACHINE_CONFIG_START( hyprolyb_adpcm )
	MCFG_CPU_ADD("adpcm", M6802, XTAL_14_31818MHz/8)    /* unknown clock */
	MCFG_CPU_PROGRAM_MAP(hyprolyb_adpcm_map)

	MCFG_GENERIC_LATCH_8_ADD("soundlatch2")

	MCFG_SOUND_ADD("hyprolyb_adpcm", HYPROLYB_ADPCM, 0)

	MCFG_SOUND_ADD("msm", MSM5205, 384000)
	MCFG_MSM5205_VCLK_CB(DEVWRITELINE("hyprolyb_adpcm", hyprolyb_adpcm_device, vck_callback)) /* VCK function */
	MCFG_MSM5205_PRESCALER_SELECTOR(S96_4B)      /* 4 kHz */
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "speaker", 0.5)
MACHINE_CONFIG_END


//-------------------------------------------------
//  sound_stream_update - handle a stream update
//-------------------------------------------------

void hyprolyb_adpcm_device::sound_stream_update(sound_stream &stream, stream_sample_t **inputs, stream_sample_t **outputs, int samples)
{
	// should never get here
	fatalerror("sound_stream_update called; not applicable to legacy sound devices\n");
}
