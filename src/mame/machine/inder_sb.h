// license:BSD-3-Clause
// copyright-holders:David Haywood
/* */


#ifndef MAME_MACHINE_INDER_SB_H
#define MAME_MACHINE_INDER_SB_H

#pragma once

#include "cpu/z80/z80.h"
#include "machine/z80ctc.h"
#include "cpu/z80/z80daisy.h"
#include "sound/dac.h"


DECLARE_DEVICE_TYPE(INDER_AUDIO, inder_sb_device)

#define MCFG_INDER_AUDIO_ADD(_tag) \
	MCFG_DEVICE_ADD(_tag, INDER_AUDIO, 0)


class inder_sb_device :  public device_t,
							public device_mixer_interface
{
public:
	// construction/destruction
	inder_sb_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	required_device<cpu_device> m_audiocpu;
	required_device<z80ctc_device> m_ctc;

	DECLARE_READ8_MEMBER(megaphx_sound_sent_r);
	DECLARE_READ8_MEMBER(megaphx_sound_cmd_r);
	DECLARE_WRITE8_MEMBER(megaphx_sound_to_68k_w);

	DECLARE_WRITE8_MEMBER(dac0_rombank_write);
	DECLARE_WRITE8_MEMBER(dac1_rombank_write);
	DECLARE_WRITE8_MEMBER(dac2_rombank_write);
	DECLARE_WRITE8_MEMBER(dac3_rombank_write);

	DECLARE_READ8_MEMBER(megaphx_02cc_hack_r);
	DECLARE_READ8_MEMBER(megaphx_02e6_hack_r);
	DECLARE_READ8_MEMBER(megaphx_0309_hack_r);
	DECLARE_READ8_MEMBER(megaphx_0323_hack_r);


	DECLARE_READ16_MEMBER(megaphx_0x050002_r);
	DECLARE_WRITE16_MEMBER(megaphx_0x050000_w);

	void install_sound_hacks(void);
	void update_sound_irqs(void);

protected:
	virtual void device_add_mconfig(machine_config &config) override;
	virtual void device_start() override;
	virtual void device_reset() override;

private:
	uint8_t m_soundbank[4];

	int m_soundsent;
	uint8_t m_sounddata;
	uint8_t m_soundback;

	int m_soundirq;

	DECLARE_WRITE_LINE_MEMBER(z80ctc_ch0);
	DECLARE_WRITE_LINE_MEMBER(z80ctc_ch1);
	DECLARE_WRITE_LINE_MEMBER(z80ctc_ch2);
	DECLARE_WRITE_LINE_MEMBER(z80ctc_ch3);
};

#endif // MAME_MACHINE_INDER_SB_H
