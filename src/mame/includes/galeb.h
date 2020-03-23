// license:BSD-3-Clause
// copyright-holders:Miodrag Milanovic
/*****************************************************************************
 *
 * includes/galeb.h
 *
 ****************************************************************************/

#ifndef GALEB_H_
#define GALEB_H_

#include "sound/dac.h"

class galeb_state : public driver_device
{
public:
	galeb_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_video_ram(*this, "video_ram"),
		m_maincpu(*this, "maincpu"),
		m_gfxdecode(*this, "gfxdecode"),
		m_palette(*this, "palette"),
		m_keyboard(*this, "LINE%u", 0),
		m_dac(*this, "dac"),
		m_dac_state(0)
	{
	}

	required_shared_ptr<uint8_t> m_video_ram;
	DECLARE_WRITE8_MEMBER(dac_w);
	DECLARE_READ8_MEMBER(keyboard_r);
	DECLARE_READ8_MEMBER(tape_status_r);
	DECLARE_READ8_MEMBER(tape_data_r);
	DECLARE_WRITE8_MEMBER(tape_data_w);
	virtual void video_start() override;
	uint32_t screen_update_galeb(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	required_device<cpu_device> m_maincpu;
	required_device<gfxdecode_device> m_gfxdecode;
	required_device<palette_device> m_palette;
	required_ioport_array<8> m_keyboard;
	required_device<dac_1bit_device> m_dac;

protected:
	virtual void machine_start() override;

private:
	int m_dac_state;
};

/*----------- defined in video/galeb.c -----------*/

extern const gfx_layout galeb_charlayout;

#endif /* GALEB_H_ */
