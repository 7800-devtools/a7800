// license:BSD-3-Clause
// copyright-holders:Sergey Svishchev
/*********************************************************************

    agat7video.h

    Implementation of Agat-7 onboard video.

*********************************************************************/

#ifndef MAME_VIDEO_AGAT7_H
#define MAME_VIDEO_AGAT7_H

#pragma once

#include "machine/ram.h"


//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

class agat7video_device : public device_t
{
public:
	// construction/destruction
	agat7video_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	DECLARE_READ8_MEMBER(read);
	DECLARE_WRITE8_MEMBER(write);

	ram_device *m_ram_dev;
	uint8_t *m_char_ptr;
	int m_char_size;

protected:
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_add_mconfig(machine_config &config) override;

	void text_update_lores(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect, int beginrow, int endrow);
	void text_update_hires(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect, int beginrow, int endrow);
	void graph_update_mono(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect, int beginrow, int endrow);
	void graph_update_lores(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect, int beginrow, int endrow);
	void graph_update_hires(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect, int beginrow, int endrow);

private:
	void do_io(int offset);

	uint32_t m_start_address;
	enum {
		TEXT_LORES = 0,
		TEXT_HIRES,
		GRAPHICS_LORES,
		GRAPHICS_HIRES,
		GRAPHICS_MONO
	} m_video_mode;

	void plot_text_character(bitmap_ind16 &bitmap, int xpos, int ypos, int xscale, uint32_t code, const uint8_t *textgfx_data, uint32_t textgfx_datalen, int fg, int bg);

	uint32_t screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);

	DECLARE_PALETTE_INIT(agat7);

	required_device<palette_device> m_palette;
};

// device type definition
DECLARE_DEVICE_TYPE(AGAT7VIDEO, agat7video_device)

#endif // MAME_VIDEO_AGAT7_H
