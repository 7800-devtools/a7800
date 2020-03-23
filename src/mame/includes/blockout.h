// license:BSD-3-Clause
// copyright-holders:Nicola Salmoria

/***************************************************************************

    Blockout

***************************************************************************/

#include "machine/gen_latch.h"
#include "screen.h"

class blockout_state : public driver_device
{
public:
	blockout_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_videoram(*this, "videoram"),
		m_frontvideoram(*this, "frontvideoram"),
		m_paletteram(*this, "paletteram"),
		m_maincpu(*this, "maincpu"),
		m_audiocpu(*this, "audiocpu"),
		m_screen(*this, "screen"),
		m_palette(*this, "palette"),
		m_soundlatch(*this, "soundlatch") { }

	/* memory pointers */
	required_shared_ptr<uint16_t> m_videoram;
	required_shared_ptr<uint16_t> m_frontvideoram;
	required_shared_ptr<uint16_t> m_paletteram;

	/* video-related */
	bitmap_ind16 m_tmpbitmap;
	uint16_t   m_color;

	/* devices */
	required_device<cpu_device> m_maincpu;
	required_device<cpu_device> m_audiocpu;
	required_device<screen_device> m_screen;
	required_device<palette_device> m_palette;
	required_device<generic_latch_8_device> m_soundlatch;

	DECLARE_WRITE_LINE_MEMBER(irq_handler);
	DECLARE_WRITE16_MEMBER(blockout_irq6_ack_w);
	DECLARE_WRITE16_MEMBER(blockout_irq5_ack_w);
	DECLARE_WRITE16_MEMBER(blockout_paletteram_w);
	DECLARE_WRITE16_MEMBER(blockout_frontcolor_w);
	DECLARE_WRITE16_MEMBER(blockout_videoram_w);
	virtual void machine_start() override;
	virtual void machine_reset() override;
	virtual void video_start() override;
	uint32_t screen_update_blockout(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	TIMER_DEVICE_CALLBACK_MEMBER(blockout_scanline);
	void setcolor( int color, int rgb );
	void update_pixels( int x, int y );
};
