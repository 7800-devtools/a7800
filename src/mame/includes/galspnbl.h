// license:BSD-3-Clause
// copyright-holders:Nicola Salmoria

/*************************************************************************

    Hot Pinball
    Gals Pinball

*************************************************************************/

#include "machine/gen_latch.h"
#include "video/tecmo_spr.h"
#include "screen.h"

class galspnbl_state : public driver_device
{
public:
	galspnbl_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_spriteram(*this, "spriteram"),
		m_colorram(*this, "colorram"),
		m_videoram(*this, "videoram"),
		m_bgvideoram(*this, "bgvideoram"),
		m_scroll(*this, "scroll"),
		m_audiocpu(*this, "audiocpu"),
		m_maincpu(*this, "maincpu"),
		m_gfxdecode(*this, "gfxdecode"),
		m_palette(*this, "palette"),
		m_sprgen(*this, "spritegen"),
		m_screen(*this, "screen"),
		m_soundlatch(*this, "soundlatch")
		{ }

	/* memory pointers */
	required_shared_ptr<uint16_t> m_spriteram;
	required_shared_ptr<uint16_t> m_colorram;
	required_shared_ptr<uint16_t> m_videoram;
	required_shared_ptr<uint16_t> m_bgvideoram;
	required_shared_ptr<uint16_t> m_scroll;

	/* devices */
	required_device<cpu_device> m_audiocpu;
	required_device<cpu_device> m_maincpu;
	required_device<gfxdecode_device> m_gfxdecode;
	required_device<palette_device> m_palette;
	required_device<tecmo_spr_device> m_sprgen;
	required_device<screen_device> m_screen;
	required_device<generic_latch_8_device> m_soundlatch;

	DECLARE_WRITE16_MEMBER(soundcommand_w);
	virtual void machine_start() override;
	DECLARE_PALETTE_INIT(galspnbl);
	uint32_t screen_update_galspnbl(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	void draw_background( bitmap_ind16 &bitmap, const rectangle &cliprect );
	bitmap_ind16 m_sprite_bitmap;
	DECLARE_VIDEO_START(galspnbl);

	void mix_sprite_layer(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect, int pri);

};
