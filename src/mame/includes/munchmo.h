// license:BSD-3-Clause
// copyright-holders:Phil Stroffolino
/*************************************************************************

    Munch Mobile

*************************************************************************/

#include "machine/gen_latch.h"

class munchmo_state : public driver_device
{
public:
	munchmo_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_sprite_xpos(*this, "sprite_xpos"),
		m_sprite_tile(*this, "sprite_tile"),
		m_sprite_attr(*this, "sprite_attr"),
		m_videoram(*this, "videoram"),
		m_status_vram(*this, "status_vram"),
		m_vreg(*this, "vreg"),
		m_maincpu(*this, "maincpu"),
		m_audiocpu(*this, "audiocpu"),
		m_gfxdecode(*this, "gfxdecode"),
		m_palette(*this, "palette"),
		m_soundlatch(*this, "soundlatch") { }

	/* memory pointers */
	required_shared_ptr<u8> m_sprite_xpos;
	required_shared_ptr<u8> m_sprite_tile;
	required_shared_ptr<u8> m_sprite_attr;
	required_shared_ptr<u8> m_videoram;
	required_shared_ptr<u8> m_status_vram;
	required_shared_ptr<u8> m_vreg;

	/* video-related */
	std::unique_ptr<bitmap_ind16> m_tmpbitmap;
	int          m_palette_bank;
	int          m_flipscreen;

	/* misc */
	int          m_nmi_enable;

	/* devices */
	required_device<cpu_device> m_maincpu;
	required_device<cpu_device> m_audiocpu;
	required_device<gfxdecode_device> m_gfxdecode;
	required_device<palette_device> m_palette;
	required_device<generic_latch_8_device> m_soundlatch;

	DECLARE_WRITE8_MEMBER(nmi_enable_w);
	DECLARE_WRITE8_MEMBER(nmi_ack_w);
	DECLARE_WRITE8_MEMBER(sound_nmi_ack_w);
	DECLARE_WRITE8_MEMBER(palette_bank_w);
	DECLARE_WRITE8_MEMBER(flipscreen_w);
	DECLARE_READ8_MEMBER(ay1reset_r);
	DECLARE_READ8_MEMBER(ay2reset_r);
	virtual void machine_start() override;
	virtual void machine_reset() override;
	virtual void video_start() override;
	DECLARE_PALETTE_INIT(munchmo);
	u32 screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	DECLARE_WRITE_LINE_MEMBER(vblank_irq);
	IRQ_CALLBACK_MEMBER(generic_irq_ack);
	void draw_status( bitmap_ind16 &bitmap, const rectangle &cliprect );
	void draw_background( bitmap_ind16 &bitmap, const rectangle &cliprect );
	void draw_sprites( bitmap_ind16 &bitmap, const rectangle &cliprect );
};
