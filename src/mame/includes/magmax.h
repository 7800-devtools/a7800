// license:BSD-3-Clause
// copyright-holders:Takahiro Nogi
#include "screen.h"
#include "machine/gen_latch.h"

class magmax_state : public driver_device
{
public:
	magmax_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_videoram(*this, "videoram"),
		m_spriteram(*this, "spriteram"),
		m_vreg(*this, "vreg"),
		m_scroll_x(*this, "scroll_x"),
		m_scroll_y(*this, "scroll_y"),
		m_maincpu(*this, "maincpu"),
		m_audiocpu(*this, "audiocpu"),
		m_soundlatch(*this, "soundlatch"),
		m_gfxdecode(*this, "gfxdecode"),
		m_screen(*this, "screen"),
		m_palette(*this, "palette") { }

	required_shared_ptr<uint16_t> m_videoram;
	required_shared_ptr<uint16_t> m_spriteram;
	required_shared_ptr<uint16_t> m_vreg;
	required_shared_ptr<uint16_t> m_scroll_x;
	required_shared_ptr<uint16_t> m_scroll_y;

	uint8_t m_sound_latch;
	uint8_t m_LS74_clr;
	uint8_t m_LS74_q;
	uint8_t m_gain_control;
	emu_timer *m_interrupt_timer;
	int m_flipscreen;
	std::unique_ptr<uint32_t[]> m_prom_tab;
	bitmap_ind16 m_bitmap;
	DECLARE_WRITE16_MEMBER(cpu_irq_ack_w);
	DECLARE_READ8_MEMBER(magmax_sound_r);
	DECLARE_WRITE16_MEMBER(magmax_vreg_w);
	DECLARE_WRITE8_MEMBER(ay8910_portB_0_w);
	DECLARE_WRITE8_MEMBER(ay8910_portA_0_w);
	virtual void machine_start() override;
	virtual void machine_reset() override;
	virtual void video_start() override;
	DECLARE_PALETTE_INIT(magmax);
	uint32_t screen_update_magmax(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	TIMER_CALLBACK_MEMBER(scanline_callback);
	required_device<cpu_device> m_maincpu;
	required_device<cpu_device> m_audiocpu;
	required_device<generic_latch_8_device> m_soundlatch;
	required_device<gfxdecode_device> m_gfxdecode;
	required_device<screen_device> m_screen;
	required_device<palette_device> m_palette;
};
