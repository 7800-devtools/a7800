// license:BSD-3-Clause
// copyright-holders:Phil Stroffolino, Stefan Jokisch
/*************************************************************************

    Atari Ultra Tank hardware

*************************************************************************/

#include "machine/watchdog.h"
#include "sound/discrete.h"
#include "screen.h"

class ultratnk_state : public driver_device
{
public:
	enum
	{
		TIMER_NMI
	};

	ultratnk_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_watchdog(*this, "watchdog"),
		m_discrete(*this, "discrete"),
		m_gfxdecode(*this, "gfxdecode"),
		m_screen(*this, "screen"),
		m_palette(*this, "palette"),
		m_videoram(*this, "videoram") { }

	required_device<cpu_device> m_maincpu;
	required_device<watchdog_timer_device> m_watchdog;
	required_device<discrete_device> m_discrete;
	required_device<gfxdecode_device> m_gfxdecode;
	required_device<screen_device> m_screen;
	required_device<palette_device> m_palette;

	required_shared_ptr<uint8_t> m_videoram;

	int m_da_latch;
	int m_collision[4];
	tilemap_t* m_playfield;
	bitmap_ind16 m_helper;
	emu_timer *m_nmi_timer;

	DECLARE_READ8_MEMBER(wram_r);
	DECLARE_READ8_MEMBER(analog_r);
	DECLARE_READ8_MEMBER(coin_r);
	DECLARE_READ8_MEMBER(collision_r);
	DECLARE_READ8_MEMBER(options_r);
	DECLARE_WRITE8_MEMBER(wram_w);
	DECLARE_WRITE8_MEMBER(collision_reset_w);
	DECLARE_WRITE8_MEMBER(da_latch_w);
	DECLARE_WRITE8_MEMBER(led_1_w);
	DECLARE_WRITE8_MEMBER(led_2_w);
	DECLARE_WRITE8_MEMBER(lockout_w);
	DECLARE_WRITE8_MEMBER(video_ram_w);
	DECLARE_CUSTOM_INPUT_MEMBER(get_collision);
	DECLARE_CUSTOM_INPUT_MEMBER(get_joystick);
	DECLARE_WRITE8_MEMBER(fire_1_w);
	DECLARE_WRITE8_MEMBER(fire_2_w);
	DECLARE_WRITE8_MEMBER(attract_w);
	DECLARE_WRITE8_MEMBER(explosion_w);

	virtual void machine_start() override;
	virtual void machine_reset() override;
	virtual void video_start() override;
	DECLARE_PALETTE_INIT(ultratnk);

	TILE_GET_INFO_MEMBER(tile_info);
	uint32_t screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	DECLARE_WRITE_LINE_MEMBER(screen_vblank);
	TIMER_CALLBACK_MEMBER(nmi_callback);

protected:
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;
};
