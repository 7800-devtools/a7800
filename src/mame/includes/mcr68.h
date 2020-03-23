// license:BSD-3-Clause
// copyright-holders:Aaron Giles, Bryan McPhail
/***************************************************************************

    Midway MCR-68k system

***************************************************************************/

#include "machine/watchdog.h"
#include "audio/midway.h"
#include "audio/williams.h"
#include "machine/6840ptm.h"
#include "screen.h"

class mcr68_state : public driver_device
{
public:
	mcr68_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_sounds_good(*this, "sg"),
		m_turbo_cheap_squeak(*this, "tcs"),
		m_cvsd_sound(*this, "cvsd"),
		m_videoram(*this, "videoram"),
		m_spriteram(*this, "spriteram") ,
		m_maincpu(*this, "maincpu"),
		m_gfxdecode(*this, "gfxdecode"),
		m_screen(*this, "screen"),
		m_ptm(*this, "ptm")
	{ }

	optional_device<midway_sounds_good_device> m_sounds_good;
	optional_device<midway_turbo_cheap_squeak_device> m_turbo_cheap_squeak;
	optional_device<williams_cvsd_sound_device> m_cvsd_sound;

	required_shared_ptr<uint16_t> m_videoram;
	required_shared_ptr<uint16_t> m_spriteram;
	uint16_t m_control_word;
	uint8_t m_protection_data[5];
	attotime m_timing_factor;
	uint8_t m_sprite_clip;
	int8_t m_sprite_xoffset;
	timer_expired_delegate m_v493_callback;
	tilemap_t *m_bg_tilemap;
	tilemap_t *m_fg_tilemap;
	DECLARE_WRITE16_MEMBER(xenophobe_control_w);
	DECLARE_WRITE16_MEMBER(blasted_control_w);
	DECLARE_READ16_MEMBER(spyhunt2_port_0_r);
	DECLARE_READ16_MEMBER(spyhunt2_port_1_r);
	DECLARE_WRITE16_MEMBER(spyhunt2_control_w);
	DECLARE_READ16_MEMBER(archrivl_port_1_r);
	DECLARE_WRITE16_MEMBER(archrivl_control_w);
	DECLARE_WRITE16_MEMBER(pigskin_protection_w);
	DECLARE_READ16_MEMBER(pigskin_protection_r);
	DECLARE_READ16_MEMBER(pigskin_port_1_r);
	DECLARE_READ16_MEMBER(pigskin_port_2_r);
	DECLARE_READ16_MEMBER(trisport_port_1_r);
	DECLARE_WRITE16_MEMBER(mcr68_videoram_w);
	DECLARE_DRIVER_INIT(intlaser);
	DECLARE_DRIVER_INIT(pigskin);
	DECLARE_DRIVER_INIT(blasted);
	DECLARE_DRIVER_INIT(trisport);
	DECLARE_DRIVER_INIT(xenophob);
	DECLARE_DRIVER_INIT(archrivl);
	DECLARE_DRIVER_INIT(spyhunt2);
	DECLARE_DRIVER_INIT(archrivlb);
	DECLARE_READ16_MEMBER(archrivlb_port_1_r);
	TILE_GET_INFO_MEMBER(get_bg_tile_info);
	DECLARE_MACHINE_START(mcr68);
	DECLARE_MACHINE_RESET(mcr68);
	DECLARE_VIDEO_START(mcr68);
	uint32_t screen_update_mcr68(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	TIMER_DEVICE_CALLBACK_MEMBER(scanline_cb);
	TIMER_CALLBACK_MEMBER(mcr68_493_off_callback);
	TIMER_CALLBACK_MEMBER(mcr68_493_callback);
	void mcr68_update_sprites(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect, int priority);
	void mcr68_common_init(int clip, int xoffset);
	required_device<cpu_device> m_maincpu;
	required_device<gfxdecode_device> m_gfxdecode;
	required_device<screen_device> m_screen;
	std::unique_ptr<uint8_t[]> m_srcdata0;
	std::unique_ptr<uint8_t[]> m_srcdata2;

private:
	required_device<ptm6840_device> m_ptm;
};
