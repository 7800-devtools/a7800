// license:BSD-3-Clause
// copyright-holders:Bryan McPhail
/*************************************************************************

    Irem M107 hardware

*************************************************************************/

#include "machine/gen_latch.h"
#include "machine/pic8259.h"
#include "screen.h"

struct pf_layer_info
{
	tilemap_t *     tmap;
	uint16_t          vram_base;
};

class m107_state : public driver_device
{
public:
	m107_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_soundcpu(*this, "soundcpu")
		, m_gfxdecode(*this, "gfxdecode")
		, m_screen(*this, "screen")
		, m_palette(*this, "palette")
		, m_upd71059c(*this, "upd71059c")
		, m_soundlatch(*this, "soundlatch")
		, m_spriteram(*this, "spriteram")
		, m_vram_data(*this, "vram_data")
		, m_user1_ptr(*this, "user1")
	{
	}

	required_device<cpu_device> m_maincpu;
	required_device<cpu_device> m_soundcpu;
	required_device<gfxdecode_device> m_gfxdecode;
	required_device<screen_device> m_screen;
	required_device<palette_device> m_palette;
	required_device<pic8259_device> m_upd71059c;
	required_device<generic_latch_8_device> m_soundlatch;

	required_shared_ptr<uint16_t> m_spriteram;
	required_shared_ptr<uint16_t> m_vram_data;
	optional_region_ptr<uint8_t> m_user1_ptr;

	// driver init
	uint8_t m_spritesystem;

	int m_sound_status;
	uint8_t m_sprite_display;
	uint16_t m_raster_irq_position;
	pf_layer_info m_pf_layer[4];
	uint16_t m_control[0x10];
	std::unique_ptr<uint16_t[]> m_buffered_spriteram;

	DECLARE_WRITE16_MEMBER(coincounter_w);
	DECLARE_WRITE16_MEMBER(bankswitch_w);
	DECLARE_WRITE16_MEMBER(soundlatch_w);
	DECLARE_READ16_MEMBER(sound_status_r);
	DECLARE_READ16_MEMBER(soundlatch_r);
	DECLARE_WRITE16_MEMBER(sound_irq_ack_w);
	DECLARE_WRITE16_MEMBER(sound_status_w);
	DECLARE_WRITE16_MEMBER(sound_reset_w);
	DECLARE_WRITE16_MEMBER(wpksoc_output_w);
	DECLARE_WRITE16_MEMBER(vram_w);
	DECLARE_WRITE16_MEMBER(control_w);
	DECLARE_WRITE16_MEMBER(spritebuffer_w);

	TILE_GET_INFO_MEMBER(get_pf_tile_info);

	TIMER_DEVICE_CALLBACK_MEMBER(scanline_interrupt);

	DECLARE_DRIVER_INIT(firebarr);
	DECLARE_DRIVER_INIT(dsoccr94);
	DECLARE_DRIVER_INIT(wpksoc);
	virtual void machine_start() override;
	virtual void video_start() override;

	uint32_t screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	void draw_sprites(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	void update_scroll_positions();
	void tilemap_draw(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect, int laynum, int category,int opaque);
	void screenrefresh(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
};
