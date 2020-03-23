// license:BSD-3-Clause
// copyright-holders:Luca Elia
/*************************************************************************

                      -= IGS Lord Of Gun =-

*************************************************************************/
#include "sound/okim6295.h"
#include "machine/eepromser.h"
#include "machine/gen_latch.h"
#include "screen.h"

struct lordgun_gun_data
{
	int     scr_x,  scr_y;
	uint16_t  hw_x,   hw_y;
};

class lordgun_state : public driver_device
{
public:
	lordgun_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_soundcpu(*this, "soundcpu"),
		m_oki(*this, "oki"),
		m_eeprom(*this, "eeprom"),
		m_gfxdecode(*this, "gfxdecode"),
		m_screen(*this, "screen"),
		m_palette(*this, "palette"),
		m_soundlatch(*this, "soundlatch"),
		m_soundlatch2(*this, "soundlatch2"),
		m_generic_paletteram_16(*this, "paletteram"),
		m_priority_ram(*this, "priority_ram"),
		m_scrollram(*this, "scrollram"),
		m_spriteram(*this, "spriteram"),
		m_vram(*this, "vram.%u", 0),
		m_scroll_x(*this, "scroll_x.%u", 0),
		m_scroll_y(*this, "scroll_y.%u", 0) { }

	required_device<cpu_device> m_maincpu;
	required_device<cpu_device> m_soundcpu;
	required_device<okim6295_device> m_oki;
	required_device<eeprom_serial_93cxx_device> m_eeprom;
	required_device<gfxdecode_device> m_gfxdecode;
	required_device<screen_device> m_screen;
	required_device<palette_device> m_palette;
	required_device<generic_latch_8_device> m_soundlatch;
	required_device<generic_latch_8_device> m_soundlatch2;

	required_shared_ptr<uint16_t> m_generic_paletteram_16;
	required_shared_ptr<uint16_t> m_priority_ram;
	required_shared_ptr<uint16_t> m_scrollram;
	required_shared_ptr<uint16_t> m_spriteram;
	required_shared_ptr_array<uint16_t, 4> m_vram;
	required_shared_ptr_array<uint16_t, 4> m_scroll_x;
	required_shared_ptr_array<uint16_t, 4> m_scroll_y;

	uint8_t m_old;
	uint8_t m_aliencha_dip_sel;
	uint16_t m_priority;
	int m_whitescreen;
	lordgun_gun_data m_gun[2];
	tilemap_t *m_tilemap[4];
	std::unique_ptr<bitmap_ind16> m_bitmaps[5];

	uint16_t m_protection_data;
	DECLARE_WRITE16_MEMBER(lordgun_protection_w);
	DECLARE_READ16_MEMBER(lordgun_protection_r);
	DECLARE_WRITE16_MEMBER(aliencha_protection_w);
	DECLARE_READ16_MEMBER(aliencha_protection_r);

	DECLARE_WRITE16_MEMBER(lordgun_priority_w);
	DECLARE_READ16_MEMBER(lordgun_gun_0_x_r);
	DECLARE_READ16_MEMBER(lordgun_gun_0_y_r);
	DECLARE_READ16_MEMBER(lordgun_gun_1_x_r);
	DECLARE_READ16_MEMBER(lordgun_gun_1_y_r);
	DECLARE_WRITE16_MEMBER(lordgun_soundlatch_w);
	DECLARE_WRITE16_MEMBER(lordgun_paletteram_w);
	DECLARE_WRITE16_MEMBER(lordgun_vram_0_w);
	DECLARE_WRITE16_MEMBER(lordgun_vram_1_w);
	DECLARE_WRITE16_MEMBER(lordgun_vram_2_w);
	DECLARE_WRITE16_MEMBER(lordgun_vram_3_w);
	DECLARE_WRITE8_MEMBER(fake_w);
	DECLARE_WRITE8_MEMBER(fake2_w);
	DECLARE_WRITE8_MEMBER(lordgun_eeprom_w);
	DECLARE_WRITE8_MEMBER(aliencha_eeprom_w);
	DECLARE_READ8_MEMBER(aliencha_dip_r);
	DECLARE_WRITE8_MEMBER(aliencha_dip_w);
	DECLARE_WRITE8_MEMBER(lordgun_okibank_w);

	DECLARE_DRIVER_INIT(aliencha);
	DECLARE_DRIVER_INIT(lordgun);

	TILE_GET_INFO_MEMBER(get_tile_info_0);
	TILE_GET_INFO_MEMBER(get_tile_info_1);
	TILE_GET_INFO_MEMBER(get_tile_info_2);
	TILE_GET_INFO_MEMBER(get_tile_info_3);

	virtual void machine_start() override;
	virtual void video_start() override;

	uint32_t screen_update_lordgun(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	inline void get_tile_info(tile_data &tileinfo, tilemap_memory_index tile_index, int _N_);
	inline void lordgun_vram_w(offs_t offset, uint16_t data, uint16_t mem_mask, int _N_);
	void lorddgun_calc_gun_scr(int i);
	void lordgun_update_gun(int i);
	void draw_sprites(bitmap_ind16 &bitmap, const rectangle &cliprect);
};

/*----------- defined in video/lordgun.c -----------*/
float lordgun_crosshair_mapper(const ioport_field *field, float linear_value);
