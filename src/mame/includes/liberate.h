// license:BSD-3-Clause
// copyright-holders:Bryan McPhail

#include "machine/gen_latch.h"

class liberate_state : public driver_device
{
public:
	liberate_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_bg_vram(*this, "bg_vram"),
		m_colorram(*this, "colorram"),
		m_videoram(*this, "videoram"),
		m_spriteram(*this, "spriteram"),
		m_scratchram(*this, "scratchram"),
		m_decrypted_opcodes(*this, "decrypted_opcodes"),
		m_maincpu(*this, "maincpu"),
		m_audiocpu(*this, "audiocpu"),
		m_gfxdecode(*this, "gfxdecode"),
		m_palette(*this, "palette"),
		m_soundlatch(*this, "soundlatch") { }

	optional_shared_ptr<uint8_t> m_bg_vram; /* prosport */
	required_shared_ptr<uint8_t> m_colorram;
	required_shared_ptr<uint8_t> m_videoram;
	required_shared_ptr<uint8_t> m_spriteram;
	optional_shared_ptr<uint8_t> m_scratchram;
	optional_shared_ptr<uint8_t> m_decrypted_opcodes;

	uint8_t *m_fg_gfx;   /* prosoccr */
	std::unique_ptr<uint8_t[]> m_charram;   /* prosoccr */
	uint8_t m_io_ram[16];

	int m_bank;
	int m_latch;
	uint8_t m_gfx_rom_readback;
	int m_background_color;
	int m_background_disable;

	tilemap_t *m_back_tilemap;
	tilemap_t *m_fix_tilemap;

	required_device<cpu_device> m_maincpu;
	required_device<cpu_device> m_audiocpu;
	required_device<gfxdecode_device> m_gfxdecode;
	required_device<palette_device> m_palette;
	required_device<generic_latch_8_device> m_soundlatch;

	DECLARE_READ8_MEMBER(deco16_bank_r);
	DECLARE_READ8_MEMBER(deco16_io_r);
	DECLARE_WRITE8_MEMBER(deco16_bank_w);
	DECLARE_READ8_MEMBER(prosoccr_bank_r);
	DECLARE_READ8_MEMBER(prosoccr_charram_r);
	DECLARE_WRITE8_MEMBER(prosoccr_charram_w);
	DECLARE_WRITE8_MEMBER(prosoccr_char_bank_w);
	DECLARE_WRITE8_MEMBER(prosoccr_io_bank_w);
	DECLARE_READ8_MEMBER(prosport_charram_r);
	DECLARE_WRITE8_MEMBER(prosport_charram_w);
	DECLARE_WRITE8_MEMBER(deco16_io_w);
	DECLARE_WRITE8_MEMBER(prosoccr_io_w);
	DECLARE_WRITE8_MEMBER(prosport_io_w);
	DECLARE_WRITE8_MEMBER(liberate_videoram_w);
	DECLARE_WRITE8_MEMBER(liberate_colorram_w);
	DECLARE_WRITE8_MEMBER(prosport_bg_vram_w);
	DECLARE_DRIVER_INIT(yellowcb);
	DECLARE_DRIVER_INIT(liberate);
	DECLARE_DRIVER_INIT(prosport);
	TILEMAP_MAPPER_MEMBER(back_scan);
	TILEMAP_MAPPER_MEMBER(fix_scan);
	TILE_GET_INFO_MEMBER(get_back_tile_info);
	TILE_GET_INFO_MEMBER(get_fix_tile_info);
	TILE_GET_INFO_MEMBER(prosport_get_back_tile_info);
	DECLARE_MACHINE_START(liberate);
	DECLARE_MACHINE_RESET(liberate);
	DECLARE_VIDEO_START(liberate);
	DECLARE_PALETTE_INIT(liberate);
	DECLARE_VIDEO_START(prosport);
	DECLARE_VIDEO_START(boomrang);
	DECLARE_VIDEO_START(prosoccr);
	uint32_t screen_update_liberate(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	uint32_t screen_update_prosport(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	uint32_t screen_update_boomrang(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	uint32_t screen_update_prosoccr(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	INTERRUPT_GEN_MEMBER(deco16_interrupt);
	void liberate_draw_sprites( bitmap_ind16 &bitmap, const rectangle &cliprect );
	void prosport_draw_sprites( bitmap_ind16 &bitmap, const rectangle &cliprect );
	void boomrang_draw_sprites( bitmap_ind16 &bitmap, const rectangle &cliprect, int pri );
	void prosoccr_draw_sprites( bitmap_ind16 &bitmap, const rectangle &cliprect );
};
