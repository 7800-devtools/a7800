// license:BSD-3-Clause
// copyright-holders:Luca Elia,David Haywood
#ifndef MAME_VIDEO_ST0020_H
#define MAME_VIDEO_ST0020_H

#pragma once


class st0020_device : public device_t, public device_gfx_interface
{
public:
	st0020_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// static configuration
	static void static_set_is_st0032(device_t &device, int is_st0032);
	static void static_set_is_jclub2(device_t &device, int is_jclub2);

	void update_screen(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect, bool update_visible_area);

	DECLARE_READ16_MEMBER(gfxram_r);
	DECLARE_WRITE16_MEMBER(gfxram_w);

	DECLARE_READ16_MEMBER(regs_r);
	DECLARE_WRITE16_MEMBER(regs_w);

	DECLARE_READ16_MEMBER(sprram_r);
	DECLARE_WRITE16_MEMBER(sprram_w);

protected:
	virtual void device_start() override;
	virtual void device_reset() override;

private:
	// see if we can handle the difference between this and the st0032 in here, or if we need another device
	int m_is_st0032;

	// per-game hack
	int m_is_jclub2;

	// RAM
	std::unique_ptr<uint16_t[]> m_gfxram;
	std::unique_ptr<uint16_t[]> m_spriteram;
	std::unique_ptr<uint16_t[]> m_regs;

	DECLARE_WRITE16_MEMBER(regs_st0020_w);
	DECLARE_WRITE16_MEMBER(regs_st0032_w);

	int m_gfxram_bank;
	DECLARE_WRITE16_MEMBER(gfxram_bank_w);

	// blitter
	uint8_t *m_rom_ptr;
	size_t m_rom_size;
	DECLARE_WRITE16_MEMBER(do_blit_w);

	// tilemaps
	tilemap_t *m_tmap[4];
	void get_tile_info_i(int i, tilemap_t &tilemap, tile_data &tileinfo, tilemap_memory_index tile_index);

	TILE_GET_INFO_MEMBER(get_tile_info_0);
	TILE_GET_INFO_MEMBER(get_tile_info_1);
	TILE_GET_INFO_MEMBER(get_tile_info_2);
	TILE_GET_INFO_MEMBER(get_tile_info_3);

	int tmap_offset(int i);
	int tmap_priority(int i);
	int tmap_is_enabled(int i);
	DECLARE_WRITE16_MEMBER(tmap_st0020_w);
	DECLARE_WRITE16_MEMBER(tmap_st0032_w);

	// sprites
	void draw_zooming_sprites(bitmap_ind16 &bitmap, const rectangle &cliprect, int priority);
};

DECLARE_DEVICE_TYPE(ST0020_SPRITES, st0020_device)

#define MCFG_ST0020_SPRITES_PALETTE(_palette_tag) \
	MCFG_GFX_PALETTE(_palette_tag)

#endif // MAME_VIDEO_ST0020_H
