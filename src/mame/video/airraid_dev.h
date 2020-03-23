// license:BSD-3-Clause
// copyright-holders:Tomasz Slanina, Angelo Salese, hap, David Haywood

#ifndef MAME_VIDEO_AIRRAID_DEV_H
#define MAME_VIDEO_AIRRAID_DEV_H

#pragma once

#include "screen.h"


DECLARE_DEVICE_TYPE(AIRRAID_VIDEO, airraid_video_device)

#define MCFG_AIRRAID_VIDEO_ADD(_tag) \
	MCFG_DEVICE_ADD(_tag, AIRRAID_VIDEO, 0)


class airraid_video_device :  public device_t
/*  public device_video_interface */
{
public:
	// construction/destruction
	airraid_video_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	DECLARE_WRITE8_MEMBER(txram_w);
	DECLARE_WRITE8_MEMBER(vregs_w);
	void layer_enable_w(uint8_t enable);

protected:
	virtual void device_add_mconfig(machine_config &config) override;
	virtual void device_start() override;
	virtual void device_reset() override;

private:
	// devices
	required_device<gfxdecode_device> m_gfxdecode;
	required_device<palette_device> m_palette;
	required_device<screen_device> m_screen;

	// region pointers
	required_region_ptr<uint8_t> m_tx_clut;
	required_region_ptr<uint8_t> m_fg_clut;
	required_region_ptr<uint8_t> m_bg_clut;
	required_region_ptr<uint8_t> m_spr_clut;
	required_region_ptr<uint8_t> m_fgmap;
	required_region_ptr<uint8_t> m_bgmap;

	// memory pointers
	required_shared_ptr<uint8_t> m_sprite_ram;
	required_shared_ptr<uint8_t> m_txram;
	required_shared_ptr<uint8_t> m_vregs;

	// tilemaps
	tilemap_t *m_bg_tilemap;
	tilemap_t *m_fg_tilemap;
	tilemap_t *m_tx_tilemap;

	TILEMAP_MAPPER_MEMBER(bg_scan);
	TILEMAP_MAPPER_MEMBER(fg_scan);

	TILE_GET_INFO_MEMBER(get_bg_tile_info);
	TILE_GET_INFO_MEMBER(get_fg_tile_info);
	TILE_GET_INFO_MEMBER(get_cstx_tile_info);

	// internal variables
	uint16_t m_hw;

	// rendering / mixing
	bitmap_ind16 m_temp_bitmap;
	void draw_sprites(bitmap_ind16 &bitmap, const rectangle &cliprect);
	void mix_layer(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect, uint8_t* clut, int base);

	uint32_t screen_update_airraid(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
};

#endif // MAME_VIDEO_AIRRAID_DEV_H
