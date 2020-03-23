// license:BSD-3-Clause
// copyright-holders:Luca Elia
/***************************************************************************

    ms1_tmap.h

    8x8/16x16 tilemap generator for Jaleco's Mega System 1 and driving
    and mahjong games from the same period.

***************************************************************************/
#ifndef MAME_VIDEO_MS1_TMAP_H
#define MAME_VIDEO_MS1_TMAP_H

#pragma once


//**************************************************************************
//  DEVICE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_MEGASYS1_TILEMAP_ADD(tag, palette_tag, colorbase) \
	MCFG_DEVICE_ADD(tag, MEGASYS1_TILEMAP, 0) \
	MCFG_GFX_PALETTE(palette_tag) \
	MCFG_MEGASYS1_TILEMAP_COLORBASE(colorbase)

#define MCFG_MEGASYS1_TILEMAP_8X8_SCROLL_FACTOR(scroll_factor) \
	megasys1_tilemap_device::static_set_8x8_scroll_factor(*device, scroll_factor);

#define MCFG_MEGASYS1_TILEMAP_16X16_SCROLL_FACTOR(scroll_factor) \
	megasys1_tilemap_device::static_set_16x16_scroll_factor(*device, scroll_factor);

#define MCFG_MEGASYS1_TILEMAP_BITS_PER_COLOR_CODE(bits) \
	megasys1_tilemap_device::static_set_bits_per_color_code(*device, bits);

#define MCFG_MEGASYS1_TILEMAP_COLORBASE(colorbase) \
	megasys1_tilemap_device::static_set_colorbase(*device, colorbase);

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> megasys1_tilemap_device

class megasys1_tilemap_device : public device_t, public device_gfx_interface
{
public:
	// construction/destruction
	megasys1_tilemap_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// static configuration
	static void static_set_8x8_scroll_factor(device_t &device, int scroll_factor);
	static void static_set_16x16_scroll_factor(device_t &device, int scroll_factor);
	static void static_set_bits_per_color_code(device_t &device, int bits);
	static void static_set_colorbase(device_t &device, uint16_t colorbase);

	// memory handlers
	DECLARE_WRITE16_MEMBER(write);
	DECLARE_READ16_MEMBER(scroll_r);
	DECLARE_WRITE16_MEMBER(scroll_w);

	// drawing and layer control
	void draw(screen_device &screen, bitmap_ind16 &dest, const rectangle &cliprect, uint32_t flags, uint8_t priority = 0, uint8_t priority_mask = 0xff);
	void enable(bool enable);
	void set_flip(uint32_t attributes);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_post_load() override;

private:
	// shared memory finder
	required_shared_ptr<uint16_t> m_scrollram;

	// configuration
	int m_8x8_scroll_factor;
	int m_16x16_scroll_factor;
	int m_bits_per_color_code;
	uint16_t m_colorbase;

	// decoding info
	DECLARE_GFXDECODE_MEMBER(gfxinfo);

	// internal state
	uint16_t m_scrollx;
	uint16_t m_scrolly;
	uint16_t m_scroll_flag;
	tilemap_t *m_tmap;
	tilemap_t *m_tilemap[2][4];

	// helpers
	TILEMAP_MAPPER_MEMBER(scan_8x8);
	TILEMAP_MAPPER_MEMBER(scan_16x16);
	TILE_GET_INFO_MEMBER(get_scroll_tile_info_8x8);
	TILE_GET_INFO_MEMBER(get_scroll_tile_info_16x16);
};

// device type definition
DECLARE_DEVICE_TYPE(MEGASYS1_TILEMAP, megasys1_tilemap_device)

#endif  // MAME_VIDEO_MS1_TMAP_H
