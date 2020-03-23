// license:BSD-3-Clause
// copyright-holders:David Haywood, Luca Elia
/*************************************************************************

    kan_pand.h

    Implementation of Kaneko Pandora sprite chip

**************************************************************************/

#ifndef MAME_VIDEO_KAN_PAND_H
#define MAME_VIDEO_KAN_PAND_H

#pragma once

/***************************************************************************
    TYPE DEFINITIONS
***************************************************************************/

class kaneko_pandora_device : public device_t,
								public device_video_interface
{
public:
	kaneko_pandora_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// static configuration
	static void static_set_gfxdecode_tag(device_t &device, const char *tag);
	static void set_gfx_region(device_t &device, int gfxregion) { downcast<kaneko_pandora_device &>(device).m_gfx_region = gfxregion; }
	static void set_offsets(device_t &device, int x_offset, int y_offset)
	{
		kaneko_pandora_device &dev = downcast<kaneko_pandora_device &>(device);
		dev.m_xoffset = x_offset;
		dev.m_yoffset = y_offset;
	}

	DECLARE_WRITE8_MEMBER ( spriteram_w );
	DECLARE_READ8_MEMBER( spriteram_r );
	DECLARE_WRITE16_MEMBER( spriteram_LSB_w );
	DECLARE_READ16_MEMBER( spriteram_LSB_r );
	void update( bitmap_ind16 &bitmap, const rectangle &cliprect );
	void set_clear_bitmap( int clear );
	void eof();
	void set_bg_pen( int pen );
	void flip_screen_set(bool flip) { m_flip_screen = flip; }

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	void draw( bitmap_ind16 &bitmap, const rectangle &cliprect );

private:
	// internal state
	std::unique_ptr<uint8_t[]>        m_spriteram;
	std::unique_ptr<bitmap_ind16> m_sprites_bitmap; /* bitmap to render sprites to, Pandora seems to be frame'buffered' */
	int             m_clear_bitmap;
	int             m_bg_pen; // might work some other way..
	uint8_t           m_gfx_region;
	int             m_xoffset;
	int             m_yoffset;
	bool            m_flip_screen;
	required_device<gfxdecode_device> m_gfxdecode;
};

DECLARE_DEVICE_TYPE(KANEKO_PANDORA, kaneko_pandora_device)


/***************************************************************************
    DEVICE CONFIGURATION MACROS
***************************************************************************/

#define MCFG_KANEKO_PANDORA_GFX_REGION(_region) \
	kaneko_pandora_device::set_gfx_region(*device, _region);

#define MCFG_KANEKO_PANDORA_OFFSETS(_xoffs, _yoffs) \
	kaneko_pandora_device::set_offsets(*device, _xoffs, _yoffs);

#define MCFG_KANEKO_PANDORA_GFXDECODE(_gfxtag) \
	kaneko_pandora_device::static_set_gfxdecode_tag(*device, "^" _gfxtag);

#endif // MAME_VIDEO_KAN_PAND_H
