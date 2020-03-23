// license:BSD-3-Clause
// copyright-holders:David Haywood
#ifndef MAME_VIDEO_VS920A_H
#define MAME_VIDEO_VS920A_H

#pragma once


class vs920a_text_tilemap_device : public device_t
{
public:
	static void static_set_gfxdecode_tag(device_t &device, const char *tag);
	static void set_gfx_region(device_t &device, int gfxregion);

	vs920a_text_tilemap_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	void set_pal_base(int pal_base) { m_pal_base = pal_base; }
	void set_transparent_pen(pen_t pen) { m_tmap->set_transparent_pen(pen); }
	void draw(screen_device &screen, bitmap_ind16& bitmap, const rectangle &cliprect, int priority);

	DECLARE_WRITE16_MEMBER(vram_w);
	DECLARE_READ16_MEMBER(vram_r);

protected:
	virtual void device_start() override;
	virtual void device_reset() override;

private:
	TILE_GET_INFO_MEMBER(get_tile_info);

	tilemap_t* m_tmap;
	std::unique_ptr<uint16_t[]> m_vram;
	uint16_t m_pal_base;

	uint8_t m_gfx_region;

	required_device<gfxdecode_device> m_gfxdecode;
};

DECLARE_DEVICE_TYPE(VS920A, vs920a_text_tilemap_device)


#define MCFG_VS920A_GFX_REGION(_region) \
	vs920a_text_tilemap_device::set_gfx_region(*device, _region);

#define MCFG_VS920A_GFXDECODE(_gfxtag) \
	vs920a_text_tilemap_device::static_set_gfxdecode_tag(*device, "^" _gfxtag);

#endif // MAME_VIDEO_VS920A_H
