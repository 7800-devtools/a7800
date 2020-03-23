// license:BSD-3-Clause
// copyright-holders:Nicola Salmoria
#ifndef MAME_VIDEO_PC080SN_H
#define MAME_VIDEO_PC080SN_H

#pragma once

class pc080sn_device : public device_t
{
public:
	pc080sn_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// static configuration
	static void static_set_gfxdecode_tag(device_t &device, const char *tag);
	static void set_gfx_region(device_t &device, int gfxregion) { downcast<pc080sn_device &>(device).m_gfxnum = gfxregion; }
	static void set_yinvert(device_t &device, int y_inv) { downcast<pc080sn_device &>(device).m_y_invert = y_inv; }
	static void set_dblwidth(device_t &device, int dblwidth) { downcast<pc080sn_device &>(device).m_dblwidth = dblwidth; }
	static void set_offsets(device_t &device, int x_offset, int y_offset)
	{
		pc080sn_device &dev = downcast<pc080sn_device &>(device);
		dev.m_x_offset = x_offset;
		dev.m_y_offset = y_offset;
	}

	DECLARE_READ16_MEMBER( word_r );
	DECLARE_WRITE16_MEMBER( word_w );
	DECLARE_WRITE16_MEMBER( xscroll_word_w );
	DECLARE_WRITE16_MEMBER( yscroll_word_w );
	DECLARE_WRITE16_MEMBER( ctrl_word_w );

	TILE_GET_INFO_MEMBER(get_bg_tile_info);
	TILE_GET_INFO_MEMBER(get_fg_tile_info);

	void common_get_pc080sn_bg_tile_info( tile_data &tileinfo, int tile_index, uint16_t *ram, int gfxnum );
	void common_get_pc080sn_fg_tile_info( tile_data &tileinfo, int tile_index, uint16_t *ram, int gfxnum );

	void set_scroll(int tilemap_num, int scrollx, int scrolly);
	void set_trans_pen(int tilemap_num, int pen);
	void tilemap_update();
	void tilemap_draw(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect, int layer, int flags, uint32_t priority);
	void tilemap_draw_offset(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect, int layer, int flags, uint32_t priority, int xoffs, int yoffs);
	void topspeed_custom_draw(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect, int layer, int flags, uint32_t priority, uint16_t *color_ctrl_ram);

	/* For Topspeed */
	void tilemap_draw_special(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect, int layer, int flags, uint32_t priority, uint16_t *ram);

	void restore_scroll();

	protected:
	// device-level overrides
	virtual void device_start() override;

	private:
	// internal state
	uint16_t         m_ctrl[8];

	std::unique_ptr<uint16_t[]>         m_ram;
	uint16_t         *m_bg_ram[2];
	uint16_t         *m_bgscroll_ram[2];

	int            m_bgscrollx[2], m_bgscrolly[2];

	tilemap_t      *m_tilemap[2];

	int            m_gfxnum;
	int            m_x_offset, m_y_offset;
	int            m_y_invert;
	int            m_dblwidth;

	required_device<gfxdecode_device> m_gfxdecode;
};

DECLARE_DEVICE_TYPE(PC080SN, pc080sn_device)


#define MCFG_PC080SN_GFX_REGION(_region) \
	pc080sn_device::set_gfx_region(*device, _region);

#define MCFG_PC080SN_OFFSETS(_xoffs, _yoffs) \
	pc080sn_device::set_offsets(*device, _xoffs, _yoffs);

#define MCFG_PC080SN_YINVERT(_yinv) \
	pc080sn_device::set_yinvert(*device, _yinv);

#define MCFG_PC080SN_DBLWIDTH(_dbl) \
	pc080sn_device::set_dblwidth(*device, _dbl);

#define MCFG_PC080SN_GFXDECODE(_gfxtag) \
	pc080sn_device::static_set_gfxdecode_tag(*device, "^" _gfxtag);

#endif // MAME_VIDEO_PC080SN_H
