// license:BSD-3-Clause
// copyright-holders:Quench, David Haywood
/* GP9001 Video Controller */
#ifndef MAME_VIDEO_GP9001_H
#define MAME_VIDEO_GP9001_H

#pragma once

#define MCFG_GP9001_VINT_CALLBACK(_devcb) \
	devcb = &gp9001vdp_device::set_vint_out_cb(*device, DEVCB_##_devcb);

class gp9001vdp_device : public device_t,
							public device_gfx_interface,
							public device_video_interface,
							public device_memory_interface
{
	enum
	{
		TIMER_RAISE_IRQ
	};

public:
	gp9001vdp_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	template<class Object> static devcb_base &set_vint_out_cb(device_t &device, Object &&obj) { return downcast<gp9001vdp_device &>(device).m_vint_out_cb.set_callback(std::forward<Object>(obj)); }

	void draw_sprites( bitmap_ind16 &bitmap, const rectangle &cliprect, const uint8_t* primap );
	void gp9001_draw_custom_tilemap( bitmap_ind16 &bitmap, tilemap_t* tilemap, const uint8_t* priremap, const uint8_t* pri_enable );
	void gp9001_render_vdp( bitmap_ind16 &bitmap, const rectangle &cliprect);
	void gp9001_screen_eof(void);
	void create_tilemaps(void);
	void init_scroll_regs(void);

	bitmap_ind8 *custom_priority_bitmap;

	DECLARE_ADDRESS_MAP(map, 16);

	// game-specific hack stuff
	void disable_sprite_buffer() { sp.use_sprite_buffer = 0; }
	void set_bg_extra_offsets(int xn, int yn, int xf, int yf) { bg.set_extra_offsets(xn, yn, xf, yf); }
	void set_fg_extra_offsets(int xn, int yn, int xf, int yf) { fg.set_extra_offsets(xn, yn, xf, yf); }
	void set_top_extra_offsets(int xn, int yn, int xf, int yf) { top.set_extra_offsets(xn, yn, xf, yf); }
	void set_sp_extra_offsets(int xn, int yn, int xf, int yf) { sp.set_extra_offsets(xn, yn, xf, yf); }

	// ROM banking control
	void set_gfxrom_banked() { gfxrom_is_banked = true; }
	void set_gfxrom_bank(unsigned index, uint16_t value)
	{
		if (gfxrom_bank[index] !=  value)
		{
			gfxrom_bank[index] = value;
			gfxrom_bank_dirty = true;
		}
	}

	// access to VDP
	DECLARE_READ16_MEMBER( gp9001_vdp_r );
	DECLARE_WRITE16_MEMBER( gp9001_vdp_w );
	DECLARE_READ16_MEMBER( gp9001_vdp_alt_r );
	DECLARE_WRITE16_MEMBER( gp9001_vdp_alt_w );

	// this bootleg has strange access
	DECLARE_READ16_MEMBER( pipibibi_bootleg_videoram16_r );
	DECLARE_WRITE16_MEMBER( pipibibi_bootleg_videoram16_w );
	DECLARE_READ16_MEMBER( pipibibi_bootleg_spriteram16_r );
	DECLARE_WRITE16_MEMBER( pipibibi_bootleg_spriteram16_w );
	DECLARE_WRITE16_MEMBER( pipibibi_bootleg_scroll_w );

	// internal handlers
	DECLARE_WRITE16_MEMBER( gp9001_bg_tmap_w );
	DECLARE_WRITE16_MEMBER( gp9001_fg_tmap_w );
	DECLARE_WRITE16_MEMBER( gp9001_top_tmap_w );

protected:
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

	virtual space_config_vector memory_space_config() const override;

	address_space_config        m_space_config;

	TILE_GET_INFO_MEMBER(get_top0_tile_info);
	TILE_GET_INFO_MEMBER(get_fg0_tile_info);
	TILE_GET_INFO_MEMBER(get_bg0_tile_info);

private:
	static constexpr unsigned BG_VRAM_SIZE   = 0x1000;   /* Background RAM size */
	static constexpr unsigned FG_VRAM_SIZE   = 0x1000;   /* Foreground RAM size */
	static constexpr unsigned TOP_VRAM_SIZE  = 0x1000;   /* Top Layer  RAM size */
	static constexpr unsigned SPRITERAM_SIZE = 0x0800;   /* Sprite     RAM size */
	static constexpr unsigned SPRITE_FLIPX   = 0x1000;   /* Sprite flip flags */
	static constexpr unsigned SPRITE_FLIPY   = 0x2000;

	struct gp9001layeroffsets
	{
		int normal;
		int flipped;
	};

	struct gp9001layer
	{
		void set_extra_offsets(int xn, int yn, int xf, int yf)
		{
			extra_xoffset.normal = xn;
			extra_yoffset.normal = yn;
			extra_xoffset.flipped = xf;
			extra_yoffset.flipped = yf;
		}

		uint16_t flip;
		uint16_t scrollx;
		uint16_t scrolly;

		gp9001layeroffsets extra_xoffset;
		gp9001layeroffsets extra_yoffset;
	};

	struct tilemaplayer : gp9001layer
	{
		void set_scrollx_and_flip_reg(uint16_t data, uint16_t mem_mask, bool f);
		void set_scrolly_and_flip_reg(uint16_t data, uint16_t mem_mask, bool f);

		tilemap_t *tmap;
	};

	struct spritelayer : gp9001layer
	{
		void set_scrollx_and_flip_reg(uint16_t data, uint16_t mem_mask, bool f);
		void set_scrolly_and_flip_reg(uint16_t data, uint16_t mem_mask, bool f);

		bool use_sprite_buffer;
		std::unique_ptr<uint16_t[]> vram16_buffer; // vram buffer for this layer
	};

	int get_tile_number(uint16_t const *vram, int index)
	{
		uint16_t const value = vram[(index << 1) | 1];
		if (gfxrom_is_banked)
			return (gfxrom_bank[(value >> 13) & 7] << 13) | (value & 0x1fff);
		else
			return value;
	}

	static const gfx_layout tilelayout, spritelayout;
	DECLARE_GFXDECODE_MEMBER(gfxinfo);

	void gp9001_voffs_w(uint16_t data, uint16_t mem_mask);
	int gp9001_videoram16_r(void);
	void gp9001_videoram16_w(uint16_t data, uint16_t mem_mask);
	uint16_t gp9001_vdpstatus_r(void);
	void gp9001_scroll_reg_select_w(uint16_t data, uint16_t mem_mask);
	void gp9001_scroll_reg_data_w(uint16_t data, uint16_t mem_mask);

	uint16_t gp9001_voffs;
	uint16_t gp9001_scroll_reg;

	tilemaplayer bg, top, fg;
	spritelayer sp;

	// technically this is just rom banking, allowing the chip to see more graphic ROM, however it's easier to handle it
	// in the chip implementation than externally for now (which would require dynamic decoding of the entire charsets every
	// time the bank was changed)
	bool gfxrom_is_banked;
	bool gfxrom_bank_dirty;       /* dirty flag of object bank (for Batrider) */
	uint16_t gfxrom_bank[8];       /* Batrider object bank */

	required_shared_ptr<uint16_t> m_vram_bg;
	required_shared_ptr<uint16_t> m_vram_fg;
	required_shared_ptr<uint16_t> m_vram_top;
	required_shared_ptr<uint16_t> m_spriteram;

	devcb_write_line m_vint_out_cb;
	emu_timer *m_raise_irq_timer;
};

DECLARE_DEVICE_TYPE(GP9001_VDP, gp9001vdp_device)

#endif // MAME_VIDEO_GP9001_H
