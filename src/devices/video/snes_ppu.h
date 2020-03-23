// license:BSD-3-Clause
// copyright-holders:Anthony Kruize, Fabio Priuli
/***************************************************************************

        SNES PPU

***************************************************************************/

#ifndef MAME_VIDEO_SNES_PPU_H
#define MAME_VIDEO_SNES_PPU_H

#pragma once

#include "screen.h"


#define MCLK_NTSC   (21477272)  /* verified */
#define MCLK_PAL    (21218370)  /* verified */

#define DOTCLK_NTSC (MCLK_NTSC/4)
#define DOTCLK_PAL  (MCLK_PAL/4)

#define SNES_SCR_WIDTH        256       /* 32 characters 8 pixels wide */
#define SNES_SCR_HEIGHT_NTSC  225       /* Can be 224 or 240 height */
#define SNES_SCR_HEIGHT_PAL   240       /* ??? */
#define SNES_VTOTAL_NTSC      262       /* Maximum number of lines for NTSC systems */
#define SNES_VTOTAL_PAL       312       /* Maximum number of lines for PAL systems */
#define SNES_HTOTAL           341       /* Maximum number pixels per line (incl. hblank) */

#define SNES_NTSC             0x00
#define SNES_PAL              0x10


#define SNES_LAYER_DEBUG  0


// ======================> snes_ppu_device

class snes_ppu_device :  public device_t,
							public device_video_interface
{
public:
	// construction/destruction
	snes_ppu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// inline configuration helpers
	template <class Object> static devcb_base &static_set_open_bus_callback(device_t &device, Object &&cb) { return downcast<snes_ppu_device &>(device).m_openbus_cb.set_callback(std::forward<Object>(cb)); }

	void refresh_scanline(bitmap_rgb32 &bitmap, uint16_t curline);

	int16_t current_x() const { return m_screen->hpos() / m_htmult; }
	int16_t current_y() const { return m_screen->vpos(); }
	void set_latch_hv(int16_t x, int16_t y);

	uint8_t read(address_space &space, uint32_t offset, uint8_t wrio_bit7);
	void write(address_space &space, uint32_t offset, uint8_t data);

	int vtotal() const { return ((m_stat78 & 0x10) == SNES_NTSC) ? SNES_VTOTAL_NTSC : SNES_VTOTAL_PAL; }
	uint16_t htmult() const { return m_htmult; }
	uint8_t interlace() const { return m_interlace; }
	bool screen_disabled() const { return bool(m_screen_disabled); }
	uint8_t last_visible_line() const { return m_beam.last_visible_line; }
	uint16_t current_vert() const { return m_beam.current_vert; }
	uint8_t saved_oam_address_low() const { return m_oam.saved_address_low; }
	uint8_t saved_oam_address_high() const { return m_oam.saved_address_high; }

	void clear_time_range_over() { m_stat77 &= 0x3f; }
	void toggle_field() { m_stat78 ^= 0x80; }
	void reset_interlace()
	{
		m_htmult = 1;
		m_interlace = 1;
		m_obj_interlace = 1;
	}
	void set_current_vert(uint16_t value) { m_beam.current_vert = value; }
	void set_first_sprite() { m_oam.first_sprite = m_oam.priority_rotation ? ((m_oam.address >> 1) & 127) : 0; }

protected:
	/* offset-per-tile modes */
	enum
	{
		SNES_OPT_NONE = 0,
		SNES_OPT_MODE2,
		SNES_OPT_MODE4,
		SNES_OPT_MODE6
	};

	/* layers */
	enum
	{
		SNES_BG1 = 0,
		SNES_BG2,
		SNES_BG3,
		SNES_BG4,
		SNES_OAM,
		SNES_COLOR
	};


	struct SNES_SCANLINE
	{
		int enable, clip;

		uint16_t buffer[SNES_SCR_WIDTH];
		uint8_t  priority[SNES_SCR_WIDTH];
		uint8_t  layer[SNES_SCR_WIDTH];
		uint8_t  blend_exception[SNES_SCR_WIDTH];
	};

	uint8_t m_regs[0x40];

	SNES_SCANLINE m_scanlines[2];

	struct
	{
		/* clipmasks */
		uint8_t window1_enabled, window1_invert;
		uint8_t window2_enabled, window2_invert;
		uint8_t wlog_mask;
		/* color math enabled */
		uint8_t color_math;

		uint8_t charmap;
		uint8_t tilemap;
		uint8_t tilemap_size;

		uint8_t tile_size;
		uint8_t mosaic_enabled;   // actually used only for layers 0->3!

		uint8_t main_window_enabled;
		uint8_t sub_window_enabled;
		uint8_t main_bg_enabled;
		uint8_t sub_bg_enabled;

		uint16_t hoffs;
		uint16_t voffs;
	} m_layer[6]; // this is for the BG1 - BG2 - BG3 - BG4 - OBJ - color layers

	struct
	{
		uint8_t address_low;
		uint8_t address_high;
		uint8_t saved_address_low;
		uint8_t saved_address_high;
		uint16_t address;
		uint16_t priority_rotation;
		uint8_t next_charmap;
		uint8_t next_size;
		uint8_t size;
		uint32_t next_name_select;
		uint32_t name_select;
		uint8_t first_sprite;
		uint8_t flip;
		uint16_t write_latch;
	} m_oam;

	struct
	{
		uint16_t latch_horz;
		uint16_t latch_vert;
		uint16_t current_vert;
		uint8_t last_visible_line;
		uint8_t interlace_count;
	} m_beam;

	struct
	{
		uint8_t repeat;
		uint8_t hflip;
		uint8_t vflip;
		int16_t matrix_a;
		int16_t matrix_b;
		int16_t matrix_c;
		int16_t matrix_d;
		int16_t origin_x;
		int16_t origin_y;
		uint16_t hor_offset;
		uint16_t ver_offset;
		uint8_t extbg;
	} m_mode7;

	struct OAM
	{
		uint16_t tile;
		int16_t x, y;
		uint8_t size, vflip, hflip, priority_bits, pal;
		int height, width;
	};

	struct OAM m_oam_spritelist[SNES_SCR_WIDTH / 2];

	uint8_t m_oam_itemlist[32];

	struct TILELIST {
		int16_t x;
		uint16_t priority, pal, tileaddr;
		int hflip;
	};

	struct TILELIST m_oam_tilelist[34];

#if SNES_LAYER_DEBUG
	struct DEBUGOPTS
	{
		uint8_t bg_disabled[5];
		uint8_t mode_disabled[8];
		uint8_t draw_subscreen;
		uint8_t windows_disabled;
		uint8_t mosaic_disabled;
		uint8_t colormath_disabled;
		uint8_t sprite_reversed;
		uint8_t select_pri[5];
	};
	struct DEBUGOPTS m_debug_options;
	uint8_t dbg_video( uint16_t curline );
#endif

	uint8_t m_mosaic_size;
	uint8_t m_clip_to_black;
	uint8_t m_prevent_color_math;
	uint8_t m_sub_add_mode;
	uint8_t m_bg3_priority_bit;
	uint8_t m_direct_color;
	uint8_t m_ppu_last_scroll;      /* as per Anomie's doc and Theme Park, all scroll regs shares (but mode 7 ones) the same
	                               'previous' scroll value */
	uint8_t m_mode7_last_scroll;    /* as per Anomie's doc mode 7 scroll regs use a different value, shared with mode 7 matrix! */

	uint8_t m_ppu1_open_bus, m_ppu2_open_bus;
	uint8_t m_ppu1_version, m_ppu2_version;
	uint8_t m_window1_left, m_window1_right, m_window2_left, m_window2_right;

	uint16_t m_mosaic_table[16][4096];
	uint8_t m_clipmasks[6][SNES_SCR_WIDTH];
	uint8_t m_update_windows;
	uint8_t m_update_offsets;
	uint8_t m_update_oam_list;
	uint8_t m_mode;
	uint8_t m_interlace; //doubles the visible resolution
	uint8_t m_obj_interlace;
	uint8_t m_screen_brightness;
	uint8_t m_screen_disabled;
	uint8_t m_pseudo_hires;
	uint8_t m_color_modes;
	uint8_t m_stat77;
	uint8_t m_stat78;

	uint16_t                m_htmult;     /* in 512 wide, we run HTOTAL double and halve it on latching */
	uint16_t                m_cgram_address;  /* CGRAM address */
	uint8_t                 m_read_ophct;
	uint8_t                 m_read_opvct;
	uint16_t                m_vram_fgr_high;
	uint16_t                m_vram_fgr_increment;
	uint16_t                m_vram_fgr_count;
	uint16_t                m_vram_fgr_mask;
	uint16_t                m_vram_fgr_shift;
	uint16_t                m_vram_read_buffer;
	uint16_t                m_vmadd;

	inline uint16_t get_bgcolor(uint8_t direct_colors, uint16_t palette, uint8_t color);
	inline void set_scanline_pixel(int screen, int16_t x, uint16_t color, uint8_t priority, uint8_t layer, int blend);
	inline void draw_bgtile_lores(uint8_t layer, int16_t ii, uint8_t colour, uint16_t pal, uint8_t direct_colors, uint8_t priority);
	inline void draw_bgtile_hires(uint8_t layer, int16_t ii, uint8_t colour, uint16_t pal, uint8_t direct_colors, uint8_t priority);
	inline void draw_oamtile(int16_t ii, uint8_t colour, uint16_t pal, uint8_t priority);
	inline void draw_tile(uint8_t planes, uint8_t layer, uint32_t tileaddr, int16_t x, uint8_t priority, uint8_t flip, uint8_t direct_colors, uint16_t pal, uint8_t hires);
	inline uint32_t get_tmap_addr(uint8_t layer, uint8_t tile_size, uint32_t base, uint32_t x, uint32_t y);
	inline void update_line(uint16_t curline, uint8_t layer, uint8_t priority_b, uint8_t priority_a, uint8_t color_depth, uint8_t hires, uint8_t offset_per_tile, uint8_t direct_colors);
	void update_line_mode7(uint16_t curline, uint8_t layer, uint8_t priority_b, uint8_t priority_a);
	void update_obsel(void);
	void oam_list_build(void);
	int is_sprite_on_scanline(uint16_t curline, uint8_t sprite);
	void update_objects_rto(uint16_t curline);
	void update_objects(uint8_t priority_oam0, uint8_t priority_oam1, uint8_t priority_oam2, uint8_t priority_oam3);
	void update_mode_0(uint16_t curline);
	void update_mode_1(uint16_t curline);
	void update_mode_2(uint16_t curline);
	void update_mode_3(uint16_t curline);
	void update_mode_4(uint16_t curline);
	void update_mode_5(uint16_t curline);
	void update_mode_6(uint16_t curline);
	void update_mode_7(uint16_t curline);
	void draw_screens(uint16_t curline);
	void update_windowmasks(void);
	void update_offsets(void);
	inline void draw_blend(uint16_t offset, uint16_t *colour, uint8_t prevent_color_math, uint8_t black_pen_clip, int switch_screens);

	void dynamic_res_change();
	inline uint32_t get_vram_address();

	DECLARE_READ8_MEMBER( oam_read );
	DECLARE_WRITE8_MEMBER( oam_write );
	DECLARE_READ8_MEMBER( cgram_read );
	DECLARE_WRITE8_MEMBER( cgram_write );
	DECLARE_READ8_MEMBER( vram_read );
	DECLARE_WRITE8_MEMBER( vram_write );
	std::unique_ptr<uint16_t[]> m_oam_ram;     /* Object Attribute Memory */
	std::unique_ptr<uint16_t[]> m_cgram;   /* Palette RAM */
	std::unique_ptr<uint8_t[]> m_vram;    /* Video RAM (TODO: Should be 16-bit, but it's easier this way) */

	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

private:
	devcb_read16  m_openbus_cb;
	optional_ioport m_options;
	optional_ioport m_debug1;
	optional_ioport m_debug2;
	optional_ioport m_debug3;
	optional_ioport m_debug4;
};


// device type definition
DECLARE_DEVICE_TYPE(SNES_PPU, snes_ppu_device)


/***************************************************************************
 INTERFACE CONFIGURATION MACROS
 ***************************************************************************/

#define MCFG_SNES_PPU_OPENBUS_CB(_read) \
	devcb = &snes_ppu_device::static_set_open_bus_callback(*device, DEVCB_##_read);

#endif // MAME_VIDEO_SNES_PPU_H
