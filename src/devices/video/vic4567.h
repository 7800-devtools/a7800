// license:BSD-3-Clause
// copyright-holders:Peter Trauner
/*****************************************************************************
 *
 * video/vic4567.h
 *
 ****************************************************************************/

#ifndef MAME_VIDEO_VIC4567_H
#define MAME_VIDEO_VIC4567_H

#pragma once


/***************************************************************************
    TYPE DEFINITIONS
***************************************************************************/


/***************************************************************************
    CONSTANTS
***************************************************************************/

#define VIC6567_CLOCK       (1022700 /* = 8181600 / 8) */ )
#define VIC6569_CLOCK       ( 985248 /* = 7881984 / 8) */ )

#define VIC6567_CYCLESPERLINE   65
#define VIC6569_CYCLESPERLINE   63

#define VIC6567_LINES       263
#define VIC6569_LINES       312

#define VIC6567_VRETRACERATE    (59.8245100906698 /* = 1022700 / (65 * 263) */ )
#define VIC6569_VRETRACERATE    (50.1245421245421 /* =  985248 / (63 * 312) */ )

#define VIC6567_HRETRACERATE    (VIC6567_CLOCK / VIC6567_CYCLESPERLINE)
#define VIC6569_HRETRACERATE    (VIC6569_CLOCK / VIC6569_CYCLESPERLINE)

#define VIC2_HSIZE      320
#define VIC2_VSIZE      200

#define VIC6567_VISIBLELINES    235
#define VIC6569_VISIBLELINES    284

#define VIC6567_FIRST_DMA_LINE  0x30
#define VIC6569_FIRST_DMA_LINE  0x30

#define VIC6567_LAST_DMA_LINE   0xf7
#define VIC6569_LAST_DMA_LINE   0xf7

#define VIC6567_FIRST_DISP_LINE 0x29
#define VIC6569_FIRST_DISP_LINE 0x10

#define VIC6567_LAST_DISP_LINE  (VIC6567_FIRST_DISP_LINE + VIC6567_VISIBLELINES - 1)
#define VIC6569_LAST_DISP_LINE  (VIC6569_FIRST_DISP_LINE + VIC6569_VISIBLELINES - 1)

#define VIC6567_RASTER_2_EMU(a) ((a >= VIC6567_FIRST_DISP_LINE) ? (a - VIC6567_FIRST_DISP_LINE) : (a + 222))
#define VIC6569_RASTER_2_EMU(a) (a - VIC6569_FIRST_DISP_LINE)

#define VIC6567_FIRSTCOLUMN 50
#define VIC6569_FIRSTCOLUMN 50

#define VIC6567_VISIBLECOLUMNS  418
#define VIC6569_VISIBLECOLUMNS  403

#define VIC6567_X_2_EMU(a)  (a)
#define VIC6569_X_2_EMU(a)  (a)

#define VIC6567_STARTVISIBLELINES ((VIC6567_LINES - VIC6567_VISIBLELINES)/2)
#define VIC6569_STARTVISIBLELINES 16 /* ((VIC6569_LINES - VIC6569_VISIBLELINES)/2) */

#define VIC6567_FIRSTRASTERLINE 34
#define VIC6569_FIRSTRASTERLINE 0

#define VIC6567_COLUMNS 512
#define VIC6569_COLUMNS 504


#define VIC6567_STARTVISIBLECOLUMNS ((VIC6567_COLUMNS - VIC6567_VISIBLECOLUMNS)/2)
#define VIC6569_STARTVISIBLECOLUMNS ((VIC6569_COLUMNS - VIC6569_VISIBLECOLUMNS)/2)

#define VIC6567_FIRSTRASTERCOLUMNS 412
#define VIC6569_FIRSTRASTERCOLUMNS 404

#define VIC6569_FIRST_X 0x194
#define VIC6567_FIRST_X 0x19c

#define VIC6569_FIRST_VISIBLE_X 0x1e0
#define VIC6567_FIRST_VISIBLE_X 0x1e8

#define VIC6569_MAX_X 0x1f7
#define VIC6567_MAX_X 0x1ff

#define VIC6569_LAST_VISIBLE_X 0x17c
#define VIC6567_LAST_VISIBLE_X 0x184

#define VIC6569_LAST_X 0x193
#define VIC6567_LAST_X 0x19b

/***************************************************************************
    DEVICE CONFIGURATION MACROS
***************************************************************************/

class vic3_device : public device_t,
					public device_video_interface
{
public:
	enum class vic3_type { NTSC, PAL };

	vic3_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	static void static_set_cpu_tag(device_t &device, const char *tag) { downcast<vic3_device &>(device).m_cpu.set_tag(tag); }
	static void set_vic3_type(device_t &device, vic3_type type) { downcast<vic3_device &>(device).m_type = type; }
	template <class Object> static devcb_base &set_dma_read_callback(device_t &device, Object &&cb) { return downcast<vic3_device &>(device).m_dma_read_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_dma_read_color_callback(device_t &device, Object &&cb) { return downcast<vic3_device &>(device).m_dma_read_color_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_interrupt_callback(device_t &device, Object &&cb) { return downcast<vic3_device &>(device).m_interrupt_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_port_changed_callback(device_t &device, Object &&cb) { return downcast<vic3_device &>(device).m_port_changed_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_lightpen_button_callback(device_t &device, Object &&cb) { return downcast<vic3_device &>(device).m_lightpen_button_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_lightpen_x_callback(device_t &device, Object &&cb) { return downcast<vic3_device &>(device).m_lightpen_x_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_lightpen_y_callback(device_t &device, Object &&cb) { return downcast<vic3_device &>(device).m_lightpen_y_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_c64_mem_r_callback(device_t &device, Object &&cb) { return downcast<vic3_device &>(device).m_c64_mem_r_cb.set_callback(std::forward<Object>(cb)); }

	DECLARE_WRITE8_MEMBER(port_w);
	DECLARE_WRITE8_MEMBER(palette_w);
	DECLARE_READ8_MEMBER(port_r);

	void raster_interrupt_gen();
	uint32_t video_update(bitmap_ind16 &bitmap, const rectangle &cliprect);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_add_mconfig(machine_config &config) override;

private:
	static constexpr unsigned SPRITE_BASE_X_SIZE = 24;
	static constexpr unsigned SPRITE_BASE_Y_SIZE = 21;

	struct vic3_sprite
	{
		int x, y;

		int repeat;                         /* expand, line once drawn */
		int line;                           /* 0 not painting, else painting */

		/* buffer for currently painted line */
		int paintedline[8];
		uint8_t bitmap[8][SPRITE_BASE_X_SIZE * 2 / 8 + 1  /*for simpler sprite collision detection*/];
	};

	// internal state
	inline int getforeground(int y, int x);
	inline int getforeground16(int y, int x);
	void set_interrupt(int mask);
	void clear_interrupt(int mask);
	void draw_character(int ybegin, int yend, int ch, int yoff, int xoff, uint16_t *color, int start_x, int end_x);
	void draw_character_multi(int ybegin, int yend, int ch, int yoff, int xoff, int start_x, int end_x);
	void draw_bitmap(int ybegin, int yend, int ch, int yoff, int xoff, int start_x, int end_x);
	void draw_bitmap_multi(int ybegin, int yend, int ch, int yoff, int xoff, int start_x, int end_x);
	void draw_sprite_code(int y, int xbegin, int code, int color, int start_x, int end_x);
	void draw_sprite_code_multi(int y, int xbegin, int code, int prior, int start_x, int end_x);
	void sprite_collision(int nr, int y, int x, int mask);
	void draw_sprite(int nr, int yoff, int ybegin, int yend, int start_x, int end_x);
	void draw_sprite_multi(int nr, int yoff, int ybegin, int yend, int start_x, int end_x);
	void drawlines(int first, int last, int start_x, int end_x);
	void vic2_drawlines(int first, int last, int start_x, int end_x);
	void interlace_draw_block(int x, int y, int offset);
	void draw_block(int x, int y, int offset);
	void draw_bitplanes();

	TIMER_CALLBACK_MEMBER(timer_timeout);

	vic3_type  m_type;

	required_device<cpu_device> m_cpu;

	uint8_t m_reg[0x80];
	int m_on;                             /* rastering of the screen */

	int m_lines;

	uint16_t m_chargenaddr, m_videoaddr, m_bitmapaddr;

	std::unique_ptr<bitmap_ind16> m_bitmap;
	int m_x_begin, m_x_end;
	int m_y_begin, m_y_end;

	uint16_t m_c64_bitmap[2], m_bitmapmulti[4], m_mono[2], m_multi[4], m_ecmcolor[2], m_colors[4], m_spritemulti[4];

	int m_lastline, m_rasterline;

	int m_interlace;
	int m_columns, m_rows;

	/* background/foreground for sprite collision */
	uint8_t *m_screenptr[216], m_shift[216];

	/* convert multicolor byte to background/foreground for sprite collision */
	uint8_t m_foreground[256];
	uint16_t m_expandx[256];
	uint16_t m_expandx_multi[256];

	/* converts sprite multicolor info to info for background collision checking */
	uint8_t m_multi_collision[256];

	vic3_sprite m_sprites[8];

	/* DMA */
	devcb_read8    m_dma_read_cb;
	devcb_read8    m_dma_read_color_cb;

	/* IRQ */
	devcb_write_line m_interrupt_cb;

	/* Port Changed */
	devcb_write8   m_port_changed_cb;

	/* lightpen */
	devcb_read8 m_lightpen_button_cb;
	devcb_read8 m_lightpen_x_cb;
	devcb_read8 m_lightpen_y_cb;

	/* C64 memory access */
	devcb_read8      m_c64_mem_r_cb;

	/* palette - vic3 specific items (the ones above are used for VIC II as well) */
	uint8_t m_palette_red[0x100];
	uint8_t m_palette_green[0x100];
	uint8_t m_palette_blue[0x100];
	int m_palette_dirty;

	required_device<palette_device> m_palette;
};

DECLARE_DEVICE_TYPE(VIC3, vic3_device)


#define MCFG_VIC3_CPU(tag) \
		vic3_device::static_set_cpu_tag(*device, ("^" tag));

#define MCFG_VIC3_TYPE(type) \
		vic3_device::set_vic3_type(*device, (vic3_device::vic3_type::type));

#define MCFG_VIC3_DMA_READ_CB(cb) \
		devcb = &vic3_device::set_dma_read_callback(*device, (DEVCB_##cb));

#define MCFG_VIC3_DMA_READ_COLOR_CB(cb) \
	devcb = &vic3_device::set_dma_read_color_callback(*device, (DEVCB_##cb));

#define MCFG_VIC3_INTERRUPT_CB(cb) \
	devcb = &vic3_device::set_interrupt_callback(*device, (DEVCB_##cb));

#define MCFG_VIC3_PORT_CHANGED_CB(cb) \
	devcb = &vic3_device::set_port_changed_callback(*device, (DEVCB_##cb));

#define MCFG_VIC3_LIGHTPEN_BUTTON_CB(cb) \
	devcb = &vic3_device::set_lightpen_button_callback(*device, (DEVCB_##cb));

#define MCFG_VIC3_LIGHTPEN_X_CB(cb) \
	devcb = &vic3_device::set_lightpen_x_callback(*device, (DEVCB_##cb));

#define MCFG_VIC3_LIGHTPEN_Y_CB(cb) \
	devcb = &vic3_device::set_lightpen_y_callback(*device, (DEVCB_##cb));

#define MCFG_VIC3_C64_MEM_R_CB(cb) \
	devcb = &vic3_device::set_c64_mem_r_callback(*device, (DEVCB_##cb));

#endif // MAME_VIDEO_VIC4567_H
