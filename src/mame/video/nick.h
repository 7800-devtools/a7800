// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    Intelligent Designs NICK emulation

**********************************************************************/

#ifndef MAME_VIDEO_NICK_H
#define MAME_VIDEO_NICK_H

#pragma once

#include "machine/rescap.h"
#include "video/resnet.h"
#include "screen.h"



///*************************************************************************
//  INTERFACE CONFIGURATION MACROS
///*************************************************************************

#define MCFG_NICK_ADD(_tag, _screen_tag, _clock) \
	MCFG_SCREEN_ADD(_screen_tag, RASTER) \
	MCFG_SCREEN_REFRESH_RATE(50) \
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(2500)) \
	MCFG_SCREEN_SIZE(ENTERPRISE_SCREEN_WIDTH, ENTERPRISE_SCREEN_HEIGHT) \
	MCFG_SCREEN_VISIBLE_AREA(0, ENTERPRISE_SCREEN_WIDTH-1, 0, ENTERPRISE_SCREEN_HEIGHT-1) \
	MCFG_SCREEN_UPDATE_DEVICE(_tag, nick_device, screen_update) \
	MCFG_DEVICE_ADD(_tag, NICK, _clock) \
	MCFG_VIDEO_SET_SCREEN(_screen_tag)


#define MCFG_NICK_VIRQ_CALLBACK(_write) \
	devcb = &nick_device::set_virq_wr_callback(*device, DEVCB_##_write);


/* there are 64us per line, although in reality
 about 50 are visible. */
#define ENTERPRISE_SCREEN_WIDTH (50*16)

/* there are 312 lines per screen, although in reality
 about 35*8 are visible */
#define ENTERPRISE_SCREEN_HEIGHT    (35*8)


///*************************************************************************
//  TYPE DEFINITIONS
///*************************************************************************

/* Nick executes a Display list, in the form of a list of Line Parameter
 Tables, this is the form of the data */
struct LPT_ENTRY
{
	uint8_t SC;       /* scanlines in this modeline (two's complement) */
	uint8_t MB;       /* the MODEBYTE (defines video display mode) */
	uint8_t LM;       /* left margin etc */
	uint8_t RM;       /* right margin etc */
	uint8_t LD1L;     /* (a7..a0) of line data pointer LD1 */
	uint8_t LD1H;     /* (a8..a15) of line data pointer LD1 */
	uint8_t LD2L;     /* (a7..a0) of line data pointer LD2 */
	uint8_t LD2H;     /* (a8..a15) of line data pointer LD2 */
	uint8_t COL[8];   /* COL0..COL7 */
};


// ======================> nick_device

class nick_device :  public device_t,
						public device_memory_interface,
						public device_video_interface
{
public:
	// construction/destruction
	nick_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	template<class _Object> static devcb_base &set_virq_wr_callback(device_t &device, _Object object) { return downcast<nick_device &>(device).m_write_virq.set_callback(object); }

	virtual DECLARE_ADDRESS_MAP(vram_map, 8);
	virtual DECLARE_ADDRESS_MAP(vio_map, 8);

	uint32_t screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

	// device_memory_interface overrides
	virtual space_config_vector memory_space_config() const override;

	DECLARE_READ8_MEMBER( vram_r );
	DECLARE_WRITE8_MEMBER( vram_w );

	DECLARE_WRITE8_MEMBER( fixbias_w );
	DECLARE_WRITE8_MEMBER( border_w );
	DECLARE_WRITE8_MEMBER( lpl_w );
	DECLARE_WRITE8_MEMBER( lph_w );

	address_space_config m_space_config;

private:
	devcb_write_line m_write_virq;

	void initialize_palette();

	void write_pixel(int ci);
	void calc_visible_clocks(int width);
	void write_border(int clocks);
	void do_left_margin();
	void do_right_margin();

	int get_color_index(int pen_index);
	void write_pixels2color(uint8_t pen0, uint8_t pen1, uint8_t data_byte);
	void write_pixels2color_lpixel(uint8_t pen0, uint8_t pen1, uint8_t data_byte);
	void write_pixels(uint8_t data_byte, uint8_t char_idx);
	void write_pixels_lpixel(uint8_t data_byte, uint8_t char_idx);

	void do_pixel(int clocks_visible);
	void do_lpixel(int clocks_visible);
	void do_attr(int clocks_visible);
	void do_ch256(int clocks_visible);
	void do_ch128(int clocks_visible);
	void do_ch64(int clocks_visible);
	void do_display();
	void update_lpt();
	void reload_lpt();
	void do_line();

	/* current scanline within LPT */
	uint8_t m_scanline_count;

	uint8_t m_FIXBIAS;
	uint8_t m_BORDER;
	uint8_t m_LPL;
	uint8_t m_LPH;

	uint16_t m_LD1;
	uint16_t m_LD2;

	LPT_ENTRY   m_LPT;

	uint32_t *m_dest;
	int m_dest_pos;
	int m_dest_max_pos;

	uint8_t m_reg[4];

	/* first clock visible on left hand side */
	int m_first_visible_clock;
	/* first clock visible on right hand side */
	int m_last_visible_clock;

	/* given a bit pattern, this will get the pen index */
	uint8_t m_pen_idx_4col[256];
	/* given a bit pattern, this will get the pen index */
	uint8_t m_pen_idx_16col[256];

	int m_virq;

	bitmap_rgb32 m_bitmap;
	rgb_t m_palette[256];

	emu_timer *m_timer_scanline;
};


// device type definition
DECLARE_DEVICE_TYPE(NICK, nick_device)

#endif // MAME_VIDEO_NICK_H
