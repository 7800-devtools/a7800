// license:BSD-3-Clause
// copyright-holders:Jean-Francois DEL NERO
/*********************************************************************

    ef9364.h

    Thomson EF9364 video controller

*********************************************************************/

#ifndef MAME_VIDEO_EF9364_H
#define MAME_VIDEO_EF9364_H

#pragma once

#define MCFG_EF9364_PALETTE(_palette_tag) \
	ef9364_device::static_set_palette_tag(*device, "^" _palette_tag);

#define MCFG_EF9364_PAGES_CNT(_pages_number) \
	ef9364_device::static_set_nb_of_pages(*device,_pages_number);

#define MCFG_EF9364_IRQ_HANDLER(_devcb) \
	devcb = &ef9364_device::set_irq_handler(*device, DEVCB_##_devcb);

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> ef9364_device

class ef9364_device :   public device_t,
						public device_memory_interface,
						public device_video_interface
{
public:
	static constexpr unsigned NB_OF_COLUMNS = 64;
	static constexpr unsigned NB_OF_ROWS = 16;

	static constexpr unsigned TXTPLANE_MAX_SIZE = NB_OF_COLUMNS * NB_OF_ROWS;
	static constexpr unsigned MAX_TXTPLANES = 2;

	// construction/destruction
	ef9364_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// static configuration
	static void static_set_palette_tag(device_t &device, const char *tag);
	static void static_set_nb_of_pages(device_t &device, int nb_bitplanes );

	// device interface
	void update_scanline(uint16_t scanline);
	void set_color_entry( int index, uint8_t r, uint8_t g, uint8_t b );

	uint32_t screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);
	void char_latch_w(uint8_t data);
	void command_w(uint8_t cmd);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// device_config_memory_interface overrides
	virtual space_config_vector memory_space_config() const override;

	// address space configurations
	const address_space_config      m_space_config;

	// inline helper

private:
	void set_video_mode(void);
	void draw_border(uint16_t line);

	// internal state

	required_region_ptr<uint8_t> m_charset;
	address_space *m_textram;

	uint8_t x_curs_pos;
	uint8_t y_curs_pos;
	uint8_t char_latch;

	uint8_t m_border[80];                     //border color

	rgb_t palette[2];
	int   nb_of_pages;
	int   bitplane_xres;
	int   bitplane_yres;
	int   vsync_scanline_pos;
	int   cursor_cnt;
	int   cursor_state;

	uint32_t clock_freq;
	bitmap_rgb32 m_screen_out;

	required_device<palette_device> m_palette;
};

// device type definition
DECLARE_DEVICE_TYPE(EF9364, ef9364_device)

#endif // MAME_VIDEO_EF9364_H
