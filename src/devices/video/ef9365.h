// license:BSD-3-Clause
// copyright-holders:Jean-Francois DEL NERO
/*********************************************************************

    ef9365.h

    Thomson EF9365/EF9366 video controller

*********************************************************************/

#ifndef MAME_VIDEO_EF9365_H
#define MAME_VIDEO_EF9365_H

#pragma once

#define MCFG_EF936X_PALETTE(palette_tag) \
		ef9365_device::static_set_palette_tag(*device, ("^" palette_tag));

#define MCFG_EF936X_BITPLANES_CNT(bitplanes_number) \
		ef9365_device::static_set_nb_bitplanes(*device, (bitplanes_number));

#define MCFG_EF936X_DISPLAYMODE(display_mode) \
		ef9365_device::static_set_display_mode(*device, (ef9365_device::display_mode));

#define MCFG_EF936X_IRQ_HANDLER(cb) \
		devcb = &ef9365_device::set_irq_handler(*device, (DEVCB_##cb));

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> ef9365_device

class ef9365_device :   public device_t,
						public device_memory_interface,
						public device_video_interface
{
public:
	static constexpr unsigned BITPLANE_MAX_SIZE = 0x8000;
	static constexpr unsigned MAX_BITPLANES = 8;

	static constexpr int DISPLAY_MODE_256x256    = 0x00;
	static constexpr int DISPLAY_MODE_512x512    = 0x01;
	static constexpr int DISPLAY_MODE_512x256    = 0x02;
	static constexpr int DISPLAY_MODE_128x128    = 0x03;
	static constexpr int DISPLAY_MODE_64x64      = 0x04;

	// construction/destruction
	ef9365_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// static configuration
	static void static_set_palette_tag(device_t &device, const char *tag);
	static void static_set_nb_bitplanes(device_t &device, int nb_bitplanes );
	static void static_set_display_mode(device_t &device, int display_mode );
	template<class _Object> static devcb_base &set_irq_handler(device_t &device, _Object object) { return downcast<ef9365_device &>(device).m_irq_handler.set_callback(object); }

	// device interface
	DECLARE_READ8_MEMBER( data_r );
	DECLARE_WRITE8_MEMBER( data_w );

	void update_scanline(uint16_t scanline);
	void set_color_filler( uint8_t color );
	void set_color_entry( int index, uint8_t r, uint8_t g, uint8_t b );

	uint8_t get_last_readback_word(int bitplane_number, int * pixel_offset);

	uint32_t screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;
	virtual const tiny_rom_entry *device_rom_region() const override;

	// device_config_memory_interface overrides
	virtual space_config_vector memory_space_config() const override;

	// address space configurations
	const address_space_config      m_space_config;

private:
	int get_char_pix( unsigned char c, int x, int y );
	void plot(int x_pos,int y_pos);
	int draw_character( unsigned char c, int block, int smallblock );
	int draw_vector(uint16_t start_x,uint16_t start_y,short delta_x,short delta_y);
	uint16_t get_x_reg();
	uint16_t get_y_reg();
	void set_x_reg(uint16_t x);
	void set_y_reg(uint16_t y);
	void screen_scanning( int force_clear );
	void set_busy_flag(int period);
	void set_video_mode(void);
	void draw_border(uint16_t line);
	void ef9365_exec(uint8_t cmd);
	int  cycles_to_us(int cycles);
	void dump_bitplanes_word();
	void update_interrupts();

	// internal state
	static constexpr device_timer_id BUSY_TIMER = 0;

	required_region_ptr<uint8_t> m_charset;
	address_space *m_videoram;

	uint8_t m_irq_state;
	uint8_t m_irq_vb;
	uint8_t m_irq_lb;
	uint8_t m_irq_rdy;
	uint8_t m_current_color;
	uint8_t m_bf;                             //busy flag
	uint8_t m_registers[0x10];                //registers
	uint8_t m_state;                          //status register
	uint8_t m_border[80];                     //border color

	rgb_t palette[256];                     // 8 bitplanes max -> 256 colors max
	int   nb_of_bitplanes;
	int   nb_of_colors;
	int   bitplane_xres;
	int   bitplane_yres;
	uint16_t overflow_mask_x;
	uint16_t overflow_mask_y;
	int   vsync_scanline_pos;

	uint8_t m_readback_latch[MAX_BITPLANES];   // Last DRAM Readback buffer (Filled after a Direct Memory Access Request command)
	int m_readback_latch_pix_offset;

	uint32_t clock_freq;
	bitmap_rgb32 m_screen_out;

	// timers
	emu_timer *m_busy_timer;

	required_device<palette_device> m_palette;
	devcb_write_line m_irq_handler;
};

// device type definition
DECLARE_DEVICE_TYPE(EF9365, ef9365_device)

#endif // MAME_VIDEO_EF9365_H
