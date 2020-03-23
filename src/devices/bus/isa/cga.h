// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
#ifndef MAME_BUS_ISA_CGA_H
#define MAME_BUS_ISA_CGA_H

#pragma once

#include "isa.h"
#include "video/mc6845.h"
#include "screen.h"


//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> isa8_cga_device

class isa8_cga_device :
		public device_t,
		public device_isa8_card_interface
{
	friend class isa8_cga_superimpose_device;
//  friend class isa8_ec1841_0002_device;
	friend class isa8_cga_poisk2_device;
	friend class isa8_cga_pc1512_device;

public:
	// construction/destruction
	isa8_cga_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual MC6845_UPDATE_ROW( crtc_update_row );
	MC6845_UPDATE_ROW( cga_text_inten_update_row );
	MC6845_UPDATE_ROW( cga_text_inten_comp_grey_update_row );
	MC6845_UPDATE_ROW( cga_text_inten_alt_update_row );
	MC6845_UPDATE_ROW( cga_text_blink_update_row );
	MC6845_UPDATE_ROW( cga_text_blink_update_row_si );
	MC6845_UPDATE_ROW( cga_text_blink_alt_update_row );
	MC6845_UPDATE_ROW( cga_gfx_4bppl_update_row );
	MC6845_UPDATE_ROW( cga_gfx_4bpph_update_row );
	MC6845_UPDATE_ROW( cga_gfx_2bpp_update_row );
	MC6845_UPDATE_ROW( cga_gfx_1bpp_update_row );

protected:
	isa8_cga_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	required_ioport m_cga_config;

	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// optional information overrides
	virtual void device_add_mconfig(machine_config &config) override;
	virtual ioport_constructor device_input_ports() const override;
	virtual const tiny_rom_entry *device_rom_region() const override;

	virtual uint32_t screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);

public:
	void mode_control_w(uint8_t data);
	void set_palette_luts();
	void plantronics_w(uint8_t data);
	virtual DECLARE_READ8_MEMBER( io_read );
	virtual DECLARE_WRITE8_MEMBER( io_write );

public:
	int     m_framecnt;

	uint8_t   m_mode_control;  /* wo 0x3d8 */
	uint8_t   m_color_select;  /* wo 0x3d9 */
	//uint8_t   m_status;   //unused?     /* ro 0x3da */

	int     m_update_row_type, m_y;
	uint8_t   m_palette_lut_2bpp[4];
	offs_t  m_chr_gen_offset[4];
	uint8_t   m_font_selection_mask;
	uint8_t   *m_chr_gen_base;
	uint8_t   *m_chr_gen;
	uint8_t   m_vsync;
	uint8_t   m_hsync;
	size_t  m_vram_size;
	std::vector<uint8_t> m_vram;
	bool    m_superimpose;
	uint8_t   m_plantronics; /* This should be moved into the appropriate subclass */
	offs_t  m_start_offset;
	required_device<palette_device> m_palette;
	required_device<screen_device> m_screen;

private:
	DECLARE_WRITE_LINE_MEMBER( hsync_changed );
	DECLARE_WRITE_LINE_MEMBER( vsync_changed );
	MC6845_RECONFIGURE(reconfigure);
};

// device type definition
DECLARE_DEVICE_TYPE(ISA8_CGA, isa8_cga_device)


// ======================> isa8_cga_superimpose_device

class isa8_cga_superimpose_device :
		public isa8_cga_device
{
public:
	// construction/destruction
	isa8_cga_superimpose_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
protected:
	isa8_cga_superimpose_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);
};

// device type definition
DECLARE_DEVICE_TYPE(ISA8_CGA_SUPERIMPOSE, isa8_cga_superimpose_device)


// ======================> isa8_poisk2_device

class isa8_cga_poisk2_device :
		public isa8_cga_device
{
public:
	// construction/destruction
	isa8_cga_poisk2_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	// optional information overrides
	virtual const tiny_rom_entry *device_rom_region() const override;
};

// device type definition
DECLARE_DEVICE_TYPE(ISA8_CGA_POISK2, isa8_cga_poisk2_device)


// ======================> isa8_pc1512_device

class isa8_cga_pc1512_device :
		public isa8_cga_device
{
public:
	// construction/destruction
	isa8_cga_pc1512_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	// optional information overrides
	virtual ioport_constructor device_input_ports() const override;
	virtual const tiny_rom_entry *device_rom_region() const override;

	virtual MC6845_UPDATE_ROW( crtc_update_row ) override;
	MC6845_UPDATE_ROW( pc1512_gfx_4bpp_update_row );

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

public:
	uint8_t   m_write;
	uint8_t   m_read;
	uint8_t   m_mc6845_address;
	uint8_t   m_mc6845_locked_register[31];

public:
	// Static information
	// mapping of the 4 planes into videoram
	// (text data should be readable at videoram+0)
	static const offs_t vram_offset[4];
	static const uint8_t mc6845_writeonce_register[31];

	virtual DECLARE_READ8_MEMBER( io_read ) override;
	virtual DECLARE_WRITE8_MEMBER( io_write ) override;

	DECLARE_WRITE8_MEMBER( vram_w );
};

// device type definition
DECLARE_DEVICE_TYPE(ISA8_CGA_PC1512, isa8_cga_pc1512_device)

// ======================> isa8_wyse700_device

class isa8_wyse700_device :
		public isa8_cga_device
{
public:
	// construction/destruction
	isa8_wyse700_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	// optional information overrides
	virtual const tiny_rom_entry *device_rom_region() const override;

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

public:
	virtual DECLARE_READ8_MEMBER( io_read ) override;
	virtual DECLARE_WRITE8_MEMBER( io_write ) override;
	virtual uint32_t screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect) override;
	void change_resolution(uint8_t mode);

	uint8_t m_bank_offset;
	uint8_t m_bank_base;
	uint8_t m_control;
};

// device type definition
DECLARE_DEVICE_TYPE(ISA8_WYSE700, isa8_wyse700_device)

// ======================> isa8_ec1841_0002_device

class isa8_ec1841_0002_device :
		public isa8_cga_device
{
public:
	// construction/destruction
	isa8_ec1841_0002_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

public:
	virtual DECLARE_READ8_MEMBER( io_read ) override;
	virtual DECLARE_WRITE8_MEMBER( io_write ) override;

	uint8_t   m_p3df;
	DECLARE_READ8_MEMBER( char_ram_read );
	DECLARE_WRITE8_MEMBER( char_ram_write );
};

// device type definition
DECLARE_DEVICE_TYPE(ISA8_EC1841_0002, isa8_ec1841_0002_device)

// ======================> isa8_cga_iskr1031_device

class isa8_cga_iskr1030m_device :
		public isa8_cga_device
{
public:
	// construction/destruction
	isa8_cga_iskr1030m_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	virtual const tiny_rom_entry *device_rom_region() const override;
};

// device type definition
DECLARE_DEVICE_TYPE(ISA8_CGA_ISKR1030M, isa8_cga_iskr1030m_device)

// ======================> isa8_cga_iskr1031_device

class isa8_cga_iskr1031_device :
		public isa8_cga_device
{
public:
	// construction/destruction
	isa8_cga_iskr1031_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	virtual const tiny_rom_entry *device_rom_region() const override;
};

// device type definition
DECLARE_DEVICE_TYPE(ISA8_CGA_ISKR1031, isa8_cga_iskr1031_device)

// ======================> isa8_cga_mc1502_device

class isa8_cga_mc1502_device :
		public isa8_cga_device
{
public:
	// construction/destruction
	isa8_cga_mc1502_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	// optional information overrides
	virtual const tiny_rom_entry *device_rom_region() const override;
};

// device type definition
DECLARE_DEVICE_TYPE(ISA8_CGA_MC1502, isa8_cga_mc1502_device)


class isa8_cga_m24_device :
		public isa8_cga_device
{
public:
	// construction/destruction
	isa8_cga_m24_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	virtual DECLARE_READ8_MEMBER( io_read ) override;
	virtual DECLARE_WRITE8_MEMBER( io_write ) override;
	virtual MC6845_UPDATE_ROW( crtc_update_row ) override;
	MC6845_UPDATE_ROW( m24_gfx_1bpp_m24_update_row );
	MC6845_RECONFIGURE(reconfigure);

protected:
	virtual void device_reset() override;
	// optional information overrides
	virtual void device_add_mconfig(machine_config &config) override;

private:
	uint8_t m_mode2, m_index;
};

// device type definition
DECLARE_DEVICE_TYPE(ISA8_CGA_M24, isa8_cga_m24_device)

#endif  // MAME_BUS_ISA_CGA_H
