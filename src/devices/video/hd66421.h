// license:BSD-3-Clause
// copyright-holders:Tim Schuerewegen
/***************************************************************************

  Hitachi HD66421 LCD Controller

  (c) 2001-2007 Tim Schuerewegen

 ***************************************************************************/

#ifndef MAME_VIDEO_HD66421_H
#define MAME_VIDEO_HD66421_H

#pragma once


///*************************************************************************
//  INTERFACE CONFIGURATION MACROS
///*************************************************************************

#define MCFG_HD66421_ADD(_tag) \
	MCFG_DEVICE_ADD(_tag, HD66421, 0)

///*************************************************************************
//  TYPE DEFINITIONS
///*************************************************************************

// ======================> hd66421_device

class hd66421_device :  public device_t,
						public device_memory_interface
{
public:
	static constexpr unsigned WIDTH   = 160;
	static constexpr unsigned HEIGHT  = 100;

	// construction/destruction
	hd66421_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	DECLARE_READ8_MEMBER( reg_idx_r );
	DECLARE_WRITE8_MEMBER( reg_idx_w );
	DECLARE_READ8_MEMBER( reg_dat_r );
	DECLARE_WRITE8_MEMBER( reg_dat_w );

	uint32_t update_screen(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_add_mconfig(machine_config &config) override;

	// device_config_memory_interface overrides
	virtual space_config_vector memory_space_config() const override;

	// address space configurations
	const address_space_config      m_space_config;

	inline uint8_t readbyte(offs_t address);
	inline void writebyte(offs_t address, uint8_t data);

	void plot_pixel(bitmap_ind16 &bitmap, int x, int y, uint32_t color);

private:
	uint8_t m_cmd, m_reg[32];
	int m_x, m_y;
	required_device<palette_device> m_palette;

	DECLARE_PALETTE_INIT(hd66421);
};


// device type definition
DECLARE_DEVICE_TYPE(HD66421, hd66421_device)

#endif // MAME_VIDEO_HD66421_H
