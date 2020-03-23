// license:BSD-3-Clause
// copyright-holders:Wilbert Pol, Miodrag Milanovic
#ifndef MAME_BUS_ISA_MDA_H
#define MAME_BUS_ISA_MDA_H

#pragma once

#include "isa.h"
#include "video/mc6845.h"

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> isa8_mda_device

class isa8_mda_device : public device_t,
	public device_isa8_card_interface
{
public:
	friend class isa8_hercules_device;

	// construction/destruction
	isa8_mda_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual DECLARE_READ8_MEMBER(io_read);
	virtual DECLARE_WRITE8_MEMBER(io_write);
	virtual DECLARE_READ8_MEMBER(status_r);
	virtual DECLARE_WRITE8_MEMBER(mode_control_w);

	DECLARE_WRITE_LINE_MEMBER(hsync_changed);
	DECLARE_WRITE_LINE_MEMBER(vsync_changed);

	virtual MC6845_UPDATE_ROW( crtc_update_row );

protected:
	isa8_mda_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// optional information overrides
	virtual void device_add_mconfig(machine_config &config) override;
	virtual const tiny_rom_entry *device_rom_region() const override;

private:
	WRITE_LINE_MEMBER(pc_cpu_line);

	MC6845_UPDATE_ROW( mda_text_inten_update_row );
	MC6845_UPDATE_ROW( mda_text_blink_update_row );

public:
	int m_framecnt;

	uint8_t   m_mode_control;

	int     m_update_row_type;
	uint8_t   *m_chr_gen;
	uint8_t   m_vsync;
	uint8_t   m_hsync;
	std::vector<uint8_t> m_videoram;
	uint8_t   m_pixel;
	required_device<palette_device> m_palette;
};


// device type definition
DECLARE_DEVICE_TYPE(ISA8_MDA, isa8_mda_device)

// ======================> isa8_hercules_device

class isa8_hercules_device :
		public isa8_mda_device
{
public:
	// construction/destruction
	isa8_hercules_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual DECLARE_READ8_MEMBER(io_read) override;
	virtual DECLARE_WRITE8_MEMBER(io_write) override;
	virtual DECLARE_READ8_MEMBER(status_r) override;
	virtual DECLARE_WRITE8_MEMBER(mode_control_w) override;

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// optional information overrides
	virtual void device_add_mconfig(machine_config &config) override;
	virtual const tiny_rom_entry *device_rom_region() const override;

private:
	virtual MC6845_UPDATE_ROW( crtc_update_row ) override;
	MC6845_UPDATE_ROW( hercules_gfx_update_row );

	uint8_t m_configuration_switch; //hercules
};


// device type definition
DECLARE_DEVICE_TYPE(ISA8_HERCULES, isa8_hercules_device)

// ======================> isa8_ec1840_0002_device

class isa8_ec1840_0002_device :
		public isa8_mda_device
{
public:
	// construction/destruction
	isa8_ec1840_0002_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// optional information overrides
	virtual void device_add_mconfig(machine_config &config) override;

private:
	virtual DECLARE_WRITE8_MEMBER(mode_control_w) override;

	virtual MC6845_UPDATE_ROW( crtc_update_row ) override;
	MC6845_UPDATE_ROW( mda_lowres_text_inten_update_row );
	MC6845_UPDATE_ROW( mda_lowres_text_blink_update_row );

	std::unique_ptr<uint8_t[]>   m_soft_chr_gen;
};

// device type definition
DECLARE_DEVICE_TYPE(ISA8_EC1840_0002, isa8_ec1840_0002_device)

#endif // MAME_BUS_ISA_MDA_H
