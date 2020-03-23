// license:BSD-3-Clause
// copyright-holders:Barry Rodewald
#ifndef MAME_BUS_ISA_SVGA_S3_H
#define MAME_BUS_ISA_SVGA_S3_H

#pragma once

#include "isa.h"
#include "video/pc_vga.h"
#include "s3virge.h"

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> isa16_vga_device

class isa16_svga_s3_device :
		public device_t,
		public device_isa16_card_interface
{
public:
	// construction/destruction
	isa16_svga_s3_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	DECLARE_READ8_MEMBER(input_port_0_r);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// optional information overrides
	virtual void device_add_mconfig(machine_config &config) override;
	virtual const tiny_rom_entry *device_rom_region() const override;

private:
	s3_vga_device *m_vga;
	ibm8514a_device *m_8514;
};

class isa16_s3virge_device :
		public device_t,
		public device_isa16_card_interface
{
public:
	// construction/destruction
	isa16_s3virge_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	DECLARE_READ8_MEMBER(input_port_0_r);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// optional information overrides
	virtual void device_add_mconfig(machine_config &config) override;
	virtual const tiny_rom_entry *device_rom_region() const override;

private:
	s3virge_vga_device *m_vga;
};

class isa16_s3virgedx_device :
		public device_t,
		public device_isa16_card_interface
{
public:
	// construction/destruction
	isa16_s3virgedx_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	DECLARE_READ8_MEMBER(input_port_0_r);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// optional information overrides
	virtual void device_add_mconfig(machine_config &config) override;
	virtual const tiny_rom_entry *device_rom_region() const override;

private:
	s3virgedx_vga_device *m_vga;
};

class isa16_stealth3d2kpro_device :
		public device_t,
		public device_isa16_card_interface
{
public:
	// construction/destruction
	isa16_stealth3d2kpro_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	DECLARE_READ8_MEMBER(input_port_0_r);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// optional information overrides
	virtual void device_add_mconfig(machine_config &config) override;
	virtual const tiny_rom_entry *device_rom_region() const override;

private:
	s3virgedx_vga_device *m_vga;
};


// device type definition
DECLARE_DEVICE_TYPE(ISA16_SVGA_S3,    isa16_svga_s3_device)
DECLARE_DEVICE_TYPE(ISA16_S3VIRGE,    isa16_s3virge_device)
DECLARE_DEVICE_TYPE(ISA16_S3VIRGEDX,  isa16_s3virgedx_device)
DECLARE_DEVICE_TYPE(ISA16_DMS3D2KPRO, isa16_stealth3d2kpro_device)

#endif // MAME_BUS_ISA_SVGA_S3_H
