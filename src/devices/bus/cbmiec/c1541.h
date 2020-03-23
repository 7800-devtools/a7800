// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    Commodore 1540/1541/1541C/1541-II Single Disk Drive emulation

**********************************************************************/

#ifndef MAME_BUS_CBMIEC_C1541_H
#define MAME_BUS_CBMIEC_C1541_H

#pragma once

#include "cbmiec.h"
#include "bus/c64/bn1541.h"
#include "cpu/m6502/m6502.h"
#include "machine/64h156.h"
#include "machine/6522via.h"
#include "machine/6821pia.h"
#include "machine/latch.h"



//**************************************************************************
//  MACROS / CONSTANTS
//**************************************************************************

#define C1541_TAG           "c1541"



//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> c1541_device_base

class c1541_device_base :  public device_t,
						public device_cbm_iec_interface,
						public device_c64_floppy_parallel_interface
{
protected:
	// construction/destruction
	c1541_device_base(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// optional information overrides
	virtual void device_add_mconfig(machine_config &config) override;
	virtual ioport_constructor device_input_ports() const override;

	// device_cbm_iec_interface overrides
	virtual void cbm_iec_atn(int state) override;
	virtual void cbm_iec_reset(int state) override;

	// device_c64_floppy_parallel_interface overrides
	virtual void parallel_data_w(uint8_t data) override;
	virtual void parallel_strobe_w(int state) override;

	required_device<floppy_image_device> m_floppy;

private:
	enum
	{
		LED_POWER = 0,
		LED_ACT
	};

	inline void set_iec_data();

	DECLARE_WRITE_LINE_MEMBER( via0_irq_w );
	virtual DECLARE_READ8_MEMBER( via0_pa_r );
	DECLARE_WRITE8_MEMBER( via0_pa_w );
	DECLARE_READ8_MEMBER( via0_pb_r );
	DECLARE_WRITE8_MEMBER( via0_pb_w );
	DECLARE_WRITE_LINE_MEMBER( via0_ca2_w );
	DECLARE_WRITE_LINE_MEMBER( via1_irq_w );
	DECLARE_READ8_MEMBER( via1_pb_r );
	DECLARE_WRITE8_MEMBER( via1_pb_w );
	DECLARE_WRITE_LINE_MEMBER( atn_w );
	DECLARE_WRITE_LINE_MEMBER( byte_w );

	DECLARE_FLOPPY_FORMATS( floppy_formats );

	required_device<m6502_device> m_maincpu;
	required_device<via6522_device> m_via0;
	required_device<via6522_device> m_via1;
	required_device<c64h156_device> m_ga;
	required_ioport m_address;

	// IEC bus
	int m_data_out;                         // serial data out

	// interrupts
	int m_via0_irq;                         // VIA #0 interrupt request
	int m_via1_irq;                         // VIA #1 interrupt request
};


// ======================> c1540_device

class c1540_device :  public c1541_device_base
{
public:
	// construction/destruction
	c1540_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	// optional information overrides
	virtual const tiny_rom_entry *device_rom_region() const override;
};


// ======================> c1541_device

class c1541_device :  public c1541_device_base
{
public:
	// construction/destruction
	c1541_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	// optional information overrides
	virtual const tiny_rom_entry *device_rom_region() const override;
};


// ======================> c1541c_device

class c1541c_device :  public c1541_device_base
{
public:
	// construction/destruction
	c1541c_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	// optional information overrides
	virtual const tiny_rom_entry *device_rom_region() const override;
	virtual void device_add_mconfig(machine_config &config) override;

private:
	// not really public
	virtual DECLARE_READ8_MEMBER( via0_pa_r ) override;
};


// ======================> c1541ii_device

class c1541ii_device :  public c1541_device_base
{
public:
	// construction/destruction
	c1541ii_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	// optional information overrides
	virtual const tiny_rom_entry *device_rom_region() const override;
};


// ======================> sx1541_device

class sx1541_device :  public c1541_device_base
{
public:
	// construction/destruction
	sx1541_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	// optional information overrides
	virtual const tiny_rom_entry *device_rom_region() const override;
};


// ======================> fsd1_device

class fsd1_device :  public c1541_device_base
{
public:
	// construction/destruction
	fsd1_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	// optional information overrides
	virtual const tiny_rom_entry *device_rom_region() const override;
};


// ======================> fsd2_device

class fsd2_device :  public c1541_device_base
{
public:
	// construction/destruction
	fsd2_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	// optional information overrides
	virtual const tiny_rom_entry *device_rom_region() const override;

	// device-level overrides
	virtual void device_start() override;
};


// ======================> csd1_device

class csd1_device :  public c1541_device_base
{
public:
	// construction/destruction
	csd1_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	// optional information overrides
	virtual const tiny_rom_entry *device_rom_region() const override;
};


// ======================> c1541_dolphin_dos_device

class c1541_dolphin_dos_device :  public c1541_device_base
{
public:
	// construction/destruction
	c1541_dolphin_dos_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	// optional information overrides
	virtual const tiny_rom_entry *device_rom_region() const override;
	virtual void device_add_mconfig(machine_config &config) override;
};


// ======================> c1541_professional_dos_v1_device

class c1541_professional_dos_v1_device :  public c1541_device_base
{
public:
	// construction/destruction
	c1541_professional_dos_v1_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	// optional information overrides
	virtual const tiny_rom_entry *device_rom_region() const override;
	virtual void device_add_mconfig(machine_config &config) override;
};


// ======================> c1541_prologic_dos_classic_device

class c1541_prologic_dos_classic_device :  public c1541_device_base
{
public:
	// construction/destruction
	c1541_prologic_dos_classic_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// not really public
	DECLARE_READ8_MEMBER( read );
	DECLARE_WRITE8_MEMBER( write );

protected:
	// optional information overrides
	virtual const tiny_rom_entry *device_rom_region() const override;
	virtual void device_add_mconfig(machine_config &config) override;

	DECLARE_READ8_MEMBER( pia_r );
	DECLARE_WRITE8_MEMBER( pia_w );

protected:
	required_device<pia6821_device> m_pia;
	required_device<output_latch_device> m_cent_data_out;
	required_memory_region m_mmu_rom;

private:
	DECLARE_WRITE8_MEMBER( pia_pa_w );
	DECLARE_READ8_MEMBER( pia_pb_r );
	DECLARE_WRITE8_MEMBER( pia_pb_w );
};


// ======================> indus_gt_device

class indus_gt_device :  public c1541_device_base
{
public:
	// construction/destruction
	indus_gt_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	// optional information overrides
	virtual const tiny_rom_entry *device_rom_region() const override;
};


// device type definition
DECLARE_DEVICE_TYPE(C1540,                      c1540_device)
DECLARE_DEVICE_TYPE(C1541,                      c1541_device)
DECLARE_DEVICE_TYPE(C1541C,                     c1541c_device)
DECLARE_DEVICE_TYPE(C1541II,                    c1541ii_device)
DECLARE_DEVICE_TYPE(SX1541,                     sx1541_device)
DECLARE_DEVICE_TYPE(FSD1,                       fsd1_device)
DECLARE_DEVICE_TYPE(FSD2,                       fsd2_device)
DECLARE_DEVICE_TYPE(CSD1,                       csd1_device)
DECLARE_DEVICE_TYPE(C1541_DOLPHIN_DOS,          c1541_dolphin_dos_device)
DECLARE_DEVICE_TYPE(C1541_PROFESSIONAL_DOS_V1,  c1541_professional_dos_v1_device)
DECLARE_DEVICE_TYPE(C1541_PROLOGIC_DOS_CLASSIC, c1541_prologic_dos_classic_device)
DECLARE_DEVICE_TYPE(INDUS_GT,                   indus_gt_device)



#endif // MAME_BUS_CBMIEC_C1541_H
