// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    IMI 5000H 5.25" Winchester Hard Disk Controller emulation

    Used in Corvus Systems H-Series drives (Model 6/11/20)

**********************************************************************/

#ifndef MAME_BUS_IMI7000_IMI5000H_H
#define MAME_BUS_IMI7000_IMI5000H_H

#pragma once

#include "imi7000.h"
#include "cpu/z80/z80.h"
#include "cpu/z80/z80daisy.h"
#include "machine/z80ctc.h"
#include "machine/z80pio.h"



//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> imi5000h_device

class imi5000h_device :  public device_t,
							public device_imi7000_interface
{
public:
	// construction/destruction
	imi5000h_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// optional information overrides
	virtual const tiny_rom_entry *device_rom_region() const override;
	virtual void device_add_mconfig(machine_config &config) override;
	virtual ioport_constructor device_input_ports() const override;

private:
	DECLARE_WRITE_LINE_MEMBER( ctc_z0_w );
	DECLARE_WRITE_LINE_MEMBER( ctc_z1_w );
	DECLARE_WRITE_LINE_MEMBER( ctc_z2_w );

	DECLARE_READ8_MEMBER( pio0_pa_r );
	DECLARE_WRITE8_MEMBER( pio0_pa_w );
	DECLARE_READ8_MEMBER( pio0_pb_r );
	DECLARE_WRITE8_MEMBER( pio0_pb_w );

	DECLARE_READ8_MEMBER( pio2_pa_r );
	DECLARE_WRITE8_MEMBER( pio2_pa_w );
	DECLARE_READ8_MEMBER( pio2_pb_r );
	DECLARE_WRITE8_MEMBER( pio2_pb_w );

	DECLARE_READ8_MEMBER( pio3_pa_r );
	DECLARE_WRITE8_MEMBER( pio3_pa_w );
	DECLARE_READ8_MEMBER( pio3_pb_r );
	DECLARE_WRITE8_MEMBER( pio3_pb_w );

	enum
	{
		LED_FAULT,
		LED_BUSY,
		LED_READY
	};

	required_device<cpu_device> m_maincpu;
	required_device<z80ctc_device> m_ctc;
	required_ioport m_lsi11;
	required_ioport m_mux;
	required_ioport m_format;
	required_ioport m_ub4;
};


// device type definition
extern const device_type IMI5000H;
DECLARE_DEVICE_TYPE(IMI5000H, imi5000h_device)

#endif // MAME_BUS_IMI7000_IMI5000H_H
