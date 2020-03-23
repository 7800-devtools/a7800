// license:GPL-2.0+
// copyright-holders:Dirk Best
/**********************************************************************

    EPSON PF-10

    Battery operated portable 3.5" floppy drive

**********************************************************************/

#ifndef MAME_BUS_EPSON_SIO_PF10_H
#define MAME_BUS_EPSON_SIO_PF10_H

#pragma once

#include "epson_sio.h"
#include "cpu/m6800/m6801.h"
#include "machine/upd765.h"


//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

class epson_pf10_device : public device_t,
							public device_epson_sio_interface
{
public:
	// construction/destruction
	epson_pf10_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// floppy disk controller
	DECLARE_READ8_MEMBER( fdc_r );
	DECLARE_WRITE8_MEMBER( fdc_w );
	DECLARE_WRITE8_MEMBER( fdc_tc_w );

	// hd6303 i/o
	DECLARE_READ8_MEMBER( port1_r );
	DECLARE_WRITE8_MEMBER( port1_w );
	DECLARE_READ8_MEMBER( port2_r );
	DECLARE_WRITE8_MEMBER( port2_w );

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

	// optional information overrides
	virtual const tiny_rom_entry *device_rom_region() const override;
	virtual void device_add_mconfig(machine_config &config) override;

	// device_epson_sio_interface overrides
	virtual void tx_w(int level) override;
	virtual void pout_w(int level) override;

private:
	// serial output from main cpu
	DECLARE_WRITE_LINE_MEMBER( hd6303_tx_w );

	// from sio output
	DECLARE_WRITE_LINE_MEMBER( rxc_w );
	DECLARE_WRITE_LINE_MEMBER( pinc_w );


	required_device<hd6303y_cpu_device> m_cpu;
	required_device<upd765a_device> m_fdc;
	required_device<epson_sio_device> m_sio_output;

	epson_sio_device *m_sio_input;
	floppy_image_device *m_floppy;

	emu_timer *m_timer;

	uint8_t m_port1;
	uint8_t m_port2;

	int m_rxc;
	int m_hd6303_tx;
	int m_pinc;

	// port definitions
	enum
	{
		PORT2_SEEK    = 0x01,
		PORT2_SWCOM   = 0x02, // ?
		PORT2_RS232ON = 0x04, // to nmi?
		PORT2_RXD     = 0x08,
		PORT2_TXD     = 0x10,
		PORT2_FDCRST  = 0x20,
		PORT2_MON     = 0x40,
		PORT2_BATCKEN = 0x80  // ?
	};
};


// device type definition
DECLARE_DEVICE_TYPE(EPSON_PF10, epson_pf10_device)


#endif // MAME_BUS_EPSON_SIO_PF10_H
