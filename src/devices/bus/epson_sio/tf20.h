// license:GPL-2.0+
// copyright-holders:Dirk Best
/**********************************************************************

    EPSON TF-20

    Dual 5.25" floppy drive with HX-20 factory option

**********************************************************************/

#ifndef MAME_BUS_EPSON_SIO_TF20_H
#define MAME_BUS_EPSON_SIO_TF20_H

#pragma once

#include "epson_sio.h"
#include "cpu/z80/z80.h"
#include "machine/ram.h"
#include "machine/upd765.h"
#include "machine/z80dart.h"


//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

class epson_tf20_device : public device_t,
							public device_epson_sio_interface
{
public:
	// construction/destruction
	epson_tf20_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// not really public
	DECLARE_READ8_MEMBER( rom_disable_r );
	DECLARE_READ8_MEMBER( upd765_tc_r );
	DECLARE_WRITE8_MEMBER( fdc_control_w );

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

	// optional information overrides
	virtual const tiny_rom_entry *device_rom_region() const override;
	virtual void device_add_mconfig(machine_config &config) override;
	virtual ioport_constructor device_input_ports() const override;

	// device_epson_sio_interface overrides
	virtual void tx_w(int level) override;
	virtual void pout_w(int level) override;

private:
	IRQ_CALLBACK_MEMBER( irq_callback );
	DECLARE_WRITE_LINE_MEMBER( txda_w );
	DECLARE_WRITE_LINE_MEMBER( dtra_w );

	// from sio output
	DECLARE_WRITE_LINE_MEMBER( rxc_w );
	DECLARE_WRITE_LINE_MEMBER( pinc_w );

	required_device<cpu_device> m_cpu;
	required_device<ram_device> m_ram;
	required_device<upd765a_device> m_fdc;
	required_device<upd7201_device> m_mpsc;
	required_device<epson_sio_device> m_sio_output;

	floppy_image_device *m_fd0;
	floppy_image_device *m_fd1;

	emu_timer *m_timer_serial;
	emu_timer *m_timer_tc;

	int m_rxc;
	int m_txda;
	int m_dtra;
	int m_pinc;

	epson_sio_device *m_sio_input;

	static const int XTAL_CR1 = XTAL_8MHz;
	static const int XTAL_CR2 = XTAL_4_9152MHz;
};


// device type definition
DECLARE_DEVICE_TYPE(EPSON_TF20, epson_tf20_device)


#endif // MAME_BUS_EPSON_SIO_TF20_H
