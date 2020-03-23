// license:LGPL-2.1+
// copyright-holders:Michael Zapf
/****************************************************************************

    TI-99/4(A) databus multiplexer circuit
    See datamux.c for documentation

    Michael Zapf

    February 2012: Rewritten as class

*****************************************************************************/

#ifndef MAME_BUS_TI99_INTERNAL_DATAMUX_H
#define MAME_BUS_TI99_INTERNAL_DATAMUX_H

#pragma once

#include "bus/ti99/ti99defs.h"
#include "machine/tmc0430.h"
#include "bus/ti99/gromport/gromport.h"
#include "bus/ti99/internal/ioport.h"
#include "sound/sn76496.h"
#include "video/tms9928a.h"
#include "machine/ram.h"

namespace bus { namespace ti99 { namespace internal {

/*
    Main class
*/
class datamux_device : public device_t
{
public:
	datamux_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	DECLARE_READ16_MEMBER( read );
	DECLARE_WRITE16_MEMBER( write );
	DECLARE_SETOFFSET_MEMBER( setoffset );

	DECLARE_WRITE_LINE_MEMBER( clock_in );
	DECLARE_WRITE_LINE_MEMBER( dbin_in );
	DECLARE_WRITE_LINE_MEMBER( ready_line );

	DECLARE_WRITE_LINE_MEMBER( gromclk_in );

	template <class Object> static devcb_base &static_set_ready_callback(device_t &device, Object &&cb)
	{
		return downcast<datamux_device &>(device).m_ready.set_callback(std::forward<Object>(cb));
	}

protected:
	/* Constructor */
	void device_start() override;
	void device_stop() override;
	void device_reset() override;
	void device_config_complete() override;
	ioport_constructor device_input_ports() const override;

private:
	// Link to the video processor
	optional_device<tms9928a_device> m_video;

	// Link to the sound processor
	optional_device<sn76496_base_device> m_sound;

	// Link to the I/O port
	required_device<bus::ti99::internal::ioport_device> m_ioport;

	// Link to the cartridge port (aka GROM port)
	required_device<bus::ti99::gromport::gromport_device> m_gromport;

	// Memory expansion (internal, 16 bit)
	required_device<ram_device> m_ram16b;

	// Console RAM
	required_device<ram_device> m_padram;

	// Keeps the address space pointer
	address_space* m_spacep;

	// Console ROM
	uint16_t* m_consolerom;

	// Console GROMs
	tmc0430_device* m_grom[3];

	// Common read routine
	void read_all(address_space& space, uint16_t addr, uint8_t *target);

	// Common write routine
	void write_all(address_space& space, uint16_t addr, uint8_t value);

	// Common set address method
	void setaddress_all(address_space& space, uint16_t addr);

	// Debugger access
	uint16_t debugger_read(address_space& space, uint16_t addr);
	void debugger_write(address_space& space, uint16_t addr, uint16_t data);

	// Join own READY and external READY
	void ready_join();

	// Ready line to the CPU
	devcb_write_line m_ready;

	// Address latch (emu). In reality, the address bus remains constant.
	uint16_t m_addr_buf;

	// DBIN line
	int m_dbin;

	// Own ready state.
	int  m_muxready;

	// Ready state. Needed to control wait state generation via inbound READY
	int  m_sysready;

	// Latch which stores the first (odd) byte
	uint8_t m_latch;

	// Counter for the wait states.
	int   m_waitcount;

	// Use the memory expansion?
	bool m_use32k;

	// Memory base for piggy-back 32K expansion. If 0, expansion is not used.
	uint16_t  m_base32k;

	// Console GROMs are available (the HSGPL expects them to be removed)
	bool m_console_groms_present;

	// GROMs are idle, no need to propagate the clock
	bool m_grom_idle;
};

/******************************************************************************/

#define MCFG_DMUX_READY_HANDLER( _intcallb ) \
	devcb = &bus::ti99::internal::datamux_device::static_set_ready_callback( *device, DEVCB_##_intcallb );

} } } // end namespace bus::ti99::internal

DECLARE_DEVICE_TYPE_NS(TI99_DATAMUX, bus::ti99::internal, datamux_device)

#endif // MAME_BUS_TI99_INTERNAL_DATAMUX_H
