// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    Wang PC keyboard emulation

*********************************************************************/

#ifndef MAME_MACHINE_WANGPCKB_H
#define MAME_MACHINE_WANGPCKB_H

#pragma once


#include "cpu/mcs51/mcs51.h"
#include "sound/sn76496.h"



//**************************************************************************
//  MACROS / CONSTANTS
//**************************************************************************

#define WANGPC_KEYBOARD_TAG "wangpckb"



//**************************************************************************
//  INTERFACE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_WANGPC_KEYBOARD_ADD() \
	MCFG_DEVICE_ADD(WANGPC_KEYBOARD_TAG, WANGPC_KEYBOARD, 0)

#define MCFG_WANGPCKB_TXD_HANDLER(_devcb) \
	devcb = &wangpc_keyboard_device::set_txd_handler(*device, DEVCB_##_devcb);


//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> wangpc_keyboard_device

class wangpc_keyboard_device :  public device_t,
						   public device_serial_interface
{
public:
	// construction/destruction
	wangpc_keyboard_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	template <class Object> static devcb_base &set_txd_handler(device_t &device, Object &&cb) { return downcast<wangpc_keyboard_device &>(device).m_txd_handler.set_callback(std::forward<Object>(cb)); }

	DECLARE_WRITE_LINE_MEMBER( write_rxd );

	// not really public
	DECLARE_READ8_MEMBER( kb_p1_r );
	DECLARE_WRITE8_MEMBER( kb_p1_w );
	DECLARE_WRITE8_MEMBER( kb_p2_w );
	DECLARE_WRITE8_MEMBER( kb_p3_w );

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// optional information overrides
	virtual const tiny_rom_entry *device_rom_region() const override;
	virtual void device_add_mconfig(machine_config &config) override;
	virtual ioport_constructor device_input_ports() const override;

	// device_serial_interface overrides
	virtual void tra_callback() override;
	virtual void tra_complete() override;
	virtual void rcv_callback() override;
	virtual void rcv_complete() override;

private:
	required_device<i8051_device> m_maincpu;
	required_ioport_array<16> m_y;
	devcb_write_line m_txd_handler;

	uint8_t m_keylatch;
	int m_rxd;

	DECLARE_READ8_MEMBER( mcs51_rx_callback );
	DECLARE_WRITE8_MEMBER( mcs51_tx_callback );
};


// device type definition
DECLARE_DEVICE_TYPE(WANGPC_KEYBOARD, wangpc_keyboard_device)

#endif // MAME_MACHINE_WANGPCKB_H
