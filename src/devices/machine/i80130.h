// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    Intel 80130 iRMX Operating System Processor emulation

**********************************************************************/

#ifndef MAME_MACHINE_I80130_H
#define MAME_MACHINE_I80130_H

#pragma once

#include "machine/pic8259.h"
#include "machine/pit8253.h"



///*************************************************************************
//  INTERFACE CONFIGURATION MACROS
///*************************************************************************

#define MCFG_I80130_IRQ_CALLBACK(_write) \
	devcb = &i80130_device::set_irq_wr_callback(*device, DEVCB_##_write);

#define MCFG_I80130_ACK_CALLBACK(_write) \
	devcb = &i80130_device::set_ack_wr_callback(*device, DEVCB_##_write);

#define MCFG_I80130_LIR_CALLBACK(_write) \
	devcb = &i80130_device::set_lir_wr_callback(*device, DEVCB_##_write);

#define MCFG_I80130_SYSTICK_CALLBACK(_write) \
	devcb = &i80130_device::set_systick_wr_callback(*device, DEVCB_##_write);

#define MCFG_I80130_DELAY_CALLBACK(_write) \
	devcb = &i80130_device::set_delay_wr_callback(*device, DEVCB_##_write);

#define MCFG_I80130_BAUD_CALLBACK(_write) \
	devcb = &i80130_device::set_baud_wr_callback(*device, DEVCB_##_write);



///*************************************************************************
//  TYPE DEFINITIONS
///*************************************************************************

// ======================> i80130_device

class i80130_device :  public device_t
{
public:
	// construction/destruction
	i80130_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	template <class Object> static devcb_base &set_irq_wr_callback(device_t &device, Object &&cb) { return downcast<i80130_device &>(device).m_write_irq.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_ack_wr_callback(device_t &device, Object &&cb) { return downcast<i80130_device &>(device).m_write_ack.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_lir_wr_callback(device_t &device, Object &&cb) { return downcast<i80130_device &>(device).m_write_lir.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_systick_wr_callback(device_t &device, Object &&cb) { return downcast<i80130_device &>(device).m_write_systick.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_delay_wr_callback(device_t &device, Object &&cb) { return downcast<i80130_device &>(device).m_write_delay.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_baud_wr_callback(device_t &device, Object &&cb) { return downcast<i80130_device &>(device).m_write_baud.set_callback(std::forward<Object>(cb)); }

	virtual DECLARE_ADDRESS_MAP(rom_map, 16);
	virtual DECLARE_ADDRESS_MAP(io_map, 16);

	uint8_t inta_r() { return m_pic->acknowledge(); }

	DECLARE_WRITE_LINE_MEMBER( ir0_w ) { m_pic->ir0_w(state); }
	DECLARE_WRITE_LINE_MEMBER( ir1_w ) { m_pic->ir1_w(state); }
	DECLARE_WRITE_LINE_MEMBER( ir2_w ) { m_pic->ir2_w(state); }
	DECLARE_WRITE_LINE_MEMBER( ir3_w ) { m_pic->ir3_w(state); }
	DECLARE_WRITE_LINE_MEMBER( ir4_w ) { m_pic->ir4_w(state); }
	DECLARE_WRITE_LINE_MEMBER( ir5_w ) { m_pic->ir5_w(state); }
	DECLARE_WRITE_LINE_MEMBER( ir6_w ) { m_pic->ir6_w(state); }
	DECLARE_WRITE_LINE_MEMBER( ir7_w ) { m_pic->ir7_w(state); }

	DECLARE_READ16_MEMBER( io_r );
	DECLARE_WRITE16_MEMBER( io_w );

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
	// optional information overrides
	virtual const tiny_rom_entry *device_rom_region() const override;
	virtual void device_add_mconfig(machine_config &config) override;

private:
	required_device<pic8259_device> m_pic;
	required_device<pit8254_device> m_pit;

	devcb_write_line m_write_irq;
	devcb_write_line m_write_ack;
	devcb_write_line m_write_lir;
	devcb_write_line m_write_systick;
	devcb_write_line m_write_delay;
	devcb_write_line m_write_baud;

	DECLARE_WRITE_LINE_MEMBER( irq_w ) { m_write_irq(state); }
	DECLARE_WRITE_LINE_MEMBER( systick_w ) { m_write_systick(state); }
	DECLARE_WRITE_LINE_MEMBER( delay_w ) { m_write_delay(state); }
	DECLARE_WRITE_LINE_MEMBER( baud_w ) { m_write_baud(state); }
};


// device type definition
DECLARE_DEVICE_TYPE(I80130, i80130_device)

#endif // MAME_MACHINE_I80130_H
