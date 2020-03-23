// license:BSD-3-Clause
// copyright-holders:Angelo Salese
/***************************************************************************

    PC-9801 Keyboard simulation

***************************************************************************/
#ifndef MAME_MACHINE_PC9801_KBD_H
#define MAME_MACHINE_PC9801_KBD_H

#pragma once



//**************************************************************************
//  INTERFACE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_PC9801_KBD_IRQ_CALLBACK(_write) \
	devcb = &pc9801_kbd_device::set_irq_wr_callback(*device, DEVCB_##_write);


//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> pc9801_kbd_device

class pc9801_kbd_device : public device_t
{
public:
	// construction/destruction
	pc9801_kbd_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	template <class Object> static devcb_base &set_irq_wr_callback(device_t &device, Object &&cb) { return downcast<pc9801_kbd_device &>(device).m_write_irq.set_callback(std::forward<Object>(cb)); }

	virtual ioport_constructor device_input_ports() const override;

	// I/O operations
	DECLARE_WRITE8_MEMBER( tx_w );
	DECLARE_READ8_MEMBER( rx_r );
	DECLARE_INPUT_CHANGED_MEMBER(key_stroke);

protected:
	// device-level overrides
	virtual void device_validity_check(validity_checker &valid) const override;
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

	devcb_write_line   m_write_irq;

	static const device_timer_id RX_TIMER = 1;
	emu_timer *         m_rxtimer;
	uint8_t               m_rx_buf[0x80];
	uint8_t               m_keyb_tx;
	uint8_t               m_keyb_rx;
	bool                m_key_avail;
};


// device type definition
DECLARE_DEVICE_TYPE(PC9801_KBD, pc9801_kbd_device)



//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************



#endif // MAME_MACHINE_PC9801_KBD_H
