// license:GPL-2.0+
// copyright-holders:Kevin Thacker,Sandro Ronco
/*********************************************************************

    kc_keyb.h

*********************************************************************/

#ifndef MAME_MACHINE_KC_KEYB_H
#define MAME_MACHINE_KC_KEYB_H

#pragma once

#define MCFG_KC_KEYBOARD_OUT_CALLBACK(_write) \
	devcb = &kc_keyboard_device::set_out_wr_callback(*device, DEVCB_##_write);

/***************************************************************************
    TYPE DEFINITIONS
***************************************************************************/

// ======================> kc_keyboard_device

class kc_keyboard_device : public device_t
{
public:
	// construction/destruction
	kc_keyboard_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	virtual ~kc_keyboard_device();

	template <class Object> static devcb_base &set_out_wr_callback(device_t &device, Object &&cb) { return downcast<kc_keyboard_device &>(device).m_write_out.set_callback(std::forward<Object>(cb)); }

	// optional information overrides
	virtual ioport_constructor device_input_ports() const override;

protected:
	/* number of pulses that can be stored */
	static constexpr unsigned TRANSMIT_BUFFER_LENGTH = 256;

	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

	void add_pulse_to_transmit_buffer(int pulse_state, int pulse_number = 1);
	void add_bit(int bit);
	void transmit_scancode(uint8_t scan_code);

private:
	static constexpr device_timer_id TIMER_TRANSMIT_PULSE = 1;

	// internal state
	emu_timer *        m_timer_transmit_pulse;
	devcb_write_line   m_write_out;

	// pulses to transmit
	struct
	{
		uint8_t data[TRANSMIT_BUFFER_LENGTH>>3];
		int pulse_sent;
		int pulse_count;
	} m_transmit_buffer;
};

// device type definition
DECLARE_DEVICE_TYPE(KC_KEYBOARD, kc_keyboard_device)

#endif // MAME_MACHINE_KC_KEYB_H
