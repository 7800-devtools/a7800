// license:BSD-3-Clause
// copyright-holders:Barry Rodewald,Vas Crabb
#ifndef MAME_MACHINE_X68K_KBD_H
#define MAME_MACHINE_X68K_KBD_H

#include "bus/rs232/keyboard.h"

class x68k_keyboard_device : public buffered_rs232_device<16U>, protected device_matrix_keyboard_interface<15U>
{
public:
	x68k_keyboard_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	virtual ioport_constructor device_input_ports() const override;

protected:
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void key_make(uint8_t row, uint8_t column) override;
	virtual void key_repeat(uint8_t row, uint8_t column) override;
	virtual void key_break(uint8_t row, uint8_t column) override;

private:
	virtual void received_byte(uint8_t data) override;

	int m_delay;  // keypress delay after initial press
	int m_repeat; // keypress repeat rate
	uint8_t m_enabled;  // keyboard enabled?
};

DECLARE_DEVICE_TYPE(X68K_KEYBOARD, x68k_keyboard_device)

#endif // MAME_MACHINE_X68K_KBD_H
