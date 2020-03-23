// license:BSD-3-Clause
// copyright-holders:Hans Ostermeyer
/*
 * apollo_kbd.h
 *
 *  Created on: Dec 27, 2010
 *      Author: Hans Ostermeyer
 *
 */

#ifndef MAME_MACHINE_APOLLO_KBD_H
#define MAME_MACHINE_APOLLO_KBD_H

#pragma once

#include "sound/beep.h"

// BSD-derived systems get very sad when you party with system reserved names.
#ifdef getchar
#undef getchar
#endif

#ifdef putchar
#undef putchar
#endif

//**************************************************************************
//  DEVICE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_APOLLO_KBD_TX_CALLBACK(_cb) \
	devcb = &apollo_kbd_device::set_tx_cb(*device, DEVCB_##_cb);

#define MCFG_APOLLO_KBD_GERMAN_CALLBACK(_cb) \
	devcb = &apollo_kbd_device::set_german_cb(*device, DEVCB_##_cb);

INPUT_PORTS_EXTERN(apollo_kbd);

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> apollo_kbd_device

class apollo_kbd_device : public device_t, public device_serial_interface
{
public:
	// construction/destruction
	apollo_kbd_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	template<class _Object> static devcb_base &set_tx_cb(device_t &device, _Object object) { return downcast<apollo_kbd_device &>(device).m_tx_w.set_callback(object); }
	template<class _Object> static devcb_base &set_german_cb(device_t &device, _Object object) { return downcast<apollo_kbd_device &>(device).m_german_r.set_callback(object); }

	devcb_write_line m_tx_w;
	devcb_read_line m_german_r;

private:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// serial overrides
	virtual void rcv_complete() override;    // Rx completed receiving byte
	virtual void tra_complete() override;    // Tx completed sending byte
	virtual void tra_callback() override;    // Tx send bit
	void input_callback(uint8_t state);

	TIMER_CALLBACK_MEMBER( kbd_scan_timer );

	const char *cpu_context() ;
	template <typename Format, typename... Params>
	void logerror(Format &&fmt, Params &&... args) const;

	void kgetchar(uint8_t data);

	int keyboard_is_german();

	void set_mode(uint16_t mode);
	void putdata(const uint8_t *data, int data_length);
	void putstring(const char *data);

	int push_scancode( uint8_t code, uint8_t repeat);
	void scan_keyboard();

	// the keyboard beeper
	class beeper
	{
	public:
		beeper();
		void start(apollo_kbd_device *device);
		void reset();
		void off();
		void on();
	private:
		int keyboard_has_beeper();
		TIMER_CALLBACK_MEMBER(beeper_callback);

		apollo_kbd_device *m_device; // pointer back to our device
		beep_device *m_beeper; // the keyboard beeper device
		emu_timer * m_timer; // timer to clock data in
	};

	// the keyboard mouse
	class mouse
	{
	public:
		mouse();
		void start(apollo_kbd_device *device);
		void reset();
		void read_mouse();
	private:
		apollo_kbd_device *m_device; // pointer back to our device

		int m_last_b;  // previous mouse button values
		int m_last_x;  // previous mouse x-axis value
		int m_last_y;  // previous mouse y-axis value
		int m_tx_pending;  // mouse data packet is pending
	};

	static const int XMIT_RING_SIZE = 64;

	uint8_t m_xmitring[XMIT_RING_SIZE];
	int m_xmit_read, m_xmit_write;
	bool m_tx_busy;

	void xmit_char(uint8_t data);

	beeper  m_beeper;
	mouse   m_mouse;

	apollo_kbd_device *m_device; // pointer to myself (nasty: used for cpu_context)

	ioport_port *m_io_keyboard1;
	ioport_port *m_io_keyboard2;
	ioport_port *m_io_keyboard3;
	ioport_port *m_io_keyboard4;
	ioport_port *m_io_mouse1;
	ioport_port *m_io_mouse2;
	ioport_port *m_io_mouse3;

	/* Receiver */
	uint32_t m_rx_message;
	uint16_t m_loopback_mode;

	emu_timer* m_timer;
	uint16_t m_mode;
	uint16_t m_delay;         // key press delay after initial press
	uint16_t m_repeat;        // key press repeat rate
	uint16_t m_last_pressed;  // last key pressed, for repeat key handling
	int m_keytime[0x80];    // time until next key press (1 ms)
	uint8_t m_keyon[0x80];    // is 1 if key is pressed

	static uint16_t m_code_table[];
};

// device type definition
DECLARE_DEVICE_TYPE(APOLLO_KBD, apollo_kbd_device)

#endif // MAME_MACHINE_APOLLO_KBD_H
