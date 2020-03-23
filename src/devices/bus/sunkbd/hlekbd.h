// license:BSD-3-Clause
// copyright-holders:Vas Crabb
#ifndef MAME_DEVICES_SUNKBD_HLEKBD_H
#define MAME_DEVICES_SUNKBD_HLEKBD_H

#pragma once

#include "sunkbd.h"
#include "machine/keyboard.h"
#include "sound/beep.h"


namespace bus { namespace sunkbd {

class hle_device_base
	: public device_t
	, public device_buffered_serial_interface<16U>
	, public device_sun_keyboard_port_interface
	, protected device_matrix_keyboard_interface<8U>
{
public:
	virtual ~hle_device_base() override;

	virtual DECLARE_WRITE_LINE_MEMBER( input_txd ) override;

protected:
	// constructor/destructor
	hle_device_base(
			machine_config const &mconfig,
			device_type type,
			char const *tag,
			device_t *owner,
			uint32_t clock);

	// device overrides
	virtual void device_add_mconfig(machine_config &config) override;
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

	// device_buffered_serial_interface overrides
	virtual void tra_callback() override;
	virtual void tra_complete() override;

	// device_matrix_keyboard_interface overrides
	virtual void key_make(uint8_t row, uint8_t column) override;
	virtual void key_break(uint8_t row, uint8_t column) override;

	// customised transmit_byte method
	void transmit_byte(uint8_t byte);

	required_ioport m_dips;

private:
	enum {
		CLICK_TIMER_ID = 30'000
	};

	// TODO: ensure these don't clash with diagnostic LEDs on host computer
	enum : int {
		LED_NUM = 0,
		LED_COMPOSE,
		LED_SCROLL,
		LED_CAPS,
		LED_KANA
	};

	enum : uint8_t {
		BEEPER_BELL = 0x01U,
		BEEPER_CLICK = 0x02U
	};

	enum : uint8_t {
		RX_IDLE,
		RX_LED
	};

	enum : uint8_t {
		COMMAND_RESET = 0x01U,
		COMMAND_BELL_ON = 0x02U,
		COMMAND_BELL_OFF = 0x03U,
		COMMAND_CLICK_ON = 0x0aU,
		COMMAND_CLICK_OFF = 0x0bU,
		COMMAND_LED = 0x0eU,
		COMMAND_LAYOUT = 0x0fU
	};

	// device_buffered_serial_interface overrides
	virtual void received_byte(uint8_t byte) override;

	virtual uint8_t ident_byte() = 0;

	emu_timer                       *m_click_timer;
	required_device<beep_device>    m_beeper;

	uint8_t   m_make_count;
	uint8_t   m_rx_state;

	uint8_t   m_keyclick;
	uint8_t   m_beeper_state;
};


class hle_type4_device_base : public hle_device_base
{
protected:
	using hle_device_base::hle_device_base;

private:
	virtual uint8_t ident_byte() override;
};


class hle_type3_device : public hle_device_base
{
public:
	hle_type3_device(
			machine_config const &mconfig,
			char const *tag,
			device_t *owner,
			uint32_t clock);

	virtual ioport_constructor device_input_ports() const override;

private:
	virtual uint8_t ident_byte() override;
};


class hle_type4_device : public hle_type4_device_base
{
public:
	hle_type4_device(
			machine_config const &mconfig,
			char const *tag,
			device_t *owner,
			uint32_t clock);

	virtual ioport_constructor device_input_ports() const override;
};


class hle_type5_device : public hle_type4_device_base
{
public:
	hle_type5_device(
			machine_config const &mconfig,
			char const *tag,
			device_t *owner,
			uint32_t clock);

	virtual ioport_constructor device_input_ports() const override;
};


class hle_type5_gb_device : public hle_type4_device_base
{
public:
	hle_type5_gb_device(
			machine_config const &mconfig,
			char const *tag,
			device_t *owner,
			uint32_t clock);

	virtual ioport_constructor device_input_ports() const override;
};


class hle_type5_se_device : public hle_type4_device_base
{
public:
	hle_type5_se_device(
			machine_config const &mconfig,
			char const *tag,
			device_t *owner,
			uint32_t clock);

	virtual ioport_constructor device_input_ports() const override;
};


class hle_type5_jp_device : public hle_type4_device_base
{
public:
	hle_type5_jp_device(
			machine_config const &mconfig,
			char const *tag,
			device_t *owner,
			uint32_t clock);

	virtual ioport_constructor device_input_ports() const override;
};

} } // namespace bus::sunkbd


DECLARE_DEVICE_TYPE_NS(SUN_TYPE3_HLE_KEYBOARD,    bus::sunkbd, hle_type3_device)
DECLARE_DEVICE_TYPE_NS(SUN_TYPE4_HLE_KEYBOARD,    bus::sunkbd, hle_type4_device)
DECLARE_DEVICE_TYPE_NS(SUN_TYPE5_HLE_KEYBOARD,    bus::sunkbd, hle_type5_device)
DECLARE_DEVICE_TYPE_NS(SUN_TYPE5_GB_HLE_KEYBOARD, bus::sunkbd, hle_type5_gb_device)
DECLARE_DEVICE_TYPE_NS(SUN_TYPE5_SE_HLE_KEYBOARD, bus::sunkbd, hle_type5_se_device)
DECLARE_DEVICE_TYPE_NS(SUN_TYPE5_JP_HLE_KEYBOARD, bus::sunkbd, hle_type5_jp_device)

#endif // MAME_DEVICES_SUNKBD_HLEKBD_H
