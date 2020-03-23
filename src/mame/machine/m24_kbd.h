// license:BSD-3-Clause
// copyright-holders:Carl
#ifndef MAME_MACHINE_M24_KBD_H
#define MAME_MACHINE_M24_KBD_H

#pragma once

#include "cpu/mcs48/mcs48.h"

#define MCFG_M24_KEYBOARD_OUT_DATA_HANDLER(_devcb) \
	devcb = &m24_keyboard_device::set_out_data_handler(*device, DEVCB_##_devcb);

class m24_keyboard_device :  public device_t
{
public:
	m24_keyboard_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	template <class Object> static devcb_base &set_out_data_handler(device_t &device, Object &&cb) { return downcast<m24_keyboard_device &>(device).m_out_data.set_callback(std::forward<Object>(cb)); }

	DECLARE_WRITE_LINE_MEMBER(clock_w);
	DECLARE_WRITE_LINE_MEMBER(data_w);

protected:
	void device_start() override;
	void device_reset() override;
	void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;
	virtual const tiny_rom_entry *device_rom_region() const override;
	virtual void device_add_mconfig(machine_config &config) override;
	virtual ioport_constructor device_input_ports() const override;

private:
	required_ioport_array<16> m_rows;
	required_ioport m_mousebtn;
	uint8_t m_p1;
	bool m_keypress, m_kbcdata;
	devcb_write_line m_out_data;
	required_device<cpu_device> m_mcu;
	emu_timer *m_reset_timer;

	DECLARE_WRITE8_MEMBER(bus_w);
	DECLARE_READ8_MEMBER(p1_r);
	DECLARE_WRITE8_MEMBER(p1_w);
	DECLARE_READ8_MEMBER(p2_r);
	DECLARE_READ_LINE_MEMBER(t0_r);
	DECLARE_READ_LINE_MEMBER(t1_r);
};

DECLARE_DEVICE_TYPE(M24_KEYBOARD, m24_keyboard_device)

#endif // MAME_MACHINE_M24_KBD_H
