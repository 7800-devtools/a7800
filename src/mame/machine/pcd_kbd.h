// license:BSD-3-Clause
// copyright-holders:Carl
#ifndef MAME_INCLUDES_PCD_KBD_H
#define MAME_INCLUDES_PCD_KBD_H

#pragma once

#define MCFG_PCD_KEYBOARD_OUT_TX_HANDLER(_devcb) \
	devcb = &pcd_keyboard_device::set_out_tx_handler(*device, DEVCB_##_devcb);

class pcd_keyboard_device : public device_t
{
public:
	pcd_keyboard_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	template <class Object> static devcb_base &set_out_tx_handler(device_t &device, Object &&cb) { return downcast<pcd_keyboard_device &>(device).m_out_tx_handler.set_callback(std::forward<Object>(cb)); }

	DECLARE_WRITE_LINE_MEMBER( t0_w );

protected:
	virtual const tiny_rom_entry *device_rom_region() const override;
	virtual void device_add_mconfig(machine_config &config) override;
	virtual ioport_constructor device_input_ports() const override;

	void device_start() override;

private:
	required_ioport_array<17> m_rows;
	uint8_t m_p1;
	bool m_t0;
	devcb_write_line m_out_tx_handler;

	DECLARE_READ8_MEMBER( bus_r );
	DECLARE_READ8_MEMBER( p1_r );
	DECLARE_WRITE8_MEMBER( p1_w );
	DECLARE_READ_LINE_MEMBER( t0_r );
};

extern const device_type PCD_KEYBOARD;
DECLARE_DEVICE_TYPE(PCD_KEYBOARD, pcd_keyboard_device)

#endif // MAME_INCLUDES_PCD_KBD_H
