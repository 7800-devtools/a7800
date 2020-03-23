// license:BSD-3-Clause
// copyright-holders:R. Belmont
#ifndef MAME_MACHINE_MACKBD_H
#define MAME_MACHINE_MACKBD_H

#pragma once



//**************************************************************************
//  INTERFACE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_MACKBD_ADD(_tag) \
	MCFG_DEVICE_ADD(_tag, MACKBD, 0)

#define MCFG_MACKBD_REPLACE(_tag) \
	MCFG_DEVICE_REPLACE(_tag, MACKBD, 0)

#define MCFG_MACKBD_REMOVE(_tag) \
	MCFG_DEVICE_REMOVE(_tag)

#define MCFG_MACKBD_CLKOUT_HANDLER(_devcb) \
	devcb = &mackbd_device::set_clkout_handler(*device, DEVCB_##_devcb);

#define MCFG_MACKBD_DATAOUT_HANDLER(_devcb) \
	devcb = &mackbd_device::set_dataout_handler(*device, DEVCB_##_devcb);

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> mackbd_device

class mackbd_device :  public device_t
{
public:
	// static config helper
	template<class _Object> static devcb_base &set_clkout_handler(device_t &device, _Object object) { return downcast<mackbd_device &>(device).m_clkout_handler.set_callback(object); }
	template<class _Object> static devcb_base &set_dataout_handler(device_t &device, _Object object) { return downcast<mackbd_device &>(device).m_dataout_handler.set_callback(object); }

	// construction/destruction
	mackbd_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	DECLARE_WRITE8_MEMBER(p0_w);

	DECLARE_WRITE_LINE_MEMBER(data_w);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_add_mconfig(machine_config &config) override;
	virtual const tiny_rom_entry *device_rom_region() const override;
	virtual ioport_constructor device_input_ports() const override;

	required_device<cpu_device> m_maincpu;

private:
	uint8_t p0, p1, p2, data_from_mac, data_to_mac;

	devcb_write_line m_clkout_handler;
	devcb_write_line m_dataout_handler;

	void scan_kbd_col(int col);

	DECLARE_READ8_MEMBER(p0_r);
	DECLARE_READ8_MEMBER(p1_r);
	DECLARE_WRITE8_MEMBER(p1_w);
	DECLARE_READ8_MEMBER(p2_r);
	DECLARE_WRITE8_MEMBER(p2_w);
};

// device type definition
DECLARE_DEVICE_TYPE(MACKBD, mackbd_device)

#endif // MAME_MACHINE_MACKBD_H
