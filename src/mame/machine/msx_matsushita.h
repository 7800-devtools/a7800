// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
#ifndef MAME_MACHINE_MSX_MATSUSHITA_H
#define MAME_MACHINE_MSX_MATSUSHITA_H


#include "msx_switched.h"


DECLARE_DEVICE_TYPE(MSX_MATSUSHITA, msx_matsushita_device)


#define MCFG_MSX_MATSUSHITA_ADD(_tag) \
	MCFG_DEVICE_ADD(_tag, MSX_MATSUSHITA, 0)

#define MCFG_MSX_MATSUSHITA_TURBO_CB(_devcb) \
	devcb = &msx_matsushita_device::set_turbo_callback(*device, DEVCB_##_devcb);


class msx_matsushita_device : public device_t,
	public msx_switched_interface,
	public device_nvram_interface
{
public:
	msx_matsushita_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	template <class Object> static devcb_base &set_turbo_callback(device_t &device, Object &&cb)
	{ return downcast<msx_matsushita_device &>(device).m_turbo_out_cb.set_callback(std::forward<Object>(cb)); }

	virtual DECLARE_READ8_MEMBER(switched_read) override;
	virtual DECLARE_WRITE8_MEMBER(switched_write) override;

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual ioport_constructor device_input_ports() const override;

	// device_nvram_interface overrides
	virtual void nvram_default() override;
	virtual void nvram_read(emu_file &file) override;
	virtual void nvram_write(emu_file &file) override;

private:
	required_ioport m_io_config;
	devcb_write_line m_turbo_out_cb;
	bool m_selected;
	uint16_t m_address;
	std::vector<uint8_t> m_sram;
	uint8_t m_nibble1;
	uint8_t m_nibble2;
	uint8_t m_pattern;
};

#endif // MAME_MACHINE_MSX_MATSUSHITA_H
