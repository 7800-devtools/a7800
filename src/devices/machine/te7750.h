// license:BSD-3-Clause
// copyright-holders:AJR
/**********************************************************************

    TE7750 Super I/O Expander

**********************************************************************/

#ifndef DEVICES_MACHINE_TE7750_H
#define DEVICES_MACHINE_TE7750_H

#pragma once

//**************************************************************************
//  CONFIGURATION MACROS
//**************************************************************************

#define MCFG_TE7750_IN_PORT1_CB(_devcb) \
	devcb = &te7750_device::set_input_cb(*device, 0, DEVCB_##_devcb);
#define MCFG_TE7750_IN_PORT2_CB(_devcb) \
	devcb = &te7750_device::set_input_cb(*device, 1, DEVCB_##_devcb);
#define MCFG_TE7750_IN_PORT3_CB(_devcb) \
	devcb = &te7750_device::set_input_cb(*device, 2, DEVCB_##_devcb);
#define MCFG_TE7750_IN_PORT4_CB(_devcb) \
	devcb = &te7750_device::set_input_cb(*device, 3, DEVCB_##_devcb);
#define MCFG_TE7750_IN_PORT5_CB(_devcb) \
	devcb = &te7750_device::set_input_cb(*device, 4, DEVCB_##_devcb);
#define MCFG_TE7750_IN_PORT6_CB(_devcb) \
	devcb = &te7750_device::set_input_cb(*device, 5, DEVCB_##_devcb);
#define MCFG_TE7750_IN_PORT7_CB(_devcb) \
	devcb = &te7750_device::set_input_cb(*device, 6, DEVCB_##_devcb);
#define MCFG_TE7750_IN_PORT8_CB(_devcb) \
	devcb = &te7750_device::set_input_cb(*device, 7, DEVCB_##_devcb);
#define MCFG_TE7750_IN_PORT9_CB(_devcb) \
	devcb = &te7750_device::set_input_cb(*device, 8, DEVCB_##_devcb);

#define MCFG_TE7750_OUT_PORT1_CB(_devcb) \
	devcb = &te7750_device::set_output_cb(*device, 0, DEVCB_##_devcb);
#define MCFG_TE7750_OUT_PORT2_CB(_devcb) \
	devcb = &te7750_device::set_output_cb(*device, 1, DEVCB_##_devcb);
#define MCFG_TE7750_OUT_PORT3_CB(_devcb) \
	devcb = &te7750_device::set_output_cb(*device, 2, DEVCB_##_devcb);
#define MCFG_TE7750_OUT_PORT4_CB(_devcb) \
	devcb = &te7750_device::set_output_cb(*device, 3, DEVCB_##_devcb);
#define MCFG_TE7750_OUT_PORT5_CB(_devcb) \
	devcb = &te7750_device::set_output_cb(*device, 4, DEVCB_##_devcb);
#define MCFG_TE7750_OUT_PORT6_CB(_devcb) \
	devcb = &te7750_device::set_output_cb(*device, 5, DEVCB_##_devcb);
#define MCFG_TE7750_OUT_PORT7_CB(_devcb) \
	devcb = &te7750_device::set_output_cb(*device, 6, DEVCB_##_devcb);
#define MCFG_TE7750_OUT_PORT8_CB(_devcb) \
	devcb = &te7750_device::set_output_cb(*device, 7, DEVCB_##_devcb);
#define MCFG_TE7750_OUT_PORT9_CB(_devcb) \
	devcb = &te7750_device::set_output_cb(*device, 8, DEVCB_##_devcb);

#define MCFG_TE7750_IOS_CB(_devcb) \
	devcb = &te7750_device::set_ios_cb(*device, DEVCB_##_devcb);

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> te7750_device

class te7750_device : public device_t
{
public:
	// construction/destruction
	te7750_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	// static configuration
	template<class Object>
	static devcb_base &set_input_cb(device_t &device, int port, Object &&obj)
	{
		assert(port >= 0 && port < 9);
		return downcast<te7750_device &>(device).m_input_cb[port].set_callback(std::forward<Object>(obj));
	}
	template<class Object>
	static devcb_base &set_output_cb(device_t &device, int port, Object &&obj)
	{
		assert(port >= 0 && port < 9);
		return downcast<te7750_device &>(device).m_output_cb[port].set_callback(std::forward<Object>(obj));
	}
	template<class Object>
	static devcb_base &set_ios_cb(device_t &device, Object &&obj)
	{
		return downcast<te7750_device &>(device).m_ios_cb.set_callback(std::forward<Object>(obj));
	}

	// bus-compatible interface
	DECLARE_READ8_MEMBER(read);
	DECLARE_WRITE8_MEMBER(write);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

private:
	// internal helpers
	void set_port_dir(int port, u8 dir);
	void set_ios();

	// input/output callbacks
	devcb_read8         m_input_cb[9];
	devcb_write8        m_output_cb[9];

	// mode callback
	devcb_read8         m_ios_cb;

	// internal state
	u8                  m_data_latch[9];
	u8                  m_data_dir[9];
};

// device type definition
DECLARE_DEVICE_TYPE(TE7750, te7750_device)

#endif // DEVICES_MACHINE_TE7750_H
