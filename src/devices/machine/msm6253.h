// license:BSD-3-Clause
// copyright-holders: AJR
/**********************************************************************

    OKI MSM6253 8-Bit 4-Channel A/D Converter

***********************************************************************
                              ____   ____
                /OSC OUT   1 |*   \_/    | 18  OSC OUT
                   D-GND   2 |           | 17  OSC IN
                   A-GND   3 |           | 16  /RD
                     IN0   4 |           | 15  /WR
                     IN1   5 | MSM6253RS | 14  ALE
                     IN2   6 |           | 13  /CS
                     IN3   7 |           | 12  A1
                      Vr   8 |           | 11  A0
                     Vdd   9 |___________| 10  S.O.

**********************************************************************/

#ifndef MAME_MACHINE_MSM6253_H
#define MAME_MACHINE_MSM6253_H

#pragma once

//**************************************************************************
//  CONFIGURATION MACROS
//**************************************************************************

#define MCFG_MSM6253_IN0_ANALOG_PORT(_input) \
	msm6253_device::static_set_input_tag(*device, 0, "^" _input);
#define MCFG_MSM6253_IN1_ANALOG_PORT(_input) \
	msm6253_device::static_set_input_tag(*device, 1, "^" _input);
#define MCFG_MSM6253_IN2_ANALOG_PORT(_input) \
	msm6253_device::static_set_input_tag(*device, 2, "^" _input);
#define MCFG_MSM6253_IN3_ANALOG_PORT(_input) \
	msm6253_device::static_set_input_tag(*device, 3, "^" _input);

#define MCFG_MSM6253_IN0_ANALOG_READ(_class, _method) \
	msm6253_device::static_set_input_cb(*device, 0, msm6253_device::port_read_delegate(&_class::_method, #_class "::" #_method, downcast<_class *>(owner)));
#define MCFG_MSM6253_IN1_ANALOG_READ(_class, _method) \
	msm6253_device::static_set_input_cb(*device, 1, msm6253_device::port_read_delegate(&_class::_method, #_class "::" #_method, downcast<_class *>(owner)));
#define MCFG_MSM6253_IN2_ANALOG_READ(_class, _method) \
	msm6253_device::static_set_input_cb(*device, 2, msm6253_device::port_read_delegate(&_class::_method, #_class "::" #_method, downcast<_class *>(owner)));
#define MCFG_MSM6253_IN3_ANALOG_READ(_class, _method) \
	msm6253_device::static_set_input_cb(*device, 3, msm6253_device::port_read_delegate(&_class::_method, #_class "::" #_method, downcast<_class *>(owner)));

#define MCFG_MSM6253_IN0_ANALOG_DEVREAD(_tag, _class, _method) \
	msm6253_device::static_set_input_cb(*device, 0, msm6253_device::port_read_delegate(&_class::_method, #_class "::" #_method, _tag));
#define MCFG_MSM6253_IN1_ANALOG_DEVREAD(_tag, _class, _method) \
	msm6253_device::static_set_input_cb(*device, 1, msm6253_device::port_read_delegate(&_class::_method, #_class "::" #_method, _tag));
#define MCFG_MSM6253_IN2_ANALOG_DEVREAD(_tag, _class, _method) \
	msm6253_device::static_set_input_cb(*device, 2, msm6253_device::port_read_delegate(&_class::_method, #_class "::" #_method, _tag));
#define MCFG_MSM6253_IN3_ANALOG_DEVREAD(_tag, _class, _method) \
	msm6253_device::static_set_input_cb(*device, 3, msm6253_device::port_read_delegate(&_class::_method, #_class "::" #_method, _tag));


//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> msm6253_device

class msm6253_device : public device_t
{
public:
	typedef device_delegate<ioport_value ()> port_read_delegate;

	// construction/destruction
	msm6253_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	// static configuration
	static void static_set_input_tag(device_t &device, int port, const char *tag) { downcast<msm6253_device &>(device).m_analog_ports[port].set_tag(tag); }
	static void static_set_input_cb(device_t &device, int port, port_read_delegate &&cb) { downcast<msm6253_device &>(device).m_analog_input_cb[port] = std::move(cb); }

	// write handlers
	WRITE8_MEMBER(address_w);
	WRITE8_MEMBER(select_w);

	// read handlers
	bool shift_out();
	READ8_MEMBER(d0_r);
	READ8_MEMBER(d7_r);

protected:
	// device-level overrides
	virtual void device_start() override;

private:
	// helpers
	template<int port> ioport_value port_read();

	// input configuration
	optional_ioport_array<4> m_analog_ports;
	port_read_delegate m_analog_input_cb[4];

	// private data
	u8 m_shift_register;
};

// device type definition
DECLARE_DEVICE_TYPE(MSM6253, msm6253_device)

#endif // DEVICES_MACHINE_MSM6253_H
