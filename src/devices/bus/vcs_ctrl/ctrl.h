// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    Atari Video Computer System controller port emulation

**********************************************************************


**********************************************************************/

#ifndef MAME_BUS_CVS_CTRL_CTRL_H
#define MAME_BUS_CVS_CTRL_CTRL_H

#pragma once




//**************************************************************************
//  INTERFACE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_VCS_CONTROL_PORT_ADD(_tag, _slot_intf, _def_slot) \
	MCFG_DEVICE_ADD(_tag, VCS_CONTROL_PORT, 0) \
	MCFG_DEVICE_SLOT_INTERFACE(_slot_intf, _def_slot, false)


#define MCFG_VCS_CONTROL_PORT_TRIGGER_CALLBACK(_write) \
	devcb = &vcs_control_port_device::set_trigger_wr_callback(*device, DEVCB_##_write);



//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

class vcs_control_port_device;


// ======================> device_vcs_control_port_interface

class device_vcs_control_port_interface : public device_slot_card_interface
{
public:
	// construction/destruction
	device_vcs_control_port_interface(const machine_config &mconfig, device_t &device);
	virtual ~device_vcs_control_port_interface() { }

	virtual uint8_t vcs_joy_r() { return 0xff; };
	virtual uint8_t vcs_pot_x_r() { return 0xff; };
	virtual uint8_t vcs_pot_y_r() { return 0xff; };
	virtual void vcs_joy_w(uint8_t data) { };

	virtual bool has_pot_x() { return false; }
	virtual bool has_pot_y() { return false; }

protected:
	vcs_control_port_device *m_port;
};


// ======================> vcs_control_port_device

class vcs_control_port_device : public device_t,
								public device_slot_interface
{
public:
	// construction/destruction
	vcs_control_port_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// static configuration helpers
	template <class Object> static devcb_base &set_trigger_wr_callback(device_t &device, Object &&cb) { return downcast<vcs_control_port_device &>(device).m_write_trigger.set_callback(std::forward<Object>(cb)); }

	// computer interface

	// Data returned by the joy_r methods:
	// bit 0 - pin 1 - Up
	// bit 1 - pin 2 - Down
	// bit 2 - pin 3 - Left
	// bit 3 - pin 4 - Right
	// bit 4 - pin 5 -
	// bit 5 - pin 6 - Button
	//         pin 7 - +5V
	//         pin 8 - GND
	// bit 6 - pin 9 -
	//
	uint8_t joy_r() { uint8_t data = 0xff; if (exists()) data = m_device->vcs_joy_r(); return data; }
	DECLARE_READ8_MEMBER( joy_r ) { return joy_r(); }
	uint8_t pot_x_r() { uint8_t data = 0xff; if (exists()) data = m_device->vcs_pot_x_r(); return data; }
	DECLARE_READ8_MEMBER( pot_x_r ) { return pot_x_r(); }
	uint8_t pot_y_r() { uint8_t data = 0xff; if (exists()) data = m_device->vcs_pot_y_r(); return data; }
	DECLARE_READ8_MEMBER( pot_y_r ) { return pot_y_r(); }

	void joy_w( uint8_t data ) { if ( exists() ) m_device->vcs_joy_w( data ); }

	bool exists() { return m_device != nullptr; }
	bool has_pot_x() { return exists() && m_device->has_pot_x(); }
	bool has_pot_y() { return exists() && m_device->has_pot_y(); }

	void trigger_w(int state) { m_write_trigger(state); }

protected:
	// device-level overrides
	virtual void device_start() override;

	device_vcs_control_port_interface *m_device;

private:
	devcb_write_line m_write_trigger;
};


// device type definition
DECLARE_DEVICE_TYPE(VCS_CONTROL_PORT, vcs_control_port_device)

SLOT_INTERFACE_EXTERN( vcs_control_port_devices );

#endif // MAME_BUS_CVS_CTRL_CTRL_H
