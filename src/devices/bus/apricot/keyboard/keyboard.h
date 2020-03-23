// license:GPL-2.0+
// copyright-holders:Dirk Best
/***************************************************************************

    ACT Apricot Keyboard Interface

    Host interface: 9-pin D-SUB

    1  +12V
    2  OUT
    3  IN
    4  N/C
    5  N/C
    6  GND
    7  -12V
    8  0V
    9  N/C

    Keyboard interface:

    A  0V
    B  +12V
    C  -12V
    D  N/C
    E  OUT
    F  IN

***************************************************************************/

#ifndef MAME_BUS_APRICOT_KEYBOARD_KEYBOARD_H
#define MAME_BUS_APRICOT_KEYBOARD_KEYBOARD_H

#pragma once



//**************************************************************************
//  INTERFACE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_APRICOT_KEYBOARD_INTERFACE_ADD(_tag, _def_slot) \
	MCFG_DEVICE_ADD(_tag, APRICOT_KEYBOARD_INTERFACE, 0) \
	MCFG_DEVICE_SLOT_INTERFACE(apricot_keyboard_devices, _def_slot, false)

#define MCFG_APRICOT_KEYBOARD_IN_HANDLER(_devcb) \
	devcb = &apricot_keyboard_bus_device::set_in_handler(*device, DEVCB_##_devcb);


//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

class device_apricot_keyboard_interface;

// ======================> apricot_keyboard_bus_device

class apricot_keyboard_bus_device : public device_t, public device_slot_interface
{
public:
	// construction/destruction
	apricot_keyboard_bus_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	virtual ~apricot_keyboard_bus_device();

	// callbacks
	template <class Object> static devcb_base &set_in_handler(device_t &device, Object &&cb)
	{ return downcast<apricot_keyboard_bus_device &>(device).m_in_handler.set_callback(std::forward<Object>(cb)); }

	// called from keyboard
	DECLARE_WRITE_LINE_MEMBER( in_w ) { m_in_handler(state); }

	// called from host
	DECLARE_WRITE_LINE_MEMBER( out_w );

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

private:
	device_apricot_keyboard_interface *m_kbd;

	devcb_write_line m_in_handler;
};

// ======================> device_apricot_keyboard_interface

class device_apricot_keyboard_interface : public device_slot_card_interface
{
public:
	// construction/destruction
	virtual ~device_apricot_keyboard_interface();

	virtual void out_w(int state) = 0;

protected:
	device_apricot_keyboard_interface(const machine_config &mconfig, device_t &device);

	apricot_keyboard_bus_device *m_host;
};


// device type definition
DECLARE_DEVICE_TYPE(APRICOT_KEYBOARD_INTERFACE, apricot_keyboard_bus_device)

// supported devices
SLOT_INTERFACE_EXTERN( apricot_keyboard_devices );


#endif // MAME_BUS_APRICOT_KEYBOARD_KEYBOARD_H
