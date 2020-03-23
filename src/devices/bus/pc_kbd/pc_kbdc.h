// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/***************************************************************************

  PC Keyboard connector interface

The data line is usually sampled on changes of the clock line. If you have
a device that changes both the data and clock lines at the same time, first
set the data line and then set the clock line.

***************************************************************************/

#ifndef MAME_BUS_PC_KBD_PC_KBDC_H
#define MAME_BUS_PC_KBD_PC_KBDC_H

#pragma once


//**************************************************************************
//  INTERFACE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_PC_KBDC_OUT_CLOCK_CB(_devcb) \
	devcb = &pc_kbdc_device::set_out_clock_callback(*device, DEVCB_##_devcb);

#define MCFG_PC_KBDC_OUT_DATA_CB(_devcb) \
	devcb = &pc_kbdc_device::set_out_data_callback(*device, DEVCB_##_devcb);

#define MCFG_PC_KBDC_SLOT_ADD(_kbdc_tag, _tag, _slot_intf, _def_slot) \
	MCFG_DEVICE_ADD(_tag, PC_KBDC_SLOT, 0 ) \
	MCFG_DEVICE_SLOT_INTERFACE(_slot_intf, _def_slot, false) \
	pc_kbdc_slot_device::static_set_pc_kbdc_slot(*device, owner->subdevice(_kbdc_tag) );

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************


class pc_kbdc_slot_device : public device_t,
							public device_slot_interface
{
public:
	// construction/destruction
	pc_kbdc_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override;

	// inline configuration
	static void static_set_pc_kbdc_slot(device_t &device, device_t *kbdc_device);

protected:
	// configuration
	device_t *m_kbdc_device;
};


// device type definition
DECLARE_DEVICE_TYPE(PC_KBDC_SLOT, pc_kbdc_slot_device)


class device_pc_kbd_interface;

class pc_kbdc_device : public device_t
{
public:
	// construction/destruction
	pc_kbdc_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	template <class Object> static devcb_base &set_out_clock_callback(device_t &device, Object &&cb) { return downcast<pc_kbdc_device &>(device).m_out_clock_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_out_data_callback(device_t &device, Object &&cb) { return downcast<pc_kbdc_device &>(device).m_out_data_cb.set_callback(std::forward<Object>(cb)); }

	void set_keyboard(device_pc_kbd_interface *keyboard);

	int clock_signal() { return m_clock_state; }
	int data_signal() { return m_data_state; }

	DECLARE_WRITE_LINE_MEMBER( clock_write_from_mb );
	DECLARE_WRITE_LINE_MEMBER( data_write_from_mb );
	DECLARE_WRITE_LINE_MEMBER( clock_write_from_kb );
	DECLARE_WRITE_LINE_MEMBER( data_write_from_kb );

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	void update_clock_state();
	void update_data_state();

	devcb_write_line    m_out_clock_cb;
	devcb_write_line    m_out_data_cb;

	int                         m_clock_state;
	int                         m_data_state;

	int                         m_mb_clock_state;
	int                         m_mb_data_state;
	int                         m_kb_clock_state;
	int                         m_kb_data_state;

	device_pc_kbd_interface     *m_keyboard;
};


// device type definition
DECLARE_DEVICE_TYPE(PC_KBDC, pc_kbdc_device)


// ======================> device_pc_pbd_interface

class device_pc_kbd_interface : public device_slot_card_interface
{
	friend class pc_kbdc_device;
public:
	// construction/destruction
	device_pc_kbd_interface(const machine_config &mconfig, device_t &device);
	virtual ~device_pc_kbd_interface();

	void set_pc_kbdc_device();

	int clock_signal() { return m_pc_kbdc ? m_pc_kbdc->clock_signal() : 1; }
	int data_signal() { return m_pc_kbdc ? m_pc_kbdc->data_signal() : 1; }

	//
	// Override the clock_write and data_write methods in a keyboard implementation
	//
	virtual DECLARE_WRITE_LINE_MEMBER( clock_write );
	virtual DECLARE_WRITE_LINE_MEMBER( data_write );

	// inline configuration
	static void static_set_pc_kbdc(device_t &device, device_t *kbdc_device);

protected:
	pc_kbdc_device          *m_pc_kbdc;
	const char              *m_pc_kbdc_tag;
};



#endif // MAME_BUS_PC_KBD_PC_KBDC_H
