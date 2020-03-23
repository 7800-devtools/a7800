// license:BSD-3-Clause
// copyright-holders:R. Belmont
/***************************************************************************

  a1bus.h - Apple I expansion slot and card emulation

***************************************************************************/

#ifndef MAME_BUS_A1BUS_A1BUS_H
#define MAME_BUS_A1BUS_A1BUS_H

#pragma once


//**************************************************************************
//  INTERFACE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_A1BUS_CPU(_cputag) \
	a1bus_device::static_set_cputag(*device, _cputag);

#define MCFG_A1BUS_OUT_IRQ_CB(_devcb) \
	devcb = &a1bus_device::set_out_irq_callback(*device, DEVCB_##_devcb);

#define MCFG_A1BUS_OUT_NMI_CB(_devcb) \
	devcb = &a1bus_device::set_out_nmi_callback(*device, DEVCB_##_devcb);

#define MCFG_A1BUS_SLOT_ADD(_nbtag, _tag, _slot_intf, _def_slot) \
	MCFG_DEVICE_ADD(_tag, A1BUS_SLOT, 0) \
	MCFG_DEVICE_SLOT_INTERFACE(_slot_intf, _def_slot, false) \
	a1bus_slot_device::static_set_a1bus_slot(*device, _nbtag, _tag);
#define MCFG_A1BUS_SLOT_REMOVE(_tag)    \
	MCFG_DEVICE_REMOVE(_tag)

#define MCFG_A1BUS_ONBOARD_ADD(_nbtag, _tag, _dev_type, _def_inp) \
	MCFG_DEVICE_ADD(_tag, _dev_type, 0) \
	MCFG_DEVICE_INPUT_DEFAULTS(_def_inp) \
	device_a1bus_card_interface::static_set_a1bus_tag(*device, _nbtag, _tag);

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

class a1bus_device;

class a1bus_slot_device : public device_t,
							public device_slot_interface
{
public:
	// construction/destruction
	a1bus_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// inline configuration
	static void static_set_a1bus_slot(device_t &device, const char *tag, const char *slottag);

protected:
	a1bus_slot_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override;

	// configuration
	const char *m_a1bus_tag, *m_a1bus_slottag;
};

// device type definition
DECLARE_DEVICE_TYPE(A1BUS_SLOT, a1bus_slot_device)


class device_a1bus_card_interface;
// ======================> a1bus_device
class a1bus_device : public device_t
{
public:
	// construction/destruction
	a1bus_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// inline configuration
	static void static_set_cputag(device_t &device, const char *tag);
	template <class Object> static devcb_base &set_out_irq_callback(device_t &device, Object &&cb) { return downcast<a1bus_device &>(device).m_out_irq_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_out_nmi_callback(device_t &device, Object &&cb) { return downcast<a1bus_device &>(device).m_out_nmi_cb.set_callback(std::forward<Object>(cb)); }

	void add_a1bus_card(device_a1bus_card_interface *card);
	device_a1bus_card_interface *get_a1bus_card();

	void set_irq_line(int state);
	void set_nmi_line(int state);

	void install_device(offs_t start, offs_t end, read8_delegate rhandler, write8_delegate whandler);
	void install_bank(offs_t start, offs_t end, const char *tag, uint8_t *data);

	DECLARE_WRITE_LINE_MEMBER( irq_w );
	DECLARE_WRITE_LINE_MEMBER( nmi_w );

protected:
	a1bus_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// internal state
	cpu_device   *m_maincpu;

	devcb_write_line    m_out_irq_cb;
	devcb_write_line    m_out_nmi_cb;

	device_a1bus_card_interface *m_device;
	const char *m_cputag;
};


// device type definition
DECLARE_DEVICE_TYPE(A1BUS, a1bus_device)

// ======================> device_a1bus_card_interface

// class representing interface-specific live a1bus card
class device_a1bus_card_interface : public device_slot_card_interface
{
	friend class a1bus_device;
public:
	// construction/destruction
	virtual ~device_a1bus_card_interface();

	device_a1bus_card_interface *next() const { return m_next; }

	void set_a1bus_device();

	void raise_slot_irq() { m_a1bus->set_irq_line(ASSERT_LINE); }
	void lower_slot_irq() { m_a1bus->set_irq_line(CLEAR_LINE); }
	void raise_slot_nmi() { m_a1bus->set_nmi_line(ASSERT_LINE); }
	void lower_slot_nmi() { m_a1bus->set_nmi_line(CLEAR_LINE); }

	void install_device(offs_t start, offs_t end, read8_delegate rhandler, write8_delegate whandler);
	void install_bank(offs_t start, offs_t end, char *tag, uint8_t *data);

	// inline configuration
	static void static_set_a1bus_tag(device_t &device, const char *tag, const char *slottag);

protected:
	device_a1bus_card_interface(const machine_config &mconfig, device_t &device);

	a1bus_device  *m_a1bus;
	const char *m_a1bus_tag, *m_a1bus_slottag;
	device_a1bus_card_interface *m_next;
};

#endif  // MAME_BUS_A1BUS_A1BUS_H
