// license:BSD-3-Clause
// copyright-holders:Miodrag Milanovic, R.Belmont
/***************************************************************************

        HP DIO and DIO-II bus devices

        DIO is 16-bit, essentially the MC68000 bus
        DIO-II extends to 32-bit for 68020/030/040 machines

        16-bit DIO cards fit and work in either 16 or 32 bit systems, much like 8-bit ISA.
        32-bit DIO-II cards only work in 32 bit DIO-II systems.

***************************************************************************/

#ifndef MAME_BUS_HPDIO_HPDIO_H
#define MAME_BUS_HPDIO_HPDIO_H

#pragma once

//**************************************************************************
//  INTERFACE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_DIO16_CPU(_cputag) \
	dio16_device::static_set_cputag(*device, _cputag);
#define MCFG_DIO16_SLOT_ADD(_diotag, _tag, _slot_intf, _def_slot, _fixed) \
	MCFG_DEVICE_ADD(_tag, DIO16_SLOT, 0) \
	MCFG_DEVICE_SLOT_INTERFACE(_slot_intf, _def_slot, _fixed) \
	dio16_slot_device::static_set_dio16_slot(*device, owner, _diotag);
#define MCFG_DIO32_CPU(_cputag) \
	dio32_device::static_set_cputag(*device, _cputag);
#define MCFG_DIO32_SLOT_ADD(_diotag, _tag, _slot_intf, _def_slot, _fixed) \
	MCFG_DEVICE_ADD(_tag, DIO32_SLOT, 0) \
	MCFG_DEVICE_SLOT_INTERFACE(_slot_intf, _def_slot, _fixed) \
	dio32_slot_device::static_set_dio32_slot(*device, owner, _diotag);

#define MCFG_ISA_OUT_IRQ3_CB(_devcb) \
	devcb = &dio16_device::set_out_irq3_callback(*device, DEVCB_##_devcb);

#define MCFG_ISA_OUT_IRQ4_CB(_devcb) \
	devcb = &dio16_device::set_out_irq4_callback(*device, DEVCB_##_devcb);

#define MCFG_ISA_OUT_IRQ5_CB(_devcb) \
	devcb = &dio16_device::set_out_irq5_callback(*device, DEVCB_##_devcb);

#define MCFG_ISA_OUT_IRQ6_CB(_devcb) \
	devcb = &dio16_device::set_out_irq6_callback(*device, DEVCB_##_devcb);

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

class dio16_device;

class dio16_slot_device : public device_t,
							public device_slot_interface
{
public:
	// construction/destruction
	dio16_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override;

	// inline configuration
	static void static_set_dio16_slot(device_t &device, device_t *owner, const char *dio_tag);

protected:
	dio16_slot_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// configuration
	device_t *m_owner;
	const char *m_dio_tag;
};

// device type definition
DECLARE_DEVICE_TYPE(DIO16_SLOT, dio16_slot_device)

class device_dio16_card_interface;
// ======================> dio16_device
class dio16_device : public device_t
{
public:
	// construction/destruction
	dio16_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	// inline configuration
	static void static_set_cputag(device_t &device, const char *tag);
	template <class Object> static devcb_base &set_out_irq3_callback(device_t &device, Object &&cb) { return downcast<dio16_device &>(device).m_out_irq3_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_out_irq4_callback(device_t &device, Object &&cb) { return downcast<dio16_device &>(device).m_out_irq4_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_out_irq5_callback(device_t &device, Object &&cb) { return downcast<dio16_device &>(device).m_out_irq5_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_out_irq6_callback(device_t &device, Object &&cb) { return downcast<dio16_device &>(device).m_out_irq6_cb.set_callback(std::forward<Object>(cb)); }

	void install_memory(offs_t start, offs_t end, read16_delegate rhandler, write16_delegate whandler);

	// DANGER: these will currently produce different results for a DIO-I card on DIO-I and DIO-II systems
	//         due to the varying bus widths.  Using all install_memory() shields you from this problem.
	//         either know what you're doing (m_prgwidth is available to cards for this purpose) or
	//         only use these for 32-bit DIO-II cards.
	void install_bank(offs_t start, offs_t end, const char *tag, uint8_t *data);
	void install_rom(offs_t start, offs_t end, const char *tag, uint8_t *data);

	void unmap_bank(offs_t start, offs_t end);
	void unmap_rom(offs_t start, offs_t end);

	// IRQs 1, 2, and 7 are reserved for non-bus usage.
	DECLARE_WRITE_LINE_MEMBER( irq3_w );
	DECLARE_WRITE_LINE_MEMBER( irq4_w );
	DECLARE_WRITE_LINE_MEMBER( irq5_w );
	DECLARE_WRITE_LINE_MEMBER( irq6_w );

	int m_prgwidth;

protected:
	dio16_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	void install_space(int spacenum, offs_t start, offs_t end, read8_delegate rhandler, write8_delegate whandler);

	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// internal state
	cpu_device   *m_maincpu;

	// address spaces
	address_space *m_prgspace;

	devcb_write_line    m_out_irq3_cb;
	devcb_write_line    m_out_irq4_cb;
	devcb_write_line    m_out_irq5_cb;
	devcb_write_line    m_out_irq6_cb;

	const char                 *m_cputag;

private:
};


// device type definition
DECLARE_DEVICE_TYPE(DIO16, dio16_device)

// ======================> device_dio16_card_interface

// class representing interface-specific live dio16 card
class device_dio16_card_interface : public device_slot_card_interface
{
	friend class dio16_device;
	template <class ElementType> friend class simple_list;
public:
	// construction/destruction
	virtual ~device_dio16_card_interface();

	device_dio16_card_interface *next() const { return m_next; }

	void set_dio_device();

	// inline configuration
	static void static_set_diobus(device_t &device, device_t *dio_device);

public:
	device_dio16_card_interface(const machine_config &mconfig, device_t &device);

	dio16_device  *m_dio;
	device_t     *m_dio_dev;

private:
	device_dio16_card_interface *m_next;
};

class dio32_device;

class dio32_slot_device : public dio16_slot_device
{
public:
	// construction/destruction
	dio32_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
		// device-level overrides
	virtual void device_start() override;

	// inline configuration
	static void static_set_dio32_slot(device_t &device, device_t *owner, const char *dio_tag);
};


// device type definition
DECLARE_DEVICE_TYPE(DIO32_SLOT, dio32_slot_device)

// ======================> dio32_device
class dio32_device : public dio16_device
{
public:
	// construction/destruction
	dio32_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	void install16_device(offs_t start, offs_t end, read16_delegate rhandler, write16_delegate whandler);


protected:
	// device-level overrides
	virtual void device_start() override;

private:
};


// device type definition
DECLARE_DEVICE_TYPE(DIO32, dio32_device)

// ======================> device_dio32_card_interface

// class representing interface-specific live dio32 card
class device_dio32_card_interface : public device_dio16_card_interface
{
	friend class dio32_device;
public:
	// construction/destruction
	virtual ~device_dio32_card_interface();

	void set_dio_device();

protected:
	device_dio32_card_interface(const machine_config &mconfig, device_t &device);

	dio32_device  *m_dio;
};

#endif // MAME_BUS_HPDIO_HPDIO_H
