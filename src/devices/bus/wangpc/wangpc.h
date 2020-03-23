// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    Wang Professional Computer bus emulation

**********************************************************************


**********************************************************************/

#ifndef MAME_BUS_WANGPC_WANGPC_H
#define MAME_BUS_WANGPC_WANGPC_H

#pragma once




//**************************************************************************
//  CONSTANTS
//**************************************************************************

#define WANGPC_BUS_TAG      "wangpcbus"



//**************************************************************************
//  INTERFACE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_WANGPC_BUS_ADD() \
	MCFG_DEVICE_ADD(WANGPC_BUS_TAG, WANGPC_BUS, 0)

#define MCFG_WANGPC_BUS_SLOT_ADD(_tag, _sid, _slot_intf, _def_slot) \
	MCFG_DEVICE_ADD(_tag, WANGPC_BUS_SLOT, 0) \
	MCFG_DEVICE_SLOT_INTERFACE(_slot_intf, _def_slot, false) \
	wangpcbus_slot_device::static_set_wangpcbus_slot(*device, _sid);


#define MCFG_WANGPC_BUS_IRQ2_CALLBACK(_write) \
	devcb = &wangpcbus_device::set_irq2_wr_callback(*device, DEVCB_##_write);

#define MCFG_WANGPC_BUS_IRQ3_CALLBACK(_write) \
	devcb = &wangpcbus_device::set_irq3_wr_callback(*device, DEVCB_##_write);

#define MCFG_WANGPC_BUS_IRQ4_CALLBACK(_write) \
	devcb = &wangpcbus_device::set_irq4_wr_callback(*device, DEVCB_##_write);

#define MCFG_WANGPC_BUS_IRQ5_CALLBACK(_write) \
	devcb = &wangpcbus_device::set_irq5_wr_callback(*device, DEVCB_##_write);

#define MCFG_WANGPC_BUS_IRQ6_CALLBACK(_write) \
	devcb = &wangpcbus_device::set_irq6_wr_callback(*device, DEVCB_##_write);

#define MCFG_WANGPC_BUS_IRQ7_CALLBACK(_write) \
	devcb = &wangpcbus_device::set_irq7_wr_callback(*device, DEVCB_##_write);

#define MCFG_WANGPC_BUS_DRQ1_CALLBACK(_write) \
	devcb = &wangpcbus_device::set_drq1_wr_callback(*device, DEVCB_##_write);

#define MCFG_WANGPC_BUS_DRQ2_CALLBACK(_write) \
	devcb = &wangpcbus_device::set_drq2_wr_callback(*device, DEVCB_##_write);

#define MCFG_WANGPC_BUS_DRQ3_CALLBACK(_write) \
	devcb = &wangpcbus_device::set_drq3_wr_callback(*device, DEVCB_##_write);

#define MCFG_WANGPC_BUS_IOERROR_CALLBACK(_write) \
	devcb = &wangpcbus_device::set_ioerror_wr_callback(*device, DEVCB_##_write);


//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> wangpcbus_slot_device

class wangpcbus_device;

class wangpcbus_slot_device : public device_t, public device_slot_interface
{
public:
	// construction/destruction
	wangpcbus_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// inline configuration
	static void static_set_wangpcbus_slot(device_t &device, int sid);

protected:
	// device-level overrides
	virtual void device_start() override;

private:
	// configuration
	wangpcbus_device  *m_bus;
	int m_sid;
};


// device type definition
DECLARE_DEVICE_TYPE(WANGPC_BUS_SLOT, wangpcbus_slot_device)


class device_wangpcbus_card_interface;


// ======================> wangpcbus_device

class wangpcbus_device : public device_t
{
public:
	// construction/destruction
	wangpcbus_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	~wangpcbus_device() { m_device_list.detach_all(); }

	template <class Object> static devcb_base &set_irq2_wr_callback(device_t &device, Object &&cb) { return downcast<wangpcbus_device &>(device).m_write_irq2.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_irq3_wr_callback(device_t &device, Object &&cb) { return downcast<wangpcbus_device &>(device).m_write_irq3.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_irq4_wr_callback(device_t &device, Object &&cb) { return downcast<wangpcbus_device &>(device).m_write_irq4.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_irq5_wr_callback(device_t &device, Object &&cb) { return downcast<wangpcbus_device &>(device).m_write_irq5.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_irq6_wr_callback(device_t &device, Object &&cb) { return downcast<wangpcbus_device &>(device).m_write_irq6.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_irq7_wr_callback(device_t &device, Object &&cb) { return downcast<wangpcbus_device &>(device).m_write_irq7.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_drq1_wr_callback(device_t &device, Object &&cb) { return downcast<wangpcbus_device &>(device).m_write_drq1.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_drq2_wr_callback(device_t &device, Object &&cb) { return downcast<wangpcbus_device &>(device).m_write_drq2.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_drq3_wr_callback(device_t &device, Object &&cb) { return downcast<wangpcbus_device &>(device).m_write_drq3.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_ioerror_wr_callback(device_t &device, Object &&cb) { return downcast<wangpcbus_device &>(device).m_write_ioerror.set_callback(std::forward<Object>(cb)); }

	void add_card(device_wangpcbus_card_interface *card, int sid);

	// computer interface
	DECLARE_READ16_MEMBER( mrdc_r );
	DECLARE_WRITE16_MEMBER( amwc_w );

	DECLARE_READ16_MEMBER( sad_r );
	DECLARE_WRITE16_MEMBER( sad_w );

	uint8_t dack_r(address_space &space, int line);
	void dack_w(address_space &space, int line, uint8_t data);

	DECLARE_READ8_MEMBER( dack0_r ) { return dack_r(space, 0); }
	DECLARE_WRITE8_MEMBER( dack0_w ) { dack_w(space, 0, data); }
	DECLARE_READ8_MEMBER( dack1_r ) { return dack_r(space, 1); }
	DECLARE_WRITE8_MEMBER( dack1_w ) { dack_w(space, 1, data); }
	DECLARE_READ8_MEMBER( dack2_r ) { return dack_r(space, 2); }
	DECLARE_WRITE8_MEMBER( dack2_w ) { dack_w(space, 2, data); }
	DECLARE_READ8_MEMBER( dack3_r ) { return dack_r(space, 3); }
	DECLARE_WRITE8_MEMBER( dack3_w ) { dack_w(space, 3, data); }

	DECLARE_WRITE_LINE_MEMBER( tc_w );

	// peripheral interface
	DECLARE_WRITE_LINE_MEMBER( irq2_w ) { m_write_irq2(state); }
	DECLARE_WRITE_LINE_MEMBER( irq3_w ) { m_write_irq3(state); }
	DECLARE_WRITE_LINE_MEMBER( irq4_w ) { m_write_irq4(state); }
	DECLARE_WRITE_LINE_MEMBER( irq5_w ) { m_write_irq5(state); }
	DECLARE_WRITE_LINE_MEMBER( irq6_w ) { m_write_irq6(state); }
	DECLARE_WRITE_LINE_MEMBER( irq7_w ) { m_write_irq7(state); }
	DECLARE_WRITE_LINE_MEMBER( drq1_w ) { m_write_drq1(state); }
	DECLARE_WRITE_LINE_MEMBER( drq2_w ) { m_write_drq2(state); }
	DECLARE_WRITE_LINE_MEMBER( drq3_w ) { m_write_drq3(state); }
	DECLARE_WRITE_LINE_MEMBER( ioerror_w ) { m_write_ioerror(state); }

protected:
	// device-level overrides
	virtual void device_start() override;

private:
	devcb_write_line   m_write_irq2;
	devcb_write_line   m_write_irq3;
	devcb_write_line   m_write_irq4;
	devcb_write_line   m_write_irq5;
	devcb_write_line   m_write_irq6;
	devcb_write_line   m_write_irq7;
	devcb_write_line   m_write_drq1;
	devcb_write_line   m_write_drq2;
	devcb_write_line   m_write_drq3;
	devcb_write_line   m_write_ioerror;

	simple_list<device_wangpcbus_card_interface> m_device_list;
};


// device type definition
DECLARE_DEVICE_TYPE(WANGPC_BUS, wangpcbus_device)


// ======================> device_wangpcbus_card_interface

// class representing interface-specific live wangpcbus card
class device_wangpcbus_card_interface : public device_slot_card_interface
{
	friend class wangpcbus_device;
	template <class ElementType> friend class simple_list;

public:
	device_wangpcbus_card_interface *next() const { return m_next; }

	// memory access
	virtual uint16_t wangpcbus_mrdc_r(address_space &space, offs_t offset, uint16_t mem_mask) { return 0; }
	virtual void wangpcbus_amwc_w(address_space &space, offs_t offset, uint16_t mem_mask, uint16_t data) { }

	// I/O access
	virtual uint16_t wangpcbus_iorc_r(address_space &space, offs_t offset, uint16_t mem_mask) { return 0; }
	virtual void wangpcbus_aiowc_w(address_space &space, offs_t offset, uint16_t mem_mask, uint16_t data) { }
	bool sad(offs_t offset) const { return (offset & 0xf80) == (0x800 | (m_sid << 7)); }

	// DMA
	virtual uint8_t wangpcbus_dack_r(address_space &space, int line) { return 0; }
	virtual void wangpcbus_dack_w(address_space &space, int line, uint8_t data) { }
	virtual void wangpcbus_tc_w(int state) { }
	virtual bool wangpcbus_have_dack(int line) { return false; }

protected:
	// construction/destruction
	device_wangpcbus_card_interface(const machine_config &mconfig, device_t &device);

	wangpcbus_device *m_bus;
	wangpcbus_slot_device *m_slot;

	int m_sid;

private:
	device_wangpcbus_card_interface *m_next;
};


SLOT_INTERFACE_EXTERN( wangpc_cards );

#endif // MAME_BUS_WANGPC_WANGPC_H
