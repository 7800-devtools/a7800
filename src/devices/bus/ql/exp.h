// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    Sinclair QL expansion port emulation

**********************************************************************

                              A     B
                        GND   *  1  *   GND
                         D3   *  2  *   D2
                         D4   *  3  *   D1
                         D5   *  4  *   D0
                         D6   *  5  *   ASL
                         D7   *  6  *   DSL
                        A19   *  7  *   RDWL
                        A18   *  8  *   DTACKL
                        A17   *  9  *   BGL
                        A16   * 10  *   BRL
                     CLKCPU   * 11  *   A15
                        RED   * 12  *   RESETCPUL
                        A14   * 13  *   CSYNCL
                        A13   * 14  *   E
                        A12   * 15  *   VSYNCH
                        A11   * 16  *   VPAL
                        A10   * 17  *   GREEN
                         A9   * 18  *   BLUE
                         A8   * 19  *   FC2
                         A7   * 20  *   FC1
                         A6   * 21  *   FC0
                         A5   * 22  *   A0
                         A4   * 23  *   ROMOEH
                         A3   * 24  *   A1
                       DBGL   * 25  *   A2
                        SP2   * 26  *   SP3
                      DSCML   * 27  *   IPL0L
                        SP1   * 28  *   BERRL
                        SP0   * 29  *   IPL1L
                       VP12   * 30  *   EXTINTL
                       VM12   * 31  *   VIN
                        VIN   * 32  *   VIN

**********************************************************************/

#ifndef MAME_BUS_QL_EXP_H
#define MAME_BUS_QL_EXP_H

#pragma once



//**************************************************************************
//  INTERFACE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_QL_EXPANSION_SLOT_ADD(_tag, _slot_intf, _def_slot) \
	MCFG_DEVICE_ADD(_tag, QL_EXPANSION_SLOT, 0) \
	MCFG_DEVICE_SLOT_INTERFACE(_slot_intf, _def_slot, false)


#define MCFG_QL_EXPANSION_SLOT_IPL0L_CALLBACK(_write) \
	devcb = &ql_expansion_slot_device::set_ipl0l_wr_callback(*device, DEVCB_##_write);

#define MCFG_QL_EXPANSION_SLOT_IPL1L_CALLBACK(_write) \
	devcb = &ql_expansion_slot_device::set_ipl1l_wr_callback(*device, DEVCB_##_write);

#define MCFG_QL_EXPANSION_SLOT_BERRL_CALLBACK(_write) \
	devcb = &ql_expansion_slot_device::set_berrl_wr_callback(*device, DEVCB_##_write);

#define MCFG_QL_EXPANSION_SLOT_EXTINTL_CALLBACK(_write) \
	devcb = &ql_expansion_slot_device::set_extintl_wr_callback(*device, DEVCB_##_write);



//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> device_ql_expansion_card_interface

class ql_expansion_slot_device;

class device_ql_expansion_card_interface : public device_slot_card_interface
{
	friend class ql_expansion_slot_device;

public:
	// construction/destruction
	device_ql_expansion_card_interface(const machine_config &mconfig, device_t &device);

	virtual void romoeh_w(int state) { m_romoeh = state; }
	virtual uint8_t read(address_space &space, offs_t offset, uint8_t data) { return data; }
	virtual void write(address_space &space, offs_t offset, uint8_t data) { }

protected:
	ql_expansion_slot_device  *m_slot;

	int m_romoeh;
};


// ======================> ql_expansion_slot_device

class ql_expansion_slot_device : public device_t, public device_slot_interface
{
public:
	// construction/destruction
	ql_expansion_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	template <class Object> static devcb_base &set_ipl0l_wr_callback(device_t &device, Object &&cb) { return downcast<ql_expansion_slot_device &>(device).m_write_ipl0l.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_ipl1l_wr_callback(device_t &device, Object &&cb) { return downcast<ql_expansion_slot_device &>(device).m_write_ipl1l.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_berrl_wr_callback(device_t &device, Object &&cb) { return downcast<ql_expansion_slot_device &>(device).m_write_berrl.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_extintl_wr_callback(device_t &device, Object &&cb) { return downcast<ql_expansion_slot_device &>(device).m_write_extintl.set_callback(std::forward<Object>(cb)); }

	// computer interface
	uint8_t read(address_space &space, offs_t offset, uint8_t data) { if (m_card) data = m_card->read(space, offset, data); return data; }
	void write(address_space &space, offs_t offset, uint8_t data) { if (m_card) m_card->write(space, offset, data); }
	DECLARE_WRITE_LINE_MEMBER( romoeh_w ) { if (m_card) m_card->romoeh_w(state); }

	// card interface
	DECLARE_WRITE_LINE_MEMBER( ipl0l_w ) { m_write_ipl0l(state); }
	DECLARE_WRITE_LINE_MEMBER( ipl1l_w ) { m_write_ipl1l(state); }
	DECLARE_WRITE_LINE_MEMBER( berrl_w ) { m_write_berrl(state); }
	DECLARE_WRITE_LINE_MEMBER( extintl_w ) { m_write_extintl(state); }

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override { if (get_card_device()) get_card_device()->reset(); }

	devcb_write_line   m_write_ipl0l;
	devcb_write_line   m_write_ipl1l;
	devcb_write_line   m_write_berrl;
	devcb_write_line   m_write_extintl;

	device_ql_expansion_card_interface *m_card;
};


// device type definition
DECLARE_DEVICE_TYPE(QL_EXPANSION_SLOT, ql_expansion_slot_device)


SLOT_INTERFACE_EXTERN( ql_expansion_cards );


#endif // MAME_BUS_QL_EXP_H
