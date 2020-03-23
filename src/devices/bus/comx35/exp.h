// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    COMX-35 Expansion Slot emulation

**********************************************************************

                    GND       1      A       GND
                     NC       2      B       DS
                    +5V       3      C       V+
                     D0       4      D       D1
                     D2       5      E       D3
                     D4       6      F       D5
                     D6       7      H       D7
                    _DP       8      J       Q
                 _CLEAR       9      K       _MRD
                    TPA      10      L       N0
                     N1      11      M       N2
                   _RAS      12      N       _INT
                  _WAIT      13      P       CLOCK
                    SC1      14      R       SC0
                   _EF4      15      S       _CASE
                    TPB      16      T       _A15
                   _MWR      17      U       A14
                    MA7      18      V       _A14
                    MA5      19      W       MA6
                    MA4      20      X       MA3
                    MA2      21      Y       _EXTROM
                    MA1      22      Z       MA0

**********************************************************************/

#ifndef MAME_BUS_COMX35_EXP_H
#define MAME_BUS_COMX35_EXP_H

#pragma once




//**************************************************************************
//  CONSTANTS
//**************************************************************************

#define COMX_EXPANSION_BUS_TAG      "comxexp"



//**************************************************************************
//  INTERFACE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_COMX_EXPANSION_SLOT_ADD(_tag, _slot_intf, _def_slot) \
	MCFG_DEVICE_ADD(_tag, COMX_EXPANSION_SLOT, 0) \
	MCFG_DEVICE_SLOT_INTERFACE(_slot_intf, _def_slot, false)


#define MCFG_COMX_EXPANSION_SLOT_IRQ_CALLBACK(_write) \
	devcb = &comx_expansion_slot_device::set_irq_wr_callback(*device, DEVCB_##_write);



//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> comx_expansion_slot_device

class device_comx_expansion_card_interface;

class comx_expansion_slot_device : public device_t,
									public device_slot_interface
{
public:
	// construction/destruction
	comx_expansion_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	template <class Object> static devcb_base &set_irq_wr_callback(device_t &device, Object &&cb) { return downcast<comx_expansion_slot_device &>(device).m_write_irq.set_callback(std::forward<Object>(cb)); }

	uint8_t mrd_r(address_space &space, offs_t offset, int *extrom);
	void mwr_w(address_space &space, offs_t offset, uint8_t data);

	uint8_t io_r(address_space &space, offs_t offset);
	void io_w(address_space &space, offs_t offset, uint8_t data);

	DECLARE_READ_LINE_MEMBER( ef4_r );

	DECLARE_WRITE_LINE_MEMBER( ds_w );
	DECLARE_WRITE_LINE_MEMBER( q_w );

	DECLARE_WRITE_LINE_MEMBER( irq_w ) { m_write_irq(state); }

protected:
	// device-level overrides
	virtual void device_start() override;

	devcb_write_line   m_write_irq;

	device_comx_expansion_card_interface *m_card;
};


// ======================> device_comx_expansion_card_interface

// class representing interface-specific live comx_expansion card
class device_comx_expansion_card_interface : public device_slot_card_interface
{
	friend class comx_expansion_slot_device;

protected:
	// construction/destruction
	device_comx_expansion_card_interface(const machine_config &mconfig, device_t &device);

	// signals
	virtual int comx_ef4_r() { return CLEAR_LINE; }
	virtual void comx_ds_w(int state) { m_ds = state; }
	virtual void comx_q_w(int state) { }

	// memory access
	virtual uint8_t comx_mrd_r(address_space &space, offs_t offset, int *extrom) { return 0; }
	virtual void comx_mwr_w(address_space &space, offs_t offset, uint8_t data) { }

	// I/O access
	virtual uint8_t comx_io_r(address_space &space, offs_t offset) { return 0; }
	virtual void comx_io_w(address_space &space, offs_t offset, uint8_t data) { }

	comx_expansion_slot_device *m_slot;

	int m_ds;
};


// device type definition
DECLARE_DEVICE_TYPE(COMX_EXPANSION_SLOT, comx_expansion_slot_device)


SLOT_INTERFACE_EXTERN( comx_expansion_cards );


#endif // MAME_BUS_COMX35_EXP_H
