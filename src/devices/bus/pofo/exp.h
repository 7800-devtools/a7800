// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    Atari Portfolio Expansion Port emulation

**********************************************************************

                   ABUF       1      2       5VS
                   REDY       3      4       VCC
                   BCOM       5      6       NCC1
                   NMD1       7      8       WAKE
                    DTR       9      10      DEN
                   PDET      11      12      IINT
                   CCLK      13      14      MRST
                   HLDA      15      16      HLDO
                   IACK      17      18      CDET
                    IOM      19      20      A19
                    A18      21      22      A17
                    A16      23      24      A15
                    A14      25      26      A13
                    A12      27      28      A11
                    A10      29      30      A9
                     A8      31      32      VRAM
                   HLDI      33      34      ALE
                    GND      35      36      NMIO
                    OA7      37      38      OA6
                    OA5      39      40      OA4
                    OA3      41      42      OA2
                    OA1      43      44      OA0
                    AD0      45      46      AD1
                    AD2      47      48      AD3
                    AD4      49      50      AD5
                    AD6      51      52      AD7
                   EINT      53      54      NRDI
                   VEXT      55      56      EACK
                   BATD      57      58      NWRI
                    5VS      59      60      BBUF

**********************************************************************/

#ifndef MAME_BUS_POFO_EXP_H
#define MAME_BUS_POFO_EXP_H

#pragma once




//**************************************************************************
//  CONSTANTS
//**************************************************************************

#define PORTFOLIO_EXPANSION_SLOT_TAG      "exp"



//**************************************************************************
//  INTERFACE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_PORTFOLIO_EXPANSION_SLOT_ADD(_tag, _clock, _slot_intf, _def_slot) \
	MCFG_DEVICE_ADD(_tag, PORTFOLIO_EXPANSION_SLOT, _clock) \
	MCFG_DEVICE_SLOT_INTERFACE(_slot_intf, _def_slot, false)

#define MCFG_PORTFOLIO_EXPANSION_SLOT_EINT_CALLBACK(_write) \
	devcb = &portfolio_expansion_slot_device::set_eint_wr_callback(*device, DEVCB_##_write);

#define MCFG_PORTFOLIO_EXPANSION_SLOT_NMIO_CALLBACK(_write) \
	devcb = &portfolio_expansion_slot_device::set_nmio_wr_callback(*device, DEVCB_##_write);

#define MCFG_PORTFOLIO_EXPANSION_SLOT_WAKE_CALLBACK(_write) \
	devcb = &portfolio_expansion_slot_device::set_wake_wr_callback(*device, DEVCB_##_write);



//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> device_portfolio_expansion_slot_interface

class portfolio_expansion_slot_device;

class device_portfolio_expansion_slot_interface : public device_slot_card_interface
{
public:
	virtual bool nmd1() { return 1; }
	virtual bool pdet() { return 0; }
	virtual bool cdet() { return 1; }

	virtual uint8_t iack_r() { return 0xff; }
	virtual uint8_t eack_r() { return 0xff; }

	virtual uint8_t nrdi_r(address_space &space, offs_t offset, uint8_t data, bool iom, bool bcom, bool ncc1) { return data; }
	virtual void nwri_w(address_space &space, offs_t offset, uint8_t data, bool iom, bool bcom, bool ncc1) { }

	virtual WRITE_LINE_MEMBER( iint_w ) { }

	DECLARE_WRITE_LINE_MEMBER( eint_w );
	DECLARE_WRITE_LINE_MEMBER( nmio_w );
	DECLARE_WRITE_LINE_MEMBER( wake_w );

protected:
	// construction/destruction
	device_portfolio_expansion_slot_interface(const machine_config &mconfig, device_t &device);

	portfolio_expansion_slot_device *m_slot;
};


// ======================> portfolio_expansion_slot_device

class portfolio_expansion_slot_device : public device_t, public device_slot_interface
{
public:
	// construction/destruction
	portfolio_expansion_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	template <class Object> static devcb_base &set_eint_wr_callback(device_t &device, Object &&cb) { return downcast<portfolio_expansion_slot_device &>(device).m_write_eint.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_nmio_wr_callback(device_t &device, Object &&cb) { return downcast<portfolio_expansion_slot_device &>(device).m_write_nmio.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_wake_wr_callback(device_t &device, Object &&cb) { return downcast<portfolio_expansion_slot_device &>(device).m_write_wake.set_callback(std::forward<Object>(cb)); }

	// computer interface
	bool nmd1_r() { return (m_card != nullptr) ? m_card->nmd1() : 1; }
	bool pdet_r() { return (m_card != nullptr) ? m_card->pdet() : 0; }
	bool cdet_r() { return (m_card != nullptr) ? m_card->cdet() : 1; }

	uint8_t iack_r() { return (m_card != nullptr) ? m_card->iack_r() : 0xff; };
	uint8_t eack_r() { return (m_card != nullptr) ? m_card->eack_r() : 0xff; };

	uint8_t nrdi_r(address_space &space, offs_t offset, uint8_t data, bool iom, bool bcom, bool ncc1) { return (m_card != nullptr) ? m_card->nrdi_r(space, offset, data, iom, bcom, ncc1) : data; }
	void nwri_w(address_space &space, offs_t offset, uint8_t data, bool iom, bool bcom, bool ncc1) { if (m_card != nullptr) m_card->nwri_w(space, offset, data, iom, bcom, ncc1); }

	WRITE_LINE_MEMBER( iint_w ) { if (m_card != nullptr) m_card->iint_w(state); }

	// peripheral interface
	WRITE_LINE_MEMBER( eint_w ) { m_write_eint(state); }
	WRITE_LINE_MEMBER( nmio_w ) { m_write_nmio(state); }
	WRITE_LINE_MEMBER( wake_w ) { m_write_wake(state); }

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	devcb_write_line   m_write_eint;
	devcb_write_line   m_write_nmio;
	devcb_write_line   m_write_wake;

	device_portfolio_expansion_slot_interface *m_card;
};


// device type definition
extern const device_type PORTFOLIO_EXPANSION_SLOT;
DECLARE_DEVICE_TYPE(PORTFOLIO_EXPANSION_SLOT, portfolio_expansion_slot_device)


SLOT_INTERFACE_EXTERN( portfolio_expansion_cards );

#endif // MAME_BUS_POFO_EXP_H
