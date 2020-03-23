// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    Atari Portfolio Memory Card port emulation

**********************************************************************

    Pin     COMMON  RAM     OPTROM              Mask ROM
                            32k 64k 128k
    1       A16
    2       A15
    3               VBB     VPP NC  VPP         NC
    4       A12
    5       A7
    6       A6
    7       A5
    8       A4
    9       A3
    10      A2
    11      A1
    12      A0
    13      D0
    14      D1
    15      D2
    16      GND
    17      D3
    18      D4
    19      D5
    20      D6
    21      D7
    22      CE
    23      A10
    24              OE      OE  OE/VPP          OE      OE
    25      A11
    26      A9
    27      A8
    28      A13
    29      A14
    30              WE      NC  NC              PGM     NC
    31      VCC
    32      CDET

**********************************************************************/

#ifndef MAME_BUS_POFO_CCM_H
#define MAME_BUS_POFO_CCM_H

#pragma once

#include "softlist_dev.h"



//**************************************************************************
//  CONSTANTS
//**************************************************************************

#define PORTFOLIO_MEMORY_CARD_SLOT_A_TAG     "ccma"
#define PORTFOLIO_MEMORY_CARD_SLOT_B_TAG     "ccmb"



//**************************************************************************
//  INTERFACE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_PORTFOLIO_MEMORY_CARD_SLOT_ADD(_tag, _slot_intf, _def_slot) \
	MCFG_DEVICE_ADD(_tag, PORTFOLIO_MEMORY_CARD_SLOT, 0) \
	MCFG_DEVICE_SLOT_INTERFACE(_slot_intf, _def_slot, false)



//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> device_portfolio_memory_card_slot_interface

class portfolio_memory_card_slot_device;

class device_portfolio_memory_card_slot_interface : public device_slot_card_interface
{
	friend class portfolio_memory_card_slot_device;

public:
	virtual bool cdet() { return 1; }

	virtual uint8_t nrdi_r(address_space &space, offs_t offset) { return 0xff; }
	virtual void nwri_w(address_space &space, offs_t offset, uint8_t data) { }

protected:
	// construction/destruction
	device_portfolio_memory_card_slot_interface(const machine_config &mconfig, device_t &device);

	optional_shared_ptr<uint8_t> m_rom;
	optional_shared_ptr<uint8_t> m_nvram;

	portfolio_memory_card_slot_device *m_slot;
};


// ======================> portfolio_memory_card_slot_device

class portfolio_memory_card_slot_device : public device_t,
									 public device_slot_interface,
									 public device_image_interface
{
public:
	// construction/destruction
	portfolio_memory_card_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// computer interface
	bool cdet_r() { return (m_card != nullptr) ? m_card->cdet() : 1; }

	DECLARE_READ8_MEMBER( nrdi_r ) { return (m_card != nullptr) ? m_card->nrdi_r(space, offset) : 0xff; }
	DECLARE_WRITE8_MEMBER( nwri_w ) { if (m_card != nullptr) m_card->nwri_w(space, offset, data); }

protected:
	// device-level overrides
	virtual void device_start() override;

	// image-level overrides
	virtual image_init_result call_load() override;
	virtual const software_list_loader &get_software_list_loader() const override { return rom_software_list_loader::instance(); }

	virtual iodevice_t image_type() const override { return IO_CARTSLOT; }

	virtual bool is_readable()  const override { return 1; }
	virtual bool is_writeable() const override { return 1; }
	virtual bool is_creatable() const override { return 1; }
	virtual bool must_be_loaded() const override { return 0; }
	virtual bool is_reset_on_load() const override { return 1; }
	virtual const char *image_interface() const override { return "pofo_card"; }
	virtual const char *file_extensions() const override { return "rom,bin"; }

	// slot interface overrides
	virtual std::string get_default_card_software(get_default_card_software_hook &hook) const override;

	device_portfolio_memory_card_slot_interface *m_card;
};


// device type definition
extern const device_type PORTFOLIO_MEMORY_CARD_SLOT;
DECLARE_DEVICE_TYPE(PORTFOLIO_MEMORY_CARD_SLOT, portfolio_memory_card_slot_device)


SLOT_INTERFACE_EXTERN( portfolio_memory_cards );



#endif // MAME_BUS_POFO_CCM_H
