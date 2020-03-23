// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    VideoBrain Expansion Port emulation

**********************************************************************

                    GND       1      2       BO2
                    BO1       3      4       BO0
                    BA0       5      6       BA1
                    BA2       7      8       BA3
                    BA4       9      10      BA5
                    BA6      11      12      BA7
                    +5V      13      14      BA8
                    BA9      15      16      _CS1
                   BA10      17      18      BO7
                    BO6      19      20      BO5
                    BO4      21      22      BO3
                   BA11      23      24      BA12
                    BRC      25      26      ?
                 EXTRES      27      28      _CS2
                      ?      29      30      UV202/35
                 HBLANK      31      32      UV201/5
                UV201/2      33      34      UV201/4
                UV201/3      35      36      BISTROBE
                UV202/1      37      38      BA13
               UV202/18      39      40      ?
                    R/W      41      42      +5V
                    GND      43      44      GND
                    GND      45      46      N/C
                      ?      47      48      N/C
                    N/C      49      50      N/C

**********************************************************************/

#ifndef MAME_BUS_VIDBRAIN_EXP_H
#define MAME_BUS_VIDBRAIN_EXP_H

#pragma once

#include "softlist_dev.h"


//**************************************************************************
//  CONSTANTS
//**************************************************************************

#define VIDEOBRAIN_EXPANSION_SLOT_TAG       "exp"



//**************************************************************************
//  INTERFACE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_VIDEOBRAIN_EXPANSION_SLOT_ADD(_tag, _slot_intf, _def_slot) \
	MCFG_DEVICE_ADD(_tag, VIDEOBRAIN_EXPANSION_SLOT, 0) \
	MCFG_DEVICE_SLOT_INTERFACE(_slot_intf, _def_slot, false)


#define MCFG_VIDEOBRAIN_EXPANSION_SLOT_EXTRES_CALLBACK(_write) \
	devcb = &videobrain_expansion_slot_device::set_extres_wr_callback(*device, DEVCB_##_write);



//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

class videobrain_expansion_slot_device;

// ======================> device_videobrain_expansion_card_interface

class device_videobrain_expansion_card_interface : public device_slot_card_interface
{
	friend class videobrain_expansion_slot_device;

public:
	// construction/destruction
	virtual ~device_videobrain_expansion_card_interface() { }

protected:
	device_videobrain_expansion_card_interface(const machine_config &mconfig, device_t &device);

	// initialization
	virtual uint8_t* videobrain_rom_pointer(running_machine &machine, size_t size);
	virtual uint8_t* videobrain_ram_pointer(running_machine &machine, size_t size);

	// runtime
	virtual uint8_t videobrain_bo_r(address_space &space, offs_t offset, int cs1, int cs2) { return 0; }
	virtual void videobrain_bo_w(address_space &space, offs_t offset, uint8_t data, int cs1, int cs2) { }
	virtual void videobrain_extres_w() { }

	videobrain_expansion_slot_device *m_slot;

	std::vector<uint8_t> m_rom;
	std::vector<uint8_t> m_ram;

	size_t m_rom_mask;
	size_t m_ram_mask;
};


// ======================> videobrain_expansion_slot_device

class videobrain_expansion_slot_device : public device_t,
											public device_slot_interface,
											public device_image_interface
{
public:
	// construction/destruction
	videobrain_expansion_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	template <class Object> static devcb_base &set_extres_wr_callback(device_t &device, Object &&cb) { return downcast<videobrain_expansion_slot_device &>(device).m_write_extres.set_callback(std::forward<Object>(cb)); }

	// computer interface
	uint8_t bo_r(address_space &space, offs_t offset, int cs1, int cs2);
	void bo_w(address_space &space, offs_t offset, uint8_t data, int cs1, int cs2);

	DECLARE_READ8_MEMBER( cs1_r ) { return bo_r(space, offset + 0x1000, 0, 1); }
	DECLARE_WRITE8_MEMBER( cs1_w ) { bo_w(space, offset + 0x1000, data, 0, 1); }
	DECLARE_READ8_MEMBER( cs2_r ) { return bo_r(space, offset + 0x1800, 1, 0); }
	DECLARE_WRITE8_MEMBER( cs2_w ) { bo_w(space, offset + 0x1800, data, 1, 0); }
	DECLARE_READ8_MEMBER( unmap_r ) { return bo_r(space, offset + 0x3000, 1, 0); }
	DECLARE_WRITE8_MEMBER( unmap_w ) { bo_w(space, offset + 0x3000, data, 1, 0); }

	// cartridge interface
	DECLARE_WRITE_LINE_MEMBER( extres_w ) { m_write_extres(state); }

protected:
	// device-level overrides
	virtual void device_start() override;

	// image-level overrides
	virtual image_init_result call_load() override;
	virtual const software_list_loader &get_software_list_loader() const override { return rom_software_list_loader::instance(); }

	virtual iodevice_t image_type() const override { return IO_CARTSLOT; }

	virtual bool is_readable()  const override { return 1; }
	virtual bool is_writeable() const override { return 0; }
	virtual bool is_creatable() const override { return 0; }
	virtual bool must_be_loaded() const override { return 0; }
	virtual bool is_reset_on_load() const override { return 1; }
	virtual const char *image_interface() const override { return "vidbrain_cart"; }
	virtual const char *file_extensions() const override { return "bin"; }

	// slot interface overrides
	virtual std::string get_default_card_software(get_default_card_software_hook &hook) const override;

	devcb_write_line   m_write_extres;

	device_videobrain_expansion_card_interface *m_cart;
};


// device type definition
DECLARE_DEVICE_TYPE(VIDEOBRAIN_EXPANSION_SLOT, videobrain_expansion_slot_device)


SLOT_INTERFACE_EXTERN( vidbrain_expansion_cards );

#endif // MAME_BUS_VIDBRAIN_EXP_H
