// license:BSD-3-Clause
// copyright-holders:Fabio Priuli
#ifndef MAME_BUS_PCE_PCE_SLOT_H
#define MAME_BUS_PCE_PCE_SLOT_H

#pragma once

#include "softlist_dev.h"


/***************************************************************************
 TYPE DEFINITIONS
 ***************************************************************************/


/* PCB */
enum
{
	PCE_STD = 0,
	PCE_CDSYS3J,
	PCE_CDSYS3U,
	PCE_POPULOUS,
	PCE_SF2
};


// ======================> device_pce_cart_interface

class device_pce_cart_interface : public device_slot_card_interface
{
public:
	// construction/destruction
	virtual ~device_pce_cart_interface();

	// reading and writing
	virtual DECLARE_READ8_MEMBER(read_cart) { return 0xff; }
	virtual DECLARE_WRITE8_MEMBER(write_cart) {};

	void rom_alloc(uint32_t size, const char *tag);
	void ram_alloc(uint32_t size);
	uint8_t* get_rom_base() { return m_rom; }
	uint8_t* get_ram_base() { return &m_ram[0]; }
	uint32_t get_rom_size() { return m_rom_size; }
	uint32_t get_ram_size() { return m_ram.size(); }

	void rom_map_setup(uint32_t size);

protected:
	device_pce_cart_interface(const machine_config &mconfig, device_t &device);

	// internal state
	uint8_t *m_rom;
	uint32_t m_rom_size;
	std::vector<uint8_t> m_ram;

	uint8_t rom_bank_map[8];    // 128K chunks of rom
};


// ======================> pce_cart_slot_device

class pce_cart_slot_device : public device_t,
								public device_image_interface,
								public device_slot_interface
{
public:
	// construction/destruction
	pce_cart_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	virtual ~pce_cart_slot_device();

	// device-level overrides
	virtual void device_start() override;

	// image-level overrides
	virtual image_init_result call_load() override;
	virtual void call_unload() override;
	virtual const software_list_loader &get_software_list_loader() const override { return rom_software_list_loader::instance(); }

	int get_type() { return m_type; }
	static int get_cart_type(const uint8_t *ROM, uint32_t len);

	void internal_header_logging(uint8_t *ROM, uint32_t len);

	void set_intf(const char * interface) { m_interface = interface; }

	virtual iodevice_t image_type() const override { return IO_CARTSLOT; }
	virtual bool is_readable()  const override { return 1; }
	virtual bool is_writeable() const override { return 0; }
	virtual bool is_creatable() const override { return 0; }
	virtual bool must_be_loaded() const override { return 1; }
	virtual bool is_reset_on_load() const override { return 1; }
	virtual const char *image_interface() const override { return m_interface; }
	virtual const char *file_extensions() const override { return "pce,bin"; }

	// slot interface overrides
	virtual std::string get_default_card_software(get_default_card_software_hook &hook) const override;

	// reading and writing
	virtual DECLARE_READ8_MEMBER(read_cart);
	virtual DECLARE_WRITE8_MEMBER(write_cart);

protected:
	const char *m_interface;
	int m_type;
	device_pce_cart_interface *m_cart;
};



// device type definition
DECLARE_DEVICE_TYPE(PCE_CART_SLOT, pce_cart_slot_device)


/***************************************************************************
 DEVICE CONFIGURATION MACROS
 ***************************************************************************/

#define PCESLOT_ROM_REGION_TAG ":cart:rom"

#define MCFG_PCE_CARTRIDGE_ADD(_tag,_slot_intf,_def_slot) \
	MCFG_DEVICE_ADD(_tag, PCE_CART_SLOT, 0) \
	MCFG_DEVICE_SLOT_INTERFACE(_slot_intf, _def_slot, false) \
	static_cast<pce_cart_slot_device *>(device)->set_intf("pce_cart");

#define MCFG_TG16_CARTRIDGE_ADD(_tag,_slot_intf,_def_slot) \
	MCFG_DEVICE_ADD(_tag, PCE_CART_SLOT, 0) \
	MCFG_DEVICE_SLOT_INTERFACE(_slot_intf, _def_slot, false) \
	static_cast<pce_cart_slot_device *>(device)->set_intf("tg16_cart");


#endif // MAME_BUS_PCE_PCE_SLOT_H
