// license:BSD-3-Clause
// copyright-holders:Fabio Priuli
#ifndef MAME_BUS_VC4000_SLOT_H
#define MAME_BUS_VC4000_SLOT_H

#include "softlist_dev.h"


/***************************************************************************
 TYPE DEFINITIONS
 ***************************************************************************/


/* PCB */
enum
{
	VC4000_STD = 0,
	VC4000_ROM4K,
	VC4000_RAM1K,
	VC4000_CHESS2
};


// ======================> device_vc4000_cart_interface

class device_vc4000_cart_interface : public device_slot_card_interface
{
public:
	// construction/destruction
	virtual ~device_vc4000_cart_interface();

	// reading and writing
	virtual DECLARE_READ8_MEMBER(read_rom) { return 0xff; }
	virtual DECLARE_READ8_MEMBER(extra_rom) { return 0xff; }
	virtual DECLARE_READ8_MEMBER(read_ram) { return 0xff; }
	virtual DECLARE_WRITE8_MEMBER(write_ram) {}

	void rom_alloc(uint32_t size, const char *tag);
	void ram_alloc(uint32_t size);
	uint8_t* get_rom_base() { return m_rom; }
	uint8_t* get_ram_base() { return &m_ram[0]; }
	uint32_t get_rom_size() { return m_rom_size; }
	uint32_t get_ram_size() { return m_ram.size(); }

	void save_ram() { device().save_item(NAME(m_ram)); }

protected:
	device_vc4000_cart_interface(const machine_config &mconfig, device_t &device);

	// internal state
	uint8_t *m_rom;
	uint32_t m_rom_size;
	std::vector<uint8_t> m_ram;
};


// ======================> vc4000_cart_slot_device

class vc4000_cart_slot_device : public device_t,
								public device_image_interface,
								public device_slot_interface
{
public:
	// construction/destruction
	vc4000_cart_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	virtual ~vc4000_cart_slot_device();

	// image-level overrides
	virtual image_init_result call_load() override;
	virtual void call_unload() override { }
	virtual const software_list_loader &get_software_list_loader() const override { return rom_software_list_loader::instance(); }

	int get_type() { return m_type; }

	void save_ram() { if (m_cart && m_cart->get_ram_size()) m_cart->save_ram(); }

	virtual iodevice_t image_type() const override { return IO_CARTSLOT; }
	virtual bool is_readable()  const override { return 1; }
	virtual bool is_writeable() const override { return 0; }
	virtual bool is_creatable() const override { return 0; }
	virtual bool must_be_loaded() const override { return 0; }
	virtual bool is_reset_on_load() const override { return 1; }
	virtual const char *image_interface() const override { return "vc4000_cart"; }
	virtual const char *file_extensions() const override { return "bin,rom"; }

	// slot interface overrides
	virtual std::string get_default_card_software(get_default_card_software_hook &hook) const override;

	// reading and writing
	virtual DECLARE_READ8_MEMBER(read_rom);
	virtual DECLARE_READ8_MEMBER(extra_rom);
	virtual DECLARE_READ8_MEMBER(read_ram);
	virtual DECLARE_WRITE8_MEMBER(write_ram);

protected:
	// device-level overrides
	virtual void device_start() override;

	vc4000_cart_slot_device(
			const machine_config &mconfig,
			device_type type,
			const char *tag,
			device_t *owner,
			uint32_t clock);

	int m_type;
	device_vc4000_cart_interface *m_cart;
};

class h21_cart_slot_device : public vc4000_cart_slot_device
{
public:
	// construction/destruction
	h21_cart_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	virtual ~h21_cart_slot_device();

	virtual const char *image_interface() const override { return "h21_cart"; }
};

// device type definition
DECLARE_DEVICE_TYPE(VC4000_CART_SLOT, vc4000_cart_slot_device)
DECLARE_DEVICE_TYPE(H21_CART_SLOT,    h21_cart_slot_device)


/***************************************************************************
 DEVICE CONFIGURATION MACROS
 ***************************************************************************/

#define VC4000SLOT_ROM_REGION_TAG ":cart:rom"

#define MCFG_VC4000_CARTRIDGE_ADD(_tag,_slot_intf,_def_slot) \
	MCFG_DEVICE_ADD(_tag, VC4000_CART_SLOT, 0) \
	MCFG_DEVICE_SLOT_INTERFACE(_slot_intf, _def_slot, false)

#define MCFG_H21_CARTRIDGE_ADD(_tag,_slot_intf,_def_slot) \
	MCFG_DEVICE_ADD(_tag, H21_CART_SLOT, 0) \
	MCFG_DEVICE_SLOT_INTERFACE(_slot_intf, _def_slot, false)

#endif // MAME_BUS_VC4000_SLOT_H
