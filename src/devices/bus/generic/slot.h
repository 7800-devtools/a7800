// license:BSD-3-Clause
// copyright-holders:Fabio Priuli
#ifndef MAME_BUS_GENERIC_SLOT_H
#define MAME_BUS_GENERIC_SLOT_H

#pragma once

#include "softlist_dev.h"


/***************************************************************************
 TYPE DEFINITIONS
 ***************************************************************************/


// ======================> device_generic_cart_interface

class device_generic_cart_interface : public device_slot_card_interface
{
public:
	// construction/destruction
	virtual ~device_generic_cart_interface();

	// reading and writing
	virtual DECLARE_READ8_MEMBER(read_rom) { return 0xff; }
	virtual DECLARE_READ16_MEMBER(read16_rom) { return 0xffff; }
	virtual DECLARE_READ32_MEMBER(read32_rom) { return 0xffffffff; }

	virtual DECLARE_READ8_MEMBER(read_ram) { return 0xff; }
	virtual DECLARE_WRITE8_MEMBER(write_ram) {};

	virtual void rom_alloc(size_t size, int width, endianness_t end, const char *tag);
	virtual void ram_alloc(uint32_t size);

	uint8_t* get_rom_base()  { return m_rom; }
	uint32_t get_rom_size() { return m_rom_size; }

	uint8_t* get_region_base()  { if (m_region.found()) return m_region->base(); return nullptr; }
	uint32_t get_region_size() { if (m_region.found()) return m_region->bytes(); return 0; }

	uint8_t* get_ram_base() { return &m_ram[0]; }
	uint32_t get_ram_size() { return m_ram.size(); }

	void save_ram() { device().save_item(NAME(m_ram)); }

protected:
	device_generic_cart_interface(const machine_config &mconfig, device_t &device);

	// internal state
	uint8_t  *m_rom;
	uint32_t  m_rom_size;
	std::vector<uint8_t> m_ram;

	// this replaces m_rom for non-user configurable carts!
	optional_memory_region  m_region;
};


enum
{
	GENERIC_ROM8_WIDTH = 1,
	GENERIC_ROM16_WIDTH = 2,
	GENERIC_ROM32_WIDTH = 4
};

#define GENERIC_ROM_REGION_TAG ":cart:rom"



#define MCFG_GENERIC_MANDATORY       \
	static_cast<generic_slot_device *>(device)->set_must_be_loaded(true);

#define MCFG_GENERIC_WIDTH(_width)       \
	static_cast<generic_slot_device *>(device)->set_width(_width);

#define MCFG_GENERIC_ENDIAN(_endianness)       \
	static_cast<generic_slot_device *>(device)->set_endian(_endianness);

#define MCFG_GENERIC_DEFAULT_CARD(_def_card)      \
	static_cast<generic_slot_device *>(device)->set_default_card(_def_card);

#define MCFG_GENERIC_INTERFACE(_intf)       \
	static_cast<generic_slot_device *>(device)->set_interface(_intf);

#define MCFG_GENERIC_EXTENSIONS(_ext)       \
	static_cast<generic_slot_device *>(device)->set_extensions(_ext);

#define MCFG_GENERIC_LOAD(_class, _method)                                \
	generic_slot_device::static_set_device_load(*device, device_image_load_delegate(&DEVICE_IMAGE_LOAD_NAME(_class,_method), downcast<_class *>(owner)));

#define MCFG_GENERIC_UNLOAD(_class, _method)                            \
	generic_slot_device::static_set_device_unload(*device, device_image_func_delegate(&DEVICE_IMAGE_UNLOAD_NAME(_class,_method), downcast<_class *>(owner)));



// ======================> generic_slot_device

class generic_slot_device : public device_t,
								public device_image_interface,
								public device_slot_interface
{
public:
	// construction/destruction
	generic_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	virtual ~generic_slot_device();

	static void static_set_device_load(device_t &device, device_image_load_delegate callback) { downcast<generic_slot_device &>(device).m_device_image_load = callback; }
	static void static_set_device_unload(device_t &device, device_image_func_delegate callback) { downcast<generic_slot_device &>(device).m_device_image_unload = callback; }

	void set_interface(const char * interface) { m_interface = interface; }
	void set_default_card(const char * def) { m_default_card = def; }
	void set_extensions(const char * exts) { m_extensions = exts; }
	void set_must_be_loaded(bool mandatory) { m_must_be_loaded = mandatory; }
	void set_width(int width) { m_width = width; }
	void set_endian(endianness_t end) { m_endianness = end; }

	// device-level overrides
	virtual void device_start() override;

	// image-level overrides
	virtual image_init_result call_load() override;
	virtual void call_unload() override;
	virtual const software_list_loader &get_software_list_loader() const override { return rom_software_list_loader::instance(); }

	uint32_t common_get_size(const char *region);
	void common_load_rom(uint8_t *ROM, uint32_t len, const char *region);

	virtual iodevice_t image_type() const override { return IO_CARTSLOT; }
	virtual bool is_readable()  const override { return 1; }
	virtual bool is_writeable() const override { return 0; }
	virtual bool is_creatable() const override { return 0; }
	virtual bool must_be_loaded() const override { return m_must_be_loaded; }
	virtual bool is_reset_on_load() const override { return 1; }
	virtual const char *image_interface() const override { return m_interface; }
	virtual const char *file_extensions() const override { return m_extensions; }

	// slot interface overrides
	virtual std::string get_default_card_software(get_default_card_software_hook &hook) const override;

	// reading and writing
	virtual DECLARE_READ8_MEMBER(read_rom);
	virtual DECLARE_READ16_MEMBER(read16_rom);
	virtual DECLARE_READ32_MEMBER(read32_rom);

	virtual DECLARE_READ8_MEMBER(read_ram);
	virtual DECLARE_WRITE8_MEMBER(write_ram);

	virtual void rom_alloc(size_t size, int width, endianness_t end) { if (m_cart) m_cart->rom_alloc(size, width, end, tag()); }
	virtual void ram_alloc(uint32_t size)  { if (m_cart) m_cart->ram_alloc(size); }

	uint8_t* get_rom_base()
	{
		if (m_cart)
		{
			if (!user_loadable())
				return m_cart->get_region_base();
			else
				return m_cart->get_rom_base();
		}
		return nullptr;
	}
	uint32_t get_rom_size()
	{
		if (m_cart)
		{
			if (!user_loadable())
				return m_cart->get_region_size();
			else
				return m_cart->get_rom_size();
		}
		return 0;
	}
	uint8_t* get_ram_base() { if (m_cart) return m_cart->get_ram_base(); return nullptr; }

	void save_ram() { if (m_cart && m_cart->get_ram_size()) m_cart->save_ram(); }

protected:
	const char *m_interface;
	const char *m_default_card;
	const char *m_extensions;
	bool        m_must_be_loaded;
	int         m_width;
	endianness_t m_endianness;
	device_generic_cart_interface  *m_cart;
	device_image_load_delegate      m_device_image_load;
	device_image_func_delegate      m_device_image_unload;
};


// device type definition
DECLARE_DEVICE_TYPE(GENERIC_SOCKET, generic_slot_device)


/***************************************************************************
 DEVICE CONFIGURATION MACROS
 ***************************************************************************/

#define MCFG_GENERIC_CARTSLOT_ADD(_tag, _slot_intf, _dev_intf) \
	MCFG_DEVICE_ADD(_tag, GENERIC_SOCKET, 0) \
	MCFG_DEVICE_SLOT_INTERFACE(_slot_intf, nullptr, false) \
	MCFG_GENERIC_INTERFACE(_dev_intf)
#define MCFG_GENERIC_SOCKET_ADD(_tag, _slot_intf, _dev_intf) \
	MCFG_DEVICE_ADD(_tag, GENERIC_SOCKET, 0) \
	MCFG_DEVICE_SLOT_INTERFACE(_slot_intf, nullptr, false) \
	MCFG_GENERIC_INTERFACE(_dev_intf)

#define MCFG_GENERIC_CARTSLOT_ADD_WITH_DEFAULT(_tag, _slot_intf, _dev_intf, _default) \
	MCFG_DEVICE_ADD(_tag, GENERIC_SOCKET, 0) \
	MCFG_DEVICE_SLOT_INTERFACE(_slot_intf, _default, false) \
	MCFG_GENERIC_INTERFACE(_dev_intf)

#endif // MAME_BUS_GENERIC_SLOT_H
