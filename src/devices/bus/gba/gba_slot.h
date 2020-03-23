// license:BSD-3-Clause
// copyright-holders:R. Belmont,Ryan Holtz,Fabio Priuli
#ifndef MAME_BUS_GBA_GBA_SLOT_H
#define MAME_BUS_GBA_GBA_SLOT_H

#pragma once

#include "softlist_dev.h"


/***************************************************************************
 TYPE DEFINITIONS
 ***************************************************************************/


/* PCB */
enum
{
	GBA_STD = 0,
	GBA_SRAM,
	GBA_DRILLDOZ,
	GBA_WARIOTWS,
	GBA_EEPROM,
	GBA_EEPROM4,
	GBA_YOSHIUG,
	GBA_EEPROM64,
	GBA_BOKTAI,
	GBA_FLASH,
	GBA_FLASH_RTC,
	GBA_FLASH512,
	GBA_FLASH1M,
	GBA_FLASH1M_RTC,
	GBA_3DMATRIX
};


// ======================> device_gba_cart_interface

class device_gba_cart_interface : public device_slot_card_interface
{
public:
	// construction/destruction
	virtual ~device_gba_cart_interface();

	// reading and writing
	virtual DECLARE_READ32_MEMBER(read_rom) { return 0xffffffff; }
	virtual DECLARE_READ32_MEMBER(read_ram) { return 0xffffffff; }
	virtual DECLARE_READ32_MEMBER(read_gpio) { return 0; }
	virtual DECLARE_READ32_MEMBER(read_tilt) { return 0xffffffff; }
	virtual DECLARE_WRITE32_MEMBER(write_ram) { }
	virtual DECLARE_WRITE32_MEMBER(write_gpio) { }
	virtual DECLARE_WRITE32_MEMBER(write_tilt) { }
	virtual DECLARE_WRITE32_MEMBER(write_mapper) { }

	void rom_alloc(uint32_t size, const char *tag);
	void nvram_alloc(uint32_t size);
	uint32_t* get_rom_base() { return m_rom; }
	uint32_t* get_romhlp_base() { return m_romhlp; }
	uint32_t* get_nvram_base() { return &m_nvram[0]; }
	uint32_t get_rom_size() { return m_rom_size; }
	uint32_t get_romhlp_size() { return m_romhlp_size; }
	uint32_t get_nvram_size() { return m_nvram.size()*sizeof(uint32_t); }
	void set_rom_size(uint32_t val) { m_rom_size = val; }

	void save_nvram()   { device().save_item(NAME(m_nvram)); }

protected:
	device_gba_cart_interface(const machine_config &mconfig, device_t &device);

	// internal state
	uint32_t *m_rom;  // this points to the cart rom region
	uint32_t m_rom_size;  // this is the actual game size, not the rom region size!
	uint32_t *m_romhlp;
	uint32_t m_romhlp_size;
	std::vector<uint32_t> m_nvram;
};


// ======================> gba_cart_slot_device

class gba_cart_slot_device : public device_t,
								public device_image_interface,
								public device_slot_interface
{
public:
	// construction/destruction
	gba_cart_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	virtual ~gba_cart_slot_device();

	// device-level overrides
	virtual void device_start() override;

	// image-level overrides
	virtual image_init_result call_load() override;
	virtual void call_unload() override;
	virtual const software_list_loader &get_software_list_loader() const override { return rom_software_list_loader::instance(); }

	int get_type() { return m_type; }
	static int get_cart_type(const uint8_t *ROM, uint32_t len);

	void internal_header_logging(uint8_t *ROM, uint32_t len);

	void save_nvram() { if (m_cart && m_cart->get_nvram_size()) m_cart->save_nvram(); }
	uint32_t get_rom_size() { if (m_cart) return m_cart->get_rom_size(); return 0; }

	virtual iodevice_t image_type() const override { return IO_CARTSLOT; }
	virtual bool is_readable()  const override { return 1; }
	virtual bool is_writeable() const override { return 0; }
	virtual bool is_creatable() const override { return 0; }
	virtual bool must_be_loaded() const override { return 0; }
	virtual bool is_reset_on_load() const override { return 1; }
	virtual const char *image_interface() const override { return "gba_cart"; }
	virtual const char *file_extensions() const override { return "gba,bin"; }

	// slot interface overrides
	virtual std::string get_default_card_software(get_default_card_software_hook &hook) const override;

	// reading and writing
	virtual DECLARE_READ32_MEMBER(read_rom);
	virtual DECLARE_READ32_MEMBER(read_ram);
	virtual DECLARE_READ32_MEMBER(read_gpio);
	virtual DECLARE_READ32_MEMBER(read_tilt) { if (m_cart) return m_cart->read_tilt(space, offset, mem_mask); else return 0xffffffff; }
	virtual DECLARE_WRITE32_MEMBER(write_ram);
	virtual DECLARE_WRITE32_MEMBER(write_gpio);
	virtual DECLARE_WRITE32_MEMBER(write_tilt) { if (m_cart) m_cart->write_tilt(space, offset, data, mem_mask); }
	virtual DECLARE_WRITE32_MEMBER(write_mapper) { if (m_cart) m_cart->write_mapper(space, offset, data, mem_mask); }


protected:

	int m_type;
	device_gba_cart_interface* m_cart;
};



// device type definition
DECLARE_DEVICE_TYPE(GBA_CART_SLOT, gba_cart_slot_device)


/***************************************************************************
 DEVICE CONFIGURATION MACROS
 ***************************************************************************/

#define GBASLOT_ROM_REGION_TAG ":cart:rom"
#define GBAHELP_ROM_REGION_TAG ":cart:romhlp"

#define MCFG_GBA_CARTRIDGE_ADD(_tag,_slot_intf,_def_slot) \
	MCFG_DEVICE_ADD(_tag, GBA_CART_SLOT, 0) \
	MCFG_DEVICE_SLOT_INTERFACE(_slot_intf, _def_slot, false)



//------------------------------------------------------------------------
//
// Misc structs to attempt NVRAM identification when loading from fullpath
//
//------------------------------------------------------------------------


#define GBA_CHIP_EEPROM     (1 << 0)
#define GBA_CHIP_SRAM       (1 << 1)
#define GBA_CHIP_FLASH      (1 << 2)
#define GBA_CHIP_FLASH_1M   (1 << 3)
#define GBA_CHIP_RTC        (1 << 4)
#define GBA_CHIP_FLASH_512  (1 << 5)
#define GBA_CHIP_EEPROM_64K (1 << 6)
#define GBA_CHIP_EEPROM_4K  (1 << 7)


#endif // MAME_BUS_GBA_GBA_SLOT_H
