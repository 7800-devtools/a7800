// license:BSD-3-Clause
// copyright-holders:Fabio Priuli
#ifndef MAME_BUS_A7800_A78_SLOT_H
#define MAME_BUS_A7800_A78_SLOT_H

#pragma once

#include "softlist_dev.h"


/***************************************************************************
 TYPE DEFINITIONS
 ***************************************************************************/


/* PCB */
enum
{
	A78_TYPE0 = 0,      // standard 8K/16K/32K games, no bankswitch
	A78_TYPE1,          // as TYPE0 + POKEY chip on the PCB
	A78_TYPE2,          // Atari SuperGame pcb (8x16K banks with bankswitch)
	A78_TYPE3,          // as TYPE1 + POKEY chip on the PCB
	A78_TYPE6,          // as TYPE1 + RAM IC on the PCB
	A78_TYPEA,          // Alien Brigade, Crossbow (9x16K banks with diff bankswitch)
	A78_TYPE8,          // Rescue on Fractalus, as TYPE0 + 2K Mirror RAM IC on the PCB
	A78_ABSOLUTE,       // F18 Hornet
	A78_ACTIVISION,     // Double Dragon, Rampage
	A78_HSC,            // Atari HighScore cart
	A78_XB_BOARD,       // A7800 Expansion Board (it shall more or less apply to the Expansion Module too, but this is not officially released yet)
	A78_XM_BOARD,       // A7800 XM Expansion Module (theoretical specs only, since this is not officially released yet)
	A78_MEGACART,               // Homebrew by CPUWIZ, consists of SuperGame bank up to 512K + 32K RAM banked
	A78_VERSABOARD = 0x10,      // Homebrew by CPUWIZ, consists of SuperGame bank up to 256K + 32K RAM banked
	// VersaBoard variants configured as Type 1/3/A or VersaBoard + POKEY at $0450
	A78_TYPE0_POK450 = 0x20,
	A78_TYPE1_POK450 = 0x21,
	A78_TYPE6_POK450 = 0x24,
	A78_TYPEA_POK450 = 0x25,
	A78_VERSA_POK450 = 0x30
};


// ======================> device_a78_cart_interface

class device_a78_cart_interface : public device_slot_card_interface
{
public:
	// construction/destruction
	virtual ~device_a78_cart_interface();

	// memory accessor
	virtual DECLARE_READ8_MEMBER(read_04xx) { return 0xff; }
	virtual DECLARE_READ8_MEMBER(read_10xx) { return 0xff; }
	virtual DECLARE_READ8_MEMBER(read_30xx) { return 0xff; }
	virtual DECLARE_READ8_MEMBER(read_40xx) { return 0xff; }
	virtual DECLARE_WRITE8_MEMBER(write_04xx) {}
	virtual DECLARE_WRITE8_MEMBER(write_10xx) {}
	virtual DECLARE_WRITE8_MEMBER(write_30xx) {}
	virtual DECLARE_WRITE8_MEMBER(write_40xx) {}

	void rom_alloc(uint32_t size, const char *tag);
	void ram_alloc(uint32_t size);
	void nvram_alloc(uint32_t size);
	uint8_t* get_rom_base() { return m_rom; }
	uint8_t* get_ram_base() { return &m_ram[0]; }
	uint8_t* get_nvram_base() { return &m_nvram[0]; }
	uint32_t get_rom_size() { return m_rom_size; }
	uint32_t get_ram_size() { return m_ram.size(); }
	uint32_t get_nvram_size() { return m_nvram.size(); }

protected:
	device_a78_cart_interface(const machine_config &mconfig, device_t &device);

	// internal state
	uint8_t *m_rom;
	uint32_t m_rom_size;
	std::vector<uint8_t> m_ram;
	std::vector<uint8_t> m_nvram; // HiScore cart can save scores!
	// helpers
	uint32_t m_base_rom;
	int m_bank_mask;
};


// ======================> a78_cart_slot_device

class a78_cart_slot_device : public device_t,
								public device_image_interface,
								public device_slot_interface
{
public:
	// construction/destruction
	a78_cart_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	virtual ~a78_cart_slot_device();

	// device-level overrides
	virtual void device_start() override;

	// image-level overrides
	virtual image_init_result call_load() override;
	virtual void call_unload() override;
	virtual const software_list_loader &get_software_list_loader() const override { return rom_software_list_loader::instance(); }

	int get_cart_type() { return m_type; };
	bool has_cart() { return m_cart != nullptr; }

	virtual iodevice_t image_type() const override { return IO_CARTSLOT; }
	virtual bool is_readable()  const override { return 1; }
	virtual bool is_writeable() const override { return 0; }
	virtual bool is_creatable() const override { return 0; }
	virtual bool must_be_loaded() const override { return 0; }
	virtual bool is_reset_on_load() const override { return 1; }
	virtual const char *image_interface() const override { return "a7800_cart"; }
	virtual const char *file_extensions() const override { return "bin,a78"; }
	virtual u32 unhashed_header_length() const override { return 128; }

	// slot interface overrides
	virtual std::string get_default_card_software(get_default_card_software_hook &hook) const override;

	// reading and writing
	virtual DECLARE_READ8_MEMBER(read_04xx);
	virtual DECLARE_READ8_MEMBER(read_10xx);
	virtual DECLARE_READ8_MEMBER(read_30xx);
	virtual DECLARE_READ8_MEMBER(read_40xx);
	virtual DECLARE_WRITE8_MEMBER(write_04xx);
	virtual DECLARE_WRITE8_MEMBER(write_10xx);
	virtual DECLARE_WRITE8_MEMBER(write_30xx);
	virtual DECLARE_WRITE8_MEMBER(write_40xx);

private:
	device_a78_cart_interface*       m_cart;
	int m_type;

	image_verify_result verify_header(char *header);
	int validate_header(int head, bool log) const;
	void internal_header_logging(uint8_t *header, uint32_t len);
};


// device type definition
DECLARE_DEVICE_TYPE(A78_CART_SLOT, a78_cart_slot_device)


/***************************************************************************
 DEVICE CONFIGURATION MACROS
 ***************************************************************************/

#define A78SLOT_ROM_REGION_TAG ":cart:rom"

#define MCFG_A78_CARTRIDGE_ADD(_tag,_slot_intf,_def_slot) \
	MCFG_DEVICE_ADD(_tag, A78_CART_SLOT, 0)  \
	MCFG_DEVICE_SLOT_INTERFACE(_slot_intf, _def_slot, false)


#endif // MAME_BUS_A7800_A78_SLOT_H
