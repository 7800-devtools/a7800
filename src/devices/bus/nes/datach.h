// license:BSD-3-Clause
// copyright-holders:Fabio Priuli
#ifndef MAME_BUS_NES_DATACH_H
#define MAME_BUS_NES_DATACH_H

#pragma once

#include "bandai.h"
#include "softlist_dev.h"
#include "machine/i2cmem.h"
#include "machine/bcreader.h"

//--------------------------------
//
//  Datach Cartslot implementation
//
//--------------------------------

// ======================> datach_cart_interface

class datach_cart_interface : public device_slot_card_interface
{
public:
	// construction/destruction
	virtual ~datach_cart_interface();

	// reading and writing
	virtual DECLARE_READ8_MEMBER(read);

	uint8_t *get_cart_base() { return m_rom; }
	void write_prg_bank(uint8_t bank) { m_bank = bank; }

protected:
	datach_cart_interface(const machine_config &mconfig, device_t &device);

	optional_device<i2cmem_device> m_i2cmem;

	// internal state
	uint8_t *m_rom;
	// ROM is accessed via two 16K banks, but only the first one can be switched
	uint8_t m_bank;
};

// ======================> nes_datach_slot_device

class nes_datach_device;

class nes_datach_slot_device : public device_t,
								public device_image_interface,
								public device_slot_interface
{
	friend class nes_datach_device;
public:
	// construction/destruction
	nes_datach_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	virtual ~nes_datach_slot_device();

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
	virtual const char *image_interface() const override { return "datach_cart"; }
	virtual const char *file_extensions() const override { return "nes,bin"; }

	// slot interface overrides
	virtual std::string get_default_card_software(get_default_card_software_hook &hook) const override;

	virtual DECLARE_READ8_MEMBER(read);
	void write_prg_bank(uint8_t bank) { if (m_cart) m_cart->write_prg_bank(bank); }

protected:
	datach_cart_interface*      m_cart;
};

// device type definition
DECLARE_DEVICE_TYPE(NES_DATACH_SLOT, nes_datach_slot_device)


#define MCFG_DATACH_MINICART_ADD(_tag, _slot_intf) \
		MCFG_DEVICE_ADD(_tag, NES_DATACH_SLOT, 0) \
		MCFG_DEVICE_SLOT_INTERFACE(_slot_intf, nullptr, false)


//--------------------------------
//
//  Datach Minicart implementation
//
//--------------------------------

// ======================> nes_datach_rom_device

class nes_datach_rom_device : public device_t, public datach_cart_interface
{
public:
	// construction/destruction
	nes_datach_rom_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// optional information overrides
	virtual const tiny_rom_entry *device_rom_region() const override;
	virtual uint8_t* get_cart_base();

protected:
	nes_datach_rom_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
};

// ======================> nes_datach_24c01_device

class nes_datach_24c01_device : public nes_datach_rom_device
{
public:
	// construction/destruction
	nes_datach_24c01_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	// optional information overrides
	virtual void device_add_mconfig(machine_config &config) override;
};

// device type definition
DECLARE_DEVICE_TYPE(NES_DATACH_ROM,   nes_datach_rom_device)
DECLARE_DEVICE_TYPE(NES_DATACH_24C01, nes_datach_24c01_device)


//---------------------------------
//
//  Datach Base Unit implementation
//
//---------------------------------

// ======================> nes_datach_device

class nes_datach_device : public nes_lz93d50_device
{
public:
	// construction/destruction
	nes_datach_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual DECLARE_READ8_MEMBER(read_m) override;
	virtual DECLARE_READ8_MEMBER(read_h) override;
	virtual DECLARE_WRITE8_MEMBER(write_h) override;

	virtual void pcb_reset() override;

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;
	virtual void device_add_mconfig(machine_config &config) override;

	uint8_t m_datach_latch;
	required_device<i2cmem_device> m_i2cmem;
	required_device<barcode_reader_device> m_reader;
	required_device<nes_datach_slot_device> m_subslot;
	uint8_t m_i2c_dir;
	uint8_t m_i2c_in_use;

	static const device_timer_id TIMER_SERIAL = 1;
	emu_timer *serial_timer;
};


// device type definition
DECLARE_DEVICE_TYPE(NES_DATACH, nes_datach_device)

#endif // MAME_BUS_NES_DATACH_H
