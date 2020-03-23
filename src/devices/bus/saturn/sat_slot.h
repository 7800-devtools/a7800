// license:BSD-3-Clause
// copyright-holders:Fabio Priuli
#ifndef MAME_BUS_SATURN_SAT_SLOT_H
#define MAME_BUS_SATURN_SAT_SLOT_H

#include "softlist_dev.h"


/***************************************************************************
 TYPE DEFINITIONS
 ***************************************************************************/


// ======================> device_sat_cart_interface

class device_sat_cart_interface : public device_slot_card_interface
{
public:
	virtual ~device_sat_cart_interface();

	// reading from ROM
	virtual DECLARE_READ32_MEMBER(read_rom) { return 0xffffffff; }
	// reading and writing to Extended DRAM chips
	virtual DECLARE_READ32_MEMBER(read_ext_dram0) { return 0xffffffff; }
	virtual DECLARE_WRITE32_MEMBER(write_ext_dram0) { }
	virtual DECLARE_READ32_MEMBER(read_ext_dram1) { return 0xffffffff; }
	virtual DECLARE_WRITE32_MEMBER(write_ext_dram1) { }
	// reading and writing to Extended BRAM chip
	virtual DECLARE_READ32_MEMBER(read_ext_bram) { return 0xffffffff; }
	virtual DECLARE_WRITE32_MEMBER(write_ext_bram) { }

	int get_cart_type() const { return m_cart_type; }

	void rom_alloc(uint32_t size, const char *tag);
	void bram_alloc(uint32_t size);
	void dram0_alloc(uint32_t size);
	void dram1_alloc(uint32_t size);
	uint32_t* get_rom_base() { return m_rom; }
	uint32_t* get_ext_dram0_base() { return &m_ext_dram0[0]; }
	uint32_t* get_ext_dram1_base() { return &m_ext_dram1[0]; }
	uint8_t*  get_ext_bram_base() { return &m_ext_bram[0]; }
	uint32_t  get_rom_size() { return m_rom_size; }
	uint32_t  get_ext_dram0_size() { return m_ext_dram0.size()*sizeof(uint32_t); }
	uint32_t  get_ext_dram1_size() { return m_ext_dram1.size()*sizeof(uint32_t); }
	uint32_t  get_ext_bram_size() { return m_ext_bram.size(); }

protected:
	// construction/destruction
	device_sat_cart_interface(const machine_config &mconfig, device_t &device, int cart_type);

	const int m_cart_type;

	// internal state
	uint32_t *m_rom;
	uint32_t m_rom_size;
	std::vector<uint32_t> m_ext_dram0;
	std::vector<uint32_t> m_ext_dram1;
	std::vector<uint8_t> m_ext_bram;
};


// ======================> sat_cart_slot_device

class sat_cart_slot_device : public device_t,
								public device_image_interface,
								public device_slot_interface
{
public:
	// construction/destruction
	sat_cart_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	virtual ~sat_cart_slot_device();

	// image-level overrides
	virtual image_init_result call_load() override;
	virtual void call_unload() override;
	virtual const software_list_loader &get_software_list_loader() const override { return rom_software_list_loader::instance(); }

	int get_cart_type();

	virtual iodevice_t image_type() const override { return IO_CARTSLOT; }
	virtual bool is_readable()  const override { return 1; }
	virtual bool is_writeable() const override { return 0; }
	virtual bool is_creatable() const override { return 0; }
	virtual bool must_be_loaded() const override { return 0; }
	virtual bool is_reset_on_load() const override { return 1; }
	virtual const char *image_interface() const override { return "sat_cart"; }
	virtual const char *file_extensions() const override { return "bin"; }

	// slot interface overrides
	virtual std::string get_default_card_software(get_default_card_software_hook &hook) const override;

	// reading and writing
	virtual DECLARE_READ32_MEMBER(read_rom);
	virtual DECLARE_READ32_MEMBER(read_ext_dram0);
	virtual DECLARE_WRITE32_MEMBER(write_ext_dram0);
	virtual DECLARE_READ32_MEMBER(read_ext_dram1);
	virtual DECLARE_WRITE32_MEMBER(write_ext_dram1);
	virtual DECLARE_READ32_MEMBER(read_ext_bram);
	virtual DECLARE_WRITE32_MEMBER(write_ext_bram);

protected:
	// device-level overrides
	virtual void device_start() override;

private:
	device_sat_cart_interface*       m_cart;
};


// device type definition
DECLARE_DEVICE_TYPE(SATURN_CART_SLOT, sat_cart_slot_device)


/***************************************************************************
 DEVICE CONFIGURATION MACROS
 ***************************************************************************/

#define SATSLOT_ROM_REGION_TAG ":cart:rom"

#define MCFG_SATURN_CARTRIDGE_ADD(_tag,_slot_intf,_def_slot) \
	MCFG_DEVICE_ADD(_tag, SATURN_CART_SLOT, 0)  \
	MCFG_DEVICE_SLOT_INTERFACE(_slot_intf, _def_slot, false)


#endif // MAME_BUS_SATURN_SAT_SLOT_H
