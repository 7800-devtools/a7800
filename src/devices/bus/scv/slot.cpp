// license:BSD-3-Clause
// copyright-holders:Fabio Priuli
/***********************************************************************************************************

    Epoch Super Cassette Vision cart emulation
    (through slot devices)

 ***********************************************************************************************************/


#include "emu.h"
#include "slot.h"

//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

DEFINE_DEVICE_TYPE(SCV_CART_SLOT, scv_cart_slot_device, "scv_cart_slot", "SCV Cartridge Slot")

//**************************************************************************
//    SCV cartridges Interface
//**************************************************************************

//-------------------------------------------------
//  device_scv_cart_interface - constructor
//-------------------------------------------------

device_scv_cart_interface::device_scv_cart_interface(const machine_config &mconfig, device_t &device)
	: device_slot_card_interface(mconfig, device),
		m_rom(nullptr),
		m_rom_size(0)
{
}


//-------------------------------------------------
//  ~device_scv_cart_interface - destructor
//-------------------------------------------------

device_scv_cart_interface::~device_scv_cart_interface()
{
}

//-------------------------------------------------
//  rom_alloc - alloc the space for the cart
//-------------------------------------------------

void device_scv_cart_interface::rom_alloc(uint32_t size, const char *tag)
{
	if (m_rom == nullptr)
	{
		m_rom = device().machine().memory().region_alloc(std::string(tag).append(SCVSLOT_ROM_REGION_TAG).c_str(), size, 1, ENDIANNESS_LITTLE)->base();
		m_rom_size = size;
	}
}


//-------------------------------------------------
//  ram_alloc - alloc the space for the ram
//-------------------------------------------------

void device_scv_cart_interface::ram_alloc(uint32_t size)
{
	m_ram.resize(size);
}


//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  scv_cart_slot_device - constructor
//-------------------------------------------------
scv_cart_slot_device::scv_cart_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, SCV_CART_SLOT, tag, owner, clock),
	device_image_interface(mconfig, *this),
	device_slot_interface(mconfig, *this),
	m_type(SCV_8K), m_cart(nullptr)
{
}


//-------------------------------------------------
//  scv_cart_slot_device - destructor
//-------------------------------------------------

scv_cart_slot_device::~scv_cart_slot_device()
{
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void scv_cart_slot_device::device_start()
{
	m_cart = dynamic_cast<device_scv_cart_interface *>(get_card_device());
}


//-------------------------------------------------
//  SCV PCB
//-------------------------------------------------

struct scv_slot
{
	int                     pcb_id;
	const char              *slot_option;
};

// Here, we take the feature attribute from .xml (i.e. the PCB name) and we assign a unique ID to it
static const scv_slot slot_list[] =
{
	{ SCV_8K,       "rom8k" },
	{ SCV_16K,      "rom16k" },
	{ SCV_32K,      "rom32k" },
	{ SCV_32K_RAM,  "rom32k_ram" },
	{ SCV_64K,      "rom64k" },
	{ SCV_128K,     "rom128k" },
	{ SCV_128K_RAM, "rom128k_ram" }
};

static int scv_get_pcb_id(const char *slot)
{
	for (auto & elem : slot_list)
	{
		if (!core_stricmp(elem.slot_option, slot))
			return elem.pcb_id;
	}

	return 0;
}

static const char *scv_get_slot(int type)
{
	for (auto & elem : slot_list)
	{
		if (elem.pcb_id == type)
			return elem.slot_option;
	}

	return "rom8k";
}


/*-------------------------------------------------
 call load
 -------------------------------------------------*/

image_init_result scv_cart_slot_device::call_load()
{
	if (m_cart)
	{
		uint8_t *ROM;
		uint32_t len = !loaded_through_softlist() ? length() : get_software_region_length("rom");
		bool has_ram = loaded_through_softlist() && get_software_region("ram");

		if (len > 0x20000)
		{
			seterror(IMAGE_ERROR_UNSPECIFIED, "Unsupported cartridge size");
			return image_init_result::FAIL;
		}

		m_cart->rom_alloc(len, tag());
		if (has_ram)
			m_cart->ram_alloc(get_software_region_length("ram"));

		ROM = m_cart->get_rom_base();

		if (!loaded_through_softlist())
			fread(ROM, len);
		else
			memcpy(ROM, get_software_region("rom"), len);

		if (!loaded_through_softlist())
			m_type = get_cart_type(ROM, len);
		else
		{
			const char *pcb_name = get_feature("slot");
			if (pcb_name)
				m_type = scv_get_pcb_id(pcb_name);
		}

		// for the moment we only support RAM from softlist and in the following configurations
		// 1) 32K ROM + 8K RAM; 2) 128K ROM + 4K RAM
		if (m_type == SCV_32K && has_ram)
			m_type = SCV_32K_RAM;
		if (m_type == SCV_128K && has_ram)
			m_type = SCV_128K_RAM;

		//printf("Type: %s\n", scv_get_slot(m_type));

		return image_init_result::PASS;
	}

	return image_init_result::PASS;
}


/*-------------------------------------------------
 get_cart_type - code to detect NVRAM type from
 fullpath
 -------------------------------------------------*/

int scv_cart_slot_device::get_cart_type(const uint8_t *ROM, uint32_t len)
{
	int type = SCV_8K;

	// TO DO: is there any way to identify carts with RAM?!?
	switch (len)
	{
		case 0x2000:
			type = SCV_8K;
			break;
		case 0x4000:
			type = SCV_16K;
			break;
		case 0x8000:
			type = SCV_32K;
			break;
		case 0x10000:
			type = SCV_64K;
			break;
		case 0x20000:
			type = SCV_128K;
			break;
	}

	return type;
}


/*-------------------------------------------------
 get default card software
 -------------------------------------------------*/

std::string scv_cart_slot_device::get_default_card_software(get_default_card_software_hook &hook) const
{
	if (hook.image_file())
	{
		const char *slot_string;
		uint32_t len = hook.image_file()->size();
		std::vector<uint8_t> rom(len);
		int type;

		hook.image_file()->read(&rom[0], len);

		type = get_cart_type(&rom[0], len);
		slot_string = scv_get_slot(type);

		//printf("type: %s\n", slot_string);

		return std::string(slot_string);
	}

	return software_get_default_slot("rom8k");
}

/*-------------------------------------------------
 read
 -------------------------------------------------*/

READ8_MEMBER(scv_cart_slot_device::read_cart)
{
	if (m_cart)
		return m_cart->read_cart(space, offset);
	else
		return 0xff;
}

/*-------------------------------------------------
 write
 -------------------------------------------------*/

WRITE8_MEMBER(scv_cart_slot_device::write_cart)
{
	if (m_cart)
		m_cart->write_cart(space, offset, data);
}


/*-------------------------------------------------
 write_bank
 -------------------------------------------------*/

WRITE8_MEMBER(scv_cart_slot_device::write_bank)
{
	if (m_cart)
		m_cart->write_bank(space, offset, data);
}
