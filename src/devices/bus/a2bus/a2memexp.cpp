// license:BSD-3-Clause
// copyright-holders:R. Belmont
/*********************************************************************

    a2memexp.c

    Implementation of the Apple II Memory Expansion Card

*********************************************************************/

#include "emu.h"
#include "a2memexp.h"


/***************************************************************************
    PARAMETERS
***************************************************************************/

//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

DEFINE_DEVICE_TYPE(A2BUS_MEMEXP,    a2bus_memexpapple_device, "a2memexp", "Apple II Memory Expansion Card")
DEFINE_DEVICE_TYPE(A2BUS_RAMFACTOR, a2bus_ramfactor_device,   "a2ramfac", "Applied Engineering RamFactor")

#define MEMEXP_ROM_REGION  "memexp_rom"

ROM_START( memexp )
	ROM_REGION(0x1000, MEMEXP_ROM_REGION, 0)
	ROM_LOAD( "341-0344a.bin", 0x0000, 0x1000, CRC(1e994e17) SHA1(6e823a1fa40ed37eeddcef23f5df24da2ea1463e) )
ROM_END

ROM_START( ramfactor )
	ROM_REGION(0x8000, MEMEXP_ROM_REGION, 0)
	ROM_LOAD( "ae ramfactor rom v1.4.bin", 0x0000, 0x2000, CRC(1c56d646) SHA1(32cb02a6a915dd3962bfa1f0184a94253e03ba6b) )
	ROM_LOAD( "ae ramfactor rom v1.3.bin", 0x2000, 0x2000, CRC(434f5c45) SHA1(cf31a370951bd9a10d5b77f179134b358683b8fa) )
	ROM_LOAD( "ae ramfactor rom v1.1.bin", 0x4000, 0x2000, CRC(328907a3) SHA1(dc25b4133a52609799098d8918a289fd973d28d9) )
	ROM_LOAD( "ae ramfactor rom v1.0.bin", 0x6000, 0x2000, CRC(39c2162a) SHA1(9286d35907939aadb1fffd3e1d75603fe3e846ad) )
ROM_END

/***************************************************************************
    FUNCTION PROTOTYPES
***************************************************************************/

//-------------------------------------------------
//  device_add_mconfig - add device configuration
//-------------------------------------------------

MACHINE_CONFIG_MEMBER( a2bus_memexp_device::device_add_mconfig )
MACHINE_CONFIG_END

//-------------------------------------------------
//  rom_region - device-specific ROM region
//-------------------------------------------------

const tiny_rom_entry *a2bus_memexp_device::device_rom_region() const
{
	return ROM_NAME( memexp );
}

const tiny_rom_entry *a2bus_ramfactor_device::device_rom_region() const
{
	return ROM_NAME( ramfactor );
}

//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

a2bus_memexp_device::a2bus_memexp_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, type, tag, owner, clock),
	device_a2bus_card_interface(mconfig, *this), m_isramfactor(false), m_bankhior(0), m_addrmask(0), m_rom(nullptr), m_wptr(0), m_liveptr(0)
{
}

a2bus_memexpapple_device::a2bus_memexpapple_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	a2bus_memexp_device(mconfig, A2BUS_MEMEXP, tag, owner, clock)
{
	m_isramfactor = false;
	m_bankhior = 0xf0;
	m_addrmask = 0xfffff;
}

a2bus_ramfactor_device::a2bus_ramfactor_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	a2bus_memexp_device(mconfig, A2BUS_RAMFACTOR, tag, owner, clock)
{
	m_isramfactor = true;
	m_bankhior = 0x00;
	m_addrmask = 0x7fffff;
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void a2bus_memexp_device::device_start()
{
	// set_a2bus_device makes m_slot valid
	set_a2bus_device();

	m_rom = device().machine().root_device().memregion(this->subtag(MEMEXP_ROM_REGION).c_str())->base();

	memset(m_ram, 0xff, 1024*1024*sizeof(uint8_t));

	save_item(NAME(m_regs));
	save_item(NAME(m_ram));
	save_item(NAME(m_wptr));
	save_item(NAME(m_liveptr));
}

void a2bus_memexp_device::device_reset()
{
	memset(m_regs, 0, sizeof(uint8_t) * 0x10);
	m_wptr = m_liveptr = 0;
}


/*-------------------------------------------------
    read_c0nx - called for reads from this card's c0nx space
-------------------------------------------------*/

uint8_t a2bus_memexp_device::read_c0nx(address_space &space, uint8_t offset)
{
	uint8_t retval = m_regs[offset];

	if (offset == 3)
	{
		retval = m_ram[m_liveptr&m_addrmask];
//        printf("Read RAM[%x] = %02x\n", m_liveptr, retval);
		m_liveptr++;
		m_regs[0] = m_liveptr & 0xff;
		m_regs[1] = (m_liveptr>>8) & 0xff;
		m_regs[2] = ((m_liveptr>>16) & 0xff) | m_bankhior;
	}

//    printf("Read c0n%x (PC=%x) = %02x\n", offset, space.device().safe_pc(), retval);

	return retval;
}


/*-------------------------------------------------
    write_c0nx - called for writes to this card's c0nx space
-------------------------------------------------*/

void a2bus_memexp_device::write_c0nx(address_space &space, uint8_t offset, uint8_t data)
{
//    printf("Write %02x to c0n%x (PC=%x)\n", data, offset, space.device().safe_pc());

	switch (offset)
	{
		case 0:
			m_wptr &= ~0xff;
			m_wptr |= data;
			m_regs[0] = m_wptr & 0xff;
			m_regs[1] = (m_wptr>>8) & 0xff;
			m_regs[2] = ((m_wptr>>16) & 0xff) | m_bankhior;
			m_liveptr = m_wptr;
			break;

		case 1:
			m_wptr &= ~0xff00;
			m_wptr |= (data<<8);
			m_regs[0] = m_wptr & 0xff;
			m_regs[1] = (m_wptr>>8) & 0xff;
			m_regs[2] = ((m_wptr>>16) & 0xff) | m_bankhior;
			m_liveptr = m_wptr;
			break;

		case 2:
			m_wptr &= ~0xff0000;
			m_wptr |= (data<<16);
			m_regs[0] = m_wptr & 0xff;
			m_regs[1] = (m_wptr>>8) & 0xff;
			m_regs[2] = ((m_wptr>>16) & 0xff) | m_bankhior;
			m_liveptr = m_wptr;
			break;

		case 3:
//            printf("Write %02x to RAM[%x]\n", data, m_liveptr);
			m_ram[(m_liveptr&m_addrmask)] = data;
			m_liveptr++;
			m_regs[0] = m_liveptr & 0xff;
			m_regs[1] = (m_liveptr>>8) & 0xff;
			m_regs[2] = ((m_liveptr>>16) & 0xff) | m_bankhior;
			break;

		default:
			m_regs[offset] = data;
			break;
	}
}

/*-------------------------------------------------
    read_cnxx - called for reads from this card's cnxx space
-------------------------------------------------*/

uint8_t a2bus_memexp_device::read_cnxx(address_space &space, uint8_t offset)
{
	int slotimg = m_slot * 0x100;

	// first 0x400 of ROM contains a CnXX image for each of slots 1-7, last 0x400 is c800 image
	if ((m_isramfactor) && (m_regs[0xf] & 0x01))
	{
		return m_rom[offset+slotimg+0x1000];
	}

	return m_rom[offset+slotimg];
}

/*-------------------------------------------------
    read_c800 - called for reads from this card's c800 space
-------------------------------------------------*/

uint8_t a2bus_memexp_device::read_c800(address_space &space, uint16_t offset)
{
	// c70a diags confirm: bit 1 of cn0F banks in the second half of the ROM
	if ((m_isramfactor) && (m_regs[0xf] & 0x01))
	{
		return m_rom[offset+0x1800];
	}

	return m_rom[offset+0x800];
}
