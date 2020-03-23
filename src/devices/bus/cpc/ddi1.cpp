// license:BSD-3-Clause
// copyright-holders:Barry Rodewald
/*
 * ddi1.cpp  --  Amstrad DDI-1 Floppy Disk Drive interface
 */

#include "emu.h"
#include "ddi1.h"
#include "softlist.h"

SLOT_INTERFACE_EXTERN(cpc_exp_cards);

//**************************************************************************
//  DEVICE DEFINITIONS
//**************************************************************************

DEFINE_DEVICE_TYPE(CPC_DDI1, cpc_ddi1_device, "cpc_ddi1", "Amstrad DDI-1")

static SLOT_INTERFACE_START( ddi1_floppies )
	SLOT_INTERFACE( "3ssdd", FLOPPY_3_SSDD )
SLOT_INTERFACE_END

//-------------------------------------------------
//  Device ROM definition
//-------------------------------------------------

ROM_START( cpc_ddi1 )
	ROM_REGION( 0x4000, "disc_rom", 0 )
	ROM_LOAD("cpcados.rom",  0x0000, 0x4000, CRC(1fe22ecd) SHA1(39102c8e9cb55fcc0b9b62098780ed4a3cb6a4bb))
ROM_END

//-------------------------------------------------
//  rom_region - device-specific ROM region
//-------------------------------------------------

const tiny_rom_entry *cpc_ddi1_device::device_rom_region() const
{
	return ROM_NAME( cpc_ddi1 );
}

// device machine config
MACHINE_CONFIG_MEMBER( cpc_ddi1_device::device_add_mconfig )
	MCFG_UPD765A_ADD("upd765", true, true)
	MCFG_FLOPPY_DRIVE_ADD("upd765:0", ddi1_floppies, "3ssdd", floppy_image_device::default_floppy_formats)
	MCFG_SOFTWARE_LIST_ADD("flop_list","cpc_flop")

	// pass-through
	MCFG_DEVICE_ADD("exp", CPC_EXPANSION_SLOT, 0)
	MCFG_DEVICE_SLOT_INTERFACE(cpc_exp_cards, nullptr, false)
	MCFG_CPC_EXPANSION_SLOT_OUT_IRQ_CB(DEVWRITELINE("^", cpc_expansion_slot_device, irq_w))
	MCFG_CPC_EXPANSION_SLOT_OUT_NMI_CB(DEVWRITELINE("^", cpc_expansion_slot_device, nmi_w))
	MCFG_CPC_EXPANSION_SLOT_OUT_ROMDIS_CB(DEVWRITELINE("^", cpc_expansion_slot_device, romdis_w))  // ROMDIS

MACHINE_CONFIG_END


//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

cpc_ddi1_device::cpc_ddi1_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, CPC_DDI1, tag, owner, clock),
	device_cpc_expansion_card_interface(mconfig, *this), m_slot(nullptr),
	m_fdc(*this,"upd765"),
	m_connector(*this,"upd765:0"), m_rom_active(false), m_romen(false)
{
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void cpc_ddi1_device::device_start()
{
	device_t* cpu = machine().device("maincpu");
	address_space& space = cpu->memory().space(AS_IO);
	m_slot = dynamic_cast<cpc_expansion_slot_device *>(owner());

	space.install_write_handler(0xfa7e,0xfa7f,write8_delegate(FUNC(cpc_ddi1_device::motor_w),this));
	space.install_readwrite_handler(0xfb7e,0xfb7f,read8_delegate(FUNC(cpc_ddi1_device::fdc_r),this),write8_delegate(FUNC(cpc_ddi1_device::fdc_w),this));
	space.install_write_handler(0xdf00,0xdfff,write8_delegate(FUNC(cpc_ddi1_device::rombank_w),this));
}

//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void cpc_ddi1_device::device_reset()
{
	m_rom_active = false;
}

WRITE8_MEMBER(cpc_ddi1_device::motor_w)
{
	switch(offset)
	{
	case 0x00:
	case 0x01:
		{
			/* FDC Motor Control - Bit 0 defines the state of the FDD motor:
			 * "1" the FDD motor will be active.
			 * "0" the FDD motor will be in-active.*/
			floppy_image_device *floppy;
			if(m_connector)
			{
				floppy = m_connector->get_device();
				if(floppy)
					floppy->mon_w(!BIT(data, 0));
			}
			break;
		}
	}
}

WRITE8_MEMBER(cpc_ddi1_device::fdc_w)
{
	switch(offset)
	{
	case 0x01:
		m_fdc->fifo_w(space, 0,data);
		break;
	}
}

READ8_MEMBER(cpc_ddi1_device::fdc_r)
{
	uint8_t data = 0xff;

	switch(offset)
	{
	case 0x00:
		data = m_fdc->msr_r(space, 0);
		break;
	case 0x01:
		data = m_fdc->fifo_r(space, 0);
		break;
	}
	return data;
}

WRITE8_MEMBER(cpc_ddi1_device::rombank_w)
{
	if(data == 0x07)
		m_rom_active = true;
	else
		m_rom_active = false;
	m_slot->rom_select(space,0,data);
}

void cpc_ddi1_device::set_mapping(uint8_t type)
{
	if(type != MAP_UPPER)
		return;
	if(m_rom_active)
	{
		uint8_t* ROM = memregion("disc_rom")->base();
		membank(":bank7")->set_base(ROM);
		membank(":bank8")->set_base(ROM+0x2000);
	}
}
