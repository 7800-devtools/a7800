// license:BSD-3-Clause
// copyright-holders:Barry Rodewald
/*

    Dobbertin HD20 hard disk

*/

#include "emu.h"
#include "hd20.h"

//**************************************************************************
//  DEVICE DEFINITIONS
//**************************************************************************

DEFINE_DEVICE_TYPE(CPC_HD20, cpc_hd20_device, "cpc_hd20", "Dobbertin HD20")

MACHINE_CONFIG_MEMBER( cpc_hd20_device::device_add_mconfig )
	MCFG_DEVICE_ADD("hdc",ST11M_HDC,0)
	MCFG_XTHDC_IRQ_HANDLER(WRITELINE(cpc_hd20_device, irq_w))
	MCFG_HARDDISK_ADD("hdc:primary")
	// no pass-through (?)
MACHINE_CONFIG_END


ROM_START( cpc_hd20 )
	ROM_REGION( 0x4000, "exp_rom", 0 )
	ROM_DEFAULT_BIOS("xddos210")

	ROM_SYSTEM_BIOS( 0, "xddos210", "X-DDOS 2.10" )
	ROMX_LOAD( "xddos210.rom",   0x0000, 0x4000, CRC(5477fdb4) SHA1(2f1bd4d6e2d2e62818b01e6e7a26488362a7a8ee), ROM_BIOS(1) )
	ROM_SYSTEM_BIOS( 1, "xddos200", "X-DDOS 2.00" )
	ROMX_LOAD( "x-ddos20.rom",   0x0000, 0x4000, CRC(c2d9cc03) SHA1(8a20788be5f957e84e849c226aa97b55b2a3aab9), ROM_BIOS(2) )
ROM_END

const tiny_rom_entry *cpc_hd20_device::device_rom_region() const
{
	return ROM_NAME( cpc_hd20 );
}

//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

cpc_hd20_device::cpc_hd20_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, CPC_HD20, tag, owner, clock),
	device_cpc_expansion_card_interface(mconfig, *this), m_slot(nullptr),
	m_hdc(*this,"hdc")
{
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void cpc_hd20_device::device_start()
{
	device_t* cpu = machine().device("maincpu");
	address_space& space = cpu->memory().space(AS_IO);
	m_slot = dynamic_cast<cpc_expansion_slot_device *>(owner());

	space.install_write_handler(0xfbe0,0xfbe4,write8_delegate(FUNC(cpc_hd20_device::hdc_w),this));
	space.install_read_handler(0xfbe0,0xfbe4,read8_delegate(FUNC(cpc_hd20_device::hdc_r),this));
}

//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void cpc_hd20_device::device_reset()
{
	// TODO
}

READ8_MEMBER(cpc_hd20_device::hdc_r)
{
	uint8_t ret = 0x00;

	switch(offset)
	{
	case 0:
		ret = m_hdc->data_r();
		break;
	case 1:
		ret = m_hdc->status_r();
		break;
	case 2:
		m_hdc->set_ready();
		ret = 0x01;
		break;
	case 4:
		m_hdc->reset_w(0);  // reset on read also?
		break;
	}

	return ret;
}

WRITE8_MEMBER(cpc_hd20_device::hdc_w)
{
	switch(offset)
	{
	case 0:
		m_hdc->data_w(data);
		break;
	case 1:
		m_hdc->reset_w(data);
		break;
	case 2:
		m_hdc->select_w(data);
		break;
	case 3:
		m_hdc->control_w(data);
		break;
	case 4:
		m_hdc->reset_w(data);
		break;
	}
}

WRITE_LINE_MEMBER(cpc_hd20_device::irq_w)
{
//  if(state)
//      m_hdc->set_ready();
}
