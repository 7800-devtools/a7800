// license:BSD-3-Clause
// copyright-holders:Fabio Priuli
/***********************************************************************************************************

    Bally Astrocade Expansion port

 ***********************************************************************************************************/


#include "emu.h"
#include "exp.h"

//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

DEFINE_DEVICE_TYPE(ASTROCADE_EXP_SLOT, astrocade_exp_device, "astrocade_exp", "Bally Astrocade expansion")


device_astrocade_card_interface::device_astrocade_card_interface(const machine_config &mconfig, device_t &device)
	: device_slot_card_interface(mconfig, device)
{
}


device_astrocade_card_interface::~device_astrocade_card_interface()
{
}


//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  astrocade_exp_device - constructor
//-------------------------------------------------
astrocade_exp_device::astrocade_exp_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, ASTROCADE_EXP_SLOT, tag, owner, clock),
	device_slot_interface(mconfig, *this),
	m_card_mounted(false), m_card(nullptr)
{
}


//-------------------------------------------------
//  astrocade_exp_device - destructor
//-------------------------------------------------

astrocade_exp_device::~astrocade_exp_device()
{
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void astrocade_exp_device::device_start()
{
	m_card = dynamic_cast<device_astrocade_card_interface *>(get_card_device());
	if (m_card)
		m_card_mounted = true;
}

/*-------------------------------------------------
 read
 -------------------------------------------------*/

READ8_MEMBER(astrocade_exp_device::read)
{
	if (m_card)
		return m_card->read(space, offset);
	else
		return 0xff;
}

/*-------------------------------------------------
 write
 -------------------------------------------------*/

WRITE8_MEMBER(astrocade_exp_device::write)
{
	if (m_card)
		m_card->write(space, offset, data);
}
