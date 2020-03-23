// license:BSD-3-Clause
// copyright-holders:Nigel Barnes
/**********************************************************************

    BBC 1MHz Bus emulation

**********************************************************************/

#include "emu.h"
#include "1mhzbus.h"


//**************************************************************************
//  DEVICE DEFINITIONS
//**************************************************************************

DEFINE_DEVICE_TYPE(BBC_1MHZBUS_SLOT, bbc_1mhzbus_slot_device, "bbc_1mhzbus_slot", "BBC Micro 1MHz Bus port")



//**************************************************************************
//  DEVICE BBC_1MHZBUS PORT INTERFACE
//**************************************************************************

//-------------------------------------------------
//  device_bbc_1mhzbus_interface - constructor
//-------------------------------------------------

device_bbc_1mhzbus_interface::device_bbc_1mhzbus_interface(const machine_config &mconfig, device_t &device)
	: device_slot_card_interface(mconfig, device)
{
	m_slot = dynamic_cast<bbc_1mhzbus_slot_device *>(device.owner());
}


//-------------------------------------------------
//  ~device_bbc_1mhzbus_interface - destructor
//-------------------------------------------------

device_bbc_1mhzbus_interface::~device_bbc_1mhzbus_interface()
{
}



//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  bbc_1mhzbus_slot_device - constructor
//-------------------------------------------------

bbc_1mhzbus_slot_device::bbc_1mhzbus_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, BBC_1MHZBUS_SLOT, tag, owner, clock),
	device_slot_interface(mconfig, *this),
	m_card(nullptr),
	m_irq_handler(*this),
	m_nmi_handler(*this)
{
}


//-------------------------------------------------
//  bbc_1mhzbus_slot_device - destructor
//-------------------------------------------------

bbc_1mhzbus_slot_device::~bbc_1mhzbus_slot_device()
{
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void bbc_1mhzbus_slot_device::device_start()
{
	m_card = dynamic_cast<device_bbc_1mhzbus_interface *>(get_card_device());

	// resolve callbacks
	m_irq_handler.resolve_safe();
	m_nmi_handler.resolve_safe();
}


//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void bbc_1mhzbus_slot_device::device_reset()
{
	if (get_card_device())
	{
		get_card_device()->reset();
	}
}


//-------------------------------------------------
//  SLOT_INTERFACE( bbc_1mhzbus_devices )
//-------------------------------------------------


// slot devices
//#include "teletext.h"
//#include "ieee488.h"
//#include "music500.h"
//#include "music5000.h"
#include "opus3.h"
//#include "ramdisc.h"
//#include "torchg400.h"
//#include "torchg800.h"
//#include "beebsid.h"
//#include "prisma3.h"


SLOT_INTERFACE_START(bbcb_1mhzbus_devices)
//  SLOT_INTERFACE("teletext",  BBC_TELETEXT)        /* Acorn ANE01 Teletext Adapter */
//  SLOT_INTERFACE("ieee488",   BBC_IEEE488)         /* Acorn ANK01 IEEE488 Interface */
//  SLOT_INTERFACE("music500",  BBC_MUSIC500)        /* Acorn ANV02 Music500 */
//  SLOT_INTERFACE("music2000", BBC_MUSIC2000)       /* Hybrid Music 2000 MIDI Interface */
//  SLOT_INTERFACE("music3000", BBC_MUSIC3000)       /* Hybrid Music 3000 Expander */
//  SLOT_INTERFACE("music5000", BBC_MUSIC5000)       /* Hybrid Music 5000 Synthesiser */
	SLOT_INTERFACE("opus3",     BBC_OPUS3)           /* Opus Challenger 3 */
//  SLOT_INTERFACE("ramdisc",   BBC_RAMDISC)         /* Morley Electronics RAM Disc */
//  SLOT_INTERFACE("torchg400", BBC_TORCHG400)       /* Torch Graduate G400 */
//  SLOT_INTERFACE("torchg800", BBC_TORCHG800)       /* Torch Graduate G800 */
//  SLOT_INTERFACE("beebsid",   BBC_BEEBSID)         /* BeebSID */
//  SLOT_INTERFACE("prisma3",   BBC_PRISMA3)         /* Prisma 3 - Millipede 1989 */
SLOT_INTERFACE_END

SLOT_INTERFACE_START( bbcm_1mhzbus_devices )
//  SLOT_INTERFACE("teletext",  BBC_TELETEXT)        /* Acorn ANE01 Teletext Adapter */
//  SLOT_INTERFACE("ieee488",   BBC_IEEE488)         /* Acorn ANK01 IEEE488 Interface */
//  SLOT_INTERFACE("music500",  BBC_MUSIC500)        /* Acorn ANV02 Music500 */
//  SLOT_INTERFACE("music2000", BBC_MUSIC2000)       /* Hybrid Music 2000 MIDI Interface */
//  SLOT_INTERFACE("music3000", BBC_MUSIC3000)       /* Hybrid Music 3000 Expander */
//  SLOT_INTERFACE("music5000", BBC_MUSIC5000)       /* Hybrid Music 5000 Synthesiser */
//  SLOT_INTERFACE("ramdisc",   BBC_RAMDISC)         /* Morley Electronics RAM Disc */
//  SLOT_INTERFACE("beebsid",   BBC_BEEBSID)         /* BeebSID */
//  SLOT_INTERFACE("prisma3",   BBC_PRISMA3)         /* Prisma 3 - Millipede 1989 */
SLOT_INTERFACE_END
