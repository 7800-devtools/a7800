// license:GPL-2.0+
// copyright-holders:Dirk Best
/***************************************************************************

    Amiga Zorro Slots

    86-pin Expansion Slot (Zorro-I), Zorro-II, Zorro-III

***************************************************************************/

#include "emu.h"
#include "zorro.h"


//**************************************************************************
//  ZORRO SLOT DEVICE
//**************************************************************************

DEFINE_DEVICE_TYPE(ZORRO_SLOT, zorro_slot_device, "zorro_slot", "Zorro slot")

//-------------------------------------------------
//  zorro_slot_device - constructor
//-------------------------------------------------

zorro_slot_device::zorro_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	zorro_slot_device(mconfig, ZORRO_SLOT, tag, owner, clock)
{
}

zorro_slot_device::zorro_slot_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, type, tag, owner, clock),
	device_slot_interface(mconfig, *this),
	m_zorro_tag(nullptr)
{
}

void zorro_slot_device::set_zorro_slot(device_t &device, device_t *owner, const char *zorro_tag)
{
	zorro_slot_device &zorro_card = dynamic_cast<zorro_slot_device &>(device);
	zorro_card.m_owner = owner;
	zorro_card.m_zorro_tag = zorro_tag;
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void zorro_slot_device::device_start()
{
	device_zorro_card_interface *dev = dynamic_cast<device_zorro_card_interface *>(get_card_device());

	if (dev)
	{
		zorro_device *m_zorro_bus = downcast<zorro_device *>(m_owner->subdevice(m_zorro_tag));
		m_zorro_bus->add_card(dev);
	}
}


//**************************************************************************
//  BASE ZORRO BUS DEVICE
//**************************************************************************

//-------------------------------------------------
//  exp_slot_device - constructor
//-------------------------------------------------

zorro_device::zorro_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, type, tag, owner, clock),
	m_space(nullptr),
	m_cputag(nullptr),
	m_ovr_handler(*this),
	m_int2_handler(*this),
	m_int6_handler(*this)
{
}

//-------------------------------------------------
//  set_cputag - set cpu we are attached to
//-------------------------------------------------

void zorro_device::set_cputag(device_t &device, const char *tag)
{
	zorro_device &zorro = downcast<zorro_device &>(device);
	zorro.m_cputag = tag;
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void zorro_device::device_start()
{
	// get address space
	device_t *cpu = machine().device(m_cputag);
	m_space = &cpu->memory().space(AS_PROGRAM);

	// resolve callbacks
	m_ovr_handler.resolve_safe();
	m_int2_handler.resolve_safe();
	m_int6_handler.resolve_safe();
}

// from slot device
WRITE_LINE_MEMBER( zorro_device::ovr_w ) { m_ovr_handler(state); }
WRITE_LINE_MEMBER( zorro_device::int2_w ) { m_int2_handler(state); }
WRITE_LINE_MEMBER( zorro_device::int6_w ) { m_int6_handler(state); }


//**************************************************************************
//  EXPANSION SLOT DEVICE
//**************************************************************************

DEFINE_DEVICE_TYPE(EXP_SLOT, exp_slot_device, "exp_slot", "86-pin expansion slot")

//-------------------------------------------------
//  exp_slot_device - constructor
//-------------------------------------------------

exp_slot_device::exp_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	zorro_device(mconfig, EXP_SLOT, tag, owner, clock),
	m_ipl_handler(*this),
	m_dev(nullptr)
{
}

exp_slot_device::exp_slot_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock) :
	zorro_device(mconfig, type, tag, owner, clock),
	m_ipl_handler(*this),
	m_dev(nullptr)
{
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void exp_slot_device::device_start()
{
	// resolve callbacks
	m_ipl_handler.resolve_safe();

	// call base device start
	zorro_device::device_start();
}

//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void exp_slot_device::device_reset()
{
	// if we have a device, start the autoconfig chain
	if (m_dev)
		m_dev->cfgin_w(0);
}

//-------------------------------------------------
//  add_card - add new card to our bus
//-------------------------------------------------

void exp_slot_device::add_card(device_zorro_card_interface *card)
{
	m_dev = downcast<device_exp_card_interface *>(card);
	card->set_zorro_bus(this);
}

// from slot device
void exp_slot_device::ipl_w(int interrupt) { m_ipl_handler(0, interrupt, 0xff); }

// from host
void exp_slot_device::fc_w(int code) { if (m_dev) m_dev->fc_w(code); }


//**************************************************************************
//  ZORRO2 DEVICE
//**************************************************************************

DEFINE_DEVICE_TYPE(ZORRO2, zorro2_device, "zorro2", "Zorro-II bus")

//-------------------------------------------------
//  zorro2_device - constructor
//-------------------------------------------------

zorro2_device::zorro2_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	zorro2_device(mconfig, ZORRO2, tag, owner, clock)
{
}

zorro2_device::zorro2_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock) :
	zorro_device(mconfig, type, tag, owner, clock),
	m_eint1_handler(*this),
	m_eint4_handler(*this),
	m_eint5_handler(*this),
	m_eint7_handler(*this),
	m_autoconfig_device(nullptr)
{
}

//-------------------------------------------------
//  zorro2_device - destructor
//-------------------------------------------------

zorro2_device::~zorro2_device()
{
	m_dev.detach_all();
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void zorro2_device::device_start()
{
	// resolve callbacks
	m_eint1_handler.resolve_safe();
	m_eint4_handler.resolve_safe();
	m_eint5_handler.resolve_safe();
	m_eint7_handler.resolve_safe();

	// call base device start
	zorro_device::device_start();
}

//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void zorro2_device::device_reset()
{
	// initiate autoconfig
	m_autoconfig_device = m_dev.first();

	// if we have a device, start the autoconfig chain
	if (m_autoconfig_device)
		m_autoconfig_device->cfgin_w(0);
}

//-------------------------------------------------
//  add_card - add new card to our bus
//-------------------------------------------------

void zorro2_device::add_card(device_zorro_card_interface *card)
{
	device_zorro2_card_interface *zorro2_card = downcast<device_zorro2_card_interface *>(card);
	card->set_zorro_bus(this);
	m_dev.append(*zorro2_card);
}

// from slot device
WRITE_LINE_MEMBER( zorro2_device::eint1_w ) { m_eint1_handler(state); }
WRITE_LINE_MEMBER( zorro2_device::eint4_w ) { m_eint4_handler(state); }
WRITE_LINE_MEMBER( zorro2_device::eint5_w ) { m_eint5_handler(state); }
WRITE_LINE_MEMBER( zorro2_device::eint7_w ) { m_eint7_handler(state); }

WRITE_LINE_MEMBER( zorro2_device::cfgout_w )
{
	m_autoconfig_device = m_autoconfig_device->next();

	// if there is still a device in the chain, tell it to configure itself
	if (m_autoconfig_device)
		m_autoconfig_device->cfgin_w(0);
}

// from host
void zorro2_device::fc_w(int code)
{
	device_zorro2_card_interface *entry = m_dev.first();

	while (entry)
	{
		entry->fc_w(code);
		entry = entry->next();
	}
}


//**************************************************************************
//  ZORRO INTERFACE
//**************************************************************************

//-------------------------------------------------
//  device_zorro_card_interface - constructor
//-------------------------------------------------

device_zorro_card_interface::device_zorro_card_interface(const machine_config &mconfig, device_t &device) :
	device_slot_card_interface(mconfig, device),
	m_zorro(nullptr)
{
}

//-------------------------------------------------
//  ~device_zorro_card_interface - destructor
//-------------------------------------------------

device_zorro_card_interface::~device_zorro_card_interface()
{
}

void device_zorro_card_interface::set_zorro_bus(zorro_device *device)
{
	m_zorro = device;
}

void device_zorro_card_interface::fc_w(int code)
{
}

WRITE_LINE_MEMBER( device_zorro_card_interface::cfgin_w )
{
}


//**************************************************************************
//  EXPANSION CARD INTERFACE
//**************************************************************************

//-------------------------------------------------
//  device_exp_card_interface - constructor
//-------------------------------------------------

device_exp_card_interface::device_exp_card_interface(const machine_config &mconfig, device_t &device) :
	device_zorro_card_interface(mconfig, device),
	m_slot(nullptr)
{
}

//-------------------------------------------------
//  ~device_exp_card_interface - destructor
//-------------------------------------------------

device_exp_card_interface::~device_exp_card_interface()
{
}

void device_exp_card_interface::set_zorro_device()
{
	m_slot = dynamic_cast<exp_slot_device *>(m_zorro);
}


//**************************************************************************
//  ZORRO-II CARD INTERFACE
//**************************************************************************

//-------------------------------------------------
//  device_zorro2_interface - constructor
//-------------------------------------------------

device_zorro2_card_interface::device_zorro2_card_interface(const machine_config &mconfig, device_t &device) :
	device_zorro_card_interface(mconfig, device),
	m_next(nullptr),
	m_slot(nullptr)
{
}

//-------------------------------------------------
//  ~device_zorro2_interface - destructor
//-------------------------------------------------

device_zorro2_card_interface::~device_zorro2_card_interface()
{
}

void device_zorro2_card_interface::set_zorro_device()
{
	m_slot = dynamic_cast<zorro2_device *>(m_zorro);
}
