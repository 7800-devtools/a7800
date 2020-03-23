// license:BSD-3-Clause
// copyright-holders:Miodrag Milanovic
/***************************************************************************

    IBM-PC printer interface

***************************************************************************/

#include "emu.h"
#include "lpt.h"
#include "machine/pc_lpt.h"

DEFINE_DEVICE_TYPE(ISA8_LPT, isa8_lpt_device, "isa_lpt", "Printer Adapter")

isa8_lpt_device::isa8_lpt_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, ISA8_LPT, tag, owner, clock),
	device_isa8_card_interface(mconfig, *this),
	m_is_primary(false)
{
}

MACHINE_CONFIG_MEMBER( isa8_lpt_device::device_add_mconfig )
	MCFG_DEVICE_ADD("lpt", PC_LPT, 0)
	MCFG_PC_LPT_IRQ_HANDLER(WRITELINE(isa8_lpt_device, pc_cpu_line))
MACHINE_CONFIG_END

static INPUT_PORTS_START( lpt_dsw )
	PORT_START("DSW")
	PORT_DIPNAME( 0x01, 0x00, "Base address")
	PORT_DIPSETTING(    0x00, "0x378" )
	PORT_DIPSETTING(    0x01, "0x278" )
INPUT_PORTS_END

ioport_constructor isa8_lpt_device::device_input_ports() const
{
	return INPUT_PORTS_NAME( lpt_dsw );
}

void isa8_lpt_device::device_start()
{
	set_isa_device();
}

void isa8_lpt_device::device_reset()
{
	m_is_primary = (ioport("DSW")->read() & 1) ? false : true;
	if (m_is_primary)
	{
		m_isa->install_device(0x0378, 0x037b, read8_delegate(FUNC(pc_lpt_device::read), subdevice<pc_lpt_device>("lpt")), write8_delegate(FUNC(pc_lpt_device::write), subdevice<pc_lpt_device>("lpt")));
	}
	else
	{
		m_isa->install_device(0x0278, 0x027b, read8_delegate(FUNC(pc_lpt_device::read), subdevice<pc_lpt_device>("lpt")), write8_delegate(FUNC(pc_lpt_device::write), subdevice<pc_lpt_device>("lpt")));
	}
}

WRITE_LINE_MEMBER(isa8_lpt_device::pc_cpu_line)
{
	if (is_primary())
		m_isa->irq7_w(state);
	else
		m_isa->irq5_w(state);
}
