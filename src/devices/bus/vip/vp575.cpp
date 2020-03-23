// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    RCA VIP Expansion Board VP-575 emulation

**********************************************************************/

#include "emu.h"
#include "vp575.h"



//**************************************************************************
//  DEVICE DEFINITIONS
//**************************************************************************

DEFINE_DEVICE_TYPE(VP575, vp575_device, "vp575", "VP-575 System Expansion")


//-------------------------------------------------
//  VIP_EXPANSION_INTERFACE( expansion_intf )
//-------------------------------------------------

void vp575_device::update_interrupts()
{
	int interrupt = CLEAR_LINE;
	int dma_out = CLEAR_LINE;
	int dma_in = CLEAR_LINE;

	for (int i = 0; i < MAX_SLOTS; i++)
	{
		interrupt |= m_int[i];
		dma_out |= m_dma_out[i];
		dma_in |= m_dma_in[i];
	}

	m_slot->interrupt_w(interrupt);
	m_slot->dma_out_w(dma_out);
	m_slot->dma_in_w(dma_in);
}


//-------------------------------------------------
//  MACHINE_CONFIG_START( vp575 )
//-------------------------------------------------

MACHINE_CONFIG_MEMBER( vp575_device::device_add_mconfig )
	MCFG_VIP_EXPANSION_SLOT_ADD("exp1", XTAL_3_52128MHz/2, vip_expansion_cards, nullptr)
	MCFG_VIP_EXPANSION_SLOT_INT_CALLBACK(WRITELINE(vp575_device, exp1_int_w))
	MCFG_VIP_EXPANSION_SLOT_DMA_OUT_CALLBACK(WRITELINE(vp575_device, exp1_dma_out_w))
	MCFG_VIP_EXPANSION_SLOT_DMA_IN_CALLBACK(WRITELINE(vp575_device, exp1_dma_in_w))

	MCFG_VIP_EXPANSION_SLOT_ADD("exp2", XTAL_3_52128MHz/2, vip_expansion_cards, nullptr)
	MCFG_VIP_EXPANSION_SLOT_INT_CALLBACK(WRITELINE(vp575_device, exp2_int_w))
	MCFG_VIP_EXPANSION_SLOT_DMA_OUT_CALLBACK(WRITELINE(vp575_device, exp2_dma_out_w))
	MCFG_VIP_EXPANSION_SLOT_DMA_IN_CALLBACK(WRITELINE(vp575_device, exp2_dma_in_w))

	MCFG_VIP_EXPANSION_SLOT_ADD("exp3", XTAL_3_52128MHz/2, vip_expansion_cards, nullptr)
	MCFG_VIP_EXPANSION_SLOT_INT_CALLBACK(WRITELINE(vp575_device, exp3_int_w))
	MCFG_VIP_EXPANSION_SLOT_DMA_OUT_CALLBACK(WRITELINE(vp575_device, exp3_dma_out_w))
	MCFG_VIP_EXPANSION_SLOT_DMA_IN_CALLBACK(WRITELINE(vp575_device, exp3_dma_in_w))

	MCFG_VIP_EXPANSION_SLOT_ADD("exp4", XTAL_3_52128MHz/2, vip_expansion_cards, nullptr)
	MCFG_VIP_EXPANSION_SLOT_INT_CALLBACK(WRITELINE(vp575_device, exp4_int_w))
	MCFG_VIP_EXPANSION_SLOT_DMA_OUT_CALLBACK(WRITELINE(vp575_device, exp4_dma_out_w))
	MCFG_VIP_EXPANSION_SLOT_DMA_IN_CALLBACK(WRITELINE(vp575_device, exp4_dma_in_w))

	MCFG_VIP_EXPANSION_SLOT_ADD("exp5", XTAL_3_52128MHz/2, vip_expansion_cards, nullptr)
	MCFG_VIP_EXPANSION_SLOT_INT_CALLBACK(WRITELINE(vp575_device, exp5_int_w))
	MCFG_VIP_EXPANSION_SLOT_DMA_OUT_CALLBACK(WRITELINE(vp575_device, exp5_dma_out_w))
	MCFG_VIP_EXPANSION_SLOT_DMA_IN_CALLBACK(WRITELINE(vp575_device, exp5_dma_in_w))
MACHINE_CONFIG_END



//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  vp575_device - constructor
//-------------------------------------------------

vp575_device::vp575_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, VP575, tag, owner, clock),
	device_vip_expansion_card_interface(mconfig, *this),
	m_expansion_slot(*this, "exp%u", 1)
{
	for (int i = 0; i < MAX_SLOTS; i++)
	{
		m_int[i] = CLEAR_LINE;
		m_dma_out[i] = CLEAR_LINE;
		m_dma_in[i] = CLEAR_LINE;
	}
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void vp575_device::device_start()
{
}


//-------------------------------------------------
//  vip_program_r - program read
//-------------------------------------------------

uint8_t vp575_device::vip_program_r(address_space &space, offs_t offset, int cs, int cdef, int *minh)
{
	uint8_t data = 0xff;

	for (auto & elem : m_expansion_slot)
	{
		data &= elem->program_r(space, offset, cs, cdef, minh);
	}

	return data;
}


//-------------------------------------------------
//  vip_program_w - program write
//-------------------------------------------------

void vp575_device::vip_program_w(address_space &space, offs_t offset, uint8_t data, int cdef, int *minh)
{
	for (auto & elem : m_expansion_slot)
	{
		elem->program_w(space, offset, data, cdef, minh);
	}
}


//-------------------------------------------------
//  vip_io_r - I/O read
//-------------------------------------------------

uint8_t vp575_device::vip_io_r(address_space &space, offs_t offset)
{
	uint8_t data = 0xff;

	for (auto & elem : m_expansion_slot)
	{
		data &= elem->io_r(space, offset);
	}

	return data;
}


//-------------------------------------------------
//  vip_io_w - I/O write
//-------------------------------------------------

void vp575_device::vip_io_w(address_space &space, offs_t offset, uint8_t data)
{
	for (auto & elem : m_expansion_slot)
	{
		elem->io_w(space, offset, data);
	}
}


//-------------------------------------------------
//  vip_dma_r - DMA read
//-------------------------------------------------

uint8_t vp575_device::vip_dma_r(address_space &space, offs_t offset)
{
	uint8_t data = 0xff;

	for (auto & elem : m_expansion_slot)
	{
		data &= elem->dma_r(space, offset);
	}

	return data;
}


//-------------------------------------------------
//  vip_dma_w - DMA write
//-------------------------------------------------

void vp575_device::vip_dma_w(address_space &space, offs_t offset, uint8_t data)
{
	for (auto & elem : m_expansion_slot)
	{
		elem->dma_w(space, offset, data);
	}
}


//-------------------------------------------------
//  vip_screen_update - screen update
//-------------------------------------------------

uint32_t vp575_device::vip_screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
	uint32_t data = 0;

	for (auto & elem : m_expansion_slot)
	{
		data |= elem->screen_update(screen, bitmap, cliprect);
	}

	return data;
}


//-------------------------------------------------
//  vip_ef1_r - EF1 flag read
//-------------------------------------------------

int vp575_device::vip_ef1_r()
{
	int state = CLEAR_LINE;

	for (auto & elem : m_expansion_slot)
	{
		state |= elem->ef1_r();
	}

	return state;
}


//-------------------------------------------------
//  vip_ef3_r - EF3 flag read
//-------------------------------------------------

int vp575_device::vip_ef3_r()
{
	int state = CLEAR_LINE;

	for (auto & elem : m_expansion_slot)
	{
		state |= elem->ef3_r();
	}

	return state;
}


//-------------------------------------------------
//  vip_ef4_r - EF4 flag read
//-------------------------------------------------

int vp575_device::vip_ef4_r()
{
	int state = CLEAR_LINE;

	for (auto & elem : m_expansion_slot)
	{
		state |= elem->ef4_r();
	}

	return state;
}


//-------------------------------------------------
//  vip_sc_w - status code write
//-------------------------------------------------

void vp575_device::vip_sc_w(int data)
{
	for (auto & elem : m_expansion_slot)
	{
		elem->sc_w(data);
	}
}


//-------------------------------------------------
//  vip_q_w - Q write
//-------------------------------------------------

void vp575_device::vip_q_w(int state)
{
	for (auto & elem : m_expansion_slot)
	{
		elem->q_w(state);
	}
}


//-------------------------------------------------
//  vip_run_w - RUN write
//-------------------------------------------------

void vp575_device::vip_run_w(int state)
{
	for (auto & elem : m_expansion_slot)
	{
		elem->run_w(state);
	}
}
