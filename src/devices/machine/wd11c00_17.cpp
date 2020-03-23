// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    Western Digital WD11C00-17 PC/XT Host Interface Logic Device

**********************************************************************/

#include "emu.h"
#include "machine/wd11c00_17.h"

#define VERBOSE 1
#include "logmacro.h"


//**************************************************************************
//  MACROS / CONSTANTS
//**************************************************************************

// status register
#define STATUS_IRQ      0x20
#define STATUS_DRQ      0x10
#define STATUS_BUSY     0x08
#define STATUS_C_D      0x04
#define STATUS_I_O      0x02
#define STATUS_REQ      0x01


// mask register
#define MASK_IRQ        0x02
#define MASK_DMA        0x01



//**************************************************************************
//  DEVICE DEFINITIONS
//**************************************************************************

DEFINE_DEVICE_TYPE(WD11C00_17, wd11c00_17_device, "wd11c00_17", "Western Digital WD11C00-17")


//**************************************************************************
//  INLINE HELPERS
//**************************************************************************

//-------------------------------------------------
//  check_interrupt -
//-------------------------------------------------

inline void wd11c00_17_device::check_interrupt()
{
	if (BIT(m_ra, 10))
	{
		m_status &= ~STATUS_DRQ;
	}

	int ra3 = BIT(m_ra, 3);

	if (m_ra3 != ra3)
	{
		m_out_ra3_cb(ra3 ? ASSERT_LINE : CLEAR_LINE);
		m_ra3 = ra3;
	}

	int irq5 = ((m_status & STATUS_IRQ) && (m_mask & MASK_IRQ)) ? ASSERT_LINE : CLEAR_LINE;

	if (m_irq5 != irq5)
	{
		m_out_irq5_cb(irq5);
		m_irq5 = irq5;
	}

	int drq3 = ((m_status & STATUS_DRQ) && (m_mask & MASK_DMA)) ? ASSERT_LINE : CLEAR_LINE;

	if (m_drq3 != drq3)
	{
		m_out_drq3_cb(drq3);
		m_drq3 = drq3;
	}

	int busy = (m_status & STATUS_BUSY) ? 0 : 1;

	if (m_busy != busy)
	{
		m_out_busy_cb(busy);
		m_busy = busy;
	}

	int req = (m_status & STATUS_REQ) ? 1 : 0;

	if (m_req != req)
	{
		m_out_req_cb(req);
		m_req = req;
	}
}


//-------------------------------------------------
//  increment_address -
//-------------------------------------------------

inline void wd11c00_17_device::increment_address()
{
	m_ra++;
	check_interrupt();
}


//-------------------------------------------------
//  read_data -
//-------------------------------------------------

inline uint8_t wd11c00_17_device::read_data()
{
	uint8_t data = 0;

	if (m_status & STATUS_BUSY)
	{
		data = m_in_ramcs_cb(m_ra & 0x7ff);

		increment_address();
	}

	return data;
}


//-------------------------------------------------
//  write_data -
//-------------------------------------------------

inline void wd11c00_17_device::write_data(uint8_t data)
{
	if (m_status & STATUS_BUSY)
	{
		m_out_ramwr_cb(m_ra & 0x7ff, data);

		increment_address();
	}
}


//-------------------------------------------------
//  software_reset -
//-------------------------------------------------

inline void wd11c00_17_device::software_reset()
{
	m_out_mr_cb(ASSERT_LINE);
	m_out_mr_cb(CLEAR_LINE);

	device_reset();
}


//-------------------------------------------------
//  select -
//-------------------------------------------------

inline void wd11c00_17_device::select()
{
	m_status = STATUS_BUSY | STATUS_C_D | STATUS_REQ;

	check_interrupt();
}



//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  wd11c00_17_device - constructor
//-------------------------------------------------

wd11c00_17_device::wd11c00_17_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, WD11C00_17, tag, owner, clock)
	, m_out_irq5_cb(*this)
	, m_out_drq3_cb(*this)
	, m_out_mr_cb(*this)
	, m_out_busy_cb(*this)
	, m_out_req_cb(*this)
	, m_out_ra3_cb(*this)
	, m_in_rd322_cb(*this)
	, m_in_ramcs_cb(*this)
	, m_out_ramwr_cb(*this)
	, m_in_cs1010_cb(*this)
	, m_out_cs1010_cb(*this)
	, m_status(0)
	, m_ra(0)
	, m_irq5(CLEAR_LINE)
	, m_drq3(CLEAR_LINE)
	, m_busy(1)
	, m_req(0)
	, m_ra3(0)
{
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void wd11c00_17_device::device_start()
{
	// resolve callbacks
	m_out_irq5_cb.resolve_safe();
	m_out_drq3_cb.resolve_safe();
	m_out_mr_cb.resolve_safe();
	m_out_busy_cb.resolve_safe();
	m_out_req_cb.resolve_safe();
	m_out_ra3_cb.resolve_safe();
	m_in_rd322_cb.resolve_safe(0);
	m_in_ramcs_cb.resolve_safe(0);
	m_out_ramwr_cb.resolve_safe();
	m_in_cs1010_cb.resolve_safe(0);
	m_out_cs1010_cb.resolve_safe();
}


//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void wd11c00_17_device::device_reset()
{
	m_status &= ~(STATUS_IRQ | STATUS_DRQ | STATUS_BUSY);
	m_mask = 0;
	m_ra = 0;

	check_interrupt();
}


//-------------------------------------------------
//  io_r -
//-------------------------------------------------

READ8_MEMBER( wd11c00_17_device::io_r )
{
	uint8_t data = 0xff;

	switch (offset)
	{
	case 0: // Read Data, Board to Host
		LOG("%s WD11C00-17 Read Data %03x:", machine().describe_context(), m_ra);
		data = read_data();
		LOG("%02x\n", data);
		break;

	case 1: // Read Board Hardware Status
		data = m_status;
		check_interrupt();
		break;

	case 2: // Read Drive Configuration Information
		data = m_in_rd322_cb(0);
		break;

	case 3: // Not Used
		break;
	}

	return data;
}


//-------------------------------------------------
//  io_w -
//-------------------------------------------------

WRITE8_MEMBER( wd11c00_17_device::io_w )
{
	switch (offset)
	{
	case 0: // Write Data, Host to Board
		LOG("%s WD11C00-17 Write Data %03x:%02x\n", machine().describe_context(), m_ra, data);
		write_data(data);
		break;

	case 1: // Board Software Reset
		LOG("%s WD11C00-17 Software Reset\n", machine().describe_context());
		software_reset();
		break;

	case 2: // Board Select
		LOG("%s WD11C00-17 Select\n", machine().describe_context());
		increment_address(); // HACK
		select();
		break;

	case 3: // Set/Reset DMA, IRQ Masks
		LOG("%s WD11C00-17 Mask IRQ %u DMA %u\n", machine().describe_context(), BIT(data, 1), BIT(data, 0));
		m_mask = data;
		check_interrupt();
		break;
	}
}


//-------------------------------------------------
//  dack_r -
//-------------------------------------------------

uint8_t wd11c00_17_device::dack_r()
{
	return read_data();
}


//-------------------------------------------------
//  dack_w -
//-------------------------------------------------

void wd11c00_17_device::dack_w(uint8_t data)
{
	write_data(data);
}


//-------------------------------------------------
//  read -
//-------------------------------------------------

READ8_MEMBER( wd11c00_17_device::read )
{
	uint8_t data = 0;

	switch (offset)
	{
	case 0x00:
		LOG("%s WD11C00-17 Read RAM %03x:", machine().describe_context(), m_ra);
		data = read_data();
		LOG("%02x\n", data);
		break;

	case 0x20:
		data = m_in_cs1010_cb(m_ra >> 8);
		break;
	}

	return data;
}


//-------------------------------------------------
//  write -
//-------------------------------------------------

WRITE8_MEMBER( wd11c00_17_device::write )
{
	switch (offset)
	{
	case 0x00:
		LOG("%s WD11C00-17 Write RAM %03x:%02x\n", machine().describe_context(), m_ra, data);
		write_data(data);
		if (m_ra > 0x400) m_ecc_not_0 = 0; // HACK
		break;

	case 0x20:
		m_out_cs1010_cb(m_ra >> 8, data);
		break;

	case 0x60:
		m_ra = (data & 0x07) << 8;
		LOG("%s WD11C00-17 RA %03x\n", machine().describe_context(), m_ra);
		check_interrupt();
		break;
	}
}


//-------------------------------------------------
//  ireq_w -
//-------------------------------------------------

WRITE_LINE_MEMBER( wd11c00_17_device::ireq_w )
{
	LOG("%s WD11C00-17 IREQ %u\n", machine().describe_context(), state);

	if (state) m_status |= STATUS_REQ; else m_status &= ~STATUS_REQ;

	if (m_status & STATUS_BUSY)
	{
		if (state)
		{
			m_status |= STATUS_IRQ | STATUS_I_O;
		}
		else
		{
			if (m_status & STATUS_I_O)
			{
				m_status &= ~(STATUS_BUSY | STATUS_I_O);
			}
		}
	}

	check_interrupt();
}


//-------------------------------------------------
//  io_w -
//-------------------------------------------------

WRITE_LINE_MEMBER( wd11c00_17_device::io_w )
{
	LOG("%s WD11C00-17 I/O %u\n", machine().describe_context(), state);

	if (state) m_status |= STATUS_I_O; else m_status &= ~STATUS_I_O;
}


//-------------------------------------------------
//  cd_w -
//-------------------------------------------------

WRITE_LINE_MEMBER( wd11c00_17_device::cd_w )
{
	LOG("%s WD11C00-17 C/D %u\n", machine().describe_context(), state);

	if (state) m_status |= STATUS_C_D; else m_status &= ~STATUS_C_D;
}


//-------------------------------------------------
//  clct_w -
//-------------------------------------------------

WRITE_LINE_MEMBER( wd11c00_17_device::clct_w )
{
	LOG("%s WD11C00-17 CLCT %u\n", machine().describe_context(), state);

	if (state)
	{
		m_ra &= 0xff00;
		check_interrupt();
	}
}


//-------------------------------------------------
//  mode_w -
//-------------------------------------------------

WRITE_LINE_MEMBER( wd11c00_17_device::mode_w )
{
	LOG("%s WD11C00-17 MODE %u\n", machine().describe_context(), state);

	m_mode = state;
	m_ecc_not_0 = state; // HACK
}


//-------------------------------------------------
//  busy_r -
//-------------------------------------------------

READ_LINE_MEMBER( wd11c00_17_device::busy_r )
{
	return (m_status & STATUS_BUSY) ? 0 : 1;
}


//-------------------------------------------------
//  ecc_not_0_r -
//-------------------------------------------------

READ_LINE_MEMBER( wd11c00_17_device::ecc_not_0_r )
{
	return m_ecc_not_0;
}
