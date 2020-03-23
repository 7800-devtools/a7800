// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    Atari Portfolio HPC-102 serial interface emulation

**********************************************************************/

#include "emu.h"
#include "hpc102.h"



//**************************************************************************
//  MACROS / CONSTANTS
//**************************************************************************

#define LOG 0

#define M82C50A_TAG     "u1"
#define RS232_TAG      "rs232"



//**************************************************************************
//  DEVICE DEFINITIONS
//**************************************************************************

DEFINE_DEVICE_TYPE(POFO_HPC102, pofo_hpc102_device, "pofo_hpc102", "Atari Portfolio HPC-102")


//-------------------------------------------------
//  device_add_mconfig - add device configuration
//-------------------------------------------------

MACHINE_CONFIG_MEMBER( pofo_hpc102_device::device_add_mconfig )
	MCFG_DEVICE_ADD(M82C50A_TAG, INS8250, XTAL_1_8432MHz) // should be INS8250A
	MCFG_INS8250_OUT_TX_CB(DEVWRITELINE(RS232_TAG, rs232_port_device, write_txd))
	MCFG_INS8250_OUT_DTR_CB(DEVWRITELINE(RS232_TAG, rs232_port_device, write_dtr))
	MCFG_INS8250_OUT_RTS_CB(DEVWRITELINE(RS232_TAG, rs232_port_device, write_rts))
	MCFG_INS8250_OUT_INT_CB(WRITELINE(device_portfolio_expansion_slot_interface, eint_w))

	MCFG_RS232_PORT_ADD(RS232_TAG, default_rs232_devices, nullptr)
	MCFG_RS232_RXD_HANDLER(DEVWRITELINE(M82C50A_TAG, ins8250_uart_device, rx_w))
	MCFG_RS232_DCD_HANDLER(DEVWRITELINE(M82C50A_TAG, ins8250_uart_device, dcd_w))
	MCFG_RS232_DSR_HANDLER(DEVWRITELINE(M82C50A_TAG, ins8250_uart_device, dsr_w))
	MCFG_RS232_RI_HANDLER(DEVWRITELINE(M82C50A_TAG, ins8250_uart_device, ri_w))
	MCFG_RS232_CTS_HANDLER(DEVWRITELINE(M82C50A_TAG, ins8250_uart_device, cts_w))
MACHINE_CONFIG_END


//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  pofo_hpc102_device - constructor
//-------------------------------------------------

pofo_hpc102_device::pofo_hpc102_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, POFO_HPC102, tag, owner, clock),
	device_portfolio_expansion_slot_interface(mconfig, *this),
	m_uart(*this, M82C50A_TAG)
{
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void pofo_hpc102_device::device_start()
{
}


//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void pofo_hpc102_device::device_reset()
{
	m_uart->reset();
}


//-------------------------------------------------
//  eack_r - external interrupt acknowledge
//-------------------------------------------------

uint8_t pofo_hpc102_device::eack_r()
{
	return m_vector;
}


//-------------------------------------------------
//  nrdi_r - read
//-------------------------------------------------

uint8_t pofo_hpc102_device::nrdi_r(address_space &space, offs_t offset, uint8_t data, bool iom, bool bcom, bool ncc1)
{
	if (!bcom)
	{
		if ((offset & 0x0f) == 0x0f)
		{
			data = 0x01;
		}

		if (!(offset & 0x08))
		{
			data = m_uart->ins8250_r(space, offset & 0x07);
		}
	}

	return data;
}


//-------------------------------------------------
//  nwri_w - write
//-------------------------------------------------

void pofo_hpc102_device::nwri_w(address_space &space, offs_t offset, uint8_t data, bool iom, bool bcom, bool ncc1)
{
	if (!bcom)
	{
		if ((offset & 0x0f) == 0x0f)
		{
			m_vector = data;
		}

		if (!(offset & 0x08))
		{
			m_uart->ins8250_w(space, offset & 0x07, data);
		}
	}
}
