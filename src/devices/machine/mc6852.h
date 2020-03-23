// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    Motorola MC6852 Synchronous Serial Data Adapter emulation

**********************************************************************
                            _____   _____
                   Vss   1 |*    \_/     | 24  _CTS
               Rx DATA   2 |             | 23  _DCD
                Rx CLK   3 |             | 22  D0
                Tx CLK   4 |             | 21  D1
               SM/_DTR   5 |             | 20  D2
               Tx DATA   6 |   MC6852    | 19  D3
                  _IRQ   7 |             | 18  D4
                   TUF   8 |             | 17  D5
                _RESET   9 |             | 16  D6
                   _CS   9 |             | 15  D7
                    RS   9 |             | 14  E
                   Vcc  10 |_____________| 13  R/_W

**********************************************************************/

#ifndef MAME_MACHINE_MC6852_H
#define MAME_MACHINE_MC6852_H

#pragma once

#include <queue>



//**************************************************************************
//  INTERFACE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_MC6852_RX_CLOCK(_clock) \
	mc6852_device::set_rx_clock(*device, _clock);

#define MCFG_MC6852_TX_CLOCK(_clock) \
	mc6852_device::set_tx_clock(*device, _clock);

#define MCFG_MC6852_TX_DATA_CALLBACK(_write) \
	devcb = &mc6852_device::set_tx_data_wr_callback(*device, DEVCB_##_write);

#define MCFG_MC6852_IRQ_CALLBACK(_write) \
	devcb = &mc6852_device::set_irq_wr_callback(*device, DEVCB_##_write);

#define MCFG_MC6852_SM_DTR_CALLBACK(_write) \
	devcb = &mc6852_device::set_sm_dtr_wr_callback(*device, DEVCB_##_write);

#define MCFG_MC6852_TUF_CALLBACK(_write) \
	devcb = &mc6852_device::set_tuf_wr_callback(*device, DEVCB_##_write);



//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> mc6852_device

class mc6852_device :   public device_t,
						public device_serial_interface
{
public:
	// construction/destruction
	mc6852_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	static void set_rx_clock(device_t &device, int clock) { downcast<mc6852_device &>(device).m_rx_clock = clock; }
	static void set_tx_clock(device_t &device, int clock) { downcast<mc6852_device &>(device).m_tx_clock = clock; }
	template <class Object> static devcb_base &set_tx_data_wr_callback(device_t &device, Object &&cb) { return downcast<mc6852_device &>(device).m_write_tx_data.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_irq_wr_callback(device_t &device, Object &&cb) { return downcast<mc6852_device &>(device).m_write_irq.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_sm_dtr_wr_callback(device_t &device, Object &&cb) { return downcast<mc6852_device &>(device).m_write_sm_dtr.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_tuf_wr_callback(device_t &device, Object &&cb) { return downcast<mc6852_device &>(device).m_write_tuf.set_callback(std::forward<Object>(cb)); }

	DECLARE_READ8_MEMBER( read );
	DECLARE_WRITE8_MEMBER( write );

	DECLARE_WRITE_LINE_MEMBER( rx_data_w ) { device_serial_interface::rx_w(state); }
	DECLARE_WRITE_LINE_MEMBER( rx_clk_w ) { rx_clock_w(state); }
	DECLARE_WRITE_LINE_MEMBER( tx_clk_w ) { tx_clock_w(state); }
	DECLARE_WRITE_LINE_MEMBER( cts_w ) { m_cts = state; }
	DECLARE_WRITE_LINE_MEMBER( dcd_w ) { m_dcd = state; }

	DECLARE_READ_LINE_MEMBER( sm_dtr_r ) { return m_sm_dtr; }
	DECLARE_READ_LINE_MEMBER( tuf_r ) { return m_tuf; }

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// device_serial_interface overrides
	virtual void tra_callback() override;
	virtual void tra_complete() override;
	virtual void rcv_complete() override;

private:
	enum
	{
		S_IRQ = 0x80,
		S_PE = 0x40,
		S_RX_OVRN = 0x20,
		S_TUF = 0x10,
		S_CTS = 0x08,
		S_DCD = 0x04,
		S_TDRA = 0x02,
		S_RDA = 0x01
	};

	enum
	{
		C1_AC_MASK = 0xc0,
		C1_AC_C2 = 0x00,
		C1_AC_C3 = 0x40,
		C1_AC_SYNC = 0x80,
		C1_AC_TX_FIFO = 0xc0,
		C1_AC2 = 0x80,
		C1_AC1 = 0x40,
		C1_RIE = 0x20,
		C1_TIE = 0x10,
		C1_CLEAR_SYNC = 0x08,
		C1_STRIP_SYNC = 0x04,
		C1_TX_RS = 0x02,
		C1_RX_RS = 0x01
	};

	enum
	{
		C2_EIE = 0x80,
		C2_TX_SYNC = 0x40,
		C2_WS_MASK = 0x38,
		C2_WS3 = 0x20,
		C2_WS2 = 0x10,
		C2_WS1 = 0x08,
		C2_1_2_BYTE = 0x04,
		C2_PC_MASK = 0x03,
		C2_PC2 = 0x02,
		C2_PC1 = 0x01
	};

	enum
	{
		C3_CTUF = 0x08,
		C3_CTS = 0x04,
		C3_1_2_SYNC = 0x02,
		C3_E_I_SYNC = 0x01
	};

	devcb_write_line       m_write_tx_data;
	devcb_write_line       m_write_irq;
	devcb_write_line       m_write_sm_dtr;
	devcb_write_line       m_write_tuf;

	uint8_t m_status;         // status register
	uint8_t m_cr[3];          // control registers
	uint8_t m_scr;            // sync code register
	uint8_t m_tdr;            // transmit data register
	uint8_t m_tsr;            // transmit shift register
	uint8_t m_rdr;            // receive data register
	uint8_t m_rsr;            // receive shift register

	std::queue<uint8_t> m_rx_fifo;
	std::queue<uint8_t> m_tx_fifo;

	int m_rx_clock;
	int m_tx_clock;
	int m_cts;              // clear to send
	int m_dcd;              // data carrier detect
	int m_sm_dtr;           // sync match/data terminal ready
	int m_tuf;              // transmitter underflow
};


// device type definition
DECLARE_DEVICE_TYPE(MC6852, mc6852_device)

#endif // MAME_MACHINE_MC6852_H
