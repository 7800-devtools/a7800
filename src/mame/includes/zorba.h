// license:BSD-3-Clause
// copyright-holders:Robbbert, Vas Crabb
#ifndef MAME_INCLUDES_ZORBA_H
#define MAME_INCLUDES_ZORBA_H

#pragma once

#include "sound/beep.h"

#include "bus/ieee488/ieee488.h"

#include "machine/6821pia.h"
#include "machine/i8251.h"
#include "machine/wd_fdc.h"
#include "machine/z80dma.h"

#include "video/i8275.h"


class zorba_state : public driver_device
{
public:
	zorba_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_config_port(*this, "CNF")
		, m_read_bank(*this, "bankr0")
		, m_p_chargen(*this, "chargen")
		, m_maincpu(*this, "maincpu")
		, m_dma(*this, "dma")
		, m_uart0(*this, "uart0")
		, m_uart1(*this, "uart1")
		, m_uart2(*this, "uart2")
		, m_pia0(*this, "pia0")
		, m_pia1(*this, "pia1")
		, m_palette(*this, "palette")
		, m_crtc(*this, "crtc")
		, m_beep(*this, "beeper")
		, m_fdc (*this, "fdc")
		, m_floppy0(*this, "fdc:0")
		, m_floppy1(*this, "fdc:1")
		, m_ieee(*this, IEEE488_TAG)
	{
	}

public:
	DECLARE_DRIVER_INIT(zorba);
	DECLARE_MACHINE_RESET(zorba);

	// Memory banking control
	DECLARE_READ8_MEMBER(ram_r);
	DECLARE_WRITE8_MEMBER(ram_w);
	DECLARE_READ8_MEMBER(rom_r);
	DECLARE_WRITE8_MEMBER(rom_w);

	// Interrupt vectoring glue
	DECLARE_WRITE8_MEMBER(intmask_w);
	template <unsigned N> DECLARE_WRITE_LINE_MEMBER(tx_rx_rdy_w);
	template <unsigned N> DECLARE_WRITE_LINE_MEMBER(irq_w);

	// DMA controller handlers
	DECLARE_WRITE_LINE_MEMBER(busreq_w);
	DECLARE_READ8_MEMBER(memory_read_byte);
	DECLARE_WRITE8_MEMBER(memory_write_byte);
	DECLARE_READ8_MEMBER(io_read_byte);
	DECLARE_WRITE8_MEMBER(io_write_byte);

	// PIT handlers
	DECLARE_WRITE_LINE_MEMBER(br1_w);

	// PIA handlers
	DECLARE_WRITE8_MEMBER(pia0_porta_w);
	DECLARE_READ8_MEMBER(pia1_portb_r);
	DECLARE_WRITE8_MEMBER(pia1_portb_w);

	// Video
	I8275_DRAW_CHARACTER_MEMBER(zorba_update_chr);

	// Printer port glue
	DECLARE_WRITE_LINE_MEMBER(printer_fault_w);
	DECLARE_WRITE_LINE_MEMBER(printer_select_w);
	DECLARE_INPUT_CHANGED_MEMBER(printer_type);

private:
	required_ioport                     m_config_port;

	required_memory_bank                m_read_bank;
	required_region_ptr<uint8_t>        m_p_chargen;

	required_device<cpu_device>         m_maincpu;
	required_device<z80dma_device>      m_dma;
	required_device<i8251_device>       m_uart0;
	required_device<i8251_device>       m_uart1;
	required_device<i8251_device>       m_uart2;
	required_device<pia6821_device>     m_pia0;
	required_device<pia6821_device>     m_pia1;

	required_device<palette_device>     m_palette;
	required_device<i8275_device>       m_crtc;

	required_device<beep_device>        m_beep;

	required_device<fd1793_device>      m_fdc;
	required_device<floppy_connector>   m_floppy0;
	required_device<floppy_connector>   m_floppy1;

	required_device<ieee488_device>     m_ieee;

	uint8_t m_intmask;
	uint8_t m_tx_rx_rdy;
	uint8_t m_irq;

	bool    m_printer_prowriter;
	int     m_printer_fault;
	int     m_printer_select;

	uint8_t m_term_data;
};

#endif // MAME_INCLUDES_ZORBA_H
