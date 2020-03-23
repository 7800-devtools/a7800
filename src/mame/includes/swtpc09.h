// license:BSD-3-Clause
// copyright-holders:Robert Justice
/***************************************************************************
    swtpc09 include file
    Robert Justice ,2009-2014

****************************************************************************/

#ifndef MAME_INCLUDES_SWTPC09_H
#define MAME_INCLUDES_SWTPC09_H

#pragma once

#include "cpu/m6809/m6809.h"
#include "video/generic.h"
#include "machine/wd_fdc.h"
#include "imagedev/flopdrv.h"
#include "machine/6840ptm.h"
#include "machine/6821pia.h"
#include "machine/6850acia.h"
#include "machine/6522via.h"
#include "machine/terminal.h"
#include "imagedev/harddriv.h"
#include "machine/idectrl.h"
#include "machine/clock.h"
#include "bus/rs232/rs232.h"



class swtpc09_state : public driver_device
{
public:
	swtpc09_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_pia(*this, "pia")
		, m_ptm(*this, "ptm")
		, m_acia(*this, "acia")
		, m_fdc(*this, "fdc")
		, m_floppy0(*this, "fdc:0")
		, m_floppy1(*this, "fdc:1")
		, m_floppy2(*this, "fdc:2")
		, m_floppy3(*this, "fdc:3")
		, m_via(*this, "via")
		, m_piaide(*this, "piaide")
		, m_harddisk(*this, "harddisk")
		, m_ide(*this, "ide")
	{ }

	DECLARE_FLOPPY_FORMATS(floppy_formats);

	DECLARE_READ8_MEMBER(pia0_a_r);
	DECLARE_READ8_MEMBER(pia0_ca1_r);
	DECLARE_WRITE_LINE_MEMBER( pia0_irq_a );

	DECLARE_WRITE_LINE_MEMBER( ptm_o1_callback );
	DECLARE_WRITE_LINE_MEMBER( ptm_o3_callback );
	DECLARE_WRITE_LINE_MEMBER( ptm_irq );

	DECLARE_WRITE_LINE_MEMBER( acia_interrupt );
	DECLARE_WRITE_LINE_MEMBER( write_acia_clock );

	DECLARE_WRITE_LINE_MEMBER( fdc_intrq_w );
	DECLARE_WRITE_LINE_MEMBER( fdc_drq_w );

	DECLARE_READ8_MEMBER( dmf3_via_read_porta );
	DECLARE_READ8_MEMBER( dmf3_via_read_portb );
	DECLARE_WRITE8_MEMBER( dmf3_via_write_porta );
	DECLARE_WRITE_LINE_MEMBER( dmf3_via_irq );

	DECLARE_READ8_MEMBER(piaide_a_r);
	DECLARE_READ8_MEMBER(piaide_b_r);
	DECLARE_WRITE8_MEMBER(piaide_a_w);
	DECLARE_WRITE8_MEMBER(piaide_b_w);

	DECLARE_READ8_MEMBER ( dmf2_dma_address_reg_r );
	DECLARE_WRITE8_MEMBER ( dmf2_dma_address_reg_w );
	DECLARE_READ8_MEMBER ( dmf2_control_reg_r );
	DECLARE_WRITE8_MEMBER ( dmf2_control_reg_w );

	DECLARE_READ8_MEMBER ( dmf3_dma_address_reg_r );
	DECLARE_WRITE8_MEMBER ( dmf3_dma_address_reg_w );
	DECLARE_READ8_MEMBER ( dmf3_control_reg_r );
	DECLARE_WRITE8_MEMBER ( dmf3_control_reg_w );

	DECLARE_WRITE8_MEMBER ( dc4_control_reg_w );

	DECLARE_WRITE8_MEMBER(dat_w);

	DECLARE_DRIVER_INIT( swtpc09 );
	DECLARE_DRIVER_INIT( swtpc09i );
	DECLARE_DRIVER_INIT( swtpc09u );
	DECLARE_DRIVER_INIT( swtpc09d3 );

	DECLARE_READ8_MEMBER ( m6844_r );
	DECLARE_WRITE8_MEMBER ( m6844_w );

protected:
	void swtpc09_fdc_dma_transfer();
	void swtpc09_irq_handler(uint8_t peripheral, uint8_t state);

	required_device<cpu_device> m_maincpu;
	required_device<pia6821_device> m_pia;
	required_device<ptm6840_device> m_ptm;
	required_device<acia6850_device> m_acia;
	required_device<fd1793_device> m_fdc;
	required_device<floppy_connector> m_floppy0;
	required_device<floppy_connector> m_floppy1;
	required_device<floppy_connector> m_floppy2;
	required_device<floppy_connector> m_floppy3;
	optional_device<via6522_device> m_via;
	optional_device<pia6821_device> m_piaide;
	optional_device<device_t> m_harddisk;
	optional_device<ide_controller_device> m_ide;

	uint8_t m_term_data;               // terminal keyboard value
	uint8_t m_pia_counter;             // this is the counter on pia porta
	uint8_t m_fdc_dma_address_reg;     // dmf2 or dmf3 dma extended address reg
	uint8_t m_system_type;             // flag to indicate hw and rom combination
	uint8_t m_fdc_status;              // for floppy controller
	uint8_t m_via_ca1_input;           // dmf3 fdc interrupt is connected here
	uint8_t m_dmf3_via_porta;
	uint8_t m_piaide_porta;
	uint8_t m_piaide_portb;
	uint8_t m_active_interrupt;
	uint8_t m_interrupt;

	// TODO: move this in proper device

	/* channel_data structure holds info about each 6844 DMA channel */
	struct m6844_channel_data
	{
		int active;
		int address;
		int counter;
		uint8_t control;
		int start_address;
		int start_counter;
	};

	/* 6844 description */
	m6844_channel_data m_m6844_channel[4];
	uint8_t m_m6844_priority;
	uint8_t m_m6844_interrupt;
	uint8_t m_m6844_chain;
};

#endif // MAME_INCLUDES_SWTPC09_H
