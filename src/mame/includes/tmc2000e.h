// license:BSD-3-Clause
// copyright-holders:Curt Coder
#pragma once

#ifndef __TMC2000E__
#define __TMC2000E__


#include "cpu/cosmac/cosmac.h"
#include "imagedev/cassette.h"
#include "machine/ram.h"
#include "machine/rescap.h"
#include "sound/cdp1864.h"

#define SCREEN_TAG      "screen"
#define CDP1802_TAG     "cdp1802"
#define CDP1864_TAG     "cdp1864"

#define TMC2000E_COLORRAM_SIZE 0x100 // ???

class tmc2000e_state : public driver_device
{
public:
	tmc2000e_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
			m_maincpu(*this, CDP1802_TAG),
			m_cti(*this, CDP1864_TAG),
			m_cassette(*this, "cassette"),
			m_colorram(*this, "colorram"),
			m_key_row(*this, {"Y0", "Y1", "Y2", "Y3", "Y4", "Y5", "Y6", "Y7"}),
			m_run(*this, "RUN")
	{ }

	required_device<cpu_device> m_maincpu;
	required_device<cdp1864_device> m_cti;
	required_device<cassette_image_device> m_cassette;
	required_shared_ptr<uint8_t> m_colorram;
	required_ioport_array<8> m_key_row;
	required_ioport m_run;

	virtual void machine_start() override;
	virtual void machine_reset() override;

	DECLARE_READ8_MEMBER( vismac_r );
	DECLARE_WRITE8_MEMBER( vismac_w );
	DECLARE_READ8_MEMBER( floppy_r );
	DECLARE_WRITE8_MEMBER( floppy_w );
	DECLARE_READ8_MEMBER( ascii_keyboard_r );
	DECLARE_READ8_MEMBER( io_r );
	DECLARE_WRITE8_MEMBER( io_w );
	DECLARE_WRITE8_MEMBER( io_select_w );
	DECLARE_WRITE8_MEMBER( keyboard_latch_w );
	DECLARE_READ_LINE_MEMBER( rdata_r );
	DECLARE_READ_LINE_MEMBER( bdata_r );
	DECLARE_READ_LINE_MEMBER( gdata_r );
	DECLARE_READ_LINE_MEMBER( clear_r );
	DECLARE_READ_LINE_MEMBER( ef2_r );
	DECLARE_READ_LINE_MEMBER( ef3_r );
	DECLARE_WRITE_LINE_MEMBER( q_w );
	DECLARE_WRITE8_MEMBER( dma_w );

	/* video state */
	int m_cdp1864_efx;      /* EFx */
	uint8_t m_color;

	/* keyboard state */
	int m_keylatch;         /* key latch */
	int m_reset;            /* reset activated */
};

#endif
