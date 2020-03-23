// license:BSD-3-Clause
// copyright-holders:Ryan Holtz
/******************************************************************************


    CD-i Mono-I CDIC MCU simulation
    -------------------

    written by Ryan Holtz


*******************************************************************************

STATUS:

- Just enough for the Mono-I CD-i board to work somewhat properly.

TODO:

- Decapping and proper emulation.

*******************************************************************************/

#ifndef MAME_MACHINE_CDICDIC_H
#define MAME_MACHINE_CDICDIC_H

#pragma once

#include "cdrom.h"
#include "sound/cdda.h"


//**************************************************************************
//  INTERFACE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_CDICDIC_ADD(_tag) \
	MCFG_DEVICE_ADD(_tag, MACHINE_CDICDIC, 0)
#define MCFG_CDICDIC_REPLACE(_tag) \
	MCFG_DEVICE_REPLACE(_tag, MACHINE_CDICDIC, 0)


//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> cdicdic_device

class cdicdic_device : public device_t
{
public:
	// construction/destruction
	cdicdic_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// non-static internal members
	void sample_trigger();
	void process_delayed_command();

	DECLARE_READ16_MEMBER( regs_r );
	DECLARE_WRITE16_MEMBER( regs_w );
	DECLARE_READ16_MEMBER( ram_r );
	DECLARE_WRITE16_MEMBER( ram_w );


protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// internal callbacks
	TIMER_CALLBACK_MEMBER( audio_sample_trigger );
	TIMER_CALLBACK_MEMBER( trigger_readback_int );

private:
	// internal state
	uint16_t m_command;           // CDIC Command Register            (0x303c00)
	uint32_t m_time;              // CDIC Time Register               (0x303c02)
	uint16_t m_file;              // CDIC File Register               (0x303c06)
	uint32_t m_channel;           // CDIC Channel Register            (0x303c08)
	uint16_t m_audio_channel;     // CDIC Audio Channel Register      (0x303c0c)

	uint16_t m_audio_buffer;      // CDIC Audio Buffer Register       (0x303ff4)
	uint16_t m_x_buffer;          // CDIC X-Buffer Register           (0x303ff6)
	uint16_t m_dma_control;       // CDIC DMA Control Register        (0x303ff8)
	uint16_t m_z_buffer;          // CDIC Z-Buffer Register           (0x303ffa)
	uint16_t m_interrupt_vector;  // CDIC Interrupt Vector Register   (0x303ffc)
	uint16_t m_data_buffer;       // CDIC Data Buffer Register        (0x303ffe)

	emu_timer *m_interrupt_timer;
	cdrom_file *m_cd;

	emu_timer *m_audio_sample_timer;
	int32_t m_audio_sample_freq;
	int32_t m_audio_sample_size;

	uint16_t m_decode_addr;
	uint8_t m_decode_delay;
	attotime m_decode_period;

	int m_xa_last[4];
	std::unique_ptr<uint16_t[]> m_ram;

	// static internal members
	static void decode_xa_mono(int32_t *cdic_xa_last, const uint8_t *xa, int16_t *dp);
	static void decode_xa_mono8(int32_t *cdic_xa_last, const uint8_t *xa, int16_t *dp);
	static void decode_xa_stereo(int32_t *cdic_xa_last, const uint8_t *xa, int16_t *dp);
	static void decode_xa_stereo8(int32_t *cdic_xa_last, const uint8_t *xa, int16_t *dp);

	static const int32_t s_cdic_adpcm_filter_coef[5][2];

	// non-static internal members
	uint32_t increment_cdda_frame_bcd(uint32_t bcd);
	uint32_t increment_cdda_sector_bcd(uint32_t bcd);
	void decode_audio_sector(const uint8_t *xa, int32_t triggered);
};

// device type definition
DECLARE_DEVICE_TYPE(MACHINE_CDICDIC, cdicdic_device)

#endif // MAME_MACHINE_CDICDIC_H
