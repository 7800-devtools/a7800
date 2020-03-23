// license:BSD-3-Clause
// copyright-holders:Mirko Buffoni,Aaron Giles
/***************************************************************************

    okim6295.h

    OKIM 6295 ADCPM sound chip.

***************************************************************************/

#ifndef MAME_SOUND_OKIM6295_H
#define MAME_SOUND_OKIM6295_H

#pragma once

#include "sound/okiadpcm.h"



//**************************************************************************
//  INTERFACE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_OKIM6295_ADD(tag, clock, pin7) \
		MCFG_DEVICE_ADD((tag), OKIM6295, (clock)) \
		MCFG_OKIM6295_PIN7(pin7)

#define MCFG_OKIM6295_REPLACE(tag, clock, pin7) \
		MCFG_DEVICE_REPLACE((tag), OKIM6295, (clock)) \
		MCFG_OKIM6295_PIN7(pin7)

#define MCFG_OKIM6295_PIN7(pin7) \
		okim6295_device::static_set_pin7(*device, (okim6295_device::pin7));


//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************


// ======================> okim6295_device

class okim6295_device : public device_t,
						public device_sound_interface,
						public device_rom_interface
{
public:
	enum
	{
		PIN7_LOW = 0,
		PIN7_HIGH = 1
	};

	// construction/destruction
	okim6295_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// inline configuration helpers
	static void static_set_pin7(device_t &device, int pin7);

	// runtime configuration
	void set_pin7(int pin7);

	uint8_t read_status();
	void write_command(uint8_t command);

	DECLARE_READ8_MEMBER( read );
	DECLARE_WRITE8_MEMBER( write );

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_post_load() override;
	virtual void device_clock_changed() override;

	// device_sound_interface overrides
	virtual void sound_stream_update(sound_stream &stream, stream_sample_t **inputs, stream_sample_t **outputs, int samples) override;

	// device_rom_interface overrides
	virtual void rom_bank_updated() override;

	// a single voice
	class okim_voice
	{
	public:
		okim_voice();
		void generate_adpcm(device_rom_interface &rom, stream_sample_t *buffer, int samples);

		oki_adpcm_state m_adpcm;        // current ADPCM state
		bool            m_playing;
		offs_t          m_base_offset;  // pointer to the base memory location
		uint32_t          m_sample;       // current sample number
		uint32_t          m_count;        // total samples to play
		int8_t            m_volume;       // output volume
	};

	// configuration state
	optional_memory_region  m_region;

	// internal state
	static constexpr int OKIM6295_VOICES = 4;

	okim_voice          m_voice[OKIM6295_VOICES];
	int32_t               m_command;
	sound_stream *      m_stream;
	uint8_t               m_pin7_state;

	static const uint8_t s_volume_table[16];
};


// device type definition
DECLARE_DEVICE_TYPE(OKIM6295, okim6295_device)


#endif // MAME_SOUND_OKIM6295_H
