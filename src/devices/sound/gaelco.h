// license:BSD-3-Clause
// copyright-holders:Manuel Abadia
#ifndef MAME_SOUND_GAELCO_H
#define MAME_SOUND_GAELCO_H

#pragma once


//**************************************************************************
//  INTERFACE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_GAELCO_SND_DATA(_tag) \
	gaelco_gae1_device::set_snd_data_tag(*device, "^" _tag);

#define MCFG_GAELCO_BANKS(_offs1, _offs2, _offs3, _offs4) \
	gaelco_gae1_device::set_bank_offsets(*device, _offs1, _offs2, _offs3, _offs4);


//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************


// ======================> gaelco_gae1_device

class gaelco_gae1_device : public device_t,
							public device_sound_interface
{
public:
	gaelco_gae1_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	static void set_snd_data_tag(device_t &device, const char *tag) { downcast<gaelco_gae1_device &>(device).m_snd_data.set_tag(tag); }
	static void set_bank_offsets(device_t &device, int offs1, int offs2, int offs3, int offs4)
	{
		gaelco_gae1_device &dev = downcast<gaelco_gae1_device &>(device);
		dev.m_banks[0] = offs1;
		dev.m_banks[1] = offs2;
		dev.m_banks[2] = offs3;
		dev.m_banks[3] = offs4;
	}

	DECLARE_WRITE16_MEMBER( gaelcosnd_w );
	DECLARE_READ16_MEMBER( gaelcosnd_r );

protected:
	gaelco_gae1_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override;
	virtual void device_stop() override;

	// sound stream update overrides
	virtual void sound_stream_update(sound_stream &stream, stream_sample_t **inputs, stream_sample_t **outputs, int samples) override;

private:
	static constexpr int NUM_CHANNELS   = 0x07;
	static constexpr int VOLUME_LEVELS  = 0x10;

	struct sound_channel
	{
		int active;         // is it playing?
		int loop;           // = 0 no looping, = 1 looping
		int chunkNum;       // current chunk if looping
	};

	sound_stream *m_stream;                     /* our stream */
	required_region_ptr<uint8_t> m_snd_data;      /* PCM data */
	int m_banks[4];                             /* start of each ROM bank */
	sound_channel m_channel[NUM_CHANNELS];      /* 7 stereo channels */

	uint16_t m_sndregs[0x38];

	// Table for converting from 8 to 16 bits with volume control
	int16_t m_volume_table[VOLUME_LEVELS][256];
};

DECLARE_DEVICE_TYPE(GAELCO_GAE1, gaelco_gae1_device)



// ======================> gaelco_cg1v_device

class gaelco_cg1v_device : public gaelco_gae1_device
{
public:
	gaelco_cg1v_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};

DECLARE_DEVICE_TYPE(GAELCO_CG1V, gaelco_cg1v_device)


#endif // MAME_SOUND_GAELCO_H
