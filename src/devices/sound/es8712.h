// license:BSD-3-Clause
// copyright-holders:Quench
/* An interface for the ES8712 ADPCM chip */

#ifndef MAME_SOUND_ES8712_H
#define MAME_SOUND_ES8712_H

#pragma once

//**************************************************************************
//  INTERFACE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_ES8712_ADD(_tag, _clock) \
	MCFG_DEVICE_ADD(_tag, ES8712, _clock)
#define MCFG_ES8712_REPLACE(_tag, _clock) \
	MCFG_DEVICE_REPLACE(_tag, ES8712, _clock)
#define MCFG_ES8712_RESET_HANDLER(_devcb) \
	devcb = &es8712_device::set_reset_handler(*device, DEVCB_##_devcb);


//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************


// ======================> es8712_device

class es8712_device : public device_t, public device_sound_interface
{
public:
	es8712_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// static configuration
	template <class Object> static devcb_base &set_reset_handler(device_t &device, Object &&cb) { return downcast<es8712_device &>(device).m_reset_handler.set_callback(std::forward<Object>(cb)); }

	DECLARE_WRITE8_MEMBER(write);
	DECLARE_READ8_MEMBER(read);

	void play();
	void set_bank_base(int base);
	void set_frequency(int frequency);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// sound stream update overrides
	virtual void sound_stream_update(sound_stream &stream, stream_sample_t **inputs, stream_sample_t **outputs, int samples) override;

private:
	void compute_tables();
	void generate_adpcm(stream_sample_t *buffer, int samples);
	void es8712_state_save_register();

	required_region_ptr<uint8_t> m_rom;

	devcb_write_line m_reset_handler;

	uint8_t m_playing;          /* 1 if we're actively playing */

	uint32_t m_base_offset;     /* pointer to the base memory location */
	uint32_t m_sample;          /* current sample number */
	uint32_t m_count;           /* total samples to play */

	uint32_t m_signal;          /* current ADPCM signal */
	uint32_t m_step;            /* current ADPCM step */

	uint32_t m_start;           /* starting address for the next loop */
	uint32_t m_end;             /* ending address for the next loop */

	int32_t m_bank_offset;
	sound_stream *m_stream;   /* which stream are we playing on? */
};

DECLARE_DEVICE_TYPE(ES8712, es8712_device)

#endif // MAME_SOUND_ES8712_H
