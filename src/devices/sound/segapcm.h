// license:BSD-3-Clause
// copyright-holders:Hiromitsu Shioya, Olivier Galibert
/*********************************************************/
/*    SEGA 8bit PCM                                      */
/*********************************************************/

#ifndef MAMESOUND_SEGAPCM_H
#define MAMESOUND_SEGAPCM_H

#pragma once


//**************************************************************************
//  INTERFACE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_SEGAPCM_ADD(tag, clock) \
		MCFG_DEVICE_ADD((tag), SEGAPCM, (clock))

#define MCFG_SEGAPCM_REPLACE(tag, clock) \
		MCFG_DEVICE_REPLACE((tag), SEGAPCM, (clock))

#define MCFG_SEGAPCM_BANK(bank) \
		segapcm_device::set_bank(*device, (segapcm_device::bank));

#define MCFG_SEGAPCM_BANK_MASK(bank, mask) \
		segapcm_device::set_bank(*device, (segapcm_device::bank) | (segapcm_device::mask));


//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

class segapcm_device : public device_t,
					   public device_sound_interface,
					   public device_rom_interface
{
public:
	static constexpr int BANK_256    = 11;
	static constexpr int BANK_512    = 12;
	static constexpr int BANK_12M    = 13;
	static constexpr int BANK_MASK7  = 0x70 << 16;
	static constexpr int BANK_MASKF  = 0xf0 << 16;
	static constexpr int BANK_MASKF8 = 0xf8 << 16;

	segapcm_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// static configuration
	static void set_bank(device_t &device, int bank) { downcast<segapcm_device &>(device).m_bank = bank; }

	DECLARE_WRITE8_MEMBER( sega_pcm_w );
	DECLARE_READ8_MEMBER( sega_pcm_r );

protected:
	// device-level overrides
	virtual void device_start() override;

	// sound stream update overrides
	virtual void sound_stream_update(sound_stream &stream, stream_sample_t **inputs, stream_sample_t **outputs, int samples) override;

	// device_rom_interface overrides
	virtual void rom_bank_updated() override;

private:
	std::unique_ptr<uint8_t[]> m_ram;
	uint8_t m_low[16];
	int m_bank;
	int m_bankshift;
	int m_bankmask;
	sound_stream* m_stream;
};

DECLARE_DEVICE_TYPE(SEGAPCM, segapcm_device)

#endif // MAMESOUND_SEGAPCM_H
