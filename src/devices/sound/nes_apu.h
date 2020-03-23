// license:GPL-2.0+
// copyright-holders:Matthew Conte
/*****************************************************************************

  MAME/MESS NES APU CORE

  Based on the Nofrendo/Nosefart NES N2A03 sound emulation core written by
  Matthew Conte (matt@conte.com) and redesigned for use in MAME/MESS by
  Who Wants to Know? (wwtk@mail.com)

  This core is written with the advise and consent of Matthew Conte and is
  released under the GNU Public License.  This core is freely available for
  use in any freeware project, subject to the following terms:

  Any modifications to this code must be duly noted in the source and
  approved by Matthew Conte and myself prior to public submission.

 *****************************************************************************

   NES_APU.H

   NES APU external interface.

 *****************************************************************************/

#ifndef MAME_SOUND_NES_APU_H
#define MAME_SOUND_NES_APU_H

#pragma once


#include "nes_defs.h"

/* AN EXPLANATION
 *
 * The NES APU is actually integrated into the Nintendo processor.
 * You must supply the same number of APUs as you do processors.
 * Also make sure to correspond the memory regions to those used in the
 * processor, as each is shared.
 */

#define MCFG_NES_APU_IRQ_HANDLER(_devcb) \
	devcb = &nesapu_device::set_irq_handler(*device, DEVCB_##_devcb);

#define MCFG_NES_APU_MEM_READ_CALLBACK(_devcb) \
	devcb = &nesapu_device::set_mem_read_callback(*device, DEVCB_##_devcb);

class nesapu_device : public device_t,
						public device_sound_interface
{
public:
	nesapu_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	// static configuration helpers
	template <class Object> static devcb_base &set_irq_handler(device_t &device, Object &&cb) { return downcast<nesapu_device &>(device).m_irq_handler.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_mem_read_callback(device_t &device, Object &&cb) { return downcast<nesapu_device &>(device).m_mem_read_cb.set_callback(std::forward<Object>(cb)); }

	virtual void device_clock_changed() override;

	DECLARE_READ8_MEMBER( read );
	DECLARE_WRITE8_MEMBER( write );

protected:
	// device-level overrides
	virtual void device_start() override;

	// sound stream update overrides
	virtual void sound_stream_update(sound_stream &stream, stream_sample_t **inputs, stream_sample_t **outputs, int samples) override;

private:
	/* GLOBAL CONSTANTS */
	static constexpr unsigned  SYNCS_MAX1     = 0x20;
	static constexpr unsigned  SYNCS_MAX2     = 0x80;

	// internal state
	apu_t   m_APU;                   /* Actual APUs */
	float   m_apu_incsize;           /* Adjustment increment */
	u32     m_samps_per_sync;        /* Number of samples per vsync */
	u32     m_buffer_size;           /* Actual buffer size in bytes */
	u32     m_real_rate;             /* Actual playback rate */
	u8      m_noise_lut[apu_t::NOISE_LONG]; /* Noise sample lookup table */
	u32     m_vbl_times[0x20];       /* VBL durations in samples */
	u32     m_sync_times1[SYNCS_MAX1]; /* Samples per sync table */
	u32     m_sync_times2[SYNCS_MAX2]; /* Samples per sync table */
	sound_stream *m_stream;
	devcb_write_line m_irq_handler;
	devcb_read8 m_mem_read_cb;

	void calculate_rates();
	void create_syncs(unsigned long sps);
	s8 apu_square(apu_t::square_t *chan);
	s8 apu_triangle(apu_t::triangle_t *chan);
	s8 apu_noise(apu_t::noise_t *chan);
	s8 apu_dpcm(apu_t::dpcm_t *chan);
	inline void apu_regwrite(int address, u8 value);
	inline u8 apu_read(int address);
	inline void apu_write(int address, u8 value);
};

DECLARE_DEVICE_TYPE(NES_APU, nesapu_device)

#endif // MAME_SOUND_NES_APU_H
