// license:BSD-3-Clause
// copyright-holders:Joseph Zbiciak,Tim Lindner
/**********************************************************************

    SP0256 Narrator Speech Processor emulation

**********************************************************************
                            _____   _____
                   Vss   1 |*    \_/     | 28  OSC 2
                _RESET   2 |             | 27  OSC 1
           ROM DISABLE   3 |             | 26  ROM CLOCK
                    C1   4 |             | 25  _SBY RESET
                    C2   5 |             | 24  DIGITAL OUT
                    C3   6 |             | 23  Vdi
                   Vdd   7 |    SP0256   | 22  TEST
                   SBY   8 |             | 21  SER IN
                  _LRQ   9 |             | 20  _ALD
                    A8  10 |             | 19  SE
                    A7  11 |             | 18  A1
               SER OUT  12 |             | 17  A2
                    A6  13 |             | 16  A3
                    A5  14 |_____________| 15  A4

**********************************************************************/

/*
   GI SP0256 Narrator Speech Processor

   By Joe Zbiciak. Ported to MESS by tim lindner.

*/

#ifndef MAME_SOUND_SP0256_H
#define MAME_SOUND_SP0256_H

#pragma once

#define MCFG_SP0256_DATA_REQUEST_CB(_devcb) \
	devcb = &sp0256_device::set_data_request_callback(*device, DEVCB_##_devcb);

#define MCFG_SP0256_STANDBY_CB(_devcb) \
	devcb = &sp0256_device::set_standby_callback(*device, DEVCB_##_devcb);


class sp0256_device : public device_t,
						public device_sound_interface
{
public:
	sp0256_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	template <class Object> static devcb_base &set_data_request_callback(device_t &device, Object &&cb) { return downcast<sp0256_device &>(device).m_drq_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_standby_callback(device_t &device, Object &&cb) { return downcast<sp0256_device &>(device).m_sby_cb.set_callback(std::forward<Object>(cb)); }

	DECLARE_WRITE8_MEMBER(ald_w);
	DECLARE_READ_LINE_MEMBER(lrq_r);
	DECLARE_READ_LINE_MEMBER(sby_r);
	DECLARE_READ16_MEMBER(spb640_r);
	DECLARE_WRITE16_MEMBER(spb640_w);

	TIMER_CALLBACK_MEMBER(set_lrq_timer_proc);
	void set_clock(int clock);
	void bitrevbuff(uint8_t *buffer, unsigned int start, unsigned int length);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// sound stream update overrides
	virtual void sound_stream_update(sound_stream &stream, stream_sample_t **inputs, stream_sample_t **outputs, int samples) override;

private:
	struct lpc12_t
	{
		int update(int num_samp, int16_t *out, uint32_t *optr);
		void regdec();

		int     rpt, cnt;       // Repeat counter, Period down-counter.
		uint32_t  per, rng;       // Period, Amplitude, Random Number Generator
		int     amp;
		int16_t   f_coef[6];      // F0 through F5.
		int16_t   b_coef[6];      // B0 through B5.
		int16_t   z_data[6][2];   // Time-delay data for the filter stages.
		uint8_t   r[16];          // The encoded register set.
		int     interp;

	private:
		static int16_t limit(int16_t s);
	};

	uint32_t getb(int len);
	void micro();

	void SET_SBY(int line_state)
	{
		if (m_sby_line != line_state)
		{
			m_sby_line = line_state;
			m_sby_cb(m_sby_line);
		}
	}

	required_region_ptr<uint8_t> m_rom; // 64K ROM.
	sound_stream  *m_stream;          // MAME core sound stream
	devcb_write_line m_drq_cb;       // Data request callback
	devcb_write_line m_sby_cb;       // Standby callback

	int            m_sby_line;        // Standby line state
	int            m_cur_len;         // Fullness of current sound buffer.

	int            m_silent;          // Flag: SP0256 is silent.

	std::unique_ptr<int16_t[]>    m_scratch;         // Scratch buffer for audio.
	uint32_t         m_sc_head;         // Head pointer into scratch circular buf
	uint32_t         m_sc_tail;         // Tail pointer into scratch circular buf

	lpc12_t m_filt;                   // 12-pole filter
	int            m_lrq;             // Load ReQuest.  == 0 if we can accept a load
	int            m_ald;             // Address LoaD.  < 0 if no command pending.
	int            m_pc;              // Microcontroller's PC value.
	int            m_stack;           // Microcontroller's PC stack.
	int            m_fifo_sel;        // True when executing from FIFO.
	int            m_halted;          // True when CPU is halted.
	uint32_t         m_mode;            // Mode register.
	uint32_t         m_page;            // Page set by SETPAGE

	uint32_t         m_fifo_head;       // FIFO head pointer (where new data goes).
	uint32_t         m_fifo_tail;       // FIFO tail pointer (where data comes from).
	uint32_t         m_fifo_bitp;       // FIFO bit-pointer (for partial decles).
	uint16_t         m_fifo[64];        // The 64-decle FIFO.

	emu_timer *m_lrq_timer;
};

DECLARE_DEVICE_TYPE(SP0256, sp0256_device)


#endif // MAME_SOUND_SP0256_H
