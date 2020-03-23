// license:GPL-2.0+
// copyright-holders:Jarek Burczynski, Hiromitsu Shioya
#ifndef MAME_SOUND_MSM5232_H
#define MAME_SOUND_MSM5232_H

#pragma once


#define MCFG_MSM5232_SET_CAPACITORS(_a, _b, _c, _d, _e, _f, _g, _h) \
	msm5232_device::static_set_capacitors(*device, _a, _b, _c, _d, _e, _f, _g, _h);

#define MCFG_MSM5232_GATE_HANDLER_CB(_devcb) \
	devcb = &msm5232_device::set_gate_handler_callback(*device, DEVCB_##_devcb);


class msm5232_device : public device_t,
									public device_sound_interface
{
public:
	msm5232_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	static void static_set_capacitors(device_t &device, double cap1, double cap2, double cap3, double cap4, double cap5, double cap6, double cap7, double cap8);
	template <class Object> static devcb_base &set_gate_handler_callback(device_t &device, Object &&cb) { return downcast<msm5232_device &>(device).m_gate_handler_cb.set_callback(std::forward<Object>(cb)); }

	DECLARE_WRITE8_MEMBER( write );
	void set_clock(int clock);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_stop() override;
	virtual void device_reset() override;

	// sound stream update overrides
	virtual void sound_stream_update(sound_stream &stream, stream_sample_t **inputs, stream_sample_t **outputs, int samples) override;

private:
	struct VOICE {
		uint8_t mode;

		int     TG_count_period;
		int     TG_count;

		uint8_t   TG_cnt;     /* 7 bits binary counter (frequency output) */
		uint8_t   TG_out16;   /* bit number (of TG_cnt) for 16' output */
		uint8_t   TG_out8;    /* bit number (of TG_cnt) for  8' output */
		uint8_t   TG_out4;    /* bit number (of TG_cnt) for  4' output */
		uint8_t   TG_out2;    /* bit number (of TG_cnt) for  2' output */

		int     egvol;
		int     eg_sect;
		int     counter;
		int     eg;

		uint8_t   eg_arm;     /* attack/release mode */

		double  ar_rate;
		double  dr_rate;
		double  rr_rate;

		int     pitch;          /* current pitch data */

		int GF;
	};

	// internal state
	sound_stream *m_stream;

	VOICE   m_voi[8];

	uint32_t m_EN_out16[2]; /* enable 16' output masks for both groups (0-disabled ; ~0 -enabled) */
	uint32_t m_EN_out8[2];  /* enable 8'  output masks */
	uint32_t m_EN_out4[2];  /* enable 4'  output masks */
	uint32_t m_EN_out2[2];  /* enable 2'  output masks */

	int m_noise_cnt;
	int m_noise_step;
	int m_noise_rng;
	int m_noise_clocks;   /* number of the noise_rng (output) level changes */

	unsigned int m_UpdateStep;

	/* rate tables */
	double  m_ar_tbl[8];
	double  m_dr_tbl[16];

	uint8_t   m_control1;
	uint8_t   m_control2;

	int     m_gate;       /* current state of the GATE output */

	int     m_chip_clock;      /* chip clock in Hz */
	int     m_rate;       /* sample rate in Hz */

	double  m_external_capacity[8]; /* in Farads, eg 0.39e-6 = 0.36 uF (microFarads) */
	devcb_write_line m_gate_handler_cb;/* callback called when the GATE output pin changes state */

	void init_tables();
	void init_voice(int i);
	void gate_update();
	void init(int clock, int rate);
	void EG_voices_advance();
	void TG_group_advance(int groupidx);
	void postload();
};

DECLARE_DEVICE_TYPE(MSM5232, msm5232_device)

#endif // MAME_SOUND_MSM5232_H
