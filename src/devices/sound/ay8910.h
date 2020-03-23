// license:BSD-3-Clause
// copyright-holders:Couriersud
#ifndef MAME_SOUND_AY8910_H
#define MAME_SOUND_AY8910_H

#pragma once

#define ALL_8910_CHANNELS -1

/* Internal resistance at Volume level 7. */

#define AY8910_INTERNAL_RESISTANCE  (356)
#define YM2149_INTERNAL_RESISTANCE  (353)

/*
 * The following is used by all drivers not reviewed yet.
 * This will like the old behavior, output between
 * 0 and 7FFF
 */
#define AY8910_LEGACY_OUTPUT        (0x01)

/*
 * Specifying the next define will simulate the special
 * cross channel mixing if outputs are tied together.
 * The driver will only provide one stream in this case.
 */
#define AY8910_SINGLE_OUTPUT        (0x02)

/*
 * The following define is the default behavior.
 * Output level 0 is 0V and 7ffff corresponds to 5V.
 * Use this to specify that a discrete mixing stage
 * follows.
 */
#define AY8910_DISCRETE_OUTPUT      (0x04)

/*
 * The following define causes the driver to output
 * resistor values. Intended to be used for
 * netlist interfacing.
 */

#define AY8910_RESISTOR_OUTPUT      (0x08)

/*
 * This define specifies the initial state of YM2149
 * pin 26 (SEL pin). By default it is set to high,
 * compatible with AY8910.
 */
/* TODO: make it controllable while it's running (used by any hw???) */
#define YM2149_PIN26_HIGH           (0x00) /* or N/C */
#define YM2149_PIN26_LOW            (0x10)


#define MCFG_AY8910_OUTPUT_TYPE(_flag) \
	ay8910_device::set_flags(*device, _flag);

#define MCFG_AY8910_RES_LOADS(_res0, _res1, _res2) \
	ay8910_device::set_resistors_load(*device, _res0, _res1, _res2);

#define MCFG_AY8910_PORT_A_READ_CB(_devcb) \
	devcb = &ay8910_device::set_port_a_read_callback(*device, DEVCB_##_devcb);

#define MCFG_AY8910_PORT_B_READ_CB(_devcb) \
	devcb = &ay8910_device::set_port_b_read_callback(*device, DEVCB_##_devcb);

#define MCFG_AY8910_PORT_A_WRITE_CB(_devcb) \
	devcb = &ay8910_device::set_port_a_write_callback(*device, DEVCB_##_devcb);

#define MCFG_AY8910_PORT_B_WRITE_CB(_devcb) \
	devcb = &ay8910_device::set_port_b_write_callback(*device, DEVCB_##_devcb);


class ay8910_device : public device_t,
									public device_sound_interface
{
public:
	enum psg_type_t
	{
		PSG_TYPE_AY,
		PSG_TYPE_YM
	};

	// construction/destruction
	ay8910_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// static configuration helpers
	static void set_flags(device_t &device, int flags) { downcast<ay8910_device &>(device).m_flags = flags; }
	static void set_psg_type(device_t &device, psg_type_t psg_type) { downcast<ay8910_device &>(device).set_type(psg_type); }
	static void set_resistors_load(device_t &device, int res_load0, int res_load1, int res_load2) { downcast<ay8910_device &>(device).m_res_load[0] = res_load0; downcast<ay8910_device &>(device).m_res_load[1] = res_load1; downcast<ay8910_device &>(device).m_res_load[2] = res_load2; }
	template <class Object> static devcb_base &set_port_a_read_callback(device_t &device, Object &&cb) { return downcast<ay8910_device &>(device).m_port_a_read_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_port_b_read_callback(device_t &device, Object &&cb) { return downcast<ay8910_device &>(device).m_port_b_read_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_port_a_write_callback(device_t &device, Object &&cb) { return downcast<ay8910_device &>(device).m_port_a_write_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_port_b_write_callback(device_t &device, Object &&cb) { return downcast<ay8910_device &>(device).m_port_b_write_cb.set_callback(std::forward<Object>(cb)); }

	DECLARE_READ8_MEMBER( data_r );
	DECLARE_WRITE8_MEMBER( address_w );
	DECLARE_WRITE8_MEMBER( data_w );

	/* /RES */
	DECLARE_WRITE8_MEMBER( reset_w );

	/* use this when BC1 == A0; here, BC1=0 selects 'data' and BC1=1 selects 'latch address' */
	DECLARE_WRITE8_MEMBER( data_address_w );

	/* use this when BC1 == !A0; here, BC1=0 selects 'latch address' and BC1=1 selects 'data' */
	DECLARE_WRITE8_MEMBER( address_data_w );

	// bc1=a0, bc2=a1
	DECLARE_WRITE8_MEMBER(write_bc1_bc2);

	void set_volume(int channel,int volume);
	void ay_set_clock(int clock);

	struct ay_ym_param
	{
		double r_up;
		double r_down;
		int    res_count;
		double res[32];
	};

	struct mosfet_param
	{
		double m_Vth;
		double m_Vg;
		int    m_count;
		double m_Kn[32];
	};

	// internal interface for PSG component of YM device
	// FIXME: these should be private, but vector06 accesses them directly
	void ay8910_write_ym(int addr, uint8_t data);
	uint8_t ay8910_read_ym();
	void ay8910_reset_ym();

protected:
	ay8910_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner,
					uint32_t clock, psg_type_t psg_type, int streams, int ioports);

	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_clock_changed() override;

	// sound stream update overrides
	virtual void sound_stream_update(sound_stream &stream, stream_sample_t **inputs, stream_sample_t **outputs, int samples) override;

	// trampolines for callbacks from fm.cpp
	static void psg_set_clock(device_t *device, int clock) { downcast<ay8910_device *>(device)->ay_set_clock(clock); }
	static void psg_write(device_t *device, int address, int data) { downcast<ay8910_device *>(device)->ay8910_write_ym(address, data); }
	static int psg_read(device_t *device) { return downcast<ay8910_device *>(device)->ay8910_read_ym(); }
	static void psg_reset(device_t *device) { downcast<ay8910_device *>(device)->ay8910_reset_ym(); }

private:
	static constexpr unsigned NUM_CHANNELS = 3;

	// internal helpers
	void set_type(psg_type_t psg_type);
	inline uint16_t mix_3D();
	void ay8910_write_reg(int r, int v);
	void build_mixer_table();
	void ay8910_statesave();

	// internal state
	psg_type_t m_type;
	int m_streams;
	int m_ioports;
	int m_ready;
	sound_stream *m_channel;
	bool m_active;
	int32_t m_register_latch;
	uint8_t m_regs[16];
	int32_t m_last_enable;
	int32_t m_count[NUM_CHANNELS];
	uint8_t m_output[NUM_CHANNELS];
	uint8_t m_prescale_noise;
	int32_t m_count_noise;
	int32_t m_count_env;
	int8_t m_env_step;
	uint32_t m_env_volume;
	uint8_t m_hold,m_alternate,m_attack,m_holding;
	int32_t m_rng;
	uint8_t m_env_step_mask;
	/* init parameters ... */
	int m_step;
	int m_zero_is_off;
	uint8_t m_vol_enabled[NUM_CHANNELS];
	const ay_ym_param *m_par;
	const ay_ym_param *m_par_env;
	int32_t m_vol_table[NUM_CHANNELS][16];
	int32_t m_env_table[NUM_CHANNELS][32];
	std::unique_ptr<int32_t[]> m_vol3d_table;
	int m_flags;          /* Flags */
	int m_res_load[3];    /* Load on channel in ohms */
	devcb_read8 m_port_a_read_cb;
	devcb_read8 m_port_b_read_cb;
	devcb_write8 m_port_a_write_cb;
	devcb_write8 m_port_b_write_cb;
};

DECLARE_DEVICE_TYPE(AY8910, ay8910_device)

class ay8912_device : public ay8910_device
{
public:
	ay8912_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};

DECLARE_DEVICE_TYPE(AY8912, ay8912_device)

class ay8913_device : public ay8910_device
{
public:
	ay8913_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};

DECLARE_DEVICE_TYPE(AY8913, ay8913_device)

class ay8914_device : public ay8910_device
{
public:
	ay8914_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	/* AY8914 handlers needed due to different register map */
	DECLARE_READ8_MEMBER( read );
	DECLARE_WRITE8_MEMBER( write );
};

DECLARE_DEVICE_TYPE(AY8914, ay8914_device)

class ay8930_device : public ay8910_device
{
public:
	ay8930_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};

DECLARE_DEVICE_TYPE(AY8930, ay8930_device)

class ym2149_device : public ay8910_device
{
public:
	ym2149_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};

DECLARE_DEVICE_TYPE(YM2149, ym2149_device)

class ym3439_device : public ay8910_device
{
public:
	ym3439_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};

DECLARE_DEVICE_TYPE(YM3439, ym3439_device)

class ymz284_device : public ay8910_device
{
public:
	ymz284_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};

DECLARE_DEVICE_TYPE(YMZ284, ymz284_device)

class ymz294_device : public ay8910_device
{
public:
	ymz294_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};

DECLARE_DEVICE_TYPE(YMZ294, ymz294_device)


#endif // MAME_DEVICES_SOUND_AY8910_H
