// license:BSD-3-Clause
// copyright-holders:Mark McDougall
#include "sound/msm5205.h"
#include "video/stfight_dev.h"
#include "video/airraid_dev.h"

class stfight_state : public driver_device
{
public:
	enum
	{
		TIMER_STFIGHT_INTERRUPT_1
	};

	stfight_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_coin_mech(*this, "COIN")
		, m_maincpu(*this, "maincpu")
		, m_audiocpu(*this, "audiocpu")
		, m_mcu(*this, "mcu")
		, m_msm(*this, "msm")
		, m_main_bank(*this, "mainbank")
		, m_samples(*this, "adpcm")
		, m_decrypted_opcodes(*this, "decrypted_opcodes")
		, m_coin_state(0)
		, m_fm_data(0)
		, m_cpu_to_mcu_empty(true)
		, m_cpu_to_mcu_data(0x0f)
		, m_port_a_out(0xff)
		, m_port_c_out(0xff)
		, m_vck2(false)
		, m_adpcm_reset(true)
		, m_adpcm_data_offs(0x0000)
	{
	}

	DECLARE_WRITE_LINE_MEMBER(stfight_adpcm_int);

	DECLARE_DRIVER_INIT(stfight);
	DECLARE_DRIVER_INIT(empcity);
	DECLARE_DRIVER_INIT(cshooter);

	DECLARE_WRITE8_MEMBER(stfight_io_w);
	DECLARE_READ8_MEMBER(stfight_coin_r);
	DECLARE_WRITE8_MEMBER(stfight_coin_w);
	DECLARE_WRITE8_MEMBER(stfight_fm_w);
	DECLARE_WRITE8_MEMBER(stfight_mcu_w);

	DECLARE_WRITE8_MEMBER(stfight_bank_w);

	DECLARE_READ8_MEMBER(stfight_fm_r);

	INTERRUPT_GEN_MEMBER(stfight_vb_interrupt);

	// MCU specifics
	DECLARE_READ8_MEMBER(stfight_68705_port_b_r);
	DECLARE_WRITE8_MEMBER(stfight_68705_port_a_w);
	DECLARE_WRITE8_MEMBER(stfight_68705_port_b_w);
	DECLARE_WRITE8_MEMBER(stfight_68705_port_c_w);

protected:
	virtual void machine_start() override;
	virtual void machine_reset() override;

	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

	required_ioport                 m_coin_mech;

	required_device<cpu_device>     m_maincpu;
	required_device<cpu_device>     m_audiocpu;
	required_device<cpu_device>     m_mcu;
	required_device<msm5205_device> m_msm;

	required_memory_bank            m_main_bank;

	required_region_ptr<uint8_t>    m_samples;
	optional_shared_ptr<uint8_t>    m_decrypted_opcodes;

	uint8_t     m_coin_state;

	uint8_t     m_fm_data;

	bool        m_cpu_to_mcu_empty;
	uint8_t     m_cpu_to_mcu_data;
	uint8_t     m_port_a_out;
	uint8_t     m_port_c_out;

	bool        m_vck2;
	bool        m_adpcm_reset;
	uint16_t    m_adpcm_data_offs;

	emu_timer   *m_int1_timer;
};
