// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/**********************************************************************

    NEC uPD1771

**********************************************************************/

#ifndef MAME_SOUND_UPD1771_H
#define MAME_SOUND_UPD1771_H


#define MCFG_UPD1771_ACK_HANDLER(_devcb) \
	devcb = &upd1771c_device::set_ack_handler(*device, DEVCB_##_devcb);


/***************************************************************************
    MACROS / CONSTANTS
***************************************************************************/

class upd1771c_device : public device_t,
						public device_sound_interface
{
public:
	upd1771c_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	template <class Object> static devcb_base &set_ack_handler(device_t &device, Object &&cb) { return downcast<upd1771c_device &>(device).m_ack_handler.set_callback(std::forward<Object>(cb)); }

	DECLARE_READ8_MEMBER( read );
	DECLARE_WRITE8_MEMBER( write );
	WRITE_LINE_MEMBER( pcm_write );

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// sound stream update overrides
	virtual void sound_stream_update(sound_stream &stream, stream_sample_t **inputs, stream_sample_t **outputs, int samples) override;

private:
	static constexpr unsigned MAX_PACKET_SIZE = 0x8000;

	TIMER_CALLBACK_MEMBER(ack_callback);

	// internal state
	sound_stream *m_channel;
	devcb_write_line m_ack_handler;
	emu_timer *m_timer;

	uint8_t   m_packet[MAX_PACKET_SIZE];
	uint32_t  m_index;
	uint8_t   m_expected_bytes;

	uint8_t   m_state;//0:silence, 1 noise, 2 tone
	uint8_t   m_pc3;

	//tone
	uint8_t    m_t_timbre; //[0;  7]
	uint8_t    m_t_offset; //[0; 32]
	uint16_t   m_t_period; //[0;255]
	uint8_t    m_t_volume; //[0; 31]
	uint8_t    m_t_tpos;//timbre pos
	uint16_t   m_t_ppos;//period pos

	//noise wavetable LFSR
	uint8_t    m_nw_timbre; //[0;  7]
	uint8_t    m_nw_volume; //[0; 31]
	uint32_t   m_nw_period;
	uint32_t   m_nw_tpos;   //timbre pos
	uint32_t   m_nw_ppos;   //period pos

	//noise pulse components
	uint8_t    m_n_value[3];  //[0;1]
	uint16_t   m_n_volume[3]; //[0; 31]
	uint32_t   m_n_period[3];
	uint32_t   m_n_ppos[3];   //period pos
};

DECLARE_DEVICE_TYPE(UPD1771C, upd1771c_device)

#endif // MAME_SOUND_UPD1771_H
