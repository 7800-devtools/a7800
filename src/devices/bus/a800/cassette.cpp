// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/***************************************************************************

  cassette.h - Atari 8 bit cassette player(s)


Known cassette players:
- Atari XC11
- Atari XC12 (no SIO connection for an additional device)

TODO:
- Implement cassette reading
- Implement cassette writing
- Add audio support
- Add SIO connector for a next device

***************************************************************************/

#include "emu.h"
#include "cassette.h"


//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

DEFINE_DEVICE_TYPE(A8SIO_CASSETTE, a8sio_cassette_device, "a8sio_cass", "Atari 8 bit cassette")
const device_timer_id TIMER_CASSETTE_READ = 1;

MACHINE_CONFIG_MEMBER( a8sio_cassette_device::device_add_mconfig )
	MCFG_CASSETTE_ADD("cassette")
	//MCFG_CASSETTE_DEFAULT_STATE(CASSETTE_STOPPED)
	MCFG_CASSETTE_DEFAULT_STATE(CASSETTE_PLAY)
	MCFG_CASSETTE_INTERFACE("atari8bit_cass")
MACHINE_CONFIG_END

//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

a8sio_cassette_device::a8sio_cassette_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: a8sio_cassette_device(mconfig, A8SIO_CASSETTE, tag, owner, clock)
{
}

a8sio_cassette_device::a8sio_cassette_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, type, tag, owner, clock)
	, device_a8sio_card_interface(mconfig, *this)
	, m_cassette(*this, "cassette")
	, m_read_timer(nullptr)
	, m_old_cass_signal(0)
	, m_signal_count(0)
{
}

void a8sio_cassette_device::device_start()
{
	set_a8sio_device();

	save_item(NAME(m_old_cass_signal));
	save_item(NAME(m_signal_count));

	m_read_timer = timer_alloc(TIMER_CASSETTE_READ);
}

void a8sio_cassette_device::device_reset()
{
}

WRITE_LINE_MEMBER( a8sio_cassette_device::motor_w )
{
	//printf("a8sio_cassette::motor_w %d\n", state);
	if (!state)
	{
		m_cassette->change_state(CASSETTE_MOTOR_ENABLED, CASSETTE_MASK_MOTOR);
		m_read_timer->adjust(attotime::zero, 0, attotime::from_hz(44100));
	}
	else
	{
		m_cassette->change_state(CASSETTE_MOTOR_DISABLED, CASSETTE_MASK_MOTOR);
		m_read_timer->reset();
	}
}

void a8sio_cassette_device::device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr)
{
	switch (id)
	{
		case TIMER_CASSETTE_READ:
			uint8_t cass_signal = m_cassette->input() < 0 ? 0 : 1;

			if (m_signal_count < 20)
			{
				m_signal_count++;
			}

			if (cass_signal != m_old_cass_signal)
			{
				//printf("cass_signal: %d, count: %d, data: %d\n", cass_signal, m_signal_count, m_signal_count < 5 ? 1 : 0);
				// ~4 kHz -> 0
				// ~5 kHz -> 1
				m_a8sio->data_in_w((m_signal_count < 5) ? 1 : 0);
				m_signal_count = 0;
				m_old_cass_signal = cass_signal;
			}
			break;
	}
}
