// license:BSD-3-Clause
// copyright-holders:Al Kossow
#include "sound/discrete.h"

class m79amb_state : public driver_device
{
public:
	m79amb_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_videoram(*this, "videoram"),
		m_mask(*this, "mask"),
		m_discrete(*this, "discrete"),
		m_maincpu(*this, "maincpu") { }

	/* memory pointers */
	required_shared_ptr<uint8_t> m_videoram;
	required_shared_ptr<uint8_t> m_mask;

	required_device<discrete_device> m_discrete;

	/* misc */
	uint8_t m_lut_gun1[0x100];
	uint8_t m_lut_gun2[0x100];
	DECLARE_WRITE8_MEMBER(ramtek_videoram_w);
	DECLARE_READ8_MEMBER(gray5bit_controller0_r);
	DECLARE_READ8_MEMBER(gray5bit_controller1_r);
	DECLARE_WRITE8_MEMBER(m79amb_8002_w);
	DECLARE_DRIVER_INIT(m79amb);
	uint32_t screen_update_ramtek(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);
	INTERRUPT_GEN_MEMBER(m79amb_interrupt);
	DECLARE_WRITE8_MEMBER(m79amb_8000_w);
	DECLARE_WRITE8_MEMBER(m79amb_8003_w);
	required_device<cpu_device> m_maincpu;
};

/*----------- defined in audio/m79amb.c -----------*/

DISCRETE_SOUND_EXTERN( m79amb );
