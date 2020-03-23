// license:BSD-3-Clause
// copyright-holders:Tomasz Slanina

#include "machine/st0016.h"
#include "screen.h"

class st0016_state : public driver_device
{
public:
	st0016_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_maincpu(*this,"maincpu"),
		m_subcpu(*this, "sub"),
		m_screen(*this, "screen")
	{ }

	int mux_port;
	// uint32_t m_st0016_rom_bank;

	optional_device<st0016_cpu_device> m_maincpu;
	DECLARE_READ8_MEMBER(mux_r);
	DECLARE_WRITE8_MEMBER(mux_select_w);
	DECLARE_READ32_MEMBER(latch32_r);
	DECLARE_WRITE32_MEMBER(latch32_w);
	DECLARE_READ8_MEMBER(latch8_r);
	DECLARE_WRITE8_MEMBER(latch8_w);

	DECLARE_WRITE8_MEMBER(st0016_rom_bank_w);
	DECLARE_DRIVER_INIT(nratechu);
	DECLARE_DRIVER_INIT(mayjinsn);
	DECLARE_DRIVER_INIT(mayjisn2);
	DECLARE_DRIVER_INIT(renju);
	virtual void machine_start() override;
	DECLARE_VIDEO_START(st0016);
	uint32_t screen_update_st0016(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	TIMER_DEVICE_CALLBACK_MEMBER(st0016_int);
	optional_device<cpu_device> m_subcpu;
	required_device<screen_device> m_screen;
};
