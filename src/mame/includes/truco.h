// license:BSD-3-Clause
// copyright-holders:Ernesto Corvi, Roberto Fresca

#include "machine/watchdog.h"
#include "sound/dac.h"

class truco_state : public driver_device
{
public:
	truco_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_watchdog(*this, "watchdog"),
		m_dac(*this, "dac"),
		m_videoram(*this, "videoram"),
		m_battery_ram(*this, "battery_ram") { }

	required_device<cpu_device> m_maincpu;
	required_device<watchdog_timer_device> m_watchdog;
	required_device<dac_bit_interface> m_dac;

	required_shared_ptr<uint8_t> m_videoram;
	required_shared_ptr<uint8_t> m_battery_ram;

	int m_trigger;

	DECLARE_WRITE8_MEMBER(porta_w);
	DECLARE_WRITE_LINE_MEMBER(pia_ca2_w);
	DECLARE_WRITE8_MEMBER(portb_w);
	DECLARE_WRITE_LINE_MEMBER(pia_irqa_w);
	DECLARE_WRITE_LINE_MEMBER(pia_irqb_w);

	virtual void machine_start() override;
	virtual void machine_reset() override;
	DECLARE_PALETTE_INIT(truco);

	uint32_t screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);

	INTERRUPT_GEN_MEMBER(interrupt);
};
