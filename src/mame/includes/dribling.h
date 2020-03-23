// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/*************************************************************************

    Model Racing Dribbling hardware

*************************************************************************/

#include "machine/i8255.h"
#include "machine/watchdog.h"

class dribling_state : public driver_device
{
public:
	dribling_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_watchdog(*this, "watchdog"),
		m_ppi8255_0(*this, "ppi8255_0"),
		m_ppi8255_1(*this, "ppi8255_1"),
		m_videoram(*this, "videoram"),
		m_colorram(*this, "colorram"){ }

	/* devices */
	required_device<cpu_device> m_maincpu;
	required_device<watchdog_timer_device> m_watchdog;
	optional_device<i8255_device>  m_ppi8255_0;
	optional_device<i8255_device>  m_ppi8255_1;
	/* memory pointers */
	required_shared_ptr<uint8_t> m_videoram;
	required_shared_ptr<uint8_t> m_colorram;

	/* misc */
	uint8_t    m_abca;
	uint8_t    m_dr;
	uint8_t    m_ds;
	uint8_t    m_sh;
	uint8_t    m_input_mux;
	uint8_t    m_di;

	DECLARE_READ8_MEMBER(ioread);
	DECLARE_WRITE8_MEMBER(iowrite);
	DECLARE_WRITE8_MEMBER(dribling_colorram_w);
	DECLARE_READ8_MEMBER(dsr_r);
	DECLARE_READ8_MEMBER(input_mux0_r);
	DECLARE_WRITE8_MEMBER(misc_w);
	DECLARE_WRITE8_MEMBER(sound_w);
	DECLARE_WRITE8_MEMBER(pb_w);
	DECLARE_WRITE8_MEMBER(shr_w);
	virtual void machine_start() override;
	virtual void machine_reset() override;
	DECLARE_PALETTE_INIT(dribling);
	uint32_t screen_update_dribling(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	INTERRUPT_GEN_MEMBER(dribling_irq_gen);
};
