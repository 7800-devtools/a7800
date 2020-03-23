// license:BSD-3-Clause
// copyright-holders:Barry Rodewald
/*
 * wpc_dot.h
 *
 *  Created on: 18/10/2013
 *      Author: bsr
 */

#ifndef WPC_DOT_H_
#define WPC_DOT_H_

#include "cpu/m6809/m6809.h"
#include "audio/wpcsnd.h"
#include "audio/dcs.h"
#include "machine/wpc.h"
#include "rendlay.h"

class wpc_dot_state : public driver_device
{
public:
	wpc_dot_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
			m_maincpu(*this, "maincpu"),
			m_wpcsnd(*this,"wpcsnd"),
			m_wpc(*this,"wpc"),
			m_cpubank(*this, "cpubank"),
			m_fixedbank(*this, "fixedbank"),
			m_dmdbank1(*this, "dmdbank1"),
			m_dmdbank2(*this, "dmdbank2"),
			m_dmdbank3(*this, "dmdbank3"),
			m_dmdbank4(*this, "dmdbank4"),
			m_dmdbank5(*this, "dmdbank5"),
			m_dmdbank6(*this, "dmdbank6")
	{ }

protected:

	// devices
	required_device<cpu_device> m_maincpu;
	optional_device<wpcsnd_device> m_wpcsnd;
	required_device<wpc_device> m_wpc;
	required_memory_bank m_cpubank;
	required_memory_bank m_fixedbank;
	required_memory_bank m_dmdbank1;
	required_memory_bank m_dmdbank2;
	required_memory_bank m_dmdbank3;
	required_memory_bank m_dmdbank4;
	required_memory_bank m_dmdbank5;
	required_memory_bank m_dmdbank6;

	// driver_device overrides
	virtual void machine_reset() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;
	static const device_timer_id TIMER_VBLANK = 0;
	static const device_timer_id TIMER_IRQ = 1;
public:
	DECLARE_DRIVER_INIT(wpc_dot);
	DECLARE_READ8_MEMBER(ram_r);
	DECLARE_WRITE8_MEMBER(ram_w);
	DECLARE_WRITE_LINE_MEMBER(wpcsnd_reply_w);
	DECLARE_WRITE_LINE_MEMBER(wpc_irq_w);
	DECLARE_WRITE_LINE_MEMBER(wpc_firq_w);
	DECLARE_READ8_MEMBER(wpc_sound_ctrl_r);
	DECLARE_WRITE8_MEMBER(wpc_sound_ctrl_w);
	DECLARE_READ8_MEMBER(wpc_sound_data_r);
	DECLARE_WRITE8_MEMBER(wpc_sound_data_w);
	DECLARE_WRITE8_MEMBER(wpc_rombank_w);
	DECLARE_WRITE8_MEMBER(wpc_dmdbank_w);

	uint32_t screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);

private:
	uint16_t m_vblank_count;
	uint32_t m_irq_count;
	uint8_t m_bankmask;
	uint8_t m_ram[0x3000];
	uint8_t m_dmdram[0x2000];
	emu_timer* m_vblank_timer;
	emu_timer* m_irq_timer;
};

#endif /* WPC_DOT_H_ */
