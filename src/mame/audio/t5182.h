// license:BSD-3-Clause
// copyright-holders:Jonathan Gevaryahu
#ifndef MAME_AUDIO_T5182_H
#define MAME_AUDIO_T5182_H

#pragma once

#include "sound/ym2151.h"
#include "cpu/z80/z80.h"

class t5182_device : public device_t

{
public:
	t5182_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	enum
	{
		VECTOR_INIT,
		YM2151_ASSERT,
		YM2151_CLEAR,
		YM2151_ACK,
		CPU_ASSERT,
		CPU_CLEAR
	};

	enum
	{
		SETIRQ_CB
	};

	DECLARE_WRITE8_MEMBER(sound_irq_w );
	DECLARE_READ8_MEMBER(sharedram_semaphore_snd_r);
	DECLARE_WRITE8_MEMBER(sharedram_semaphore_main_acquire_w);
	DECLARE_WRITE8_MEMBER(sharedram_semaphore_main_release_w);
	DECLARE_WRITE8_MEMBER(sharedram_semaphore_snd_acquire_w);
	DECLARE_WRITE8_MEMBER(sharedram_semaphore_snd_release_w);
	DECLARE_READ8_MEMBER(sharedram_semaphore_main_r);
	DECLARE_READ8_MEMBER(sharedram_r);
	DECLARE_WRITE8_MEMBER(sharedram_w);
	DECLARE_WRITE_LINE_MEMBER(ym2151_irq_handler);
	DECLARE_WRITE8_MEMBER(ym2151_irq_ack_w);
	DECLARE_WRITE8_MEMBER(cpu_irq_ack_w);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;
	virtual const tiny_rom_entry *device_rom_region() const override;
	virtual ioport_constructor device_input_ports() const override;
	virtual void device_add_mconfig(machine_config &config) override;

private:
	// internal state
	required_device<cpu_device> m_ourcpu;
	required_shared_ptr<uint8_t> m_sharedram;
	int m_irqstate;
	int m_semaphore_main;
	int m_semaphore_snd;
	emu_timer *m_setirq_cb;
	TIMER_CALLBACK_MEMBER( setirq_callback );
};

DECLARE_DEVICE_TYPE(T5182, t5182_device)

#endif // MAME_AUDIO_T5182_H
