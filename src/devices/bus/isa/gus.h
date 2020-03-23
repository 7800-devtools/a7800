// license:BSD-3-Clause
// copyright-holders:Barry Rodewald
/*
 *  Gravis Ultrasound ISA card
 *
 *  Started: 28/01/2012
 *
 *  I/O port map (info from the Gravis Ultrasound SDK documentation):
 *  Base port is 0x2X0 - where X is defined by a jumper
 *
 *  MIDI:
 *  0x3X0 - MIDI Control (read), MIDI Status (write)
 *  0x3X1 - MIDI Transmit (write), MIDI Receive (read)
 *  MIDI operates identically to a 6850 UART
 *
 *  Joystick:
 *  0x201 - Joystick trigger timer (write), Joystick data (read)
 *
 *  GF1 Synthesiser:
 *  0x3X2 - Page register (voice select)
 *  0x3X3 - Global Register select
 *  0x3X4 - Global Data (low byte)
 *  0x3X5 - Global Data (high byte)
 *  0x2X6 - IRQ status register (read only, active high)
 *  0x2X8 - Timer control register
 *  0x2X9 - Timer data
 *  0x3X7 - DRAM data (can be via DMA also)
 *
 *  Board:
 *  0x2X0: Mix control register (write only)
 *  0x2XB: IRQ/DMA control register (write only) - dependant on mix control bit 6
 *  0x2XF: Register controls (board rev 3.4+ only)
 *  0x7X6: Board version (read only, board rev 3.7+ only)
 *
 *  Mixer Control:
 *  0x7X6: Control port (write only)
 *  0x3X6: Data port (write only)
 */

#ifndef MAME_BUS_ISA_GUS_H
#define MAME_BUS_ISA_GUS_H

#pragma once

#include "isa.h"
#include "machine/6850acia.h"

#define MCFG_GF1_TXIRQ_HANDLER(_devcb) \
	devcb = &gf1_device::set_txirq_handler(*device, DEVCB_##_devcb);

#define MCFG_GF1_RXIRQ_HANDLER(_devcb) \
	devcb = &gf1_device::set_rxirq_handler(*device, DEVCB_##_devcb);

#define MCFG_GF1_WAVE_IRQ_HANDLER(_devcb) \
	devcb = &gf1_device::set_wave_irq_handler(*device, DEVCB_##_devcb);

#define MCFG_GF1_RAMP_IRQ_HANDLER(_devcb) \
	devcb = &gf1_device::set_ramp_irq_handler(*device, DEVCB_##_devcb);

#define MCFG_GF1_TIMER1_IRQ_HANDLER(_devcb) \
	devcb = &gf1_device::set_timer1_irq_handler(*device, DEVCB_##_devcb);

#define MCFG_GF1_TIMER2_IRQ_HANDLER(_devcb) \
	devcb = &gf1_device::set_timer2_irq_handler(*device, DEVCB_##_devcb);

#define MCFG_GF1_SB_IRQ_HANDLER(_devcb) \
	devcb = &gf1_device::set_sb_irq_handler(*device, DEVCB_##_devcb);

#define MCFG_GF1_DMA_IRQ_HANDLER(_devcb) \
	devcb = &gf1_device::set_dma_irq_handler(*device, DEVCB_##_devcb);

#define MCFG_GF1_DRQ1_HANDLER(_devcb) \
	devcb = &gf1_device::set_drq1_handler(*device, DEVCB_##_devcb);

#define MCFG_GF1_DRQ2_HANDLER(_devcb) \
	devcb = &gf1_device::set_drq2_handler(*device, DEVCB_##_devcb);

#define MCFG_GF1_NMI_HANDLER(_devcb) \
	devcb = &gf1_device::set_nmi_handler(*device, DEVCB_##_devcb);

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> gf1_device

#define GF1_CLOCK 9878400

class gf1_device :
	public acia6850_device,
	public device_sound_interface
{
public:
	struct gus_voice
	{
		uint8_t voice_ctrl;
		uint16_t freq_ctrl;
		uint32_t start_addr;
		uint32_t end_addr;
		uint8_t vol_ramp_rate;
		uint8_t vol_ramp_start;
		uint8_t vol_ramp_end;
		uint16_t current_vol;
		uint32_t current_addr;
		uint8_t pan_position;
		uint8_t vol_ramp_ctrl;
		uint32_t vol_count;
		bool rollover;
		int16_t sample;  // current sample data
	};

	// construction/destruction
	gf1_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	template <class Object> static devcb_base &set_txirq_handler(device_t &device, Object &&cb) { return downcast<gf1_device &>(device).m_txirq_handler.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_rxirq_handler(device_t &device, Object &&cb) { return downcast<gf1_device &>(device).m_rxirq_handler.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_wave_irq_handler(device_t &device, Object &&cb) { return downcast<gf1_device &>(device).m_wave_irq_handler.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_ramp_irq_handler(device_t &device, Object &&cb) { return downcast<gf1_device &>(device).m_ramp_irq_handler.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_timer1_irq_handler(device_t &device, Object &&cb) { return downcast<gf1_device &>(device).m_timer1_irq_handler.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_timer2_irq_handler(device_t &device, Object &&cb) { return downcast<gf1_device &>(device).m_timer2_irq_handler.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_sb_irq_handler(device_t &device, Object &&cb) { return downcast<gf1_device &>(device).m_sb_irq_handler.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_dma_irq_handler(device_t &device, Object &&cb) { return downcast<gf1_device &>(device).m_dma_irq_handler.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_drq1_handler(device_t &device, Object &&cb) { return downcast<gf1_device &>(device).m_drq1_handler.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_drq2_handler(device_t &device, Object &&cb) { return downcast<gf1_device &>(device).m_drq2_handler.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_nmi_handler(device_t &device, Object &&cb) { return downcast<gf1_device &>(device).m_nmi_handler.set_callback(std::forward<Object>(cb)); }

	// current IRQ/DMA channel getters
	uint8_t gf1_irq() { if(m_gf1_irq != 0) return m_gf1_irq; else return m_midi_irq; }  // workaround for win95 loading dumb values
	uint8_t midi_irq() { if(m_irq_combine == 0) return m_midi_irq; else return m_gf1_irq; }
	uint8_t dma_channel1() { return m_dma_channel1; }
	uint8_t dma_channel2() { if(m_dma_combine == 0) return m_dma_channel2; else return m_dma_channel1; }

	DECLARE_READ8_MEMBER(global_reg_select_r);
	DECLARE_WRITE8_MEMBER(global_reg_select_w);
	DECLARE_READ8_MEMBER(global_reg_data_r);
	DECLARE_WRITE8_MEMBER(global_reg_data_w);
	DECLARE_READ8_MEMBER(dram_r);
	DECLARE_WRITE8_MEMBER(dram_w);
	DECLARE_READ8_MEMBER(adlib_r);
	DECLARE_WRITE8_MEMBER(adlib_w);
	DECLARE_READ8_MEMBER(adlib_cmd_r);
	DECLARE_WRITE8_MEMBER(adlib_cmd_w);
	DECLARE_READ8_MEMBER(mix_ctrl_r);
	DECLARE_WRITE8_MEMBER(mix_ctrl_w);
	DECLARE_READ8_MEMBER(stat_r);
	DECLARE_WRITE8_MEMBER(stat_w);
	DECLARE_READ8_MEMBER(sb_r);
	DECLARE_WRITE8_MEMBER(sb_w);
	DECLARE_WRITE8_MEMBER(sb2x6_w);

	// DMA signals
	uint8_t dack_r(int line);
	void dack_w(int line,uint8_t data);
	void eop_w(int state);

	// optional information overrides
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;
	virtual void sound_stream_update(sound_stream &stream, stream_sample_t **inputs, stream_sample_t **outputs, int samples) override;

protected:
	// voice-specific registers
	gus_voice m_voice[32];

	// global registers (not voice-specific)
	uint8_t m_dma_dram_ctrl;
	uint16_t m_dma_start_addr;
	uint32_t m_dram_addr;
	uint8_t m_timer_ctrl;
	uint8_t m_timer1_count;
	uint8_t m_timer2_count;
	uint8_t m_timer1_value;
	uint8_t m_timer2_value;
	uint16_t m_sampling_freq;
	uint8_t m_sampling_ctrl;
	uint8_t m_joy_trim_dac;
	uint8_t m_reset;
	uint8_t m_active_voices;
	uint8_t m_irq_source;

	void set_irq(uint8_t source, uint8_t voice);
	void reset_irq(uint8_t source);
	void update_volume_ramps();

	std::vector<uint8_t> m_wave_ram;

	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_stop() override;

	virtual void update_irq() override;

private:
	// internal state
	sound_stream* m_stream;

	emu_timer* m_timer1;
	emu_timer* m_timer2;
	emu_timer* m_dmatimer;
	emu_timer* m_voltimer;

	uint8_t m_current_voice;
	uint8_t m_current_reg;
	//uint8_t m_port;
	//uint8_t m_irq;
	//uint8_t m_dma;

	uint8_t m_adlib_cmd;
	uint8_t m_mix_ctrl;
	uint8_t m_gf1_irq;
	uint8_t m_midi_irq;
	uint8_t m_dma_channel1;
	uint8_t m_dma_channel2;
	uint8_t m_irq_combine;
	uint8_t m_dma_combine;
	uint8_t m_adlib_timer_cmd;
	uint8_t m_adlib_timer1_enable;
	uint8_t m_adlib_timer2_enable;
	uint8_t m_adlib_status;
	uint8_t m_adlib_data;
	uint8_t m_voice_irq_fifo[32];
	uint8_t m_voice_irq_ptr;
	uint8_t m_voice_irq_current;
	uint8_t m_dma_16bit;  // set by bit 6 of the DMA DRAM control reg
	uint8_t m_statread;
	uint8_t m_sb_data_2xc;
	uint8_t m_sb_data_2xe;
	uint8_t m_reg_ctrl;
	uint8_t m_fake_adlib_status;
	uint32_t m_dma_current;
	uint16_t m_volume_table[4096];

	static const device_timer_id ADLIB_TIMER1 = 0;
	static const device_timer_id ADLIB_TIMER2 = 1;
	static const device_timer_id DMA_TIMER = 2;
	static const device_timer_id VOL_RAMP_TIMER = 3;

	int m_txirq;
	int m_rxirq;

	devcb_write_line m_txirq_handler;
	devcb_write_line m_rxirq_handler;
	devcb_write_line m_wave_irq_handler;
	devcb_write_line m_ramp_irq_handler;
	devcb_write_line m_timer1_irq_handler;
	devcb_write_line m_timer2_irq_handler;
	devcb_write_line m_sb_irq_handler;
	devcb_write_line m_dma_irq_handler;
	devcb_write_line m_drq1_handler;
	devcb_write_line m_drq2_handler;
	devcb_write_line m_nmi_handler;
};

class isa16_gus_device :
	public device_t,
	public device_isa16_card_interface
{
public:
	isa16_gus_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	void set_irq(uint8_t source);
	void reset_irq(uint8_t source);
	void set_midi_irq(uint8_t source);
	void reset_midi_irq(uint8_t source);

	DECLARE_READ8_MEMBER(board_r);
	DECLARE_READ8_MEMBER(synth_r);
	DECLARE_WRITE8_MEMBER(board_w);
	DECLARE_WRITE8_MEMBER(synth_w);
	DECLARE_READ8_MEMBER(adlib_r);
	DECLARE_WRITE8_MEMBER(adlib_w);
	DECLARE_READ8_MEMBER(joy_r);
	DECLARE_WRITE8_MEMBER(joy_w);

	// DMA overrides
	virtual uint8_t dack_r(int line) override;
	virtual void dack_w(int line,uint8_t data) override;
	virtual void eop_w(int state) override;

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_stop() override;

	// optional information overrides
	virtual void device_add_mconfig(machine_config &config) override;
	virtual ioport_constructor device_input_ports() const override;

private:
	DECLARE_WRITE_LINE_MEMBER(midi_txirq);
	DECLARE_WRITE_LINE_MEMBER(midi_rxirq);
	DECLARE_WRITE_LINE_MEMBER(wavetable_irq);
	DECLARE_WRITE_LINE_MEMBER(volumeramp_irq);
	DECLARE_WRITE_LINE_MEMBER(timer1_irq);
	DECLARE_WRITE_LINE_MEMBER(timer2_irq);
	DECLARE_WRITE_LINE_MEMBER(sb_irq);
	DECLARE_WRITE_LINE_MEMBER(dma_irq);
	DECLARE_WRITE_LINE_MEMBER(drq1_w);
	DECLARE_WRITE_LINE_MEMBER(drq2_w);
	DECLARE_WRITE_LINE_MEMBER(nmi_w);
	DECLARE_WRITE_LINE_MEMBER(write_acia_clock);

	required_device<gf1_device> m_gf1;

	uint8_t m_irq_status;
	attotime m_joy_time;
};

// device type definition
DECLARE_DEVICE_TYPE(GF1,       gf1_device)
DECLARE_DEVICE_TYPE(ISA16_GUS, isa16_gus_device)

#endif // MAME_BUS_ISA_GUS_H
