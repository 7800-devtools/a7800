// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/*************************************************************************

    ldpr8210.h

    Pioneer PR-8210 laserdisc emulation.

*************************************************************************/

#ifndef MAME_MACHINE_LDPR8210_H
#define MAME_MACHINE_LDPR8210_H

#pragma once

#include "laserdsc.h"
#include "cpu/mcs48/mcs48.h"


//**************************************************************************
//  DEVICE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_LASERDISC_PR8210_ADD(_tag) \
	MCFG_DEVICE_ADD(_tag, PIONEER_PR8210, 0)
#define MCFG_LASERDISC_SIMUTREK_ADD(_tag) \
	MCFG_DEVICE_ADD(_tag, SIMUTREK_SPECIAL, 0)


//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

// device type definition
DECLARE_DEVICE_TYPE(PIONEER_PR8210,   pioneer_pr8210_device)
DECLARE_DEVICE_TYPE(SIMUTREK_SPECIAL, simutrek_special_device)



//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> pioneer_pr8210_device

// base pr8210 class
class pioneer_pr8210_device : public laserdisc_device
{
public:
	// construction/destruction
	pioneer_pr8210_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// input and output
	void control_w(uint8_t data);

protected:
	// timer IDs
	enum
	{
		TID_VSYNC_OFF = TID_FIRST_PLAYER_TIMER,
		TID_VBI_DATA_FETCH,
		TID_FIRST_SUBCLASS_TIMER
	};

	pioneer_pr8210_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;
	virtual const tiny_rom_entry *device_rom_region() const override;
	virtual void device_add_mconfig(machine_config &config) override;

	// subclass overrides
	virtual void player_vsync(const vbi_metadata &vbi, int fieldnum, const attotime &curtime) override;
	virtual int32_t player_update(const vbi_metadata &vbi, int fieldnum, const attotime &curtime) override;
	virtual void player_overlay(bitmap_yuy16 &bitmap) override;

	// internal helpers
	bool focus_on() const { return !(m_i8049_port1 & 0x08); }
	bool spdl_on() const { return !(m_i8049_port1 & 0x10); }
	bool laser_on() const { return !(m_i8049_port2 & 0x01); }
	virtual bool override_control() const { return false; }
	void update_video_squelch() { set_video_squelch((m_i8049_port1 & 0x20) != 0); }
	virtual void update_audio_squelch() { set_audio_squelch((m_i8049_port1 & 0x40) || !(m_pia.portb & 0x01), (m_i8049_port1 & 0x40) || !(m_pia.portb & 0x02)); }

public:
	DECLARE_READ8_MEMBER( i8049_pia_r );
	DECLARE_WRITE8_MEMBER( i8049_pia_w );

protected:
	// internal read/write handlers
	DECLARE_READ8_MEMBER( i8049_bus_r );
	DECLARE_WRITE8_MEMBER( i8049_port1_w );
	DECLARE_WRITE8_MEMBER( i8049_port2_w );
	DECLARE_READ_LINE_MEMBER( i8049_t0_r );
	DECLARE_READ_LINE_MEMBER( i8049_t1_r );

	// pioneer PIA subclass
	class pioneer_pia
	{
	public:
		uint8_t               frame[7];               // (20-26) 7 characters for the chapter/frame
		uint8_t               text[17];               // (20-30) 17 characters for the display
		uint8_t               control;                // (40) control lines
		uint8_t               latchdisplay;           //   flag: set if the display was latched
		uint8_t               portb;                  // (60) port B value (LEDs)
		uint8_t               display;                // (80) display enable
		uint8_t               porta;                  // (A0) port A value (from serial decoder)
		uint8_t               vbi1;                   // (C0) VBI decoding state 1
		uint8_t               vbi2;                   // (E0) VBI decoding state 2
	};

	// internal overlay helpers
	void overlay_draw_group(bitmap_yuy16 &bitmap, const uint8_t *text, int count, float xstart);
	void overlay_erase(bitmap_yuy16 &bitmap, float xstart, float xend);
	void overlay_draw_char(bitmap_yuy16 &bitmap, uint8_t ch, float xstart);

	// internal state
	uint8_t               m_control;              // control line state
	uint8_t               m_lastcommand;          // last command seen
	uint16_t              m_accumulator;          // bit accumulator
	attotime            m_lastcommandtime;      // time of the last command
	attotime            m_lastbittime;          // time of last bit received
	attotime            m_firstbittime;         // time of first bit in command

	// low-level emulation data
	required_device<i8049_device> m_i8049_cpu;  // 8049 CPU device
	attotime            m_slowtrg;              // time of the last SLOW TRG
	pioneer_pia         m_pia;                  // PIA state
	bool                m_vsync;                // live VSYNC state
	uint8_t               m_i8049_port1;          // 8049 port 1 state
	uint8_t               m_i8049_port2;          // 8049 port 2 state
};


// ======================> simutrek_special_device

class simutrek_special_device : public pioneer_pr8210_device
{
public:
	// construction/destruction
	simutrek_special_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// input and output
	void data_w(uint8_t data);
	uint8_t ready_r() const { return !m_data_ready; }
	uint8_t status_r() const { return ((m_i8748_port2 & 0x03) == 0x03) ? ASSERT_LINE : CLEAR_LINE; }

	// external controls
	void set_external_audio_squelch(int state);

protected:
	// timer IDs
	enum
	{
		TID_IRQ_OFF = TID_FIRST_SUBCLASS_TIMER,
		TID_LATCH_DATA
	};

	// subclass overrides
	virtual void player_vsync(const vbi_metadata &vbi, int fieldnum, const attotime &curtime) override;

	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;
	virtual const tiny_rom_entry *device_rom_region() const override;
	virtual void device_add_mconfig(machine_config &config) override;

	// internal helpers
	virtual bool override_control() const override { return m_controlthis; }
	virtual void update_audio_squelch() override { set_audio_squelch(m_audio_squelch, m_audio_squelch); }

public:
	DECLARE_READ8_MEMBER( i8748_data_r );

private:
	// internal read/write handlers
	DECLARE_READ8_MEMBER( i8748_port2_r );
	DECLARE_WRITE8_MEMBER( i8748_port2_w );
	DECLARE_READ_LINE_MEMBER( i8748_t0_r );

	// internal state
	required_device<i8748_device> m_i8748_cpu;
	uint8_t               m_audio_squelch;            // audio squelch value
	uint8_t               m_data;                 // parallel data for simutrek
	bool                m_data_ready;               // ready flag for simutrek data
	uint8_t               m_i8748_port2;                  // 8748 port 2 state
	uint8_t               m_controlnext;          // latch to control next pair of fields
	uint8_t               m_controlthis;          // latched value for our control over the current pair of fields
};

#endif // MAME_MACHINE_LDPR8210_H
