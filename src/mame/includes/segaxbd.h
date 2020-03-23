// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/***************************************************************************

    Sega X-Board hardware

***************************************************************************/
#ifndef MAME_INCLUDES_SEGAXBD_H
#define MAME_INCLUDES_SEGAXBD_H

#pragma once

#include "machine/segaic16.h"
#include "video/segaic16.h"
#include "video/segaic16_road.h"
#include "video/sega16sp.h"

#include "cpu/m68000/m68000.h"
#include "cpu/mcs51/mcs51.h"
#include "cpu/z80/z80.h"
#include "machine/gen_latch.h"
#include "machine/mb3773.h"
#include "machine/watchdog.h"
#include "video/resnet.h"
#include "screen.h"

// ======================> segaxbd_state


class segaxbd_state : public device_t
{
public:
	// construction/destruction
	segaxbd_state(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// main CPU read/write handlers
	DECLARE_READ16_MEMBER(adc_r);
	DECLARE_WRITE16_MEMBER(adc_w);
	DECLARE_WRITE16_MEMBER(iocontrol_w);

	// game-specific main CPU read/write handlers
	DECLARE_WRITE16_MEMBER(loffire_sync0_w);
	DECLARE_READ16_MEMBER(rascot_excs_r);
	DECLARE_WRITE16_MEMBER(rascot_excs_w);
	DECLARE_READ16_MEMBER(smgp_excs_r);
	DECLARE_WRITE16_MEMBER(smgp_excs_w);

	// custom I/O
	DECLARE_READ8_MEMBER(aburner2_motor_r);
	DECLARE_WRITE8_MEMBER(aburner2_motor_w);
	DECLARE_READ8_MEMBER(smgp_motor_r);
	DECLARE_WRITE8_MEMBER(smgp_motor_w);
	DECLARE_READ8_MEMBER(lastsurv_port_r);
	DECLARE_WRITE8_MEMBER(lastsurv_muxer_w);

	// sound Z80 CPU read/write handlers
	DECLARE_READ8_MEMBER(sound_data_r);

	// video updates
	uint32_t screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);

	// palette helpers
	DECLARE_WRITE16_MEMBER(paletteram_w);

	void install_aburner2(void);
	void install_loffire(void);
	void install_smgp(void);
	void install_gprider(void);

protected:
	// timer IDs
	enum
	{
		TID_SCANLINE,
		TID_IRQ2_GEN,
		TID_SOUND_WRITE
	};

	segaxbd_state(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// device overrides
//  virtual void machine_reset();
	virtual void video_start();
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

	// internal helpers
	void update_main_irqs();
	DECLARE_WRITE_LINE_MEMBER(m68k_reset_callback);
	void generic_iochip0_lamps_w(uint8_t data);

		// compare/timer chip callbacks
	void timer_ack_callback();
	void sound_data_w(uint8_t data);

	DECLARE_WRITE8_MEMBER(pc_0_w);
	DECLARE_WRITE8_MEMBER(pd_0_w);

	// devices
public:
	required_device<m68000_device> m_maincpu;
protected:
	required_device<m68000_device> m_subcpu;
	required_device<z80_device> m_soundcpu;
	optional_device<z80_device> m_soundcpu2;
	optional_device<i8751_device> m_mcu;
	required_device<mb3773_device> m_watchdog;
	required_device<sega_315_5250_compare_timer_device> m_cmptimer_1;
	required_device<sega_xboard_sprite_device> m_sprites;
	required_device<segaic16_video_device> m_segaic16vid;
	required_device<segaic16_road_device> m_segaic16road;
	required_device<generic_latch_8_device> m_soundlatch;
	required_shared_ptr<uint16_t> m_subram0;

	// configuration
	bool            m_adc_reverse[8];
	uint8_t           m_road_priority;

	// internal state
	emu_timer *     m_scanline_timer;
	uint8_t           m_timer_irq_state;
	uint8_t           m_vblank_irq_state;
	uint8_t           m_pc_0;

	// game-specific state
	uint16_t *        m_loffire_sync;
	uint8_t           m_lastsurv_mux;
public: // -- stupid system16.c
	// memory pointers
	required_shared_ptr<uint16_t> m_paletteram;
	bool            m_gprider_hack;

protected:
	void palette_init();
	uint32_t      m_palette_entries;          // number of palette entries
	uint8_t       m_palette_normal[32];       // RGB translations for normal pixels
	uint8_t       m_palette_shadow[32];       // RGB translations for shadowed pixels
	uint8_t       m_palette_hilight[32];      // RGB translations for hilighted pixels
	required_device<screen_device> m_screen;
	required_device<palette_device> m_palette;
	required_ioport m_io0_porta;
	optional_ioport_array<8> m_adc_ports;
	optional_ioport_array<4> m_mux_ports;

protected:
	virtual void device_start() override;
	virtual void device_reset() override;
	void xboard_base_mconfig(machine_config &config);
};


class segaxbd_regular_state :  public segaxbd_state
{
public:
	segaxbd_regular_state(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	virtual void device_add_mconfig(machine_config &config) override;
};



class segaxbd_fd1094_state :  public segaxbd_state
{
public:
	segaxbd_fd1094_state(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	virtual void device_add_mconfig(machine_config &config) override;
};

class segaxbd_aburner2_state :  public segaxbd_state
{
public:
	segaxbd_aburner2_state(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	virtual void device_add_mconfig(machine_config &config) override;
//  virtual void device_start();
//  virtual void device_reset();
};

class segaxbd_lastsurv_fd1094_state :  public segaxbd_state
{
public:
	segaxbd_lastsurv_fd1094_state(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	virtual void device_add_mconfig(machine_config &config) override;
};

class segaxbd_lastsurv_state :  public segaxbd_state
{
public:
	segaxbd_lastsurv_state(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	virtual void device_add_mconfig(machine_config &config) override;
};


class segaxbd_smgp_fd1094_state :  public segaxbd_state
{
public:
	segaxbd_smgp_fd1094_state(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	virtual void device_add_mconfig(machine_config &config) override;
};


class segaxbd_smgp_state :  public segaxbd_state
{
public:
	segaxbd_smgp_state(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	virtual void device_add_mconfig(machine_config &config) override;
};


class segaxbd_rascot_state :  public segaxbd_state
{
public:
	segaxbd_rascot_state(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	virtual void device_add_mconfig(machine_config &config) override;
};

#endif // MAME_INCLUDES_SEGAXBD_H
