// license:BSD-3-Clause
// copyright-holders:Curt Coder
/*****************************************************************************
 *
 * includes/abc80.h
 *
 ****************************************************************************/

#ifndef MAME_INCLUDES_ABC80_H
#define MAME_INCLUDES_ABC80_H

#pragma once

#include "bus/abcbus/abcbus.h"
#include "bus/rs232/rs232.h"
#include "cpu/z80/z80.h"
#include "cpu/z80/z80daisy.h"
#include "imagedev/flopdrv.h"
#include "imagedev/printer.h"
#include "imagedev/cassette.h"
#include "imagedev/snapquik.h"
#include "machine/abc80kb.h"
#include "machine/keyboard.h"
#include "machine/ram.h"
#include "machine/z80pio.h"
#include "sound/sn76477.h"
#include "sound/wave.h"

#define ABC80_HTOTAL    384
#define ABC80_HBEND     30
#define ABC80_HBSTART   384
#define ABC80_VTOTAL    313
#define ABC80_VBEND     15
#define ABC80_VBSTART   313

#define ABC80_K5_HSYNC          0x01
#define ABC80_K5_DH             0x02
#define ABC80_K5_LINE_END       0x04
#define ABC80_K5_ROW_START      0x08

#define ABC80_K2_VSYNC          0x01
#define ABC80_K2_DV             0x02
#define ABC80_K2_FRAME_END      0x04
#define ABC80_K2_FRAME_RESET    0x08

#define ABC80_J3_BLANK          0x01
#define ABC80_J3_TEXT           0x02
#define ABC80_J3_GRAPHICS       0x04
#define ABC80_J3_VERSAL         0x08

#define ABC80_E7_VIDEO_RAM      0x01
#define ABC80_E7_INT_RAM        0x02
#define ABC80_E7_31K_EXT_RAM    0x04
#define ABC80_E7_16K_INT_RAM    0x08

#define ABC80_CHAR_CURSOR       0x80

#define SCREEN_TAG          "screen"
#define Z80_TAG             "ab67"
#define Z80PIO_TAG          "cd67"
#define SN76477_TAG         "g8"
#define RS232_TAG           "ser"
#define CASSETTE_TAG        "cassette"
#define KEYBOARD_TAG        "keyboard"
#define TIMER_CASSETTE_TAG  "cass"

class abc80_state : public driver_device
{
public:
	abc80_state(const machine_config &mconfig, device_type type, const char *tag) :
		driver_device(mconfig, type, tag),
		m_maincpu(*this, Z80_TAG),
		m_pio(*this, Z80PIO_TAG),
		m_csg(*this, SN76477_TAG),
		m_cassette(*this, "cassette"),
		m_bus(*this, ABCBUS_TAG),
		m_kb(*this, ABC80_KEYBOARD_TAG),
		m_ram(*this, RAM_TAG),
		m_rs232(*this, RS232_TAG),
		m_palette(*this, "palette"),
		m_screen(*this, SCREEN_TAG),
		m_rom(*this, Z80_TAG),
		m_mmu_rom(*this, "mmu"),
		m_char_rom(*this, "chargen"),
		m_hsync_prom(*this, "hsync"),
		m_vsync_prom(*this, "vsync"),
		m_line_prom(*this, "line"),
		m_attr_prom(*this, "attr"),
		m_video_ram(*this, "video_ram"),
		m_motor(false),
		m_tape_in(1),
		m_tape_in_latch(1)
	{ }

	required_device<cpu_device> m_maincpu;
	required_device<z80pio_device> m_pio;
	required_device<sn76477_device> m_csg;
	required_device<cassette_image_device> m_cassette;
	required_device<abcbus_slot_device> m_bus;
	required_device<abc80_keyboard_device> m_kb;
	required_device<ram_device> m_ram;
	required_device<rs232_port_device> m_rs232;
	required_device<palette_device> m_palette;
	required_device<screen_device> m_screen;
	required_memory_region m_rom;
	required_memory_region m_mmu_rom;
	required_memory_region m_char_rom;
	required_memory_region m_hsync_prom;
	required_memory_region m_vsync_prom;
	required_memory_region m_line_prom;
	required_memory_region m_attr_prom;
	optional_shared_ptr<uint8_t> m_video_ram;

	enum
	{
		TIMER_ID_SCANLINE,
		TIMER_ID_CASSETTE,
		TIMER_ID_BLINK,
		TIMER_ID_VSYNC_ON,
		TIMER_ID_VSYNC_OFF,
		TIMER_ID_FAKE_KEYBOARD_CLEAR
	};

	enum
	{
		BOFA = 0xfe1c,
		EOFA = 0xfe1e,
		HEAD = 0xfe20
	};

	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

	virtual void machine_start() override;

	virtual void video_start() override;
	uint32_t screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);

	void draw_scanline(bitmap_rgb32 &bitmap, int y);

	DECLARE_READ8_MEMBER( read );
	DECLARE_WRITE8_MEMBER( write );

	DECLARE_WRITE_LINE_MEMBER( vco_voltage_w );

	DECLARE_READ8_MEMBER( pio_pa_r );
	DECLARE_READ8_MEMBER( pio_pb_r );
	DECLARE_WRITE8_MEMBER( pio_pb_w );

	DECLARE_WRITE_LINE_MEMBER( keydown_w );
	void kbd_w(u8 data);
	DECLARE_WRITE8_MEMBER( csg_w );

	DECLARE_QUICKLOAD_LOAD_MEMBER( bac );

	enum
	{
		MMU_XM      = 0x01,
		MMU_ROM     = 0x02,
		MMU_VRAMS   = 0x04,
		MMU_RAM     = 0x08
	};

	// keyboard state
	int m_key_data;
	int m_key_strobe;
	int m_pio_astb;

	// video state
	bitmap_rgb32 m_bitmap;
	uint8_t m_latch;
	int m_blink;
	int m_c;
	int m_r;
	int m_mode;

	// cassette state
	bool m_motor;
	int m_tape_in;
	int m_tape_in_latch;

	// timers
	emu_timer *m_scanline_timer;
	emu_timer *m_cassette_timer;
	emu_timer *m_blink_timer;
	emu_timer *m_vsync_on_timer;
	emu_timer *m_vsync_off_timer;
};

//----------- defined in video/abc80.c -----------

MACHINE_CONFIG_EXTERN( abc80_video );

#endif // MAME_INCLUDES_ABC80_H
