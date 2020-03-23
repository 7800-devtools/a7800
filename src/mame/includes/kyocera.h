// license:BSD-3-Clause
// copyright-holders:Curt Coder
#ifndef MAME_INCLUDES_KYOCERA_H
#define MAME_INCLUDES_KYOCERA_H

#pragma once


#include "cpu/i8085/i8085.h"
#include "imagedev/cassette.h"
#include "machine/buffer.h"
#include "machine/i8155.h"
#include "machine/i8251.h"
#include "machine/im6402.h"
#include "machine/ram.h"
#include "machine/rp5c01.h"
#include "machine/upd1990a.h"
#include "sound/spkrdev.h"
#include "video/hd44102.h"
#include "video/hd61830.h"

#include "bus/generic/slot.h"
#include "bus/generic/carts.h"
#include "bus/centronics/ctronics.h"
#include "bus/rs232/rs232.h"

#include "rendlay.h"


#define SCREEN_TAG      "screen"
#define I8085_TAG       "m19"
#define I8155_TAG       "m25"
#define UPD1990A_TAG    "m18"
#define IM6402_TAG      "m22"
#define MC14412_TAG     "m31"
#define HD44102_0_TAG   "m1"
#define HD44102_1_TAG   "m2"
#define HD44102_2_TAG   "m3"
#define HD44102_3_TAG   "m4"
#define HD44102_4_TAG   "m5"
#define HD44102_5_TAG   "m6"
#define HD44102_6_TAG   "m7"
#define HD44102_7_TAG   "m8"
#define HD44102_8_TAG   "m9"
#define HD44102_9_TAG   "m10"
#define CENTRONICS_TAG  "centronics"
#define RS232_TAG       "rs232"

//#define I8085_TAG     "m19"
//#define I8155_TAG     "m12"
//#define MC14412_TAG   "m8"
#define RP5C01A_TAG     "m301"
#define TCM5089_TAG     "m11"
#define I8251_TAG       "m20"
#define HD61830_TAG     "m18"

class kc85_state : public driver_device
{
public:
	kc85_state(const machine_config &mconfig, device_type type, const char *tag) :
		driver_device(mconfig, type, tag),
		m_maincpu(*this, I8085_TAG),
		m_rtc(*this, UPD1990A_TAG),
		m_uart(*this, IM6402_TAG),
		m_lcdc0(*this, HD44102_0_TAG),
		m_lcdc1(*this, HD44102_1_TAG),
		m_lcdc2(*this, HD44102_2_TAG),
		m_lcdc3(*this, HD44102_3_TAG),
		m_lcdc4(*this, HD44102_4_TAG),
		m_lcdc5(*this, HD44102_5_TAG),
		m_lcdc6(*this, HD44102_6_TAG),
		m_lcdc7(*this, HD44102_7_TAG),
		m_lcdc8(*this, HD44102_8_TAG),
		m_lcdc9(*this, HD44102_9_TAG),
		m_centronics(*this, CENTRONICS_TAG),
		m_speaker(*this, "speaker"),
		m_cassette(*this, "cassette"),
		m_opt_cart(*this, "opt_cartslot"),
		m_ram(*this, RAM_TAG),
		m_rs232(*this, RS232_TAG),
		m_rom(*this, I8085_TAG),
		m_y(*this, "Y%u", 0),
		m_battery(*this, "BATTERY")
	{ }

	required_device<cpu_device> m_maincpu;
	required_device<upd1990a_device> m_rtc;
	optional_device<im6402_device> m_uart;
	required_device<hd44102_device> m_lcdc0;
	required_device<hd44102_device> m_lcdc1;
	required_device<hd44102_device> m_lcdc2;
	required_device<hd44102_device> m_lcdc3;
	required_device<hd44102_device> m_lcdc4;
	required_device<hd44102_device> m_lcdc5;
	required_device<hd44102_device> m_lcdc6;
	required_device<hd44102_device> m_lcdc7;
	required_device<hd44102_device> m_lcdc8;
	required_device<hd44102_device> m_lcdc9;
	required_device<centronics_device> m_centronics;
	required_device<speaker_sound_device> m_speaker;
	required_device<cassette_image_device> m_cassette;
	required_device<generic_slot_device> m_opt_cart;
	required_device<ram_device> m_ram;
	required_device<rs232_port_device> m_rs232;
	required_memory_region m_rom;
	required_ioport_array<9> m_y;
	required_ioport m_battery;

	virtual void machine_start() override;
	memory_region *m_opt_region;

	uint32_t screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);

	DECLARE_READ8_MEMBER( uart_status_r );
	DECLARE_WRITE8_MEMBER( uart_ctrl_w );
	DECLARE_WRITE8_MEMBER( modem_w );
	DECLARE_WRITE8_MEMBER( ctrl_w );
	DECLARE_READ8_MEMBER( keyboard_r );
	DECLARE_READ8_MEMBER( lcd_r );
	DECLARE_WRITE8_MEMBER( lcd_w );
	DECLARE_WRITE8_MEMBER( i8155_pa_w );
	DECLARE_WRITE8_MEMBER( i8155_pb_w );
	DECLARE_READ8_MEMBER( i8155_pc_r );
	DECLARE_WRITE_LINE_MEMBER( i8155_to_w );
	DECLARE_WRITE_LINE_MEMBER( write_centronics_busy );
	DECLARE_WRITE_LINE_MEMBER( write_centronics_select );

	/* memory state */
	uint8_t m_bank;           /* memory bank selection */

	/* keyboard state */
	uint16_t m_keylatch;      /* keyboard latch */

	/* sound state */
	int m_buzzer;               /* buzzer select */
	int m_bell;             /* bell output */

	int m_centronics_busy;
	int m_centronics_select;

	DECLARE_PALETTE_INIT(kc85);
	DECLARE_WRITE_LINE_MEMBER(kc85_sod_w);
	DECLARE_READ_LINE_MEMBER(kc85_sid_r);
};

class trsm100_state : public kc85_state
{
public:
	trsm100_state(const machine_config &mconfig, device_type type, const char *tag)
		: kc85_state(mconfig, type, tag) { }

	virtual void machine_start() override;
};

class pc8201_state : public kc85_state
{
public:
	pc8201_state(const machine_config &mconfig, device_type type, const char *tag)
		: kc85_state(mconfig, type, tag),
			m_cas_cart(*this, "cas_cartslot")
	{ }

	virtual void machine_start() override;
	required_device<generic_slot_device> m_cas_cart;

	DECLARE_READ8_MEMBER( bank_r );
	DECLARE_WRITE8_MEMBER( bank_w );
	DECLARE_WRITE8_MEMBER( scp_w );
	DECLARE_READ8_MEMBER( uart_status_r );
	DECLARE_WRITE8_MEMBER( romah_w );
	DECLARE_WRITE8_MEMBER( romal_w );
	DECLARE_WRITE8_MEMBER( romam_w );
	DECLARE_READ8_MEMBER( romrd_r );

	void bankswitch(uint8_t data);

	// ROM cassette
	int m_rom_sel;
	uint32_t m_rom_addr;

	/* peripheral state */
	int m_iosel;                /* serial interface select */
};

class tandy200_state : public driver_device
{
public:
	tandy200_state(const machine_config &mconfig, device_type type, const char *tag) :
		driver_device(mconfig, type, tag),
		m_maincpu(*this, I8085_TAG),
		m_rtc(*this, RP5C01A_TAG),
		m_lcdc(*this, HD61830_TAG),
		m_centronics(*this, CENTRONICS_TAG),
		m_cent_data_out(*this, "cent_data_out"),
		m_speaker(*this, "speaker"),
		m_cassette(*this, "cassette"),
		m_opt_cart(*this, "opt_cartslot"),
		m_ram(*this, RAM_TAG),
		m_rs232(*this, RS232_TAG),
		m_rom(*this, I8085_TAG),
		m_y(*this, "Y%u", 0)
	{ }

	required_device<cpu_device> m_maincpu;
	required_device<rp5c01_device> m_rtc;
	required_device<hd61830_device> m_lcdc;
	required_device<centronics_device> m_centronics;
	required_device<output_latch_device> m_cent_data_out;
	required_device<speaker_sound_device> m_speaker;
	required_device<cassette_image_device> m_cassette;
	required_device<generic_slot_device> m_opt_cart;
	required_device<ram_device> m_ram;
	required_device<rs232_port_device> m_rs232;
	required_memory_region m_rom;
	required_ioport_array<9> m_y;

	virtual void machine_start() override;
	memory_region *m_opt_region;

	uint32_t screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);

	DECLARE_READ8_MEMBER( bank_r );
	DECLARE_WRITE8_MEMBER( bank_w );
	DECLARE_READ8_MEMBER( stbk_r );
	DECLARE_WRITE8_MEMBER( stbk_w );
	DECLARE_WRITE8_MEMBER( i8155_pa_w );
	DECLARE_WRITE8_MEMBER( i8155_pb_w );
	DECLARE_READ8_MEMBER( i8155_pc_r );
	DECLARE_WRITE_LINE_MEMBER( i8155_to_w );
	DECLARE_WRITE_LINE_MEMBER(kc85_sod_w);
	DECLARE_READ_LINE_MEMBER(kc85_sid_r);
	DECLARE_WRITE_LINE_MEMBER( write_centronics_busy );
	DECLARE_WRITE_LINE_MEMBER( write_centronics_select );

	DECLARE_PALETTE_INIT(tandy200);

	TIMER_DEVICE_CALLBACK_MEMBER(tandy200_tp_tick);

	void bankswitch(uint8_t data);

	/* memory state */
	uint8_t m_bank;           /* memory bank selection */

	/* keyboard state */
	uint16_t m_keylatch;      /* keyboard latch */
	int m_tp;               /* timing pulse */

	/* sound state */
	int m_buzzer;           /* buzzer select */
	int m_bell;             /* bell output */

	int m_centronics_busy;
	int m_centronics_select;
};

/* ---------- defined in video/kyocera.c ---------- */

MACHINE_CONFIG_EXTERN( kc85_video );
MACHINE_CONFIG_EXTERN( tandy200_video );

#endif // MAME_INCLUDES_KYOCERA_H
