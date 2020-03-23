// license:BSD-3-Clause
// copyright-holders:Curt Coder
#ifndef MAME_INCLUDES_PC1512_H
#define MAME_INCLUDES_PC1512_H

#pragma once

#include "bus/centronics/ctronics.h"
#include "bus/isa/isa.h"
#include "bus/isa/isa_cards.h"
#include "bus/isa/pc1640_iga.h"
#include "bus/pc1512/mouse.h"
#include "cpu/i86/i86.h"
#include "cpu/mcs48/mcs48.h"
#include "machine/am9517a.h"
#include "machine/buffer.h"
#include "machine/ins8250.h"
#include "machine/mc146818.h"
#include "machine/pic8259.h"
#include "machine/pit8253.h"
#include "machine/pc1512kb.h"
#include "machine/pc_fdc.h"
#include "machine/ram.h"
#include "sound/spkrdev.h"
#include "video/mc6845.h"

#include "formats/pc_dsk.h"

#define I8086_TAG       "ic120"
#define I8087_TAG       "ic119"
#define I8048_TAG       "i8048"
#define I8237A5_TAG     "ic130"
#define I8259A2_TAG     "ic109"
#define I8253_TAG       "ic114"
#define MC146818_TAG    "ic134"
#define PC_FDC_XT_TAG   "ic112"
#define INS8250_TAG     "ic106"
#define AMS40041_TAG    "ic126"
#define CENTRONICS_TAG  "centronics"
#define SPEAKER_TAG     "speaker"
#define ISA_BUS_TAG     "isa"
#define RS232_TAG       "rs232"
#define SCREEN_TAG      "screen"

class pc1512_state : public driver_device
{
public:
	pc1512_state(const machine_config &mconfig, device_type type, const char *tag) :
		driver_device(mconfig, type, tag),
		m_maincpu(*this, I8086_TAG),
		m_dmac(*this, I8237A5_TAG),
		m_pic(*this, I8259A2_TAG),
		m_pit(*this, I8253_TAG),
		m_rtc(*this, MC146818_TAG),
		m_fdc(*this, PC_FDC_XT_TAG),
		m_uart(*this, INS8250_TAG),
		m_vdu(*this, AMS40041_TAG),
		m_centronics(*this, CENTRONICS_TAG),
		m_cent_data_out(*this, "cent_data_out"),
		m_speaker(*this, SPEAKER_TAG),
		m_kb(*this, PC1512_KEYBOARD_TAG),
		m_ram(*this, RAM_TAG),
		m_floppy0(*this, PC_FDC_XT_TAG ":0:525dd" ),
		m_floppy1(*this, PC_FDC_XT_TAG ":1:525dd" ),
		m_bus(*this, ISA_BUS_TAG),
		m_char_rom(*this, AMS40041_TAG),
		m_video_ram(*this, "video_ram"),
		m_lk(*this, "LK"),
		m_pit1(0),
		m_pit2(0),
		m_status1(0),
		m_status2(0),
		m_port61(0),
		m_nmi_enable(0),
		m_kb_bits(0),
		m_kbclk(1),
		m_kbdata(1),
		m_dreq0(0),
		m_nden(1),
		m_dint(0),
		m_ddrq(0),
		m_fdc_dsr(0),
		m_neop(0),
		m_ack_int_enable(1),
		m_centronics_ack(0),
		m_speaker_drive(0)
	{ }

	required_device<cpu_device> m_maincpu;
	required_device<am9517a_device> m_dmac;
	required_device<pic8259_device> m_pic;
	required_device<pit8253_device> m_pit;
	required_device<mc146818_device> m_rtc;
	required_device<pc_fdc_xt_device> m_fdc;
	required_device<ins8250_device> m_uart;
	optional_device<ams40041_device> m_vdu;
	required_device<centronics_device> m_centronics;
	required_device<output_latch_device> m_cent_data_out;
	required_device<speaker_sound_device> m_speaker;
	required_device<pc1512_keyboard_device> m_kb;
	required_device<ram_device> m_ram;
	required_device<floppy_image_device> m_floppy0;
	optional_device<floppy_image_device> m_floppy1;
	required_device<isa8_device> m_bus;
	optional_memory_region m_char_rom;
	optional_shared_ptr<uint8_t> m_video_ram;
	required_ioport m_lk;

	virtual void machine_start() override;
	virtual void machine_reset() override;

	virtual void video_start() override;

	uint32_t screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);

	void update_speaker();
	void update_fdc_int();
	void update_fdc_drq();
	void update_fdc_tc();
	void update_ack();
	int get_display_mode(uint8_t mode);
	offs_t get_char_rom_offset();
	int get_color(uint8_t data);
	MC6845_UPDATE_ROW(draw_alpha);
	MC6845_UPDATE_ROW(draw_graphics_1);
	MC6845_UPDATE_ROW(draw_graphics_2);

	DECLARE_READ8_MEMBER( video_ram_r );
	DECLARE_WRITE8_MEMBER( video_ram_w );
	DECLARE_READ8_MEMBER( system_r );
	DECLARE_WRITE8_MEMBER( system_w );
	DECLARE_READ8_MEMBER( mouse_r );
	DECLARE_WRITE8_MEMBER( mouse_w );
	DECLARE_WRITE8_MEMBER( dma_page_w );
	DECLARE_WRITE8_MEMBER( nmi_mask_w );
	DECLARE_READ8_MEMBER( printer_r );
	DECLARE_WRITE8_MEMBER( printer_w );
	DECLARE_READ8_MEMBER( vdu_r );
	DECLARE_WRITE8_MEMBER( vdu_w );
	DECLARE_WRITE_LINE_MEMBER( kbdata_w );
	DECLARE_WRITE_LINE_MEMBER( kbclk_w );
	DECLARE_WRITE_LINE_MEMBER( pit1_w );
	DECLARE_WRITE_LINE_MEMBER( pit2_w );
	DECLARE_WRITE_LINE_MEMBER( hrq_w );
	DECLARE_WRITE_LINE_MEMBER( eop_w );
	DECLARE_READ8_MEMBER( memr_r );
	DECLARE_WRITE8_MEMBER( memw_w );
	DECLARE_READ8_MEMBER( ior1_r );
	DECLARE_READ8_MEMBER( ior2_r );
	DECLARE_READ8_MEMBER( ior3_r );
	DECLARE_WRITE8_MEMBER( iow0_w );
	DECLARE_WRITE8_MEMBER( iow1_w );
	DECLARE_WRITE8_MEMBER( iow2_w );
	DECLARE_WRITE8_MEMBER( iow3_w );
	DECLARE_WRITE_LINE_MEMBER( dack0_w );
	DECLARE_WRITE_LINE_MEMBER( dack1_w );
	DECLARE_WRITE_LINE_MEMBER( dack2_w );
	DECLARE_WRITE_LINE_MEMBER( dack3_w );
	DECLARE_FLOPPY_FORMATS( floppy_formats );
	DECLARE_WRITE_LINE_MEMBER( fdc_int_w );
	DECLARE_WRITE_LINE_MEMBER( fdc_drq_w );
	DECLARE_WRITE_LINE_MEMBER(write_centronics_ack);
	DECLARE_WRITE_LINE_MEMBER(write_centronics_busy);
	DECLARE_WRITE_LINE_MEMBER(write_centronics_perror);
	DECLARE_WRITE_LINE_MEMBER(write_centronics_select);
	DECLARE_WRITE_LINE_MEMBER(write_centronics_fault);
	DECLARE_WRITE8_MEMBER( mouse_x_w );
	DECLARE_WRITE8_MEMBER( mouse_y_w );
	MC6845_UPDATE_ROW(crtc_update_row);

	// system status register
	int m_pit1;
	int m_pit2;
	uint8_t m_status1;
	uint8_t m_status2;
	uint8_t m_port61;

	// interrupt state
	int m_nmi_enable;

	// keyboard state
	uint8_t m_kbd;
	int m_kb_bits;
	int m_kbclk;
	int m_kbdata;

	// mouse state
	uint8_t m_mouse_x_old;
	uint8_t m_mouse_y_old;
	uint8_t m_mouse_x;
	uint8_t m_mouse_y;

	// DMA state
	uint8_t m_dma_page[4];
	int m_dma_channel;
	int m_dreq0;

	// floppy state
	int m_nden;
	int m_dint;
	int m_ddrq;
	uint8_t m_fdc_dsr;
	int m_neop;

	// printer state
	int m_ack_int_enable;
	int m_centronics_ack;
	int m_centronics_busy;
	int m_centronics_perror;
	int m_centronics_select;
	int m_centronics_fault;
	uint8_t m_printer_data;
	uint8_t m_printer_control;

	// video state
	int m_toggle;
	int m_lpen;
	int m_blink;
	int m_cursor;
	int m_blink_ctr;
	uint8_t m_vdu_mode;
	uint8_t m_vdu_color;
	uint8_t m_vdu_plane;
	uint8_t m_vdu_rdsel;
	uint8_t m_vdu_border;

	// sound state
	int m_speaker_drive;
};

class pc1640_state : public pc1512_state
{
public:
	pc1640_state(const machine_config &mconfig, device_type type, const char *tag) :
		pc1512_state(mconfig, type, tag),
		m_sw(*this, "SW"),
		m_opt(0)
	{ }

	virtual void machine_start() override;
	virtual void machine_reset() override;

	DECLARE_READ8_MEMBER( io_r );
	DECLARE_READ8_MEMBER( printer_r );

	required_ioport m_sw;

	int m_opt;
};

// ---------- defined in video/pc1512.c ----------

MACHINE_CONFIG_EXTERN( pc1512_video );

#endif // MAME_INCLUDES_PC1512_H
