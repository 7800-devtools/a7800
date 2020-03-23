// license:BSD-3-Clause
// copyright-holders:Phill Harvey-Smith, Carl
/*
    rmnimbus.c
    Machine driver for the Research Machines Nimbus.

    Phill Harvey-Smith
    2009-11-29.
*/
#ifndef MAME_INCLUDES_RMNIMBUS_H
#define MAME_INCLUDES_RMNIMBUS_H

#pragma once

#include "cpu/i86/i186.h"
#include "machine/z80dart.h"
#include "machine/wd_fdc.h"
#include "bus/scsi/scsi.h"
#include "machine/6522via.h"
#include "machine/ram.h"
#include "machine/eepromser.h"
#include "sound/ay8910.h"
#include "sound/msm5205.h"
#include "bus/centronics/ctronics.h"
#include "screen.h"

#define MAINCPU_TAG "maincpu"
#define IOCPU_TAG   "iocpu"
#define Z80SIO_TAG  "z80sio"
#define FDC_TAG     "wd2793"
#define SCSIBUS_TAG "scsibus"
#define ER59256_TAG "er59256"
#define AY8910_TAG  "ay8910"
#define MONO_TAG    "mono"
#define MSM5205_TAG "msm5205"
#define VIA_TAG     "via6522"
#define CENTRONICS_TAG "centronics"

/* Mouse / Joystick */

#define JOYSTICK0_TAG           "joystick0"
#define MOUSE_BUTTON_TAG        "mousebtn"
#define MOUSEX_TAG              "mousex"
#define MOUSEY_TAG              "mousey"

/* Memory controller */
#define RAM_BANK00_TAG  "bank0"
#define RAM_BANK01_TAG  "bank1"
#define RAM_BANK02_TAG  "bank2"
#define RAM_BANK03_TAG  "bank3"
#define RAM_BANK04_TAG  "bank4"
#define RAM_BANK05_TAG  "bank5"
#define RAM_BANK06_TAG  "bank6"
#define RAM_BANK07_TAG  "bank7"

class rmnimbus_state : public driver_device
{
public:
	rmnimbus_state(const machine_config &mconfig, device_type type, const char *tag) :
		driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_msm(*this, MSM5205_TAG),
		m_scsibus(*this, SCSIBUS_TAG),
		m_ram(*this, RAM_TAG),
		m_eeprom(*this, ER59256_TAG),
		m_via(*this, VIA_TAG),
		m_centronics(*this, CENTRONICS_TAG),
		m_palette(*this, "palette"),
		m_scsi_data_out(*this, "scsi_data_out"),
		m_scsi_data_in(*this, "scsi_data_in"),
		m_scsi_ctrl_out(*this, "scsi_ctrl_out"),
		m_fdc(*this, FDC_TAG),
		m_z80sio(*this, Z80SIO_TAG),
		m_screen(*this, "screen"),
		m_io_config(*this, "config"),
		m_io_joystick0(*this, JOYSTICK0_TAG),
		m_io_mouse_button(*this, MOUSE_BUTTON_TAG),
		m_io_mousex(*this, MOUSEX_TAG),
		m_io_mousey(*this, MOUSEY_TAG)
	{
	}

	required_device<i80186_cpu_device> m_maincpu;
	required_device<msm5205_device> m_msm;
	required_device<scsi_port_device> m_scsibus;
	required_device<ram_device> m_ram;
	required_device<eeprom_serial_93cxx_device> m_eeprom;
	required_device<via6522_device> m_via;
	required_device<centronics_device> m_centronics;
	required_device<palette_device> m_palette;
	required_device<output_latch_device> m_scsi_data_out;
	required_device<input_buffer_device> m_scsi_data_in;
	required_device<output_latch_device> m_scsi_ctrl_out;
	required_device<wd2793_device> m_fdc;
	required_device<z80sio2_device> m_z80sio;
	required_device<screen_device> m_screen;
	required_ioport m_io_config;
	required_ioport m_io_joystick0;
	required_ioport m_io_mouse_button;
	required_ioport m_io_mousex;
	required_ioport m_io_mousey;

	bitmap_ind16 m_video_mem;

	uint32_t m_debug_machine;
	uint8_t m_mcu_reg080;
	uint8_t m_iou_reg092;
	uint8_t m_last_playmode;
	uint8_t m_ay8910_a;
	uint16_t m_x, m_y, m_yline;
	uint8_t m_colours, m_mode, m_op;
	uint32_t m_debug_video;
	uint8_t m_vector;
	uint8_t m_eeprom_bits;
	uint8_t m_eeprom_state;

	DECLARE_READ8_MEMBER(nimbus_mcu_r);
	DECLARE_WRITE8_MEMBER(nimbus_mcu_w);
	DECLARE_READ8_MEMBER(scsi_r);
	DECLARE_WRITE8_MEMBER(scsi_w);
	DECLARE_WRITE8_MEMBER(fdc_ctl_w);
	DECLARE_READ8_MEMBER(nimbus_pc8031_r);
	DECLARE_WRITE8_MEMBER(nimbus_pc8031_w);
	DECLARE_READ8_MEMBER(nimbus_pc8031_iou_r);
	DECLARE_WRITE8_MEMBER(nimbus_pc8031_iou_w);
	DECLARE_READ8_MEMBER(nimbus_pc8031_port_r);
	DECLARE_WRITE8_MEMBER(nimbus_pc8031_port_w);
	DECLARE_READ8_MEMBER(nimbus_iou_r);
	DECLARE_WRITE8_MEMBER(nimbus_iou_w);
	DECLARE_WRITE8_MEMBER(nimbus_sound_ay8910_porta_w);
	DECLARE_WRITE8_MEMBER(nimbus_sound_ay8910_portb_w);
	DECLARE_READ8_MEMBER(nimbus_mouse_js_r);
	DECLARE_WRITE8_MEMBER(nimbus_mouse_js_w);
	DECLARE_READ16_MEMBER(nimbus_video_io_r);
	DECLARE_WRITE16_MEMBER(nimbus_video_io_w);
	virtual void machine_start() override;
	virtual void machine_reset() override;
	virtual void video_start() override;
	virtual void video_reset() override;
	uint32_t screen_update_nimbus(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	DECLARE_WRITE_LINE_MEMBER(sio_interrupt);
	DECLARE_WRITE_LINE_MEMBER(nimbus_fdc_intrq_w);
	DECLARE_WRITE_LINE_MEMBER(nimbus_fdc_drq_w);
	DECLARE_WRITE8_MEMBER(nimbus_via_write_portb);
	DECLARE_WRITE_LINE_MEMBER(write_scsi_bsy);
	DECLARE_WRITE_LINE_MEMBER(write_scsi_cd);
	DECLARE_WRITE_LINE_MEMBER(write_scsi_io);
	DECLARE_WRITE_LINE_MEMBER(write_scsi_msg);
	DECLARE_WRITE_LINE_MEMBER(write_scsi_req);
	DECLARE_WRITE_LINE_MEMBER(nimbus_msm5205_vck);
	DECLARE_WRITE_LINE_MEMBER(write_scsi_iena);

	uint8_t get_pixel(uint16_t x, uint16_t y);
	uint16_t read_pixel_line(uint16_t x, uint16_t y, uint8_t pixels, uint8_t bpp);
	uint16_t read_pixel_data(uint16_t x, uint16_t y);
	void set_pixel(uint16_t x, uint16_t y, uint8_t colour);
	void set_pixel40(uint16_t x, uint16_t y, uint8_t colour);
	void write_pixel_line(uint16_t x, uint16_t y, uint16_t, uint8_t pixels, uint8_t bpp);
	void move_pixel_line(uint16_t x, uint16_t y, uint8_t width);
	void write_pixel_data(uint16_t x, uint16_t y, uint16_t    data);
	void change_palette(uint8_t bank, uint16_t colours);
	void external_int(uint8_t vector, bool state);
	DECLARE_READ8_MEMBER(cascade_callback);
	void nimbus_bank_memory();
	void memory_reset();
	void fdc_reset();
	uint8_t fdc_driveno(uint8_t drivesel);
	void hdc_reset();
	void hdc_post_rw();
	void hdc_drq(bool state);
	void pc8031_reset();
	//void ipc_dumpregs();
	void iou_reset();
	void rmni_sound_reset();
	void mouse_js_reset();
	void check_scsi_irq();

	int m_scsi_iena;
	int m_scsi_msg;
	int m_scsi_bsy;
	int m_scsi_io;
	int m_scsi_cd;
	int m_scsi_req;

	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

	enum
	{
		TIMER_MOUSE
	};

	// Static data related to Floppy and SCSI hard disks
	struct
	{
		uint8_t   reg400;
	} m_nimbus_drives;

	/* 8031 Peripheral controller */
	struct
	{
		uint8_t   ipc_in;
		uint8_t   ipc_out;
		uint8_t   status_in;
		uint8_t   status_out;
	} m_ipc_interface;

	/* Mouse/Joystick */
	struct
	{
		uint8_t   m_mouse_px;
		uint8_t   m_mouse_py;

		uint8_t   m_mouse_x;
		uint8_t   m_mouse_y;
		uint8_t   m_mouse_pc;
		uint8_t   m_mouse_pcx;
		uint8_t   m_mouse_pcy;

		uint8_t   m_intstate_x;
		uint8_t   m_intstate_y;

		uint8_t   m_reg0a4;

		emu_timer   *m_mouse_timer;
	} m_nimbus_mouse;

private:
	void debug_command(int ref, const std::vector<std::string> &params);
	void video_debug(int ref, const std::vector<std::string> &params);
};

#endif // MAME_INCLUDES_RMNIMBUS_H
