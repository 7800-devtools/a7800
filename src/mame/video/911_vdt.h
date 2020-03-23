// license:GPL-2.0+
// copyright-holders:Raphael Nabet
#ifndef MAME_VIDEO_911_VDT_H
#define MAME_VIDEO_911_VDT_H

#pragma once

#include "sound/beep.h"

#define vdt911_chr_region ":gfx1"

class vdt911_device : public device_t, public device_gfx_interface
{
public:
	enum
	{
		/* 10 bytes per character definition */
		single_char_len = 10,

		US_chr_offset        = 0,
		UK_chr_offset        = US_chr_offset+128*single_char_len,
		german_chr_offset    = UK_chr_offset+128*single_char_len,
		swedish_chr_offset   = german_chr_offset+128*single_char_len,
		norwegian_chr_offset = swedish_chr_offset+128*single_char_len,
		frenchWP_chr_offset  = norwegian_chr_offset+128*single_char_len,
		japanese_chr_offset  = frenchWP_chr_offset+128*single_char_len,

		chr_region_len   = japanese_chr_offset+256*single_char_len
	};

	enum class screen_size { char_960 = 0, char_1920 };

	enum class model
	{
		US = 0,
		UK,
		French,
		German,
		Swedish,      // Swedish/Finnish
		Norwegian,    // Norwegian/Danish
		Japanese,     // Katakana Japanese
		/*Arabic,*/   // Arabic
		FrenchWP      // French word processing
	};

	vdt911_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	DECLARE_READ8_MEMBER(cru_r);
	DECLARE_WRITE8_MEMBER(cru_w);

	template <class Object> static devcb_base &static_set_keyint_callback(device_t &device, Object &&cb)
	{
		return downcast<vdt911_device &>(device).m_keyint_line.set_callback(std::forward<Object>(cb));
	}

	template <class Object> static devcb_base &static_set_lineint_callback(device_t &device, Object &&cb)
	{
		return downcast<vdt911_device &>(device).m_lineint_line.set_callback(std::forward<Object>(cb));
	}

protected:
	// device-level overrides
	void device_start() override;
	void device_reset() override;
	virtual void device_add_mconfig(machine_config &config) override;
	ioport_constructor device_input_ports() const override;

	void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

private:
	void refresh(bitmap_ind16 &bitmap, const rectangle &cliprect, int x, int y);
	int get_refresh_rate();
	void check_keyboard();

	DECLARE_PALETTE_INIT(vdt911);

	uint32_t screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);

	screen_size    m_screen_size;  // char_960 for 960-char, 12-line model; char_1920 for 1920-char, 24-line model
	model          m_model;        // country code

	uint8_t m_data_reg;                       // dt911 write buffer
	uint8_t m_display_RAM[2048];              // vdt911 char buffer (1kbyte for 960-char model, 2kbytes for 1920-char model)

	unsigned int m_cursor_address;          // current cursor address (controlled by the computer, affects both display and I/O protocol)
	unsigned int m_cursor_address_mask; // 1023 for 960-char model, 2047 for 1920-char model

	emu_timer *m_beep_timer;                // beep clock (beeps ends when timer times out)
	emu_timer *m_blink_timer;               // cursor blink clock
	emu_timer *m_line_timer;                // screen line timer

	uint8_t m_keyboard_data;                  // last code pressed on keyboard
	bool m_keyboard_data_ready;             // true if there is a new code in keyboard_data
	bool m_keyboard_interrupt_enable;       // true when keyboard interrupts are enabled

	bool m_display_enable;                  // screen is black when false
	bool m_dual_intensity_enable;           // if true, MSBit of ASCII codes controls character highlight
	bool m_display_cursor;                  // if true, the current cursor location is displayed on screen
	bool m_blinking_cursor_enable;          // if true, the cursor will blink when displayed
	bool m_blink_state;                     // current cursor blink state

	bool m_word_select;                     // CRU interface mode
	bool m_previous_word_select;            // value of word_select is saved here

	uint8_t m_last_key_pressed;
	int m_last_modifier_state;
	char m_foreign_mode;

	required_device<beep_device>        m_beeper;
	devcb_write_line                   m_keyint_line;
	devcb_write_line                   m_lineint_line;
};

DECLARE_DEVICE_TYPE(VDT911, vdt911_device)

#define MCFG_VDT911_KEYINT_HANDLER( _intcallb ) \
	devcb = &vdt911_device::static_set_keyint_callback( *device, DEVCB_##_intcallb );

#define MCFG_VDT911_LINEINT_HANDLER( _intcallb ) \
	devcb = &vdt911_device::static_set_lineint_callback( *device, DEVCB_##_intcallb );

#endif // MAME_VIDEO_911_VDT_H
