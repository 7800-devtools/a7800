// license:BSD-3-Clause
// copyright-holders:Peter Trauner, Kevin Thacker
/**********************************************************************

    pckeybrd.h

    PC-style keyboard emulation

    This emulation is decoupled from the AT 8042 emulation used in the
    IBM ATs and above

**********************************************************************/

#ifndef MAME_MACHINE_PCKEYBRD_H
#define MAME_MACHINE_PCKEYBRD_H

#pragma once


class pc_keyboard_device : public device_t
{
public:
	pc_keyboard_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	DECLARE_READ8_MEMBER(read);
	DECLARE_WRITE_LINE_MEMBER(enable);

	template <class Object> static devcb_base &static_set_keypress_callback(device_t &device, Object &&cb)
	{ return downcast<pc_keyboard_device &>(device).m_out_keypress_func.set_callback(std::forward<Object>(cb)); }

	enum class KEYBOARD_TYPE
	{
		PC,
		AT,
		MF2
	};

protected:
	pc_keyboard_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;
	void queue_insert(uint8_t data);
	void clear_buffer();

	int m_numlock;
	KEYBOARD_TYPE m_type;

private:
	void polling();
	uint32_t readport(int port);
	uint8_t unicode_char_to_at_keycode(char32_t ch);

	virtual void standard_scancode_insert(int our_code, int pressed);
	virtual void extended_scancode_insert(int code, int pressed) { }
	int queue_size();
	int queue_chars(const char32_t *text, size_t text_len);
	bool accept_char(char32_t ch);
	bool charqueue_empty();

	bool m_on;
	uint8_t m_delay;   /* 240/60 -> 0,25s */
	uint8_t m_repeat;   /* 240/ 8 -> 30/s */

	uint8_t m_queue[256];
	uint8_t m_head;
	uint8_t m_tail;
	uint8_t m_make[128];

	optional_ioport_array<8> m_ioport;

	devcb_write_line m_out_keypress_func;
	emu_timer *m_keyboard_timer;
};

class at_keyboard_device : public pc_keyboard_device
{
public:
	at_keyboard_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	DECLARE_WRITE8_MEMBER( write );

	static void static_set_type(device_t &device, KEYBOARD_TYPE type, int default_set)
	{ downcast<at_keyboard_device &>(device).m_scan_code_set = default_set; downcast<at_keyboard_device &>(device).m_type = type; }

protected:
	virtual void device_reset() override;
	virtual void device_start() override;

private:
	virtual void standard_scancode_insert(int our_code, int pressed) override;
	virtual void extended_scancode_insert(int code, int pressed) override;
	void helper(const char *codes);
	void clear_buffer_and_acknowledge();

	struct extended_keyboard_code
	{
		const char *pressed;
		const char *released;
	};

	static const extended_keyboard_code m_extended_codes_set_2_3[];
	static const int m_scancode_set_2_3[];
	static const extended_keyboard_code m_mf2_code[0x10][2];

	int m_scan_code_set;
	int m_input_state;
};

INPUT_PORTS_EXTERN( pc_keyboard );
INPUT_PORTS_EXTERN( at_keyboard );

DECLARE_DEVICE_TYPE(PC_KEYB, pc_keyboard_device)
DECLARE_DEVICE_TYPE(AT_KEYB, at_keyboard_device)

#define MCFG_PC_KEYB_ADD(_tag, _cb) \
	MCFG_DEVICE_ADD(_tag, PC_KEYB, 0) \
	devcb = &pc_keyboard_device::static_set_keypress_callback(*device, DEVCB_##_cb);

#define MCFG_AT_KEYB_ADD(_tag, _def_set, _cb) \
	MCFG_DEVICE_ADD(_tag, AT_KEYB, 0) \
	at_keyboard_device::static_set_type(*device, pc_keyboard_device::KEYBOARD_TYPE::AT, _def_set); \
	devcb = &pc_keyboard_device::static_set_keypress_callback(*device, DEVCB_##_cb);

#define MCFG_AT_MF2_KEYB_ADD(_tag, _def_set, _cb) \
	MCFG_DEVICE_ADD(_tag, AT_KEYB, 0) \
	at_keyboard_device::static_set_type(*device, pc_keyboard_device::KEYBOARD_TYPE_MF2, _def_set); \
	devcb = &pc_keyboard_device::static_set_keypress_callback(*device, DEVCB_##_cb);

#endif // MAME_MACHINE_PCKEYBRD_H
