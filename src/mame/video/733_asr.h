// license:GPL-2.0+
// copyright-holders:Raphael Nabet
#ifndef MAME_VIDEO_733_ASR
#define MAME_VIDEO_733_ASR

#pragma once

#define asr733_chr_region ":gfx1"

class asr733_device : public device_t, public device_gfx_interface
{
public:
	enum
	{
		/* 8 bytes per character definition */
		single_char_len = 8,
		chr_region_len   = 128*single_char_len
	};

	asr733_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	DECLARE_READ8_MEMBER(cru_r);
	DECLARE_WRITE8_MEMBER(cru_w);

	template <class Object> static devcb_base &static_set_keyint_callback(device_t &device, Object &&cb)
	{
		return downcast<asr733_device &>(device).m_keyint_line.set_callback(std::forward<Object>(cb));
	}
	template <class Object> static devcb_base &static_set_lineint_callback(device_t &device, Object &&cb)
	{
		return downcast<asr733_device &>(device).m_lineint_line.set_callback(std::forward<Object>(cb));
	}

protected:
	// device-level overrides
	void device_start() override;
	void device_reset() override;
	virtual void device_add_mconfig(machine_config &config) override;
	void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;
	ioport_constructor device_input_ports() const override;

private:
	// internal state
#if 0
	uint8_t m_OutQueue[ASROutQueueSize];
	int m_OutQueueHead;
	int m_OutQueueLen;
#endif

	void check_keyboard();
	void refresh(bitmap_ind16 &bitmap, int x, int y);

	void set_interrupt_line();
	void draw_char(int character, int x, int y, int color);
	void linefeed();
	void transmit(uint8_t data);

	DECLARE_PALETTE_INIT(asr733);

	uint32_t screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);

	emu_timer *m_line_timer;                // screen line timer

	uint8_t   m_recv_buf;
	uint8_t   m_xmit_buf;

	uint8_t   m_status;
	uint8_t   m_mode;
	uint8_t   m_last_key_pressed;
	int     m_last_modifier_state;

	unsigned char m_repeat_timer;
	int     m_new_status_flag;

	int     m_x;

	std::unique_ptr<bitmap_ind16>       m_bitmap;

	devcb_write_line                   m_keyint_line;
	devcb_write_line                   m_lineint_line;
};

DECLARE_DEVICE_TYPE(ASR733, asr733_device)

#define MCFG_ASR733_KEYINT_HANDLER( _intcallb ) \
	devcb = &asr733_device::static_set_keyint_callback( *device, DEVCB_##_intcallb );

#define MCFG_ASR733_LINEINT_HANDLER( _intcallb ) \
	devcb = &asr733_device::static_set_lineint_callback( *device, DEVCB_##_intcallb );

#endif // MAME_VIDEO_733_ASR
