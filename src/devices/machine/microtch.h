// license:BSD-3-Clause
// copyright-holders:Mariusz Wojcieszek
#ifndef MAME_MACHINE_MICROTCH_H
#define MAME_MACHINE_MICROTCH_H

#pragma once


class microtouch_device :
		public device_t,
		public device_serial_interface
{
public:
	microtouch_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	template <class Object> static devcb_base &static_set_stx_callback(device_t &device, Object &&cb) { return downcast<microtouch_device &>(device).m_out_stx_func.set_callback(std::forward<Object>(cb)); }

	virtual ioport_constructor device_input_ports() const override;
	DECLARE_WRITE_LINE_MEMBER(rx) { device_serial_interface::rx_w(state); }
	DECLARE_INPUT_CHANGED_MEMBER(touch);

	typedef delegate<int (int *, int *)> touch_cb;
	static void static_set_touch_callback(device_t &device, touch_cb &&object) { downcast<microtouch_device &>(device).m_out_touch_cb = std::move(object); }

protected:
	virtual void device_start() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;
	virtual void tra_callback() override;
	virtual void tra_complete() override;
	virtual void rcv_complete() override;

private:
	int check_command( const char* commandtocheck, int command_len, uint8_t* command_data );
	void send_format_table_packet(uint8_t flag, int x, int y);
	void send_format_decimal_packet(int x, int y);
	void send_touch_packet();

	enum
	{
		FORMAT_UNKNOWN,
		FORMAT_TABLET,
		FORMAT_DECIMAL
	};

	enum
	{
		MODE_INACTIVE,
		MODE_STREAM,
		MODE_POINT
	};

	uint8_t       m_rx_buffer[16];
	int         m_rx_buffer_ptr;
	uint8_t       m_tx_buffer[16];
	uint8_t       m_tx_buffer_num;
	uint8_t       m_tx_buffer_ptr;
	int         m_reset_done;
	int         m_format;
	int         m_mode;
	int         m_last_touch_state;
	int         m_last_x;
	int         m_last_y;
	touch_cb m_out_touch_cb;
	devcb_write_line m_out_stx_func;
	required_ioport m_touch;
	required_ioport m_touchx;
	required_ioport m_touchy;
	emu_timer*  m_timer;
	bool m_output_valid;
	uint8_t m_output;
};

DECLARE_DEVICE_TYPE(MICROTOUCH, microtouch_device)

#define MCFG_MICROTOUCH_ADD(_tag, _clock, _devcb) \
	MCFG_DEVICE_ADD(_tag, MICROTOUCH, _clock) \
	devcb = &microtouch_device::static_set_stx_callback(*device, DEVCB_##_devcb);

#define MCFG_MICROTOUCH_TOUCH_CB(_class, _touch_cb) \
	microtouch_device::static_set_touch_callback(*device, microtouch_device::touch_cb(&_class::_touch_cb, (_class *)owner));

#endif // MAME_MACHINE_MICROTCH_H
