// license:GPL-2.0+
// copyright-holders:Dirk Best
/***************************************************************************

    Intel 82730

    Text Coprocessor

***************************************************************************/

#ifndef MAME_VIDEO_I82730_H
#define MAME_VIDEO_I82730_H

#pragma once



//**************************************************************************
//  INTERFACE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_I82730_ADD(_tag, _cpu_tag, _clock) \
	MCFG_DEVICE_ADD(_tag, I82730, _clock) \
	i82730_device::set_cpu_tag(*device, owner, _cpu_tag);

#define MCFG_I82730_SINT_HANDLER(_devcb) \
	devcb = &i82730_device::set_sint_handler(*device, DEVCB_##_devcb);

#define MCFG_I82730_UPDATE_ROW_CB(_class, _method) \
	i82730_device::set_update_row_callback(*device, i82730_device::update_row_delegate(&_class::_method, #_class "::" #_method, downcast<_class *>(owner)));


//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

#define I82730_UPDATE_ROW(name) \
	void name(bitmap_rgb32 &bitmap, uint16_t *data, uint8_t lc, uint16_t y, int x_count)


// ======================> i82730_device

class i82730_device : public device_t, public device_video_interface
{
public:
	typedef device_delegate<void (bitmap_rgb32 &bitmap, uint16_t *data, uint8_t lc, uint16_t y, int x_count)> update_row_delegate;

	// construction/destruction
	i82730_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// callbacks
	template <class Object> static devcb_base &set_sint_handler(device_t &device, Object &&cb)
	{ return downcast<i82730_device &>(device).m_sint_handler.set_callback(std::forward<Object>(cb)); }

	// inline configuration
	static void set_cpu_tag(device_t &device, device_t *owner, const char *tag);
	static void set_update_row_callback(device_t &device, update_row_delegate &&cb) { downcast<i82730_device &>(device).m_update_row_cb = std::move(cb); }

	uint32_t screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);

	DECLARE_WRITE_LINE_MEMBER(ca_w);
	DECLARE_WRITE_LINE_MEMBER(irst_w);

protected:
	virtual void device_start() override;
	virtual void device_reset() override;

private:
	// status
	enum
	{
		DUR  = 0x001,  // data underrun
		LPU  = 0x002,  // light pen update
		DBOR = 0x004,  // data buffer overrun
		EONF = 0x008,  // end of n frames
		FDE  = 0x010,  // frame data error
		RCC  = 0x020,  // reserved channel command executed
		RDC  = 0x040,  // reserved data stream command executed
		DIP  = 0x080,  // display in progress
		VDIP = 0x100   // virtual display in progress
	};

	static const char *const s_command_names[];

	bool sysbus_16bit() { return BIT(m_sysbus, 0); }

	uint8_t read_byte(offs_t address);
	uint16_t read_word(offs_t address);
	void write_byte(offs_t address, uint8_t data);
	void write_word(offs_t address, uint16_t data);

	void update_interrupts();
	void mode_set();
	void execute_command();
	void load_row();

	TIMER_CALLBACK_MEMBER(row_update);

	devcb_write_line m_sint_handler;
	update_row_delegate m_update_row_cb;

	const char *m_cpu_tag;
	address_space *m_program;

	emu_timer *m_row_timer;

	bitmap_rgb32 m_bitmap;

	bool m_initialized;
	bool m_mode_set;

	int m_ca;

	// internal registers
	uint8_t m_sysbus;
	uint32_t m_ibp;
	uint32_t m_cbp;
	uint16_t m_intmask;
	uint16_t m_status;

	int m_list_switch;
	int m_auto_line_feed;
	uint8_t m_max_dma_count;

	uint32_t m_lptr;
	uint32_t m_sptr;

	int m_dma_burst_space;
	int m_dma_burst_length;

	// display parameters
	int m_hfldstrt;
	int m_margin;
	int m_lpr;
	uint16_t m_field_attribute_mask;
	int m_vsyncstp;
	int m_vfldstrt;
	int m_vfldstp;

	int m_frame_int_count;

	// row buffers
	struct row_buffer
	{
		uint16_t data[200];
		int count;
	};

	row_buffer m_row[2];
	int m_row_index;
};

// device type definition
DECLARE_DEVICE_TYPE(I82730, i82730_device)

#endif // MAME_VIDEO_I82730_H
