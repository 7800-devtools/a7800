// license:BSD-3-Clause
// copyright-holders:Sandro Ronco
/**********************************************************************

    NEC uPD65031 'BLINK' emulation

**********************************************************************/

#ifndef MAME_MACHINE_UPD65031_H
#define MAME_MACHINE_UPD65031_H

#pragma once



//**************************************************************************
//  INTERFACE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_UPD65031_KB_CALLBACK(_read) \
	devcb = &upd65031_device::set_kb_rd_callback(*device, DEVCB_##_read);

#define MCFG_UPD65031_INT_CALLBACK(_write) \
	devcb = &upd65031_device::set_int_wr_callback(*device, DEVCB_##_write);

#define MCFG_UPD65031_NMI_CALLBACK(_write) \
	devcb = &upd65031_device::set_nmi_wr_callback(*device, DEVCB_##_write);

#define MCFG_UPD65031_SPKR_CALLBACK(_write) \
	devcb = &upd65031_device::set_spkr_wr_callback(*device, DEVCB_##_write);

#define MCFG_UPD65031_SCR_UPDATE_CB(_class, _method) \
	upd65031_device::set_screen_update_callback(*device, upd65031_screen_update_delegate(&_class::_method, #_class "::" #_method, downcast<_class *>(owner)));

#define MCFG_UPD65031_MEM_UPDATE_CB(_class, _method) \
	upd65031_device::set_memory_update_callback(*device, upd65031_memory_update_delegate(&_class::_method, #_class "::" #_method, downcast<_class *>(owner)));


//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

typedef device_delegate<void (bitmap_ind16 &bitmap, uint16_t sbf, uint16_t hires0, uint16_t hires1, uint16_t lores0, uint16_t lores1, int flash)> upd65031_screen_update_delegate;
typedef device_delegate<void (int bank, uint16_t page, int rams)> upd65031_memory_update_delegate;

#define UPD65031_SCREEN_UPDATE(_name) void _name(bitmap_ind16 &bitmap, uint16_t sbf, uint16_t hires0, uint16_t hires1, uint16_t lores0, uint16_t lores1, int flash)
#define UPD65031_MEMORY_UPDATE(_name) void _name(int bank, uint16_t page, int rams)


// ======================> upd65031_device

class upd65031_device : public device_t
{
public:
	// construction/destruction
	upd65031_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	template<class _Object> static devcb_base &set_kb_rd_callback(device_t &device, _Object object) { return downcast<upd65031_device &>(device).m_read_kb.set_callback(object); }
	template<class _Object> static devcb_base &set_int_wr_callback(device_t &device, _Object object) { return downcast<upd65031_device &>(device).m_write_int.set_callback(object); }
	template<class _Object> static devcb_base &set_nmi_wr_callback(device_t &device, _Object object) { return downcast<upd65031_device &>(device).m_write_nmi.set_callback(object); }
	template<class _Object> static devcb_base &set_spkr_wr_callback(device_t &device, _Object object) { return downcast<upd65031_device &>(device).m_write_spkr.set_callback(object); }

	static void set_screen_update_callback(device_t &device, upd65031_screen_update_delegate callback) { downcast<upd65031_device &>(device).m_screen_update_cb = callback; }
	static void set_memory_update_callback(device_t &device, upd65031_memory_update_delegate callback) { downcast<upd65031_device &>(device).m_out_mem_cb = callback; }

	DECLARE_READ8_MEMBER( read );
	DECLARE_WRITE8_MEMBER( write );
	DECLARE_WRITE_LINE_MEMBER( flp_w );
	DECLARE_WRITE_LINE_MEMBER( btl_w );
	uint32_t screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

private:
	inline void interrupt_refresh();
	inline void update_rtc_interrupt();
	inline void set_mode(int mode);
	static const device_timer_id TIMER_RTC = 0;
	static const device_timer_id TIMER_FLASH = 1;
	static const device_timer_id TIMER_SPEAKER = 2;

	devcb_read8        m_read_kb;
	devcb_write_line   m_write_int;
	devcb_write_line   m_write_nmi;
	devcb_write_line   m_write_spkr;

	upd65031_screen_update_delegate m_screen_update_cb;  // callback for update the LCD
	upd65031_memory_update_delegate m_out_mem_cb;        // callback for update bankswitch

	int     m_mode;
	uint16_t  m_lcd_regs[5];      // LCD registers
	uint8_t   m_tim[5];           // RTC registers
	uint8_t   m_sr[4];            // segment registers
	uint8_t   m_sta;              // interrupt status
	uint8_t   m_int;              // interrupts mask
	uint8_t   m_ack;              // interrupts acknowledge
	uint8_t   m_tsta;             // timer interrupt status
	uint8_t   m_tmk;              // timer interrupt mask
	uint8_t   m_tack;             // timer interrupts acknowledge
	uint8_t   m_com;              // command register
	int     m_flash;            // cursor flash
	int     m_speaker_state;    // spkr line

	// timers
	emu_timer *m_rtc_timer;
	emu_timer *m_flash_timer;
	emu_timer *m_speaker_timer;
};


// device type definition
DECLARE_DEVICE_TYPE(UPD65031, upd65031_device)


#endif // MAME_MACHINE_UPD65031_H
