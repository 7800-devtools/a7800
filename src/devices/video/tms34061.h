// license:BSD-3-Clause
// copyright-holders:Zsolt Vasvari, Aaron Giles
/****************************************************************************
 *                                                                          *
 *  Function prototypes and constants used by the TMS34061 emulator         *
 *                                                                          *
 *  Created by Zsolt Vasvari on 5/26/1998.                                  *
 *  Updated by Aaron Giles on 11/21/2000.                                   *
 *                                                                          *
 ****************************************************************************/

#ifndef MAME_VIDEO_TMS34061_H
#define MAME_VIDEO_TMS34061_H

#pragma once


#define MCFG_TMS34061_ROWSHIFT(_shift) \
	tms34061_device::set_rowshift(*device, _shift);

#define MCFG_TMS34061_VRAM_SIZE(_size) \
	tms34061_device::set_vram_size(*device, _size);

#define MCFG_TMS34061_INTERRUPT_CB(_devcb) \
	devcb = &tms34061_device::set_interrupt_callback(*device, DEVCB_##_devcb);



// ======================> tms34061_device

class tms34061_device :  public device_t, public device_video_interface
{
public:
	/* display state structure */
	struct tms34061_display
	{
		uint8_t   blanked;        /* true if blanked */
		uint8_t   *vram;          /* base of VRAM */
		uint8_t   *latchram;      /* base of latch RAM */
		uint16_t  *regs;          /* pointer to array of registers */
		offs_t  dispstart;      /* display start */
	};

	// construction/destruction
	tms34061_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	static void set_rowshift(device_t &device, uint8_t rowshift) { downcast<tms34061_device &>(device).m_rowshift = rowshift; }
	static void set_vram_size(device_t &device, uint32_t vramsize) { downcast<tms34061_device &>(device).m_vramsize = vramsize; }
	template <class Object> static devcb_base &set_interrupt_callback(device_t &device, Object &&cb) { return downcast<tms34061_device &>(device).m_interrupt_cb.set_callback(std::forward<Object>(cb)); }

	/* reads/writes to the 34061 */
	uint8_t read(address_space &space, int col, int row, int func);
	void write(address_space &space, int col, int row, int func, uint8_t data);

	/* latch settings */
	DECLARE_READ8_MEMBER( latch_r );
	DECLARE_WRITE8_MEMBER( latch_w );

	/* video update handling */
	void get_display_state();

	bool blanked() const { return bool(m_display.blanked); }
	uint8_t const &vram(unsigned row) const { return m_display.vram[row << m_rowshift]; }
	uint16_t xyoffset() const { return m_display.regs[TMS34061_XYOFFSET]; }
	uint16_t xyaddress() const { return m_display.regs[TMS34061_XYADDRESS]; }

	// TODO: encapsulate this properly
	tms34061_display m_display;

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

private:
	/* register constants */
	enum
	{
		TMS34061_HORENDSYNC = 0,
		TMS34061_HORENDBLNK,
		TMS34061_HORSTARTBLNK,
		TMS34061_HORTOTAL,
		TMS34061_VERENDSYNC,
		TMS34061_VERENDBLNK,
		TMS34061_VERSTARTBLNK,
		TMS34061_VERTOTAL,
		TMS34061_DISPUPDATE,
		TMS34061_DISPSTART,
		TMS34061_VERINT,
		TMS34061_CONTROL1,
		TMS34061_CONTROL2,
		TMS34061_STATUS,
		TMS34061_XYOFFSET,
		TMS34061_XYADDRESS,
		TMS34061_DISPADDRESS,
		TMS34061_VERCOUNTER,
		TMS34061_REGCOUNT
	};

	uint8_t               m_rowshift;         /* VRAM address is (row << rowshift) | col */
	uint32_t              m_vramsize;         /* size of video RAM */
	devcb_write_line   m_interrupt_cb;     /* interrupt gen callback */

	uint16_t              m_regs[TMS34061_REGCOUNT];
	uint16_t              m_xmask;
	uint8_t               m_yshift;
	uint32_t              m_vrammask;
	uint8_t *             m_vram;
	uint8_t *             m_latchram;
	uint8_t               m_latchdata;
	uint8_t *             m_shiftreg;
	emu_timer *         m_timer;

	void update_interrupts();
	TIMER_CALLBACK_MEMBER( interrupt );
	void register_w(address_space &space, offs_t offset, uint8_t data);
	uint8_t register_r(address_space &space, offs_t offset);
	void adjust_xyaddress(int offset);
	void xypixel_w(address_space &space, int offset, uint8_t data);
	uint8_t xypixel_r(address_space &space, int offset);
};

// device type definition
DECLARE_DEVICE_TYPE(TMS34061, tms34061_device)

#endif // MAME_VIDEO_TMS34061_H
