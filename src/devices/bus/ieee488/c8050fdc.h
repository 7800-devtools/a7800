// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    Commodore 8050 floppy disk controller emulation

**********************************************************************/

#ifndef MAME_BUS_IEEE488_C8050FDC_H
#define MAME_BUS_IEEE488_C8050FDC_H

#pragma once

#include "formats/d80_dsk.h"
#include "formats/d82_dsk.h"
#include "imagedev/floppy.h"
#include "machine/fdc_pll.h"



//**************************************************************************
//  INTERFACE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_C8050_SYNC_CALLBACK(_write) \
	devcb = &c8050_fdc_device::set_sync_wr_callback(*device, DEVCB_##_write);

#define MCFG_C8050_READY_CALLBACK(_write) \
	devcb = &c8050_fdc_device::set_ready_wr_callback(*device, DEVCB_##_write);

#define MCFG_C8050_BRDY_CALLBACK(_write) \
	devcb = &c8050_fdc_device::set_brdy_wr_callback(*device, DEVCB_##_write);

#define MCFG_C8050_ERROR_CALLBACK(_write) \
	devcb = &c8050_fdc_device::set_error_wr_callback(*device, DEVCB_##_write);



//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> c8050_fdc_device

class c8050_fdc_device :  public device_t
{
public:
	// construction/destruction
	c8050_fdc_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	template <class Object> static devcb_base &set_sync_wr_callback(device_t &device, Object &&cb) { return downcast<c8050_fdc_device &>(device).m_write_sync.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_ready_wr_callback(device_t &device, Object &&cb) { return downcast<c8050_fdc_device &>(device).m_write_ready.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_brdy_wr_callback(device_t &device, Object &&cb) { return downcast<c8050_fdc_device &>(device).m_write_brdy.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_error_wr_callback(device_t &device, Object &&cb) { return downcast<c8050_fdc_device &>(device).m_write_error.set_callback(std::forward<Object>(cb)); }

	DECLARE_READ8_MEMBER( read );
	DECLARE_WRITE8_MEMBER( write );

	DECLARE_WRITE_LINE_MEMBER( ds0_w );
	DECLARE_WRITE_LINE_MEMBER( ds1_w );
	DECLARE_WRITE_LINE_MEMBER( drv_sel_w );
	DECLARE_WRITE_LINE_MEMBER( mode_sel_w );
	DECLARE_WRITE_LINE_MEMBER( rw_sel_w );
	DECLARE_WRITE_LINE_MEMBER( mtr0_w );
	DECLARE_WRITE_LINE_MEMBER( mtr1_w );
	DECLARE_WRITE_LINE_MEMBER( odd_hd_w );
	DECLARE_WRITE_LINE_MEMBER( pull_sync_w );

	DECLARE_READ_LINE_MEMBER( wps_r ) { return checkpoint_live.drv_sel ? m_floppy1->wpt_r() : m_floppy0->wpt_r(); }

	void stp0_w(int stp);
	void stp1_w(int stp);
	void ds_w(int ds);

	void set_floppy(floppy_connector *floppy0, floppy_connector *floppy1);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

	// optional information overrides
	virtual const tiny_rom_entry *device_rom_region() const override;

	void stp_w(floppy_image_device *floppy, int mtr, int &old_stp, int stp);

	enum {
		IDLE,
		RUNNING,
		RUNNING_SYNCPOINT
	};

	struct live_info {
		attotime tm;
		int state, next_state;
		int sync;
		int ready;
		int brdy;
		int error;
		int ds;
		int drv_sel;
		int mode_sel;
		int rw_sel;
		int odd_hd;

		attotime edge;
		uint16_t shift_reg;
		int bit_counter;
		uint8_t e;
		offs_t i;

		uint8_t pi;
		uint16_t shift_reg_write;
	};

	devcb_write_line m_write_sync;
	devcb_write_line m_write_ready;
	devcb_write_line m_write_brdy;
	devcb_write_line m_write_error;

	required_memory_region m_gcr_rom;

	floppy_image_device *m_floppy0;
	floppy_image_device *m_floppy1;

	int m_mtr0;
	int m_mtr1;
	int m_stp0;
	int m_stp1;
	int m_ds;
	int m_ds0;
	int m_ds1;
	int m_drv_sel;
	int m_mode_sel;
	int m_rw_sel;
	int m_odd_hd;
	uint8_t m_pi;

	live_info cur_live, checkpoint_live;
	fdc_pll_t cur_pll, checkpoint_pll;
	emu_timer *t_gen;

	floppy_image_device* get_floppy();

	void live_start();
	void checkpoint();
	void rollback();
	void pll_reset(const attotime &when);
	void pll_start_writing(const attotime &tm);
	void pll_commit(floppy_image_device *floppy, const attotime &tm);
	void pll_stop_writing(floppy_image_device *floppy, const attotime &tm);
	int pll_get_next_bit(attotime &tm, floppy_image_device *floppy, const attotime &limit);
	bool pll_write_next_bit(bool bit, attotime &tm, floppy_image_device *floppy, const attotime &limit);
	void pll_save_checkpoint();
	void pll_retrieve_checkpoint();
	void live_delay(int state);
	void live_sync();
	void live_abort();
	void live_run(const attotime &limit = attotime::never);
};


// device type definition
DECLARE_DEVICE_TYPE(C8050_FDC, c8050_fdc_device)

#endif // MAME_BUS_IEEE488_C8050FDC_H
