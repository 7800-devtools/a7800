// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/**********************************************************************

    PC-style floppy disk controller emulation

**********************************************************************/

#ifndef MAME_MACHINE_PC_FDC_H
#define MAME_MACHINE_PC_FDC_H

#pragma once

#include "machine/upd765.h"

#define MCFG_PC_FDC_XT_ADD(_tag) \
	MCFG_DEVICE_ADD(_tag, PC_FDC_XT, 0)

#define MCFG_PC_FDC_AT_ADD(_tag) \
	MCFG_DEVICE_ADD(_tag, PC_FDC_AT, 0)

#define MCFG_PC_FDC_INTRQ_CALLBACK(_write) \
	devcb = &pc_fdc_family_device::set_intrq_wr_callback(*device, DEVCB_##_write);

#define MCFG_PC_FDC_DRQ_CALLBACK(_write) \
	devcb = &pc_fdc_family_device::set_drq_wr_callback(*device, DEVCB_##_write);

class pc_fdc_family_device : public pc_fdc_interface {
public:
	template <class Object> static devcb_base &set_intrq_wr_callback(device_t &device, Object &&cb) { return downcast<pc_fdc_family_device &>(device).intrq_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_drq_wr_callback(device_t &device, Object &&cb) { return downcast<pc_fdc_family_device &>(device).drq_cb.set_callback(std::forward<Object>(cb)); }

	virtual DECLARE_ADDRESS_MAP(map, 8) override;

	virtual void tc_w(bool state) override;
	virtual uint8_t dma_r() override;
	virtual void dma_w(uint8_t data) override;
	virtual uint8_t do_dir_r() override;

	READ8_MEMBER(dor_r);
	WRITE8_MEMBER(dor_w);
	READ8_MEMBER(dir_r);
	WRITE8_MEMBER(ccr_w);

	required_device<upd765a_device> fdc;

protected:
	pc_fdc_family_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_add_mconfig(machine_config &config) override;

private:
	DECLARE_WRITE_LINE_MEMBER( irq_w );
	DECLARE_WRITE_LINE_MEMBER( drq_w );

	bool irq, drq, fdc_drq, fdc_irq;
	devcb_write_line intrq_cb, drq_cb;
	uint8_t dor;

	floppy_image_device *floppy[4];

	void check_irq();
	void check_drq();
};

class pc_fdc_xt_device : public pc_fdc_family_device {
public:
	pc_fdc_xt_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual DECLARE_ADDRESS_MAP(map, 8) override;
	WRITE8_MEMBER(dor_fifo_w);
};

class pc_fdc_at_device : public pc_fdc_family_device {
public:
	pc_fdc_at_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual DECLARE_ADDRESS_MAP(map, 8) override;
};

DECLARE_DEVICE_TYPE(PC_FDC_XT, pc_fdc_xt_device)
DECLARE_DEVICE_TYPE(PC_FDC_AT, pc_fdc_at_device)

#endif // MAME_MACHINE_PC_FDC_H
