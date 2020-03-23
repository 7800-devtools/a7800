// license:BSD-3-Clause
// copyright-holders:Carl

#ifndef MAME_MACHINE_ISBC_208_H
#define MAME_MACHINE_ISBC_208_H

#pragma once

#include "machine/upd765.h"
#include "machine/am9517a.h"

class isbc_208_device : public device_t
{
public:
	isbc_208_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	DECLARE_ADDRESS_MAP(map, 8);
	DECLARE_READ8_MEMBER(stat_r);
	DECLARE_WRITE8_MEMBER(aux_w);

	template <class Object> static devcb_base &static_set_irq_callback(device_t &device, Object &&cb) { return downcast<isbc_208_device &>(device).m_out_irq_func.set_callback(std::forward<Object>(cb)); }
	static void static_set_maincpu_tag(device_t &device, const char *maincpu_tag) { downcast<isbc_208_device &>(device).m_maincpu_tag = maincpu_tag; }

protected:
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_add_mconfig(machine_config &config) override;

private:
	required_device<am9517a_device> m_dmac;
	required_device<i8272a_device> m_fdc;

	devcb_write_line m_out_irq_func;
	u8 m_aux;
	u16 m_seg;
	const char *m_maincpu_tag;
	address_space *m_maincpu_mem;

	DECLARE_WRITE_LINE_MEMBER(out_eop_w);
	DECLARE_WRITE_LINE_MEMBER(hreq_w);
	DECLARE_READ8_MEMBER(dma_read_byte);
	DECLARE_WRITE8_MEMBER(dma_write_byte);
	DECLARE_WRITE_LINE_MEMBER(irq_w);
	DECLARE_FLOPPY_FORMATS( floppy_formats );
};
#define MCFG_ISBC_208_MAINCPU(_maincpu_tag) \
	isbc_208_device::static_set_maincpu_tag(*device, _maincpu_tag);

#define MCFG_ISBC_208_IRQ(_irq_line) \
	devcb = &isbc_208_device::static_set_irq_callback(*device, DEVCB_##_irq_line);

DECLARE_DEVICE_TYPE(ISBC_208, isbc_208_device)

#endif // MAME_MACHINE_ISBC_208_H
