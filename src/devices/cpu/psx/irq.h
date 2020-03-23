// license:BSD-3-Clause
// copyright-holders:smf
/*
 * PlayStation IRQ emulator
 *
 * Copyright 2003-2011 smf
 *
 */

#ifndef MAME_CPU_PSX_IRQ_H
#define MAME_CPU_PSX_IRQ_H

#pragma once


extern const device_type PSX_IRQ;

#define MCFG_PSX_IRQ_HANDLER(_devcb) \
	devcb = &psxirq_device::set_irq_handler(*device, DEVCB_##_devcb);

class psxirq_device : public device_t
{
public:
	psxirq_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// static configuration helpers
	template <class Object> static devcb_base &set_irq_handler(device_t &device, Object &&cb) { return downcast<psxirq_device &>(device).m_irq_handler.set_callback(std::forward<Object>(cb)); }

	DECLARE_READ32_MEMBER( read );
	DECLARE_WRITE32_MEMBER( write );

	DECLARE_WRITE_LINE_MEMBER( intin0 );
	DECLARE_WRITE_LINE_MEMBER( intin1 );
	DECLARE_WRITE_LINE_MEMBER( intin2 );
	DECLARE_WRITE_LINE_MEMBER( intin3 );
	DECLARE_WRITE_LINE_MEMBER( intin4 );
	DECLARE_WRITE_LINE_MEMBER( intin5 );
	DECLARE_WRITE_LINE_MEMBER( intin6 );
	DECLARE_WRITE_LINE_MEMBER( intin7 );
	DECLARE_WRITE_LINE_MEMBER( intin8 );
	DECLARE_WRITE_LINE_MEMBER( intin9 );
	DECLARE_WRITE_LINE_MEMBER( intin10 );

protected:
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_post_load() override;

private:
	void psx_irq_update( void );
	void set( uint32_t bitmask );

	uint32_t n_irqdata;
	uint32_t n_irqmask;

	devcb_write_line m_irq_handler;
};

#endif // MAME_CPU_PSX_IRQ_H
