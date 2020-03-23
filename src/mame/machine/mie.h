// license:BSD-3-Clause
// copyright-holders:Olivier Galibert
#ifndef MAME_MACHINE_MIE_H
#define MAME_MACHINE_MIE_H

#pragma once

#include "machine/mapledev.h"

#include "cpu/z80/z80.h"
#include "machine/eepromser.h"
#include "machine/jvshost.h"

#define MCFG_MIE_ADD(_tag, _clock, _host_tag, _host_port, g0, g1, g2, g3, g4, g5, g6, g7) \
	MCFG_MAPLE_DEVICE_ADD(_tag "_maple", MIE, _clock, _host_tag, _host_port) \
	mie_device::static_set_gpio_name(*device, 0, g0);   \
	mie_device::static_set_gpio_name(*device, 1, g1);   \
	mie_device::static_set_gpio_name(*device, 2, g2);   \
	mie_device::static_set_gpio_name(*device, 3, g3);   \
	mie_device::static_set_gpio_name(*device, 4, g4);   \
	mie_device::static_set_gpio_name(*device, 5, g5);   \
	mie_device::static_set_gpio_name(*device, 6, g6);   \
	mie_device::static_set_gpio_name(*device, 7, g7); \
	mie_device::static_set_jvs_name(*device, _tag); \
	MCFG_DEVICE_ADD(_tag, MIE_JVS, _clock)

class mie_jvs_device;

// ======================> mie_device

class mie_device : public maple_device
{
public:
	mie_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	static void static_set_gpio_name(device_t &device, int entry, const char *name);
	static void static_set_jvs_name(device_t &device, const char *name);

	DECLARE_READ8_MEMBER(control_r);
	DECLARE_WRITE8_MEMBER(control_w);
	DECLARE_READ8_MEMBER(lreg_r);
	DECLARE_WRITE8_MEMBER(lreg_w);
	DECLARE_READ8_MEMBER(tbuf_r);
	DECLARE_WRITE8_MEMBER(tbuf_w);
	DECLARE_READ8_MEMBER(gpio_r);
	DECLARE_WRITE8_MEMBER(gpio_w);
	DECLARE_READ8_MEMBER(gpiodir_r);
	DECLARE_WRITE8_MEMBER(gpiodir_w);
	DECLARE_READ8_MEMBER(adc_r);
	DECLARE_WRITE8_MEMBER(adc_w);

	DECLARE_READ8_MEMBER(irq_enable_r);
	DECLARE_WRITE8_MEMBER(irq_enable_w);
	DECLARE_READ8_MEMBER(maple_irqlevel_r);
	DECLARE_WRITE8_MEMBER(maple_irqlevel_w);
	DECLARE_READ8_MEMBER(irq_pending_r);
	DECLARE_WRITE8_MEMBER(irq_pending_w);

	DECLARE_READ8_MEMBER(jvs_r);
	DECLARE_WRITE8_MEMBER(jvs_w);
	DECLARE_WRITE8_MEMBER(jvs_dest_w);
	DECLARE_READ8_MEMBER(jvs_status_r);
	DECLARE_WRITE8_MEMBER(jvs_control_w);
	DECLARE_READ8_MEMBER(jvs_sense_r);
	DECLARE_WRITE8_MEMBER(jvs_lcr_w);

	DECLARE_READ8_MEMBER(read_ff);
	DECLARE_READ8_MEMBER(read_00);
	DECLARE_READ8_MEMBER(read_78xx);

	void maple_w(const uint32_t *data, uint32_t in_size) override;
	virtual void maple_reset() override;

protected:
	const char *gpio_name[8];
	const char *jvs_name;

	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

	// optional information overrides
	virtual const tiny_rom_entry *device_rom_region() const override;
	virtual void device_add_mconfig(machine_config &config) override;

private:
	enum { TBUF_SIZE = 8 };

	enum {
		CTRL_TXB  = 0x000001, // Start transmit
		CTRL_CTXB = 0x000002, // Continue transmit flag (when doing multipart)
		CTRL_ENDP = 0x000004, // Add an end pattern
		CTRL_EMP  = 0x000100, // Empty flag
		CTRL_PERR = 0x000200, // Parity error
		CTRL_BFOV = 0x000800, // Set when overflow, cleared to 0 when starting send/receive
		CTRL_RXB  = 0x001000, // Receiving
		CTRL_RFB  = 0x002000, // Receive done
		CTRL_TFB  = 0x004000, // Transmit done
		CTRL_HRES = 0x008000  // Reset pattern received
	};

	// internal state
	z80_device *cpu;
	emu_timer *timer;
	mie_jvs_device *jvs;
	ioport_port *gpio_port[8];

	uint32_t tbuf[TBUF_SIZE];
	uint32_t control, lreg, jvs_rpos;
	uint8_t gpiodir, gpio_val[8];
	uint8_t irq_enable, irq_pending, maple_irqlevel;
	uint8_t jvs_control, jvs_dest;
	uint8_t jvs_lcr;

	void raise_irq(int level);
	void recalc_irq();
	IRQ_CALLBACK_MEMBER(irq_callback);
};

// Trampoline class, required for device discovery
class mie_jvs_device : public jvs_host
{
public:
	friend class mie_device;

	// construction/destruction
	mie_jvs_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};


// device type definition
DECLARE_DEVICE_TYPE(MIE,     mie_device)
DECLARE_DEVICE_TYPE(MIE_JVS, mie_jvs_device)

#endif // MAME_MACHINE_MIE_H
