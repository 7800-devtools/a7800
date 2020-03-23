// license:BSD-3-Clause
// copyright-holders:Fabio Priuli
#ifndef MAME_BUS_NES_BANDAI_H
#define MAME_BUS_NES_BANDAI_H

#pragma once

#include "nxrom.h"
#include "machine/i2cmem.h"


// ======================> nes_oekakids_device

class nes_oekakids_device : public nes_nrom_device
{
public:
	// construction/destruction
	nes_oekakids_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual DECLARE_WRITE8_MEMBER(write_h) override;
	virtual DECLARE_READ8_MEMBER(nt_r) override;
	virtual DECLARE_WRITE8_MEMBER(nt_w) override;

	virtual void pcb_reset() override;

	virtual void ppu_latch(offs_t offset) override;

	// TODO: add oeka kids controller emulation

protected:
	// device-level overrides
	virtual void device_start() override;

	void update_chr();

	uint8_t m_reg, m_latch;
};


// ======================> nes_fcg_device

class nes_fcg_device : public nes_nrom_device
{
public:
	// construction/destruction
	nes_fcg_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;
	virtual DECLARE_WRITE8_MEMBER(fcg_write);
	virtual DECLARE_WRITE8_MEMBER(write_m) override;

	virtual void pcb_reset() override;

protected:
	static constexpr device_timer_id TIMER_IRQ = 0;

	nes_fcg_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override;

	uint16_t   m_irq_count;
	int        m_irq_enable;

	emu_timer *irq_timer;
};


// ======================> nes_lz93d50_device

class nes_lz93d50_device : public nes_fcg_device
{
public:
	// construction/destruction
	nes_lz93d50_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual DECLARE_WRITE8_MEMBER(write_h) override { fcg_write(space, offset, data, mem_mask); }

protected:
	nes_lz93d50_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);
};


// ======================> nes_lz93d50_24c01_device

class nes_lz93d50_24c01_device : public nes_lz93d50_device
{
public:
	// construction/destruction
	nes_lz93d50_24c01_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual DECLARE_READ8_MEMBER(read_m) override;
	virtual DECLARE_WRITE8_MEMBER(write_h) override;

	virtual void pcb_reset() override;

protected:
	nes_lz93d50_24c01_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override;

	virtual void device_add_mconfig(machine_config &config) override;

	// TODO: fix EEPROM I/O emulation
	required_device<i2cmem_device> m_i2cmem;
	uint8_t m_i2c_dir;
};


// ======================> nes_lz93d50_24c02_device

class nes_lz93d50_24c02_device : public nes_lz93d50_24c01_device
{
public:
	// construction/destruction
	nes_lz93d50_24c02_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	// device-level overrides
	virtual void device_add_mconfig(machine_config &config) override;
};


// ======================> nes_fjump2_device

class nes_fjump2_device : public nes_lz93d50_device
{
public:
	// construction/destruction
	nes_fjump2_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual DECLARE_READ8_MEMBER(read_m) override;
	virtual DECLARE_WRITE8_MEMBER(write_m) override;
	virtual DECLARE_WRITE8_MEMBER(write_h) override;

	virtual void pcb_reset() override;

protected:
	// device-level overrides
	virtual void device_start() override;

	void set_prg();
	uint8_t m_reg[5];
};


// device type definition
DECLARE_DEVICE_TYPE(NES_OEKAKIDS,      nes_oekakids_device)
DECLARE_DEVICE_TYPE(NES_FCG,           nes_fcg_device)
DECLARE_DEVICE_TYPE(NES_LZ93D50,       nes_lz93d50_device)
DECLARE_DEVICE_TYPE(NES_LZ93D50_24C01, nes_lz93d50_24c01_device)
DECLARE_DEVICE_TYPE(NES_LZ93D50_24C02, nes_lz93d50_24c02_device)
DECLARE_DEVICE_TYPE(NES_FJUMP2,        nes_fjump2_device)

#endif // MAME_BUS_NES_BANDAI_H
