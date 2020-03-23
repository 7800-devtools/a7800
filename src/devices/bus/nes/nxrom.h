// license:BSD-3-Clause
// copyright-holders:Fabio Priuli
#ifndef MAME_BUS_NES_NXROM_H
#define MAME_BUS_NES_NXROM_H

#pragma once

#include "nes_slot.h"
#include "sound/samples.h"


// ======================> nes_nrom_device

class nes_nrom_device : public device_t, public device_nes_cart_interface
{
public:
	// construction/destruction
	nes_nrom_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual void pcb_reset() override;

protected:
	nes_nrom_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	virtual void device_start() override { common_start(); }

	void common_start();
};


// ======================> nes_nrom368_device

class nes_nrom368_device : public nes_nrom_device
{
public:
	// construction/destruction
	nes_nrom368_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual DECLARE_READ8_MEMBER(read_l) override;
	virtual DECLARE_READ8_MEMBER(read_m) override;
	virtual DECLARE_READ8_MEMBER(read_h) override;
};


// ======================> nes_fcbasic_device

class nes_fcbasic_device : public nes_nrom_device
{
public:
	// construction/destruction
	nes_fcbasic_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// emulate the additional WRAM
};


// ======================> nes_axrom_device

class nes_axrom_device : public nes_nrom_device
{
public:
	// construction/destruction
	nes_axrom_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual DECLARE_WRITE8_MEMBER(write_h) override;

	virtual void pcb_reset() override;

protected:
	// device-level overrides
	virtual void device_start() override;
};


// ======================> nes_bxrom_device

class nes_bxrom_device : public nes_nrom_device
{
public:
	// construction/destruction
	nes_bxrom_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual DECLARE_WRITE8_MEMBER(write_h) override;

	virtual void pcb_reset() override;

protected:
	// device-level overrides
	virtual void device_start() override;
};


// ======================> nes_cnrom_device

class nes_cnrom_device : public nes_nrom_device
{
public:
	// construction/destruction
	nes_cnrom_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual DECLARE_READ8_MEMBER(chr_r) override;
	virtual DECLARE_WRITE8_MEMBER(write_h) override;

	virtual void pcb_reset() override;

protected:
	nes_cnrom_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override;

private:
	uint8_t m_chr_open_bus;
};


// ======================> nes_cprom_device

class nes_cprom_device : public nes_nrom_device
{
public:
	// construction/destruction
	nes_cprom_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual DECLARE_WRITE8_MEMBER(write_h) override;

	virtual void pcb_reset() override;

protected:
	// device-level overrides
	virtual void device_start() override;
};


// ======================> nes_gxrom_device

class nes_gxrom_device : public nes_nrom_device
{
public:
	// construction/destruction
	nes_gxrom_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual DECLARE_WRITE8_MEMBER(write_h) override;

	virtual void pcb_reset() override;

protected:
	// device-level overrides
	virtual void device_start() override;
};


// ======================> nes_uxrom_device

class nes_uxrom_device : public nes_nrom_device
{
public:
	// construction/destruction
	nes_uxrom_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual DECLARE_WRITE8_MEMBER(write_h) override;

	virtual void pcb_reset() override;

protected:
	// device-level overrides
	virtual void device_start() override;
};


// ======================> nes_uxrom_cc_device

class nes_uxrom_cc_device : public nes_nrom_device
{
public:
	// construction/destruction
	nes_uxrom_cc_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual DECLARE_WRITE8_MEMBER(write_h) override;

	virtual void pcb_reset() override;

protected:
	// device-level overrides
	virtual void device_start() override;
};


// ======================> nes_un1rom_device

class nes_un1rom_device : public nes_nrom_device
{
public:
	// construction/destruction
	nes_un1rom_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual DECLARE_WRITE8_MEMBER(write_h) override;

	virtual void pcb_reset() override;

protected:
	// device-level overrides
	virtual void device_start() override;
};


// ======================> nes_nochr_device

class nes_nochr_device : public nes_nrom_device
{
public:
	// construction/destruction
	nes_nochr_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual DECLARE_READ8_MEMBER(chr_r) override;
	virtual DECLARE_WRITE8_MEMBER(chr_w) override;
};



// device type definition
DECLARE_DEVICE_TYPE(NES_NROM,     nes_nrom_device)
DECLARE_DEVICE_TYPE(NES_NROM368,  nes_nrom368_device)
DECLARE_DEVICE_TYPE(NES_FCBASIC,  nes_fcbasic_device)
DECLARE_DEVICE_TYPE(NES_AXROM,    nes_axrom_device)
DECLARE_DEVICE_TYPE(NES_BXROM,    nes_bxrom_device)
DECLARE_DEVICE_TYPE(NES_CNROM,    nes_cnrom_device)
DECLARE_DEVICE_TYPE(NES_CPROM,    nes_cprom_device)
DECLARE_DEVICE_TYPE(NES_GXROM,    nes_gxrom_device)
DECLARE_DEVICE_TYPE(NES_UXROM,    nes_uxrom_device)
DECLARE_DEVICE_TYPE(NES_UXROM_CC, nes_uxrom_cc_device)
DECLARE_DEVICE_TYPE(NES_UN1ROM,   nes_un1rom_device)
DECLARE_DEVICE_TYPE(NES_NOCHR,    nes_nochr_device)

#endif // MAME_BUS_NES_NXROM_H
