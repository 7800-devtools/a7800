// license:BSD-3-Clause
// copyright-holders:Fabio Priuli
#ifndef MAME_BUS_NES_CNE_H
#define MAME_BUS_NES_CNE_H

#pragma once

#include "nxrom.h"


// ======================> nes_cne_decathl_device

class nes_cne_decathl_device : public nes_nrom_device
{
public:
	// construction/destruction
	nes_cne_decathl_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual DECLARE_WRITE8_MEMBER(write_h) override;

	virtual void pcb_reset() override;

protected:
	// device-level overrides
	virtual void device_start() override;
};


// ======================> nes_cne_fsb_device

class nes_cne_fsb_device : public nes_nrom_device
{
public:
	// construction/destruction
	nes_cne_fsb_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual DECLARE_READ8_MEMBER(read_m) override;
	virtual DECLARE_WRITE8_MEMBER(write_m) override;

	virtual void pcb_reset() override;

protected:
	// device-level overrides
	virtual void device_start() override;
};


// ======================> nes_cne_shlz_device

class nes_cne_shlz_device : public nes_nrom_device
{
public:
	// construction/destruction
	nes_cne_shlz_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual DECLARE_WRITE8_MEMBER(write_l) override;

	virtual void pcb_reset() override;

protected:
	// device-level overrides
	virtual void device_start() override;
};


// device type definition
DECLARE_DEVICE_TYPE(NES_CNE_DECATHL, nes_cne_decathl_device)
DECLARE_DEVICE_TYPE(NES_CNE_FSB,     nes_cne_fsb_device)
DECLARE_DEVICE_TYPE(NES_CNE_SHLZ,    nes_cne_shlz_device)

#endif // MAME_BUS_NES_CNE_H
