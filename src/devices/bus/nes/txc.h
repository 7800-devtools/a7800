// license:BSD-3-Clause
// copyright-holders:Fabio Priuli
#ifndef MAME_BUS_NES_TXC_H
#define MAME_BUS_NES_TXC_H

#pragma once

#include "nxrom.h"


// ======================> nes_txc_22211_device

class nes_txc_22211_device : public nes_nrom_device
{
public:
	// construction/destruction
	nes_txc_22211_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual DECLARE_READ8_MEMBER(read_l) override;
	virtual DECLARE_WRITE8_MEMBER(write_l) override;
	virtual DECLARE_WRITE8_MEMBER(write_h) override;

	virtual void pcb_reset() override;

protected:
	nes_txc_22211_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override;

	uint8_t m_reg[4];
};


// ======================> nes_txc_dumarc_device

class nes_txc_dumarc_device : public nes_txc_22211_device
{
public:
	// construction/destruction
	nes_txc_dumarc_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual DECLARE_WRITE8_MEMBER(write_h) override;
};


// ======================> nes_txc_mjblock_device

class nes_txc_mjblock_device : public nes_txc_22211_device
{
public:
	// construction/destruction
	nes_txc_mjblock_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual DECLARE_READ8_MEMBER(read_l) override;
};


// ======================> nes_txc_strikew_device

class nes_txc_strikew_device : public nes_nrom_device
{
public:
	// construction/destruction
	nes_txc_strikew_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual DECLARE_WRITE8_MEMBER(write_h) override;

	virtual void pcb_reset() override;

protected:
	// device-level overrides
	virtual void device_start() override;
};


// ======================> nes_txc_commandos_device

class nes_txc_commandos_device : public nes_nrom_device
{
public:
	// construction/destruction
	nes_txc_commandos_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual DECLARE_READ8_MEMBER(read_l) override;
	virtual DECLARE_WRITE8_MEMBER(write_h) override;

	virtual void pcb_reset() override;

protected:
	// device-level overrides
	virtual void device_start() override;
};


// device type definition
DECLARE_DEVICE_TYPE(NES_TXC_22211,      nes_txc_22211_device)
DECLARE_DEVICE_TYPE(NES_TXC_DUMARACING, nes_txc_dumarc_device)
DECLARE_DEVICE_TYPE(NES_TXC_MJBLOCK,    nes_txc_mjblock_device)
DECLARE_DEVICE_TYPE(NES_TXC_STRIKEW,    nes_txc_strikew_device)
DECLARE_DEVICE_TYPE(NES_TXC_COMMANDOS,  nes_txc_commandos_device)

#endif // MAME_BUS_NES_TXC_H
