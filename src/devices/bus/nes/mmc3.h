// license:BSD-3-Clause
// copyright-holders:Fabio Priuli
#ifndef MAME_BUS_NES_MMC3_H
#define MAME_BUS_NES_MMC3_H

#pragma once

#include "nxrom.h"


// ======================> nes_txrom_device

class nes_txrom_device : public nes_nrom_device
{
public:
	// construction/destruction
	nes_txrom_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual DECLARE_READ8_MEMBER(read_m) override;
	virtual DECLARE_WRITE8_MEMBER(write_m) override;
	virtual DECLARE_WRITE8_MEMBER(txrom_write);
	virtual DECLARE_WRITE8_MEMBER(write_h) override { txrom_write(space, offset, data, mem_mask); }
	virtual void prg_cb(int start, int bank);
	virtual void chr_cb(int start, int bank, int source);

	virtual void hblank_irq(int scanline, int vblank, int blanked) override;
	virtual void pcb_reset() override;

protected:
	nes_txrom_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override { mmc3_start(); }

	virtual void set_prg(int prg_base, int prg_mask);
	virtual void set_chr(uint8_t chr, int chr_base, int chr_mask);
	void mmc3_start();
	void mmc3_common_initialize(int prg_mask, int chr_mask, int IRQ_type);

	// are there MMC3 clones which need more regs?
	uint16_t m_mmc_prg_bank[4];
	uint16_t m_mmc_vrom_bank[8];  // a few clones need more than the 6 banks used by base MMC3 (e.g. waixing_g)

	int m_prg_base, m_prg_mask; // MMC3 based multigame carts select a block of banks by using these (and then act like normal MMC3),
	int m_chr_base, m_chr_mask; // while MMC3 and clones (mapper 118 & 119) simply set them as 0 and 0xff resp.

	int m_latch;
	int m_wram_protect;
	int m_alt_irq;

	uint16_t m_irq_count, m_irq_count_latch;
	uint8_t m_irq_clear;
	int m_irq_enable;
};


// ======================> nes_hkrom_device

class nes_hkrom_device : public nes_txrom_device
{
public:
	// construction/destruction
	nes_hkrom_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual DECLARE_READ8_MEMBER(read_m) override;
	virtual DECLARE_WRITE8_MEMBER(write_m) override;
	virtual DECLARE_WRITE8_MEMBER(write_h) override;

	virtual void pcb_reset() override;

protected:
	// device-level overrides
	virtual void device_start() override;

	int m_wram_enable;
	uint8_t m_mmc6_reg;

	// MMC-6 contains 1K of internal ram, battery backed up
	uint8_t m_mmc6_ram[0x400];
};


// ======================> nes_txsrom_device

class nes_txsrom_device : public nes_txrom_device
{
public:
	// construction/destruction
	nes_txsrom_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual DECLARE_WRITE8_MEMBER(write_h) override;
	virtual void chr_cb(int start, int bank, int source) override;

protected:
	void set_mirror();
};


// ======================> nes_tqrom_device

class nes_tqrom_device : public nes_txrom_device
{
public:
	// construction/destruction
	nes_tqrom_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	virtual void set_chr( uint8_t chr, int chr_base, int chr_mask ) override;
};


// ======================> nes_qj_device

class nes_qj_device : public nes_txrom_device
{
public:
	// construction/destruction
	nes_qj_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual DECLARE_WRITE8_MEMBER(write_m) override;
	virtual void pcb_reset() override;
};


// ======================> nes_zz_device

class nes_zz_device : public nes_txrom_device
{
public:
	// construction/destruction
	nes_zz_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual DECLARE_WRITE8_MEMBER(write_m) override;
	virtual void pcb_reset() override;
};


// device type definition
DECLARE_DEVICE_TYPE(NES_TXROM,  nes_txrom_device)
DECLARE_DEVICE_TYPE(NES_HKROM,  nes_hkrom_device)
DECLARE_DEVICE_TYPE(NES_TXSROM, nes_txsrom_device)
DECLARE_DEVICE_TYPE(NES_TQROM,  nes_tqrom_device)
DECLARE_DEVICE_TYPE(NES_QJ_PCB, nes_qj_device)
DECLARE_DEVICE_TYPE(NES_ZZ_PCB, nes_zz_device)

#endif // MAME_BUS_NES_MMC3_H
