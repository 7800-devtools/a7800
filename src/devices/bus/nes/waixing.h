// license:BSD-3-Clause
// copyright-holders:Fabio Priuli
#ifndef MAME_BUS_NES_WAIXING_H
#define MAME_BUS_NES_WAIXING_H

#pragma once

#include "mmc3.h"


// ======================> nes_waixing_a_device

class nes_waixing_a_device : public nes_txrom_device
{
public:
	// construction/destruction
	nes_waixing_a_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual DECLARE_READ8_MEMBER(read_l) override;
	virtual DECLARE_WRITE8_MEMBER(write_l) override;
	virtual DECLARE_WRITE8_MEMBER(waixing_write);
	virtual DECLARE_WRITE8_MEMBER(write_h) override { waixing_write(space, offset, data, mem_mask); }
	virtual void chr_cb(int start, int bank, int source) override;

	virtual void pcb_reset() override;

protected:
	nes_waixing_a_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override;

	void set_mirror(uint8_t nt);

	uint8_t mapper_ram[0x400];
};


// ======================> nes_waixing_a1_device

class nes_waixing_a1_device : public nes_waixing_a_device
{
public:
	// construction/destruction
	nes_waixing_a1_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual void chr_cb(int start, int bank, int source) override;
};


// ======================> nes_waixing_b_device

class nes_waixing_b_device : public nes_waixing_a_device
{
public:
	// construction/destruction
	nes_waixing_b_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void chr_cb(int start, int bank, int source) override;
};


// ======================> nes_waixing_c_device

class nes_waixing_c_device : public nes_waixing_a_device
{
public:
	// construction/destruction
	nes_waixing_c_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void chr_cb(int start, int bank, int source) override;
};


// ======================> nes_waixing_d_device

class nes_waixing_d_device : public nes_waixing_a_device
{
public:
	// construction/destruction
	nes_waixing_d_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void chr_cb(int start, int bank, int source) override;
};


// ======================> nes_waixing_e_device

class nes_waixing_e_device : public nes_waixing_a_device
{
public:
	// construction/destruction
	nes_waixing_e_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void chr_cb(int start, int bank, int source) override;
};


// ======================> nes_waixing_f_device

class nes_waixing_f_device : public nes_waixing_a_device
{
public:
	// construction/destruction
	nes_waixing_f_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual DECLARE_WRITE8_MEMBER(write_h) override;
	virtual void prg_cb(int start, int bank) override;
	virtual void chr_cb(int start, int bank, int source) override;

	virtual void pcb_reset() override;

private:
	virtual void set_prg(int prg_base, int prg_mask) override;
};


// ======================> nes_waixing_g_device

class nes_waixing_g_device : public nes_waixing_a_device
{
public:
	// construction/destruction
	nes_waixing_g_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual DECLARE_WRITE8_MEMBER(write_h) override;
	virtual void chr_cb(int start, int bank, int source) override;

	virtual void pcb_reset() override;

protected:
	virtual void set_chr(uint8_t chr, int chr_base, int chr_mask) override;
};


// ======================> nes_waixing_h_device

class nes_waixing_h_device : public nes_txrom_device
{
public:
	// construction/destruction
	nes_waixing_h_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual void chr_cb(int start, int bank, int source) override;

	// This PCB does not have 1K of internal RAM, so it's not derived from nes_waixing_a_device!!

protected:
	nes_waixing_h_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual DECLARE_WRITE8_MEMBER(write_h) override;
};


// ======================> nes_waixing_h1_device

class nes_waixing_h1_device : public nes_waixing_h_device
{
public:
	// construction/destruction
	nes_waixing_h1_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual DECLARE_WRITE8_MEMBER(write_h) override;

	// This variant does not ignore the wram protect!
};


// ======================> nes_waixing_i_device

class nes_waixing_i_device : public nes_waixing_a_device
{
public:
	// construction/destruction
	nes_waixing_i_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// still to emulate this variant
};


// ======================> nes_waixing_j_device

class nes_waixing_j_device : public nes_waixing_a_device
{
public:
	// construction/destruction
	nes_waixing_j_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual DECLARE_WRITE8_MEMBER(write_h) override;

	virtual void pcb_reset() override;

protected:
	// device-level overrides
	virtual void device_start() override;

	virtual void set_prg(int prg_base, int prg_mask) override;
	uint8_t m_reg[4];
};


// ======================> nes_waixing_sh2_device

class nes_waixing_sh2_device : public nes_txrom_device
{
public:
	// construction/destruction
	nes_waixing_sh2_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override;
	virtual DECLARE_READ8_MEMBER(chr_r) override;
	virtual void chr_cb(int start, int bank, int source) override;

	virtual void pcb_reset() override;

protected:
	uint8_t m_reg[2];
};


// ======================> nes_waixing_sec_device

class nes_waixing_sec_device : public nes_txrom_device
{
public:
	// construction/destruction
	nes_waixing_sec_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual DECLARE_WRITE8_MEMBER(write_l) override;
	virtual void prg_cb(int start, int bank) override;
	virtual void chr_cb(int start, int bank, int source) override;

	virtual void pcb_reset() override;

protected:
	// device-level overrides
	virtual void device_start() override;

	uint8_t m_reg;
};


// ======================> nes_waixing_sgz_device

class nes_waixing_sgz_device : public nes_nrom_device
{
public:
	// construction/destruction
	nes_waixing_sgz_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual DECLARE_WRITE8_MEMBER(write_h) override;

	virtual void hblank_irq(int scanline, int vblank, int blanked) override;
	virtual void pcb_reset() override;

protected:
	// device-level overrides
	virtual void device_start() override;

	uint16_t m_irq_count, m_irq_count_latch;
	int m_irq_enable, m_irq_enable_latch;

	uint8_t m_mmc_vrom_bank[8];
};


// ======================> nes_waixing_sgzlz_device

class nes_waixing_sgzlz_device : public nes_nrom_device
{
public:
	// construction/destruction
	nes_waixing_sgzlz_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual DECLARE_WRITE8_MEMBER(write_l) override;

	virtual void pcb_reset() override;

protected:
	// device-level overrides
	virtual void device_start() override;

	uint8_t m_reg[4];
};


// ======================> nes_waixing_ffv_device

class nes_waixing_ffv_device : public nes_nrom_device
{
public:
	// construction/destruction
	nes_waixing_ffv_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual DECLARE_WRITE8_MEMBER(write_l) override;

	virtual void pcb_reset() override;

protected:
	// device-level overrides
	virtual void device_start() override;

	uint8_t m_reg[2];
};


// ======================> nes_waixing_wxzs_device

class nes_waixing_wxzs_device : public nes_nrom_device
{
public:
	// construction/destruction
	nes_waixing_wxzs_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual DECLARE_WRITE8_MEMBER(write_h) override;

	virtual void pcb_reset() override;

protected:
	// device-level overrides
	virtual void device_start() override;
};


// ======================> nes_waixing_dq8_device

class nes_waixing_dq8_device : public nes_nrom_device
{
public:
	// construction/destruction
	nes_waixing_dq8_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual DECLARE_WRITE8_MEMBER(write_h) override;

	virtual void pcb_reset() override;

protected:
	// device-level overrides
	virtual void device_start() override;
};


// ======================> nes_waixing_wxzs2_device

class nes_waixing_wxzs2_device : public nes_nrom_device
{
public:
	// construction/destruction
	nes_waixing_wxzs2_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual DECLARE_WRITE8_MEMBER(write_h) override;

	virtual void pcb_reset() override;

protected:
	// device-level overrides
	virtual void device_start() override;
};


// ======================> nes_waixing_fs304_device

class nes_waixing_fs304_device : public nes_nrom_device
{
public:
	// construction/destruction
	nes_waixing_fs304_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual DECLARE_WRITE8_MEMBER(write_l) override;

	virtual void pcb_reset() override;

protected:
	// device-level overrides
	virtual void device_start() override;

	uint8_t m_reg[4];
};


// device type definition
DECLARE_DEVICE_TYPE(NES_WAIXING_A,     nes_waixing_a_device)
DECLARE_DEVICE_TYPE(NES_WAIXING_A1,    nes_waixing_a1_device)
DECLARE_DEVICE_TYPE(NES_WAIXING_B,     nes_waixing_b_device)
DECLARE_DEVICE_TYPE(NES_WAIXING_C,     nes_waixing_c_device)
DECLARE_DEVICE_TYPE(NES_WAIXING_D,     nes_waixing_d_device)
DECLARE_DEVICE_TYPE(NES_WAIXING_E,     nes_waixing_e_device)
DECLARE_DEVICE_TYPE(NES_WAIXING_F,     nes_waixing_f_device)
DECLARE_DEVICE_TYPE(NES_WAIXING_G,     nes_waixing_g_device)
DECLARE_DEVICE_TYPE(NES_WAIXING_H,     nes_waixing_h_device)
DECLARE_DEVICE_TYPE(NES_WAIXING_H1,    nes_waixing_h1_device)
DECLARE_DEVICE_TYPE(NES_WAIXING_I,     nes_waixing_i_device)
DECLARE_DEVICE_TYPE(NES_WAIXING_J,     nes_waixing_j_device)
DECLARE_DEVICE_TYPE(NES_WAIXING_SH2,   nes_waixing_sh2_device)
DECLARE_DEVICE_TYPE(NES_WAIXING_SEC,   nes_waixing_sec_device)
DECLARE_DEVICE_TYPE(NES_WAIXING_SGZ,   nes_waixing_sgz_device)
DECLARE_DEVICE_TYPE(NES_WAIXING_SGZLZ, nes_waixing_sgzlz_device)
DECLARE_DEVICE_TYPE(NES_WAIXING_FFV,   nes_waixing_ffv_device)
DECLARE_DEVICE_TYPE(NES_WAIXING_WXZS,  nes_waixing_wxzs_device)
DECLARE_DEVICE_TYPE(NES_WAIXING_DQ8,   nes_waixing_dq8_device)
DECLARE_DEVICE_TYPE(NES_WAIXING_WXZS2, nes_waixing_wxzs2_device)
DECLARE_DEVICE_TYPE(NES_WAIXING_FS304, nes_waixing_fs304_device)

#endif // MAME_BUS_NES_WAIXING_H
