// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/***************************************************************************

    Midway DCS Audio Board

****************************************************************************/

#ifndef MAME_AUDIO_DCS_H
#define MAME_AUDIO_DCS_H

#pragma once

#include "cpu/adsp2100/adsp2100.h"
#include "sound/dmadac.h"
#include "machine/bankdev.h"

#define MCFG_DCS2_AUDIO_DRAM_IN_MB(_dram_in_mb) \
	dcs_audio_device::static_set_dram_in_mb(*device, _dram_in_mb);

#define MCFG_DCS2_AUDIO_POLLING_OFFSET(_polling_offset) \
	dcs_audio_device::static_set_polling_offset(*device, _polling_offset);


class dcs_audio_device : public device_t
{
public:
	// for dcs2 (int dram_in_mb, offs_t polling_offset)
	static void static_set_dram_in_mb(device_t &device, int dram_in_mb) { downcast<dcs_audio_device &>(device).m_dram_in_mb = dram_in_mb; }
	static void static_set_polling_offset(device_t &device, offs_t polling_offset) { downcast<dcs_audio_device &>(device).m_polling_offset = polling_offset; }

	void set_auto_ack(int state);

	void set_fifo_callbacks(read16_delegate fifo_data_r, read16_delegate fifo_status_r, write_line_delegate fifo_reset_w);
	void set_io_callbacks(write_line_delegate output_full_cb, write_line_delegate input_empty_cb);

	uint16_t data_r();
	void ack_w();
	int data2_r();
	int control_r();

	void data_w(uint16_t data);
	void reset_w(int state);

	void fifo_notify(int count, int max);

	DECLARE_WRITE32_MEMBER( dsio_idma_addr_w );
	DECLARE_WRITE32_MEMBER( dsio_idma_data_w );
	DECLARE_READ32_MEMBER( dsio_idma_data_r );
	void dmovlay_remap_memory();
	WRITE32_MEMBER(dmovlay_callback);
	void denver_postload(void);
	void install_speedup(void);

	// non public
	void dcs_boot();
	TIMER_CALLBACK_MEMBER( dcs_reset );
	void dcs_register_state();
	DECLARE_READ16_MEMBER( dcs_dataram_r );
	DECLARE_WRITE16_MEMBER( dcs_dataram_w );
	DECLARE_WRITE16_MEMBER( dcs_data_bank_select_w );
	DECLARE_WRITE16_MEMBER( dcs_data_bank_select2_w );
	inline void sdrc_update_bank_pointers();
	void sdrc_remap_memory();
	void sdrc_reset();
	DECLARE_READ16_MEMBER( sdrc_r );
	DECLARE_WRITE16_MEMBER( sdrc_w );
	void dsio_reset();
	DECLARE_READ16_MEMBER( dsio_r );
	DECLARE_WRITE16_MEMBER( dsio_w );
	void denver_alloc_dmadac(void);
	void denver_reset();
	DECLARE_READ16_MEMBER( denver_r );
	DECLARE_WRITE16_MEMBER( denver_w );
	DECLARE_READ16_MEMBER( latch_status_r );
	DECLARE_READ16_MEMBER( fifo_input_r );
	void dcs_delayed_data_w(uint16_t data);
	TIMER_CALLBACK_MEMBER( dcs_delayed_data_w_callback );
	DECLARE_WRITE16_MEMBER( input_latch_ack_w );
	DECLARE_READ16_MEMBER( input_latch_r );
	DECLARE_READ32_MEMBER( input_latch32_r );
	TIMER_CALLBACK_MEMBER( latch_delayed_w );
	DECLARE_WRITE16_MEMBER( output_latch_w );
	DECLARE_WRITE32_MEMBER( output_latch32_w );
	void delayed_ack_w();
	TIMER_CALLBACK_MEMBER( delayed_ack_w_callback );
	TIMER_CALLBACK_MEMBER( output_control_delayed_w );
	DECLARE_WRITE16_MEMBER( output_control_w );
	DECLARE_READ16_MEMBER( output_control_r );
	void update_timer_count();
	TIMER_DEVICE_CALLBACK_MEMBER( internal_timer_callback );
	void reset_timer();
	DECLARE_WRITE_LINE_MEMBER(timer_enable_callback);
	DECLARE_READ16_MEMBER( adsp_control_r );
	DECLARE_WRITE16_MEMBER( adsp_control_w );
	TIMER_DEVICE_CALLBACK_MEMBER( dcs_irq );
	TIMER_DEVICE_CALLBACK_MEMBER( sport0_irq );
	void recompute_sample_rate();
	WRITE32_MEMBER(sound_tx_callback);
	DECLARE_READ16_MEMBER( dcs_polling_r );
	DECLARE_WRITE16_MEMBER( dcs_polling_w );
	DECLARE_READ32_MEMBER(dcs_polling32_r);
	DECLARE_WRITE32_MEMBER(dcs_polling32_w);
	TIMER_DEVICE_CALLBACK_MEMBER( transfer_watchdog_callback );
	TIMER_CALLBACK_MEMBER( s1_ack_callback2 );
	TIMER_CALLBACK_MEMBER( s1_ack_callback1 );
	int preprocess_stage_1(uint16_t data);
	TIMER_CALLBACK_MEMBER( s2_ack_callback );
	int preprocess_stage_2(uint16_t data);
	int preprocess_write(uint16_t data);

protected:
	// construction/destruction
	dcs_audio_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock, int rev);

	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
	void add_mconfig_dcs(machine_config &config);

	struct sdrc_state
	{
		uint16_t      reg[4];
		uint8_t       seed;
	};


	struct dsio_state
	{
		uint16_t      reg[4];
		uint8_t       start_on_next_write;
		uint16_t      channelbits;
	};


	struct hle_transfer_state
	{
		uint8_t       hle_enabled;
		int32_t       dcs_state;
		int32_t       state;
		int32_t       start;
		int32_t       stop;
		int32_t       type;
		int32_t       temp;
		int32_t       writes_left;
		uint16_t      sum;
		int32_t       fifo_entries;
		timer_device *watchdog;
	};

	adsp21xx_device *m_cpu;
	address_space *m_program;
	address_space *m_data;
	uint8_t       m_rev;
	offs_t      m_polling_offset;
	uint32_t      m_polling_count;
	/* sound output */
	uint8_t       m_channels;
	uint16_t      m_size;
	uint16_t      m_incs;
	dmadac_sound_device *m_dmadac[6];
	timer_device *m_reg_timer;
	timer_device *m_sport0_timer;
	timer_device *m_internal_timer;
	int32_t       m_ireg;
	uint16_t      m_ireg_base;
	uint16_t      m_control_regs[32];

	/* memory access/booting */
	uint16_t *    m_bootrom;
	uint32_t      m_bootrom_words;
	uint16_t *    m_sounddata;
	uint32_t      m_sounddata_words;
	uint32_t      m_sounddata_banks;
	uint16_t      m_sounddata_bank;

	optional_device<address_map_bank_device> m_ram_map;
	optional_memory_bank    m_data_bank;
	memory_bank *           m_rom_page;
	memory_bank *           m_dram_page;

	/* I/O with the host */
	uint8_t       m_auto_ack;
	uint16_t      m_latch_control;
	uint16_t      m_input_data;
	uint16_t      m_output_data;
	uint16_t      m_pre_output_data;
	uint16_t      m_output_control;
	uint64_t      m_output_control_cycles;
	uint8_t       m_last_output_full;
	uint8_t       m_last_input_empty;
	uint16_t      m_progflags;

	write_line_delegate m_output_full_cb;
	write_line_delegate m_input_empty_cb;

	read16_delegate m_fifo_data_r;
	read16_delegate m_fifo_status_r;
	write_line_delegate m_fifo_reset_w;

	/* timers */
	uint8_t       m_timer_enable;
	uint8_t       m_timer_ignore;
	uint64_t      m_timer_start_cycles;
	uint32_t      m_timer_start_count;
	uint32_t      m_timer_scale;
	uint32_t      m_timer_period;
	uint32_t      m_timers_fired;

	uint16_t *m_sram;
	uint16_t m_polling_value;
	uint32_t m_polling32_value;
	uint32_t *m_internal_program_ram;
	uint32_t *m_external_program_ram;
	uint32_t *m_internal_data_ram;

	int m_dmovlay_val;

	sdrc_state m_sdrc;
	dsio_state m_dsio;
	hle_transfer_state m_transfer;

	int m_dram_in_mb;

	optional_shared_ptr<uint16_t> m_iram;
};


// dcs_audio_2k_device

class dcs_audio_2k_device : public dcs_audio_device
{
public:
	// construction/destruction
	dcs_audio_2k_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	// optional information overrides
	virtual void device_add_mconfig(machine_config &config) override;

};

// device type definition
DECLARE_DEVICE_TYPE(DCS_AUDIO_2K, dcs_audio_2k_device)

// dcs_audio_2k_uart_device

class dcs_audio_2k_uart_device : public dcs_audio_device
{
public:
	// construction/destruction
	dcs_audio_2k_uart_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	// optional information overrides
	virtual void device_add_mconfig(machine_config &config) override;
};

// device type definition
DECLARE_DEVICE_TYPE(DCS_AUDIO_2K_UART, dcs_audio_2k_uart_device)

// dcs_audio_8k_device

class dcs_audio_8k_device : public dcs_audio_device
{
public:
	// construction/destruction
	dcs_audio_8k_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	// optional information overrides
	virtual void device_add_mconfig(machine_config &config) override;
};

// device type definition
DECLARE_DEVICE_TYPE(DCS_AUDIO_8K, dcs_audio_8k_device)

// dcs_audio_wpc_device

class dcs_audio_wpc_device : public dcs_audio_device
{
public:
	// construction/destruction
	dcs_audio_wpc_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	// optional information overrides
	virtual void device_add_mconfig(machine_config &config) override;
};

// device type definition
DECLARE_DEVICE_TYPE(DCS_AUDIO_WPC, dcs_audio_wpc_device)


// dcs2_audio_device

class dcs2_audio_device : public dcs_audio_device
{
protected:
	// construction/destruction
	dcs2_audio_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override;
	void add_mconfig_dcs2(machine_config &config);
};

// dcs2_audio_2115_device

class dcs2_audio_2115_device : public dcs2_audio_device
{
public:
	// construction/destruction
	dcs2_audio_2115_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	// optional information overrides
	virtual void device_add_mconfig(machine_config &config) override;
};

// device type definition
DECLARE_DEVICE_TYPE(DCS2_AUDIO_2115, dcs2_audio_2115_device)

// dcs2_audio_2104_device

class dcs2_audio_2104_device : public dcs2_audio_device
{
public:
	// construction/destruction
	dcs2_audio_2104_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	// optional information overrides
	virtual void device_add_mconfig(machine_config &config) override;
};

// device type definition
DECLARE_DEVICE_TYPE(DCS2_AUDIO_2104, dcs2_audio_2104_device)

// dcs2_audio_dsio_device

class dcs2_audio_dsio_device : public dcs2_audio_device
{
public:
	// construction/destruction
	dcs2_audio_dsio_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	// optional information overrides
	virtual void device_add_mconfig(machine_config &config) override;
};

// device type definition
DECLARE_DEVICE_TYPE(DCS2_AUDIO_DSIO, dcs2_audio_dsio_device)

// dcs2_audio_denver_device

class dcs2_audio_denver_device : public dcs2_audio_device
{
public:
	// construction/destruction
	dcs2_audio_denver_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	// optional information overrides
	virtual void device_add_mconfig(machine_config &config) override;
};

// device type definition
DECLARE_DEVICE_TYPE(DCS2_AUDIO_DENVER, dcs2_audio_denver_device)

#endif // MAME_AUDIO_DCS_H
