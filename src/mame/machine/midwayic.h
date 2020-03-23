// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/***************************************************************************

    Emulation of various Midway ICs

***************************************************************************/
#ifndef MAME_MACHINE_MIDWAY_IC_H
#define MAME_MACHINE_MIDWAY_IC_H

#pragma once


#include "audio/cage.h"
#include "audio/dcs.h"
#include "cpu/pic16c5x/pic16c5x.h"

/* 1st generation Midway serial PIC - simulation*/

class midway_serial_pic_device : public device_t
{
public:
	// construction/destruction
	midway_serial_pic_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	static void static_set_upper(device_t &device, int upper) { downcast<midway_serial_pic_device &>(device).m_upper = upper; }

	DECLARE_READ8_MEMBER( read );
	DECLARE_WRITE8_MEMBER( write );
	DECLARE_READ8_MEMBER( status_r );
	DECLARE_WRITE_LINE_MEMBER( reset_w );

protected:
	midway_serial_pic_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override;

	void generate_serial_data(int upper);
	void serial_register_state();

	uint8_t   m_data[16]; // reused by other devices
	int     m_upper;
private:
	uint8_t   m_buff;
	uint8_t   m_idx;
	uint8_t   m_status;
	uint8_t   m_bits;
	uint8_t   m_ormask;
};


// device type definition
DECLARE_DEVICE_TYPE(MIDWAY_SERIAL_PIC, midway_serial_pic_device)

#define MCFG_MIDWAY_SERIAL_PIC_UPPER(_upper) \
	midway_serial_pic_device::static_set_upper(*device, _upper);

/* 1st generation Midway serial PIC - emulation */

class midway_serial_pic_emu_device : public device_t
{
public:
	// construction/destruction
	midway_serial_pic_emu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	midway_serial_pic_emu_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_add_mconfig(machine_config &config) override;
	virtual void device_start() override;

private:
	DECLARE_READ8_MEMBER(read_a);
	DECLARE_READ8_MEMBER(read_b);
	DECLARE_READ8_MEMBER(read_c);
	DECLARE_WRITE8_MEMBER(write_a);
	DECLARE_WRITE8_MEMBER(write_b);
	DECLARE_WRITE8_MEMBER(write_c);
};


// device type definition
DECLARE_DEVICE_TYPE(MIDWAY_SERIAL_PIC_EMU, midway_serial_pic_emu_device)



/* 2nd generation Midway serial/NVRAM/RTC PIC */

// ======================> midway_serial_pic2_device

class midway_serial_pic2_device : public midway_serial_pic_device, public device_nvram_interface
{
public:
	// construction/destruction
	midway_serial_pic2_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	static void static_set_yearoffs(device_t &device, int yearoffs) { downcast<midway_serial_pic2_device &>(device).m_yearoffs = yearoffs; }

	DECLARE_READ8_MEMBER( read );
	DECLARE_WRITE8_MEMBER( write );
	DECLARE_READ8_MEMBER( status_r );

	void set_default_nvram(const uint8_t *nvram);

protected:
	midway_serial_pic2_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override;

	// device_nvram_interface overrides
	virtual void nvram_default() override;
	virtual void nvram_read(emu_file &file) override;
	virtual void nvram_write(emu_file &file) override;

private:

	void pic_register_state();
	TIMER_CALLBACK_MEMBER( reset_timer );

	uint16_t  m_latch;
	attotime m_latch_expire_time;
	uint8_t   m_state;
	uint8_t   m_index;
	uint8_t   m_total;
	uint8_t   m_nvram_addr;
	uint8_t   m_buffer[0x10];
	uint8_t   m_nvram[0x100];
	uint8_t   m_default_nvram[0x100];
	uint8_t   m_time_buf[8];
	uint8_t   m_time_index;
	uint8_t   m_time_just_written;
	uint16_t  m_yearoffs;
	emu_timer *m_time_write_timer;
};


// device type definition
DECLARE_DEVICE_TYPE(MIDWAY_SERIAL_PIC2, midway_serial_pic2_device)

#define MCFG_MIDWAY_SERIAL_PIC2_UPPER   MCFG_MIDWAY_SERIAL_PIC_UPPER

#define MCFG_MIDWAY_SERIAL_PIC2_YEAR_OFFS(_yearoffs) \
	midway_serial_pic2_device::static_set_yearoffs(*device, _yearoffs);

/* I/O ASIC connected to 2nd generation PIC */

// ======================> midway_ioasic_device

class midway_ioasic_device : public midway_serial_pic2_device
{
public:
	// construction/destruction
	midway_ioasic_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	static void static_set_shuffle(device_t &device, uint8_t shuffle) { downcast<midway_ioasic_device &>(device).m_shuffle_type = shuffle; }
	static void static_set_shuffle_default(device_t &device, uint8_t shuffle) { downcast<midway_ioasic_device &>(device).m_shuffle_default = shuffle; }
	static void static_set_auto_ack(device_t &device, uint8_t auto_ack) { downcast<midway_ioasic_device &>(device).m_auto_ack = auto_ack; }
	template<class _Object> static devcb_base &set_irqhandler_callback(device_t &device, _Object object) { return downcast<midway_ioasic_device &>(device).m_irq_callback.set_callback(object); }
	template<class _Object> static devcb_base &set_serial_tx_callback(device_t &device, _Object object) { return downcast<midway_ioasic_device &>(device).m_serial_tx_cb.set_callback(object); }
	template<class _Object> static devcb_base &set_aux_output_callback(device_t &device, _Object object) { return downcast<midway_ioasic_device &>(device).m_aux_output_cb.set_callback(object); }

	void set_shuffle_state(int state);
	void fifo_w(uint16_t data);
	void fifo_full_w(uint16_t data);

	DECLARE_WRITE_LINE_MEMBER(fifo_reset_w);
	DECLARE_READ16_MEMBER(fifo_r);
	DECLARE_READ16_MEMBER(fifo_status_r);

	DECLARE_WRITE_LINE_MEMBER(ioasic_input_empty);
	DECLARE_WRITE_LINE_MEMBER(ioasic_output_full);

	DECLARE_READ32_MEMBER( read );
	DECLARE_WRITE32_MEMBER( write );
	DECLARE_READ32_MEMBER( packed_r );
	DECLARE_WRITE32_MEMBER( packed_w );

	DECLARE_WRITE8_MEMBER(cage_irq_handler);

	void serial_rx_w(u8 data);

	void ioasic_reset();

protected:
	// device-level overrides
	virtual void device_start() override;

private:
	void ioasic_register_state();
	void update_ioasic_irq();

	devcb_write8    m_serial_tx_cb;
	devcb_write32   m_aux_output_cb;

	uint32_t  m_reg[16];
	uint8_t   m_has_dcs;
	uint8_t   m_has_cage;
	device_t *m_dcs_cpu;
	uint8_t   m_shuffle_type;
	uint8_t   m_shuffle_default;
	uint8_t   m_shuffle_active;
	const uint8_t *   m_shuffle_map;
	devcb_write8 m_irq_callback;
	uint8_t   m_irq_state;
	uint16_t  m_sound_irq_state;
	uint8_t   m_auto_ack;
	uint8_t   m_force_fifo_full;

	uint16_t  m_fifo[512];
	uint16_t  m_fifo_in;
	uint16_t  m_fifo_out;
	uint16_t  m_fifo_bytes;
	offs_t  m_fifo_force_buffer_empty_pc;

	atari_cage_device *m_cage;
	dcs_audio_device *m_dcs;
};


// device type definition
DECLARE_DEVICE_TYPE(MIDWAY_IOASIC, midway_ioasic_device)

#define MCFG_MIDWAY_IOASIC_UPPER    MCFG_MIDWAY_SERIAL_PIC_UPPER

#define MCFG_MIDWAY_IOASIC_YEAR_OFFS MCFG_MIDWAY_SERIAL_PIC2_YEAR_OFFS

#define MCFG_MIDWAY_IOASIC_SHUFFLE(_shuffle) \
	midway_ioasic_device::static_set_shuffle(*device, _shuffle);

#define MCFG_MIDWAY_IOASIC_SHUFFLE_DEFAULT(_shuffle) \
	midway_ioasic_device::static_set_shuffle_default(*device, _shuffle);

#define MCFG_MIDWAY_IOASIC_IRQ_CALLBACK(_write) \
	devcb = &midway_ioasic_device::set_irqhandler_callback(*device, DEVCB_##_write);

#define MCFG_MIDWAY_IOASIC_AUTO_ACK(_ack) \
	midway_ioasic_device::static_set_auto_ack(*device, _ack);

#define MCFG_MIDWAY_IOASIC_OUT_TX_CB(_devcb) \
	devcb = &midway_ioasic_device::set_serial_tx_callback(*device, DEVCB_##_devcb);

#define MCFG_MIDWAY_IOASIC_AUX_OUT_CB(_devcb) \
	devcb = &midway_ioasic_device::set_aux_output_callback(*device, DEVCB_##_devcb);


enum
{
	MIDWAY_IOASIC_STANDARD = 0,
	MIDWAY_IOASIC_BLITZ99,
	MIDWAY_IOASIC_CARNEVIL,
	MIDWAY_IOASIC_CALSPEED,
	MIDWAY_IOASIC_MACE,
	MIDWAY_IOASIC_GAUNTDL,
	MIDWAY_IOASIC_VAPORTRX,
	MIDWAY_IOASIC_SFRUSHRK,
	MIDWAY_IOASIC_HYPRDRIV
};

#endif // MAME_MACHINE_MIDWAY_IC_H
