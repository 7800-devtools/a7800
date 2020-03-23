// license:BSD-3-Clause
// copyright-holders:Ernesto Corvi, Nicola Salmoria, David Haywood, Vas Crabb
#ifndef MAME_MACHINE_TAITO68705INTERFACE_H
#define MAME_MACHINE_TAITO68705INTERFACE_H

#pragma once

#include "cpu/m6805/m68705.h"


DECLARE_DEVICE_TYPE(TAITO68705_MCU,       taito68705_mcu_device)
DECLARE_DEVICE_TYPE(TAITO68705_MCU_TIGER, taito68705_mcu_tiger_device)
DECLARE_DEVICE_TYPE(ARKANOID_68705P3,     arkanoid_68705p3_device)
DECLARE_DEVICE_TYPE(ARKANOID_68705P5,     arkanoid_68705p5_device)


class taito68705_mcu_device_base : public device_t
{
public:
	template <typename Obj> static devcb_base &set_semaphore_cb(device_t &device, Obj &&cb)
	{ return downcast<taito68705_mcu_device_base &>(device).m_semaphore_cb.set_callback(std::forward<Obj>(cb)); }

	// host interface
	DECLARE_READ8_MEMBER(data_r);
	DECLARE_WRITE8_MEMBER(data_w);
	DECLARE_WRITE_LINE_MEMBER(reset_w);
	DECLARE_READ_LINE_MEMBER(host_semaphore_r) { return m_host_flag ? ASSERT_LINE : CLEAR_LINE; }
	DECLARE_READ_LINE_MEMBER(mcu_semaphore_r) { return m_mcu_flag ? ASSERT_LINE : CLEAR_LINE; }
	DECLARE_CUSTOM_INPUT_MEMBER(host_semaphore_r) { return m_host_flag ? 1 : 0; }
	DECLARE_CUSTOM_INPUT_MEMBER(mcu_semaphore_r) { return m_mcu_flag ? 1 : 0; }

protected:
	taito68705_mcu_device_base(
			machine_config const &mconfig,
			device_type type,
			char const *tag,
			device_t *owner,
			u32 clock);

	// MCU callbacks
	DECLARE_WRITE8_MEMBER(mcu_pa_w);

	virtual void device_start() override;
	virtual void device_reset() override;

	bool host_flag() const { return m_host_flag; }
	bool mcu_flag() const { return m_mcu_flag; }
	u8 pa_value() const { return m_pa_output & (m_latch_driven ? m_host_latch : 0xff); }
	void latch_control(u8 data, u8 &value, unsigned host_bit, unsigned mcu_bit);

	required_device<m68705p_device> m_mcu;

private:
	devcb_write_line    m_semaphore_cb;

	bool    m_latch_driven;
	bool    m_reset_input;
	bool    m_host_flag;
	bool    m_mcu_flag;
	u8      m_host_latch;
	u8      m_mcu_latch;
	u8      m_pa_output;
};


#define MCFG_TAITO_M68705_AUX_STROBE_CB(cb) \
		devcb = &taito68705_mcu_device::set_aux_strobe_cb(*device, DEVCB_##cb);

class taito68705_mcu_device : public taito68705_mcu_device_base
{
public:
	template <typename Obj> static devcb_base &set_aux_strobe_cb(device_t &device, Obj &&cb)
	{ return downcast<taito68705_mcu_device &>(device).m_aux_strobe_cb.set_callback(std::forward<Obj>(cb)); }

	taito68705_mcu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	taito68705_mcu_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, u32 clock);

	virtual DECLARE_READ8_MEMBER(mcu_portc_r);
	DECLARE_WRITE8_MEMBER(mcu_portb_w);

	virtual void device_add_mconfig(machine_config &config) override;
	virtual void device_start() override;

	devcb_write8    m_aux_strobe_cb;

	u8  m_pb_output;
};


class taito68705_mcu_tiger_device : public taito68705_mcu_device
{
public:
	taito68705_mcu_tiger_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

protected:
	virtual DECLARE_READ8_MEMBER(mcu_portc_r) override;
};


#define MCFG_ARKANOID_MCU_SEMAPHORE_CB(cb) \
		devcb = &arkanoid_mcu_device_base::set_semaphore_cb(*device, DEVCB_##cb);

#define MCFG_ARKANOID_MCU_PORTB_R_CB(cb) \
		devcb = &arkanoid_mcu_device_base::set_portb_r_cb(*device, DEVCB_##cb);

class arkanoid_mcu_device_base : public taito68705_mcu_device_base
{
public:
	template <typename Obj> static devcb_base &set_portb_r_cb(device_t &device, Obj &&cb)
	{ return downcast<arkanoid_mcu_device_base &>(device).m_portb_r_cb.set_callback(std::forward<Obj>(cb)); }

protected:
	arkanoid_mcu_device_base(
			machine_config const &mconfig,
			device_type type,
			char const *tag,
			device_t *owner,
			u32 clock);

	// MCU callbacks
	DECLARE_READ8_MEMBER(mcu_pb_r);
	DECLARE_READ8_MEMBER(mcu_pc_r);
	DECLARE_WRITE8_MEMBER(mcu_pc_w);

	virtual void device_start() override;

	devcb_read8 m_portb_r_cb;

	u8  m_pc_output;
};


class arkanoid_68705p3_device : public arkanoid_mcu_device_base
{
public:
	arkanoid_68705p3_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock);

protected:
	virtual void device_add_mconfig(machine_config &config) override;
};


class arkanoid_68705p5_device : public arkanoid_mcu_device_base
{
public:
	arkanoid_68705p5_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock);

protected:
	virtual void device_add_mconfig(machine_config &config) override;
};

#endif // MAME_MACHINE_TAITO68705INTERFACE_H
