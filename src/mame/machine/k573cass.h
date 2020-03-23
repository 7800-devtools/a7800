// license:BSD-3-Clause
// copyright-holders:smf
/*
 * Konami 573 Security Cassette
 *
 */
#ifndef MAME_MACHINE_K573CASS_H
#define MAME_MACHINE_K573CASS_H

#pragma once


#include "machine/adc083x.h"
#include "machine/ds2401.h"
#include "machine/x76f041.h"
#include "machine/x76f100.h"
#include "machine/zs01.h"

#define MCFG_KONAMI573_CASSETTE_DSR_HANDLER(_devcb) \
	devcb = &konami573_cassette_slot_device::set_dsr_handler(*device, DEVCB_##_devcb);


DECLARE_DEVICE_TYPE(KONAMI573_CASSETTE_SLOT, konami573_cassette_slot_device)

class konami573_cassette_interface;

class konami573_cassette_slot_device : public device_t,
	public device_slot_interface
{
	friend class konami573_cassette_interface;

public:
	konami573_cassette_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	template <class Object> static devcb_base &set_dsr_handler(device_t &device, Object &&cb) { return downcast<konami573_cassette_slot_device &>(device).m_dsr_handler.set_callback(std::forward<Object>(cb)); }

	DECLARE_WRITE_LINE_MEMBER(write_line_d0);
	DECLARE_WRITE_LINE_MEMBER(write_line_d1);
	DECLARE_WRITE_LINE_MEMBER(write_line_d2);
	DECLARE_WRITE_LINE_MEMBER(write_line_d3);
	DECLARE_WRITE_LINE_MEMBER(write_line_d4);
	DECLARE_WRITE_LINE_MEMBER(write_line_d5);
	DECLARE_WRITE_LINE_MEMBER(write_line_d6);
	DECLARE_WRITE_LINE_MEMBER(write_line_d7);
	DECLARE_WRITE_LINE_MEMBER(write_line_zs01_sda);
	DECLARE_READ_LINE_MEMBER(read_line_ds2401);
	DECLARE_READ_LINE_MEMBER(read_line_secflash_sda);
	DECLARE_READ_LINE_MEMBER(read_line_adc083x_do);
	DECLARE_READ_LINE_MEMBER(read_line_adc083x_sars);

protected:
	virtual void device_start() override;

	devcb_write_line m_dsr_handler;

private:
	konami573_cassette_interface *m_dev;
};

class konami573_cassette_interface : public device_slot_card_interface
{
	friend class konami573_cassette_slot_device;

public:
	virtual ~konami573_cassette_interface();

	DECLARE_WRITE_LINE_MEMBER(output_dsr) { m_slot->m_dsr_handler(state); }

	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d0);
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d1) = 0;
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d2) = 0;
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d3) = 0;
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d4);
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d5);
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d6);
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d7);
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_zs01_sda);
	virtual DECLARE_READ_LINE_MEMBER(read_line_ds2401);
	virtual DECLARE_READ_LINE_MEMBER(read_line_secflash_sda) = 0;
	virtual DECLARE_READ_LINE_MEMBER(read_line_adc083x_do);
	virtual DECLARE_READ_LINE_MEMBER(read_line_adc083x_sars);

protected:
	konami573_cassette_interface(const machine_config &mconfig, device_t &device);

	konami573_cassette_slot_device *m_slot;
};


DECLARE_DEVICE_TYPE(KONAMI573_CASSETTE_X, konami573_cassette_x_device)

class konami573_cassette_x_device: public device_t,
	public konami573_cassette_interface
{
public:
	konami573_cassette_x_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual DECLARE_READ_LINE_MEMBER(read_line_secflash_sda) override;
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d0) override;
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d1) override;
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d2) override;
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d3) override;

protected:
	konami573_cassette_x_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	virtual void device_start() override;
	virtual void device_add_mconfig(machine_config &config) override;

private:
	required_device<x76f041_device> m_x76f041;
};


DECLARE_DEVICE_TYPE(KONAMI573_CASSETTE_XI, konami573_cassette_xi_device)

class konami573_cassette_xi_device: public konami573_cassette_x_device
{
public:
	konami573_cassette_xi_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual DECLARE_READ_LINE_MEMBER(read_line_ds2401) override;
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d4) override;

	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d0) override;
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d1) override;
	virtual DECLARE_READ_LINE_MEMBER(read_line_adc083x_do) override;
	virtual DECLARE_READ_LINE_MEMBER(read_line_adc083x_sars) override;
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d5) override;

	ADC083X_INPUT_CB(punchmania_inputs_callback);

protected:
	virtual void device_add_mconfig(machine_config &config) override;

private:
	required_device<ds2401_device> m_ds2401;
	required_device<adc0838_device> m_adc0838;
};


DECLARE_DEVICE_TYPE(KONAMI573_CASSETTE_Y, konami573_cassette_y_device)


#define MCFG_KONAMI573_CASSETTE_Y_D0_HANDLER(_devcb) \
	devcb = &konami573_cassette_y_device::set_d0_handler(*device, DEVCB_##_devcb);

#define MCFG_KONAMI573_CASSETTE_Y_D1_HANDLER(_devcb) \
	devcb = &konami573_cassette_y_device::set_d1_handler(*device, DEVCB_##_devcb);

#define MCFG_KONAMI573_CASSETTE_Y_D2_HANDLER(_devcb) \
	devcb = &konami573_cassette_y_device::set_d2_handler(*device, DEVCB_##_devcb);

#define MCFG_KONAMI573_CASSETTE_Y_D3_HANDLER(_devcb) \
	devcb = &konami573_cassette_y_device::set_d3_handler(*device, DEVCB_##_devcb);

#define MCFG_KONAMI573_CASSETTE_Y_D4_HANDLER(_devcb) \
	devcb = &konami573_cassette_y_device::set_d4_handler(*device, DEVCB_##_devcb);

#define MCFG_KONAMI573_CASSETTE_Y_D5_HANDLER(_devcb) \
	devcb = &konami573_cassette_y_device::set_d5_handler(*device, DEVCB_##_devcb);

#define MCFG_KONAMI573_CASSETTE_Y_D6_HANDLER(_devcb) \
	devcb = &konami573_cassette_y_device::set_d6_handler(*device, DEVCB_##_devcb);

#define MCFG_KONAMI573_CASSETTE_Y_D7_HANDLER(_devcb) \
	devcb = &konami573_cassette_y_device::set_d7_handler(*device, DEVCB_##_devcb);

class konami573_cassette_y_device: public device_t,
	public konami573_cassette_interface
{
public:
	konami573_cassette_y_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// static configuration helpers
	template<class _Object> static devcb_base &set_d0_handler(device_t &device, _Object object) { return downcast<konami573_cassette_y_device &>(device).m_d0_handler.set_callback(object); }
	template<class _Object> static devcb_base &set_d1_handler(device_t &device, _Object object) { return downcast<konami573_cassette_y_device &>(device).m_d1_handler.set_callback(object); }
	template<class _Object> static devcb_base &set_d2_handler(device_t &device, _Object object) { return downcast<konami573_cassette_y_device &>(device).m_d2_handler.set_callback(object); }
	template<class _Object> static devcb_base &set_d3_handler(device_t &device, _Object object) { return downcast<konami573_cassette_y_device &>(device).m_d3_handler.set_callback(object); }
	template<class _Object> static devcb_base &set_d4_handler(device_t &device, _Object object) { return downcast<konami573_cassette_y_device &>(device).m_d4_handler.set_callback(object); }
	template<class _Object> static devcb_base &set_d5_handler(device_t &device, _Object object) { return downcast<konami573_cassette_y_device &>(device).m_d5_handler.set_callback(object); }
	template<class _Object> static devcb_base &set_d6_handler(device_t &device, _Object object) { return downcast<konami573_cassette_y_device &>(device).m_d6_handler.set_callback(object); }
	template<class _Object> static devcb_base &set_d7_handler(device_t &device, _Object object) { return downcast<konami573_cassette_y_device &>(device).m_d7_handler.set_callback(object); }

	virtual DECLARE_READ_LINE_MEMBER(read_line_secflash_sda) override;
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d0) override;
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d1) override;
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d2) override;
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d3) override;
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d4) override;
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d5) override;
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d6) override;
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d7) override;

protected:
	konami573_cassette_y_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	virtual void device_start() override;
	virtual void device_add_mconfig(machine_config &config) override;

private:
	required_device<x76f100_device> m_x76f100;
	devcb_write_line m_d0_handler;
	devcb_write_line m_d1_handler;
	devcb_write_line m_d2_handler;
	devcb_write_line m_d3_handler;
	devcb_write_line m_d4_handler;
	devcb_write_line m_d5_handler;
	devcb_write_line m_d6_handler;
	devcb_write_line m_d7_handler;
};


DECLARE_DEVICE_TYPE(KONAMI573_CASSETTE_YI, konami573_cassette_yi_device)

class konami573_cassette_yi_device: public konami573_cassette_y_device
{
public:
	konami573_cassette_yi_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual DECLARE_READ_LINE_MEMBER(read_line_ds2401) override;
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d4) override;

protected:
	virtual void device_add_mconfig(machine_config &config) override;

private:
	required_device<ds2401_device> m_ds2401;
};


DECLARE_DEVICE_TYPE(KONAMI573_CASSETTE_ZI, konami573_cassette_zi_device)

class konami573_cassette_zi_device: public device_t,
	public konami573_cassette_interface
{
public:
	konami573_cassette_zi_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual DECLARE_READ_LINE_MEMBER(read_line_ds2401) override;
	virtual DECLARE_READ_LINE_MEMBER(read_line_secflash_sda) override;
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d4) override;
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d1) override;
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d2) override;
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d3) override;
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_zs01_sda) override;

protected:
	virtual void device_start() override;
	virtual void device_add_mconfig(machine_config &config) override;

private:
	required_device<zs01_device> m_zs01;
	required_device<ds2401_device> m_ds2401;
};


#endif // MAME_MACHINE_K573CASS_H
