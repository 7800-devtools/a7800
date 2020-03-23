// license:BSD-3-Clause
// copyright-holders:R. Belmont
#ifndef MAME_MACHINE_ESQPANEL_H
#define MAME_MACHINE_ESQPANEL_H

#pragma once

#include "machine/esqvfd.h"
#include "machine/esqlcd.h"

#include <vector>

//**************************************************************************
//  INTERFACE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_ESQPANEL1X22_ADD(_tag) \
	MCFG_DEVICE_ADD(_tag, ESQPANEL1X22, 0)

#define MCFG_ESQPANEL1X22_REPLACE(_tag) \
	MCFG_DEVICE_REPLACE(_tag, ESQPANEL1X22, 0)

#define MCFG_ESQPANEL1X22_REMOVE(_tag) \
	MCFG_DEVICE_REMOVE(_tag)

#define MCFG_ESQPANEL2X40_ADD(_tag) \
	MCFG_DEVICE_ADD(_tag, ESQPANEL2X40, 0)

#define MCFG_ESQPANEL2X40_REPLACE(_tag) \
	MCFG_DEVICE_REPLACE(_tag, ESQPANEL2X40, 0)

#define MCFG_ESQPANEL2X40_REMOVE(_tag) \
	MCFG_DEVICE_REMOVE(_tag)

#define MCFG_ESQPANEL2X40_VFX_ADD(_tag) \
	MCFG_DEVICE_ADD(_tag, ESQPANEL2X40_VFX, 0)

#define MCFG_ESQPANEL2X40_VFX_REPLACE(_tag) \
	MCFG_DEVICE_REPLACE(_tag, ESQPANEL2X40_VFX, 0)

#define MCFG_ESQPANEL2X40_VFX_REMOVE(_tag) \
	MCFG_DEVICE_REMOVE(_tag)

#define MCFG_ESQPANEL2X16_SQ1_ADD(_tag) \
	MCFG_DEVICE_ADD(_tag, ESQPANEL2X16_SQ1, 0)

#define MCFG_ESQPANEL2X16_SQ1_REPLACE(_tag) \
	MCFG_DEVICE_REPLACE(_tag, ESQPANEL2X16_SQ1, 0)

#define MCFG_ESQPANEL2X16_SQ1_REMOVE(_tag) \
	MCFG_DEVICE_REMOVE(_tag)

#define MCFG_ESQPANEL_TX_CALLBACK(_write) \
	devcb = &esqpanel_device::set_tx_wr_callback(*device, DEVCB_##_write);

#define MCFG_ESQPANEL_ANALOG_CALLBACK(_write) \
	devcb = &esqpanel_device::set_analog_wr_callback(*device, DEVCB_##_write);

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> esqpanel_device

class esqpanel_external_panel_server;

class esqpanel_device : public device_t, public device_serial_interface
{
public:
	template <class Object>
	static devcb_base &set_tx_wr_callback(device_t &device, Object &&cb) {
		return downcast<esqpanel_device &>(device).m_write_tx.set_callback(std::forward<Object>(cb));
	}

	template <class Object>
	static devcb_base &set_analog_wr_callback(device_t &device, Object &&cb) {
		return downcast<esqpanel_device &>(device).m_write_analog.set_callback(std::forward<Object>(cb));
	}

	void xmit_char(uint8_t data);
	void set_analog_value(offs_t offset, uint16_t value);

protected:
	// construction/destruction
	esqpanel_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_stop() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

	// serial overrides
	virtual void rcv_complete() override;    // Rx completed receiving byte
	virtual void tra_complete() override;    // Tx completed sending byte
	virtual void tra_callback() override;    // Tx send bit

	virtual void send_to_display(uint8_t data) = 0;

	void check_external_panel_server();

	virtual const std::string get_front_panel_html_file() const { return ""; }
	virtual const std::string get_front_panel_js_file() const { return ""; }
	virtual bool write_contents(std::ostream &o) { return false; }

	std::vector<uint8_t> m_light_states;

	bool m_eps_mode;

	esqpanel_external_panel_server *m_external_panel_server;

private:
	static const int XMIT_RING_SIZE = 16;

	bool  m_bCalibSecondByte;
	bool  m_bButtonLightSecondByte;

	devcb_write_line m_write_tx;
	devcb_write16 m_write_analog;
	uint8_t m_xmitring[XMIT_RING_SIZE];
	int m_xmit_read, m_xmit_write;
	bool m_tx_busy;

	emu_timer *m_external_timer;
};

class esqpanel1x22_device : public esqpanel_device {
public:
	esqpanel1x22_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	virtual void device_add_mconfig(machine_config &config) override;

	virtual void send_to_display(uint8_t data) override { m_vfd->write_char(data); }

	required_device<esq1x22_device> m_vfd;
};

class esqpanel2x40_device : public esqpanel_device {
public:
	esqpanel2x40_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	virtual void device_add_mconfig(machine_config &config) override;

	virtual void send_to_display(uint8_t data) override { m_vfd->write_char(data); }

	required_device<esq2x40_device> m_vfd;
};

class esqpanel2x40_vfx_device : public esqpanel_device {
public:
	esqpanel2x40_vfx_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	virtual void device_add_mconfig(machine_config &config) override;

	virtual void send_to_display(uint8_t data) override { m_vfd->write_char(data); }

	virtual const std::string get_front_panel_html_file() const override { return "/esqpanel/vfx/FrontPanel.html"; }
	virtual const std::string get_front_panel_js_file() const override { return "/esqpanel/vfx/FrontPanel.js"; }
	virtual bool write_contents(std::ostream &o) override;

	required_device<esq2x40_device> m_vfd;

private:
	static const char *html;
	static const char *js;
};

class esqpanel2x40_sq1_device : public esqpanel_device {
public:
	esqpanel2x40_sq1_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	virtual void device_add_mconfig(machine_config &config) override;

	virtual void send_to_display(uint8_t data) override { m_vfd->write_char(data); }

	required_device<esq2x40_sq1_device> m_vfd;
};

// --- SQ1 - Parduz --------------------------------------------------------------------------------------------------------------------------
class esqpanel2x16_sq1_device : public esqpanel_device {
public:
	esqpanel2x16_sq1_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	virtual void device_add_mconfig(machine_config &config) override;

	virtual void send_to_display(uint8_t data) override { m_vfd->write_char(data); }

	required_device<esq2x16_sq1_device> m_vfd;
};

DECLARE_DEVICE_TYPE(ESQPANEL1X22,     esqpanel1x22_device)
DECLARE_DEVICE_TYPE(ESQPANEL2X40,     esqpanel2x40_device)
DECLARE_DEVICE_TYPE(ESQPANEL2X40_VFX, esqpanel2x40_vfx_device)
DECLARE_DEVICE_TYPE(ESQPANEL2X40_SQ1, esqpanel2x40_sq1_device)
DECLARE_DEVICE_TYPE(ESQPANEL2X16_SQ1, esqpanel2x16_sq1_device)

#endif // MAME_MACHINE_ESQPANEL_H
