// license:BSD-3-Clause
// copyright-holders:R. Belmont
#ifndef MAME_MACHINE_EGRET_H
#define MAME_MACHINE_EGRET_H

#pragma once



//**************************************************************************
//  MACROS / CONSTANTS
//**************************************************************************

#define EGRET_TAG   "egret"

#define EGRET_341S0851  0x1100
#define EGRET_341S0850  0x2200
#define EGRET_344S0100  0x3300

//**************************************************************************
//  INTERFACE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_EGRET_ADD(_type) \
	MCFG_DEVICE_ADD(EGRET_TAG, EGRET, 0) \
	MCFG_EGRET_TYPE(_type)

#define MCFG_EGRET_REPLACE(_type) \
	MCFG_DEVICE_REPLACE(EGRET_TAG, EGRET, 0) \
	MCFG_EGRET_TYPE(_type)

#define MCFG_EGRET_TYPE(_type) \
	egret_device::static_set_type(*device, _type);

#define MCFG_EGRET_REMOVE() \
	MCFG_DEVICE_REMOVE(EGRET_TAG)

#define MCFG_EGRET_RESET_CALLBACK(_cb) \
	devcb = &egret_device::set_reset_cb(*device, DEVCB_##_cb);

#define MCFG_EGRET_LINECHANGE_CALLBACK(_cb) \
	devcb = &egret_device::set_linechange_cb(*device, DEVCB_##_cb);

#define MCFG_EGRET_VIA_CLOCK_CALLBACK(_cb) \
	devcb = &egret_device::set_via_clock_cb(*device, DEVCB_##_cb);

#define MCFG_EGRET_VIA_DATA_CALLBACK(_cb) \
	devcb = &egret_device::set_via_data_cb(*device, DEVCB_##_cb);

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> egret_device

class egret_device :  public device_t, public device_nvram_interface
{
public:
	// construction/destruction
	egret_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// inline configuration helpers
	static void static_set_type(device_t &device, int type);

	// device_config_nvram_interface overrides
	virtual void nvram_default() override;
	virtual void nvram_read(emu_file &file) override;
	virtual void nvram_write(emu_file &file) override;

	DECLARE_READ8_MEMBER( ddr_r );
	DECLARE_WRITE8_MEMBER( ddr_w );
	DECLARE_READ8_MEMBER( ports_r );
	DECLARE_WRITE8_MEMBER( ports_w );
	DECLARE_READ8_MEMBER( pll_r );
	DECLARE_WRITE8_MEMBER( pll_w );
	DECLARE_READ8_MEMBER( timer_ctrl_r );
	DECLARE_WRITE8_MEMBER( timer_ctrl_w );
	DECLARE_READ8_MEMBER( timer_counter_r );
	DECLARE_WRITE8_MEMBER( timer_counter_w );
	DECLARE_READ8_MEMBER( onesec_r );
	DECLARE_WRITE8_MEMBER( onesec_w );
	DECLARE_READ8_MEMBER( pram_r );
	DECLARE_WRITE8_MEMBER( pram_w );

	// interface routines
	uint8_t get_xcvr_session() { return xcvr_session; }
	void set_via_full(uint8_t val) { via_full = val; }
	void set_sys_session(uint8_t val) { sys_session = val; }
	uint8_t get_via_data() { return via_data; }
	void set_via_data(uint8_t dat) { via_data = dat; }
	uint8_t get_via_clock() { return via_clock; }
	void set_adb_line(int linestate) { adb_in = (linestate == ASSERT_LINE) ? true : false; }
	int get_adb_dtime() { return m_adb_dtime; }

	int rom_offset;

	template<class _Object> static devcb_base &set_reset_cb(device_t &device, _Object wr) { return downcast<egret_device &>(device).write_reset.set_callback(wr); }
	template<class _Object> static devcb_base &set_linechange_cb(device_t &device, _Object wr) { return downcast<egret_device &>(device).write_linechange.set_callback(wr); }
	template<class _Object> static devcb_base &set_via_clock_cb(device_t &device, _Object wr) { return downcast<egret_device &>(device).write_via_clock.set_callback(wr); }
	template<class _Object> static devcb_base &set_via_data_cb(device_t &device, _Object wr) { return downcast<egret_device &>(device).write_via_data.set_callback(wr); }

	devcb_write_line write_reset, write_linechange, write_via_clock, write_via_data;

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_add_mconfig(machine_config &config) override;
	virtual const tiny_rom_entry *device_rom_region() const override;

	required_device<cpu_device> m_maincpu;

	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

private:
	uint8_t ddrs[3];
	uint8_t ports[3];
	uint8_t pll_ctrl;
	uint8_t timer_ctrl;
	uint8_t timer_counter;
	uint8_t onesec;
	uint8_t xcvr_session, via_full, sys_session, via_data, via_clock, last_adb;
	uint64_t last_adb_time;
	bool egret_controls_power;
	bool adb_in;
	int reset_line;
	int m_adb_dtime;
	emu_timer *m_timer;
	uint8_t pram[0x100], disk_pram[0x100];
	bool pram_loaded;

	void send_port(address_space &space, uint8_t offset, uint8_t data);
};

// device type definition
DECLARE_DEVICE_TYPE(EGRET, egret_device)

#endif // MAME_MACHINE_EGRET_H
