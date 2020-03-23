// license:LGPL-2.1+
// copyright-holders:Michael Zapf
/****************************************************************************

    TI-99/8 main board logic

    This component implements the address decoder and mapper logic from the
    TI-99/8 console.

    See 998board.c for documentation

    Michael Zapf

*****************************************************************************/

#ifndef MAME_BUS_TI99_INTERNAL_998BOARD_H
#define MAME_BUS_TI99_INTERNAL_998BOARD_H

#pragma once

#include "bus/ti99/ti99defs.h"
#include "bus/ti99/gromport/gromport.h"
#include "bus/hexbus/hexbus.h"

#include "bus/ti99/internal/ioport.h"
#include "machine/ram.h"
#include "machine/tmc0430.h"
#include "sound/sn76496.h"
#include "sound/tms5220.h"
#include "video/tms9928a.h"

// -------------- Defines ------------------------------------

#define TI998_SRAM_TAG        "sram8"
#define TI998_DRAM_TAG        "dram8"
#define TI998_MAPPER_TAG      "mapper"
#define TI998_MAINBOARD_TAG  "mainboard8"
#define TI998_SPEECHSYN_TAG     "speech"

#define TI998_ROM0_REG        "rom0_region"
#define TI998_ROM1_REG        "rom1_region"
#define TI998_PASCAL_REG      "pascal_region"
#define TI998_SYSGROM_REG     "sysgrom_region"
#define TI998_GROMLIB1_REG    "gromlib1_region"
#define TI998_GROMLIB2_REG    "gromlib2_region"
#define TI998_GROMLIB3_REG    "gromlib3_region"
#define TI998_SPEECHROM_REG       "speech_region"

#define TI998_GROMLIB_TAG "gromlib"
#define TI998_SYSGROM_TAG TI998_GROMLIB_TAG "0"
#define TI998_SYSGROM0_TAG TI998_SYSGROM_TAG "_0"
#define TI998_SYSGROM1_TAG TI998_SYSGROM_TAG "_1"
#define TI998_SYSGROM2_TAG TI998_SYSGROM_TAG "_2"

#define TI998_GLIB1_TAG TI998_GROMLIB_TAG "1"
#define TI998_GLIB10_TAG TI998_GLIB1_TAG "_0"
#define TI998_GLIB11_TAG TI998_GLIB1_TAG "_1"
#define TI998_GLIB12_TAG TI998_GLIB1_TAG "_2"
#define TI998_GLIB13_TAG TI998_GLIB1_TAG "_3"
#define TI998_GLIB14_TAG TI998_GLIB1_TAG "_4"
#define TI998_GLIB15_TAG TI998_GLIB1_TAG "_5"
#define TI998_GLIB16_TAG TI998_GLIB1_TAG "_6"
#define TI998_GLIB17_TAG TI998_GLIB1_TAG "_7"

#define TI998_GLIB2_TAG TI998_GROMLIB_TAG "2"
#define TI998_GLIB20_TAG TI998_GLIB2_TAG "_0"
#define TI998_GLIB21_TAG TI998_GLIB2_TAG "_1"
#define TI998_GLIB22_TAG TI998_GLIB2_TAG "_2"
#define TI998_GLIB23_TAG TI998_GLIB2_TAG "_3"
#define TI998_GLIB24_TAG TI998_GLIB2_TAG "_4"
#define TI998_GLIB25_TAG TI998_GLIB2_TAG "_5"
#define TI998_GLIB26_TAG TI998_GLIB2_TAG "_6"
#define TI998_GLIB27_TAG TI998_GLIB2_TAG "_7"

#define TI998_GLIB3_TAG TI998_GROMLIB_TAG "3"
#define TI998_GLIB30_TAG TI998_GLIB3_TAG "_0"
#define TI998_GLIB31_TAG TI998_GLIB3_TAG "_1"
#define TI998_GLIB32_TAG TI998_GLIB3_TAG "_2"

#define TI998_VAQUERRO_TAG "vaquerro"
#define TI998_MOFETTA_TAG "mofetta"
#define TI998_AMIGO_TAG "amigo"
#define TI998_OSO_TAG "oso"

// --------------------------------------------------

namespace bus { namespace ti99 { namespace internal {

class mainboard8_device;

/*
    Custom chip: Vaquerro
*/
class vaquerro_device : public device_t
{
public:
	vaquerro_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	void device_start() override;
	void device_reset() override;

	line_state ready();
	void treset();

	DECLARE_READ8_MEMBER( read );
	DECLARE_SETADDRESS_DBIN_MEMBER( set_address );

	DECLARE_READ_LINE_MEMBER( sprd_out );
	DECLARE_READ_LINE_MEMBER( spwt_out );
	DECLARE_READ_LINE_MEMBER( sccs_out );
	DECLARE_READ_LINE_MEMBER( sromcs_out );

	// Collective select line query
	int gromcs_out();

	DECLARE_READ_LINE_MEMBER( vdprd_out );
	DECLARE_READ_LINE_MEMBER( vdpwt_out );
	DECLARE_READ_LINE_MEMBER( lascsq_out );
	DECLARE_READ_LINE_MEMBER( ggrdy_out );
	DECLARE_WRITE_LINE_MEMBER( hold_cpu );

	DECLARE_WRITE_LINE_MEMBER( crus_in );
	DECLARE_WRITE_LINE_MEMBER( crusgl_in );
	DECLARE_WRITE_LINE_MEMBER( clock_in );
	DECLARE_WRITE_LINE_MEMBER( memen_in );

	DECLARE_WRITE_LINE_MEMBER( sgmry );
	DECLARE_WRITE_LINE_MEMBER( tsgry );
	DECLARE_WRITE_LINE_MEMBER( p8gry );
	DECLARE_WRITE_LINE_MEMBER( p3gry );

private:
	/*
	    Wait state generator (part of Vaquerro)
	*/
	class waitstate_generator
	{
	public:
		waitstate_generator() :
			m_counting(false),
			m_generate(false),
			m_counter(0),
			m_addressed(true),
			m_ready(true)
		{
		}

		virtual ~waitstate_generator() { }
		void select_in(bool addressed);
		virtual void ready_in(line_state ready) = 0;
		virtual void clock_in(line_state clkout) = 0;
		void treset_in(line_state reset);

		int select_out();
		void init(int select_value) { m_selvalue = select_value; }

		line_state ready_out();

		bool is_counting();
		bool is_generating();
		bool is_ready();

	protected:
		// Two flipflops
		bool m_counting;
		bool m_generate;
		// Counter
		int  m_counter;

		// Select value (indicates selected line)
		int  m_selvalue;

		// Line state flags
		bool m_addressed;
		bool m_ready;
	};

	class grom_waitstate_generator : public waitstate_generator
	{
	public:
		void ready_in(line_state ready) override;
		void clock_in(line_state clkout) override;
	};

	class video_waitstate_generator : public waitstate_generator
	{
	public:
		void ready_in(line_state ready) override { }
		void clock_in(line_state clkout) override;
	};

	// Memory cycle state
	bool m_memen;

	// Waiting for video
	bool m_video_wait;

	// State of the CRUS line
	int m_crus;

	// Are the GROM libraries turned on?
	bool m_crugl;

	// Do we have a logical address space match?
	bool m_lasreq = false;

	// Keep the decoding result (opens the SRY gate)
	bool m_grom_or_video = false;

	// Select lines
	bool m_spwt;
	bool m_sccs;
	bool m_sromcs;
	bool m_sprd;
	bool m_vdprd;
	bool m_vdpwt;

	// Collective GROM select state
	int m_gromsel;

	// Outgoing READY
	int m_ggrdy;

	// Outgoing READY latch (common flipflop driving SRY)
	bool m_sry;

	// Holds the A14 address line state. We need this for the clock_in method.
	int m_a14;

	// Keeps the recent DBIN level
	int m_dbin_level;

	// Wait state logic components
	grom_waitstate_generator m_sgmws, m_tsgws, m_p8gws, m_p3gws;
	video_waitstate_generator m_vidws;

	// Pointer to mainboard
	mainboard8_device* m_mainboard;
};

/*
    Custom chip: Mofetta
*/
class mofetta_device : public device_t
{
public:
	mofetta_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	void device_start() override;
	void device_reset() override;

	DECLARE_WRITE8_MEMBER( cruwrite );
	DECLARE_SETADDRESS_DBIN_MEMBER( set_address );

	// Debugger support
	bool hexbus_access_debug();
	bool intdsr_access_debug();

	DECLARE_WRITE_LINE_MEMBER( clock_in );
	DECLARE_WRITE_LINE_MEMBER( msast_in );
	DECLARE_WRITE_LINE_MEMBER( lascs_in );
	DECLARE_WRITE_LINE_MEMBER( pmemen_in );
	DECLARE_WRITE_LINE_MEMBER( skdrcs_in );

	DECLARE_READ8_MEMBER( rom1cs_out );
	DECLARE_READ_LINE_MEMBER( gromclk_out );

	DECLARE_READ_LINE_MEMBER( alccs_out );
	DECLARE_READ_LINE_MEMBER( prcs_out );
	DECLARE_READ_LINE_MEMBER( cmas_out );
	DECLARE_READ_LINE_MEMBER( dbc_out );

	DECLARE_READ_LINE_MEMBER( rom1cs_out );
	DECLARE_READ_LINE_MEMBER( rom1am_out );
	DECLARE_READ_LINE_MEMBER( rom1al_out );

private:
	// Memory cycle state
	bool m_pmemen;

	// Logical access
	bool m_lasreq;

	// DRAM access
	bool m_skdrcs;

	// Indicates the UP level of the GROMCLK
	bool m_gromclk_up;

	// Have we got the upper word of the address?
	bool m_gotfirstword;

	// Address latch
	int m_address_latch;

	// Most significant byte of the 24-bit address
	int m_prefix;

	// CRU select of the 1700 device
	bool m_alcpg;

	// CRU select of the 2700 device
	bool m_txspg;

	// ROM1 select lines
	bool m_rom1cs;
	bool m_rom1am;
	bool m_rom1al;

	// OSO select
	bool m_alccs;

	// Pascal ROM select line
	bool m_prcs;

	// Cartridge port select line
	bool m_cmas;

	// GROM clock count (as frequency divider)
	int m_gromclock_count;

	// Remember last msast state for edge detection
	int m_msast;

	// Pointer to mainboard
	mainboard8_device* m_mainboard;
};

/*
    Custom chip: Amigo
*/
class amigo_device : public device_t
{
public:
	amigo_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	void device_start() override;
	void device_reset() override;

	DECLARE_READ8_MEMBER( read );
	DECLARE_WRITE8_MEMBER( write );
	DECLARE_SETOFFSET_MEMBER( set_address );

	// Debugger support
	int get_physical_address_debug(offs_t offset);
	void mapper_access_debug(int data);

	DECLARE_WRITE_LINE_MEMBER( srdy_in );
	DECLARE_WRITE_LINE_MEMBER( clock_in );
	DECLARE_WRITE_LINE_MEMBER( crus_in );
	DECLARE_WRITE_LINE_MEMBER( lascs_in );
	DECLARE_WRITE_LINE_MEMBER( memen_in );

	DECLARE_WRITE_LINE_MEMBER( holda_in );

	DECLARE_READ_LINE_MEMBER( cpury_out );
	DECLARE_READ_LINE_MEMBER( sramcs_out );
	DECLARE_READ_LINE_MEMBER( skdrcs_out );

	void connect_sram(uint8_t* sram) { m_sram = sram; }
	bool mapper_accessed() { return m_mapper_accessed; }

private:
	// Memory cycle state
	bool m_memen;

	// DMA methods for loading/saving maps
	void mapper_load();
	void mapper_save();

	// Address mapper registers. Each offset is selected by the first 4 bits
	// of the logical address.
	uint32_t  m_base_register[16];

	// Indicates a logical space access
	bool m_logical_space;

	// Physical address
	uint32_t  m_physical_address;

	// Pointer to SRAM where AMIGO needs to upload/download its map values
	uint8_t* m_sram;

	// Pointer to mainboard
	mainboard8_device* m_mainboard;

	// Keep the system ready state
	int m_srdy;

	// Outgoing READY level
	int m_ready_out;

	// Keep the CRUS setting
	int m_crus;

	// State of the address creation
	int m_amstate;

	// Protection flags
	int m_protflag;

	// Accessing SRAM
	bool m_sram_accessed;

	// Accessing DRAM
	bool m_dram_accessed;

	// Accessing the mapper
	bool m_mapper_accessed;

	// HOLDA flag
	bool m_hold_acknowledged;

	// Address in SRAM during DMA
	uint32_t  m_sram_address;

	// Number of the currently loaded/save base register
	int m_basereg;

	// Latched value for mapper DMA transfer
	uint32_t m_mapvalue;
};

/*
    Custom chip: OSO
*/
class oso_device : public bus::hexbus::hexbus_chained_device
{
public:
	oso_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	DECLARE_READ8_MEMBER( read );
	DECLARE_WRITE8_MEMBER( write );
	void device_start() override;
	// Don't add a hexbus connector; we use the one from the driver instance
	virtual void device_add_mconfig(machine_config &config) override { };

	void hexbus_value_changed(uint8_t data) override;

	WRITE_LINE_MEMBER( clock_in );

private:
	uint8_t m_data;
	uint8_t m_status;
	uint8_t m_control;
	uint8_t m_xmit;

	int m_clkcount;

	int m_xmit_send;
};

class mainboard8_device : public device_t
{
public:
	mainboard8_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// Memory space
	DECLARE_READ8_MEMBER( read );
	DECLARE_WRITE8_MEMBER( write );
	DECLARE_SETOFFSET_MEMBER( setoffset );

	// Memory space for debugger access
	DECLARE_READ8_MEMBER( debugger_read );
	DECLARE_WRITE8_MEMBER( debugger_write );

	// I/O space
	DECLARE_READ8Z_MEMBER( crureadz );
	DECLARE_WRITE8_MEMBER( cruwrite );

	// Control lines
	DECLARE_WRITE_LINE_MEMBER( clock_in );
	DECLARE_WRITE_LINE_MEMBER( dbin_in );
	DECLARE_WRITE_LINE_MEMBER( msast_in );
	DECLARE_WRITE_LINE_MEMBER( crus_in );
	DECLARE_WRITE_LINE_MEMBER( ptgen_in );
	DECLARE_WRITE_LINE_MEMBER( reset_console );
	DECLARE_WRITE_LINE_MEMBER( hold_cpu );
	DECLARE_WRITE_LINE_MEMBER( ggrdy_in );

	DECLARE_WRITE_LINE_MEMBER( holda_line );

	template<class _Object> static devcb_base &set_ready_wr_callback(device_t &device, _Object object)
	{
		return downcast<mainboard8_device &>(device).m_ready.set_callback(object);
	}

	template<class _Object> static devcb_base &set_reset_wr_callback(device_t &device, _Object object)
	{
		return downcast<mainboard8_device &>(device).m_console_reset.set_callback(object);
	}

	template<class _Object> static devcb_base &set_hold_wr_callback(device_t &device, _Object object)
	{
		return downcast<mainboard8_device &>(device).m_hold_line.set_callback(object);
	}

	void set_paddress(int address);

	// Ready lines from GROMs
	DECLARE_WRITE_LINE_MEMBER( system_grom_ready );
	DECLARE_WRITE_LINE_MEMBER( ptts_grom_ready );
	DECLARE_WRITE_LINE_MEMBER( p8_grom_ready );
	DECLARE_WRITE_LINE_MEMBER( p3_grom_ready );
	DECLARE_WRITE_LINE_MEMBER( sound_ready );
	DECLARE_WRITE_LINE_MEMBER( speech_ready );
	DECLARE_WRITE_LINE_MEMBER( pbox_ready );

protected:
	void device_start() override;
	void device_reset() override;
	virtual void device_add_mconfig(machine_config &config) override;

private:
	// Holds the state of the A14 line
	bool    m_A14_set;

	// Propagates the end of the memory cycle
	void cycle_end();

	// Original logical address.
	int     m_logical_address;

	// Mapped physical address.
	int     m_physical_address;

	// Hold the address space value so that we can use it in other methods.
	address_space*  m_space;

	// Indicates that a byte is waiting on the data bus (see m_latched_data)
	bool    m_pending_write;

	// Hold the value of the data bus. In a real machine, the data bus continues
	// to show that value, but in this emulation we have a push mechanism.
	uint8_t   m_latched_data;

	// Hold the level of the GROMCLK line
	int m_gromclk;

	// Selecting GROM libraries
	void select_groms();

	// Previous select state
	int m_prev_grom;

	// Ready states
	bool m_speech_ready;
	bool m_sound_ready;
	bool m_pbox_ready;

	// Keeps the recent DBIN level
	int m_dbin_level;

	// Ready line to the CPU
	devcb_write_line m_ready;

	// Reset line to the main system
	devcb_write_line m_console_reset;

	// Hold line to the main system
	devcb_write_line m_hold_line;

	// Custom chips
	required_device<vaquerro_device> m_vaquerro;
	required_device<mofetta_device>  m_mofetta;
	required_device<amigo_device>    m_amigo;
	required_device<oso_device>       m_oso;

	// More devices
	required_device<tms9928a_device>        m_video;
	required_device<sn76496_base_device>    m_sound;
	required_device<cd2501ecd_device>       m_speech;
	required_device<bus::ti99::gromport::gromport_device>   m_gromport;
	required_device<bus::ti99::internal::ioport_device>     m_ioport;

	required_device<ram_device>             m_sram;
	required_device<ram_device>             m_dram;

	// Debugging
	int m_last_ready;
	line_state m_crus_debug;

	// System GROM library
	tmc0430_device* m_sgrom[3];

	// Text-to-speech GROM library
	tmc0430_device* m_tsgrom[8];

	// Pascal 8 GROM library
	tmc0430_device* m_p8grom[8];

	// Pascal 3 GROM library
	tmc0430_device* m_p3grom[3];

	// Idle flags for GROMs
	bool m_sgrom_idle;
	bool m_tsgrom_idle;
	bool m_p8grom_idle;
	bool m_p3grom_idle;

	// ROM area of the system.
	uint8_t*   m_rom0;
	uint8_t*   m_rom1;
	uint8_t*   m_pascalrom;
};

} } } // end namespace bus::ti99::internal

#define MCFG_MAINBOARD8_READY_CALLBACK(_write) \
	devcb = &bus::ti99::internal::mainboard8_device::set_ready_wr_callback(*device, DEVCB_##_write);

#define MCFG_MAINBOARD8_RESET_CALLBACK(_write) \
	devcb = &bus::ti99::internal::mainboard8_device::set_reset_wr_callback(*device, DEVCB_##_write);

#define MCFG_MAINBOARD8_HOLD_CALLBACK(_write) \
	devcb = &bus::ti99::internal::mainboard8_device::set_hold_wr_callback(*device, DEVCB_##_write);

DECLARE_DEVICE_TYPE_NS(TI99_MAINBOARD8, bus::ti99::internal, mainboard8_device)
DECLARE_DEVICE_TYPE_NS(TI99_VAQUERRO, bus::ti99::internal, vaquerro_device)
DECLARE_DEVICE_TYPE_NS(TI99_MOFETTA,  bus::ti99::internal, mofetta_device)
DECLARE_DEVICE_TYPE_NS(TI99_AMIGO,    bus::ti99::internal, amigo_device)
DECLARE_DEVICE_TYPE_NS(TI99_OSO,      bus::ti99::internal, oso_device)

#endif // MAME_BUS_TI99_INTERNAL_998BOARD_H
