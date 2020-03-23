// license:LGPL-2.1+
// copyright-holders:Michael Zapf
/****************************************************************************

    Myarc Hard and Floppy Disk Controller
    See hfdc.c for documentation

    January 2012: rewritten as class
    June 2014: rewritten for modern floppy implementation

    Michael Zapf
    July 2015

****************************************************************************/

#ifndef MAME_BUS_TI99_PEB_HFDC_H
#define MAME_BUS_TI99_PEB_HFDC_H

#pragma once

#include "peribox.h"

#include "imagedev/floppy.h"
#include "imagedev/mfmhd.h"

#include "machine/mm58274c.h"
#include "machine/hdc92x4.h"
#include "machine/ram.h"

namespace bus { namespace ti99 { namespace peb {

/*
    Implementation for modern floppy system.
*/
class myarc_hfdc_device : public device_t, public device_ti99_peribox_card_interface
{
public:
	myarc_hfdc_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	DECLARE_READ8Z_MEMBER(readz) override;
	DECLARE_WRITE8_MEMBER(write) override;
	DECLARE_SETADDRESS_DBIN_MEMBER(setaddress_dbin) override;
	DECLARE_READ8Z_MEMBER(crureadz) override;
	DECLARE_WRITE8_MEMBER(cruwrite) override;

protected:
	void device_config_complete() override;

private:
	void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;
	void device_start() override;
	void device_reset() override;

	const tiny_rom_entry *device_rom_region() const override;
	virtual void device_add_mconfig(machine_config &config) override;
	ioport_constructor device_input_ports() const override;

	DECLARE_WRITE_LINE_MEMBER( dmarq_w );
	DECLARE_WRITE_LINE_MEMBER( intrq_w );
	DECLARE_WRITE_LINE_MEMBER( dip_w );
	DECLARE_WRITE8_MEMBER( auxbus_out );
	DECLARE_READ8_MEMBER( read_buffer );
	DECLARE_WRITE8_MEMBER( write_buffer );

	DECLARE_FLOPPY_FORMATS( floppy_formats );

	// Debug accessors
	void debug_read(offs_t offset, uint8_t* value);
	void debug_write(offs_t offset, uint8_t data);

	// Callbacks for the index hole and seek complete
	void floppy_index_callback(floppy_image_device *floppy, int state);
	void harddisk_index_callback(mfm_harddisk_device *harddisk, int state);
	void harddisk_ready_callback(mfm_harddisk_device *harddisk, int state);
	void harddisk_skcom_callback(mfm_harddisk_device *harddisk, int state);

	// Operate the floppy motors
	void set_floppy_motors_running(bool run);

	// Connect floppy drives
	void connect_floppy_unit(int index);

	// Connect harddisk drives
	void connect_harddisk_unit(int index);

	// Disconnect drives
	void disconnect_floppy_drives();
	void disconnect_hard_drives();

	// Pushes the drive status to the HDC
	void signal_drive_status();

	// Motor monoflop (4.23 sec)
	emu_timer*      m_motor_on_timer;

	// HDC9234 controller on the board
	required_device<hdc9234_device> m_hdc9234;

	// Clock chip on the board
	required_device<mm58274c_device> m_clock;

	// Link to the attached floppy drives
	floppy_image_device*    m_floppy_unit[4];

	// Link to the attached hard disks
	mfm_harddisk_device*    m_harddisk_unit[3];

	// Currently selected floppy drive
	floppy_image_device*    m_current_floppy;

	// Currently selected hard drive
	mfm_harddisk_device*    m_current_harddisk;

	// True: Access to DIP switch settings, false: access to line states
	bool    m_see_switches;

	// IRQ state
	int    m_irq;

	// DMA in Progress state
	int    m_dip;

	// When true, motor monoflop is high
	bool    m_motor_running;

	// Address in card area
	bool m_inDsrArea;

	// HDC selected
	bool m_HDCsel;

	// RTC selected
	bool m_RTCsel;

	// Tape selected
	bool m_tapesel;

	// RAM selected
	bool m_RAMsel;

	// RAM selected
	bool m_ROMsel;

	// Recent address
	int m_address;

	// Wait for HD. This was an addition in later cards.
	bool m_wait_for_hd1;

	// Device Service Routine ROM (firmware)
	uint8_t*  m_dsrrom;

	// ROM banks.
	int     m_rom_page;

	// HFDC on-board SRAM (8K or 32K)
	required_device<ram_device> m_buffer_ram;

	// RAM page registers
	int     m_ram_page[4];

	// Drive status latch (STB0)
	uint8_t   m_status_latch;

	// DMA address latch (in Gate Array) (STB1)
	uint32_t  m_dma_address;

	// Output 1 latch (STB2)
	uint8_t   m_output1_latch;

	// Output 2 latch (STB3)
	uint8_t   m_output2_latch;

	// Needed for triggering the motor monoflop
	uint8_t m_lastval;

	// Signal motor_on. When true, makes all drives turning.
	int m_MOTOR_ON;

	// Calculates the index from the bit
	int bit_to_index(int value);

	// Utility function to set or unset bits in a byte
	void set_bits(uint8_t& byte, int mask, bool set);

	// Joined ready line towards the controller
	int  m_readyflags;
};

} } } // end namespace bus::ti99::peb

DECLARE_DEVICE_TYPE_NS(TI99_HFDC, bus::ti99::peb, myarc_hfdc_device)

#endif // MAME_BUS_TI99_PEB_HFDC_H
