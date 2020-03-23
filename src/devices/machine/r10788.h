// license:BSD-3-Clause
// copyright-holders:Juergen Buchmueller
/**********************************************************************

    Rockwell 10788 General Purpose Keyboard and Display circuit

    Juergen Buchmueller <pullmoll@t-online.de>

    The device decodes reads/write to a 16 byte I/O range defined
    by three wired inputs SC5, SC6 and SC7. The range is one of
    80-8f, 90-9f, ..., f0-ff depending on the wiring.

**********************************************************************/

#ifndef MAME_MACHINE_R10788_H
#define MAME_MACHINE_R10788_H

#pragma once

#include "device.h"

/*************************************
 *
 *  Device configuration macros
 *
 *************************************/

/* Set the writer used to update a display digit */
#define MCFG_R10788_UPDATE(devcb) \
		r10788_device::set_update(*device, DEVCB_##devcb);

class r10788_device : public device_t
{
public:
	r10788_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	enum {
		KTR = 0xc,  //!< Transfer Keyboard Return
		KTS = 0xa,  //!< Transfer Keyboard Strobe
		KLA = 0xe,  //!< Load Display Register A
		KLB = 0xd,  //!< Load Display Register B
		KDN = 0x3,  //!< Turn On Display
		KAF = 0xb,  //!< Turn Off A
		KBF = 0x7,  //!< Turn Off B
		KER = 0x6   //!< Reset Keyboard Error
	};

	DECLARE_READ8_MEMBER ( io_r );
	DECLARE_WRITE8_MEMBER( io_w );

	template <class Object> static devcb_base &set_update(device_t &device, Object &&cb) { return downcast<r10788_device &>(device).m_display.set_callback(std::forward<Object>(cb)); }

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

private:
	static const device_timer_id TIMER_DISPLAY = 0;

	uint8_t        m_reg[2][16];          //!< display registers
	uint8_t        m_ktr;                 //!< transfer keyboard return value
	uint8_t        m_kts;                 //!< transfer keyboard strobe value
	uint8_t        m_kla;                 //!< display register A value
	uint8_t        m_klb;                 //!< display register B value
	uint8_t        m_mask_a;              //!< display enable bits for A
	uint8_t        m_mask_b;              //!< display enable bits for B
	uint8_t        m_ker;                 //!< keyboard error value
	int          m_io_counter;          //!< current I/O register index
	int          m_scan_counter;        //!< current display scan
	devcb_write8 m_display;             //!< display updater
	emu_timer*   m_timer;               //!< timer running at clock / 18 / 36
};

DECLARE_DEVICE_TYPE(R10788, r10788_device)

#endif // MAME_MACHINE_R10788_H
