// license:BSD-3-Clause
// copyright-holders:Sandro Ronco
/*********************************************************************

    z88.h

    Z88 cartridge slots emulation

**********************************************************************

    pins    Slot 1  Slot 2  Slot 3

    1       A16     A16     A16
    2       A15     A15     A15
    3       A12     A12     A12
    4       A7      A7      A7
    5       A6      A6      A6
    6       A5      A5      A5
    7       A4      A4      A4
    8       A3      A3      A3
    9       A2      A2      A2
    10      A1      A1      A1
    11      A0      A0      A0
    12      D0      D0      D0
    13      D1      D1      D1
    14      D2      D2      D2
    15      SNSL    SNSL    SNSL
    16      GND     GND     GND
    17      GND     GND     GND
    18      A14     A14     A14
    19      VCC     VCC     VPP
    20      VCC     VCC     VCC
    21      VCC     VCC     VCC
    22      WEL     WEL     PGML
    23      A13     A13     A13
    24      A8      A8      A8
    25      A9      A9      A9
    26      A11     A11     A11
    27      POE     POE     POE
    28      ROE     ROE     EOE
    29      A10     A10     A10
    30      SE1     SE2     SE3
    31      D7      D7      D7
    32      D6      D6      D6
    33      D3      D3      D3
    34      D4      D4      D4
    35      D5      D5      D5
    36      A17     A17     A17
    37      A18     A18     A18
    38      A19     A19     A19

*********************************************************************/

#ifndef MAME_BUS_Z88_Z88_H
#define MAME_BUS_Z88_Z88_H

#pragma once

#include "softlist_dev.h"


/***************************************************************************
    TYPE DEFINITIONS
***************************************************************************/

// ======================> device_z88cart_interface

class device_z88cart_interface : public device_slot_card_interface
{
public:
	// construction/destruction
	virtual ~device_z88cart_interface();

	// reading and writing
	virtual DECLARE_READ8_MEMBER(read) { return 0xff; }
	virtual DECLARE_WRITE8_MEMBER(write) { }
	virtual uint8_t* get_cart_base() { return nullptr; }
	virtual uint32_t get_cart_size() { return 0; }

protected:
	device_z88cart_interface(const machine_config &mconfig, device_t &device);
};


// ======================> z88cart_slot_device

class z88cart_slot_device : public device_t,
							public device_image_interface,
							public device_slot_interface
{
public:
	// construction/destruction
	z88cart_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	virtual ~z88cart_slot_device();

	template <class Object> static devcb_base &set_out_flp_callback(device_t &device, Object &&cb) { return downcast<z88cart_slot_device &>(device).m_out_flp_cb.set_callback(std::forward<Object>(cb)); }

	// image-level overrides
	virtual image_init_result call_load() override;
	virtual void call_unload() override;
	virtual const software_list_loader &get_software_list_loader() const override { return rom_software_list_loader::instance(); }

	virtual iodevice_t image_type() const override { return IO_CARTSLOT; }
	virtual bool is_readable()  const override { return 1; }
	virtual bool is_writeable() const override { return 1; }
	virtual bool is_creatable() const override { return 1; }
	virtual bool must_be_loaded() const override { return 0; }
	virtual bool is_reset_on_load() const override { return 0; }
	virtual const char *image_interface() const override { return "z88_cart"; }
	virtual const char *file_extensions() const override { return "epr,bin"; }

	// slot interface overrides
	virtual std::string get_default_card_software(get_default_card_software_hook &hook) const override;

	// reading and writing
	virtual DECLARE_READ8_MEMBER(read);
	virtual DECLARE_WRITE8_MEMBER(write);
	virtual uint8_t* get_cart_base();

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

private:
	static constexpr device_timer_id TIMER_FLP_CLEAR = 0;

	devcb_write_line               m_out_flp_cb;
	device_z88cart_interface*       m_cart;
	emu_timer *                     m_flp_timer;
};


// device type definition
DECLARE_DEVICE_TYPE(Z88CART_SLOT, z88cart_slot_device)


/***************************************************************************
    DEVICE CONFIGURATION MACROS
***************************************************************************/

#define MCFG_Z88CART_SLOT_OUT_FLP_CB(_devcb) \
		devcb = &z88cart_slot_device::set_out_flp_callback(*device, DEVCB_##_devcb);

#endif // MAME_BUS_Z88_Z88_H
