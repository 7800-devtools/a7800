// license:BSD-3-Clause
// copyright-holders:Sandro Ronco
/*********************************************************************

    iq151cart.h

    IQ151 cartridge slot pinout:

                     +--------+
        +12V         | 01  32 |   IO     /MW
        +12V         | 02  33 |   IO     /IOR
        +5V          | 03  34 |   IO     /IOW
        +5V          | 04  35 |    O     /NRDY
        GND          | 05  36 |    O     /HOLD
        GND          | 06  37 |   I      HLDA
        A0      IO   | 07  38 |    O     /RAM
        A1      IO   | 08  39 |    O     /INT0
        A2      IO   | 09  40 |    O     /INT1
        A3      IO   | 10  41 |    O     /INT2
        A4      IO   | 11  42 |    O     /INT3
        A5      IO   | 12  43 |    O     /INT4
        A6      IO   | 13  44 |    O     /VID
        A7      IO   | 14  45 |   I       OSC
        A8      IO   | 15  46 |   I       TTL
        A9      IO   | 16  47 |    O      NF
        A10     IO   | 17  48 |           N.C.
        A11     IO   | 18  49 |           N.C.
        A12     IO   | 19  50 |   I       /INIT
        A13     IO   | 20  51 |   IO      /SS
        A14     IO   | 21  52 |   IO      /SR
        A15     IO   | 22  53 |           N.C.
        D0      IO   | 23  54 |   IO      /ZS
        D1      IO   | 24  55 |   IO      /ZR
        D2      IO   | 25  56 |    O      /DMA
        D3      IO   | 26  57 |           GND
        D4      IO   | 27  58 |           GND
        D5      IO   | 28  59 |           -5V
        D6      IO   | 29  60 |           -5V
        D7      IO   | 30  61 |           -12V
        /MR     IO   | 31  62 |           -12V
                     +--------+

*********************************************************************/

#ifndef MAME_BUS_IQ151_IQ151_H
#define MAME_BUS_IQ151_IQ151_H

#pragma once

#include "softlist_dev.h"

/***************************************************************************
    TYPE DEFINITIONS
***************************************************************************/

// ======================> device_iq151cart_interface

class device_iq151cart_interface : public device_slot_card_interface
{
public:
	// construction/destruction
	virtual ~device_iq151cart_interface();

	// reading and writing
	virtual void read(offs_t offset, uint8_t &data) { }
	virtual void write(offs_t offset, uint8_t data) { }
	virtual void io_read(offs_t offset, uint8_t &data) { }
	virtual void io_write(offs_t offset, uint8_t data) { }
	virtual uint8_t* get_cart_base() { return nullptr; }

	// video update
	virtual void video_update(bitmap_ind16 &bitmap, const rectangle &cliprect) { }

protected:
	device_iq151cart_interface(const machine_config &mconfig, device_t &device);
};

// ======================> iq151cart_slot_device

class iq151cart_slot_device : public device_t,
								public device_slot_interface,
								public device_image_interface
{
public:
	// construction/destruction
	iq151cart_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	virtual ~iq151cart_slot_device();

	template <class Object> static devcb_base &set_out_irq0_callback(device_t &device, Object &&cb) { return downcast<iq151cart_slot_device &>(device).m_out_irq0_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_out_irq1_callback(device_t &device, Object &&cb) { return downcast<iq151cart_slot_device &>(device).m_out_irq1_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_out_irq2_callback(device_t &device, Object &&cb) { return downcast<iq151cart_slot_device &>(device).m_out_irq2_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_out_irq3_callback(device_t &device, Object &&cb) { return downcast<iq151cart_slot_device &>(device).m_out_irq3_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_out_irq4_callback(device_t &device, Object &&cb) { return downcast<iq151cart_slot_device &>(device).m_out_irq4_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_out_drq_callback(device_t &device, Object &&cb) { return downcast<iq151cart_slot_device &>(device).m_out_drq_cb.set_callback(std::forward<Object>(cb)); }

	// device-level overrides
	virtual void device_start() override;

	// image-level overrides
	virtual image_init_result call_load() override;
	virtual const software_list_loader &get_software_list_loader() const override { return rom_software_list_loader::instance(); }

	virtual iodevice_t image_type() const override { return IO_CARTSLOT; }
	virtual bool is_readable()  const override { return 1; }
	virtual bool is_writeable() const override { return 0; }
	virtual bool is_creatable() const override { return 0; }
	virtual bool must_be_loaded() const override { return 0; }
	virtual bool is_reset_on_load() const override { return 1; }
	virtual const char *image_interface() const override { return "iq151_cart"; }
	virtual const char *file_extensions() const override { return "bin,rom"; }

	// slot interface overrides
	virtual std::string get_default_card_software(get_default_card_software_hook &hook) const override;

	// reading and writing
	virtual void read(offs_t offset, uint8_t &data);
	virtual void write(offs_t offset, uint8_t data);
	virtual void io_read(offs_t offset, uint8_t &data);
	virtual void io_write(offs_t offset, uint8_t data);
	virtual void video_update(bitmap_ind16 &bitmap, const rectangle &cliprect);

	devcb_write_line                m_out_irq0_cb;
	devcb_write_line                m_out_irq1_cb;
	devcb_write_line                m_out_irq2_cb;
	devcb_write_line                m_out_irq3_cb;
	devcb_write_line                m_out_irq4_cb;
	devcb_write_line                m_out_drq_cb;

	device_iq151cart_interface* m_cart;
};


// device type definition
DECLARE_DEVICE_TYPE(IQ151CART_SLOT, iq151cart_slot_device)


/***************************************************************************
    DEVICE CONFIGURATION MACROS
***************************************************************************/

#define MCFG_IQ151CART_SLOT_OUT_IRQ0_CB(_devcb) \
	devcb = &iq151cart_slot_device::set_out_irq0_callback(*device, DEVCB_##_devcb);

#define MCFG_IQ151CART_SLOT_OUT_IRQ1_CB(_devcb) \
	devcb = &iq151cart_slot_device::set_out_irq1_callback(*device, DEVCB_##_devcb);

#define MCFG_IQ151CART_SLOT_OUT_IRQ2_CB(_devcb) \
	devcb = &iq151cart_slot_device::set_out_irq2_callback(*device, DEVCB_##_devcb);

#define MCFG_IQ151CART_SLOT_OUT_IRQ3_CB(_devcb) \
	devcb = &iq151cart_slot_device::set_out_irq3_callback(*device, DEVCB_##_devcb);

#define MCFG_IQ151CART_SLOT_OUT_IRQ4_CB(_devcb) \
	devcb = &iq151cart_slot_device::set_out_irq4_callback(*device, DEVCB_##_devcb);

#define MCFG_IQ151CART_SLOT_OUT_DRQ_CB(_devcb) \
	devcb = &iq151cart_slot_device::set_out_drq_callback(*device, DEVCB_##_devcb);

#endif // MAME_BUS_IQ151_IQ151_H
