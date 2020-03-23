// license:BSD-3-Clause
// copyright-holders:Nathan Woods
/*********************************************************************

    cococart.h

    CoCo/Dragon cartridge management

*********************************************************************/

#ifndef MAME_BUS_COCO_COCOCART_H
#define MAME_BUS_COCO_COCOCART_H

#pragma once

#include "softlist_dev.h"

// The following are modules included by the various CoCo cartridge
// devices.  For some reason, the build system will not necessarily
// identify them as dependencies.  Adding these #include's here seems
// to rectify the problem
#include "cpu/tms7000/tms7000.h"
#include "machine/mos6551.h"
#include "machine/6850acia.h"
#include "machine/msm6242.h"
#include "machine/ds1315.h"
#include "machine/wd_fdc.h"
#include "sound/ay8910.h"
#include "sound/sp0256.h"
#include "sound/sn76496.h"
#include "formats/dmk_dsk.h"
#include "formats/sdf_dsk.h"
#include "formats/jvc_dsk.h"
#include "formats/vdk_dsk.h"


/***************************************************************************
    TYPE DEFINITIONS
***************************************************************************/

// ======================> cococart_base_update_delegate

// direct region update handler
typedef delegate<void (uint8_t *)> cococart_base_update_delegate;

#define MCFG_COCO_CARTRIDGE_CART_CB(_devcb) \
	devcb = &cococart_slot_device::static_set_cart_callback(*device, DEVCB_##_devcb);

#define MCFG_COCO_CARTRIDGE_NMI_CB(_devcb) \
	devcb = &cococart_slot_device::static_set_nmi_callback(*device, DEVCB_##_devcb);

#define MCFG_COCO_CARTRIDGE_HALT_CB(_devcb) \
	devcb = &cococart_slot_device::static_set_halt_callback(*device, DEVCB_##_devcb);


// ======================> cococart_slot_device
class device_cococart_interface;

class cococart_slot_device final : public device_t,
								public device_slot_interface,
								public device_image_interface
{
public:
	// output lines on the CoCo cartridge slot
	enum class line
	{
		CART,             // connects to PIA1 CB1
		NMI,              // connects to NMI line on CPU
		HALT,             // connects to HALT line on CPU
		SOUND_ENABLE      // sound enable
	};

	// since we have a special value "Q" - we have to use a special enum here
	enum class line_value
	{
		CLEAR,
		ASSERT,
		Q
	};

	// construction/destruction
	cococart_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	template <class Object> static devcb_base &static_set_cart_callback(device_t &device, Object &&cb) { return downcast<cococart_slot_device &>(device).m_cart_callback.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &static_set_nmi_callback(device_t &device, Object &&cb) { return downcast<cococart_slot_device &>(device).m_nmi_callback.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &static_set_halt_callback(device_t &device, Object &&cb) { return downcast<cococart_slot_device &>(device).m_halt_callback.set_callback(std::forward<Object>(cb)); }

	// device-level overrides
	virtual void device_start() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

	// image-level overrides
	virtual image_init_result call_load() override;
	virtual const software_list_loader &get_software_list_loader() const override { return rom_software_list_loader::instance(); }

	virtual iodevice_t image_type() const override { return IO_CARTSLOT; }

	virtual bool is_readable()  const override { return 1; }
	virtual bool is_writeable() const override { return 0; }
	virtual bool is_creatable() const override { return 0; }
	virtual bool must_be_loaded() const override { return 0; }
	virtual bool is_reset_on_load() const override { return 1; }
	virtual const char *image_interface() const override { return "coco_cart"; }
	virtual const char *file_extensions() const override { return "ccc,rom"; }

	// slot interface overrides
	virtual std::string get_default_card_software(get_default_card_software_hook &hook) const override;

	// reading and writing to $FF40-$FF5F
	DECLARE_READ8_MEMBER(scs_read);
	DECLARE_WRITE8_MEMBER(scs_write);

	// manipulation of cartridge lines
	void set_line_value(line line, line_value value);
	void set_line_delay(line line, int cycles);
	line_value get_line_value(line line) const;

	// hack to support twiddling the Q line
	void twiddle_q_lines();

	// cart base
	uint8_t* get_cart_base();
	void set_cart_base_update(cococart_base_update_delegate update);

private:
	// TIMER_POOL: Must be power of two
	static constexpr int TIMER_POOL = 2;

	struct coco_cartridge_line
	{
		emu_timer                   *timer[TIMER_POOL];
		int                         timer_index;
		int                         delay;
		line_value                  value;
		int                         line;
		int                         q_count;
		devcb_write_line *          callback;
	};

	// configuration
	coco_cartridge_line         m_cart_line;
	coco_cartridge_line         m_nmi_line;
	coco_cartridge_line         m_halt_line;
public:
	devcb_write_line        m_cart_callback;
	devcb_write_line            m_nmi_callback;
	devcb_write_line            m_halt_callback;
private:
	// cartridge
	device_cococart_interface   *m_cart;

	// methods
	void set_line(const char *line_name, coco_cartridge_line &line, line_value value);
	void set_line_timer(coco_cartridge_line &line, line_value value);
	void twiddle_line_if_q(coco_cartridge_line &line);
	static const char *line_value_string(line_value value);
};

// device type definition
extern const device_type COCOCART_SLOT;
DECLARE_DEVICE_TYPE(COCOCART_SLOT, cococart_slot_device)


// ======================> device_cococart_host_interface

// this is implemented by the CoCo root device itself and the Multi-Pak interface
class device_cococart_host_interface
{
public:
	virtual address_space &cartridge_space() = 0;
};


// ======================> device_cococart_interface

class device_cococart_interface : public device_slot_card_interface
{
public:
	// construction/destruction
	virtual ~device_cococart_interface();

	virtual DECLARE_READ8_MEMBER(scs_read);
	virtual DECLARE_WRITE8_MEMBER(scs_write);
	virtual void set_sound_enable(bool sound_enable);

	virtual uint8_t* get_cart_base();
	void set_cart_base_update(cococart_base_update_delegate update);

	virtual void interface_config_complete() override;
	virtual void interface_pre_start() override;

protected:
	device_cococart_interface(const machine_config &mconfig, device_t &device);

	void cart_base_changed(void);

	// accessors for containers
	cococart_slot_device &owning_slot()     { assert(m_owning_slot); return *m_owning_slot; }
	device_cococart_host_interface &host()  { assert(m_host); return *m_host; }

	// CoCo cartridges can read directly from the address bus.  This is used by a number of
	// cartridges (e.g. - Orch-90, Multi-Pak interface) for their control registers, independently
	// of the SCS or CTS lines
	address_space &cartridge_space();
	void install_read_handler(uint16_t addrstart, uint16_t addrend, read8_delegate rhandler);
	void install_write_handler(uint16_t addrstart, uint16_t addrend, write8_delegate whandler);
	void install_readwrite_handler(uint16_t addrstart, uint16_t addrend, read8_delegate rhandler, write8_delegate whandler);

	// setting line values
	void set_line_value(cococart_slot_device::line line, cococart_slot_device::line_value value);
	void set_line_value(cococart_slot_device::line line, bool value) { set_line_value(line, value ? cococart_slot_device::line_value::ASSERT : cococart_slot_device::line_value::CLEAR); }

	typedef cococart_slot_device::line line;
	typedef cococart_slot_device::line_value line_value;

private:
	cococart_base_update_delegate    m_update;
	cococart_slot_device *           m_owning_slot;
	device_cococart_host_interface * m_host;
};


/***************************************************************************
    DEVICE CONFIGURATION MACROS
***************************************************************************/

#define MCFG_COCO_CARTRIDGE_ADD(_tag,_slot_intf,_def_slot) \
	MCFG_DEVICE_ADD(_tag, COCOCART_SLOT, DERIVED_CLOCK(1, 1)) \
	MCFG_DEVICE_SLOT_INTERFACE(_slot_intf, _def_slot, false)

#define MCFG_COCO_CARTRIDGE_REMOVE(_tag)        \
	MCFG_DEVICE_REMOVE(_tag)


/***************************************************************************
    COCO CARTRIDGE DEVICES
***************************************************************************/

// device type definitions - CoCo FDC
extern const device_type COCO_FDC;
extern const device_type COCO_FDC_V11;
extern const device_type COCO3_HDB1;
extern const device_type CP400_FDC;

// device type definitions - Dragon FDC
extern const device_type DRAGON_FDC;
extern const device_type SDTANDY_FDC;

// device type definitions - other
extern const device_type COCO_ORCH90;
extern const device_type COCO_MULTIPAK;
extern const device_type COCO_RS232;
extern const device_type COCO_DCMODEM;
extern const device_type COCO_SSC;
extern const device_type COCO_PAK;
extern const device_type COCO_PAK_BANKED;
extern const device_type COCO_PAK_GMC;
extern const device_type COCO_T4426;
extern const device_type DRAGON_JCBSND;

#endif // MAME_BUS_COCO_COCOCART_H
