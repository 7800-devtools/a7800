// license:BSD-3-Clause
// copyright-holders:Curt Coder
#ifndef __SG1000__
#define __SG1000__

#include "cpu/z80/z80.h"
#include "formats/sf7000_dsk.h"
#include "imagedev/printer.h"
#include "bus/centronics/ctronics.h"
#include "machine/i8255.h"
#include "machine/i8251.h"
#include "machine/ram.h"
#include "bus/sega8/sega8_slot.h"
#include "bus/sg1000_exp/sg1000exp.h"
#include "machine/upd765.h"
#include "sound/sn76496.h"
#include "video/tms9928a.h"
#include "crsshair.h"

#define SCREEN_TAG      "screen"
#define Z80_TAG         "z80"
#define SN76489AN_TAG   "sn76489an"
#define UPD765_TAG      "upd765"
#define UPD8251_TAG     "upd8251"
#define UPD9255_TAG     "upd9255"
#define UPD9255_1_TAG   "upd9255_1" // "upd9255_0" is being used by sk1100 device
#define CENTRONICS_TAG  "centronics"
#define TMS9918A_TAG    "tms9918a"
#define RS232_TAG       "rs232"
#define CARTSLOT_TAG    "slot"
#define EXPSLOT_TAG     "sgexp"



class sg1000_state : public driver_device
{
public:
	enum
	{
		TIMER_LIGHTGUN_TICK
	};

	sg1000_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
			m_maincpu(*this, Z80_TAG),
			m_ram(*this, RAM_TAG),
			m_rom(*this, Z80_TAG),
			m_cart(*this, CARTSLOT_TAG),
			m_sgexpslot(*this, EXPSLOT_TAG),
			m_pa7(*this, "PA7"),
			m_pb7(*this, "PB7")
	{ }

	required_device<cpu_device> m_maincpu;
	required_device<ram_device> m_ram;
	required_memory_region m_rom;
	optional_device<sega8_cart_slot_device> m_cart;
	optional_device<sg1000_expansion_slot_device> m_sgexpslot;
	optional_ioport m_pa7;
	optional_ioport m_pb7;

	virtual void machine_start() override;

	DECLARE_READ8_MEMBER( peripheral_r );
	DECLARE_WRITE8_MEMBER( peripheral_w );
	DECLARE_INPUT_CHANGED_MEMBER( trigger_nmi );

	DECLARE_READ8_MEMBER( omv_r );
	DECLARE_WRITE8_MEMBER( omv_w );
};

class sc3000_state : public sg1000_state
{
public:
	sc3000_state(const machine_config &mconfig, device_type type, const char *tag)
		: sg1000_state(mconfig, type, tag)
	{ }

	virtual void machine_start() override;
};

class sf7000_state : public sc3000_state
{
public:
	sf7000_state(const machine_config &mconfig, device_type type, const char *tag)
		: sc3000_state(mconfig, type, tag),
			m_fdc(*this, UPD765_TAG),
			m_centronics(*this, CENTRONICS_TAG),
			m_floppy0(*this, UPD765_TAG ":0:3ssdd")
	{ }

	required_device<upd765a_device> m_fdc;
	required_device<centronics_device> m_centronics;
	required_device<floppy_image_device> m_floppy0;

	virtual void machine_start() override;
	virtual void machine_reset() override;

	int m_centronics_busy;
	DECLARE_WRITE_LINE_MEMBER( write_centronics_busy );
	DECLARE_READ8_MEMBER( ppi_pa_r );
	DECLARE_WRITE8_MEMBER( ppi_pc_w );

	DECLARE_FLOPPY_FORMATS( floppy_formats );
};

#endif
