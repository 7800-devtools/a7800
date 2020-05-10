// license:BSD-3-Clause
// copyright-holders:Fabio Priuli, Mike Saarna
/***********************************************************************************************************

 A7800  XM expansion module emulation

	pokey   $450

	ym2151  $460 (R)  FM status register
		$460 (W)  FM address register
		$461 (RW) FM data register

	cntrl1	$470
		d0 rof lo on
		d1 rof hi on
		d2 0=bios,1=top slot
		d3 1=hsc on
		d4 1=pokey on
		d5 1=bank0 on 4000-5fff
		d6 1=bank1 on 6000-7fff
		d7 1=ym2151 on

	cntrl2	$478  - SALLY RAM bank 8K page multiplexer.
		d0-d3 sally ram page 0 a0-a3
		d4-d7 sally ram page 1 a0-a3

	cntrl3	$47c  - MARIA RAM bank 8K page multiplexer.
		d0-d3 maria ram page 0 a0-a3
		d4-d7 maria ram page 1 a0-a3

	cntrl4	$471
		d0 1=pia on
		d1-d3 flash bank lo a1-a3
		d4-d6 flash bank hi a1-a3
		d7 1=top slot lock

	cntrl5 	$472
		d0 1=48k ram enable
		d1 1=ram we# disabled
		d2 1=bios enabled (in test mode)
		d3 1=POKEY enable/disable locked
		d4 1=HSC enable/disable locked - cannot disable after enable
		d5 1=PAL HSC enabled, 0=NTSC HSC enabled - cannot disable after enable
 

	RAM0	$4000    $5FFF     8192 bytes
	RAM1	$6000    $7FFF     8192 bytes

***********************************************************************************************************/


#include "emu.h"
#include "xm.h"
#include "a78_carts.h"
#include "speaker.h"


//-------------------------------------------------
//  constructor
//-------------------------------------------------

DEFINE_DEVICE_TYPE(A78_XM,     a78_xm_device,     "a78_xm",     "Atari 7800 XM expansion module")


a78_xm_device::a78_xm_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock)
        : a78_rom_device(mconfig, type, tag, owner, clock)
        , m_xmslot(*this, "xm_slot")
        , m_pokey(*this, "xm_pokey")
        , m_ym(*this, "xm_ym2151")
        , m_cntrl1(0), m_cntrl2(0),m_cntrl3(0),m_cntrl4(0),m_cntrl5(0)
{
}

a78_xm_device::a78_xm_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
        : a78_xm_device(mconfig, A78_XM, tag, owner, clock)
{
}

void a78_xm_device::device_start()
{
	save_item(NAME(m_cntrl1));
	save_item(NAME(m_cntrl2));
	save_item(NAME(m_cntrl3));
	save_item(NAME(m_cntrl4));
	save_item(NAME(m_cntrl5));
}

void a78_xm_device::device_reset()
{
	m_cntrl1= 0;
	m_cntrl2= 0;
	m_cntrl3= 0;
	m_cntrl4= 0;
	m_cntrl5= 0;
}


MACHINE_CONFIG_MEMBER( a78_xm_device::device_add_mconfig )
	MCFG_A78_CARTRIDGE_ADD("xm_slot", a7800_cart, nullptr)

	MCFG_SPEAKER_STANDARD_MONO("xm_speaker")

	MCFG_SOUND_ADD("xm_pokey", POKEY, XTAL_14_31818MHz/8)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "xm_speaker", 1.00)

	MCFG_SOUND_ADD("xm_ym2151", YM2151, XTAL_14_31818MHz/4)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "xm_speaker", 1.00)
MACHINE_CONFIG_END


/*-------------------------------------------------
 mapper specific handlers
 -------------------------------------------------*/

READ8_MEMBER(a78_xm_device::read_40xx)
{
        if (BIT(m_cntrl1,5) && offset < 0x2000)
                return m_ram[ (offset&0x1fff) + (((m_cntrl2&15) * 0x2000)) ];
        else if ( BIT(m_cntrl1,6) && offset >= 0x2000 && offset < 0x4000)
                return m_ram[ (offset&0x1fff) + ((((m_cntrl2>>4)&15) * 0x2000)) ];
        else
                return m_xmslot->read_40xx(space, offset);
	// TODO: implement ROF bits in cntrl1
}

WRITE8_MEMBER(a78_xm_device::write_40xx)
{
        if (BIT(m_cntrl1, 5) && offset < 0x2000)
                m_ram[ (offset) + (((m_cntrl2&15) * 0x2000)) ] = data;
        else if (BIT(m_cntrl1,6) && offset >= 0x2000 && offset < 0x4000)
                 m_ram[ offset + ((((m_cntrl2>>4)&15) * 0x2000)) ] = data;
        else
                m_xmslot->write_40xx(space, offset, data);
	// TODO: implement ROF bits in cntrl1
}


READ8_MEMBER(a78_xm_device::read_10xx)
{
	if (BIT(m_cntrl1, 3))
		return m_nvram[offset];
	else
		return 0xff;
}

WRITE8_MEMBER(a78_xm_device::write_10xx)
{
	if (BIT(m_cntrl1, 3))
		m_nvram[offset] = data;
}

READ8_MEMBER(a78_xm_device::read_30xx)
{
	if (BIT(m_cntrl1, 3))
		return m_rom[offset];
	else
		return 0xff;
}

READ8_MEMBER(a78_xm_device::read_04xx)
{
	if (BIT(m_cntrl1, 4) && offset >= 0x50 && offset < 0x60)
		return m_pokey->read(space, offset & 0x0f);
	else if (BIT(m_cntrl1, 7) && offset >= 0x60 && offset <= 0x61)
		return m_ym->read(space, offset & 1);
	else
		return 0xff;
}

WRITE8_MEMBER(a78_xm_device::write_04xx)
{
	if (BIT(m_cntrl1, 4) && offset >= 0x50 && offset < 0x60)
		m_pokey->write(space, offset & 0x0f, data);
	else if (BIT(m_cntrl1, 7) && offset >= 0x60 && offset <= 0x61)
		m_ym->write(space, offset & 1, data);
	else if (offset == 0x70) 
		m_cntrl1 = data;
	else if (offset == 0x78)
		m_cntrl2 = data;
	else if (offset == 0x7c)
		m_cntrl3 = data;
	else if (offset == 0x71)
		m_cntrl4 = data;
	else if (offset == 0x72)
		m_cntrl5 = data;
	// else do nothing
}
