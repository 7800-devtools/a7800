// license:BSD-3-Clause
// copyright-holders:Miodrag Milanovic
/*******************************************************************************************

MicroART ATM (clone of Spectrum)

Not working because of banking issues.

*******************************************************************************************/

#include "emu.h"
#include "includes/spectrum.h"
#include "includes/spec128.h"
#include "includes/specpls3.h"

#include "sound/ay8910.h"
#include "machine/beta.h"


class atm_state : public spectrum_state
{
public:
	atm_state(const machine_config &mconfig, device_type type, const char *tag)
		: spectrum_state(mconfig, type, tag)
		, m_bank1(*this, "bank1")
		, m_bank2(*this, "bank2")
		, m_bank3(*this, "bank3")
		, m_bank4(*this, "bank4")
		, m_beta(*this, BETA_DISK_TAG)
	{ }

	DECLARE_WRITE8_MEMBER(atm_port_7ffd_w);
	DECLARE_READ8_MEMBER(beta_neutral_r);
	DECLARE_READ8_MEMBER(beta_enable_r);
	DECLARE_READ8_MEMBER(beta_disable_r);
	DECLARE_MACHINE_RESET(atm);

protected:
	required_memory_bank m_bank1;
	required_memory_bank m_bank2;
	required_memory_bank m_bank3;
	required_memory_bank m_bank4;
	required_device<beta_disk_device> m_beta;

private:
	address_space *m_program;
	uint8_t *m_p_ram;
	void atm_update_memory();
};

void atm_state::atm_update_memory()
{
	uint8_t *messram = m_ram->pointer();

	m_screen_location = messram + ((m_port_7ffd_data & 8) ? (7<<14) : (5<<14));

	m_bank4->set_base(messram + ((m_port_7ffd_data & 0x07) * 0x4000));

	if (m_beta->started() && m_beta->is_active() && !( m_port_7ffd_data & 0x10 ) )
		m_ROMSelection = 3;
	else
		/* ROM switching */
		m_ROMSelection = BIT(m_port_7ffd_data, 4) ;

	/* rom 0 is 128K rom, rom 1 is 48 BASIC */
	m_bank1->set_base(&m_p_ram[0x10000 + (m_ROMSelection<<14)]);
}

WRITE8_MEMBER(atm_state::atm_port_7ffd_w)
{
	/* disable paging */
	if (m_port_7ffd_data & 0x20)
		return;

	/* store new state */
	m_port_7ffd_data = data;

	/* update memory */
	atm_update_memory();
}

READ8_MEMBER(atm_state::beta_neutral_r)
{
	return m_program->read_byte(offset);
}

READ8_MEMBER(atm_state::beta_enable_r)
{
	if(m_ROMSelection == 1) {
		m_ROMSelection = 3;
		if (m_beta->started()) {
			m_beta->enable();
			m_bank1->set_base(&m_p_ram[0x18000]);
		}
	}
	return m_program->read_byte(offset + 0x3d00);
}

READ8_MEMBER(atm_state::beta_disable_r)
{
	if (m_beta->started() && m_beta->is_active()) {
		m_ROMSelection = BIT(m_port_7ffd_data, 4);
		m_beta->disable();
		m_bank1->set_base(&m_p_ram[0x10000 + (m_ROMSelection<<14)]);
	}
	return m_program->read_byte(offset + 0x4000);
}

static ADDRESS_MAP_START (atm_io, AS_IO, 8, atm_state )
	ADDRESS_MAP_UNMAP_HIGH
	AM_RANGE(0x001f, 0x001f) AM_DEVREADWRITE(BETA_DISK_TAG, beta_disk_device, status_r, command_w) AM_MIRROR(0xff00)
	AM_RANGE(0x003f, 0x003f) AM_DEVREADWRITE(BETA_DISK_TAG, beta_disk_device, track_r, track_w) AM_MIRROR(0xff00)
	AM_RANGE(0x005f, 0x005f) AM_DEVREADWRITE(BETA_DISK_TAG, beta_disk_device, sector_r, sector_w) AM_MIRROR(0xff00)
	AM_RANGE(0x007f, 0x007f) AM_DEVREADWRITE(BETA_DISK_TAG, beta_disk_device, data_r, data_w) AM_MIRROR(0xff00)
	AM_RANGE(0x00fe, 0x00fe) AM_READWRITE(spectrum_port_fe_r,spectrum_port_fe_w) AM_SELECT(0xff00)
	AM_RANGE(0x00ff, 0x00ff) AM_DEVREADWRITE(BETA_DISK_TAG, beta_disk_device, state_r, param_w) AM_MIRROR(0xff00)
	AM_RANGE(0x4000, 0x4000) AM_WRITE(atm_port_7ffd_w)  AM_MIRROR(0x3ffd)
	AM_RANGE(0x8000, 0x8000) AM_DEVWRITE("ay8912", ay8910_device, data_w) AM_MIRROR(0x3ffd)
	AM_RANGE(0xc000, 0xc000) AM_DEVREADWRITE("ay8912", ay8910_device, data_r, address_w) AM_MIRROR(0x3ffd)
ADDRESS_MAP_END

static ADDRESS_MAP_START (atm_switch, AS_OPCODES, 8, atm_state)
	AM_RANGE(0x3d00, 0x3dff) AM_READ(beta_enable_r)
	AM_RANGE(0x0000, 0x3fff) AM_READ(beta_neutral_r) // Overlap with previous because we want real addresses on the 3e00-3fff range
	AM_RANGE(0x4000, 0xffff) AM_READ(beta_disable_r)
ADDRESS_MAP_END

MACHINE_RESET_MEMBER(atm_state,atm)
{
	uint8_t *messram = m_ram->pointer();
	m_program = &m_maincpu->space(AS_PROGRAM);
	m_p_ram = memregion("maincpu")->base();

	if (m_beta->started())
		m_beta->enable();

	memset(messram,0,128*1024);

	/* Bank 5 is always in 0x4000 - 0x7fff */
	m_bank2->set_base(messram + (5<<14));

	/* Bank 2 is always in 0x8000 - 0xbfff */
	m_bank3->set_base(messram + (2<<14));

	m_port_7ffd_data = 0;
	m_port_1ffd_data = -1;

	atm_update_memory();
}

/* F4 Character Displayer */
static const gfx_layout spectrum_charlayout =
{
	8, 8,                   /* 8 x 8 characters */
	96,                 /* 96 characters */
	1,                  /* 1 bits per pixel */
	{ 0 },                  /* no bitplanes */
	/* x offsets */
	{ 0, 1, 2, 3, 4, 5, 6, 7 },
	/* y offsets */
	{ 0*8, 1*8, 2*8, 3*8, 4*8, 5*8, 6*8, 7*8 },
	8*8                 /* every char takes 8 bytes */
};

static GFXDECODE_START( atm )
	GFXDECODE_ENTRY( "maincpu", 0x1fd00, spectrum_charlayout, 0, 8 )
GFXDECODE_END

static GFXDECODE_START( atmtb2 )
	GFXDECODE_ENTRY( "maincpu", 0x13d00, spectrum_charlayout, 0, 8 )
GFXDECODE_END


static MACHINE_CONFIG_DERIVED( atm, spectrum_128 )
	MCFG_CPU_MODIFY("maincpu")
	MCFG_CPU_IO_MAP(atm_io)
	MCFG_CPU_DECRYPTED_OPCODES_MAP(atm_switch)
	MCFG_MACHINE_RESET_OVERRIDE(atm_state, atm )

	MCFG_BETA_DISK_ADD(BETA_DISK_TAG)

	MCFG_GFXDECODE_MODIFY("gfxdecode", atm)
MACHINE_CONFIG_END

static MACHINE_CONFIG_DERIVED( atmtb2, atm )
	MCFG_GFXDECODE_MODIFY("gfxdecode", atmtb2)
MACHINE_CONFIG_END


/***************************************************************************

  Game driver(s)

***************************************************************************/

ROM_START( atm )
	ROM_REGION(0x020000, "maincpu", ROMREGION_ERASEFF)
	ROM_SYSTEM_BIOS(0, "v1", "v.1.03")
	ROMX_LOAD( "atm103.rom", 0x010000, 0x10000, CRC(4912e249) SHA1(a4adff05bb215dd126c47201b36956115b8fed76), ROM_BIOS(1))
	ROM_SYSTEM_BIOS(1, "v2", "v.1.06 joined")
	ROMX_LOAD( "atm106.rom", 0x010000, 0x10000, CRC(75350b37) SHA1(2afc9994f026645c74b6c4b35bcee2e0bc0d6edc), ROM_BIOS(2))
	ROM_SYSTEM_BIOS(2, "v3", "v.1.06")
	ROMX_LOAD( "atm106-1.rom", 0x010000, 0x4000, CRC(658c98f1) SHA1(1ec694795aa6cac10147e58f38a9db0bdf7ed89b), ROM_BIOS(3))
	ROMX_LOAD( "atm106-2.rom", 0x014000, 0x4000, CRC(8fe367f9) SHA1(56de8fd39061663b9c315b74fd3c31acddae279c), ROM_BIOS(3))
	ROMX_LOAD( "atm106-3.rom", 0x018000, 0x4000, CRC(124ad9e0) SHA1(d07fcdeca892ee80494d286ea9ea5bf3928a1aca), ROM_BIOS(3))
	ROMX_LOAD( "atm106-4.rom", 0x01c000, 0x4000, CRC(f352f2ab) SHA1(6045500ab01be708cef62327e9821b4a358a4673), ROM_BIOS(3))
	ROM_SYSTEM_BIOS(3, "v4", "v.1.03rs")
	ROMX_LOAD( "atm103rs.rom", 0x010000, 0x10000, CRC(cdec1dfb) SHA1(08190807c6b110cb2e657d8e7d0ad18668915375), ROM_BIOS(4))
ROM_END

ROM_START( atmtb2 )
	ROM_REGION(0x020000, "maincpu", ROMREGION_ERASEFF)
	ROM_SYSTEM_BIOS(0, "v1", "v.1.07.12 joined")
	ROMX_LOAD( "atmtb2.rom",   0x010000, 0x10000,CRC(05218c26) SHA1(71ed9864e7aa85131de97cf1e53dc152e7c79488), ROM_BIOS(1))
	ROM_SYSTEM_BIOS(1, "v2", "v.1.07.12")
	ROMX_LOAD( "atmtb2-1.rom", 0x010000, 0x4000, CRC(658c98f1) SHA1(1ec694795aa6cac10147e58f38a9db0bdf7ed89b), ROM_BIOS(2))
	ROMX_LOAD( "atmtb2-2.rom", 0x014000, 0x4000, CRC(bc3f6b2b) SHA1(afa9df63857141fef270e2c97e12d2edc60cf919), ROM_BIOS(2))
	ROMX_LOAD( "atmtb2-3.rom", 0x018000, 0x4000, CRC(124ad9e0) SHA1(d07fcdeca892ee80494d286ea9ea5bf3928a1aca), ROM_BIOS(2))
	ROMX_LOAD( "atmtb2-4.rom", 0x01c000, 0x4000, CRC(5869d8c4) SHA1(c3e198138f528ac4a8dff3c76cd289fd4713abff), ROM_BIOS(2))
	ROM_SYSTEM_BIOS(2, "v3", "v.1.07.13")
	ROMX_LOAD( "atmtb213.rom", 0x010000, 0x10000, CRC(34a91d53) SHA1(8f0af0f3c0ff1644535f20545c73d01576d6e52f), ROM_BIOS(3))

	ROM_REGION(0x01000, "keyboard", ROMREGION_ERASEFF)
	// XT Keyboard
	ROM_LOAD( "rf2ve3.rom",  0x0000, 0x0580, CRC(35e0f9ec) SHA1(adcf14758fab8472cfa0167af7e8326c66416416))
	// AT Keyboard
	ROM_LOAD( "rfat710.rom", 0x0600, 0x0680, CRC(03734365) SHA1(6cb6311727fad9bc4ccb18919c3c39b37529b8e6))
	ROM_REGION(0x08000, "charrom", ROMREGION_ERASEFF)
	// Char gen rom
	ROM_LOAD( "sgen.rom", 0x0000, 0x0800, CRC(1f4387d6) SHA1(93b3774dc8a486643a1bdd48c606b0c84fa0e22b))
ROM_END

/*    YEAR  NAME    PARENT   COMPAT  MACHINE INPUT      CLASS      INIT  COMPANY     FULLNAME */
COMP( 1991, atm,    spec128, 0,      atm,    spec_plus, atm_state, 0,    "MicroART", "ATM", MACHINE_NOT_WORKING)
//COMP( 1991, atmtb1, spec128, 0,      atm,    spec_plus, atm_state, 0,    "MicroART", "ATM-turbo1", MACHINE_NOT_WORKING)
COMP( 1993, atmtb2, spec128, 0,      atmtb2, spec_plus, atm_state, 0,    "MicroART", "ATM-turbo2", MACHINE_NOT_WORKING)
//COMP( 1994, turbo2, spec128, 0,      atm,    spec_plus, atm_state, 0,    "MicroART", "TURBO 2+", MACHINE_NOT_WORKING)
