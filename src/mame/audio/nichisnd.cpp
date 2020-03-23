// license:BSD-3-Clause
// copyright-holders:Angelo Salese,Takahiro Nogi
/***************************************************************************

    Nichibutsu sound HW

    Shared component between niyanpai.cpp and csplayh5.cpp

    Uses a TMPZ84C011 with YM3812 and two DACs

    TODO:
    - DVD sound routing in here

***************************************************************************/

#include "emu.h"
#include "nichisnd.h"



//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

// device type definition
DEFINE_DEVICE_TYPE(NICHISND, nichisnd_device, "nichisnd", "Nichibutsu Sound Device")


//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  nichisnd_device - constructor
//-------------------------------------------------

nichisnd_device::nichisnd_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, NICHISND, tag, owner, clock),
	m_soundlatch(*this, "soundlatch"),
	m_sound_rom(*this, "audiorom")
{
}

static ADDRESS_MAP_START( nichisnd_map, AS_PROGRAM, 8, nichisnd_device )
	AM_RANGE(0x0000, 0x77ff) AM_ROM AM_REGION("audiorom",0)
	AM_RANGE(0x7800, 0x7fff) AM_RAM
	AM_RANGE(0x8000, 0xffff) AM_ROMBANK("soundbank")
ADDRESS_MAP_END

static ADDRESS_MAP_START( nichisnd_io_map, AS_IO, 8, nichisnd_device )
	AM_RANGE(0x80, 0x81) AM_MIRROR(0xff00) AM_DEVWRITE("ymsnd", ym3812_device, write)
ADDRESS_MAP_END


WRITE8_MEMBER(nichisnd_device::soundbank_w)
{
	membank("soundbank")->set_entry(data & 0x03);
}

WRITE8_MEMBER(nichisnd_device::soundlatch_clear_w)
{
	if (!(data & 0x01)) m_soundlatch->clear_w(space, 0, 0);
}


//-------------------------------------------------
//  add_device_mconfig - device-specific machine
//  configuration addiitons
//-------------------------------------------------

static const z80_daisy_config daisy_chain[] =
{
	TMPZ84C011_DAISY_INTERNAL,
	{ nullptr }
};



MACHINE_CONFIG_MEMBER(nichisnd_device::device_add_mconfig)
	MCFG_CPU_ADD("audiocpu", TMPZ84C011, 8000000) /* TMPZ84C011, 8.00 MHz */
	MCFG_Z80_DAISY_CHAIN(daisy_chain)
	MCFG_CPU_PROGRAM_MAP(nichisnd_map)
	MCFG_CPU_IO_MAP(nichisnd_io_map)
	MCFG_TMPZ84C011_PORTD_READ_CB(DEVREAD8("soundlatch", generic_latch_8_device, read))
	MCFG_TMPZ84C011_PORTA_WRITE_CB(WRITE8(nichisnd_device, soundbank_w))
	MCFG_TMPZ84C011_PORTB_WRITE_CB(DEVWRITE8("dac1", dac_byte_interface, write))
	MCFG_TMPZ84C011_PORTC_WRITE_CB(DEVWRITE8("dac2", dac_byte_interface, write))
	MCFG_TMPZ84C011_PORTE_WRITE_CB(WRITE8(nichisnd_device, soundlatch_clear_w))
	MCFG_TMPZ84C011_ZC0_CB(DEVWRITELINE("audiocpu", tmpz84c011_device, trg3))

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("speaker")

	MCFG_GENERIC_LATCH_8_ADD("soundlatch")

	MCFG_SOUND_ADD("ymsnd", YM3812, 4000000)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "speaker", 1.0)

	MCFG_SOUND_ADD("dac1", DAC_8BIT_R2R, 0) MCFG_SOUND_ROUTE(ALL_OUTPUTS, "speaker", 0.37) // unknown DAC
	MCFG_SOUND_ADD("dac2", DAC_8BIT_R2R, 0) MCFG_SOUND_ROUTE(ALL_OUTPUTS, "speaker", 0.37) // unknown DAC
	MCFG_DEVICE_ADD("vref", VOLTAGE_REGULATOR, 0) MCFG_VOLTAGE_REGULATOR_OUTPUT(5.0)
	MCFG_SOUND_ROUTE_EX(0, "dac1", 1.0, DAC_VREF_POS_INPUT) MCFG_SOUND_ROUTE_EX(0, "dac1", -1.0, DAC_VREF_NEG_INPUT)
	MCFG_SOUND_ROUTE_EX(0, "dac2", 1.0, DAC_VREF_POS_INPUT) MCFG_SOUND_ROUTE_EX(0, "dac2", -1.0, DAC_VREF_NEG_INPUT)
MACHINE_CONFIG_END


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void nichisnd_device::device_start()
{
	uint8_t *SNDROM = m_sound_rom;

	// sound program patch
	SNDROM[0x0213] = 0x00;          // DI -> NOP

	// initialize sound rom bank
	membank("soundbank")->configure_entries(0, 3, m_sound_rom + 0x8000, 0x8000);
	membank("soundbank")->set_entry(0);
}


//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void nichisnd_device::device_reset()
{
}


//**************************************************************************
//  READ/WRITE HANDLERS
//**************************************************************************

// use this to connect to the sound board
WRITE8_MEMBER(nichisnd_device::sound_host_command_w)
{
	m_soundlatch->write(space,0,data);
}
