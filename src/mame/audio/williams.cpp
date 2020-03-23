// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/***************************************************************************

    williams.h

    Functions to emulate general the various Williams/Midway sound cards.

****************************************************************************

    Midway/Williams Audio Boards
    ----------------------------

    6809 MEMORY MAP

    Function                                  Address     R/W  Data
    ---------------------------------------------------------------
    Program RAM                               0000-07FF   R/W  D0-D7

    Music (YM-2151)                           2000-2001   R/W  D0-D7

    6821 PIA                                  4000-4003   R/W  D0-D7

    HC55516 clock low, digit latch            6000        W    D0
    HC55516 clock high                        6800        W    xx

    Bank select                               7800        W    D0-D2

    Banked Program ROM                        8000-FFFF   R    D0-D7

****************************************************************************/

#include "emu.h"
#include "williams.h"
#include "machine/6821pia.h"
#include "cpu/m6809/m6809.h"
#include "sound/ym2151.h"
#include "sound/okim6295.h"
#include "sound/hc55516.h"
#include "sound/dac.h"
#include "sound/volt_reg.h"


#define NARC_MASTER_CLOCK       XTAL_8MHz
#define NARC_FM_CLOCK           XTAL_3_579545MHz

#define CVSD_MASTER_CLOCK       XTAL_8MHz
#define CVSD_FM_CLOCK           XTAL_3_579545MHz

#define ADPCM_MASTER_CLOCK      XTAL_8MHz
#define ADPCM_FM_CLOCK          XTAL_3_579545MHz



//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

DEFINE_DEVICE_TYPE(WILLIAMS_CVSD_SOUND, williams_cvsd_sound_device, "wmscvsd", "Williams CVSD Sound Board")
DEFINE_DEVICE_TYPE(WILLIAMS_NARC_SOUND, williams_narc_sound_device, "wmsnarc", "Williams NARC Sound Board")
DEFINE_DEVICE_TYPE(WILLIAMS_ADPCM_SOUND, williams_adpcm_sound_device, "wmsadpcm", "Williams ADPCM Sound Board")



//**************************************************************************
//  CVSD SOUND BOARD
//**************************************************************************

//-------------------------------------------------
//  williams_cvsd_sound_device - constructor
//-------------------------------------------------

williams_cvsd_sound_device::williams_cvsd_sound_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, WILLIAMS_CVSD_SOUND, tag, owner, clock),
		device_mixer_interface(mconfig, *this),
		m_cpu(*this, "cpu"),
		m_pia(*this, "pia"),
		m_hc55516(*this, "cvsd"),
		m_talkback(0)
{
}


//-------------------------------------------------
//  write - handle an external write to the input
//  latch
//-------------------------------------------------

WRITE16_MEMBER(williams_cvsd_sound_device::write)
{
	synchronize(0, data);
}


//-------------------------------------------------
//  reset_write - write to the reset line
//-------------------------------------------------

WRITE_LINE_MEMBER(williams_cvsd_sound_device::reset_write)
{
	// going high halts the CPU
	if (state)
	{
		bank_select_w(m_cpu->space(), 0, 0);
		device_reset();
		m_cpu->set_input_line(INPUT_LINE_RESET, ASSERT_LINE);
	}

	// going low resets and reactivates the CPU
	else
		m_cpu->set_input_line(INPUT_LINE_RESET, CLEAR_LINE);
}


//-------------------------------------------------
//  bank_select_w - change memory banks
//-------------------------------------------------

WRITE8_MEMBER(williams_cvsd_sound_device::bank_select_w)
{
	membank("rombank")->set_entry(data & 0x0f);
}


//-------------------------------------------------
//  talkback_w - write to the talkback latch
//-------------------------------------------------

WRITE8_MEMBER(williams_cvsd_sound_device::talkback_w)
{
	m_talkback = data;
	logerror("CVSD Talkback = %02X\n", data);
}


//-------------------------------------------------
//  cvsd_digit_clock_clear_w - clear the clock on
//  the HC55516 and clock the data
//-------------------------------------------------

WRITE8_MEMBER(williams_cvsd_sound_device::cvsd_digit_clock_clear_w)
{
	m_hc55516->digit_w(data);
	m_hc55516->clock_w(0);
}


//-------------------------------------------------
//  cvsd_clock_set_w - set the clock on the HC55516
//-------------------------------------------------

WRITE8_MEMBER(williams_cvsd_sound_device::cvsd_clock_set_w)
{
	m_hc55516->clock_w(1);
}


//-------------------------------------------------
//  audio CPU map
//-------------------------------------------------

static ADDRESS_MAP_START( williams_cvsd_map, AS_PROGRAM, 8, williams_cvsd_sound_device )
	AM_RANGE(0x0000, 0x07ff) AM_MIRROR(0x1800) AM_RAM
	AM_RANGE(0x2000, 0x2001) AM_MIRROR(0x1ffe) AM_DEVREADWRITE("ym2151", ym2151_device, read, write)
	AM_RANGE(0x4000, 0x4003) AM_MIRROR(0x1ffc) AM_DEVREADWRITE("pia", pia6821_device, read, write)
	AM_RANGE(0x6000, 0x6000) AM_MIRROR(0x07ff) AM_WRITE(cvsd_digit_clock_clear_w)
	AM_RANGE(0x6800, 0x6800) AM_MIRROR(0x07ff) AM_WRITE(cvsd_clock_set_w)
	AM_RANGE(0x7800, 0x7800) AM_MIRROR(0x07ff) AM_WRITE(bank_select_w)
	AM_RANGE(0x8000, 0xffff) AM_ROMBANK("rombank")
ADDRESS_MAP_END


//-------------------------------------------------
// device_add_mconfig - add device configuration
//-------------------------------------------------

MACHINE_CONFIG_MEMBER( williams_cvsd_sound_device::device_add_mconfig )
	MCFG_CPU_ADD("cpu", M6809E, CVSD_MASTER_CLOCK)
	MCFG_CPU_PROGRAM_MAP(williams_cvsd_map)

	MCFG_DEVICE_ADD("pia", PIA6821, 0)
	MCFG_PIA_WRITEPA_HANDLER(DEVWRITE8("dac", dac_byte_interface, write))
	MCFG_PIA_WRITEPB_HANDLER(WRITE8(williams_cvsd_sound_device, talkback_w))
	MCFG_PIA_IRQA_HANDLER(INPUTLINE("cpu", M6809_FIRQ_LINE))
	MCFG_PIA_IRQB_HANDLER(INPUTLINE("cpu", INPUT_LINE_NMI))

	MCFG_YM2151_ADD("ym2151", CVSD_FM_CLOCK)
	MCFG_YM2151_IRQ_HANDLER(DEVWRITELINE("pia", pia6821_device, ca1_w)) MCFG_DEVCB_INVERT // IRQ is not true state
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, DEVICE_SELF_OWNER, 0.10)

	MCFG_SOUND_ADD("dac", MC1408, 0) MCFG_SOUND_ROUTE(ALL_OUTPUTS, DEVICE_SELF_OWNER, 0.25)
	MCFG_DEVICE_ADD("vref", VOLTAGE_REGULATOR, 0) MCFG_VOLTAGE_REGULATOR_OUTPUT(5.0)
	MCFG_SOUND_ROUTE_EX(0, "dac", 1.0, DAC_VREF_POS_INPUT) MCFG_SOUND_ROUTE_EX(0, "dac", -1.0, DAC_VREF_NEG_INPUT)

	MCFG_SOUND_ADD("cvsd", HC55516, 0)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, DEVICE_SELF_OWNER, 0.60)
MACHINE_CONFIG_END


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void williams_cvsd_sound_device::device_start()
{
	// configure master CPU banks
	uint8_t *rom = memregion("cpu")->base();
	for (int bank = 0; bank < 16; bank++)
	{
		//
		//  D0/D1 -> selects: 0=U4 1=U19 2=U20 3=n/c
		//  D2 -> A15
		//  D3 -> A16
		//
		offs_t offset = 0x8000 * ((bank >> 2) & 3) + 0x20000 * (bank & 3);
		membank("rombank")->configure_entry(bank, &rom[0x10000 + offset]);
	}
	membank("rombank")->set_entry(0);

	// reset the IRQ state
	m_pia->ca1_w(1);

	// register for save states
	save_item(NAME(m_talkback));
}


//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void williams_cvsd_sound_device::device_reset()
{
	// reset interrupt states
	m_cpu->set_input_line(M6809_FIRQ_LINE, CLEAR_LINE);
	m_cpu->set_input_line(M6809_IRQ_LINE, CLEAR_LINE);
	m_cpu->set_input_line(INPUT_LINE_NMI, CLEAR_LINE);
}


//-------------------------------------------------
//  device_timer - timer callbacks
//-------------------------------------------------

void williams_cvsd_sound_device::device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr)
{
	// process incoming data write
	m_pia->portb_w(param & 0xff);
	m_pia->cb1_w((param >> 8) & 1);
	m_pia->cb2_w((param >> 9) & 1);
}



//**************************************************************************
//  NARC SOUND BOARD
//**************************************************************************

//-------------------------------------------------
//  williams_narc_sound_device - constructor
//-------------------------------------------------

williams_narc_sound_device::williams_narc_sound_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, WILLIAMS_NARC_SOUND, tag, owner, clock),
		device_mixer_interface(mconfig, *this),
		m_cpu0(*this, "cpu0"),
		m_cpu1(*this, "cpu1"),
		m_hc55516(*this, "cvsd"),
		m_latch(0),
		m_latch2(0),
		m_talkback(0),
		m_audio_sync(0),
		m_sound_int_state(0)
{
}


//-------------------------------------------------
//  read - return the talkback register with the
//  SYNC bits in bits 8 and 9
//-------------------------------------------------

READ16_MEMBER(williams_narc_sound_device::read)
{
	return m_talkback | (m_audio_sync << 8);
}


//-------------------------------------------------
//  write - handle an external write to the input
//  latch
//-------------------------------------------------

WRITE16_MEMBER(williams_narc_sound_device::write)
{
	synchronize(TID_MASTER_COMMAND, data);
}


//-------------------------------------------------
//  reset_write - write to the reset line
//-------------------------------------------------

WRITE_LINE_MEMBER(williams_narc_sound_device::reset_write)
{
	// going high halts the CPU
	if (state)
	{
		master_bank_select_w(m_cpu0->space(), 0, 0);
		slave_bank_select_w(m_cpu1->space(), 0, 0);
		device_reset();
		m_cpu0->set_input_line(INPUT_LINE_RESET, ASSERT_LINE);
		m_cpu1->set_input_line(INPUT_LINE_RESET, ASSERT_LINE);
	}

	// going low resets and reactivates the CPU
	else
	{
		m_cpu0->set_input_line(INPUT_LINE_RESET, CLEAR_LINE);
		m_cpu1->set_input_line(INPUT_LINE_RESET, CLEAR_LINE);
	}
}


//-------------------------------------------------
//  master_bank_select_w - select the bank for the
//  master CPU
//-------------------------------------------------

WRITE8_MEMBER(williams_narc_sound_device::master_bank_select_w)
{
	membank("masterbank")->set_entry(data & 0x0f);
}


//-------------------------------------------------
//  slave_bank_select_w - select the bank for the
//  slave CPU
//-------------------------------------------------

WRITE8_MEMBER(williams_narc_sound_device::slave_bank_select_w)
{
	membank("slavebank")->set_entry(data & 0x0f);
}


//-------------------------------------------------
//  command_r - read command written by external
//  agent
//-------------------------------------------------

READ8_MEMBER(williams_narc_sound_device::command_r)
{
	m_cpu0->set_input_line(M6809_IRQ_LINE, CLEAR_LINE);
	m_sound_int_state = 0;
	return m_latch;
}


//-------------------------------------------------
//  command2_w - write command from master CPU to
//  slave CPU
//-------------------------------------------------

WRITE8_MEMBER(williams_narc_sound_device::command2_w)
{
	synchronize(TID_SLAVE_COMMAND, data);
}


//-------------------------------------------------
//  command2_r - read command written by master
//  CPU
//-------------------------------------------------

READ8_MEMBER(williams_narc_sound_device::command2_r)
{
	m_cpu1->set_input_line(M6809_FIRQ_LINE, CLEAR_LINE);
	return m_latch2;
}


//-------------------------------------------------
//  master_talkback_w - handle writes to the
//  talkback latch from the master CPU
//-------------------------------------------------

WRITE8_MEMBER(williams_narc_sound_device::master_talkback_w)
{
	m_talkback = data;
	logerror("Master Talkback = %02X\n", data);
}


//-------------------------------------------------
//  master_sync_w - handle writes to the master
//  SYNC register
//-------------------------------------------------

WRITE8_MEMBER(williams_narc_sound_device::master_sync_w)
{
	timer_set(attotime::from_double(TIME_OF_74LS123(180000, 0.000001)), TID_SYNC_CLEAR, 0x01);
	m_audio_sync |= 0x01;
	logerror("Master sync = %02X\n", data);
}


//-------------------------------------------------
//  slave_talkback_w - handle writes to the
//  talkback latch from the slave CPU
//-------------------------------------------------

WRITE8_MEMBER(williams_narc_sound_device::slave_talkback_w)
{
	logerror("Slave Talkback = %02X\n", data);
}


//-------------------------------------------------
//  slave_sync_w - handle writes to the slave
//  SYNC register
//-------------------------------------------------

WRITE8_MEMBER(williams_narc_sound_device::slave_sync_w)
{
	timer_set(attotime::from_double(TIME_OF_74LS123(180000, 0.000001)), TID_SYNC_CLEAR, 0x02);
	m_audio_sync |= 0x02;
	logerror("Slave sync = %02X\n", data);
}


//-------------------------------------------------
//  cvsd_digit_clock_clear_w - clear the clock on
//  the HC55516 and clock the data
//-------------------------------------------------

WRITE8_MEMBER(williams_narc_sound_device::cvsd_digit_clock_clear_w)
{
	m_hc55516->digit_w(data);
	m_hc55516->clock_w(0);
}


//-------------------------------------------------
//  cvsd_clock_set_w - set the clock on the HC55516
//-------------------------------------------------

WRITE8_MEMBER(williams_narc_sound_device::cvsd_clock_set_w)
{
	m_hc55516->clock_w(1);
}


//-------------------------------------------------
//  master CPU map
//-------------------------------------------------

static ADDRESS_MAP_START( williams_narc_master_map, AS_PROGRAM, 8, williams_narc_sound_device )
	AM_RANGE(0x0000, 0x1fff) AM_RAM
	AM_RANGE(0x2000, 0x2001) AM_MIRROR(0x03fe) AM_DEVREADWRITE("ym2151", ym2151_device, read, write)
	AM_RANGE(0x2800, 0x2800) AM_MIRROR(0x03ff) AM_WRITE(master_talkback_w)
	AM_RANGE(0x2c00, 0x2c00) AM_MIRROR(0x03ff) AM_WRITE(command2_w)
	AM_RANGE(0x3000, 0x3000) AM_MIRROR(0x03ff) AM_DEVWRITE("dac1", dac_byte_interface, write)
	AM_RANGE(0x3400, 0x3400) AM_MIRROR(0x03ff) AM_READ(command_r)
	AM_RANGE(0x3800, 0x3800) AM_MIRROR(0x03ff) AM_WRITE(master_bank_select_w)
	AM_RANGE(0x3c00, 0x3c00) AM_MIRROR(0x03ff) AM_WRITE(master_sync_w)
	AM_RANGE(0x4000, 0xbfff) AM_ROMBANK("masterbank")
	AM_RANGE(0xc000, 0xffff) AM_ROMBANK("masterupper")
ADDRESS_MAP_END


//-------------------------------------------------
//  slave CPU map
//-------------------------------------------------

static ADDRESS_MAP_START( williams_narc_slave_map, AS_PROGRAM, 8, williams_narc_sound_device )
	AM_RANGE(0x0000, 0x1fff) AM_RAM
	AM_RANGE(0x2000, 0x2000) AM_MIRROR(0x03ff) AM_WRITE(cvsd_clock_set_w)
	AM_RANGE(0x2400, 0x2400) AM_MIRROR(0x03ff) AM_WRITE(cvsd_digit_clock_clear_w)
	AM_RANGE(0x2800, 0x2800) AM_MIRROR(0x03ff) AM_WRITE(slave_talkback_w)
	AM_RANGE(0x3000, 0x3000) AM_MIRROR(0x03ff) AM_DEVWRITE("dac2", dac_byte_interface, write)
	AM_RANGE(0x3400, 0x3400) AM_MIRROR(0x03ff) AM_READ(command2_r)
	AM_RANGE(0x3800, 0x3800) AM_MIRROR(0x03ff) AM_WRITE(slave_bank_select_w)
	AM_RANGE(0x3c00, 0x3c00) AM_MIRROR(0x03ff) AM_WRITE(slave_sync_w)
	AM_RANGE(0x4000, 0xbfff) AM_ROMBANK("slavebank")
	AM_RANGE(0xc000, 0xffff) AM_ROMBANK("slaveupper")
ADDRESS_MAP_END


//-------------------------------------------------
// device_add_mconfig - add device configuration
//-------------------------------------------------


MACHINE_CONFIG_MEMBER( williams_narc_sound_device::device_add_mconfig )
	MCFG_CPU_ADD("cpu0", M6809E, NARC_MASTER_CLOCK)
	MCFG_CPU_PROGRAM_MAP(williams_narc_master_map)

	MCFG_CPU_ADD("cpu1", M6809E, NARC_MASTER_CLOCK)
	MCFG_CPU_PROGRAM_MAP(williams_narc_slave_map)

	MCFG_YM2151_ADD("ym2151", NARC_FM_CLOCK)
	MCFG_YM2151_IRQ_HANDLER(INPUTLINE("cpu0", M6809_FIRQ_LINE))
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, DEVICE_SELF_OWNER, 0.10)

	MCFG_SOUND_ADD("dac1", AD7224, 0) MCFG_SOUND_ROUTE(ALL_OUTPUTS, DEVICE_SELF_OWNER, 0.25)
	MCFG_SOUND_ADD("dac2", AD7224, 0) MCFG_SOUND_ROUTE(ALL_OUTPUTS, DEVICE_SELF_OWNER, 0.25)
	MCFG_DEVICE_ADD("vref", VOLTAGE_REGULATOR, 0) MCFG_VOLTAGE_REGULATOR_OUTPUT(5.0)
	MCFG_SOUND_ROUTE_EX(0, "dac1", 1.0, DAC_VREF_POS_INPUT) MCFG_SOUND_ROUTE_EX(0, "dac1", -1.0, DAC_VREF_NEG_INPUT)
	MCFG_SOUND_ROUTE_EX(0, "dac2", 1.0, DAC_VREF_POS_INPUT) MCFG_SOUND_ROUTE_EX(0, "dac2", -1.0, DAC_VREF_NEG_INPUT)

	MCFG_SOUND_ADD("cvsd", HC55516, 0)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, DEVICE_SELF_OWNER, 0.60)
MACHINE_CONFIG_END


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void williams_narc_sound_device::device_start()
{
	// configure master CPU banks
	uint8_t *rom = memregion("cpu0")->base();
	for (int bank = 0; bank < 16; bank++)
	{
		//
		//  D0 -> A15
		//  D1/D2 -> selects: 0=n/c 1=U3 2=U4 3=U5
		//  D3 -> A16
		//
		offs_t offset = 0x8000 * (bank & 1) + 0x10000 * ((bank >> 3) & 1) + 0x20000 * ((bank >> 1) & 3);
		membank("masterbank")->configure_entry(bank, &rom[0x10000 + offset]);
	}
	membank("masterupper")->set_base(&rom[0x10000 + 0x4000 + 0x8000 + 0x10000 + 0x20000 * 3]);

	// configure slave CPU banks
	rom = memregion("cpu1")->base();
	for (int bank = 0; bank < 16; bank++)
	{
		//
		//  D0 -> A15
		//  D1/D2 -> selects: 0=U35 1=U36 2=U37 3=U38
		//  D3 -> A16
		//
		offs_t offset = 0x8000 * (bank & 1) + 0x10000 * ((bank >> 3) & 1) + 0x20000 * ((bank >> 1) & 3);
		membank("slavebank")->configure_entry(bank, &rom[0x10000 + offset]);
	}
	membank("slaveupper")->set_base(&rom[0x10000 + 0x4000 + 0x8000 + 0x10000 + 0x20000 * 3]);

	// register for save states
	save_item(NAME(m_latch));
	save_item(NAME(m_latch2));
	save_item(NAME(m_talkback));
	save_item(NAME(m_audio_sync));
	save_item(NAME(m_sound_int_state));
}


//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void williams_narc_sound_device::device_reset()
{
	// reset interrupt states
	m_sound_int_state = 0;
	m_cpu0->set_input_line(M6809_FIRQ_LINE, CLEAR_LINE);
	m_cpu0->set_input_line(M6809_IRQ_LINE, CLEAR_LINE);
	m_cpu0->set_input_line(INPUT_LINE_NMI, CLEAR_LINE);
	m_cpu1->set_input_line(M6809_FIRQ_LINE, CLEAR_LINE);
	m_cpu1->set_input_line(M6809_IRQ_LINE, CLEAR_LINE);
	m_cpu1->set_input_line(INPUT_LINE_NMI, CLEAR_LINE);
}


//-------------------------------------------------
//  device_timer - timer callbacks
//-------------------------------------------------

void williams_narc_sound_device::device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr)
{
	switch (id)
	{
		case TID_MASTER_COMMAND:
			m_latch = param & 0xff;
			m_cpu0->set_input_line(INPUT_LINE_NMI, (param & 0x100) ? CLEAR_LINE : ASSERT_LINE);
			if ((param & 0x200) == 0)
			{
				m_cpu0->set_input_line(M6809_IRQ_LINE, ASSERT_LINE);
				m_sound_int_state = 1;
			}
			break;

		case TID_SLAVE_COMMAND:
			m_latch2 = param & 0xff;
			m_cpu1->set_input_line(M6809_FIRQ_LINE, ASSERT_LINE);
			break;

		case TID_SYNC_CLEAR:
			m_audio_sync &= ~param;
			break;
	}
}


//**************************************************************************
//  ADPCM SOUND BOARD
//**************************************************************************

//-------------------------------------------------
//  williams_adpcm_sound_device - constructor
//-------------------------------------------------

williams_adpcm_sound_device::williams_adpcm_sound_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, WILLIAMS_ADPCM_SOUND, tag, owner, clock),
		device_mixer_interface(mconfig, *this),
		m_cpu(*this, "cpu"),
		m_latch(0),
		m_talkback(0),
		m_sound_int_state(0)
{
}


//-------------------------------------------------
//  write - handle an external write to the input
//  latch
//-------------------------------------------------

WRITE16_MEMBER(williams_adpcm_sound_device::write)
{
	synchronize(TID_COMMAND, data);
}


//-------------------------------------------------
//  reset_write - write to the reset line
//-------------------------------------------------

WRITE_LINE_MEMBER(williams_adpcm_sound_device::reset_write)
{
	// going high halts the CPU
	if (state)
	{
		bank_select_w(m_cpu->space(), 0, 0);
		device_reset();
		m_cpu->set_input_line(INPUT_LINE_RESET, ASSERT_LINE);
	}

	// going low resets and reactivates the CPU
	else
		m_cpu->set_input_line(INPUT_LINE_RESET, CLEAR_LINE);
}


//-------------------------------------------------
//  irq_read - read the sound IRQ state
//-------------------------------------------------

READ_LINE_MEMBER(williams_adpcm_sound_device::irq_read)
{
	return m_sound_int_state;
}


//-------------------------------------------------
//  bank_select_w - select the sound CPU memory
//  bank
//-------------------------------------------------

WRITE8_MEMBER(williams_adpcm_sound_device::bank_select_w)
{
	membank("rombank")->set_entry(data & 0x07);
}


//-------------------------------------------------
//  bank_select_w - select the OKI6295 memory
//  bank
//-------------------------------------------------

WRITE8_MEMBER(williams_adpcm_sound_device::oki6295_bank_select_w)
{
	membank("okibank")->set_entry(data & 7);
}


//-------------------------------------------------
//  command_r - read the command from the external
//  latch
//-------------------------------------------------

READ8_MEMBER(williams_adpcm_sound_device::command_r)
{
	m_cpu->set_input_line(M6809_IRQ_LINE, CLEAR_LINE);

	// don't clear the external IRQ state for a short while; this allows the
	// self-tests to pass
	timer_set(attotime::from_usec(10), TID_IRQ_CLEAR);
	return m_latch;
}


//-------------------------------------------------
//  talkback_w - write to the talkback latch
//-------------------------------------------------

WRITE8_MEMBER(williams_adpcm_sound_device::talkback_w)
{
	m_talkback = data;
	logerror("ADPCM Talkback = %02X\n", data);
}


//-------------------------------------------------
//  audio CPU map
//-------------------------------------------------

static ADDRESS_MAP_START( williams_adpcm_map, AS_PROGRAM, 8, williams_adpcm_sound_device )
	AM_RANGE(0x0000, 0x1fff) AM_RAM
	AM_RANGE(0x2000, 0x2000) AM_MIRROR(0x03ff) AM_WRITE(bank_select_w)
	AM_RANGE(0x2400, 0x2401) AM_MIRROR(0x03fe) AM_DEVREADWRITE("ym2151", ym2151_device, read, write)
	AM_RANGE(0x2800, 0x2800) AM_MIRROR(0x03ff) AM_DEVWRITE("dac", dac_byte_interface, write)
	AM_RANGE(0x2c00, 0x2c00) AM_MIRROR(0x03ff) AM_DEVREADWRITE("oki", okim6295_device, read, write)
	AM_RANGE(0x3000, 0x3000) AM_MIRROR(0x03ff) AM_READ(command_r)
	AM_RANGE(0x3400, 0x3400) AM_MIRROR(0x03ff) AM_WRITE(oki6295_bank_select_w)
	AM_RANGE(0x3c00, 0x3c00) AM_MIRROR(0x03ff) AM_WRITE(talkback_w)
	AM_RANGE(0x4000, 0xbfff) AM_ROMBANK("rombank")
	AM_RANGE(0xc000, 0xffff) AM_ROMBANK("romupper")
ADDRESS_MAP_END


//-------------------------------------------------
//  OKI6295 map
//-------------------------------------------------

static ADDRESS_MAP_START( williams_adpcm_oki_map, 0, 8, williams_adpcm_sound_device )
	AM_RANGE(0x00000, 0x1ffff) AM_ROMBANK("okibank")
	AM_RANGE(0x20000, 0x3ffff) AM_ROM AM_REGION("oki", 0x60000)
ADDRESS_MAP_END


//-------------------------------------------------
// device_add_mconfig - add device configuration
//-------------------------------------------------

MACHINE_CONFIG_MEMBER( williams_adpcm_sound_device::device_add_mconfig )
	MCFG_CPU_ADD("cpu", M6809E, ADPCM_MASTER_CLOCK)
	MCFG_CPU_PROGRAM_MAP(williams_adpcm_map)

	MCFG_YM2151_ADD("ym2151", ADPCM_FM_CLOCK)
	MCFG_YM2151_IRQ_HANDLER(INPUTLINE("cpu", M6809_FIRQ_LINE))
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, DEVICE_SELF_OWNER, 0.10)

	MCFG_SOUND_ADD("dac", AD7524, 0) MCFG_SOUND_ROUTE(ALL_OUTPUTS, DEVICE_SELF_OWNER, 0.5)
	MCFG_DEVICE_ADD("vref", VOLTAGE_REGULATOR, 0) MCFG_VOLTAGE_REGULATOR_OUTPUT(5.0)
	MCFG_SOUND_ROUTE_EX(0, "dac", 1.0, DAC_VREF_POS_INPUT) MCFG_SOUND_ROUTE_EX(0, "dac", -1.0, DAC_VREF_NEG_INPUT)

	MCFG_OKIM6295_ADD("oki", ADPCM_MASTER_CLOCK/8, PIN7_HIGH) // clock frequency & pin 7 not verified
	MCFG_DEVICE_ADDRESS_MAP(0, williams_adpcm_oki_map)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, DEVICE_SELF_OWNER, 0.5)
MACHINE_CONFIG_END


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void williams_adpcm_sound_device::device_start()
{
	// configure banks
	uint8_t *rom = memregion("cpu")->base();
	membank("rombank")->configure_entries(0, 8, &rom[0x10000], 0x8000);
	membank("romupper")->set_base(&rom[0x10000 + 0x4000 + 7 * 0x8000]);

	// expand ADPCM data
	rom = memregion("oki")->base();
	// it is assumed that U12 is loaded @ 0x00000 and U13 is loaded @ 0x40000
	membank("okibank")->configure_entry(0, &rom[0x40000]);
	membank("okibank")->configure_entry(1, &rom[0x40000]);
	membank("okibank")->configure_entry(2, &rom[0x20000]);
	membank("okibank")->configure_entry(3, &rom[0x00000]);
	membank("okibank")->configure_entry(4, &rom[0xe0000]);
	membank("okibank")->configure_entry(5, &rom[0xc0000]);
	membank("okibank")->configure_entry(6, &rom[0xa0000]);
	membank("okibank")->configure_entry(7, &rom[0x80000]);

	// register for save states
	save_item(NAME(m_latch));
	save_item(NAME(m_talkback));
	save_item(NAME(m_sound_int_state));
}


//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void williams_adpcm_sound_device::device_reset()
{
	// reset interrupt states
	m_sound_int_state = 0;
	m_cpu->set_input_line(M6809_FIRQ_LINE, CLEAR_LINE);
	m_cpu->set_input_line(M6809_IRQ_LINE, CLEAR_LINE);
	m_cpu->set_input_line(INPUT_LINE_NMI, CLEAR_LINE);
}


//-------------------------------------------------
//  device_timer - timer callbacks
//-------------------------------------------------

void williams_adpcm_sound_device::device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr)
{
	switch (id)
	{
		case TID_COMMAND:
			m_latch = param & 0xff;
			if (!(param & 0x200))
			{
				m_cpu->set_input_line(M6809_IRQ_LINE, ASSERT_LINE);
				m_sound_int_state = 1;
				machine().scheduler().boost_interleave(attotime::zero, attotime::from_usec(100));
			}
			break;

		case TID_IRQ_CLEAR:
			m_sound_int_state = 0;
			break;
	}
}
