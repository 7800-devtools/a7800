// license:BSD-3-Clause
// copyright-holders:Robbbert,AJR
/*****************************************************************************************

  8088-based pinball games by Unidesa/Stargame:
  - Mephisto
  - Cirsa Sport 2000

  Serial communication with the sound board is handled by a 8256 MUART (not emulated yet).

******************************************************************************************/

#include "emu.h"
#include "cpu/i86/i86.h"
#include "cpu/mcs51/mcs51.h"
#include "machine/i8155.h"
//#include "machine/i8256.h"
#include "machine/nvram.h"
#include "sound/ay8910.h"
#include "sound/dac.h"
#include "sound/volt_reg.h"
#include "sound/3812intf.h"
#include "speaker.h"

// mephisto_state was also defined in mess/drivers/mephisto.c
class mephisto_pinball_state : public driver_device
{
public:
	mephisto_pinball_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
			m_maincpu(*this, "maincpu"),
			m_aysnd(*this, "aysnd"),
			m_soundbank(*this, "soundbank")
	{ }

	DECLARE_WRITE8_MEMBER(shift_load_w);
	DECLARE_READ8_MEMBER(ay8910_read);
	DECLARE_WRITE8_MEMBER(ay8910_write);
	DECLARE_WRITE8_MEMBER(t0_t1_w);
	DECLARE_WRITE8_MEMBER(ay8910_columns_w);
	DECLARE_READ8_MEMBER(ay8910_inputs_r);
	DECLARE_WRITE8_MEMBER(sound_rombank_w);

protected:

	// devices
	required_device<cpu_device> m_maincpu;
	required_device<ay8910_device> m_aysnd;
	required_memory_bank m_soundbank;

	// driver_device overrides
	virtual void machine_start() override;
	virtual void machine_reset() override;

	// internal data
	u8 m_ay8910_data;
	bool m_ay8910_bdir;
	bool m_ay8910_bc1;

private:
	void ay8910_update();
};


WRITE8_MEMBER(mephisto_pinball_state::shift_load_w)
{
}

READ8_MEMBER(mephisto_pinball_state::ay8910_read)
{
	return m_ay8910_data;
}

WRITE8_MEMBER(mephisto_pinball_state::ay8910_write)
{
	m_ay8910_data = data;
	ay8910_update();
}

WRITE8_MEMBER(mephisto_pinball_state::t0_t1_w)
{
	m_ay8910_bdir = BIT(data, 4); // T0
	m_ay8910_bc1 = BIT(data, 5); // T1
	ay8910_update();
}

void mephisto_pinball_state::ay8910_update()
{
	if (m_ay8910_bdir)
		m_aysnd->data_address_w(machine().dummy_space(), m_ay8910_bc1, m_ay8910_data);
	else if (m_ay8910_bc1)
		m_ay8910_data = m_aysnd->data_r(machine().dummy_space(), 0);
}

WRITE8_MEMBER(mephisto_pinball_state::ay8910_columns_w)
{
}

READ8_MEMBER(mephisto_pinball_state::ay8910_inputs_r)
{
	return 0xff;
}

WRITE8_MEMBER(mephisto_pinball_state::sound_rombank_w)
{
	m_soundbank->set_entry(data & 0xf);
}

static ADDRESS_MAP_START( mephisto_map, AS_PROGRAM, 8, mephisto_pinball_state )
	AM_RANGE(0x00000, 0x07fff) AM_ROM AM_MIRROR(0x08000) AM_REGION("maincpu", 0)
	AM_RANGE(0x10000, 0x107ff) AM_RAM AM_SHARE("nvram")
	AM_RANGE(0x12000, 0x1201f) AM_NOP //AM_DEVREADWRITE("muart", i8256_device, read, write)
	AM_RANGE(0x13000, 0x130ff) AM_DEVREADWRITE("ic20", i8155_device, memory_r, memory_w)
	AM_RANGE(0x13800, 0x13807) AM_DEVREADWRITE("ic20", i8155_device, io_r, io_w)
	AM_RANGE(0x14000, 0x140ff) AM_DEVREADWRITE("ic9", i8155_device, memory_r, memory_w)
	AM_RANGE(0x14800, 0x14807) AM_DEVREADWRITE("ic9", i8155_device, io_r, io_w)
	AM_RANGE(0x16000, 0x16000) AM_WRITE(shift_load_w)
	AM_RANGE(0x17000, 0x17001) AM_WRITENOP //???
	AM_RANGE(0xf8000, 0xfffff) AM_ROM AM_REGION("maincpu", 0)
ADDRESS_MAP_END

static ADDRESS_MAP_START( mephisto_8051_map, AS_PROGRAM, 8, mephisto_pinball_state )
	AM_RANGE(0x0000, 0x7fff) AM_ROM
	AM_RANGE(0x8000, 0xffff) AM_ROMBANK("soundbank")
ADDRESS_MAP_END

static ADDRESS_MAP_START( mephisto_8051_io, AS_IO, 8, mephisto_pinball_state )
	AM_RANGE(0x0000, 0x07ff) AM_RAM
	AM_RANGE(0x0800, 0x0800) AM_WRITE(sound_rombank_w)
	AM_RANGE(0x1000, 0x1000) AM_DEVWRITE("dac", dac08_device, write)
	AM_RANGE(MCS51_PORT_P0, MCS51_PORT_P0) AM_NOP // AD0-AD7
	AM_RANGE(MCS51_PORT_P1, MCS51_PORT_P1) AM_READWRITE(ay8910_read, ay8910_write)
	AM_RANGE(MCS51_PORT_P2, MCS51_PORT_P2) AM_WRITENOP // A8-A15
	AM_RANGE(MCS51_PORT_P3, MCS51_PORT_P3) AM_WRITE(t0_t1_w)
	AM_RANGE(MCS51_PORT_TX, MCS51_PORT_TX) AM_READNOP // from MUART
ADDRESS_MAP_END

#ifdef UNUSED_DEFINITION
static ADDRESS_MAP_START( sport2k_8051_io, AS_IO, 8, mephisto_pinball_state )
	AM_IMPORT_FROM(mephisto_8051_data)
	AM_RANGE(0x1800, 0x1801) AM_DEVREADWRITE("ymsnd", ym3812_device, read, write)
ADDRESS_MAP_END
#endif

static INPUT_PORTS_START( mephisto )
INPUT_PORTS_END

void mephisto_pinball_state::machine_start()
{
	m_soundbank->configure_entries(0, 16, memregion("sound1")->base(), 0x8000);
	m_soundbank->set_entry(0);

	m_ay8910_data = 0;
	m_ay8910_bdir = 1;
	m_ay8910_bc1 = 1;
	save_item(NAME(m_ay8910_data));
	save_item(NAME(m_ay8910_bdir));
	save_item(NAME(m_ay8910_bc1));
}

void mephisto_pinball_state::machine_reset()
{
}

static MACHINE_CONFIG_START( mephisto )
	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", I8088, XTAL_18MHz/3)
	MCFG_CPU_PROGRAM_MAP(mephisto_map)
	//MCFG_CPU_IRQ_ACKNOWLEDGE_DEVICE("muart", i8256_device, inta_cb)

	MCFG_NVRAM_ADD_0FILL("nvram")

	//MCFG_DEVICE_ADD("muart", I8256, XTAL_18MHz/3)
	//MCFG_I8256_IRQ_HANDLER(INPUTLINE("maincpu", INPUT_LINE_IRQ0))
	//MCFG_I8256_TXD_HANDLER(INPUTLINE("audiocpu", MCS51_RX_LINE))

	MCFG_DEVICE_ADD("ic20", I8155, XTAL_18MHz/6)
	//MCFG_I8155_OUT_TIMEROUT_CB(DEVWRITELINE("muart", i8256_device, write_txc))

	MCFG_DEVICE_ADD("ic9", I8155, XTAL_18MHz/6)
	//MCFG_I8155_OUT_TIMEROUT_CB(WRITELINE(mephisto_pinball_state, clk_shift_w))

	MCFG_CPU_ADD("soundcpu", I8051, XTAL_12MHz)
	MCFG_CPU_PROGRAM_MAP(mephisto_8051_map)
	MCFG_CPU_IO_MAP(mephisto_8051_io)

	MCFG_SPEAKER_STANDARD_MONO("mono")

	MCFG_SOUND_ADD("aysnd", AY8910, XTAL_12MHz/8)
	MCFG_AY8910_PORT_A_WRITE_CB(WRITE8(mephisto_pinball_state, ay8910_columns_w))
	MCFG_AY8910_PORT_B_READ_CB(READ8(mephisto_pinball_state, ay8910_inputs_r))
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.5)

	MCFG_SOUND_ADD("dac", DAC08, 0)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.5)
	MCFG_DEVICE_ADD("vref", VOLTAGE_REGULATOR, 0) MCFG_VOLTAGE_REGULATOR_OUTPUT(5.0)
	MCFG_SOUND_ROUTE_EX(0, "dac", 1.0, DAC_VREF_POS_INPUT) MCFG_SOUND_ROUTE_EX(0, "dac", -1.0, DAC_VREF_NEG_INPUT)
MACHINE_CONFIG_END

#ifdef UNUSED_DEFINITION
static MACHINE_CONFIG_DERIVED(sport2k, mephisto)
	MCFG_CPU_MODIFY("soundcpu")
	MCFG_CPU_IO_MAP(sport2k_8051_io)

	MCFG_SOUND_ADD("ymsnd", YM3812, XTAL_14_31818MHz/4)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.5)
MACHINE_CONFIG_END
#endif

/*-------------------------------------------------------------------
/ Mephisto
/-------------------------------------------------------------------*/
ROM_START(mephistp)
	ROM_REGION(0x08000, "maincpu", 0)
	ROM_LOAD("cpu_ver1.2", 0x00000, 0x8000, CRC(845c8eb4) SHA1(2a705629990950d4e2d3a66a95e9516cf112cc88))

	ROM_REGION(0x08000, "soundcpu", 0)
	ROM_LOAD("ic15_02", 0x00000, 0x8000, CRC(2accd446) SHA1(7297e4825c33e7cf23f86fe39a0242e74874b1e2))

	ROM_REGION(0x80000, "sound1", 0)
	ROM_LOAD("ic14_s0", 0x40000, 0x8000, CRC(7cea3018) SHA1(724fe7a4456cbf2ac01466d946668ee86f4410ae))
	ROM_LOAD("ic13_s1", 0x48000, 0x8000, CRC(5a9e0f1d) SHA1(dbfd307706c51f8809f4867a199b4b62beb64379))
	ROM_LOAD("ic12_s2", 0x50000, 0x8000, CRC(b3cc962a) SHA1(521376cab7e917a5d5f5f183bccb21bd13327c48))
	ROM_LOAD("ic11_s3", 0x58000, 0x8000, CRC(8aaa21ec) SHA1(29f17249cac62128fd8b0eee415ce399ee2ec672))
	ROM_LOAD("ic16_c", 0x60000, 0x8000, CRC(5f12b4f4) SHA1(73fbdb57fca0dbc918e6665a6cb949e741f2720a))
	ROM_LOAD("ic17_d", 0x68000, 0x8000, CRC(d17e18a8) SHA1(372eaf209ea5d26f3c096aadd7d028ef68bfb68e))
	ROM_LOAD("ic18_e", 0x70000, 0x8000, CRC(eac6dbba) SHA1(f4971c8b0aa3a72c396b943a0ee3094afb902ec1))
	ROM_LOAD("ic19_f", 0x78000, 0x8000, CRC(cc4bb629) SHA1(db46be2a8034bbd106b7dd80f50988c339684b5e))
ROM_END

ROM_START(mephistp1)
	ROM_REGION(0x08000, "maincpu", 0)
	ROM_LOAD("cpu_ver1.1", 0x00000, 0x8000, CRC(ce584902) SHA1(dd05d008bbd9b6588cb204e8d901537ffe7ddd43))

	ROM_REGION(0x08000, "soundcpu", 0)
	ROM_LOAD("ic15_02", 0x00000, 0x8000, CRC(2accd446) SHA1(7297e4825c33e7cf23f86fe39a0242e74874b1e2))

	ROM_REGION(0x80000, "sound1", 0)
	ROM_LOAD("ic14_s0", 0x40000, 0x8000, CRC(7cea3018) SHA1(724fe7a4456cbf2ac01466d946668ee86f4410ae))
	ROM_LOAD("ic13_s1", 0x48000, 0x8000, CRC(5a9e0f1d) SHA1(dbfd307706c51f8809f4867a199b4b62beb64379))
	ROM_LOAD("ic12_s2", 0x50000, 0x8000, CRC(b3cc962a) SHA1(521376cab7e917a5d5f5f183bccb21bd13327c48))
	ROM_LOAD("ic11_s3", 0x58000, 0x8000, CRC(8aaa21ec) SHA1(29f17249cac62128fd8b0eee415ce399ee2ec672))
	ROM_LOAD("ic16_c", 0x60000, 0x8000, CRC(5f12b4f4) SHA1(73fbdb57fca0dbc918e6665a6cb949e741f2720a))
	ROM_LOAD("ic17_d", 0x68000, 0x8000, CRC(d17e18a8) SHA1(372eaf209ea5d26f3c096aadd7d028ef68bfb68e))
	ROM_LOAD("ic18_e", 0x70000, 0x8000, CRC(eac6dbba) SHA1(f4971c8b0aa3a72c396b943a0ee3094afb902ec1))
	ROM_LOAD("ic19_f", 0x78000, 0x8000, CRC(cc4bb629) SHA1(db46be2a8034bbd106b7dd80f50988c339684b5e))
ROM_END


GAME(1987,  mephistp,   0,         mephisto,  mephisto, mephisto_pinball_state,  0,  ROT0,  "Stargame",    "Mephisto (rev. 1.2)",     MACHINE_IS_SKELETON_MECHANICAL)
GAME(1987,  mephistp1,  mephistp,  mephisto,  mephisto, mephisto_pinball_state,  0,  ROT0,  "Stargame",    "Mephisto (rev. 1.1)",     MACHINE_IS_SKELETON_MECHANICAL)
//GAME(1988,  sport2k,    0,         sport2k,   sport2k, mephisto_pinball_state,   0,  ROT0,  "Unidesa",     "Cirsa Sport 2000",        MACHINE_IS_SKELETON_MECHANICAL)
