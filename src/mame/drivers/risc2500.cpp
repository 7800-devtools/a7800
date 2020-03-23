// license:BSD-3-Clause
// copyright-holders:Sandro Ronco
/******************************************************************************

    Saitek RISC 2500

    TODO:
     - Sound is too short and high pitch, better when you underclock the cpu.
       Is cpu cycle timing wrong? or waitstate on p1000_w?

******************************************************************************/


#include "emu.h"
#include "cpu/arm/arm.h"
#include "machine/ram.h"
#include "machine/nvram.h"
#include "sound/dac.h"
#include "sound/volt_reg.h"
#include "screen.h"
#include "speaker.h"

#include "risc2500.lh"


class risc2500_state : public driver_device
{
public:
	risc2500_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
			m_maincpu(*this, "maincpu"),
			m_ram(*this, "ram"),
			m_nvram(*this, "nvram"),
			m_dac(*this, "dac"),
			m_inputs(*this, "P%u", 0)
		{ }

	DECLARE_READ32_MEMBER(p1000_r);
	DECLARE_WRITE32_MEMBER(p1000_w);
	DECLARE_READ32_MEMBER(disable_boot_rom);
	TIMER_CALLBACK_MEMBER(disable_boot_rom);

	virtual void machine_start() override;
	virtual void machine_reset() override;
	uint32_t screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	DECLARE_INPUT_CHANGED_MEMBER(on_button);
	void install_boot_rom();

private:
	required_device<cpu_device> m_maincpu;
	required_device<ram_device> m_ram;
	required_device<nvram_device> m_nvram;
	required_device<dac_byte_interface> m_dac;
	required_ioport_array<8> m_inputs;

	uint32_t  m_p1000;
	uint16_t  m_vram_addr;
	uint8_t   m_vram[0x100];
};


void risc2500_state::install_boot_rom()
{
	m_maincpu->space(AS_PROGRAM).install_rom(0x00000000, 0x001ffff, memregion("maincpu")->base());
}

uint32_t risc2500_state::screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	for(int c=0; c<12; c++)
	{
		// 12 characters 5 x 7
		for(int x=0; x<5; x++)
		{
			uint8_t gfx = BITSWAP8(m_vram[c*5 + x], 6,5,0,1,2,3,4,7);

			for(int y=0; y<7; y++)
				bitmap.pix16(y, 71 - (c*6 + x)) = (gfx >> (y + 1)) & 1;
		}

		// LCD digits and symbols
		int data_addr = 0x40 + c * 5;
		uint16_t data = ((m_vram[data_addr + 1] & 0x3) << 5) | ((m_vram[data_addr + 2] & 0x7) << 2) | (m_vram[data_addr + 4] & 0x3);
		data = BITSWAP8(data, 7,3,0,1,4,6,5,2) | ((m_vram[data_addr - 1] & 0x04) ? 0x80 : 0);

		output().set_digit_value(c, data);
		output().set_indexed_value("sym", c, BIT(m_vram[data_addr + 1], 2));
	}

	output().set_indexed_value("sym", 12, BIT(m_vram[0x63], 0));
	output().set_indexed_value("sym", 13, BIT(m_vram[0x4a], 0));

	return 0;
}

INPUT_CHANGED_MEMBER(risc2500_state::on_button)
{
	if (newval)
	{
		install_boot_rom();
		m_maincpu->reset();
	}
}

static INPUT_PORTS_START( risc2500 )
	PORT_START("P0")
	PORT_BIT(0x00000001, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // H1
	PORT_BIT(0x00000002, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // G1
	PORT_BIT(0x00000004, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // F1
	PORT_BIT(0x00000008, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // E1
	PORT_BIT(0x00000010, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // D1
	PORT_BIT(0x00000020, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // C1
	PORT_BIT(0x00000040, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // B1
	PORT_BIT(0x00000080, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // A1
	PORT_BIT(0x40000000, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_1_PAD)     PORT_NAME("Pawn")
	PORT_BIT(0x80000000, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_BACKSPACE) PORT_NAME("BACK")

	PORT_START("P1")
	PORT_BIT(0x00000001, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // H2
	PORT_BIT(0x00000002, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // G2
	PORT_BIT(0x00000004, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // F2
	PORT_BIT(0x00000008, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // E2
	PORT_BIT(0x00000010, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // D2
	PORT_BIT(0x00000020, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // C2
	PORT_BIT(0x00000040, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // B2
	PORT_BIT(0x00000080, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // A2
	PORT_BIT(0x40000000, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_2_PAD)     PORT_NAME("Knight")
	PORT_BIT(0x80000000, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_ENTER)     PORT_NAME("ENTER")

	PORT_START("P2")
	PORT_BIT(0x00000001, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // H3
	PORT_BIT(0x00000002, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // G3
	PORT_BIT(0x00000004, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // F3
	PORT_BIT(0x00000008, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // E3
	PORT_BIT(0x00000010, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // D3
	PORT_BIT(0x00000020, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // C3
	PORT_BIT(0x00000040, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // B3
	PORT_BIT(0x00000080, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // A3
	PORT_BIT(0x40000000, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_3_PAD)     PORT_NAME("Bishop")
	PORT_BIT(0x80000000, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_DOWN)      PORT_NAME("DOWN")

	PORT_START("P3")
	PORT_BIT(0x00000001, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // H4
	PORT_BIT(0x00000002, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // G4
	PORT_BIT(0x00000004, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // F4
	PORT_BIT(0x00000008, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // E4
	PORT_BIT(0x00000010, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // D4
	PORT_BIT(0x00000020, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // C4
	PORT_BIT(0x00000040, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // B4
	PORT_BIT(0x00000080, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // A4
	PORT_BIT(0x40000000, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_4_PAD)     PORT_NAME("Rook")
	PORT_BIT(0x80000000, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_UP)        PORT_NAME("UP")

	PORT_START("P4")
	PORT_BIT(0x00000001, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // H5
	PORT_BIT(0x00000002, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // G5
	PORT_BIT(0x00000004, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // F5
	PORT_BIT(0x00000008, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // E5
	PORT_BIT(0x00000010, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // D5
	PORT_BIT(0x00000020, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // C5
	PORT_BIT(0x00000040, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // B5
	PORT_BIT(0x00000080, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // A5
	PORT_BIT(0x40000000, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_5_PAD)     PORT_NAME("Queen")
	PORT_BIT(0x80000000, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_M)         PORT_NAME("MENU")

	PORT_START("P5")
	PORT_BIT(0x00000001, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // H6
	PORT_BIT(0x00000002, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // G6
	PORT_BIT(0x00000004, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // F6
	PORT_BIT(0x00000008, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // E6
	PORT_BIT(0x00000010, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // D6
	PORT_BIT(0x00000020, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // C6
	PORT_BIT(0x00000040, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // B6
	PORT_BIT(0x00000080, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // A6
	PORT_BIT(0x40000000, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_6_PAD)     PORT_NAME("King")
	PORT_BIT(0x80000000, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_L)         PORT_NAME("PLAY")

	PORT_START("P6")
	PORT_BIT(0x00000001, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // H7
	PORT_BIT(0x00000002, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // G7
	PORT_BIT(0x00000004, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // F7
	PORT_BIT(0x00000008, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // E7
	PORT_BIT(0x00000010, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // D7
	PORT_BIT(0x00000020, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // C7
	PORT_BIT(0x00000040, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // B7
	PORT_BIT(0x00000080, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // A7
	PORT_BIT(0x40000000, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_RIGHT)     PORT_NAME("RIGHT")
	PORT_BIT(0x80000000, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_N)         PORT_NAME("NEW GAME")

	PORT_START("P7")
	PORT_BIT(0x00000001, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // H8
	PORT_BIT(0x00000002, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // G8
	PORT_BIT(0x00000004, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // F8
	PORT_BIT(0x00000008, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // E8
	PORT_BIT(0x00000010, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // D8
	PORT_BIT(0x00000020, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // C8
	PORT_BIT(0x00000040, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // B8
	PORT_BIT(0x00000080, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_IMPULSE(10)             // A8
	PORT_BIT(0x40000000, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_LEFT)      PORT_NAME("LEFT")
	PORT_BIT(0x80000000, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_O)         PORT_NAME("OFF")

	PORT_START("RESET")
	PORT_BIT(0x00000001, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_I)         PORT_NAME("ON")  PORT_CHANGED_MEMBER(DEVICE_SELF, risc2500_state, on_button, 0)
INPUT_PORTS_END


READ32_MEMBER(risc2500_state::p1000_r)
{
	uint32_t data = 0;

	for(int i=0; i<8; i++)
	{
		if (m_p1000 & (1 << i))
			data |= m_inputs[i]->read();
	}

	return data;
}

WRITE32_MEMBER(risc2500_state::p1000_w)
{
	if ((data & 0xff000000) == 0x01000000)          // VRAM address
	{
		if (data & 0x80)
			m_vram_addr = (m_vram_addr & ~0x40) | (data & 0x01 ? 0x40 : 0);
		else
			m_vram_addr = (m_vram_addr & 0x40) | (data & 0xff);
	}
	else if (data & 0x04000000)                     // VRAM write
	{
		if (!(data & 0x08000000))
			m_vram[m_vram_addr++ & 0x7f] = data & 0xff;
	}
	else if (data & 0x80000000)                     // Vertical LED
	{
		for(int i=0; i<8; i++)
			output().set_led_value(i, BIT(data, i));
	}
	else if (data & 0x40000000)                     // Horizontal LED
	{
		for(int i=0; i<8; i++)
			output().set_led_value(8 + i, BIT(data, i));
	}
	else if ((data & 0xff000000) == 0x08000000)     // Power OFF
	{
		memset(m_vram, 0, sizeof(m_vram));
	}

	m_dac->write(data >> 28 & 3);                   // Speaker

	m_p1000 = data;
}

READ32_MEMBER(risc2500_state::disable_boot_rom)
{
	machine().scheduler().timer_set(m_maincpu->cycles_to_attotime(10), timer_expired_delegate(FUNC(risc2500_state::disable_boot_rom), this));
	return 0;
}

TIMER_CALLBACK_MEMBER(risc2500_state::disable_boot_rom)
{
	m_maincpu->space(AS_PROGRAM).install_ram(0x00000000, m_ram->size() - 1, m_ram->pointer());
}

void risc2500_state::machine_start()
{
	m_nvram->set_base(m_ram->pointer(), m_ram->size());

	save_item(NAME(m_p1000));
	save_item(NAME(m_vram_addr));
	save_item(NAME(m_vram));
}

void risc2500_state::machine_reset()
{
	m_p1000 = 0;
	m_vram_addr = 0;

	install_boot_rom();
}

static ADDRESS_MAP_START(risc2500_mem, AS_PROGRAM, 32, risc2500_state )
	AM_RANGE( 0x00000000,  0x0001ffff )  AM_RAM
	AM_RANGE( 0x01800000,  0x01800003 )  AM_READ(disable_boot_rom)
	AM_RANGE( 0x01000000,  0x01000003 )  AM_READWRITE(p1000_r, p1000_w)
	AM_RANGE( 0x02000000,  0x0203ffff )  AM_ROM AM_REGION("maincpu", 0)
ADDRESS_MAP_END


static MACHINE_CONFIG_START( risc2500 )
	MCFG_CPU_ADD("maincpu", ARM, XTAL_28_322MHz / 2)      // VY86C010
	MCFG_CPU_PROGRAM_MAP(risc2500_mem)
	MCFG_ARM_COPRO(VL86C020)
	MCFG_CPU_PERIODIC_INT_DRIVER(risc2500_state, irq1_line_hold, 250)

	MCFG_SCREEN_ADD("screen", LCD)
	MCFG_SCREEN_REFRESH_RATE(50)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(2500)) /* not accurate */
	MCFG_SCREEN_SIZE(12*6+1, 7)
	MCFG_SCREEN_VISIBLE_AREA(0, 12*6, 0, 7-1)
	MCFG_SCREEN_UPDATE_DRIVER(risc2500_state, screen_update)
	MCFG_SCREEN_PALETTE("palette")

	MCFG_DEFAULT_LAYOUT(layout_risc2500)

	MCFG_PALETTE_ADD_MONOCHROME("palette")

	MCFG_RAM_ADD("ram")
	MCFG_RAM_DEFAULT_SIZE("2M")
	MCFG_RAM_EXTRA_OPTIONS("128K, 256K, 512K, 1M, 2M")

	MCFG_NVRAM_ADD_NO_FILL("nvram")

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("speaker")
	MCFG_SOUND_ADD("dac", DAC_2BIT_BINARY_WEIGHTED_ONES_COMPLEMENT, 0) MCFG_SOUND_ROUTE(ALL_OUTPUTS, "speaker", 0.25) // unknown DAC
	MCFG_DEVICE_ADD("vref", VOLTAGE_REGULATOR, 0) MCFG_VOLTAGE_REGULATOR_OUTPUT(5.0)
	MCFG_SOUND_ROUTE_EX(0, "dac", 1.0, DAC_VREF_POS_INPUT) MCFG_SOUND_ROUTE_EX(0, "dac", -1.0, DAC_VREF_NEG_INPUT)
MACHINE_CONFIG_END


/* ROM definitions */

ROM_START( risc )
	ROM_REGION( 0x40000, "maincpu", ROMREGION_ERASE )
	ROM_SYSTEM_BIOS( 0, "v104", "v1.04" )
	ROMX_LOAD("s2500_v104.bin", 0x000000, 0x020000, CRC(84a06178) SHA1(66f4d9f53de6da865a3ebb4af1d6a3e245c59a3c), ROM_BIOS(1))
	ROM_SYSTEM_BIOS( 1, "v103", "v1.03" )
	ROMX_LOAD("s2500_v103.bin", 0x000000, 0x020000, CRC(7a707e82) SHA1(87187fa58117a442f3abd30092cfcc2a4d7c7efc), ROM_BIOS(2))
ROM_END

ROM_START( montreux )
	ROM_REGION( 0x40000, "maincpu", ROMREGION_ERASE )
	ROM_SYSTEM_BIOS( 0, "v100", "v1.00" )
	ROMX_LOAD("montreux.bin", 0x000000, 0x040000, CRC(db374cf3) SHA1(44dd60d56779084326c3dfb41d2137ebf0b4e0ac), ROM_BIOS(1))
ROM_END


/*    YEAR  NAME      PARENT   COMPAT  MACHINE    INPUT     STATE            INIT  COMPANY                      FULLNAME             FLAGS */
CONS( 1992, risc,     0,       0,      risc2500,  risc2500, risc2500_state,  0,    "Saitek",                    "RISC 2500",         MACHINE_CLICKABLE_ARTWORK )
CONS( 1995, montreux, 0,       0,      risc2500,  risc2500, risc2500_state,  0,    "Saitek / Hegener & Glaser", "Mephisto Montreux", MACHINE_CLICKABLE_ARTWORK )
