// license:BSD-3-Clause
// copyright-holders:Ryan Holtz
/**********************************************************************

 "Craft" demo platform by Linus Akesson

 driver by Ryan Holtz

 **********************************************************************/

#include "emu.h"
#include "cpu/avr8/avr8.h"
#include "sound/dac.h"
#include "sound/volt_reg.h"
#include "screen.h"
#include "speaker.h"

#define VERBOSE_LEVEL   (0)

#define ENABLE_VERBOSE_LOG (0)

#define MASTER_CLOCK    20000000

#define VISIBLE_CYCLES      480
#define HSYNC_CYCLES        155
#define LINE_CYCLES         (VISIBLE_CYCLES + HSYNC_CYCLES)
#define VISIBLE_LINES       480
#define VSYNC_LINES         45
#define LINES_PER_FRAME     (VISIBLE_LINES + VSYNC_LINES)
#define CYCLES_PER_FRAME    (LINES_PER_FRAME * LINE_CYCLES)
#define PIXELS_PER_FRAME    (CYCLES_PER_FRAME)

/****************************************************\
* I/O devices                                        *
\****************************************************/

class craft_state : public driver_device
{
public:
	craft_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_dac(*this, "dac")
	{
	}

	void video_update();

	virtual void machine_start() override;

	uint32_t m_last_cycles;
	uint64_t m_frame_start_cycle;

	uint8_t m_port_b;
	uint8_t m_port_c;
	uint8_t m_port_d;

	uint8_t m_latched_color;
	uint8_t m_pixels[PIXELS_PER_FRAME];

	required_device<avr8_device> m_maincpu;

	DECLARE_READ8_MEMBER(port_r);
	DECLARE_WRITE8_MEMBER(port_w);
	DECLARE_DRIVER_INIT(craft);
	virtual void machine_reset() override;
	uint32_t screen_update_craft(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);
	inline void verboselog(int n_level, const char *s_fmt, ...) ATTR_PRINTF(3,4);
	required_device<dac_byte_interface> m_dac;
};

inline void craft_state::verboselog(int n_level, const char *s_fmt, ...)
{
#if ENABLE_VERBOSE_LOG
	if( VERBOSE_LEVEL >= n_level )
	{
		va_list v;
		char buf[ 32768 ];
		va_start( v, s_fmt );
		vsprintf( buf, s_fmt, v );
		va_end( v );
		logerror( "%08x: %s", m_maincpu->safe_pc(), buf );
	}
#endif
}

void craft_state::machine_start()
{
}

READ8_MEMBER(craft_state::port_r)
{
	switch( offset )
	{
		case 0x00: // Port A
		case 0x01: // Port B
		case 0x02: // Port C
		case 0x03: // Port D
			// Unhandled
			return 0x00;
	}

	return 0;
}

WRITE8_MEMBER(craft_state::port_w)
{
	switch( offset )
	{
		case AVR8_IO_PORTA: // Port A
			// Unhandled
			break;

		case AVR8_IO_PORTB: // Port B
		{
			uint8_t old_port_b = m_port_b;
			uint8_t pins = data;
			uint8_t changed = pins ^ old_port_b;
			if(pins & changed & 0x02)
			{
				m_frame_start_cycle = m_maincpu->get_elapsed_cycles();
				video_update();
			}
			if(changed & 0x08)
			{
				video_update();
				m_latched_color = (pins & 0x08) ? (m_port_c & 0x3f) : 0x3f;
			}
			m_port_b = data;
			break;
		}

		case AVR8_IO_PORTC: // Port C
			video_update();
			m_port_c = data;
			m_latched_color = m_port_c;
			break;

		case AVR8_IO_PORTD: // Port D
		{
			m_port_d = data;
			uint8_t audio_sample = (data & 0x02) | ((data & 0xf4) >> 2);
			m_dac->write(audio_sample);
			break;
		}
	}
}

void craft_state::video_update()
{
	uint64_t cycles = m_maincpu->get_elapsed_cycles();
	uint32_t frame_cycles = (uint32_t)(cycles - m_frame_start_cycle);

	if (m_last_cycles < frame_cycles)
	{
		for (uint32_t pixidx = m_last_cycles; pixidx < frame_cycles && pixidx < PIXELS_PER_FRAME; pixidx++)
		{
			m_pixels[pixidx] = m_latched_color;
		}
	}
	else
	{
		uint32_t end_clear = sizeof(m_pixels) - m_last_cycles;
		uint32_t start_clear = frame_cycles;
		end_clear = (end_clear > PIXELS_PER_FRAME) ? (PIXELS_PER_FRAME - m_last_cycles) : end_clear;
		start_clear = (start_clear > PIXELS_PER_FRAME) ? PIXELS_PER_FRAME : start_clear;
		if (m_last_cycles < PIXELS_PER_FRAME)
		{
			memset(m_pixels + m_last_cycles, 0, end_clear);
		}
		if (start_clear < PIXELS_PER_FRAME)
		{
			memset(m_pixels, 0, start_clear);
		}
	}

	m_last_cycles = frame_cycles;
}

/****************************************************\
* Address maps                                       *
\****************************************************/

static ADDRESS_MAP_START( craft_prg_map, AS_PROGRAM, 8, craft_state )
	AM_RANGE(0x0000, 0x1fff) AM_ROM
ADDRESS_MAP_END

static ADDRESS_MAP_START( craft_data_map, AS_DATA, 8, craft_state )
	AM_RANGE(0x0100, 0x04ff) AM_RAM
ADDRESS_MAP_END

static ADDRESS_MAP_START( craft_io_map, AS_IO, 8, craft_state )
	AM_RANGE(AVR8_IO_PORTA, AVR8_IO_PORTD) AM_READWRITE( port_r, port_w )
ADDRESS_MAP_END

/****************************************************\
* Input ports                                        *
\****************************************************/

static INPUT_PORTS_START( craft )
	PORT_START("MAIN")
		PORT_BIT(0xff, IP_ACTIVE_HIGH, IPT_UNUSED)
INPUT_PORTS_END

/****************************************************\
* Video hardware                                     *
\****************************************************/

uint32_t craft_state::screen_update_craft(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
	for(int y = 0; y < LINES_PER_FRAME; y++)
	{
		uint32_t *line = &bitmap.pix32(y);
		for(int x = 0; x < LINE_CYCLES; x++)
		{
			uint8_t pixel = m_pixels[y * LINE_CYCLES + x];
			uint8_t r = 0x55 * ((pixel & 0x30) >> 4);
			uint8_t g = 0x55 * ((pixel & 0x0c) >> 2);
			uint8_t b = 0x55 * (pixel & 0x03);
			line[x] = 0xff000000 | (r << 16) | (g << 8) | b;
		}
	}
	return 0;
}

/****************************************************\
* Machine definition                                 *
\****************************************************/

DRIVER_INIT_MEMBER(craft_state,craft)
{
}

void craft_state::machine_reset()
{
	m_frame_start_cycle = 0;
	m_last_cycles = 0;
}

static MACHINE_CONFIG_START( craft )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", ATMEGA88, MASTER_CLOCK)
	MCFG_CPU_PROGRAM_MAP(craft_prg_map)
	MCFG_CPU_DATA_MAP(craft_data_map)
	MCFG_CPU_IO_MAP(craft_io_map)
	MCFG_CPU_AVR8_EEPROM("eeprom")

	/* video hardware */
	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(59.99)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(1429)) /* accurate */
	MCFG_SCREEN_SIZE(635, 525)
	MCFG_SCREEN_VISIBLE_AREA(47, 526, 36, 515)
	MCFG_SCREEN_UPDATE_DRIVER(craft_state, screen_update_craft)
	MCFG_PALETTE_ADD("palette", 0x1000)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("avr8")
	MCFG_SOUND_ADD("dac", DAC_6BIT_R2R, 0) MCFG_SOUND_ROUTE(0, "avr8", 0.25) // pd1/pd2/pd4/pd5/pd6/pd7 + 2k(x7) + 1k(x5)
	MCFG_DEVICE_ADD("vref", VOLTAGE_REGULATOR, 0) MCFG_VOLTAGE_REGULATOR_OUTPUT(5.0)
	MCFG_SOUND_ROUTE_EX(0, "dac", 1.0, DAC_VREF_POS_INPUT) MCFG_SOUND_ROUTE_EX(0, "dac", -1.0, DAC_VREF_NEG_INPUT)
MACHINE_CONFIG_END

ROM_START( craft )
	ROM_REGION( 0x2000, "maincpu", 0 )  /* Main program store */
	ROM_LOAD( "craft.bin", 0x0000, 0x2000, CRC(2e6f9ad2) SHA1(75e495bf18395d74289ca7ee2649622fc4010457) )
	ROM_REGION( 0x200, "eeprom", 0 )  /* on-die eeprom */
	ROM_LOAD( "eeprom.raw", 0x0000, 0x0200, CRC(e18a2af9) SHA1(81fc6f2d391edfd3244870214fac37929af0ac0c) )
ROM_END

/*   YEAR  NAME      PARENT    COMPAT    MACHINE   INPUT  STATE           INIT      COMPANY          FULLNAME */
CONS(2008, craft,    0,        0,        craft,    craft, craft_state,    craft,    "Linus Akesson", "Craft", MACHINE_NOT_WORKING)
