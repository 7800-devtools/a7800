// license: BSD-3-Clause
// copyright-holders: Frank Palazzolo, Dirk Best
/***************************************************************************

    Blockade/CoMOTION/Blasto/Hustle/Minesweeper

    TODO:
    - Noise generator
    - Timing for the coin input is a guess

***************************************************************************/

#include "emu.h"

#include "cpu/i8085/i8085.h"
#include "sound/discrete.h"
#include "sound/samples.h"
#include "screen.h"
#include "speaker.h"

#include "blockade.lh"


//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

class blockade_state : public driver_device
{
public:
	blockade_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_videoram(*this, "videoram"),
		m_gfxdecode(*this, "gfxdecode"),
		m_screen(*this, "screen"),
		m_discrete(*this, "discrete"),
		m_samples(*this, "samples"),
		m_vblank_timer(nullptr),
		m_tilemap(nullptr),
		m_coin_latch(0), m_coin_inserted(0)
	{ }

	DECLARE_INPUT_CHANGED_MEMBER(coin_inserted);
	DECLARE_CUSTOM_INPUT_MEMBER(coin_r);
	DECLARE_WRITE8_MEMBER(coin_latch_w);

	DECLARE_WRITE8_MEMBER(videoram_w);
	uint32_t screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	TILE_GET_INFO_MEMBER(tile_info);

	DECLARE_WRITE8_MEMBER(sound_freq_w);
	DECLARE_WRITE8_MEMBER(env_on_w);
	DECLARE_WRITE8_MEMBER(env_off_w);

protected:
	virtual void machine_start() override;
	virtual void machine_reset() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

private:
	required_device<cpu_device> m_maincpu;
	required_shared_ptr<uint8_t> m_videoram;
	required_device<gfxdecode_device> m_gfxdecode;
	required_device<screen_device> m_screen;
	required_device<discrete_device> m_discrete;
	required_device<samples_device> m_samples;

	emu_timer *m_vblank_timer;
	tilemap_t *m_tilemap;

	uint8_t m_coin_latch;
	uint8_t m_coin_inserted;
};


//**************************************************************************
//  ADDRESS MAPS
//**************************************************************************

static ADDRESS_MAP_START( main_map, AS_PROGRAM, 8, blockade_state )
	AM_RANGE(0x0000, 0x03ff) AM_MIRROR(0x6000) AM_ROM
	AM_RANGE(0x0400, 0x07ff) AM_MIRROR(0x6000) AM_ROM // comotion, blasto, hustle
	AM_RANGE(0x8000, 0x83ff) AM_MIRROR(0x6c00) AM_RAM_WRITE(videoram_w) AM_SHARE("videoram")
	AM_RANGE(0x9000, 0x90ff) AM_MIRROR(0x6f00) AM_RAM
ADDRESS_MAP_END

static ADDRESS_MAP_START( main_io_map, AS_IO, 8, blockade_state )
	AM_RANGE(0x01, 0x01) AM_READ_PORT("IN0") AM_WRITE(coin_latch_w)
	AM_RANGE(0x02, 0x02) AM_READ_PORT("IN1") AM_WRITE(sound_freq_w)
	AM_RANGE(0x04, 0x04) AM_READ_PORT("IN2") AM_WRITE(env_on_w)
	AM_RANGE(0x08, 0x08) AM_WRITE(env_off_w)
ADDRESS_MAP_END


//**************************************************************************
//  INPUT PORT DEFINITIONS
//**************************************************************************

static INPUT_PORTS_START( blockade )
	PORT_START("coin")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_COIN1) PORT_IMPULSE(24) PORT_CHANGED_MEMBER(DEVICE_SELF, blockade_state, coin_inserted, nullptr)

	// These are not dip switches, they are mapped to connectors on the board.  Different games
	// had different harnesses which plugged in here, and some pins were unused.
	PORT_START("IN0")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_CONFNAME(0x04, 0x04, "Boom Switch")
	PORT_CONFSETTING(   0x00, DEF_STR( Off ))
	PORT_CONFSETTING(   0x04, DEF_STR( On ))
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_CONFNAME(0x70, 0x70, DEF_STR( Lives ))
	PORT_CONFSETTING(   0x60, "3" )
	PORT_CONFSETTING(   0x50, "4" )
	PORT_CONFSETTING(   0x30, "5" )
	PORT_CONFSETTING(   0x70, "6" )
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_CUSTOM) PORT_CUSTOM_MEMBER(DEVICE_SELF, blockade_state, coin_r, nullptr)

	PORT_START("IN1")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_JOYSTICK_UP)    PORT_4WAY PORT_PLAYER(2)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT) PORT_4WAY PORT_PLAYER(2)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN)  PORT_4WAY PORT_PLAYER(2)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT)  PORT_4WAY PORT_PLAYER(2)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_JOYSTICK_UP)    PORT_4WAY PORT_PLAYER(1)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT) PORT_4WAY PORT_PLAYER(1)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN)  PORT_4WAY PORT_PLAYER(1)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT)  PORT_4WAY PORT_PLAYER(1)

	PORT_START("IN2")
	PORT_BIT(0xff, IP_ACTIVE_LOW, IPT_UNUSED)
INPUT_PORTS_END

static INPUT_PORTS_START( comotion )
	PORT_INCLUDE(blockade)

	PORT_MODIFY("IN0")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_CONFNAME(0x04, 0x04, "Boom Switch")
	PORT_CONFSETTING(   0x00, DEF_STR( Off ))
	PORT_CONFSETTING(   0x04, DEF_STR( On ))
	PORT_CONFNAME(0x08, 0x00, DEF_STR( Lives ))
	PORT_CONFSETTING(   0x00, "3")
	PORT_CONFSETTING(   0x08, "4")
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_START1)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_CUSTOM) PORT_CUSTOM_MEMBER(DEVICE_SELF, blockade_state, coin_r, nullptr)

	PORT_MODIFY("IN1")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_JOYSTICK_UP)    PORT_4WAY PORT_PLAYER(1)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT) PORT_4WAY PORT_PLAYER(1)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN)  PORT_4WAY PORT_PLAYER(1)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT)  PORT_4WAY PORT_PLAYER(1)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_JOYSTICK_UP)    PORT_4WAY PORT_PLAYER(3)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT) PORT_4WAY PORT_PLAYER(3)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN)  PORT_4WAY PORT_PLAYER(3)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT)  PORT_4WAY PORT_PLAYER(3)

	PORT_MODIFY("IN2")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_JOYSTICK_UP)    PORT_4WAY PORT_PLAYER(2)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT) PORT_4WAY PORT_PLAYER(2)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN)  PORT_4WAY PORT_PLAYER(2)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT)  PORT_4WAY PORT_PLAYER(2)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_JOYSTICK_UP)    PORT_4WAY PORT_PLAYER(4)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT) PORT_4WAY PORT_PLAYER(4)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN)  PORT_4WAY PORT_PLAYER(4)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT)  PORT_4WAY PORT_PLAYER(4)
INPUT_PORTS_END

static INPUT_PORTS_START( blasto )
	PORT_INCLUDE(blockade)

	PORT_MODIFY("IN0")
	PORT_CONFNAME(0x03, 0x03, DEF_STR( Coinage ))
	PORT_CONFSETTING(   0x00, DEF_STR( 4C_1C ))
	PORT_CONFSETTING(   0x01, DEF_STR( 3C_1C ))
	PORT_CONFSETTING(   0x02, DEF_STR( 2C_1C ))
	PORT_CONFSETTING(   0x03, DEF_STR( 1C_1C ))
	PORT_CONFNAME(0x04, 0x04, DEF_STR( Demo_Sounds ))
	PORT_CONFSETTING(   0x00, DEF_STR( Off ))
	PORT_CONFSETTING(   0x04, DEF_STR( On ))
	PORT_CONFNAME(0x08, 0x08, DEF_STR( Game_Time ))
	PORT_CONFSETTING(   0x00, "70 Secs") // though service manual says 60
	PORT_CONFSETTING(   0x08, "90 Secs")
	PORT_BIT(0x70, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_CUSTOM) PORT_CUSTOM_MEMBER(DEVICE_SELF, blockade_state, coin_r, nullptr)

	PORT_MODIFY("IN1")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_BUTTON1) PORT_PLAYER(2)
	PORT_BIT(0x1e, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_START1)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_START2)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_BUTTON1) PORT_PLAYER(1)

	PORT_MODIFY("IN2")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT) PORT_4WAY PORT_PLAYER(2)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN)  PORT_4WAY PORT_PLAYER(2)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT)  PORT_4WAY PORT_PLAYER(2)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_JOYSTICK_UP)    PORT_4WAY PORT_PLAYER(2)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT) PORT_4WAY PORT_PLAYER(1)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN)  PORT_4WAY PORT_PLAYER(1)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT)  PORT_4WAY PORT_PLAYER(1)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_JOYSTICK_UP)    PORT_4WAY PORT_PLAYER(1)
INPUT_PORTS_END

static INPUT_PORTS_START( hustle )
	PORT_INCLUDE(blockade)

	PORT_MODIFY("IN0")
	PORT_CONFNAME(0x03, 0x03, DEF_STR( Coinage ))
	PORT_CONFSETTING(   0x00, DEF_STR( 4C_1C ))
	PORT_CONFSETTING(   0x01, DEF_STR( 3C_1C ))
	PORT_CONFSETTING(   0x02, DEF_STR( 2C_1C ))
	PORT_CONFSETTING(   0x03, DEF_STR( 1C_1C ))
	PORT_CONFNAME(0x04, 0x04, DEF_STR( Game_Time ))
	PORT_CONFSETTING(   0x00, "1.5 mins" )
	PORT_CONFSETTING(   0x04, "2 mins" )
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_START1)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_START2)
	PORT_BIT(0x60, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_CUSTOM) PORT_CUSTOM_MEMBER(DEVICE_SELF, blockade_state, coin_r, nullptr)

	PORT_MODIFY("IN1")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_JOYSTICK_UP)    PORT_4WAY PORT_PLAYER(2)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT) PORT_4WAY PORT_PLAYER(2)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN)  PORT_4WAY PORT_PLAYER(2)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT)  PORT_4WAY PORT_PLAYER(2)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_JOYSTICK_UP)    PORT_4WAY PORT_PLAYER(1)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT) PORT_4WAY PORT_PLAYER(1)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN)  PORT_4WAY PORT_PLAYER(1)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT)  PORT_4WAY PORT_PLAYER(1)

	PORT_MODIFY("IN2")
	PORT_CONFNAME(0xf1, 0xf0, "Free Game")
	PORT_CONFSETTING(   0x71, "11000")
	PORT_CONFSETTING(   0xb1, "13000")
	PORT_CONFSETTING(   0xd1, "15000")
	PORT_CONFSETTING(   0xe1, "17000")
	PORT_CONFSETTING(   0xf0, "Disabled")
	PORT_BIT(0x0e, IP_ACTIVE_LOW, IPT_UNUSED)
INPUT_PORTS_END

static INPUT_PORTS_START( mineswpr )
	PORT_INCLUDE(blockade)

	PORT_MODIFY("IN0")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_OTHER) // This wiring selects upright mode
	PORT_CONFNAME(0x04, 0x04, "Boom Switch")
	PORT_CONFSETTING(   0x00, DEF_STR( Off ))
	PORT_CONFSETTING(   0x04, DEF_STR( On ))
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_CONFNAME(0x70, 0x70, DEF_STR( Lives ))
	PORT_CONFSETTING(   0x60, "3")
	PORT_CONFSETTING(   0x50, "4")
	PORT_CONFSETTING(   0x30, "5")
	PORT_CONFSETTING(   0x70, "6")
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_CUSTOM) PORT_CUSTOM_MEMBER(DEVICE_SELF, blockade_state, coin_r, nullptr)

	PORT_MODIFY("IN1")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_JOYSTICK_UP)    PORT_4WAY PORT_PLAYER(1)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT) PORT_4WAY PORT_PLAYER(1)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN)  PORT_4WAY PORT_PLAYER(1)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT)  PORT_4WAY PORT_PLAYER(1)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_JOYSTICK_UP)    PORT_4WAY PORT_PLAYER(2)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT) PORT_4WAY PORT_PLAYER(2)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN)  PORT_4WAY PORT_PLAYER(2)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT)  PORT_4WAY PORT_PLAYER(2)

	PORT_MODIFY("IN2")
	PORT_BIT(0xff, IP_ACTIVE_LOW, IPT_UNUSED)
INPUT_PORTS_END

static INPUT_PORTS_START( mineswpr4 )
	PORT_INCLUDE(blockade)

	PORT_MODIFY("IN0")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_OTHER) // This wiring selects cocktail mode
	PORT_CONFNAME(0x04, 0x04, "Boom Switch")
	PORT_CONFSETTING(   0x00, DEF_STR( Off ))
	PORT_CONFSETTING(   0x04, DEF_STR( On ))
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_CONFNAME(0x70, 0x70, DEF_STR( Lives ))
	PORT_CONFSETTING(   0x60, "3")
	PORT_CONFSETTING(   0x50, "4")
	PORT_CONFSETTING(   0x30, "5")
	PORT_CONFSETTING(   0x70, "6")
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_CUSTOM) PORT_CUSTOM_MEMBER(DEVICE_SELF, blockade_state, coin_r, nullptr)

	PORT_MODIFY("IN1")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_JOYSTICK_UP)    PORT_4WAY PORT_PLAYER(1)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT) PORT_4WAY PORT_PLAYER(1)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN)  PORT_4WAY PORT_PLAYER(1)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT)  PORT_4WAY PORT_PLAYER(1)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_JOYSTICK_UP)    PORT_4WAY PORT_PLAYER(2)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT) PORT_4WAY PORT_PLAYER(2)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN)  PORT_4WAY PORT_PLAYER(2)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT)  PORT_4WAY PORT_PLAYER(2)

	PORT_MODIFY("IN2")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_JOYSTICK_UP)    PORT_4WAY PORT_PLAYER(3)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT) PORT_4WAY PORT_PLAYER(3)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN)  PORT_4WAY PORT_PLAYER(3)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT)  PORT_4WAY PORT_PLAYER(3)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_JOYSTICK_UP)    PORT_4WAY PORT_PLAYER(4)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT) PORT_4WAY PORT_PLAYER(4)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN)  PORT_4WAY PORT_PLAYER(4)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT)  PORT_4WAY PORT_PLAYER(4)
INPUT_PORTS_END


//**************************************************************************
//  INPUT PORT HANDLING
//**************************************************************************

INPUT_CHANGED_MEMBER( blockade_state::coin_inserted )
{
	m_coin_inserted = newval;

	if (newval)
		m_maincpu->set_input_line(INPUT_LINE_RESET, PULSE_LINE);
}

CUSTOM_INPUT_MEMBER( blockade_state::coin_r )
{
	return m_coin_latch;
}

WRITE8_MEMBER( blockade_state::coin_latch_w )
{
	if (BIT(data, 7))
	{
		m_coin_latch = m_coin_inserted;
		m_coin_inserted = 0;
	}
}


//**************************************************************************
//  VIDEO
//**************************************************************************

WRITE8_MEMBER( blockade_state::videoram_w )
{
	m_videoram[offset] = data;
	m_tilemap->mark_tile_dirty(offset);

	// halt the cpu if it writes to ram while not in vblank
	if (!m_screen->vblank())
	{
		m_maincpu->set_input_line(INPUT_LINE_HALT, ASSERT_LINE);
		m_vblank_timer->adjust(m_screen->time_until_vblank_start());
	}
}

uint32_t blockade_state::screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	m_tilemap->draw(screen, bitmap, cliprect, 0, 0);
	return 0;
}

static GFXDECODE_START( blockade )
	GFXDECODE_ENTRY( "gfx1", 0, gfx_8x8x1, 0, 1 )
GFXDECODE_END

TILE_GET_INFO_MEMBER( blockade_state::tile_info )
{
	int code = m_videoram[tile_index];
	SET_TILE_INFO_MEMBER(0, code, 0, 0);
}


//**************************************************************************
//  AUDIO
//**************************************************************************

// This still needs the noise generator stuff along with proper mixing and volume control

#define BLOCKADE_NOTE_DATA      NODE_01
#define BLOCKADE_NOTE           NODE_02

DISCRETE_SOUND_START( blockade )
	DISCRETE_INPUT_DATA  (BLOCKADE_NOTE_DATA)

	/************************************************/
	/* Note sound is created by a divider circuit.  */
	/* The master clock is the 93681.5 Hz, from the */
	/* 555 oscillator.  This is then sent to a      */
	/* preloadable 8 bit counter, which loads the   */
	/* value from OUT02 when overflowing from 0xFF  */
	/* to 0x00.  Therefore it divides by 2 (OUT02   */
	/* = FE) to 256 (OUT02 = 00).                   */
	/* There is also a final /2 stage.              */
	/* Note that there is no music disable line.    */
	/* When there is no music, the game sets the    */
	/* oscillator to 0Hz.  (OUT02 = FF)             */
	/************************************************/
	DISCRETE_NOTE(BLOCKADE_NOTE, 1, 93681.5, BLOCKADE_NOTE_DATA, 255, 1, DISC_CLK_IS_FREQ | DISC_OUT_IS_ENERGY)
	DISCRETE_CRFILTER(NODE_10, BLOCKADE_NOTE, RES_K(35), CAP_U(.01))

	DISCRETE_OUTPUT(NODE_10, 7500)
DISCRETE_SOUND_END

WRITE8_MEMBER( blockade_state::sound_freq_w )
{
	m_discrete->write(space, BLOCKADE_NOTE_DATA, data);
	return;
}

WRITE8_MEMBER( blockade_state::env_on_w )
{
	m_samples->start(0, 0);
	return;
}

WRITE8_MEMBER( blockade_state::env_off_w )
{
	return;
}

const char *const blockade_sample_names[] =
{
	"*blockade",
	"boom",
	nullptr
};


//**************************************************************************
//  MACHINE EMULATION
//**************************************************************************

void blockade_state::machine_start()
{
	m_tilemap = &machine().tilemap().create(*m_gfxdecode, tilemap_get_info_delegate(FUNC(blockade_state::tile_info), this), TILEMAP_SCAN_ROWS, 8, 8, 32, 32);

	m_vblank_timer = timer_alloc(0);

	// register for save states
	save_item(NAME(m_coin_latch));
	save_item(NAME(m_coin_inserted));
}

void blockade_state::machine_reset()
{
	m_coin_latch = 0;
	m_coin_inserted = 0;
}

void blockade_state::device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr)
{
	// resume cpu, on the real system, this is connected the READY input
	m_maincpu->set_input_line(INPUT_LINE_HALT, CLEAR_LINE);
}


//**************************************************************************
//  MACHINE DEFINTIONS
//**************************************************************************

static MACHINE_CONFIG_START( blockade )
	MCFG_CPU_ADD("maincpu", I8080A, XTAL_20_079MHz / 10)
	MCFG_CPU_PROGRAM_MAP(main_map)
	MCFG_CPU_IO_MAP(main_io_map)

	// video hardware
	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_RAW_PARAMS(XTAL_20_079MHz / 4, 330, 0, 256, 262, 0, 224)
	MCFG_SCREEN_UPDATE_DRIVER(blockade_state, screen_update)
	MCFG_SCREEN_PALETTE("palette")

	MCFG_GFXDECODE_ADD("gfxdecode", "palette", blockade)

	MCFG_PALETTE_ADD_MONOCHROME("palette")

	// sound hardware
	MCFG_SPEAKER_STANDARD_MONO("mono")

	MCFG_SOUND_ADD("samples", SAMPLES, 0)
	MCFG_SAMPLES_CHANNELS(1)
	MCFG_SAMPLES_NAMES(blockade_sample_names)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.25)

	MCFG_SOUND_ADD("discrete", DISCRETE, 0)
	MCFG_DISCRETE_INTF(blockade)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 1.0)
MACHINE_CONFIG_END


//**************************************************************************
//  ROM DEFINITIONS
//**************************************************************************

ROM_START( blockade )
	ROM_REGION(0x800, "maincpu", 0)
	ROM_LOAD_NIB_HIGH("316-0004.u2", 0x000, 0x400, CRC(a93833e9) SHA1(e29e7b29900f8305effa700a53806a12bf9d37bd))
	ROM_LOAD_NIB_LOW( "316-0003.u3", 0x000, 0x400, CRC(85960d3b) SHA1(aabfe8f9c26126299d6c07a31ef1aac5300deff5))

	ROM_REGION(0x100, "gfx1", 0)
	ROM_LOAD_NIB_HIGH("316-0002.u29", 0x000, 0x100, CRC(409f610f) SHA1(0c2253f4b72d8aa395f87cc0abe07f0b46fa538b))
	ROM_LOAD_NIB_LOW( "316-0001.u43", 0x000, 0x100, CRC(41a00b28) SHA1(2d0a90aac9d10a1ded240e5202fdf9cd7f70c4a7))
ROM_END

ROM_START( comotion )
	ROM_REGION(0x800, "maincpu", 0)
	ROM_LOAD_NIB_HIGH("316-0007.u2", 0x000, 0x400, CRC(5b9bd054) SHA1(324b844788945e7bc82d096d6d375e79e3e1a634))
	ROM_LOAD_NIB_LOW( "316-0008.u3", 0x000, 0x400, CRC(1a856042) SHA1(91bdc260e8c88ce2b6ac05bfba043ed611bc30de))
	ROM_LOAD_NIB_HIGH("316-0009.u4", 0x400, 0x400, CRC(2590f87c) SHA1(95a7af04b610d79fb3f6d74dda322e66164b9484))
	ROM_LOAD_NIB_LOW( "316-0010.u5", 0x400, 0x400, CRC(fb49a69b) SHA1(4009c3256a86508d981c1f77b65e6bff1face1e7))

	ROM_REGION(0x100, "gfx1", 0)
	ROM_LOAD_NIB_HIGH("316-0006.u43", 0x000, 0x100, CRC(8f071297) SHA1(811471c87b77b4b9ab056cf0c0743fc2616b754c))  // these are reversed
	ROM_LOAD_NIB_LOW( "316-0005.u29", 0x000, 0x100, CRC(53fb8821) SHA1(0a499aa4cf15f7ebea155aacd914de8851544215))
ROM_END

ROM_START( blasto )
	ROM_REGION(0x800, "maincpu", 0)
	ROM_LOAD_NIB_HIGH("316-0089.u2", 0x000, 0x400, CRC(ec99d043) SHA1(10650e54bf55f3ace5c199215c2fce211916d3b7))
	ROM_LOAD_NIB_LOW( "316-0090.u3", 0x000, 0x400, CRC(be333415) SHA1(386cab720f0c2da16b9ec84f67ccebf23406c58d))
	ROM_LOAD_NIB_HIGH("316-0091.u4", 0x400, 0x400, CRC(1c889993) SHA1(e23c72d075cf3d209081bca5a953c33c8ae042ea))
	ROM_LOAD_NIB_LOW( "316-0092.u5", 0x400, 0x400, CRC(efb640cb) SHA1(2dff5b249f876d7d13cc6dfad652ce7e5af10370))

	ROM_REGION(0x200, "gfx1", 0)
	ROM_LOAD_NIB_HIGH("316-0093.u29", 0x000, 0x200, CRC(4dd69499) SHA1(34f097477a297bf5f986804e5967c92f9292be29))
	ROM_LOAD_NIB_LOW( "316-0094.u43", 0x000, 0x200, CRC(104051a4) SHA1(cae6b9d48e3eda5ba12ff5d9835ce2733e90f774))
ROM_END

ROM_START( hustle )
	ROM_REGION(0x800, "maincpu", 0)
	ROM_LOAD_NIB_HIGH("316-0016.u2", 0x000, 0x400, CRC(d983de7c) SHA1(af6e0ea78449bfba4fe8affd724d7b0eb3d38706))
	ROM_LOAD_NIB_LOW( "316-0017.u3", 0x000, 0x400, CRC(edec9cb9) SHA1(548cc7b0a15a1c977b7ef4a99ff88101893f661a))
	ROM_LOAD_NIB_HIGH("316-0018.u4", 0x400, 0x400, CRC(f599b9c0) SHA1(c55ed33ac51b9cfbb2fe4321bbb1e0a16694f065))
	ROM_LOAD_NIB_LOW( "316-0019.u5", 0x400, 0x400, CRC(7794bc7e) SHA1(b3d577291dea0e096b2ee56b0ef612f41b2e859c))

	ROM_REGION(0x200, "gfx1", 0)
	ROM_LOAD_NIB_HIGH("316-0020.u29", 0x000, 0x200, CRC(541d2c67) SHA1(abdb918f302352693870b0a50eabaf95acf1cf63))
	ROM_LOAD_NIB_LOW( "316-0021.u43", 0x000, 0x200, CRC(b5083128) SHA1(d7e8242e9d12d09f3d69c08e373ede2bdd4deba9))
ROM_END

ROM_START( mineswpr )
	ROM_REGION(0x800, "maincpu", 0)
	ROM_LOAD_NIB_HIGH("mineswee.h0p", 0x000, 0x400, CRC(5850a4ba) SHA1(9f097d31428f4494573187049c53fbed2075ff32))
	ROM_LOAD_NIB_LOW( "mineswee.l0p", 0x000, 0x400, CRC(05961379) SHA1(3d59341be8a663e8c54c1556442c992a6eb886ab))

	ROM_REGION(0x200, "gfx1", 0)
	ROM_LOAD_NIB_HIGH("mineswee.ums", 0x000, 0x200, CRC(0e1c5c37) SHA1(d4d56bd63307e387771c48304724dfc1ea1306d9))
	ROM_LOAD_NIB_LOW( "mineswee.uls", 0x000, 0x200, CRC(3a4f66e1) SHA1(bd7f6c51d568a79fb06414b2a6ef245d0d983c3e))
ROM_END

ROM_START( mineswpr4 )
	ROM_REGION(0x800, "maincpu", 0)
	ROM_LOAD_NIB_HIGH("mineswee.h0p", 0x000, 0x400, CRC(5850a4ba) SHA1(9f097d31428f4494573187049c53fbed2075ff32))
	ROM_LOAD_NIB_LOW( "mineswee.l0p", 0x000, 0x400, CRC(05961379) SHA1(3d59341be8a663e8c54c1556442c992a6eb886ab))

	ROM_REGION(0x200, "gfx1", 0)
	ROM_LOAD_NIB_HIGH("mineswee.cms", 0x000, 0x200, CRC(aad3ce0c) SHA1(92257706ae0c9c1a258eed3311116063e647e1ae))
	ROM_LOAD_NIB_LOW( "mineswee.cls", 0x000, 0x200, CRC(70959755) SHA1(f62d448742da3fae8bbd96eb3a2714db500cecce))
ROM_END


//**************************************************************************
//  SYSTEM DRIVERS
//**************************************************************************

//    YEAR  NAME       PARENT    MACHINE   INPUT      CLASS           INIT  ROTATION  COMPANY    FULLNAME                  FLAGS                                            LAYOUT
GAMEL(1976, blockade,  0,        blockade, blockade,  blockade_state, 0,    ROT0,     "Gremlin", "Blockade",               MACHINE_IMPERFECT_SOUND | MACHINE_SUPPORTS_SAVE, layout_blockade )
GAMEL(1976, comotion,  0,        blockade, comotion,  blockade_state, 0,    ROT0,     "Gremlin", "CoMOTION",               MACHINE_IMPERFECT_SOUND | MACHINE_SUPPORTS_SAVE, layout_blockade )
GAME( 1978, blasto,    0,        blockade, blasto,    blockade_state, 0,    ROT0,     "Gremlin", "Blasto",                 MACHINE_IMPERFECT_SOUND | MACHINE_SUPPORTS_SAVE ) // b/w, no overlay
GAMEL(1977, hustle,    0,        blockade, hustle,    blockade_state, 0,    ROT0,     "Gremlin", "Hustle",                 MACHINE_IMPERFECT_SOUND | MACHINE_SUPPORTS_SAVE, layout_blockade )
GAME( 1977, mineswpr,  0,        blockade, mineswpr,  blockade_state, 0,    ROT0,     "Amutech", "Minesweeper",            MACHINE_IMPERFECT_SOUND | MACHINE_SUPPORTS_SAVE )
GAME( 1977, mineswpr4, mineswpr, blockade, mineswpr4, blockade_state, 0,    ROT0,     "Amutech", "Minesweeper (4-Player)", MACHINE_IMPERFECT_SOUND | MACHINE_SUPPORTS_SAVE )
