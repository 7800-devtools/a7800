// license:BSD-3-Clause
// copyright-holders:Fabio Priuli, Scott Stone, Couriersud
/***************************************************************************

 Atari / Kee Games Driver - Discrete Games made in the 1970's


 Atari / Kee Games List (except for most Pong games) - Data based, in part from:

 - "Andy's collection of Bronzeage Atari Video Arcade PCBs"
 http://www.andysarcade.net/personal/bronzeage/index.htm

 - "Atari's Technical Manual Log"
 http://www.atarigames.com/manuals.txt

 Suspected "same games" are grouped together.  These are usually the exact same game but different cabinet/name.


 Technical Manual #s      Game Name(s)                                                    Atari Part #'s                     Data      PROM/ROM Chip Numbers
 -----------------------+---------------------------------------------------------------+----------------------------------+---------+---------------------------------------
 TM-025                   Anti-Aircraft (1975)                                            A000951                            YES       003127
 TM-048                   Crash 'N Score/Stock Car (1975)                                 A004256                            YES       003186(x2), 003187(x2), 004248, 004247
 TM-030                   Crossfire (1975)                                                A003022                            NO?
 TM-022                   Elimination! (1973)                                             A000845                            NO
 TM-035                   Goal IV (1975)                                                  A000823                            NO
 TM-016,029               Gotcha/Color Gotcha (1973)                                      A000816                            NO
 TM-003,005,011,020,029   Gran Trak 10/Trak 10/Formula K (1974)                           A000872,A000872 K3RT               YES       74186 Racetrack Prom (K5)
 TM-004,021               Gran Trak 20/Trak 20/Twin Racer (1974)                          A001791(RT20),A001793(A20-K4DRTA)  YES       74186 Racetrack prom (K5)
 TM-028                   Hi-Way/Highway (1975)                                           A003211                            NO
 TM-055                   Indy 4 (1976)                                                   A003000,A006268,A006270            YES       003186, 003187, 005502-01, 05503-01
 TM-026                   Indy 800 (1975)                                                 A003000,A003170,A003182            YES       003186-003189 (4)
                                                                                          A003184,A003191,A003198,A003199
 TM-027,052               Jet Fighter/Jet Fighter Cocktail/Launch Aircraft (1975)         A004254,A004255                    YES       004250-004252, 004253-01 to 03 (3)
 TM-077                   Le Mans (1976)                                                  A005844,A005845                    YES       005837-01, 005838-01, 005839-01
 TM-040                   Outlaw (1976)                                                   A003213                            YES       003323 - ROM (8205 @ J4)
 TM-007                   Pin Pong (1974)                                                 A001660                            NO
 TM-019                   Pursuit (1975)                                                  K8P-B 90128                        NO
 TM-012,029,034           Quadrapong (1974)                                               A000845                            NO
 TM-009                   Qwak!/Quack (1974)                                              A000937,A000953                    YES       72074/37-2530N (K9)
 TM-001,023,029,032       Rebound/Spike/Volleyball (1974)                                 A000517,A000846,SPIKE-(A or B)     NO
 TM-047                   Shark JAWS (1975)                                               A003806                            YES       004182, 004183
 TM-008,029               Space Race (1973)                                               A000803                            NO
 TM-046                   Steeplechase/Astroturf (1975)                                   A003750                            YES       003774 ROM Bugle (C8), 003773-01 "A" Horse (C4), 003773-02 "B" Horse (D4)
 TM-057                   Stunt Cycle (1976)                                              A004128                            YES       004275 ROM Motorcycle/Bus (1F), 004811 ROM Score Translator (D7)
 TM-010,036               Tank/Tank Cocktail (1974)                                       A003111 (K5T-F 90124)              YES       90-2006 004800SD Tank Rom (K10)
 TM-049                   Tank II (1975)                                                  K5T-F 90124                        YES       90-2006
 TM-002                   Touch-Me (1974)                                                 ???????                            NO
 TM-006,017,029           World Cup/World Cup Football/Coupe du Monde/Coup Franc (1974)   A000823                            NO

 - Not Known to be released or produced, but at least announced.

 TM-0??                   Arcade Driver/Driver 1st Person (Not Produced/Released) (1974-75?)
 TM-018                   Dodgeball/Dodgem (Not Produced/Released) (1975)
 TM-024                   Qwakers (Not Produced/Released) (1974?) (Kee Games clone of Qwak!?)

 - Information (current as of 21 Dec. 2016) on what logic chips (and some analog parts) are still needed to be emulated in the
   netlist system per-game:

 TM-057 (Stunt Cycle)
    566    Voltage-Controlled Oscillator
    1N751A Zener Diode
    1N752A Zener Diode

 TM-055 (Indy 4)
    7406  Hex Inverter Buffers/Drivers with O.C. H.V. Outputs (note: Might not be needed, could just clone from 7404)
    7414  Hex Schmitt-Trigger Inverters
    7417  Hex Buffers/Drivers
    74164 8-bit Serial-In, Parallel-Out Shift Register
    9301  1-of-10 Decoder
    LM339 Quad Comparator

***************************************************************************/


#include "emu.h"

#include "machine/netlist.h"
#include "machine/nl_stuntcyc.h"
#include "netlist/devices/net_lib.h"
#include "video/fixfreq.h"
#include "screen.h"


// copied by Pong, not accurate for this driver!
// start
#define MASTER_CLOCK    7159000
#define V_TOTAL         (0x105+1)       // 262
#define H_TOTAL         (0x1C6+1)       // 454

#define HBSTART                 (H_TOTAL)
#define HBEND                   (32)
#define VBSTART                 (V_TOTAL)
#define VBEND                   (16)

#define HRES_MULT                   (1)
// end

#define SC_VIDCLOCK     (14318181/2)
#define SC_HTOTAL       (0x1C8+1)       // 456
#define SC_VTOTAL       (0x103+1)       // 259
#define SC_HBSTART      (SC_HTOTAL)
#define SC_HBEND        (32)
#define SC_VBSTART      (SC_VTOTAL)
#define SC_VBEND        (8)

class atarikee_state : public driver_device
{
public:
	atarikee_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_video(*this, "fixfreq")
	{
	}

	// devices
	required_device<netlist_mame_device> m_maincpu;
	required_device<fixedfreq_device> m_video;

protected:

	// driver_device overrides
	virtual void machine_start() override;
	virtual void machine_reset() override;

	virtual void video_start() override;

private:

};

class stuntcyc_state : public driver_device
{
public:
	stuntcyc_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		//, m_video(*this, "fixfreq")
		, m_probe_screen(*this, "screen")
		, m_probe_bit0(0.0)
		, m_probe_bit1(0.0)
		, m_probe_bit2(0.0)
		, m_probe_bit3(0.0)
		, m_probe_bit4(0.0)
		, m_probe_bit5(0.0)
		, m_probe_bit6(0.0)
		, m_probe_data(nullptr)
		, m_last_beam(0.0)
		, m_last_hpos(0)
		, m_last_vpos(0)
		, m_last_fraction(0.0)
	{
	}

	NETDEV_LOGIC_CALLBACK_MEMBER(probe_bit0_cb);
	NETDEV_LOGIC_CALLBACK_MEMBER(probe_bit1_cb);
	NETDEV_LOGIC_CALLBACK_MEMBER(probe_bit2_cb);
	NETDEV_LOGIC_CALLBACK_MEMBER(probe_bit3_cb);
	NETDEV_LOGIC_CALLBACK_MEMBER(probe_bit4_cb);
	NETDEV_LOGIC_CALLBACK_MEMBER(probe_bit5_cb);
	NETDEV_LOGIC_CALLBACK_MEMBER(probe_bit6_cb);
	NETDEV_LOGIC_CALLBACK_MEMBER(probe_clock_cb);

	uint32_t screen_update_stuntcyc(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);

protected:

	// driver_device overrides
	virtual void machine_start() override;
	virtual void machine_reset() override;

private:
	required_device<netlist_mame_device> m_maincpu;
	//required_device<fixedfreq_device> m_video;
	required_device<screen_device> m_probe_screen;

	int m_probe_bit0;
	int m_probe_bit1;
	int m_probe_bit2;
	int m_probe_bit3;
	int m_probe_bit4;
	int m_probe_bit5;
	int m_probe_bit6;

	std::unique_ptr<int[]> m_probe_data;

	int m_last_beam;
	int m_last_hpos;
	int m_last_vpos;
	double m_last_fraction;
};

static NETLIST_START(atarikee)
	SOLVER(Solver, 48000)
//  PARAM(Solver.FREQ, 48000)
	PARAM(Solver.ACCURACY, 1e-4) // works and is sufficient

	// schematics
	//...

//  NETDEV_ANALOG_CALLBACK(sound_cb, sound, atarikee_state, sound_cb, "")
//  NETDEV_ANALOG_CALLBACK(video_cb, videomix, fixedfreq_device, update_vid, "fixfreq")
NETLIST_END()


void atarikee_state::machine_start()
{
}

void atarikee_state::machine_reset()
{
}

void atarikee_state::video_start()
{
}

void stuntcyc_state::machine_start()
{
	save_item(NAME(m_probe_bit0));
	save_item(NAME(m_probe_bit1));
	save_item(NAME(m_probe_bit2));
	save_item(NAME(m_probe_bit3));
	save_item(NAME(m_probe_bit4));
	save_item(NAME(m_probe_bit5));
	save_item(NAME(m_probe_bit6));
	save_item(NAME(m_last_beam));
	save_item(NAME(m_last_hpos));
	save_item(NAME(m_last_vpos));
	save_item(NAME(m_last_fraction));

	m_probe_bit0 = 0;
	m_probe_bit1 = 0;
	m_probe_bit2 = 0;
	m_probe_bit3 = 0;
	m_probe_bit4 = 0;
	m_probe_bit5 = 0;
	m_probe_bit6 = 0;

	m_probe_data = std::make_unique<int[]>(SC_HTOTAL * SC_VTOTAL);
}

void stuntcyc_state::machine_reset()
{
	m_probe_bit0 = 0;
	m_probe_bit1 = 0;
	m_probe_bit2 = 0;
	m_probe_bit3 = 0;
	m_probe_bit4 = 0;
	m_probe_bit5 = 0;
	m_probe_bit6 = 0;
}

NETDEV_LOGIC_CALLBACK_MEMBER(stuntcyc_state::probe_bit0_cb) { m_probe_bit0 = data; }
NETDEV_LOGIC_CALLBACK_MEMBER(stuntcyc_state::probe_bit1_cb) { m_probe_bit1 = data; }
NETDEV_LOGIC_CALLBACK_MEMBER(stuntcyc_state::probe_bit2_cb) { m_probe_bit2 = data; }
NETDEV_LOGIC_CALLBACK_MEMBER(stuntcyc_state::probe_bit3_cb) { m_probe_bit3 = data; }
NETDEV_LOGIC_CALLBACK_MEMBER(stuntcyc_state::probe_bit4_cb) { m_probe_bit4 = data; }
NETDEV_LOGIC_CALLBACK_MEMBER(stuntcyc_state::probe_bit5_cb) { m_probe_bit5 = data; }
NETDEV_LOGIC_CALLBACK_MEMBER(stuntcyc_state::probe_bit6_cb) { m_probe_bit6 = data; }
NETDEV_LOGIC_CALLBACK_MEMBER(stuntcyc_state::probe_clock_cb)
{
	synchronize();
	attotime second_fraction(0, time.attoseconds());
	attotime frame_fraction(0, (second_fraction * 60).attoseconds());
	attotime pixel_time = frame_fraction * (SC_HTOTAL * SC_VTOTAL);
	int32_t pixel_index = (frame_fraction * (SC_HTOTAL * SC_VTOTAL)).seconds();
	double pixel_fraction = ATTOSECONDS_TO_DOUBLE(pixel_time.attoseconds());

	const int hpos = pixel_index % SC_HTOTAL;//m_screen->hpos();
	const int vpos = pixel_index / SC_HTOTAL;//m_screen->vpos();
	const int curr_index = vpos * SC_HTOTAL + hpos;

	int last_index = m_last_vpos * SC_HTOTAL + m_last_hpos;
	if (last_index != curr_index)
	{
		m_probe_data[last_index] = int(double(m_probe_data[last_index]) * m_last_fraction);
		m_probe_data[last_index] += int(double(m_last_beam) * (1.0 - m_last_fraction));
		last_index++;
		while (last_index <= curr_index)
			m_probe_data[last_index++] = m_last_beam;
	}

	//m_last_beam = float(data);
	m_last_beam = m_probe_bit0 + m_probe_bit1 * 2 + m_probe_bit2 * 4 + m_probe_bit3 * 8 + m_probe_bit4 * 16 + m_probe_bit5 * 32 + m_probe_bit6 * 64;
	m_last_hpos = hpos;
	m_last_vpos = vpos;
	m_last_fraction = pixel_fraction;
}

uint32_t stuntcyc_state::screen_update_stuntcyc(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
	m_last_hpos = 0;
	m_last_vpos = 0;

	uint32_t pixindex = 0;
	for (int y = 0; y < SC_VTOTAL; y++)
	{
		uint32_t *scanline = &bitmap.pix32(y);
		pixindex = y * SC_HTOTAL;
		for (int x = 0; x < SC_HTOTAL; x++)
			*scanline++ = 0xff000000 | (m_probe_data[pixindex++] * 0x010101);
			//*scanline++ = 0xff000000 | (uint8_t(m_screen_buf[pixindex++] * 63.0) * 0x010101);
	}

	return 0;
}

static MACHINE_CONFIG_START( atarikee )
	/* basic machine hardware */
	MCFG_DEVICE_ADD("maincpu", NETLIST_CPU, NETLIST_CLOCK)
	MCFG_NETLIST_SETUP(atarikee)

	/* video hardware */
	MCFG_FIXFREQ_ADD("fixfreq", "screen")
	MCFG_FIXFREQ_MONITOR_CLOCK(MASTER_CLOCK)
	MCFG_FIXFREQ_HORZ_PARAMS(H_TOTAL-67,H_TOTAL-40,H_TOTAL-8,H_TOTAL)
	MCFG_FIXFREQ_VERT_PARAMS(V_TOTAL-22,V_TOTAL-19,V_TOTAL-12,V_TOTAL)
	MCFG_FIXFREQ_FIELDCOUNT(1)
	MCFG_FIXFREQ_SYNC_THRESHOLD(0.30)
MACHINE_CONFIG_END

//#define STUNTCYC_NL_CLOCK (14318181*69)
#define STUNTCYC_NL_CLOCK (SC_HTOTAL*SC_VTOTAL*60*140)

static MACHINE_CONFIG_START( stuntcyc )
	/* basic machine hardware */
	MCFG_DEVICE_ADD("maincpu", NETLIST_CPU, STUNTCYC_NL_CLOCK)
	MCFG_NETLIST_SETUP(stuntcyc)

	//MCFG_NETLIST_ANALOG_OUTPUT("maincpu", "vid0", "VIDEO_OUT", fixedfreq_device, update_vid, "fixfreq")
	MCFG_NETLIST_LOGIC_OUTPUT("maincpu", "probe_bit0",  "probe_bit0",  stuntcyc_state, probe_bit0_cb, "")
	MCFG_NETLIST_LOGIC_OUTPUT("maincpu", "probe_bit1",  "probe_bit1",  stuntcyc_state, probe_bit1_cb, "")
	MCFG_NETLIST_LOGIC_OUTPUT("maincpu", "probe_bit2",  "probe_bit2",  stuntcyc_state, probe_bit2_cb, "")
	MCFG_NETLIST_LOGIC_OUTPUT("maincpu", "probe_bit3",  "probe_bit3",  stuntcyc_state, probe_bit3_cb, "")
	MCFG_NETLIST_LOGIC_OUTPUT("maincpu", "probe_bit4",  "probe_bit4",  stuntcyc_state, probe_bit4_cb, "")
	MCFG_NETLIST_LOGIC_OUTPUT("maincpu", "probe_bit5",  "probe_bit5",  stuntcyc_state, probe_bit5_cb, "")
	MCFG_NETLIST_LOGIC_OUTPUT("maincpu", "probe_bit6",  "probe_bit6",  stuntcyc_state, probe_bit6_cb, "")
	MCFG_NETLIST_LOGIC_OUTPUT("maincpu", "probe_clock", "probe_clock", stuntcyc_state, probe_clock_cb, "")

/* video hardware */
	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_UPDATE_DRIVER(stuntcyc_state, screen_update_stuntcyc)
	MCFG_SCREEN_RAW_PARAMS(SC_HTOTAL*SC_VTOTAL*60, SC_HTOTAL, 0, SC_HTOTAL, SC_VTOTAL, 0, SC_VTOTAL)
	//MCFG_FIXFREQ_ADD("fixfreq", "screen")
	//MCFG_FIXFREQ_MONITOR_CLOCK(SC_VIDCLOCK)
	//MCFG_FIXFREQ_HORZ_PARAMS(SC_HTOTAL-67,SC_HTOTAL-40,SC_HTOTAL-8, SC_HTOTAL)
	//MCFG_FIXFREQ_VERT_PARAMS(SC_VTOTAL-22,SC_VTOTAL-19,SC_VTOTAL-12,SC_VTOTAL)
	//MCFG_FIXFREQ_FIELDCOUNT(1)
	//MCFG_FIXFREQ_SYNC_THRESHOLD(0.30)
MACHINE_CONFIG_END


/***************************************************************************

  Game driver(s)

***************************************************************************/


ROM_START( antiairc )
	ROM_REGION( 0x10000, "maincpu", ROMREGION_ERASE00 )

	ROM_REGION( 0x20, "gfx", ROMREGION_ERASE00 )
	ROM_LOAD( "003127.k1",     0x0000, 0x0020, CRC(9de772d5) SHA1(2855ba908d8e14a5aca43d4e0594d19f23fe9aae) ) // Anti-Aircraft Target
ROM_END


ROM_START( crashnsc )
	ROM_REGION( 0x10000, "maincpu", ROMREGION_ERASE00 )

	ROM_REGION( 0x0400, "gfx", ROMREGION_ERASE00 )
	ROM_LOAD( "003186.f6",     0x0000, 0x0200, CRC(b3443354) SHA1(f43b82fd5d02dad2f597f890f5845701e73476a5) ) // Car Video #1
	ROM_LOAD( "003186.p6",     0x0200, 0x0200, CRC(b3443354) SHA1(f43b82fd5d02dad2f597f890f5845701e73476a5) ) // Car Video #2

	ROM_REGION( 0x0040, "motion", ROMREGION_ERASE00 )
	ROM_LOAD( "003187.f7",     0x0000, 0x0020, CRC(01dca5b9) SHA1(0e3fbefc5df993b5a6a724aee258653897954255) ) // Car Motion #1
	ROM_LOAD( "003187.p7",     0x0020, 0x0020, CRC(01dca5b9) SHA1(0e3fbefc5df993b5a6a724aee258653897954255) ) // Car Motion #2

	ROM_REGION( 0x0200, "location", ROMREGION_ERASE00 )
	ROM_LOAD( "004248.d2",     0x0000, 0x0200, CRC(683b203b) SHA1(97202da5dd4a6cb66714d8e58ecee5c6efa65c1c) ) // Car Location Code

	ROM_REGION( 0x0200, "shape", ROMREGION_ERASE00 )
	ROM_LOAD( "004247.e2",     0x0000, 0x0200, CRC(478afac2) SHA1(fb15af0d2fc9d9ed0e92a3e7610c22dadf91d012) ) // Car Shape Code
ROM_END


ROM_START( indy4 )
	ROM_REGION( 0x10000, "maincpu", ROMREGION_ERASE00 )

	ROM_REGION( 0x0200, "gfx", ROMREGION_ERASE00 )
	ROM_LOAD( "003186.p6",     0x0000, 0x0200, CRC(b3443354) SHA1(f43b82fd5d02dad2f597f890f5845701e73476a5) ) // Car Video

	ROM_REGION( 0x0020, "motion", ROMREGION_ERASE00 )
	ROM_LOAD( "003187.f7",     0x0000, 0x0020, CRC(01dca5b9) SHA1(0e3fbefc5df993b5a6a724aee258653897954255) ) // Car Motion

	ROM_REGION( 0x0020, "checkpoint", ROMREGION_ERASE00 )
	ROM_LOAD( "005502.e5",     0x0000, 0x0020, CRC(e30ea877) SHA1(86f1f2c2e6e8472f7019f17bac723cb36faf098a) ) // Check Points

	ROM_REGION( 0x0200, "racetrack", ROMREGION_ERASE00 )
	ROM_LOAD( "005503.f4",     0x0000, 0x0200, CRC(1aafbe72) SHA1(c59829eccfe5a6014acad9682c401ca3f32fdfc9) ) // Race Track
ROM_END


ROM_START( indy800 )
	ROM_REGION( 0x10000, "maincpu", ROMREGION_ERASE00 )

	ROM_REGION( 0x0200, "gfx", ROMREGION_ERASE00 )
	ROM_LOAD( "003186.p6",     0x0000, 0x0200, CRC(b3443354) SHA1(f43b82fd5d02dad2f597f890f5845701e73476a5) ) // Car Video

	ROM_REGION( 0x0020, "motion", ROMREGION_ERASE00 )
	ROM_LOAD( "003187.f7",     0x0000, 0x0020, CRC(01dca5b9) SHA1(0e3fbefc5df993b5a6a724aee258653897954255) ) // Car Motion

	ROM_REGION( 0x0020, "checkpoint", ROMREGION_ERASE00 )
	ROM_LOAD( "003188.e5",     0x0000, 0x0020, NO_DUMP ) // Check Points - Might be same as indy4?

	ROM_REGION( 0x0200, "racetrack", ROMREGION_ERASE00 )
	ROM_LOAD( "003189.f4",     0x0000, 0x0200, NO_DUMP ) // Race Track - Might be same as indy4?
ROM_END


ROM_START( jetfight )
	ROM_REGION( 0x10000, "maincpu", ROMREGION_ERASE00 )

	ROM_REGION( 0x0040, "shell", ROMREGION_ERASE00 )
	ROM_LOAD( "004250.m1",     0x0000, 0x0020, CRC(bee62d20) SHA1(2ea5fd7b087004c37901d2a56da2d6f6dcce9e29) ) // Shell Rom
	ROM_LOAD( "004250.j1",     0x0020, 0x0020, CRC(bee62d20) SHA1(2ea5fd7b087004c37901d2a56da2d6f6dcce9e29) ) // Shell Rom

	ROM_REGION( 0x0020, "singleplayer", ROMREGION_ERASE00 )
	ROM_LOAD( "004251.r5",     0x0000, 0x0020, CRC(bd95f87e) SHA1(4bd863104f1a7260b95f3fb2c13f40b7337d3dd9) ) // Single Player Rom

	ROM_REGION( 0x0100, "score", ROMREGION_ERASE00 )
	ROM_LOAD( "004252.a4",     0x0000, 0x0100, CRC(08a0b011) SHA1(71998728604a152006550869afe60d405643ccf1) ) // Score Rom

	ROM_REGION( 0x0400, "gfx", ROMREGION_ERASE00 )
	/* Note:  Use 004253-01 and 004253-02 or use 004253-03 ONLY, not both together.  Presumably, -03 = data from -02 and -01 */
	ROM_LOAD( "004253-02.j5",  0x0000, 0x0200, CRC(c58ee65d) SHA1(785f842897a2ce92ce2f009e9b6d8e96950deb1f) ) // Picture & S.C. Rom A
	ROM_LOAD( "004253-01.k5",  0x0200, 0x0200, CRC(0d5648a9) SHA1(7a79ca587376678d9735f025d59088e6686fd783) ) // Picture & S.C. Rom B
//  ROM_LOAD( "004253-03.f5",  0x0000, 0x0400, NO_DUMP ) // Picture & S.C. Rom C
ROM_END


ROM_START( jetfighta )
	ROM_REGION( 0x10000, "maincpu", ROMREGION_ERASE00 )

	ROM_REGION( 0x0040, "shell", ROMREGION_ERASE00 )
	ROM_LOAD( "004250.m1",     0x0000, 0x0020, CRC(bee62d20) SHA1(2ea5fd7b087004c37901d2a56da2d6f6dcce9e29) ) // Shell Rom
	ROM_LOAD( "004250.j1",     0x0020, 0x0020, CRC(bee62d20) SHA1(2ea5fd7b087004c37901d2a56da2d6f6dcce9e29) ) // Shell Rom

	ROM_REGION( 0x0020, "singleplayer", ROMREGION_ERASE00 )
	ROM_LOAD( "004251.r5",     0x0000, 0x0020, CRC(bd95f87e) SHA1(4bd863104f1a7260b95f3fb2c13f40b7337d3dd9) ) // Single Player Rom

	ROM_REGION( 0x0200, "score", ROMREGION_ERASE00 )
	ROM_LOAD( "jet.a4",        0x0000, 0x0200, CRC(9e267e44) SHA1(b1c74ab275e30ed41c60e8490eaaf5211ec14ec5) ) // Score Rom

	ROM_REGION( 0x0800, "gfx", ROMREGION_ERASE00 )
	ROM_LOAD( "jet.j5",        0x0000, 0x0400, CRC(853d61b3) SHA1(c5e1b09153b813b7b4042246e5634cc83de9654c) ) // Picture & S.C. Rom A
	ROM_LOAD( "jet.k5",        0x0400, 0x0400, CRC(a3fada62) SHA1(2efed600683e35ffa10acc5a301e736989c9f236) ) // Picture & S.C. Rom B
ROM_END


ROM_START( outlaw )
	ROM_REGION( 0x10000, "maincpu", ROMREGION_ERASE00 )

	ROM_REGION( 0x0200, "gfx", ROMREGION_ERASE00 )
	ROM_LOAD( "0003323.j4",  0x0000, 0x0200, CRC(3166dad9) SHA1(4fca88b4256d8fb3e0deca54a15ffaafb830831e) ) // Rom (8205)
ROM_END


ROM_START( sharkjaw )
	ROM_REGION( 0x10000, "maincpu", ROMREGION_ERASE00 )

	ROM_REGION( 0x0200, "gfx", ROMREGION_ERASE00 )
	ROM_LOAD( "004182.da1",  0x0000, 0x0100, CRC(05242912) SHA1(d3925cde795f04ac04151165bbbff74b15dce5ca) ) // Shark & Fish P-Rom
	ROM_LOAD( "004183.db1",  0x0100, 0x0100, CRC(b161b889) SHA1(009c6fc93174df15fb6a7993a73cfda56c8edfa2) )// Diver P-Rom
ROM_END


ROM_START( steeplec )
	ROM_REGION( 0x10000, "maincpu", ROMREGION_ERASE00 )

	ROM_REGION( 0x0220, "gfx", ROMREGION_ERASE00 )
	ROM_LOAD( "003773-a.4c",  0x0000, 0x0100, CRC(5ddc49b6) SHA1(58eba996703cbb7b3f66ff97357e191c9a3ab340) ) // Horse Graphics
	ROM_LOAD( "003773-b.4d",  0x0100, 0x0100, CRC(e6994cde) SHA1(504f92dba0c8640d55c7412697868582043f3817) ) // Horse Graphics
	ROM_LOAD( "003774.8c",  0x0200, 0x0020, CRC(f3785f4a) SHA1(98f4015049279de5ba109e6dd87bb94071df5860) ) // Bugle
ROM_END


ROM_START( stuntcyc )
	ROM_REGION( 0x10000, "maincpu", ROMREGION_ERASE00 )

	ROM_REGION( 0x0200, "maincpu:004275.f1", ROMREGION_ERASE00 )
	ROM_LOAD( "004275.f1",  0x0000, 0x0200, CRC(4ed5a99d) SHA1(1e5f439bce72e78dfff76fd8f61187c6ef484a64) ) // Motorcycle & Bus

	ROM_REGION( 0x0020, "maincpu:004811.d7", ROMREGION_ERASE00 )
	ROM_LOAD( "004811.d7",  0x0000, 0x0020, CRC(31a09efb) SHA1(fd5d538c9ec1234acf7c74ca0704113d220abbf6) ) // Score Translator
ROM_END


ROM_START( tank )
	ROM_REGION( 0x10000, "maincpu", ROMREGION_ERASE00 )

	/* The "custom" 24-pin ROM used in Atari/Kee Games "Tank" is known as a MOSTEK MK28000P. */
	ROM_REGION( 0x0801, "gfx", ROMREGION_ERASE00 ) // 2049 Byte Size?
	ROM_LOAD( "90-2006.k10" ,0x0000, 0x0801, CRC(c25f6014) SHA1(7bd3fca5f64c928a645ca27c643b736667cef216) )
ROM_END

ROM_START( tankii )
	ROM_REGION( 0x10000, "maincpu", ROMREGION_ERASE00 )

	/* The "custom" 24-pin ROM used in Atari/Kee Games "Tank" is known as a MOSTEK MK28000P. */
	ROM_REGION( 0x0801, "gfx", ROMREGION_ERASE00 ) // 2049 Byte Size?
	ROM_LOAD( "90-2006.k10" ,0x0000, 0x0801, CRC(c25f6014) SHA1(7bd3fca5f64c928a645ca27c643b736667cef216) )
ROM_END

/*  // NO DUMPED ROMS

ROM_START( astrotrf )
    ROM_REGION( 0x10000, "maincpu", ROMREGION_ERASE00 )

    ROM_REGION( 0x0400, "gfx", ROMREGION_ERASE00 ) // Region Size unknown, dump size unknown
    ROM_LOAD( "003774.c8",     0x0000, 0x0100, NO_DUMP ) // Bugle
    ROM_LOAD( "003773-02.c4",  0x0100, 0x0100, NO_DUMP ) // Graphics (Astroturf - Rev.A)
ROM_END

ROM_START( gtrak10 )  // Unknown size, assumed 2K Bytes
    ROM_REGION( 0x10000, "maincpu", ROMREGION_ERASE00 )

    ROM_REGION( 0x0800, "racetrack", ROMREGION_ERASE00 )
    ROM_LOAD( "74186.k5",     0x0000, 0x0800, NO_DUMP) // Racetrack, not actually a SN74186 but an Electronic Arrays, Inc. EA4800 16K (2048 x 8) ROM. TI TMS4800 clone (EA4800). Intentionally mislabeled by Atari.
ROM_END

ROM_START( gtrak20 )  // Unknown size, assumed 2K Bytes
    ROM_REGION( 0x10000, "maincpu", ROMREGION_ERASE00 )

    ROM_REGION( 0x0800, "racetrack", ROMREGION_ERASE00 )
    ROM_LOAD( "74186.k5",     0x0000, 0x0800, NO_DUMP) // Racetrack, not actually a SN74186 but an Electronic Arrays, Inc. EA4800 16K (2048 x 8) ROM. TI TMS4800 clone (EA4800). Intentionally mislabeled by Atari.
ROM_END

ROM_START( lemans )
    ROM_REGION( 0x10000, "maincpu", ROMREGION_ERASE00 )

    ROM_REGION( 0x0400, "gfx", ROMREGION_ERASE00 ) // Region Size unknown, dump size unknown
    ROM_LOAD( "005837-01.n5",  0x0000, 0x0100, NO_DUMP ) // Rom 1
    ROM_LOAD( "005838-01.n4",  0x0100, 0x0100, NO_DUMP ) // Rom 2
    ROM_LOAD( "005839-01.n6",  0x0200, 0x0100, NO_DUMP ) // Rom 3
ROM_END

ROM_START( qwak )
    ROM_REGION( 0x10000, "maincpu", ROMREGION_ERASE00 )

    ROM_REGION( 0x0200, "gfx", ROMREGION_ERASE00 ) // Region Size unknown, dump size unknown
    ROM_LOAD( "37-2530n.k9",  0x0000, 0x0200, NO_DUMP ) // Custom Rom (2530 N)
ROM_END

*/


/*  // 100% TTL - NO ROMS

// Crossfire (1975)
// Unclear if this is 100% TTL or if it uses a ROM:
// IC description in manual says a rom is used (74186 ROM)
// but the parts list in the same manual mentions no IC 74186!
// Simulated in DICE without ROMs from schematics, so unlikely
// it uses any, and is in fact 100% TTL..
ROM_START ( crossfir )
    ROM_REGION( 0x10000, "maincpu", ROMREGION_ERASE00 )
ROM_END

ROM_START( eliminat )
    ROM_REGION( 0x10000, "maincpu", ROMREGION_ERASE00 )
ROM_END

ROM_START( goaliv )
    ROM_REGION( 0x10000, "maincpu", ROMREGION_ERASE00 )
ROM_END

ROM_START( gotchaat )
    ROM_REGION( 0x10000, "maincpu", ROMREGION_ERASE00 )
ROM_END

ROM_START( gotchaatc )
    ROM_REGION( 0x10000, "maincpu", ROMREGION_ERASE00 )
ROM_END

ROM_START( hiway )
    ROM_REGION( 0x10000, "maincpu", ROMREGION_ERASE00 )
ROM_END

ROM_START( pinpong )
    ROM_REGION( 0x10000, "maincpu", ROMREGION_ERASE00 )
ROM_END

ROM_START( pongdbl )
    ROM_REGION( 0x10000, "maincpu", ROMREGION_ERASE00 )
ROM_END

ROM_START( pursuit )
    ROM_REGION( 0x10000, "maincpu", ROMREGION_ERASE00 )
ROM_END

ROM_START( quadpong )
    ROM_REGION( 0x10000, "maincpu", ROMREGION_ERASE00 )
ROM_END

ROM_START( rebound )
    ROM_REGION( 0x10000, "maincpu", ROMREGION_ERASE00 )
ROM_END

ROM_START( spacrace )
    ROM_REGION( 0x10000, "maincpu", ROMREGION_ERASE00 )
ROM_END

ROM_START( touchme )
    ROM_REGION( 0x10000, "maincpu", ROMREGION_ERASE00 )
ROM_END

ROM_START( worldcup )
    ROM_REGION( 0x10000, "maincpu", ROMREGION_ERASE00 )
ROM_END

ROM_START( coupdmnd ) // dummy to satisfy game entry
    ROM_REGION( 0x10000, "maincpu", ROMREGION_ERASE00 )
ROM_END

ROM_START( coupfran ) // dummy to satisfy game entry
    ROM_REGION( 0x10000, "maincpu", ROMREGION_ERASE00 )
ROM_END

*/

GAME(1975,  antiairc,  0,         atarikee,   0,  atarikee_state, 0,  ROT0,  "Atari",        "Anti-Aircraft [TTL]",    MACHINE_IS_SKELETON)
GAME(1975,  crashnsc,  0,         atarikee,   0,  atarikee_state, 0,  ROT0,  "Atari",        "Crash 'n Score/Stock Car [TTL]",   MACHINE_IS_SKELETON)
GAME(1976,  indy4,     0,         atarikee,   0,  atarikee_state, 0,  ROT0,  "Atari/Kee",    "Indy 4 [TTL]",           MACHINE_IS_SKELETON)
GAME(1975,  indy800,   0,         atarikee,   0,  atarikee_state, 0,  ROT90, "Atari/Kee",    "Indy 800 [TTL]",         MACHINE_IS_SKELETON)
GAME(1975,  jetfight,  0,         atarikee,   0,  atarikee_state, 0,  ROT0,  "Atari",        "Jet Fighter/Jet Fighter Cocktail/Launch Aircraft (set 1) [TTL]",      MACHINE_IS_SKELETON)
GAME(1975,  jetfighta, jetfight,  atarikee,   0,  atarikee_state, 0,  ROT0,  "Atari",        "Jet Fighter/Jet Fighter Cocktail/Launch Aircraft (set 2) [TTL]",      MACHINE_IS_SKELETON)
GAME(1976,  outlaw,    0,         atarikee,   0,  atarikee_state, 0,  ROT0,  "Atari",        "Outlaw [TTL]",           MACHINE_IS_SKELETON)
GAME(1975,  sharkjaw,  0,         atarikee,   0,  atarikee_state, 0,  ROT0,  "Atari/Horror Games",    "Shark JAWS [TTL]",     MACHINE_IS_SKELETON)
GAME(1975,  steeplec,  0,         atarikee,   0,  atarikee_state, 0,  ROT0,  "Atari",        "Steeplechase [TTL]",     MACHINE_IS_SKELETON)
GAME(1976,  stuntcyc,  0,         stuntcyc,   0,  stuntcyc_state, 0,  ROT0,  "Atari",        "Stunt Cycle [TTL]",      MACHINE_IS_SKELETON)
GAME(1974,  tank,      0,         atarikee,   0,  atarikee_state, 0,  ROT0,  "Atari/Kee",    "Tank/Tank Cocktail [TTL]",     MACHINE_IS_SKELETON)
GAME(1975,  tankii,    0,         atarikee,   0,  atarikee_state, 0,  ROT0,  "Atari/Kee",    "Tank II [TTL]",          MACHINE_IS_SKELETON)

// MISSING ROM DUMPS
//GAME(1975,  astrotrf,  steeplec,  atarikee,   0,  atarikee_state, 0,  ROT0,  "Atari",        "Astroturf [TTL]",        MACHINE_IS_SKELETON)
//GAME(1974,  gtrak10,   0,         atarikee,   0,  atarikee_state, 0,  ROT0,  "Atari/Kee",    "Gran Trak 10/Trak 10/Formula K [TTL]",     MACHINE_IS_SKELETON) //?
//GAME(1974,  gtrak20,   0,         atarikee,   0,  atarikee_state, 0,  ROT0,  "Atari/Kee",    "Gran Trak 20/Trak 20/Twin Racer [TTL]",    MACHINE_IS_SKELETON) //?
//GAME(1976,  lemans,    0,         atarikee,   0,  atarikee_state, 0,  ROT0,  "Atari",        "Le Mans [TTL]",          MACHINE_IS_SKELETON)
//GAME(1974,  qwak,      0,         atarikee,   0,  atarikee_state, 0,  ROT0,  "Atari",        "Qwak!/Quack [TTL]",      MACHINE_IS_SKELETON)

// 100% TTL
//GAME(1974,  coupfran,  worldcup,  atarikee,   0,  atarikee_state, 0,  ROT0,  "Atari Europe", "Coup Franc [TTL]",       MACHINE_IS_SKELETON)
//GAME(1974,  coupdmnd,  worldcup,  atarikee,   0,  atarikee_state, 0,  ROT0,  "Atari France", "Coup du Monde [TTL]",    MACHINE_IS_SKELETON)
//GAME(1975,  crossfir,  0,         atarikee,   0,  atarikee_state, 0,  ROT0,  "Atari/Kee",    "Crossfire [TTL]",        MACHINE_IS_SKELETON)
//GAME(1973,  eliminat,  0,         atarikee,   0,  atarikee_state, 0,  ROT0,  "Atari/Kee",    "Elimination! [TTL]",     MACHINE_IS_SKELETON)
//GAME(1975,  goaliv,    0,         atarikee,   0,  atarikee_state, 0,  ROT0,  "Atari",        "Goal IV [TTL]",          MACHINE_IS_SKELETON)
//GAME(1973,  gotchaat,  0,         atarikee,   0,  atarikee_state, 0,  ROT0,  "Atari",        "Gotcha [TTL]",           MACHINE_IS_SKELETON) //?
//GAME(1973,  gotchaatc, 0,         atarikee,   0,  atarikee_state, 0,  ROT0,  "Atari",        "Gotcha Color [TTL]",     MACHINE_IS_SKELETON) //?
//GAME(1975,  hiway,     0,         atarikee,   0,  atarikee_state, 0,  ROT0,  "Atari",        "Hi-Way/Highway [TTL]",   MACHINE_IS_SKELETON)
//GAME(1974,  pinpong,   0,         atarikee,   0,  atarikee_state, 0,  ROT0,  "Atari",        "Pin Pong [TTL]",         MACHINE_IS_SKELETON)
//GAME(1975,  pursuit,   0,         atarikee,   0,  atarikee_state, 0,  ROT0,  "Atari",        "Pursuit [TTL]",          MACHINE_IS_SKELETON)
//GAME(1974,  quadpong,  eliminat,  atarikee,   0,  atarikee_state, 0,  ROT0,  "Atari",        "Quadrapong [TTL]",       MACHINE_IS_SKELETON)
//GAME(1974,  rebound,   0,         atarikee,   0,  atarikee_state, 0,  ROT0,  "Atari/Kee",    "Rebound/Spike/Volleyball [TTL]",   MACHINE_IS_SKELETON)
//GAME(1973,  spacrace,  0,         atarikee,   0,  atarikee_state, 0,  ROT0,  "Atari",        "Space Race [TTL]",       MACHINE_IS_SKELETON)
//GAME(1974,  touchme,   0,         atarikee,   0,  atarikee_state, 0,  ROT0,  "Atari",        "Touch-Me [TTL]",         MACHINE_IS_SKELETON) //?
//GAME(1974,  worldcup,  0,         atarikee,   0,  atarikee_state, 0,  ROT0,  "Atari",        "World Cup/World Cup Football [TTL]",   MACHINE_IS_SKELETON)
