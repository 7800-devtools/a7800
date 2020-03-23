// license:BSD-3-Clause
// copyright-holders:Roberto Fresca
/******************************************************************************

  Wild Poker
  TAB Austria.

  Preliminary driver by Roberto Fresca.


  Games running in this hardware:

  * Wild Poker (ver. D 1.01),         199?, TAB Austria.


  The HD63484 ACRTC support is incomplete,
  due to the preliminary emulation state.

*******************************************************************************

  Hardware Notes:
  ---------------

  CPU:
  - 1x MC68000P12        ; 68000 CPU @ 12 MHz, from Motorola.
  - 1x D8751H            ; 8751 MCU (3.6864 MHz?)

  Sound device:
  - 1x AY8930            ; Sound IC, from Microchip Technology.

  Video:
  - 1x HD63484CP8 @ 8MHz ; Advanced CRT Controller (ACRTC), from Hitachi Semiconductor.
  - 1x HD63485CP64       ; Hitachi - Graphic Memory Interface Controller (GMIC).
  - 2x HD63486CP32       ; Hitachi - Graphic Video Attribute Controller (GVAC).

  Other:
  - 1x MC68681           ; Motorola - Dual Asynchronous Receiver/Transmitter.
  - 4x XTALs....         ; 3.6864 / 12.000 / 26.000 / 24.000 MHz.

                                                                                          .--------.
  PCB Layout:                                                                           --+--------+--
  .---------------------------------------------------------------------------------------+        +-----------------------------------------------------.
  |                                                                                       |  DB9   |                                                     |
  |                        .--------.                    .--------.                       |        |                             .--------.              |
  |                        |::::::::|                    |::::::::|                       '--------'                             |74HCT32P|              |
  |                        '--------'                    '--------'                                                              '--------'              |
  |                        .--------.                    .---------. .----------.         .--------. .----------.  .----------.  .--------.              |
  |                        |LT1084CN|                    |SN75116N | | MM57410N |         |74HCT14P| |74HCT245P |  |74HCT245P |  |74HCT86P| .-------.    |
  |                        '--------'                    '---------' '----------'         '--------' '----------'  '----------'  '--------' |XTAL 3 |    |
  |                                                                                                                  .--------.   .-------. |       |    |
  |                                         .--------.    .---------.        .-..-.                                  |        |   |DM74S04| '-------'    |
  |                                         |  PC617 |    |74HCT14P |        | || |               .----------------. '--------'   '-------'              |
  |                                         '--------'    '---------'        '-''-'               |  inmos 8941-C  |                                     |
  |.--. .---------.  .---------.    .-----------. .--------.     .---------------.                |  IMS G176P-50  |           .----------. .----------. |
  ||..| |ULN2803A |  |74HCT533P|    |PC74HC245P | |74HCT125|     |    HYUNDAI    |                |                |           |HY53C464LS| |HY53C464LS| |
  ||..| '---------'  '---------'    '-----------' '--------'     |  HY6264LP-10  |                '----------------'           '----------' '----------' |
  ||..|                           .------------------------.     |  9040D KOREA  |  .-------------.        .-------------.     .----------. .----------. |
  ||..| .---------.  .---------.  |        AY8930 /P       |     '---------------'  |             |        |             |     |HY53C464LS| |HY53C464LS| |
  ||..| |ULN2803A |  |74HCT533P|  |        9019CCA         |                        |             |        |             |     '----------' '----------' |
  ||..| '---------'  '---------'  |        TAIWAN          |     .---------------.  |    IE1 U    |        |    9117     |     .----------. .----------. |
  ||..|                           '------------------------'     |    HYUNDAI    |  | HD63484CP8  |        | HD63486CP32 |     |HY53C464LS| |HY53C464LS| |
  ||..| .---------.  .---------.                                 |  HY6264LP-10  |  |             |        |             |     '----------' '----------' |
  ||..| |ULN2803A |  |74HCT533P|       .--------. .--------.     |  9040D KOREA  |  |        Japan|        |        Japan|     .----------. .----------. |
  ||..| '---------'  '---------'  .---.|8      1| |8      1|     '---------------'  |             |        |             |     |HY53C464LS| |HY53C464LS| |
  |'--'                           |   ||  DSW1  | |  DSW2  |                        |             |        |             |     '----------' '----------' |
  |     .---------.  .---------.  '---''--------' '--------'                        '-------------'        '-------------'                               |
  |.--. |ULN2803A |  |74HCT533P|                                                                                                                         |
  ||..| '---------'  '---------'  .------------------------.                                                                   .----------. .----------. |
  ||..|                           |D8751H                  |                                                                   |HY53C464LS| |HY53C464LS| |
  ||..|                           |L0381103                |                        .-------------.        .-------------.     '----------' '----------' |
  ||..|  .--------.  .---------.  |          VD1.00        |  .------------------.  |             |        |             |     .----------. .----------. |
  ||..|  |MDP1603 |  |74HCT245P|  '------------------------'  |D27C020           |  |             |        |             |     |HY53C464LS| |HY53C464LS| |
  |'--'  '--------'  '---------'  .-------.     .-------.     |                  |  |     9109    |        |    9117     |     '----------' '----------' |
  |                               |XTAL 1 |     |XTAL 2 |     |   VD / 1.01 / 3  |  | HD63485CP64 |        | HD63486CP32 |     .----------. .----------. |
  |.--.  .--------.  .---------.  |       |     |       |     '------------------'  |             |        |             |     |HY53C464LS| |HY53C464LS| |
  ||..|  |MDP1603 |  |74HCT245P|  '-------'     '-------'                           |        Japan|        |        Japan|     '----------' '----------' |
  ||..|  '--------'  '---------'  .------------------------.                        |             |        |             |     .----------. .----------. |
  ||..|                           |        MC68681P        |                        |             |        |             |     |HY53C464LS| |HY53C464LS| |
  ||..|  .--------.  .---------.  |         2C98R          |                        '-------------'        '-------------'     '----------' '----------' |
  ||..|  |MDP1603 |  |74HCT245P|  |        QQPQ9051        |                                                                                             |
  ||..|  '--------'  '---------'  '------------------------'  .------------------.                                             .--------.   .--------.   |
  ||..|                             .--------.    .--------.  |D27C020           |                                             |        |   |        |   |
  ||..|  .--------.  .---------.    |8      1|    |74HCT147|  |                  |                                             '--------'   '--------'   |
  ||..|  |MDP1603 |  |74HCT245P|    |  DSW3  |    '--------'  |   VD / 1.01 / 1  |                 .--------.  .--------.                                |
  ||..|  '--------'  '---------'    '--------'                '------------------'                 |74HCT138|  |74HCT74P|                                |
  |'--'                                                                     .----------. .-------. '--------'  '--------'                           .------.
  |                             .---------------------------------------.   | GAL16V8S | |74HCT74|                                                  |      |
  |      .-------.              |                                       |   '----------' '-------' .--------.  .--------.                           |      |
  |      |       |              |             MC68000P12                |  .--------.   .------.   |74HCT138|  |74HCT21P|                           |      |
  |      |Battery|              |               2C91E                   |  |74HCT04P|   |XTAL 4|   '--------'  '--------'                           |      |
  |      |       |              |              QZUZ9102                 |  '--------'   |      |   .--------.  .--------.                           |      |
  |      |       |              |                                       |  .--------.   '------'   |74HCT138|  |74HCT161|                           |      |
  |      '-------'              '---------------------------------------'  |74HCT14P|              '--------'  '--------'                           |      |
  |                                                                        '--------'  .-------.   .--------.  .--------.                           |      |
  |               .--.  .--.                                                           |74HCT08|   |74HCT21 |  |1      8|                           |      |
  |               |TL|  |TL|                                                           '-------'   '--------'  |  DSW4  |                           |      |
  |               '--'  '--'    ========================================                                       '--------'                           '------'
  |                            | |::::::::::::::::::::::::::::::::::::| |                                                                                |
  |                            | |::::::::::::::::::::::::::::::::::::| |                                                                                |
  |                             ========================================                                                                                 |
  '------------------------------------------------------------------------------------------------------------------------------------------------------'

  XTAL 1: 3.6864 MHz.
  XTAL 2: 12.000 MHz.
  XTAL 3: 26.000 MHz.
  XTAL 4: 24.000 MHz.

  TL: TL7705ACP


      DSW1:          DSW2:          DSW3:          DSW4:
   .--------.     .--------.     .--------.     .--------.
  1| oo oooo|8   1|oooooooo|8   1|oooooooo|8   1|  o     |8    ON
   |--------|     |--------|     |--------|     |--------|
   |o  o    |     |        |     |        |     |oo ooooo|     OFF
   '--------'     '--------'     '--------'     '--------'


*******************************************************************************

  *** Game Notes ***

  Nothing yet...


*******************************************************************************

  ---------------------------------
  ***  Memory Map (preliminary) ***
  ---------------------------------

  00000 - 7FFFF  ; ROM space.


*******************************************************************************

  DRIVER UPDATES:

  [2012-06-11]

  - Initial release.
  - Pre-defined Xtals.
  - Added ASCII PCB layout.
  - Started a preliminary memory map.
  - Added technical notes.


  TODO:

  - Improve memory map.
  - ACRTC support.
  - GFX decode.
  - Sound support.
  - A lot!.


*******************************************************************************/

#include "emu.h"
#include "cpu/m68000/m68000.h"
#include "machine/clock.h"
#include "machine/ds2401.h"
#include "machine/mc68681.h"
#include "machine/nvram.h"
#include "sound/ay8910.h"
#include "sound/dac.h"
#include "sound/volt_reg.h"
#include "video/hd63484.h"
#include "video/ramdac.h"
#include "screen.h"
#include "speaker.h"


#define MAIN_CLOCK  XTAL_12MHz
#define AY_CLOCK    MAIN_CLOCK / 8
#define SEC_CLOCK   XTAL_3_6864MHz
#define AUX1_CLOCK  XTAL_26MHz
#define AUX2_CLOCK  XTAL_24MHz


class wildpkr_state : public driver_device
{
public:
	wildpkr_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_duart(*this, "duart"),
		m_id(*this, "id"),
		m_dac(*this, "dac"),
		m_dac_clock(*this, "dacclock"),
		m_nvram(*this, "nvram")
	{ }

	required_device<cpu_device> m_maincpu;
	required_device<mc68681_device> m_duart;
	optional_device<ds2401_device> m_id;
	optional_device<dac_byte_interface> m_dac;
	optional_device<clock_device> m_dac_clock;

	optional_shared_ptr<u16> m_nvram;

	u16 m_clock_rate;

	DECLARE_DRIVER_INIT(wildpkr);
	virtual void machine_start() override;
	virtual void video_start() override;
	DECLARE_PALETTE_INIT(wildpkr);
	DECLARE_READ8_MEMBER(unknown_read8);
	DECLARE_WRITE8_MEMBER(unknown_write8);
	DECLARE_WRITE16_MEMBER(nvram_w);
	DECLARE_READ16_MEMBER(id_serial_r);
	DECLARE_WRITE16_MEMBER(id_serial_w);
	DECLARE_WRITE16_MEMBER(out0_w);
	DECLARE_WRITE16_MEMBER(out1_w);
	DECLARE_WRITE8_MEMBER(dac_w);
	DECLARE_WRITE16_MEMBER(clock_start_w);
	DECLARE_WRITE16_MEMBER(clock_rate_w);
	DECLARE_WRITE16_MEMBER(unknown_trigger_w);
	IRQ_CALLBACK_MEMBER(tabpkr_irq_ack);
};


/*************************
*     Video Hardware     *
*************************/

void wildpkr_state::video_start()
{
}

PALETTE_INIT_MEMBER(wildpkr_state, wildpkr)
{
}


/*************************
*      ACRTC Access      *
*************************/


/*************************
*      Misc Handlers     *
*************************/

READ8_MEMBER(wildpkr_state::unknown_read8)
{
	return 0xff;
}

WRITE8_MEMBER(wildpkr_state::unknown_write8)
{
}

WRITE16_MEMBER(wildpkr_state::nvram_w)
{
	m_nvram[offset] = data | 0xff00;
}

READ16_MEMBER(wildpkr_state::id_serial_r)
{
	return m_id->read();
}

WRITE16_MEMBER(wildpkr_state::id_serial_w)
{
	m_id->write(data & 1);
}

WRITE16_MEMBER(wildpkr_state::out0_w)
{
}

WRITE16_MEMBER(wildpkr_state::out1_w)
{
}

WRITE8_MEMBER(wildpkr_state::dac_w)
{
	m_dac->write(space, 0, data);
}

WRITE16_MEMBER(wildpkr_state::clock_start_w)
{
	if (data != 0 && m_clock_rate != 0)
		m_dac_clock->set_clock_scale(1.0 / m_clock_rate);
	else
		m_dac_clock->set_clock_scale(0.0);
}

WRITE16_MEMBER(wildpkr_state::clock_rate_w)
{
	m_clock_rate = data;
}

WRITE16_MEMBER(wildpkr_state::unknown_trigger_w)
{
}

/*************************
*      Memory Map        *
*************************/

static ADDRESS_MAP_START( wildpkr_map, AS_PROGRAM, 16, wildpkr_state )
	AM_RANGE(0x000000, 0x0fffff) AM_ROM
	AM_RANGE(0x100000, 0x113fff) AM_RAM
	AM_RANGE(0x800000, 0x800001) AM_DEVREADWRITE("acrtc", hd63484_device, status_r, address_w)
	AM_RANGE(0x800002, 0x800003) AM_DEVREADWRITE("acrtc", hd63484_device, data_r, data_w)
	AM_RANGE(0x800080, 0x80009f) AM_DEVREADWRITE8("duart", mc68681_device, read, write, 0x00ff)
	AM_RANGE(0x800180, 0x800181) AM_READ8(unknown_read8, 0xff00)
	AM_RANGE(0x800180, 0x800181) AM_WRITE8(unknown_write8, 0x00ff)
	AM_RANGE(0x800200, 0x800201) AM_DEVWRITE8("ramdac", ramdac_device, index_w, 0xff00)
	AM_RANGE(0x800202, 0x800203) AM_DEVWRITE8("ramdac", ramdac_device, pal_w, 0xff00)
	AM_RANGE(0x800204, 0x800205) AM_DEVWRITE8("ramdac", ramdac_device, mask_w, 0xff00)
	AM_RANGE(0x800280, 0x800281) AM_DEVWRITE8("aysnd", ay8930_device, data_w, 0xff00)
	AM_RANGE(0x800282, 0x800283) AM_DEVWRITE8("aysnd", ay8930_device, address_w, 0xff00)
	AM_RANGE(0x800284, 0x800285) AM_DEVREAD8("aysnd", ay8930_device, data_r, 0x00ff) // (odd!)
	AM_RANGE(0x800286, 0x800289) AM_WRITENOP
ADDRESS_MAP_END

static ADDRESS_MAP_START( tabpkr_map, AS_PROGRAM, 16, wildpkr_state )
	AM_RANGE(0x000000, 0x2fffff) AM_ROM
	AM_RANGE(0x300000, 0x303fff) AM_RAM
	AM_RANGE(0x400000, 0x400fff) AM_RAM_WRITE(nvram_w) AM_SHARE("nvram")
	AM_RANGE(0x500000, 0x500001) AM_DEVREADWRITE("acrtc", hd63484_device, status_r, address_w)
	AM_RANGE(0x500002, 0x500003) AM_DEVREADWRITE("acrtc", hd63484_device, data_r, data_w)
	AM_RANGE(0x500020, 0x500021) AM_DEVREADWRITE8("ramdac", ramdac_device, index_r, index_w, 0x00ff)
	AM_RANGE(0x500022, 0x500023) AM_DEVREADWRITE8("ramdac", ramdac_device, pal_r, pal_w, 0x00ff)
	AM_RANGE(0x500024, 0x500025) AM_DEVREADWRITE8("ramdac", ramdac_device, mask_r, mask_w, 0x00ff)
	AM_RANGE(0x500040, 0x50005f) AM_DEVREADWRITE8("duart", mc68681_device, read, write, 0x00ff)
	AM_RANGE(0x500060, 0x500061) AM_READWRITE(id_serial_r, id_serial_w)
	AM_RANGE(0x600000, 0x600001) AM_READ_PORT("IN0") AM_WRITE(out0_w)
	AM_RANGE(0x600002, 0x600003) AM_READ_PORT("IN1") AM_WRITE(out1_w)
	AM_RANGE(0x600004, 0x600005) AM_READ_PORT("IN2")
	AM_RANGE(0x600004, 0x600005) AM_WRITE8(dac_w, 0xff00)
	AM_RANGE(0x700000, 0x700001) AM_WRITE(clock_start_w)
	AM_RANGE(0x700002, 0x700003) AM_WRITE(clock_rate_w)
	AM_RANGE(0x700004, 0x700007) AM_WRITE(unknown_trigger_w)
	AM_RANGE(0x70000a, 0x70000b) AM_WRITENOP // only writes 0 at POST
ADDRESS_MAP_END

static ADDRESS_MAP_START( hd63484_map, 0, 16, wildpkr_state )
	AM_RANGE(0x00000, 0x3ffff) AM_RAM
ADDRESS_MAP_END

/* Unknown R/W:


*/


/*************************
*      Input Ports       *
*************************/

static INPUT_PORTS_START( wildpkr )
INPUT_PORTS_END

static INPUT_PORTS_START( tabpkr )
	PORT_START("IN0")
	PORT_BIT(0x0001, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x0002, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x0004, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x0008, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x0010, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x0020, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x0040, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x0080, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x0100, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x0200, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x0400, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x0800, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x1000, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x2000, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x4000, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x8000, IP_ACTIVE_LOW, IPT_UNKNOWN)

	PORT_START("IN1")
	PORT_BIT(0x0001, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x0002, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x0004, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x0008, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x0010, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x0020, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x0040, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x0080, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x0100, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x0200, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x0400, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x0800, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x1000, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x2000, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x4000, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x8000, IP_ACTIVE_LOW, IPT_UNKNOWN)

	PORT_START("IN2")
	PORT_BIT(0x0001, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x0002, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x0004, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x0008, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x0010, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x0020, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x0040, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x0080, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x0100, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x0200, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x0400, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x0800, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x1000, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x2000, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x4000, IP_ACTIVE_LOW, IPT_UNKNOWN)
	PORT_BIT(0x8000, IP_ACTIVE_LOW, IPT_UNKNOWN)
INPUT_PORTS_END


/*************************
*     Machine Start      *
*************************/

void wildpkr_state::machine_start()
{
/*
  ACRTC memory:

  00000-3ffff = RAM
  40000-7ffff = ROM
  80000-bffff = unused
  c0000-fffff = unused
*/
	m_clock_rate = 0;
}


static ADDRESS_MAP_START( ramdac_map, 0, 8, wildpkr_state )
	AM_RANGE(0x000, 0x3ff) AM_DEVREADWRITE("ramdac",ramdac_device,ramdac_pal_r,ramdac_rgb666_w)
ADDRESS_MAP_END


IRQ_CALLBACK_MEMBER(wildpkr_state::tabpkr_irq_ack)
{
	m_maincpu->set_input_line(irqline, CLEAR_LINE);
	if (irqline == M68K_IRQ_2)
		return m_duart->get_irq_vector();
	else
		return M68K_INT_ACK_AUTOVECTOR;
}


/*************************
*    Machine Drivers     *
*************************/

static MACHINE_CONFIG_START( wildpkr )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", M68000, MAIN_CLOCK)
	MCFG_CPU_PROGRAM_MAP(wildpkr_map)
	//MCFG_CPU_VBLANK_INT_DRIVER("screen", wildpkr_state, irq2_line_hold) // guess

	MCFG_MC68681_ADD("duart", SEC_CLOCK)

	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(60)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(2500))
	MCFG_SCREEN_SIZE(384, 280)
	MCFG_SCREEN_VISIBLE_AREA(0, 384-1, 0, 280-1)
	MCFG_SCREEN_UPDATE_DEVICE("acrtc", hd63484_device, update_screen)
	MCFG_SCREEN_PALETTE("palette")

	MCFG_HD63484_ADD("acrtc", 0, hd63484_map)

	MCFG_RAMDAC_ADD("ramdac", ramdac_map, "palette")

	MCFG_PALETTE_ADD("palette", 256)
	MCFG_PALETTE_INIT_OWNER(wildpkr_state, wildpkr)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("mono")
	MCFG_SOUND_ADD("aysnd", AY8930, AY_CLOCK)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.50)
MACHINE_CONFIG_END


static MACHINE_CONFIG_START( tabpkr )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", M68000, XTAL_24MHz / 2)
	MCFG_CPU_PROGRAM_MAP(tabpkr_map)
	MCFG_CPU_PERIODIC_INT_DRIVER(wildpkr_state, irq3_line_assert, 60*256)
	MCFG_CPU_IRQ_ACKNOWLEDGE_DRIVER(wildpkr_state, tabpkr_irq_ack)

	MCFG_NVRAM_ADD_1FILL("nvram") // DS1220Y

	MCFG_MC68681_ADD("duart", 3686400)
	MCFG_MC68681_IRQ_CALLBACK(ASSERTLINE("maincpu", M68K_IRQ_2))

	MCFG_DEVICE_ADD("id", DS2401, 0)

	MCFG_DEVICE_ADD("dacclock", CLOCK, 1500000) // base rate derived from program code
	MCFG_CLOCK_SIGNAL_HANDLER(ASSERTLINE("maincpu", M68K_IRQ_5))

	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(60)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(2500))
	MCFG_SCREEN_SIZE(384, 280)
	MCFG_SCREEN_VISIBLE_AREA(0, 384-1, 0, 280-1)
	MCFG_SCREEN_UPDATE_DEVICE("acrtc", hd63484_device, update_screen)
	MCFG_SCREEN_PALETTE("palette")
	MCFG_SCREEN_VBLANK_CALLBACK(ASSERTLINE("maincpu", M68K_IRQ_4))

	MCFG_HD63484_ADD("acrtc", 0, hd63484_map)

	MCFG_RAMDAC_ADD("ramdac", ramdac_map, "palette")

	MCFG_PALETTE_ADD("palette", 256)
	MCFG_PALETTE_INIT_OWNER(wildpkr_state, wildpkr)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("mono")
	MCFG_SOUND_ADD("dac", AD557, 0)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.50)
	MCFG_DEVICE_ADD("vref", VOLTAGE_REGULATOR, 0) MCFG_VOLTAGE_REGULATOR_OUTPUT(5.0)
	MCFG_SOUND_ROUTE_EX(0, "dac", 1.0, DAC_VREF_POS_INPUT) MCFG_SOUND_ROUTE_EX(0, "dac", -1.0, DAC_VREF_NEG_INPUT)
MACHINE_CONFIG_END


/*************************
*        Rom Load        *
*************************/

ROM_START( wildpkr )
	ROM_REGION( 0x100000, "maincpu", 0 )
	ROM_LOAD16_BYTE( "vd_1.01_3.bin", 0x000000, 0x40000, CRC(d19d5609) SHA1(87eedb7daaa8ac33c0a73e4e849b9a0f76152261) )
	ROM_LOAD16_BYTE( "vd_1.01_1.bin", 0x000001, 0x40000, CRC(f10644ab) SHA1(5872fe41b8c7fec5e83011abdf82a85f064b734f) )

	ROM_REGION( 0x1000, "mcu", 0 )
	ROM_LOAD( "d8751h",  0x0000, 0x1000, NO_DUMP )

	ROM_REGION( 0x0200, "plds", 0 )
	ROM_LOAD( "gal6v8s.bin",  0x0000, 0x0117, CRC(389c63a7) SHA1(4ebb26a001ed14a9e96dd268ed1c7f298f0c086b) )
ROM_END

/* seems to be different hardware, but same basic video chips, keep here or move?

cpu 68000-16
Xtal 24Mhaz
cpu ram 2x 6264

Audio DAC AD557JN

video area
insg176p-66 ramdac?
hd63487cp  Memory interface and video attribute controller
hd63484cp8 advanced CRT controller
4x km44c258cz-10 rams

*/

ROM_START( tabpkr ) // Royal Poker V 1.85 Oct 29 1996 12:20:07
	ROM_REGION( 0x300000, "maincpu", ROMREGION_ERASEFF )
	ROM_LOAD16_BYTE( "rop1851.bin", 0x000001, 0x80000, CRC(fbe13fa8) SHA1(7c19b6b4d9a9935b6feb70b6261bafc6d9afb59f) )
	ROM_LOAD16_BYTE( "rop1853.bin", 0x000000, 0x80000, CRC(e0c312b4) SHA1(57c64c82f723067b7b2f9bf3fdaf5aedeb4f9dc3) )
	// are these missing, or just unpopulated but checked anyway?
	/* reads 0x100000 - 0x1fffff ? - 2x sockets for same type of roms as above */
	/* reads 0x200000 - 0x2fffff ? - 1x socket for larger ROM? */

	ROM_REGION(8, "id", 0)
	ROM_LOAD("ds2401.bin", 0, 8, NO_DUMP)
	// Dummy data to appease POST
	ROM_FILL(0, 1, 0x66)
	ROM_FILL(1, 1, 0xfa)
	ROM_FILL(2, 1, 0xce)
	ROM_FILL(3, 1, 0xde)
	ROM_FILL(4, 1, 0xad)
	ROM_FILL(5, 1, 0xbe)
	ROM_FILL(6, 1, 0xef)
	ROM_FILL(7, 1, 0x01)
ROM_END



/*************************
*      Driver Init       *
*************************/

DRIVER_INIT_MEMBER(wildpkr_state,wildpkr)
{
}


/*************************
*      Game Drivers      *
*************************/

//    YEAR  NAME       PARENT    MACHINE   INPUT    STATE           INIT      ROT   COMPANY        FULLNAME                    FLAGS
GAME( 199?, wildpkr,   0,        wildpkr,  wildpkr, wildpkr_state,  wildpkr,  ROT0, "TAB Austria", "Wild Poker (ver. D 1.01)", MACHINE_NO_SOUND | MACHINE_NOT_WORKING | MACHINE_UNEMULATED_PROTECTION )
GAME( 1996, tabpkr,    0,        tabpkr,   tabpkr,  wildpkr_state,  wildpkr,  ROT0, "TAB Austria", "Royal Poker V 1.85",       MACHINE_NO_SOUND | MACHINE_NOT_WORKING | MACHINE_UNEMULATED_PROTECTION )
