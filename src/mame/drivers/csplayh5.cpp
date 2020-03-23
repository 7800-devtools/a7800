// license:LGPL-2.1+
// copyright-holders:Angelo Salese
/***********************************************************************************************************

    'High Rate DVD' HW (c) 1998 Nichibutsu

    preliminary driver by Angelo Salese

    TODO:
    - Implement DVD routing and YUV decoding;
    - game timings seem busted, could be due of missing DVD hook-up
    - csplayh1: inputs doesn't work at all, slower than the others too.
      Probably not a DVD but CD rom game?

    DVD Notes:
    - TMP68301 communicates with h8 via their respective internal serial comms
    - First command is a "?P<CR>", which, according to the Pioneer V5000 protocol manual
      is an Active Mode request. Manual is at:
      http://www.pioneerelectronics.com/ephox/StaticFiles/Manuals/Business/Pio%20V5000-RS232%20-%20CPM.pdf
      After returning a correct status code, tmp68301 sends "FSDVD04.MPG00001<CR>" to serial, probably tries
      to playback the file ...
    - h8 board components:
      H8/3002
      MN7100 8-bit channel data acquisition system
      Fujitsu MD0208
      Heatsinked chip (TBD)
      IDE and RS232c ports
      xtal 27 MHz

***********************************************************************************************************/

#include "emu.h"
#include "cpu/h8/h83002.h"
#include "cpu/m68000/m68000.h"
#include "machine/nvram.h"
#include "machine/tmp68301.h"
#include "machine/idectrl.h"
#include "machine/idehd.h"
#include "video/v9938.h"
#include "audio/nichisnd.h"

#define USE_H8 0
#define DVD_CLOCK XTAL_27MHz

class csplayh5_state : public driver_device
{
public:
	csplayh5_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_tmp68301(*this, "tmp68301"),
		m_v9958(*this, "v9958"),
		m_nichisnd(*this, "nichisnd"),
		m_key(*this, "KEY.%u", 0),
		m_region_maincpu(*this, "maincpu")
	{ }

	required_device<cpu_device> m_maincpu;
	required_device<tmp68301_device> m_tmp68301;
	required_device<v9958_device> m_v9958;
	required_device<nichisnd_device> m_nichisnd;
	required_ioport_array<5> m_key;
	required_memory_region m_region_maincpu;

	uint16_t m_mux_data;

	DECLARE_READ16_MEMBER(csplayh5_mux_r);
	DECLARE_WRITE16_MEMBER(csplayh5_mux_w);
	DECLARE_WRITE16_MEMBER(tmp68301_parallel_port_w);

	#if USE_H8
	DECLARE_READ16_MEMBER(test_r);
	DECLARE_WRITE_LINE_MEMBER(ide_irq);
	#endif

	DECLARE_DRIVER_INIT(csplayh1);

	DECLARE_DRIVER_INIT(aimode);
	DECLARE_DRIVER_INIT(bikiniko);
	DECLARE_DRIVER_INIT(csplayh5);
	DECLARE_DRIVER_INIT(csplayh6);
	DECLARE_DRIVER_INIT(csplayh7);
	DECLARE_DRIVER_INIT(fuudol);
	DECLARE_DRIVER_INIT(junai);
	DECLARE_DRIVER_INIT(junai2);
	DECLARE_DRIVER_INIT(mjgalpri);
	DECLARE_DRIVER_INIT(mjmania);
	DECLARE_DRIVER_INIT(mogitate);
	DECLARE_DRIVER_INIT(nichisel);
	DECLARE_DRIVER_INIT(pokoachu);
	DECLARE_DRIVER_INIT(renaimj);
	DECLARE_DRIVER_INIT(thenanpa);
	DECLARE_DRIVER_INIT(tsuwaku);

	virtual void machine_reset() override;
	TIMER_DEVICE_CALLBACK_MEMBER(csplayh5_irq);
	DECLARE_WRITE_LINE_MEMBER(csplayh5_vdp0_interrupt);

	void general_init(int patchaddress, int patchvalue);
};





READ16_MEMBER(csplayh5_state::csplayh5_mux_r)
{
	for(int i=0;i<5;i++)
	{
		if(m_mux_data & 1 << i)
			return m_key[i]->read();
	}

	popmessage("Multiple bytes used for mux %02x",m_mux_data);

	return 0xffff;
}

WRITE16_MEMBER(csplayh5_state::csplayh5_mux_w)
{
	m_mux_data = (~data & 0x1f);
}

static ADDRESS_MAP_START( csplayh5_map, AS_PROGRAM, 16, csplayh5_state )
	AM_RANGE(0x000000, 0x03ffff) AM_ROM

	AM_RANGE(0x200000, 0x200001) AM_READ_PORT("DSW") AM_DEVWRITE8("nichisnd", nichisnd_device,sound_host_command_w,0xff00)
	AM_RANGE(0x200200, 0x200201) AM_READWRITE(csplayh5_mux_r,csplayh5_mux_w)
	AM_RANGE(0x200400, 0x200401) AM_READ_PORT("SYSTEM")

	AM_RANGE(0x200600, 0x200607) AM_DEVREADWRITE8("v9958", v9958_device, read, write, 0x00ff)

	AM_RANGE(0x800000, 0xbfffff) AM_ROM AM_REGION("blit_gfx",0) // GFX ROM routes here

	AM_RANGE(0xfffc00, 0xffffff) AM_DEVREADWRITE("tmp68301", tmp68301_device, regs_r, regs_w)  // TMP68301 Registers

	AM_RANGE(0xc00000, 0xc7ffff) AM_RAM AM_SHARE("nvram") AM_MIRROR(0x380000) // work RAM
ADDRESS_MAP_END

#if USE_H8
READ16_MEMBER(csplayh5_state::test_r)
{
	return machine().rand();
}

static ADDRESS_MAP_START( csplayh5_sub_map, AS_PROGRAM, 16, csplayh5_state )
	AM_RANGE(0x000000, 0x01ffff) AM_ROM

	AM_RANGE(0x02000a, 0x02000b) AM_READ(test_r)
//  AM_RANGE(0x020008, 0x02000f) AM_DEVREADWRITE("ide", ide_controller_device, read_cs0, write_cs0)

	AM_RANGE(0x040018, 0x040019) AM_READ(test_r)
	AM_RANGE(0x040028, 0x04002f) AM_DEVREADWRITE("ide", ide_controller_device, read_cs0, write_cs0) // correct?
	AM_RANGE(0x040036, 0x040037) AM_READ(test_r)

	AM_RANGE(0x078000, 0x07ffff) AM_MIRROR(0xf80000) AM_RAM //AM_SHARE("nvram")
ADDRESS_MAP_END


static ADDRESS_MAP_START( csplayh5_sub_io_map, AS_IO, 16, csplayh5_state )
	AM_RANGE(0x0a, 0x0b) AM_READ(test_r)
ADDRESS_MAP_END
#endif


static INPUT_PORTS_START( csplayh5 )
	PORT_START("KEY.0")
	PORT_BIT( 0x0001, IP_ACTIVE_LOW, IPT_START1 )
	PORT_BIT( 0x0002, IP_ACTIVE_LOW, IPT_MAHJONG_KAN ) PORT_PLAYER(1)
	PORT_BIT( 0x0004, IP_ACTIVE_LOW, IPT_MAHJONG_M ) PORT_PLAYER(1)
	PORT_BIT( 0x0008, IP_ACTIVE_LOW, IPT_MAHJONG_I ) PORT_PLAYER(1)
	PORT_BIT( 0x0010, IP_ACTIVE_LOW, IPT_MAHJONG_E ) PORT_PLAYER(1)
	PORT_BIT( 0x0020, IP_ACTIVE_LOW, IPT_MAHJONG_A ) PORT_PLAYER(1)
	PORT_BIT( 0x0040, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x0080, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x0100, IP_ACTIVE_LOW, IPT_START2 )
	PORT_BIT( 0x0200, IP_ACTIVE_LOW, IPT_MAHJONG_KAN ) PORT_PLAYER(2)
	PORT_BIT( 0x0400, IP_ACTIVE_LOW, IPT_MAHJONG_M ) PORT_PLAYER(2)
	PORT_BIT( 0x0800, IP_ACTIVE_LOW, IPT_MAHJONG_I ) PORT_PLAYER(2)
	PORT_BIT( 0x1000, IP_ACTIVE_LOW, IPT_MAHJONG_E ) PORT_PLAYER(2)
	PORT_BIT( 0x2000, IP_ACTIVE_LOW, IPT_MAHJONG_A ) PORT_PLAYER(2)
	PORT_BIT( 0x4000, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x8000, IP_ACTIVE_LOW, IPT_UNKNOWN )

	PORT_START("KEY.1")
	PORT_BIT( 0x0001, IP_ACTIVE_LOW, IPT_MAHJONG_BET ) PORT_PLAYER(1)
	PORT_BIT( 0x0002, IP_ACTIVE_LOW, IPT_MAHJONG_REACH ) PORT_PLAYER(1)
	PORT_BIT( 0x0004, IP_ACTIVE_LOW, IPT_MAHJONG_N ) PORT_PLAYER(1)
	PORT_BIT( 0x0008, IP_ACTIVE_LOW, IPT_MAHJONG_J ) PORT_PLAYER(1)
	PORT_BIT( 0x0010, IP_ACTIVE_LOW, IPT_MAHJONG_F ) PORT_PLAYER(1)
	PORT_BIT( 0x0020, IP_ACTIVE_LOW, IPT_MAHJONG_B ) PORT_PLAYER(1)
	PORT_BIT( 0x0040, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x0080, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x0100, IP_ACTIVE_LOW, IPT_MAHJONG_BET ) PORT_PLAYER(2)
	PORT_BIT( 0x0200, IP_ACTIVE_LOW, IPT_MAHJONG_REACH ) PORT_PLAYER(2)
	PORT_BIT( 0x0400, IP_ACTIVE_LOW, IPT_MAHJONG_N ) PORT_PLAYER(2)
	PORT_BIT( 0x0800, IP_ACTIVE_LOW, IPT_MAHJONG_J ) PORT_PLAYER(2)
	PORT_BIT( 0x1000, IP_ACTIVE_LOW, IPT_MAHJONG_F ) PORT_PLAYER(2)
	PORT_BIT( 0x2000, IP_ACTIVE_LOW, IPT_MAHJONG_B ) PORT_PLAYER(2)
	PORT_BIT( 0x4000, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x8000, IP_ACTIVE_LOW, IPT_UNKNOWN )

	PORT_START("KEY.2")
	PORT_BIT( 0x0001, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x0002, IP_ACTIVE_LOW, IPT_MAHJONG_RON ) PORT_PLAYER(1)
	PORT_BIT( 0x0004, IP_ACTIVE_LOW, IPT_MAHJONG_CHI ) PORT_PLAYER(1)
	PORT_BIT( 0x0008, IP_ACTIVE_LOW, IPT_MAHJONG_K ) PORT_PLAYER(1)
	PORT_BIT( 0x0010, IP_ACTIVE_LOW, IPT_MAHJONG_G ) PORT_PLAYER(1)
	PORT_BIT( 0x0020, IP_ACTIVE_LOW, IPT_MAHJONG_C ) PORT_PLAYER(1)
	PORT_BIT( 0x0040, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x0080, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x0100, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x0200, IP_ACTIVE_LOW, IPT_MAHJONG_RON ) PORT_PLAYER(2)
	PORT_BIT( 0x0400, IP_ACTIVE_LOW, IPT_MAHJONG_CHI ) PORT_PLAYER(2)
	PORT_BIT( 0x0800, IP_ACTIVE_LOW, IPT_MAHJONG_K ) PORT_PLAYER(2)
	PORT_BIT( 0x1000, IP_ACTIVE_LOW, IPT_MAHJONG_G ) PORT_PLAYER(2)
	PORT_BIT( 0x2000, IP_ACTIVE_LOW, IPT_MAHJONG_C ) PORT_PLAYER(2)
	PORT_BIT( 0x4000, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x8000, IP_ACTIVE_LOW, IPT_UNKNOWN )

	PORT_START("KEY.3")
	PORT_BIT( 0x0001, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x0002, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x0004, IP_ACTIVE_LOW, IPT_MAHJONG_PON ) PORT_PLAYER(1)
	PORT_BIT( 0x0008, IP_ACTIVE_LOW, IPT_MAHJONG_L ) PORT_PLAYER(1)
	PORT_BIT( 0x0010, IP_ACTIVE_LOW, IPT_MAHJONG_H ) PORT_PLAYER(1)
	PORT_BIT( 0x0020, IP_ACTIVE_LOW, IPT_MAHJONG_D ) PORT_PLAYER(1)
	PORT_BIT( 0x0040, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x0080, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x0100, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x0200, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x0400, IP_ACTIVE_LOW, IPT_MAHJONG_PON ) PORT_PLAYER(2)
	PORT_BIT( 0x0800, IP_ACTIVE_LOW, IPT_MAHJONG_L ) PORT_PLAYER(2)
	PORT_BIT( 0x1000, IP_ACTIVE_LOW, IPT_MAHJONG_H ) PORT_PLAYER(2)
	PORT_BIT( 0x2000, IP_ACTIVE_LOW, IPT_MAHJONG_D ) PORT_PLAYER(2)
	PORT_BIT( 0x4000, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x8000, IP_ACTIVE_LOW, IPT_UNKNOWN )

	PORT_START("KEY.4")
	PORT_BIT( 0x0001, IP_ACTIVE_LOW, IPT_MAHJONG_SMALL ) PORT_PLAYER(1)
	PORT_BIT( 0x0002, IP_ACTIVE_LOW, IPT_MAHJONG_BIG ) PORT_PLAYER(1)
	PORT_BIT( 0x0004, IP_ACTIVE_LOW, IPT_MAHJONG_FLIP_FLOP ) PORT_PLAYER(1)
	PORT_BIT( 0x0008, IP_ACTIVE_LOW, IPT_MAHJONG_DOUBLE_UP ) PORT_PLAYER(1)
	PORT_BIT( 0x0010, IP_ACTIVE_LOW, IPT_MAHJONG_SCORE ) PORT_PLAYER(1)
	PORT_BIT( 0x0020, IP_ACTIVE_LOW, IPT_MAHJONG_LAST_CHANCE ) PORT_PLAYER(1)
	PORT_BIT( 0x0040, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x0080, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x0100, IP_ACTIVE_LOW, IPT_MAHJONG_SMALL ) PORT_PLAYER(2)
	PORT_BIT( 0x0200, IP_ACTIVE_LOW, IPT_MAHJONG_BIG ) PORT_PLAYER(2)
	PORT_BIT( 0x0400, IP_ACTIVE_LOW, IPT_MAHJONG_FLIP_FLOP ) PORT_PLAYER(2)
	PORT_BIT( 0x0800, IP_ACTIVE_LOW, IPT_MAHJONG_DOUBLE_UP ) PORT_PLAYER(2)
	PORT_BIT( 0x1000, IP_ACTIVE_LOW, IPT_MAHJONG_SCORE ) PORT_PLAYER(2)
	PORT_BIT( 0x2000, IP_ACTIVE_LOW, IPT_MAHJONG_LAST_CHANCE ) PORT_PLAYER(2)
	PORT_BIT( 0x4000, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x8000, IP_ACTIVE_LOW, IPT_UNKNOWN )

	PORT_START("DSW")
	PORT_DIPNAME( 0x0001, 0x0000, "DSWA" )
	PORT_DIPSETTING(      0x0000, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0001, DEF_STR( On ) )
	PORT_DIPNAME( 0x0002, 0x0000, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0002, DEF_STR( On ) )
	PORT_DIPNAME( 0x0004, 0x0000, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0004, DEF_STR( On ) )
	PORT_DIPNAME( 0x0008, 0x0000, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0008, DEF_STR( On ) )
	PORT_DIPNAME( 0x0010, 0x0000, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0010, DEF_STR( On ) )
	PORT_DIPNAME( 0x0020, 0x0000, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0020, DEF_STR( On ) )
	PORT_DIPNAME( 0x0040, 0x0000, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0040, DEF_STR( On ) )
	PORT_DIPNAME( 0x0080, 0x0000, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0080, DEF_STR( On ) )
	PORT_DIPNAME( 0x0100, 0x0000, "DSWB" )
	PORT_DIPSETTING(      0x0000, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0100, DEF_STR( On ) )
	PORT_DIPNAME( 0x0200, 0x0000, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0200, DEF_STR( On ) )
	PORT_DIPNAME( 0x0400, 0x0000, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0400, DEF_STR( On ) )
	PORT_DIPNAME( 0x0800, 0x0000, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0800, DEF_STR( On ) )
	PORT_DIPNAME( 0x1000, 0x0000, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x1000, DEF_STR( On ) )
	PORT_DIPNAME( 0x2000, 0x2000, DEF_STR( Unknown ) ) //enters into analyzer in some games otherwise
	PORT_DIPSETTING(      0x0000, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x2000, DEF_STR( On ) )
	PORT_DIPNAME( 0x4000, 0x0000, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x4000, DEF_STR( On ) )
	PORT_DIPNAME( 0x8000, 0x0000, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x8000, DEF_STR( On ) )

	PORT_START("SYSTEM")
	PORT_BIT( 0x0001, IP_ACTIVE_LOW, IPT_COIN1 )            // COIN1
	PORT_BIT( 0x0002, IP_ACTIVE_LOW, IPT_COIN2 )            // COIN2
	PORT_BIT( 0x0004, IP_ACTIVE_LOW, IPT_SERVICE1 ) PORT_NAME("Credit Clear")
	PORT_BIT( 0x0008, IP_ACTIVE_LOW, IPT_MEMORY_RESET )
	PORT_BIT( 0x0010, IP_ACTIVE_LOW, IPT_SERVICE ) // labeled analyzer in self-test
	PORT_BIT( 0x0020, IP_ACTIVE_LOW, IPT_GAMBLE_PAYOUT ) PORT_CODE(KEYCODE_4) PORT_NAME("Out Coin")
	PORT_BIT( 0xffc0, IP_ACTIVE_LOW, IPT_UNUSED )
INPUT_PORTS_END


void csplayh5_state::machine_reset()
{
}

TIMER_DEVICE_CALLBACK_MEMBER(csplayh5_state::csplayh5_irq)
{
	int scanline = param;

	if(scanline == 212*2)
		m_tmp68301->external_interrupt_0();
}

WRITE_LINE_MEMBER(csplayh5_state::csplayh5_vdp0_interrupt)
{
	/* this is not used as the v9938 interrupt callbacks are broken
	   interrupts seem to be fired quite randomly */
}

#if USE_H8
WRITE_LINE_MEMBER(csplayh5_state::ide_irq)
{
	printf("h8 ide alive %d\n",state);
}
#endif

WRITE16_MEMBER(csplayh5_state::tmp68301_parallel_port_w)
{
	/*
	    -x-- ---- used during ROM check, h8 reset assert?
	    ---- x--- enable DVD sound? Used by aimode at very least
	*/

	if(data & ~0x48)
		printf("%04x\n",data);
}


static MACHINE_CONFIG_START( csplayh5 )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu",M68000,16000000) /* TMP68301-16 */
	MCFG_CPU_PROGRAM_MAP(csplayh5_map)
	MCFG_CPU_IRQ_ACKNOWLEDGE_DEVICE("tmp68301", tmp68301_device, irq_callback)

	MCFG_TIMER_DRIVER_ADD_SCANLINE("scantimer", csplayh5_state, csplayh5_irq, "screen", 0, 1)

	MCFG_DEVICE_ADD("tmp68301", TMP68301, 0)
	MCFG_TMP68301_OUT_PARALLEL_CB(WRITE16(csplayh5_state, tmp68301_parallel_port_w))

#if USE_H8
	MCFG_CPU_ADD("subcpu", H83002, DVD_CLOCK/2)    /* unknown divider */
	MCFG_CPU_PROGRAM_MAP(csplayh5_sub_map)
	MCFG_CPU_IO_MAP(csplayh5_sub_io_map)

	MCFG_IDE_CONTROLLER_ADD("ide", ata_devices, "hdd", nullptr, true) // dvd
	MCFG_ATA_INTERFACE_IRQ_HANDLER(WRITELINE(csplayh5_state, ide_irq))
#endif

	MCFG_NVRAM_ADD_0FILL("nvram")

	/* video hardware */
	MCFG_V9958_ADD("v9958", "screen", 0x20000, XTAL_21_4772MHz) // typical 9958 clock, not verified
	MCFG_V99X8_INTERRUPT_CALLBACK(WRITELINE(csplayh5_state, csplayh5_vdp0_interrupt))
	MCFG_V99X8_SCREEN_ADD_NTSC("screen", "v9958", XTAL_21_4772MHz)

	/* sound hardware */
	MCFG_NICHISND_ADD("nichisnd")
MACHINE_CONFIG_END

/***************************************************************************

  Game driver(s)

***************************************************************************/

void csplayh5_state::general_init(int patchaddress, int patchvalue)
{
	#if !USE_H8
	uint16_t *MAINROM = (uint16_t *)m_region_maincpu->base();
	/* patch DVD comms check */
	MAINROM[patchaddress] = patchvalue;
	#endif

	//uint8_t *SNDROM = m_region_:nichisnd:audiorom->base();

	/* initialize sound rom bank */
	//soundbank_w(0);

	/* patch sound program */
	//SNDROM[0x0213] = 0x00;          // DI -> NOP

}

DRIVER_INIT_MEMBER(csplayh5_state,csplayh1)  { general_init(0x6880/2, 0x6020); }

DRIVER_INIT_MEMBER(csplayh5_state,aimode)    { general_init(0x9cda/2, 0x6018); }
DRIVER_INIT_MEMBER(csplayh5_state,bikiniko)  { general_init(0x585c/2, 0x6018); }
DRIVER_INIT_MEMBER(csplayh5_state,csplayh5)  { general_init(0x4cb4/2, 0x6018); }
DRIVER_INIT_MEMBER(csplayh5_state,csplayh6)  { general_init(0x5976/2, 0x6018); }
DRIVER_INIT_MEMBER(csplayh5_state,csplayh7)  { general_init(0x7a20/2, 0x6018); }
DRIVER_INIT_MEMBER(csplayh5_state,junai)     { general_init(0x679c/2, 0x6018); }
DRIVER_INIT_MEMBER(csplayh5_state,junai2)    { general_init(0x6588/2, 0x6018); }
DRIVER_INIT_MEMBER(csplayh5_state,fuudol)    { general_init(0x9166/2, 0x6018); }
DRIVER_INIT_MEMBER(csplayh5_state,mjgalpri)  { general_init(0x5396/2, 0x6018); }
DRIVER_INIT_MEMBER(csplayh5_state,mjmania)   { general_init(0x6b96/2, 0x6018); }
DRIVER_INIT_MEMBER(csplayh5_state,mogitate)  { general_init(0x6ab4/2, 0x6018); }
DRIVER_INIT_MEMBER(csplayh5_state,nichisel)  { general_init(0x9cd6/2, 0x6018); }
DRIVER_INIT_MEMBER(csplayh5_state,pokoachu)  { general_init(0x7b1e/2, 0x6018); }
DRIVER_INIT_MEMBER(csplayh5_state,renaimj)   { general_init(0x568c/2, 0x6018); }
DRIVER_INIT_MEMBER(csplayh5_state,thenanpa)  { general_init(0x69ec/2, 0x6018); }
DRIVER_INIT_MEMBER(csplayh5_state,tsuwaku)   { general_init(0x856e/2, 0x6018); }


/*
 * Base BIOS root (DVD board is common for all DVD games)
 */

#define ROM_LOAD16_WORD_SWAP_BIOS(bios,name,offset,length,hash) \
		ROMX_LOAD(name, offset, length, hash, ROM_GROUPWORD | ROM_REVERSE | ROM_BIOS(bios+1)) /* Note '+1' */

#define DVD_BIOS \
	ROM_REGION( 0x20000, "subcpu", 0 ) \
	ROM_SYSTEM_BIOS( 0,  "vb102",    "va1b102" ) \
	ROM_LOAD16_WORD_SWAP_BIOS( 0, "va1b102.u2",   0x00000, 0x20000, CRC(67374715) SHA1(8767cbd81614c2686a1adb70159f909e8ffd634d) ) \
	ROM_SYSTEM_BIOS( 1,  "vb101",    "va1b101" ) \
	ROM_LOAD16_WORD_SWAP_BIOS( 1, "va1b101.u2",   0x00000, 0x20000, CRC(b92a83c8) SHA1(bd6d9adaa74cf7512478838d1bd5a79dbd0c4aa1) ) \
	ROM_SYSTEM_BIOS( 2,  "va101",    "va1a101" ) \
	ROM_LOAD16_WORD_SWAP_BIOS( 2, "va1a101.u2",   0x00000, 0x20000, CRC(36135792) SHA1(1b9c50bd02df8227b228b35cc485efd5a13ec639) )


// dummy ROM definition
ROM_START( nichidvd )
	ROM_REGION( 0x40000, "maincpu", ROMREGION_ERASE00 ) // tmp68301 prg

	DVD_BIOS

	ROM_REGION( 0x20000, ":nichisnd:audiorom", ROMREGION_ERASE00 ) // z80

	ROM_REGION( 0x400000, "blit_gfx", ROMREGION_ERASEFF ) // blitter based gfxs

	DISK_REGION( "ide:0:hdd:image" )
ROM_END

// TODO: this surely uses a different subboard
ROM_START( csplayh1 )
	ROM_REGION( 0x40000, "maincpu", 0 ) // tmp68301 prg
	ROM_LOAD16_BYTE( "3.bin", 0x000000, 0x020000, CRC(86ac0289) SHA1(7ae3047fc7ea22705cc5b04d0ec6c792c429e8ee) )
	ROM_LOAD16_BYTE( "2.bin", 0x000001, 0x020000, CRC(1f056e64) SHA1(7c5fb318abcd87313ef739dec191af9bcf284f24) )

	ROM_REGION( 0x20000, "subcpu", 0 ) // h8, cd-rom player
	ROM_LOAD16_WORD_SWAP( "u2",   0x00000, 0x20000, NO_DUMP )

	ROM_REGION( 0x20000, ":nichisnd:audiorom", 0 ) // z80
	ROM_LOAD( "1.bin", 0x000000, 0x020000, CRC(8296d67f) SHA1(20eb944a2bd27980e1aaf60ca544059e84129760) )

	ROM_REGION( 0x400000, "blit_gfx", ROMREGION_ERASEFF ) // blitter based gfxs
	ROM_LOAD16_BYTE( "4.bin", 0x000000, 0x080000, CRC(2e63ee15) SHA1(78fefbc277234458212cded997d393bd8b82cf76) )
	ROM_LOAD16_BYTE( "8.bin", 0x000001, 0x080000, CRC(a8567f1b) SHA1(2a854ef8b1988ad097bbcbeddc4b275ad738e1e1) )

	DISK_REGION( "ide:0:hdd:image" )
	DISK_IMAGE_READONLY( "csplayh1", 0, SHA1(d6514882c2626e62c5079df9ac68ecb70fc33209) )

	ROM_REGION( 0x1000, "gal", ROMREGION_ERASE00 )
	ROM_LOAD( "gal16v8b.ic8", 0x000000, 0x0008c1, NO_DUMP )
ROM_END

ROM_START( mjgalpri )
	ROM_REGION( 0x40000, "maincpu", 0 ) // tmp68301 prg
	ROM_LOAD16_BYTE( "2.ic3",            0x000000, 0x020000, CRC(e8427076) SHA1(9b449599ffac2b67a29fac11d1e85218668d805d) )
	ROM_LOAD16_BYTE( "1.ic2",            0x000001, 0x020000, CRC(653fcc14) SHA1(6231ec5f45a9f5e587dcd00ff85f9bbfae7364ab) )

	ROM_REGION( 0x20000, ":nichisnd:audiorom", 0 ) // z80
	ROM_LOAD( "11.ic51",           0x000000, 0x020000, CRC(7b9b1887) SHA1(1393a1d79f3cc7ab68275791af4ec16e825056df) )

	DVD_BIOS

	ROM_REGION( 0x400000, "blit_gfx", ROMREGION_ERASEFF ) // blitter based gfxs
	ROM_LOAD16_BYTE( "3.ic40",            0x000000, 0x080000, CRC(6497bc8f) SHA1(ce0ecfab8df87f7356aa42648e47ffda53840188) )
	ROM_LOAD16_BYTE( "4.ic41",            0x000001, 0x080000, CRC(3ac982e8) SHA1(d889d45888cf7bcb5af808f63e9ad41204bd5992) )

	ROM_REGION( 0x040000, "gal", ROMREGION_ERASE00 )
	ROM_LOAD( "gal16v8b.020", 0x000000, 0x040000, CRC(4c92a523) SHA1(51da73fdfdfccdc070fa8a13163e031438b50876) )

	DISK_REGION( "ide:0:hdd:image" )
	DISK_IMAGE_READONLY( "nb8001", 0, SHA1(30f356af4e08567273a88758bb0ddd3544eea228) )
ROM_END

ROM_START( junai )
	ROM_REGION( 0x40000, "maincpu", 0 ) // tmp68301 prg
	ROM_LOAD16_BYTE( "2.ic3",   0x00000, 0x20000, CRC(5923bf2e) SHA1(8fc7b95a44eb792ce03c1bffb9ad56f82d34b470) )
	ROM_LOAD16_BYTE( "1.ic2",   0x00001, 0x20000, CRC(4ac649ee) SHA1(f5b5bccecb6eba5addcf6a57e54deff7f29f6381) )

	DVD_BIOS

	ROM_REGION( 0x20000, ":nichisnd:audiorom", 0 ) // z80
	ROM_LOAD( "11.ic51",   0x00000, 0x20000, CRC(a0472ea5) SHA1(0fd04941ff595cffe64357f3a1a9dc1170db8703) )

	ROM_REGION( 0x400000, "blit_gfx", ROMREGION_ERASEFF ) // blitter based gfxs
	ROM_LOAD16_BYTE( "3.ic40",   0x00000, 0x80000, CRC(f17fa4c4) SHA1(fd8b69b18f9ac00f468d751bf1ea0715498ea742) )
	ROM_LOAD16_BYTE( "4.ic41",   0x00001, 0x80000, CRC(4182dc30) SHA1(89601c62b74aff3d65b075d4b5cd1eb2ccf4e386) )
	// 0x100000 - 0x3fffff empty sockets

	DISK_REGION( "ide:0:hdd:image" )
	DISK_IMAGE_READONLY( "junai", 0, SHA1(0491533e0ce3e4d2af608ea0b9d9646316b512bd) )
ROM_END

ROM_START( csplayh5 )
	ROM_REGION( 0x40000, "maincpu", 0 ) // tmp68301 prg
	ROM_LOAD16_BYTE( "2.ic3",   0x00000, 0x20000, CRC(980bf3b0) SHA1(89da7354552f30aaa9d46442972c060b4b0f8979) )
	ROM_LOAD16_BYTE( "1.ic2",   0x00001, 0x20000, CRC(81ca49a4) SHA1(601b6802ab85be61f45a64f5b4c7e1f1ae5ee887) )

	DVD_BIOS

	ROM_REGION( 0x20000, ":nichisnd:audiorom", 0 ) // z80
	ROM_LOAD( "11.ic51",   0x00000, 0x20000, CRC(0b920806) SHA1(95f50ebfb296ba29aaa8079a41f5362cb9e879cc) )

	ROM_REGION( 0x400000, "blit_gfx", ROMREGION_ERASEFF ) // blitter based gfxs
	ROM_LOAD16_BYTE( "3.ic40",   0x00000, 0x80000, CRC(895b5e1f) SHA1(9398ee95d391f74d62fe641cb75311f31d4d1c8d) )
	ROM_LOAD16_BYTE( "4.ic41",   0x00001, 0x80000, CRC(113d7e96) SHA1(f3fb9c719544417a6a018b82f07c65bf73de21ff) )
	// 0x100000 - 0x3fffff empty sockets

	DISK_REGION( "ide:0:hdd:image" )
	DISK_IMAGE_READONLY( "csplayh5", 0, SHA1(ce4883ce1351ce5299e41bfbd9a5ae8078b82b8c) )
ROM_END

ROM_START( junai2 )
	ROM_REGION( 0x40000, "maincpu", 0 ) // tmp68301 prg
	ROM_LOAD16_BYTE( "2.ic3",   0x00000, 0x20000, CRC(b0ce71d6) SHA1(35cff8f8b18312808e2f6b96f02d952b0d1f04a1) )
	ROM_LOAD16_BYTE( "1.ic2",   0x00001, 0x20000, CRC(5a428e91) SHA1(dffce6f0a48cc4110970f124684dcaa267fe1b7f) )

	DVD_BIOS

	ROM_REGION( 0x20000, ":nichisnd:audiorom", 0 ) // z80
	ROM_LOAD( "11.ic51",   0x00000, 0x20000, CRC(a4b07757) SHA1(5010f28d7a80af0cc3f4fd135f777950fb2cf679) )

	ROM_REGION( 0x400000, "blit_gfx", ROMREGION_ERASEFF ) // blitter based gfxs
	ROM_LOAD16_BYTE( "3.ic40",   0x00000, 0x80000, CRC(95ecb29d) SHA1(e07bb0ff15aaee9fb26d8ef7f4644b47045c81a8) )
	ROM_LOAD16_BYTE( "4.ic41",   0x00001, 0x80000, CRC(5b37c8dd) SHA1(8de5e2f92721c6679c6506850a442cafff89653f) )
	// 0x100000 - 0x3fffff empty sockets

	DISK_REGION( "ide:0:hdd:image" )
	DISK_IMAGE_READONLY( "junai2", 0, SHA1(dc9633a101f20f03fd9b4414c10274d2539fb7c2) )

	ROM_REGION( 0x1000, "gal", ROMREGION_ERASE00 )
	ROM_LOAD( "gal16v8b.ic8", 0x000000, 0x0008c1, BAD_DUMP CRC(01c2895a) SHA1(782166a60fa14d5faa5a92629f7ca65a878ad7fe) )
ROM_END


ROM_START( mogitate )
	ROM_REGION( 0x40000, "maincpu", 0 ) // tmp68301 prg
	ROM_LOAD16_BYTE( "1.ic2",            0x000001, 0x020000, CRC(42ec6c2e) SHA1(a0279502e1f7e62f072ec6612caf198aa0ae3af7) )
	ROM_LOAD16_BYTE( "2.ic3",            0x000000, 0x020000, CRC(f71546c6) SHA1(546b0d12e7b1627c96d5a17c015bdbbca1e93232) )

	DVD_BIOS

	ROM_REGION( 0x20000, ":nichisnd:audiorom", 0 ) // z80
	ROM_LOAD( "11.ic51",           0x000000, 0x020000, CRC(7927c1d6) SHA1(15f0c0051124e7b7667eb721dd12938333b31899) )

	ROM_REGION( 0x400000, "blit_gfx", ROMREGION_ERASEFF ) // blitter based gfxs
	ROM_LOAD16_BYTE( "3.ic40",            0x000000, 0x080000, CRC(ea655990) SHA1(7f59cfab21e8858625e82a9501acc943b07f799c) )
	ROM_LOAD16_BYTE( "4.ic41",            0x000001, 0x080000, CRC(4c910b86) SHA1(48007f03f4e445b9de15531afe821c1b18fccae1) )

	DISK_REGION( "ide:0:hdd:image" )
	DISK_IMAGE_READONLY( "nb8006", 0, SHA1(aa911e46e791d89ce4fed4a32b4b0637ba3a9920) )

	ROM_REGION( 0x040000, "gal", ROMREGION_ERASE00 )
	ROM_LOAD( "gal16v8b.020", 0x000000, 0x040000, CRC(ac5c9495) SHA1(1c54ecf6dedbf8c3a29207c1c91b52e2ff394d9d) )
ROM_END

ROM_START( mjmania )
	ROM_REGION( 0x40000, "maincpu", 0 ) // tmp68301 prg
	ROM_LOAD16_BYTE( "2.ic3", 0x000000, 0x020000, CRC(7b0f79c5) SHA1(73f23f68db4426b32583a7922abf773d67c76862) )
	ROM_LOAD16_BYTE( "1.ic2", 0x000001, 0x020000, CRC(19192ae7) SHA1(4e9fca04b567c8ef9136a3ab87b21207a44a24c4) )

	DVD_BIOS

	ROM_REGION( 0x20000, ":nichisnd:audiorom", 0 ) // z80
	ROM_LOAD( "11.ic51", 0x000000, 0x020000, CRC(f0c3bb11) SHA1(691a0ff53a9417e69051e9e2bdee7500bc6a746b) )

	ROM_REGION( 0x400000, "blit_gfx", ROMREGION_ERASEFF ) // blitter based gfxs
	ROM_LOAD16_BYTE( "3.ic40", 0x000000, 0x080000, CRC(37dde764) SHA1(0530b63d8e682cdf01128057fdc3a8c23262afc9) )
	ROM_LOAD16_BYTE( "4.ic41", 0x000001, 0x080000, CRC(dea4a2d2) SHA1(0118eb1330c9da8fead99f64fc015fd343fed79b) )

	DISK_REGION( "ide:0:hdd:image" )
	DISK_IMAGE_READONLY( "mjmania", 0, SHA1(7117f2045fd04a3d8f8e06a6a98e8f585c4da301) )

	ROM_REGION( 0x1000, "gal", ROMREGION_ERASE00 )
	ROM_LOAD( "gal16v8b.ic8", 0x000000, 0x0008c1, BAD_DUMP CRC(6a92b563) SHA1(a6c4305cf021f37845f99713427daa9394b6ec7d) )
ROM_END

ROM_START( renaimj )
	ROM_REGION( 0x40000, "maincpu", 0 ) // tmp68301 prg
	ROM_LOAD16_BYTE( "2.ic3",   0x00000, 0x20000, CRC(5455e94c) SHA1(97257ed020848611bf9f9637f1eb9ee3433a6e20) )
	ROM_LOAD16_BYTE( "1.ic2",   0x00001, 0x20000, CRC(285a5651) SHA1(c572a7c82759600e29e31518c69b17ae173c2263) )

	DVD_BIOS

	ROM_REGION( 0x20000, ":nichisnd:audiorom", 0 ) // z80
	ROM_LOAD( "11.ic51",   0x00000, 0x20000, CRC(614d17b9) SHA1(d6fb4441f55902c2b89b4bec53aae5311d81f07b) )

	ROM_REGION( 0x400000, "blit_gfx", ROMREGION_ERASEFF ) // blitter based gfxs
	ROM_LOAD16_BYTE( "3.ic40",   0x00000, 0x80000, CRC(790aa63d) SHA1(d94b88084311f317d584a33ad5b483403f2bf226) )
	ROM_LOAD16_BYTE( "4.ic41",   0x00001, 0x80000, CRC(6d1c9efd) SHA1(c9ea9d6e6d34db5635fc55d41e7bb54a41948d27) )
	// 0x100000 - 0x3fffff empty sockets

	DISK_REGION( "ide:0:hdd:image" )
	DISK_IMAGE_READONLY( "nb8008", 0, SHA1(49c92cb9b08ee7773f3d93fce0bbecc3c0ae654d) )

	ROM_REGION( 0x40000, "gal", ROMREGION_ERASE00 )
	ROM_LOAD( "gal18v8b.020", 0x000000, 0x040000, CRC(0a32a144) SHA1(f3b4a1174adbb2f7b7500adeafa20142f6a16d08) )
ROM_END

ROM_START( bikiniko )
	ROM_REGION( 0x40000, "maincpu", 0 ) // tmp68301 prg
	ROM_LOAD16_BYTE( "2.ic3",   0x00000, 0x20000, CRC(b80b5484) SHA1(35769d9502cbe587dad6380c35e535cea1578227) )
	ROM_LOAD16_BYTE( "1.ic2",   0x00001, 0x20000, CRC(13a885af) SHA1(ba8221fab1a37f1937e4399eabe3eaa9093884d3) )

	DVD_BIOS

	ROM_REGION( 0x20000, ":nichisnd:audiorom", 0 ) // z80
	ROM_LOAD( "11.ic51",   0x00000, 0x20000, CRC(4a2142d6) SHA1(3a762f7b7cccdb6715b5f59524b04b12694fc130) )

	ROM_REGION( 0x400000, "blit_gfx", ROMREGION_ERASEFF ) // blitter based gfxs
	ROM_LOAD16_BYTE( "3.ic40",   0x00000, 0x80000, CRC(12914d3b) SHA1(de0cdb47ee5cbf8bd19ab19b1b8d8afe103dcedf) )
	ROM_LOAD16_BYTE( "4.ic41",   0x00001, 0x80000, CRC(1e2e1cf3) SHA1(f71b5dedf4f897644d519e412651152d0d81edb8) )
	// 0x100000 - 0x3fffff empty sockets

	DISK_REGION( "ide:0:hdd:image" )
	DISK_IMAGE_READONLY( "bikiniko", 0, SHA1(2189b676746dd848b9b5eb69f9663d6dccd63787) )
ROM_END

ROM_START( csplayh6 )
	ROM_REGION( 0x40000, "maincpu", 0 ) // tmp68301 prg
	ROM_LOAD16_BYTE( "2.ic3",   0x00000, 0x20000, CRC(12d896cc) SHA1(7d602b44cb781dbc52b112c9f1a5d88a332dfbe0) )
	ROM_LOAD16_BYTE( "1.ic2",   0x00001, 0x20000, CRC(1e4679ca) SHA1(f5df03c07f749906bbcef26a4a5d433564d4aeb8) )

	DVD_BIOS

	ROM_REGION( 0x20000, ":nichisnd:audiorom", 0 ) // z80
	ROM_LOAD( "11.ic51",   0x00000, 0x20000, CRC(3ce03f2d) SHA1(5ccdcac8bad25b4f680ed7a2074575711c25af41) )

	ROM_REGION( 0x400000, "blit_gfx", ROMREGION_ERASEFF ) // blitter based gfxs
	ROM_LOAD16_BYTE( "3.ic40",   0x00000, 0x80000, CRC(a09e7575) SHA1(76f4d7562a3fd479c1c6de22f704a0953a39bb0c) )
	ROM_LOAD16_BYTE( "4.ic41",   0x00001, 0x80000, CRC(858e0604) SHA1(64c23bc06898188798937770129697b3c4b547d6) )
	// 0x100000 - 0x3fffff empty sockets

	DISK_REGION( "ide:0:hdd:image" )
	DISK_IMAGE_READONLY( "nb8010", 0, SHA1(01e247fe1b86bbfe743e09a625432874f881a9a0) )

	ROM_REGION( 0x40000, "gal", ROMREGION_ERASE00 )
	ROM_LOAD( "palce16v8h.020_bad", 0x000000, 0x040000, BAD_DUMP CRC(2aec4e37) SHA1(79d64394c0f6f2c5e17ae9fc62eaa279da466ccd) )
ROM_END

ROM_START( thenanpa )
	ROM_REGION( 0x40000, "maincpu", 0 ) // tmp68301 prg
	ROM_LOAD16_BYTE( "2.ic3", 0x000000, 0x020000, CRC(ab0b686f) SHA1(a5681dbacbc60f3eb40e079779967cf69d9cb292) )
	ROM_LOAD16_BYTE( "1.ic2", 0x000001, 0x020000, CRC(48b65f9a) SHA1(ce35475d3b0e9e8dc69892428f3957d8d3d5f22c) )

	DVD_BIOS

	ROM_REGION( 0x20000, ":nichisnd:audiorom", 0 ) // z80
	ROM_LOAD( "11.ic51", 0x000000, 0x020000, CRC(f44c4095) SHA1(d43e464bd6d614c34791445f8fd4af2f62a4dfc2) )

	ROM_REGION( 0x400000, "blit_gfx", ROMREGION_ERASEFF ) // blitter based gfxs
	ROM_LOAD16_BYTE( "3.ic40", 0x000000, 0x080000, CRC(ee6b88c4) SHA1(64ae66a24f1639801c7bdda7faa0d604bb97ceb1) )
	ROM_LOAD16_BYTE( "4.ic41", 0x000001, 0x080000, CRC(ce987845) SHA1(2f7dca32a79ad6afbc55ca1d492b582f952688ff) )

	DISK_REGION( "ide:0:hdd:image" )
	DISK_IMAGE_READONLY( "thenanpa", 0,  SHA1(72bf8c75189e877508c5a64d5591738d23ed7e96) )

	ROM_REGION( 0x1000, "gal", ROMREGION_ERASE00 )
	ROM_LOAD( "gal16v8b.ic8", 0x000000, 0x0008c1, BAD_DUMP CRC(daffd0ac)SHA1(cbeff914163d425a9cb30fe8d62f91fca281b11f) )
ROM_END

ROM_START( pokoachu )
	ROM_REGION( 0x40000, "maincpu", 0 ) // tmp68301 prg
	ROM_LOAD16_BYTE( "2.ic3", 0x000000, 0x020000, CRC(db63c2c3) SHA1(528b0eead52e54af0c5accb5f96a382b1f9b7123) )
	ROM_LOAD16_BYTE( "1.ic2", 0x000001, 0x020000, CRC(789ffbc8) SHA1(44f3846414682e19465b485ffb89c7b78920cb0a)  )

	DVD_BIOS

	ROM_REGION( 0x20000, ":nichisnd:audiorom", 0 ) // z80
	ROM_LOAD( "11.ic51", 0x000000, 0x020000, CRC(9d344bad) SHA1(276c8066a2b5090edf6ba00843b7a9496c90f99f) )

	ROM_REGION( 0x400000, "blit_gfx", ROMREGION_ERASEFF ) // blitter based gfxs
	ROM_LOAD16_BYTE( "3.ic40", 0x000000, 0x080000, CRC(843c288e) SHA1(2741b9da83fd35c7472b8c67bc02313a1c5e4e25) )
	ROM_LOAD16_BYTE( "4.ic41", 0x000001, 0x080000, CRC(6920a9b8) SHA1(0a4cb9e2a0d871aed60c1293b7cac4bf79a9446c) )

	DISK_REGION( "ide:0:hdd:image" )
	DISK_IMAGE_READONLY( "nb8012", 0, SHA1(06c611f110377f5d02bbde1ab1d43d3623772b7b) )

	ROM_REGION( 0x40000, "gal", ROMREGION_ERASE00 )
	ROM_LOAD( "gal16v8b.020", 0x000000, 0x040000, CRC(ac5c9495) SHA1(1c54ecf6dedbf8c3a29207c1c91b52e2ff394d9d) )
ROM_END

ROM_START( csplayh7 )
	ROM_REGION( 0x40000, "maincpu", 0 ) // tmp68301 prg
	ROM_LOAD16_BYTE( "2.ic3", 0x000000, 0x020000, CRC(c5ce76a6) SHA1(f8878285d2318c1ec50ba98607eb3f15a7f69913) )
	ROM_LOAD16_BYTE( "1.ic2", 0x000001, 0x020000, CRC(162f8cff) SHA1(8aa185fd1daa943d0b21fdf6e692f7782bc6dac4) )

	DVD_BIOS

	ROM_REGION( 0x20000, ":nichisnd:audiorom", 0 ) // z80
	ROM_LOAD( "11.ic51", 0x000000, 0x020000, CRC(5905b199) SHA1(9155455bc21d23d439c4732549ff1143ee17b9d3) )

	ROM_REGION( 0x400000, "blit_gfx", ROMREGION_ERASEFF ) // blitter based gfxs
	ROM_LOAD16_BYTE( "3.ic40", 0x000000, 0x080000, CRC(1d67ca95) SHA1(9b45045b6fa67308bade324f91c21010aa8d121e) )
	ROM_LOAD16_BYTE( "4.ic41", 0x000001, 0x080000, CRC(b4f5f990) SHA1(88cccae04f89fef43d88f4e82b65de3de946e9af) )

	DISK_REGION( "ide:0:hdd:image" )
	DISK_IMAGE_READONLY( "csplayh7", 0, SHA1(f81e772745b0c62b17d91bd294993e49fe8da4d9) )

	ROM_REGION( 0x1000, "gal", ROMREGION_ERASE00 )
	ROM_LOAD( "mjdvd12.gal16v8b.ic8.bin", 0x000000, 0x0008c1, BAD_DUMP CRC(6a92b563)SHA1(a6c4305cf021f37845f99713427daa9394b6ec7d) )
ROM_END

ROM_START( aimode )
	ROM_REGION( 0x40000, "maincpu", 0 ) // tmp68301 prg
	ROM_LOAD16_BYTE( "2.ic3", 0x000000, 0x020000, CRC(fd7fda98) SHA1(d938391cc99d9ffdb427ec491403f81d14e09f5a) )
	ROM_LOAD16_BYTE( "1.ic2", 0x000001, 0x020000, CRC(c86765a8) SHA1(924831c07191e046beec79dd1da30c1944cfe57c) )

	DVD_BIOS

	ROM_REGION( 0x20000, ":nichisnd:audiorom", 0 ) // z80
	ROM_LOAD( "11.ic51", 0x000000, 0x020000, CRC(e6404950) SHA1(bb179c27ce65f7dc58d2aeed4710347e7953e11c) )

	ROM_REGION( 0x400000, "blit_gfx", ROMREGION_ERASEFF ) // blitter based gfxs
	ROM_LOAD16_BYTE( "3.ic40", 0x000000, 0x080000, CRC(4a9863cf) SHA1(ccf08befe773fb94fa78423ed19b6b8d255ca3a7) )
	ROM_LOAD16_BYTE( "4.ic41", 0x000001, 0x080000, CRC(893aac1a) SHA1(14dd3f07363858c2be3a9400793f720b1f5baf1a) )

	DISK_REGION( "ide:0:hdd:image" )
	DISK_IMAGE_READONLY( "nb8014", 0, SHA1(c5ad9bd66f0930e1c477126301286e38f077c164) )

	ROM_REGION( 0x40000, "gal", ROMREGION_ERASE00 )
	ROM_LOAD( "gal16v8b.020", 0x000000, 0x040000, CRC(0a32a144) SHA1(f3b4a1174adbb2f7b7500adeafa20142f6a16d08) )
ROM_END

ROM_START( fuudol )
	ROM_REGION( 0x40000, "maincpu", 0 ) // tmp68301 prg
	ROM_LOAD16_BYTE( "1.ic2", 0x000001, 0x020000, CRC(0cab2a72) SHA1(32d098bdd693a11f3cea6bbed3515c4217f40e23) )
	ROM_LOAD16_BYTE( "2.ic3", 0x000000, 0x020000, CRC(b1fa335e) SHA1(8a881c9c511fb63b00a3a7e433bae12aa9c2c262) )

	DVD_BIOS

	ROM_REGION( 0x20000, ":nichisnd:audiorom", 0 ) // z80
	ROM_LOAD( "11.ic51", 0x000000, 0x020000, CRC(f6442026) SHA1(f49ddeeeaf6fffdccea9ba73bce3ca60c07a7647) )

	ROM_REGION( 0x400000, "blit_gfx", ROMREGION_ERASEFF ) // blitter based gfxs
	ROM_LOAD16_BYTE( "3.ic40", 0x000000, 0x080000, CRC(5c9e8665) SHA1(2a1b040e5c72d4400d4b5c467c75ae99e9bb01e2) )
	ROM_LOAD16_BYTE( "4.ic41", 0x000001, 0x080000, CRC(fdd79d8f) SHA1(f8bb82afaa28affb04b83270eb407129f1c7e611) )

	DISK_REGION( "ide:0:hdd:image" )
	DISK_IMAGE_READONLY( "fuudol", 0, SHA1(fabab43543ed14da4fe7c63a2a2cc4e68936938a) )

	ROM_REGION( 0x1000, "gal", ROMREGION_ERASE00 )
	ROM_LOAD( "gal16v8b.ic8", 0x000000, 0x0008c1, CRC(30719630) SHA1(a8c7b6d0304c38691775c5af6c32fbeeefd9f9fa) )
ROM_END

ROM_START( tsuwaku )
	ROM_REGION( 0x40000, "maincpu", 0 ) // tmp68301 prg
	ROM_LOAD16_BYTE( "1.ic2",            0x000001, 0x020000, CRC(a9890007) SHA1(3cd36c653d387842289f74c3cf35435f9d2a3aca) )
	ROM_LOAD16_BYTE( "2.ic3",            0x000000, 0x020000, CRC(4577bf7b) SHA1(fed88157ded8ac72cc28cdd3b2ee36c293a6ee93) )

	DVD_BIOS

	ROM_REGION( 0x20000, ":nichisnd:audiorom", 0 ) // z80
	ROM_LOAD( "11.ic51",           0x000000, 0x020000, CRC(8451b9a9) SHA1(4e61c4b5ea7e91b53c97bd060b41466ba5005fd0) )

	ROM_REGION( 0x400000, "blit_gfx", ROMREGION_ERASEFF ) // blitter based gfxs
	ROM_LOAD16_BYTE( "3.ic40",            0x000000, 0x080000, CRC(00657ca3) SHA1(a02bb8a177f3915ddf0bf97fd69426a3a28061a5) )
	ROM_LOAD16_BYTE( "4.ic41",            0x000001, 0x080000, CRC(edf56c94) SHA1(76d95a45aced3ad8bfe8a561f355731f4f99603e) )

	DISK_REGION( "ide:0:hdd:image" )
	DISK_IMAGE_READONLY( "nb8017", 0, SHA1(6c86985574d53f990c4eec573d7fa84782cb9c4c) )

	ROM_REGION( 0x040000, "gal", ROMREGION_ERASE00 )
	ROM_LOAD( "gal16v8h.020", 0x000000, 0x040000, CRC(ac5c9495) SHA1(1c54ecf6dedbf8c3a29207c1c91b52e2ff394d9d) )
ROM_END

ROM_START( nichisel )
	ROM_REGION( 0x40000, "maincpu", 0 ) // tmp68301 prg
	ROM_LOAD16_BYTE( "1.ic2",            0x000001, 0x020000, CRC(95fb8e74) SHA1(79aa45ed1c3bd3e1a83b02afb64268efb386100e) )
	ROM_LOAD16_BYTE( "2.ic3",            0x000000, 0x020000, CRC(fb84fc3e) SHA1(6b87c3516ceec59ec96012ea6a3d2fa9670a1cb3) )

	DVD_BIOS

	ROM_REGION( 0x20000, ":nichisnd:audiorom", 0 ) // z80
	ROM_LOAD( "11.ic51",           0x000000, 0x020000, CRC(f94981fd) SHA1(84dae027f10717a084016310cd245bb4c2ee6a56) )

	ROM_REGION( 0x400000, "blit_gfx", ROMREGION_ERASEFF ) // blitter based gfxs
	ROM_LOAD16_BYTE( "3.ic40",            0x000000, 0x080000, CRC(5ab63481) SHA1(fc81fbdd1df496813fc0d80bcab6d0434b75d311) )
	ROM_LOAD16_BYTE( "4.ic41",            0x000001, 0x080000, CRC(50085861) SHA1(b8f99a66a743c9bf66ef307fe4b581586e293fe5) )

	DISK_REGION( "ide:0:hdd:image" )
	DISK_IMAGE_READONLY( "nb80sp", 0, SHA1(48eb9f8adba0ea5f59cfcbdee61c29b4af84ac97) )

	ROM_REGION( 0x040000, "gal", ROMREGION_ERASE00 )
	ROM_LOAD( "palce16v8h.020", 0x000000, 0x040000, CRC(228b98fb) SHA1(53b57a09610425a5bb9d0ffe0f68dce2d9ab3bf6) )
ROM_END

// 1995
GAME( 1995, csplayh1,   0,   csplayh5,  csplayh5, csplayh5_state,  csplayh1,                ROT0, "Sphinx/AV Japan/Astro System Japan",   "Super CD Dai8dan Mahjong Hanafuda Cosplay Tengoku (Japan)", MACHINE_NOT_WORKING )

GAME( 1998, nichidvd,   0,   csplayh5,  csplayh5, csplayh5_state,  0,                       ROT0, "Nichibutsu",                           "Nichibutsu High Rate DVD BIOS", MACHINE_IS_BIOS_ROOT )

// 1998
/* 01 */ GAME( 1998, mjgalpri,  nichidvd,   csplayh5, csplayh5,  csplayh5_state,  mjgalpri,        ROT0, "Nichibutsu/Just&Just", "Mahjong Gal-pri - World Gal-con Grandprix (Japan)", MACHINE_NOT_WORKING )
// 02 : Sengoku Mahjong Kurenai Otome-tai : Nichibutsu/Just&Just
/* 03 */ GAME( 1998, junai,     nichidvd,   csplayh5,  csplayh5, csplayh5_state,  junai,           ROT0, "Nichibutsu/eic",   "Junai - Manatsu no First Kiss (Japan)", MACHINE_NOT_WORKING )
/* 04 */ GAME( 1998, csplayh5,  nichidvd,   csplayh5,  csplayh5, csplayh5_state,  csplayh5,        ROT0, "Nichibutsu",       "Mahjong Hanafuda Cosplay Tengoku 5 (Japan)", MACHINE_NOT_WORKING )
/* 05 */ GAME( 1998, junai2,    nichidvd,   csplayh5,  csplayh5, csplayh5_state,  junai2,          ROT0, "Nichibutsu/eic",   "Junai 2 - White Love Story (Japan)", MACHINE_NOT_WORKING )
/* 06 */ GAME( 1998, mogitate,  nichidvd,   csplayh5,  csplayh5, csplayh5_state,  mogitate,        ROT0, "Nichibutsu/Just&Just/NVS/Astro System/AV Japan", "Mahjong Mogitate", MACHINE_NOT_WORKING )

// 1999
/* 07 */ GAME( 1999, mjmania,   nichidvd,   csplayh5,  csplayh5, csplayh5_state,  mjmania,         ROT0, "Sphinx/Just&Just", "Mahjong Mania - Kairakukan e Youkoso (Japan)", MACHINE_NOT_WORKING )
/* 08 */ GAME( 1999, renaimj,   nichidvd,   csplayh5,  csplayh5, csplayh5_state,  renaimj,         ROT0, "Nichibutsu/eic",   "Renai Mahjong Idol Gakuen (Japan)", MACHINE_NOT_WORKING )
/* 09 */ GAME( 1999, bikiniko,  nichidvd,   csplayh5,  csplayh5, csplayh5_state,  bikiniko,        ROT0, "Nichibutsu/eic",   "BiKiNikko - Okinawa de Ippai Shichaimashita (Japan)", MACHINE_NOT_WORKING )
/* 10 */ GAME( 1999, csplayh6,  nichidvd,   csplayh5,  csplayh5, csplayh5_state,  csplayh6,        ROT0, "Nichibutsu/eic",   "Mahjong Hanafuda Cosplay Tengoku 6 - Junai-hen (Japan)", MACHINE_NOT_WORKING )
/* 11 */ GAME( 1999, thenanpa,  nichidvd,   csplayh5,  csplayh5, csplayh5_state,  thenanpa,        ROT0, "Nichibutsu/Love Factory/eic", "The Nanpa (Japan)", MACHINE_NOT_WORKING )
/* 12 */ GAME( 1999, pokoachu,  nichidvd,   csplayh5,  csplayh5, csplayh5_state,  pokoachu,        ROT0, "Nichibutsu/eic", "PokoaPoka Onsen de CHU - Bijin 3 Shimai ni Kiotsukete! (Japan)", MACHINE_NOT_WORKING )
/* 13 */ GAME( 1999, csplayh7,  nichidvd,   csplayh5,  csplayh5, csplayh5_state,  csplayh7,        ROT0, "Nichibutsu/eic", "Cosplay Tengoku 7 - Super Kogal Grandprix (Japan)", MACHINE_NOT_WORKING )
/* 14 */ GAME( 1999, aimode,    nichidvd,   csplayh5,  csplayh5, csplayh5_state,  aimode,          ROT0, "Nichibutsu/eic", "Ai-mode - Pet Shiiku", MACHINE_NOT_WORKING )

// 2000
/* 15 */ GAME( 2000, fuudol,    nichidvd,   csplayh5,  csplayh5, csplayh5_state,  fuudol,          ROT0, "Nichibutsu/eic", "Fuudol (Japan)", MACHINE_NOT_WORKING )
// 16 : Nurete Mitaino... - Net Idol Hen : Nichibutsu/Love Factory
/* 17 */ GAME( 2000, tsuwaku,    nichidvd,   csplayh5,  csplayh5, csplayh5_state, tsuwaku,         ROT0, "Nichibutsu/Love Factory/Just&Just", "Tsuugakuro no Yuuwaku (Japan)", MACHINE_NOT_WORKING )
// 18 : Torarechattano - AV Kantoku Hen : Nichibutsu/Love Factory/M Friend
/* sp */ GAME( 2000, nichisel,    nichidvd,  csplayh5,  csplayh5, csplayh5_state, nichisel,        ROT0, "Nichibutsu", "DVD Select (Japan)", MACHINE_NOT_WORKING )

// 2001
// 19 : Konnano Hajimete! : Nichibutsu/Love Factory
// 20 : Uwasa no Deaikei Site : Nichibutsu/Love Factory/eic
