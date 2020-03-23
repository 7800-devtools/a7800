// license:BSD-3-Clause
// copyright-holders:Samuele Zannoli
/*
  ATV Track
  (c)2002 Gaelco

ATV Track
Gaelco 2002

PCB Layout

GAELCO
REF. 020419
 |--------------------------------------------------------------|
 |                                                              |
 |   SW3                                               EPC1PC8  |
 |                                         K4S643232            |
 |    LC245A                                         7LB176    |-|
 |                        FLASH.IC14  FLASH.IC19     7LB176    | |
 |       |-----|                                     7LB176    | |DB9
 |       | SH4 |                                     7LB176    |-|
 |       |     |          FLASH.IC15  FLASH.IC20                |
 |       |-----|                                                |
 |                                                |----------|  |
|-|                            K4S643232          |ALTERA    |  |
| |  L4955                                        |FLEX      |  |
| |      |-----|               K4S643232          |EPF10K50  |  |
| |CN1   | SH4 |                                  |EQC240-3  |  |
| |      |     |                                  |----------|  |
| |      |-----|                                                |
|-|                                                             |
 |                                                              |
 |      33MHz                       K4S643232     |----------|  |
 |                                  K4S643232     | GFX      |  |
 |             LED                                |          | |-|
 |             LED                                |          | | |
 |                                  K4S643232     |          | | |DB9
 |                                  K4S643232     |----------| |-|
 | TL074C   TL074C                                     385-1    |
 |     TDA1387   TDA1387                          14.31818MHz   |
 |                                                              |
 |--------------------------------------------------------------|
Notes:
      SH4       - Hitachi HD6417750S SH4 CPU (BGA)
      K4S643232 - Samsung K4S643232E-TC70 64M x 32-bit SDRAM (TSSOP86)
      GFX       - NEC PowerVR Neon 250
      FLASH.IC* - Samsung K9F2808U0B 128MBit (16M + 512k Spare x 8-bit) FlashROM (TSOP48)
      EPF10K50  - Altera Flex EPF10K50EQC240-3 FPGA (QFP240)
      EPC1PC8   - Altera EPC1PC8 FPGA Configuration Device (DIP8)
      TL074C    - Texas Instruments TL074C Low Noise Quad JFet Operational Amplifier (SOIC14)
      TDA1387   - Philips TDA1387 Stereo Continuous Calibration DAC (SOIC8)
      L4955     - ST Microelectronics L4955 low-power, quad channel, 8-bit buffered voltage output DAC and amplifier
      7LB176    - Texas Instruments 7LB176 Differential Bus Tranceiver (SOIC8)
      385-1     - National LM385 Adjustable Micropower Voltage Reference Diode (SOIC8)
      CN1       - Multi-pin connector for filter board (input, video, power & controls connectors etc)
      DB9       - Probably used for cabinet linking
      SW3       - Push button switch

*/

/*

notes from DEMUL team

Smashing Drive needs a working SH4 MMU emulation, ATV Track does not.

Audio - is a simple buffered DAC.
frequency is 32kHz
data written by CPU to buffer have such meaning:
offs 0 - s16 bass channel 0
offs 2 - s16 bass channel 1
offs 4 - s16 left channel
offs 6 - s16 right channel
and so on

buffer is 2x32bytes
then it becomes (I suppose half) empty - SH4 IRL5 IRQ generated


"control registers" (Smashing Drive)
0 - read - various statuses, returning -1 is OK
write - enable slave CPU, gpu, etc most of bits is unclear
4 - w - RS422/485 communication port (for cabinet linking)

SH4 XTAL is 33MHz, SH4 MD0-2 pins is 001 or 011 (CPU core clk = XTAL*6, preipheral clk = XTAL, bus clk is XTAL or XTAL*2)

TODO:
    devicify NAND
    somehow hook PVR2 renderer here
    add sound

*/

#include "emu.h"
#include "cpu/sh4/sh4.h"
#include "debugger.h"
#include "screen.h"


//#define SPECIALMODE 1 // Alternate code path

class atvtrack_state : public driver_device
{
public:
	atvtrack_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_subcpu(*this, "subcpu") { }

	DECLARE_READ64_MEMBER(control_r);
	DECLARE_WRITE64_MEMBER(control_w);
	DECLARE_READ64_MEMBER(nand_data_r);
	DECLARE_WRITE64_MEMBER(nand_data_w);
	DECLARE_WRITE64_MEMBER(nand_cmd_w);
	DECLARE_WRITE64_MEMBER(nand_addr_w);
	DECLARE_READ64_MEMBER(ioport_r);
	DECLARE_WRITE64_MEMBER(ioport_w);
	DECLARE_READ32_MEMBER(gpu_r);
	DECLARE_WRITE32_MEMBER(gpu_w);
	virtual void machine_start() override;
	virtual void machine_reset() override;
	virtual void video_start() override;
	uint32_t screen_update_atvtrack(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);
	inline uint32_t decode64_32(offs_t offset64, uint64_t data, uint64_t mem_mask, offs_t &offset32);
	void logbinary(uint32_t data,int high,int low);

	memory_region *m_nandregion;
	int m_nandcommand[4], m_nandoffset[4], m_nandaddressstep, m_nandaddress[4];
	uint32_t m_area1_data[4];

	required_device<sh4_device> m_maincpu;
	required_device<sh4_device> m_subcpu;

	u16 gpu_irq_pending;
	u16 gpu_irq_mask;
	void gpu_irq_test();
	void gpu_irq_set(int);
protected:
	bool m_slaverun;
};


class smashdrv_state : public atvtrack_state
{
public:
	smashdrv_state(const machine_config &mconfig, device_type type, const char *tag)
		: atvtrack_state(mconfig, type, tag) { }

	virtual void machine_start() override;
	virtual void machine_reset() override;
};

void atvtrack_state::logbinary(uint32_t data,int high=31,int low=0)
{
	uint32_t s;
	int z;

	s=1 << high;
	for (z = high;z >= low;z--) {
		if (data & s)
			logerror("1");
		else
			logerror("0");
		s=s >> 1;
	}
}

inline uint32_t atvtrack_state::decode64_32(offs_t offset64, uint64_t data, uint64_t mem_mask, offs_t &offset32)
{
	if (ACCESSING_BITS_0_31) {
		offset32 = offset64 << 1;
		return (uint32_t)data;
	}
	if (ACCESSING_BITS_32_63) {
		offset32 = (offset64 << 1)+1;
		return (uint32_t)(data >> 32);
	}
	logerror("Wrong word size in external access\n");
	//machine().debug_break();
	return 0;
}

READ64_MEMBER(atvtrack_state::control_r)
{
	uint32_t addr;

	addr = 0;
	decode64_32(offset, 0, mem_mask, addr);
	if (addr == (0x00020000-0x00020000)/4)
		return -1;
	else if (addr == (0x00020004-0x00020000)/4)
		return -1;
	return -1;
}

WRITE64_MEMBER(atvtrack_state::control_w)
{
	uint32_t addr, dat; //, old;

	addr = 0;
	dat = decode64_32(offset, data, mem_mask, addr);
//  old = m_area1_data[addr];
	m_area1_data[addr] = dat;
	if (addr == (0x00020000-0x00020000)/4) {
		if ((data & 4) && m_slaverun)
			m_subcpu->set_input_line(INPUT_LINE_RESET, CLEAR_LINE);
		else
			m_subcpu->set_input_line(INPUT_LINE_RESET, ASSERT_LINE);
	}
//  logerror("Write %08x at %08x ",dat, 0x20000+addr*4+0);
//  logbinary(dat);
//  logerror("\n");
}

READ64_MEMBER(atvtrack_state::nand_data_r)
{
	u32 dat = 0;
	for (int c = 3; c >= 0; c--) {
		if (m_nandcommand[c] <= 0x50) {
			u32 addr = m_nandaddress[c] + m_nandoffset[c];
			dat = (dat << 8) | m_nandregion->as_u8(addr + c);
			m_nandoffset[c] += 4;
		}
		else
			dat = (dat << 8) | 0xc0;
	}
	return dat;
}

WRITE64_MEMBER(atvtrack_state::nand_data_w)
{
	for (int c = 0; c < 4; c++) {
		if (m_nandcommand[c] == 0x80) {
			u8 *ptr = m_nandregion->base();
			u32 addr = m_nandaddress[c] + m_nandoffset[c] + c;
			ptr[addr] &= data & 0xff;
			m_nandoffset[c] += 4;
		}
		data >>= 8;
	}
}

WRITE64_MEMBER(atvtrack_state::nand_cmd_w)
{
	m_nandaddressstep = 0;
	for (int c = 0;c < 4;c++) {
		m_nandcommand[c] = data & 0xff;
		if (m_nandcommand[c] == 0x00) {
			m_nandoffset[c] = 0;
		} else if (m_nandcommand[c] == 0x01) {
			m_nandoffset[c] = 256*4;
		} else if (m_nandcommand[c] == 0x50) {
			m_nandoffset[c] = 512*4;
		} else if (m_nandcommand[c] == 0x90) {
		} else if (m_nandcommand[c] == 0xff) {
		} else if (m_nandcommand[c] == 0x80) {
		} else if (m_nandcommand[c] == 0x60) {
			m_nandaddressstep = 1;
			m_nandaddress[c] = 0;
		} else if (m_nandcommand[c] == 0x70) {
		} else if (m_nandcommand[c] == 0x10) {
		} else if (m_nandcommand[c] == 0xd0) {
			u8 *ptr = m_nandregion->base();
			u32 addr = m_nandaddress[c] + c;
			for (int i = 0; i < 32 * 528; i++)
				ptr[addr + i * 4] = 0xff;
		} else {
			m_nandcommand[c] = 0xff;
		}
		data=data >> 8;
	}
}

WRITE64_MEMBER(atvtrack_state::nand_addr_w)
{
	for (int c = 0;c < 4;c++) {
		if (m_nandaddressstep == 0) {
			m_nandaddress[c] = (data & 0xff)*4;
		} else if (m_nandaddressstep == 1) {
			m_nandaddress[c] = m_nandaddress[c]+(data & 0xff)*0x840;
		} else if (m_nandaddressstep == 2) {
			m_nandaddress[c] = m_nandaddress[c]+(data & 0xff)*0x84000;
		}
		data = data >> 8;
	}
	m_nandaddressstep++;
}

void atvtrack_state::gpu_irq_test()
{
	if (gpu_irq_pending & ~gpu_irq_mask)
		m_subcpu->sh4_set_irln_input(14);
	else
		m_subcpu->sh4_set_irln_input(15);
}

void atvtrack_state::gpu_irq_set(int bit)
{
	gpu_irq_pending |= 1 << bit;
	gpu_irq_test();
}

READ32_MEMBER(atvtrack_state::gpu_r)
{
	switch (offset)
	{
	case 0x70/4:
		return gpu_irq_pending;
	case 0x74/4:
		return gpu_irq_mask;
	default:
		logerror("GPU: unhandled reg read @ %04X\n", offset * 4);
		return 0;
	}
}

WRITE32_MEMBER(atvtrack_state::gpu_w)
{
	switch (offset)
	{
	case 0x00/4:
		// not really required, game code will go even if GPU CPU shows no signs of live
		if (data)   // internal CPU start ?
			m_subcpu->space(AS_PROGRAM).write_byte(0x18001350, 1); // simulate GPUs internal CPU reply to skip busy loop
		break;
	case 0x70/4:
		gpu_irq_pending &= ~data;
		gpu_irq_test();
		break;
	case 0x74/4:
		gpu_irq_mask = data;
		gpu_irq_test();
		break;
	case 0xb0/4:
		if (data == 0x0001)
		{
			// GPU render happens here
			gpu_irq_set(7);
		}
		// not really required, game code will go even if GPU CPU shows no signs of live
		if (data == 0x8001)
			m_subcpu->space(AS_PROGRAM).write_byte(0x18814804, 1); // simulate GPUs internal CPU reply to skip busy loop
		break;
	default:
		logerror("GPU: unhandled reg write @ %04X data %08X\n", offset * 4, data);
		break;
	}
}

READ64_MEMBER(atvtrack_state::ioport_r)
{
	if (offset == SH4_IOPORT_16/8) {
#ifndef SPECIALMODE
		return -1; // normal
#else
		return 0; // testing
#endif
	}
	return 0;
}

WRITE64_MEMBER(atvtrack_state::ioport_w)
{
	// SH4 GPIO port A used in this way:
	// bits 15-11  O - port select: F002 (In)  E802 (In)  F800 (Out)        7800 (Out)
	//                              JP10 conn  JP8 conn   ADC/DAC control   System Control and/or diagnostics
	// bit  10    IO - data bit                 *1        GPO JP3           motion enabled indicator ?
	// bit  9     IO - data bit     ADC data    *1        GPO JP3           sound (AMP) enable ? (ATV - set/res then music starts/ends, SD - always set)
	// bit  8     IO - data bit     Coin        *1        GPO JP3            \ slave CPU start/stop/reset related
	// bit  7     IO - data bit                 *1        always 1           / set to 30 during slave CPU test, then to 10
	// bit  6     IO - data bit     Test                  ADC CS, ~DAC CS    unused
	// bit  5     IO - data bit     Down                  ADC CLK, ~DAC CLK  unused
	// bit  4     IO - data bit     Service               DAC data JP6       lamp
	// bit  3     IO - data bit     Up                    ADC data JP5       coin counter
	// bit  2     I  - unk, (SD: 1 = FPGA ready after config)
	// bit  1      O - data bits operation direction: 0 - output, 1 - input (SD: FPGA config data)
	// bit  0      O - unk (SD: FPGA config clock)
	// ADC and DAC is 4-channel SPI devices
	// Switch inputs is active low, *1 - game specific inputs
	// All above IO multiplexing, ADC and DAC implemented in FPGA

#ifdef SPECIALMODE
	uint64_t d;
	static int cnt=0;
	sh4_device_dma dm;
#endif

	if (offset == SH4_IOPORT_16/8) {
		if ((data & 0xf000) == 0x7000) {
			if (data & 0x0100)
				m_slaverun = true;
		}
//      logerror("SH4 16bit i/o port write ");
//      logbinary((uint32_t)data,15,0);
//      logerror("\n");
	}
#ifdef SPECIALMODE
	if (offset == SH4_IOPORT_DMA/8) {
		dm.buffer = &d;
		dm.channel = data & 0xffff;
		dm.length = 1;
		dm.size = 4;
		if (cnt == 0)
			d=0x12340153;
		else
			d=0x11223344;
		if (cnt == 0)
			sh4_dma_data(cpu,&dm);
		else
			sh4_dma_data(cpu,&dm);
		cnt++;
	}
#endif
}

void atvtrack_state::video_start()
{
}

uint32_t atvtrack_state::screen_update_atvtrack(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
	return 0;
}

void get_altera10ke_eab(u8* dst, u8 *pof, int eab)
{
	// extract Altera FLEX 10KE 4kbit Embedded Array Block (EAB)
	u32 startbit = 0x45b * 8 + 1 + (0x25e6 * 8) * eab;  // base ofsset actually depends on POF header size, however this EPC1PC8 dump havent it (bad dumper software?)

	for (u32 bit = 0; bit < 4096; bit++)
	{
		u32 tbit = BITSWAP16(bit, 15, 14, 13, 12,
			9, 8, 7, 6, 5, 4, 3,
			11, 10,
			2, 1, 0);
		tbit ^= 0x0fc0;
		u32 pofbit = startbit + (tbit & 0x1f) + (((tbit >> 5) & 1) * 63) + (((tbit >> 6) & 0x3f) * 0x4D * 8);
		dst[bit / 8] &= ~(1 << (bit & 7));
		dst[bit / 8] |= ((pof[pofbit / 8] >> (pofbit & 7)) & 1) << (bit & 7);
	}
}

void atvtrack_state::machine_start()
{
	m_nandaddressstep = 0;
	m_nandregion = memregion("nand");
}

void atvtrack_state::machine_reset()
{
	std::vector<u8> tdata(1024);
	u8 *pof = memregion("fpga")->base();

	// first 2 Altera's EABs is 2xSH4s shared RAM, its initial content is boot loader
	// extract EABs 0 and 1 from POF
	get_altera10ke_eab(&tdata[0], pof, 0);
	get_altera10ke_eab(&tdata[512], pof, 1);

	// deshuffle data bits and put it to shared RAM
	u8 *dst = (u8*)(m_maincpu->space(AS_PROGRAM).get_write_ptr(0));
	for (u32 i = 0; i < 256; i++)
	{
		u16 lword = tdata[i * 2 + 512] | (tdata[i * 2 + 513] << 8);
		u16 hword = tdata[i * 2] | (tdata[i * 2 + 1] << 8);
		lword = BITSWAP16(lword, 7, 9, 0, 10, 3, 11, 4, 12, 2, 15, 1, 13, 6, 8, 5, 14);
		hword = BITSWAP16(hword, 5, 10, 7, 9, 6, 13, 3, 15, 2, 11, 1, 8, 0, 12, 4, 14);
		dst[i * 4 + 0] = lword & 0xff;
		dst[i * 4 + 1] = lword >> 8;
		dst[i * 4 + 2] = hword & 0xff;
		dst[i * 4 + 3] = hword >> 8;
	}

	m_subcpu->set_input_line(INPUT_LINE_RESET, ASSERT_LINE);
	gpu_irq_pending = 0;
	gpu_irq_mask = 0xFFFF;
}


void smashdrv_state::machine_start()
{
}

void smashdrv_state::machine_reset()
{
	m_slaverun = false;
	m_subcpu->set_input_line(INPUT_LINE_RESET, ASSERT_LINE);
	gpu_irq_pending = 0;
	gpu_irq_mask = 0xFFFF;
}

// ATV Track

static ADDRESS_MAP_START( atvtrack_main_map, AS_PROGRAM, 64, atvtrack_state )
	AM_RANGE(0x00000000, 0x000003ff) AM_RAM AM_SHARE("sharedmem")
	AM_RANGE(0x00020000, 0x00020007) AM_READWRITE(control_r, control_w) // control registers
//  AM_RANGE(0x00020040, 0x0002007f) // audio DAC buffer
	AM_RANGE(0x14000000, 0x14000007) AM_READWRITE(nand_data_r, nand_data_w)
	AM_RANGE(0x14100000, 0x14100007) AM_WRITE(nand_cmd_w)
	AM_RANGE(0x14200000, 0x14200007) AM_WRITE(nand_addr_w)
	AM_RANGE(0x0c000000, 0x0c7fffff) AM_RAM
ADDRESS_MAP_END

static ADDRESS_MAP_START( atvtrack_main_port, AS_IO, 64, atvtrack_state )
	AM_RANGE(0x00, 0x1f) AM_READWRITE(ioport_r, ioport_w)
ADDRESS_MAP_END

// Smashing Drive

static ADDRESS_MAP_START( smashdrv_main_map, AS_PROGRAM, 64, smashdrv_state )
	AM_RANGE(0x00000000, 0x03ffffff) AM_ROM
	AM_RANGE(0x0c000000, 0x0c7fffff) AM_RAM
	AM_RANGE(0x10000000, 0x100003ff) AM_RAM AM_SHARE("sharedmem")
	AM_RANGE(0x10000400, 0x10000407) AM_READWRITE(control_r, control_w) // control registers

// 0x10000400 - 0x1000043F control registers
// 0x10000440 - 0x1000047F Audio DAC buffer
	AM_RANGE(0x14000000, 0x143fffff) AM_ROM AM_REGION("data", 0)
ADDRESS_MAP_END

static ADDRESS_MAP_START( smashdrv_main_port, AS_IO, 64, smashdrv_state )
	AM_RANGE(0x00, 0x1f) AM_READWRITE(ioport_r, ioport_w)
ADDRESS_MAP_END

// Sub CPU (same for both games)

static ADDRESS_MAP_START( atvtrack_sub_map, AS_PROGRAM, 64, atvtrack_state )
	AM_RANGE(0x00000000, 0x000003ff) AM_RAM AM_SHARE("sharedmem")
	AM_RANGE(0x0c000000, 0x0cffffff) AM_RAM
	AM_RANGE(0x14000000, 0x14003fff) AM_READWRITE32(gpu_r, gpu_w, 0xffffffffffffffffU)
// 0x14004xxx GPU PCI CONFIG registers
	AM_RANGE(0x18000000, 0x19ffffff) AM_RAM
// 0x18000000 - 0x19FFFFFF GPU RAM (32MB)
ADDRESS_MAP_END

static ADDRESS_MAP_START( atvtrack_sub_port, AS_IO, 64, atvtrack_state )
ADDRESS_MAP_END


static INPUT_PORTS_START( atvtrack )
INPUT_PORTS_END

#define ATV_CPU_CLOCK XTAL_33MHz*6

static MACHINE_CONFIG_START( atvtrack )
	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", SH4LE, ATV_CPU_CLOCK)
	MCFG_SH4_MD0(1)
	MCFG_SH4_MD1(1)
	MCFG_SH4_MD2(0)
	MCFG_SH4_MD3(0)
	MCFG_SH4_MD4(0)
	MCFG_SH4_MD5(1)
	MCFG_SH4_MD6(0)
	MCFG_SH4_MD7(1)
	MCFG_SH4_MD8(0)
	MCFG_SH4_CLOCK(ATV_CPU_CLOCK)
	MCFG_CPU_PROGRAM_MAP(atvtrack_main_map)
	MCFG_CPU_IO_MAP(atvtrack_main_port)

	MCFG_CPU_ADD("subcpu", SH4LE, ATV_CPU_CLOCK)
	MCFG_SH4_MD0(1)
	MCFG_SH4_MD1(1)
	MCFG_SH4_MD2(0)
	MCFG_SH4_MD3(0)
	MCFG_SH4_MD4(0)
	MCFG_SH4_MD5(1)
	MCFG_SH4_MD6(0)
	MCFG_SH4_MD7(1)
	MCFG_SH4_MD8(0)
	MCFG_SH4_CLOCK(ATV_CPU_CLOCK)
	MCFG_CPU_PROGRAM_MAP(atvtrack_sub_map)
	MCFG_CPU_IO_MAP(atvtrack_sub_port)

	/* video hardware */
	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(60)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(2500))  /* not accurate */
	MCFG_SCREEN_SIZE(640, 480)
	MCFG_SCREEN_VISIBLE_AREA(0, 640-1, 0, 480-1)
	MCFG_SCREEN_UPDATE_DRIVER(atvtrack_state, screen_update_atvtrack)

	MCFG_PALETTE_ADD("palette", 0x1000)
MACHINE_CONFIG_END

static MACHINE_CONFIG_DERIVED( smashdrv, atvtrack )
	MCFG_CPU_MODIFY("maincpu")
	MCFG_CPU_PROGRAM_MAP(smashdrv_main_map)
	MCFG_CPU_IO_MAP(smashdrv_main_port)

MACHINE_CONFIG_END


ROM_START( atvtrack )
	ROM_REGION( 0x4200000, "nand", ROMREGION_ERASEFF) // NAND roms, contain additional data hence the sizes
	ROM_LOAD32_BYTE("15.bin", 0x0000000, 0x1080000, CRC(84eaede7) SHA1(6e6230165c3bb35e49c660dfd0d07c132ed89e6a) )
	ROM_LOAD32_BYTE("20.bin", 0x0000001, 0x1080000, CRC(649dc331) SHA1(0cac2d0c15dd564c7fdebdf4365422958f453d63) )
	ROM_LOAD32_BYTE("14.bin", 0x0000002, 0x1080000, CRC(67983453) SHA1(05389a0ffc1a1bae9bac16a53a97d78b6eccc626) )
	ROM_LOAD32_BYTE("19.bin", 0x0000003, 0x1080000, CRC(9fc5c579) SHA1(8829329ef229564952aea2108ef1750dc226cbac) )

	ROM_REGION( 0x20000, "fpga", ROMREGION_ERASEFF)
	ROM_LOAD("epc1pc8.ic23", 0x0000000, 0x1ff01, CRC(752444c7) SHA1(c77e8fcfcbe15b53eda25553763bdac45f0ef7df) ) // contains configuration data for the fpga
ROM_END

ROM_START( atvtracka )
	ROM_REGION( 0x4200000, "nand", ROMREGION_ERASEFF) // NAND roms, contain additional data hence the sizes
	ROM_LOAD32_BYTE("k9f2808u0b.ic15", 0x0000000, 0x1080000, CRC(10730001) SHA1(48c685a6ff7135abd074dc7fb7d10834c44da58f) )
	ROM_LOAD32_BYTE("k9f2808u0b.ic20", 0x0000001, 0x1080000, CRC(b0c34433) SHA1(852c79bb3d7082cd2c056140071ae7d71679ec1d) )
	ROM_LOAD32_BYTE("k9f2808u0b.ic14", 0x0000002, 0x1080000, CRC(02a12085) SHA1(acb112c9c7b29d92610465fb92268ce787ca06f4) )
	ROM_LOAD32_BYTE("k9f2808u0b.ic19", 0x0000003, 0x1080000, CRC(856c1e6a) SHA1(a6b2839120d61811c36cc6b4095de9cefceb394b) )

	ROM_REGION( 0x20000, "fpga", ROMREGION_ERASEFF)
	ROM_LOAD("epc1pc8.ic23", 0x0000000, 0x1ff01, CRC(752444c7) SHA1(c77e8fcfcbe15b53eda25553763bdac45f0ef7df) ) // contains configuration data for the fpga
ROM_END

/*

Smashing Drive
Gaelco 2000

PCB Layout
----------

REF 010131
|----------------------------------------------|
|                                              |
|                              K4S643232C      |
|        |------|SDRB.IC14                     |
|        |SH4   |                              |-|
|        |      |                              | |DB9
|        |      |SDRA.IC15  SDRC.IC20  PRG.IC23|-|
|        |------|             |----------|     |
|                             |ALTERA    |     |
|                             |FLEX0K50  |     |
|        |------|  K4S643232C |EPF10K50  |     |
|        |SH4   |  K4S643232C |EQC240-3  |     |
|        |      |             |          |     |
|        |      |             |          |     |
|        |------|             |----------|     |
|                                              |
|         33MHz                 |--------|     |
|                    K4S643232C |NEC     |     |
|                    K4S643232C |POWERVR |     |
|                               |250     |     |
|                    K4S643232C |        |     |-|
|                    K4S643232C |        |     | |DB9
|                               |--------|     |-|
|                                              |
|  TL074C   TL074C                14.31818MHz  |
|  TDA1543  TDA1543                            |
|----------------------------------------------|

*/

ROM_START( smashdrv )
	ROM_REGION64_LE( 0x0400000, "data", ROMREGION_ERASEFF)
	ROM_LOAD("prg.ic23", 0x0000000, 0x0400000, CRC(5cc6d3ac) SHA1(0c8426774212d891796b59c95b8c70f64db5b67a) )

	ROM_REGION( 0x4000000, "maincpu", ROMREGION_ERASEFF)
	ROM_LOAD32_WORD("sdra.ic15",    0x00000000, 0x01000000, CRC(cf702287) SHA1(84cd83c339831deff15fe5fcc353e0b596667500) )
	ROM_LOAD32_WORD("sdrb.ic14",    0x00000002, 0x01000000, CRC(39b76f0e) SHA1(529943b6075925e5f72c6e966796e04b2c33686c) )
	ROM_LOAD32_WORD("sdrc.ic20",    0x02000000, 0x01000000, CRC(c9021dd7) SHA1(1d08aab433614810af858a0fc5d7f03c7b782237) )
	// ic21 unpopulated
ROM_END

GAME( 2002, atvtrack,  0,          atvtrack,    atvtrack, atvtrack_state,   0, ROT0, "Gaelco", "ATV Track (set 1)", MACHINE_NOT_WORKING | MACHINE_NO_SOUND )
GAME( 2002, atvtracka, atvtrack,   atvtrack,    atvtrack, atvtrack_state,   0, ROT0, "Gaelco", "ATV Track (set 2)", MACHINE_NOT_WORKING | MACHINE_NO_SOUND )

// almost identical PCB, FlashROM mapping and master registers addresses different
GAME( 2000, smashdrv, 0,           smashdrv,    atvtrack, smashdrv_state,   0, ROT0, "Gaelco", "Smashing Drive (UK)", MACHINE_NOT_WORKING | MACHINE_NO_SOUND )
