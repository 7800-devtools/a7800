// license:BSD-3-Clause
// copyright-holders: R. Belmont
/****************************************************************************

    micro20.cpp
    GMX Micro 20 single-board computer

    68020 + 68881 FPU

    800a5e = end of initial 68020 torture test
****************************************************************************/

#include "emu.h"
#include "cpu/m68000/m68000.h"
#include "machine/mc68681.h"
#include "machine/msm58321.h"
#include "machine/wd_fdc.h"
#include "machine/68230pit.h"
#include "bus/rs232/rs232.h"
#include "softlist.h"

#define MAINCPU_TAG "maincpu"
#define DUART_A_TAG "duarta"
#define DUART_B_TAG "duartb"
#define RTC_TAG     "rtc"
#define FDC_TAG     "fdc"
#define PIT_TAG     "pit"

class micro20_state : public driver_device
{
public:
	micro20_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_maincpu(*this, MAINCPU_TAG),
		m_rom(*this, "bootrom"),
		m_mainram(*this, "mainram"),
		m_pit(*this, PIT_TAG),
		m_rtc(*this, RTC_TAG)
	{
	}

	required_device<m68020_device> m_maincpu;
	required_memory_region m_rom;
	required_shared_ptr<uint32_t> m_mainram;
	required_device<pit68230_device> m_pit;
	required_device<msm58321_device> m_rtc;

	virtual void machine_start() override;
	virtual void machine_reset() override;

	DECLARE_WRITE_LINE_MEMBER(m68k_reset_callback);
	DECLARE_READ32_MEMBER(buserror_r);

	TIMER_DEVICE_CALLBACK_MEMBER(micro20_timer);
	DECLARE_WRITE_LINE_MEMBER(h4_w);
	DECLARE_WRITE8_MEMBER(portb_w);
	DECLARE_WRITE8_MEMBER(portc_w);

	DECLARE_WRITE_LINE_MEMBER(timerirq_w)
	{
		m_maincpu->set_input_line(M68K_IRQ_4, state);
	}

private:
	u8 m_tin;
	u8 m_h4;
};

void micro20_state::machine_start()
{
}

void micro20_state::machine_reset()
{
	u32 *pROM = (uint32_t *)m_rom->base();
	u32 *pRAM = (uint32_t *)m_mainram.target();

	pRAM[0] = pROM[2];
	pRAM[1] = pROM[3];
	m_maincpu->reset();

	m_maincpu->set_reset_callback(write_line_delegate(FUNC(micro20_state::m68k_reset_callback),this));

	m_tin = 0;
}

TIMER_DEVICE_CALLBACK_MEMBER(micro20_state::micro20_timer)
{
	m_pit->update_tin(m_tin ? ASSERT_LINE : CLEAR_LINE);
	if ((!m_h4) && (m_tin))
	{
		m_maincpu->set_input_line(M68K_IRQ_6, HOLD_LINE);
	}
	m_tin ^= 1;
}

WRITE_LINE_MEMBER(micro20_state::h4_w)
{
	printf("h4_w: %d\n", state);
	m_h4 = state ^ 1;
}

WRITE_LINE_MEMBER(micro20_state::m68k_reset_callback)
{
	// startup test explicitly checks if the m68k RESET opcode resets the 68230
	m_pit->reset();
}

WRITE8_MEMBER(micro20_state::portb_w)
{
	m_rtc->d0_w((data & 1) ? ASSERT_LINE : CLEAR_LINE);
	m_rtc->d1_w((data & 2) ? ASSERT_LINE : CLEAR_LINE);
	m_rtc->d2_w((data & 4) ? ASSERT_LINE : CLEAR_LINE);
	m_rtc->d3_w((data & 8) ? ASSERT_LINE : CLEAR_LINE);
}

WRITE8_MEMBER(micro20_state::portc_w)
{
	// MSM58321 CS1 and CS2 are tied to /RST, inverted RESET.
	// So they're always high when the system is not reset.
	m_rtc->cs1_w(ASSERT_LINE);
	m_rtc->cs2_w(ASSERT_LINE);
	m_rtc->stop_w((data & 1) ? ASSERT_LINE : CLEAR_LINE);
	m_rtc->write_w((data & 2) ? ASSERT_LINE : CLEAR_LINE);
	m_rtc->read_w((data & 0x10) ? ASSERT_LINE : CLEAR_LINE);
	m_rtc->address_write_w((data & 0x40) ? ASSERT_LINE : CLEAR_LINE);
	m_rtc->test_w((data & 0x80) ? ASSERT_LINE : CLEAR_LINE);
}

READ32_MEMBER(micro20_state::buserror_r)
{
	m_maincpu->set_input_line(M68K_LINE_BUSERROR, ASSERT_LINE);
	m_maincpu->set_input_line(M68K_LINE_BUSERROR, CLEAR_LINE);
	return 0xffff;
}
/***************************************************************************
    ADDRESS MAPS
***************************************************************************/

static ADDRESS_MAP_START(micro20_map, AS_PROGRAM, 32, micro20_state )
	AM_RANGE(0x00000000, 0x001fffff) AM_RAM AM_SHARE("mainram")
	AM_RANGE(0x00200000, 0x002fffff) AM_READ(buserror_r)
	AM_RANGE(0x00800000, 0x0083ffff) AM_ROM AM_REGION("bootrom", 0)
	AM_RANGE(0xffff8000, 0xffff8003) AM_DEVREADWRITE8(FDC_TAG, wd1772_device, status_r, cmd_w,    0xff000000)
	AM_RANGE(0xffff8000, 0xffff8003) AM_DEVREADWRITE8(FDC_TAG, wd1772_device, track_r, track_w,   0x00ff0000)
	AM_RANGE(0xffff8000, 0xffff8003) AM_DEVREADWRITE8(FDC_TAG, wd1772_device, sector_r, sector_w, 0x0000ff00)
	AM_RANGE(0xffff8000, 0xffff8003) AM_DEVREADWRITE8(FDC_TAG, wd1772_device, data_r, data_w,     0x000000ff)
	AM_RANGE(0xffff8080, 0xffff808f) AM_DEVREADWRITE8(DUART_A_TAG, mc68681_device, read, write, 0xffffffff)
	AM_RANGE(0xffff80a0, 0xffff80af) AM_DEVREADWRITE8(DUART_B_TAG, mc68681_device, read, write, 0xffffffff)
	AM_RANGE(0xffff80c0, 0xffff80df) AM_DEVREADWRITE8(PIT_TAG, pit68230_device, read, write, 0xffffffff)
ADDRESS_MAP_END

static MACHINE_CONFIG_START( micro20 )
	/* basic machine hardware */
	MCFG_CPU_ADD(MAINCPU_TAG, M68020, XTAL_16_67MHz)
	MCFG_CPU_PROGRAM_MAP(micro20_map)

	MCFG_MC68681_ADD(DUART_A_TAG, XTAL_3_6864MHz)
	MCFG_MC68681_A_TX_CALLBACK(DEVWRITELINE("rs232", rs232_port_device, write_txd))

	MCFG_RS232_PORT_ADD("rs232", default_rs232_devices, "terminal")
	MCFG_RS232_RXD_HANDLER(DEVWRITELINE(DUART_A_TAG, mc68681_device, rx_a_w))

	MCFG_MC68681_ADD(DUART_B_TAG, XTAL_3_6864MHz)

	MCFG_WD1772_ADD(FDC_TAG, XTAL_16_67MHz / 2)

	MCFG_DEVICE_ADD(PIT_TAG, PIT68230, XTAL_16_67MHz / 2)
	MCFG_PIT68230_TIMER_IRQ_CB(WRITELINE(micro20_state, timerirq_w))
	MCFG_PIT68230_H4_CB(WRITELINE(micro20_state, h4_w))
	MCFG_PIT68230_PB_OUTPUT_CB(WRITE8(micro20_state, portb_w))
	MCFG_PIT68230_PC_OUTPUT_CB(WRITE8(micro20_state, portc_w))

	MCFG_DEVICE_ADD(RTC_TAG, MSM58321, XTAL_32_768kHz)
	MCFG_MSM58321_DEFAULT_24H(false)
	MCFG_MSM58321_D0_HANDLER(DEVWRITELINE(PIT_TAG, pit68230_device, pb0_w))
	MCFG_MSM58321_D1_HANDLER(DEVWRITELINE(PIT_TAG, pit68230_device, pb1_w))
	MCFG_MSM58321_D2_HANDLER(DEVWRITELINE(PIT_TAG, pit68230_device, pb2_w))
	MCFG_MSM58321_D3_HANDLER(DEVWRITELINE(PIT_TAG, pit68230_device, pb3_w))
	MCFG_MSM58321_BUSY_HANDLER(DEVWRITELINE(PIT_TAG, pit68230_device, pb7_w))

	MCFG_TIMER_DRIVER_ADD_PERIODIC("timer", micro20_state, micro20_timer, attotime::from_hz(200))
MACHINE_CONFIG_END

static INPUT_PORTS_START( micro20 )
INPUT_PORTS_END

/***************************************************************************

  Machine driver(s)

***************************************************************************/


ROM_START( micro20 )
	ROM_REGION32_BE(0x40000, "bootrom", 0)
	ROM_LOAD32_BYTE( "d00-07_u6_6791.bin",  0x000003, 0x010000, CRC(63d66ea1) SHA1(c5dfbc4d81920e1d2e981c52c1af3d486d382a35) )
	ROM_LOAD32_BYTE( "d08-15_u8_0dc6.bin",  0x000002, 0x010000, CRC(d62ef21f) SHA1(2779d430b1a0b835807627e707d46547b29ef579) )
	ROM_LOAD32_BYTE( "d16-23_u10_e5b0.bin", 0x000001, 0x010000, CRC(cd7acf86) SHA1(db994ed714a1079fbb66616355e8f18d2d1a2005) )
	ROM_LOAD32_BYTE( "d24-31_u13_d115.bin", 0x000000, 0x010000, CRC(3646d943) SHA1(97ee54063e2fe49fef2ff68d0f2e39345a75eac5) )
ROM_END

COMP( 1984, micro20,  0,        0,      micro20,  micro20, micro20_state, 0,  "GMX", "Micro 20",  MACHINE_NOT_WORKING | MACHINE_NO_SOUND )
