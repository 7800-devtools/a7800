// license:BSD-3-Clause
// copyright-holders:Sergey Svishchev
#include "emu.h"
#include "ibm6580_fdc.h"

#include "cpu/mcs48/mcs48.h"


#define VERBOSE_DBG 2       /* general debug messages */

#define DBG_LOG(N,M,A) \
	do { \
	if(VERBOSE_DBG>=N) \
		{ \
			if( M ) \
				logerror("%11.6f at %s: %-10s",machine().time().as_double(),machine().describe_context(),(char*)M ); \
			logerror A; \
		} \
	} while (0)


DEFINE_DEVICE_TYPE(DW_FDC, dw_fdc_device, "dw_fdc", "IBM Displaywriter Floppy")

ROM_START( dw_fdc )
	ROM_REGION(0x800, "mcu", 0)
	ROM_LOAD("4430030_FLP_8041.BIN", 0x0000, 0x400, CRC(2bb96799) SHA1(e30b0f2d790197f290858eab74ad5e151ded78c3))
ROM_END


const tiny_rom_entry *dw_fdc_device::device_rom_region() const
{
	return ROM_NAME( dw_fdc );
}

MACHINE_CONFIG_MEMBER( dw_fdc_device::device_add_mconfig )
	MCFG_CPU_ADD("mcu", I8048, XTAL_24MHz/4)    // divisor is unverified
//  MCFG_MCS48_PORT_BUS_IN_CB(READ8(dw_fdc_device, bus_r))
//  MCFG_MCS48_PORT_BUS_OUT_CB(WRITE8(dw_fdc_device, bus_w))
	MCFG_MCS48_PORT_P1_OUT_CB(WRITE8(dw_fdc_device, p1_w))
	MCFG_MCS48_PORT_P2_OUT_CB(WRITE8(dw_fdc_device, p2_w))
//  MCFG_MCS48_PORT_T0_IN_CB(READLINE(dw_fdc_device, t0_r))
	MCFG_MCS48_PORT_T1_IN_CB(READLINE(dw_fdc_device, t1_r))

	MCFG_DEVICE_ADD("ppi8255", I8255, 0)

	MCFG_UPD765A_ADD("upd765", false, false)
//  MCFG_UPD765_INTRQ_CALLBACK(DEVWRITELINE("pic8259", pic8259_device, ir4_w))
//  MCFG_UPD765_DRQ_CALLBACK(DEVWRITELINE("dma8257", dma8257_device, XXX))
//  MCFG_FLOPPY_DRIVE_ADD(UPD765_TAG ":0", wangpc_floppies, "525dd", wangpc_state::floppy_formats)
//  MCFG_FLOPPY_DRIVE_ADD(UPD765_TAG ":1", wangpc_floppies, "525dd", wangpc_state::floppy_formats)
MACHINE_CONFIG_END


dw_fdc_device::dw_fdc_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, DW_FDC, tag, owner, clock)
	, m_out_data(*this)
	, m_out_clock(*this)
	, m_out_strobe(*this)
	, m_mcu(*this, "mcu")
{
}

void dw_fdc_device::device_start()
{
	m_out_data.resolve_safe();
	m_out_clock.resolve_safe();
	m_out_strobe.resolve_safe();
	m_reset_timer = timer_alloc();
}

void dw_fdc_device::device_reset()
{
	m_p1 = m_p2 = m_t0 = m_t1 = 0;
}

void dw_fdc_device::device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr)
{
	m_mcu->set_input_line(INPUT_LINE_RESET, ASSERT_LINE);
}

WRITE8_MEMBER( dw_fdc_device::p1_w )
{
	m_p1 = data;

	DBG_LOG(2,"p1",( "<- %02x\n", data, m_p1));
}

WRITE8_MEMBER( dw_fdc_device::p2_w )
{
	m_p2 = data;

	DBG_LOG(2,"p2",( "<- %02x\n", data));
}

READ8_MEMBER( dw_fdc_device::p2_r )
{
	uint8_t data = m_p2;

	DBG_LOG(2,"p2",( "== %02x\n", data));

	return data;
}

READ_LINE_MEMBER( dw_fdc_device::t0_r )
{
	DBG_LOG(2,"t0",( "== %d\n", m_t0));

	return m_t0;
}

READ_LINE_MEMBER( dw_fdc_device::t1_r )
{
	DBG_LOG(2,"t1",( "== %d\n", m_t1));

	return m_t1;
}

WRITE8_MEMBER( dw_fdc_device::bus_w )
{
	m_bus = data;
}

READ8_MEMBER( dw_fdc_device::bus_r )
{
	return m_bus;
}

WRITE_LINE_MEMBER( dw_fdc_device::reset_w )
{
	if(!state)
		m_reset_timer->adjust(attotime::from_msec(50));
	else
	{
		m_reset_timer->adjust(attotime::never);
		m_mcu->set_input_line(INPUT_LINE_RESET, CLEAR_LINE);
	}
}

WRITE_LINE_MEMBER( dw_fdc_device::ack_w )
{
	m_t0 = state;
}
