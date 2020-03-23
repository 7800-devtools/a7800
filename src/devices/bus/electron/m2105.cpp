// license:BSD-3-Clause
// copyright-holders:Nigel Barnes
/**********************************************************************

    BT Merlin M2105

**********************************************************************/


#include "emu.h"
#include "m2105.h"
#include "speaker.h"


//**************************************************************************
//  DEVICE DEFINITIONS
//**************************************************************************

DEFINE_DEVICE_TYPE(ELECTRON_M2105, electron_m2105_device, "electron_m2105", "Acorn M2105 Expansion")


//-------------------------------------------------
//  ROM( m2105 )
//-------------------------------------------------

ROM_START( m2105 )
	ROM_REGION(0x10000, "exp_rom", 0)
	ROM_DEFAULT_BIOS("v350")

	ROM_SYSTEM_BIOS(0, "v350", "V3.50 16/02/87")
	ROMX_LOAD("ic22-sm-35l-1.ic22", 0x0000, 0x4000, CRC(e8f8a639) SHA1(eb7fa1e884be9c072ae0c1e598507b802422127f), ROM_BIOS(1))
	ROMX_LOAD("ic23-sm-35l-1.ic23", 0x4000, 0x4000, CRC(b1bb1d83) SHA1(07ca3a93744519b8d03bbf1c3c3537c0a0a3c6fe), ROM_BIOS(1))
	ROMX_LOAD("sk01-pc-35l-1.ic24", 0x8000, 0x4000, CRC(54fd4c09) SHA1(9588296306581580ba223cf6bce4be61476f14c4), ROM_BIOS(1))
	ROMX_LOAD("sk02-pc-35l-1.ic24", 0xc000, 0x4000, CRC(c08de988) SHA1(86f2da5f8e9a5301ad40360e286f841f42e94a99), ROM_BIOS(1))

	ROM_SYSTEM_BIOS(1, "v341", "V3.41 26/11/85")
	ROMX_LOAD("ic22-sm-34l-1.ic22", 0x0000, 0x4000, CRC(b514b15f) SHA1(a9c6c20b5a4f860b000511dde2f54497bcdd97b0), ROM_BIOS(2))
	ROMX_LOAD("ic23-sm-34l-1.ic23", 0x4000, 0x4000, CRC(18875889) SHA1(d1a7dd87c4d99869a1961becec5e9d567d8fad53), ROM_BIOS(2))
	ROMX_LOAD("sk01-pc-34l-1.ic24", 0x8000, 0x4000, CRC(a8796c9e) SHA1(29bc01b8f7617b252e4b243d13b1bbd3cd32cc3b), ROM_BIOS(2))
	ROMX_LOAD("sk02-pc-34l-1.ic24", 0xc000, 0x4000, CRC(fa74063c) SHA1(cdc31c606e69e7a6d221b7340a310d475d487fc9), ROM_BIOS(2))

	ROM_SYSTEM_BIOS(2, "v207", "V2.07 14/03/85")
	ROMX_LOAD("ic22-sm-207l-1.ic22", 0x0000, 0x4000, CRC(0c431547) SHA1(13d2eab49b9c79f507b7dd8436d1e56cf43be412), ROM_BIOS(3))
	ROMX_LOAD("ic23-sm-207l-1.ic23", 0x4000, 0x4000, CRC(15044d49) SHA1(e75fe4321579a9027527a0e256050d1444b3fe82), ROM_BIOS(3))
	ROMX_LOAD("sk01-pc-207l-1.ic24", 0x8000, 0x4000, CRC(0850bcea) SHA1(270e7a31e69e1454cfb70ced23a50f5d97efe4d5), ROM_BIOS(3))
	ROMX_LOAD("sk02-pc-207l-1.ic24", 0xc000, 0x4000, CRC(d8b9143f) SHA1(4e132c7a6dae4caf7203139b51882706d508c449), ROM_BIOS(3))

	ROM_REGION(0x4000, "vsm", 0) /* system speech PHROM */
	ROM_LOAD("phroma.bin", 0x0000, 0x4000, CRC(98e1bf9e) SHA1(b369809275cb67dfd8a749265e91adb2d2558ae6))
ROM_END


//-------------------------------------------------
//  device_add_mconfig - add device configuration
//-------------------------------------------------

MACHINE_CONFIG_MEMBER( electron_m2105_device::device_add_mconfig )
	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("mono")

	MCFG_INPUT_MERGER_ACTIVE_HIGH("irqs")
	MCFG_INPUT_MERGER_OUTPUT_HANDLER(WRITELINE(electron_m2105_device, intrq_w))

	/* system via */
	MCFG_DEVICE_ADD("via6522_0", VIA6522, 1000000)
	/*MCFG_VIA6522_READPA_HANDLER(READ8(electron_m2105_device, m2105_via_system_read_porta))
	MCFG_VIA6522_READPB_HANDLER(READ8(electron_m2105_device, m2105_via_system_read_portb))
	MCFG_VIA6522_WRITEPA_HANDLER(WRITE8(electron_m2105_device, m2105_via_system_write_porta))
	MCFG_VIA6522_WRITEPB_HANDLER(WRITE8(electron_m2105_device, m2105_via_system_write_portb))*/
	MCFG_VIA6522_IRQ_HANDLER(DEVWRITELINE("irqs", input_merger_active_high_device, in0_w))

	/* user via */
	MCFG_DEVICE_ADD("via6522_1", VIA6522, 1000000)
	//MCFG_VIA6522_READPB_HANDLER(READ8(electron_m2105_device, m2105_via_user_read_portb))
	MCFG_VIA6522_WRITEPA_HANDLER(DEVWRITE8("cent_data_out", output_latch_device, write))
	//MCFG_VIA6522_WRITEPB_HANDLER(WRITE8(electron_m2105_device, m2105_via_user_write_portb))
	MCFG_VIA6522_CA2_HANDLER(DEVWRITELINE("centronics", centronics_device, write_strobe))
	MCFG_VIA6522_IRQ_HANDLER(DEVWRITELINE("irqs", input_merger_active_high_device, in1_w))

	/* duart */
	MCFG_MC68681_ADD("sc2681", XTAL_3_6864MHz)
	MCFG_MC68681_IRQ_CALLBACK(DEVWRITELINE("irqs", input_merger_active_high_device, in2_w))
	MCFG_MC68681_A_TX_CALLBACK(DEVWRITELINE("rs232", rs232_port_device, write_txd))
	//MCFG_MC68681_OUTPORT_CALLBACK(WRITE8(electron_m2105_device, sio_out_w))

	MCFG_RS232_PORT_ADD("rs232", default_rs232_devices, nullptr)
	MCFG_RS232_RXD_HANDLER(DEVWRITELINE("sc2681", mc68681_device, rx_a_w))

	/* printer */
	MCFG_CENTRONICS_ADD("centronics", centronics_devices, "printer")
	MCFG_CENTRONICS_ACK_HANDLER(DEVWRITELINE("via6522_1", via6522_device, write_ca1)) MCFG_DEVCB_INVERT /* ack seems to be inverted? */
	MCFG_CENTRONICS_OUTPUT_LATCH_ADD("cent_data_out", "centronics")

	/* speech hardware */
	MCFG_DEVICE_ADD("vsm", SPEECHROM, 0)
	MCFG_SOUND_ADD("tms5220", TMS5220, 640000)
	MCFG_TMS52XX_SPEECHROM("vsm")
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 1.0)
MACHINE_CONFIG_END

const tiny_rom_entry *electron_m2105_device::device_rom_region() const
{
	return ROM_NAME( m2105 );
}

//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  electron_m2105_device - constructor
//-------------------------------------------------

electron_m2105_device::electron_m2105_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, ELECTRON_M2105, tag, owner, clock)
	, device_electron_expansion_interface(mconfig, *this)
	, m_exp_rom(*this, "exp_rom")
	, m_via6522_0(*this, "via6522_0")
	, m_via6522_1(*this, "via6522_1")
	, m_duart(*this, "sc2681")
	, m_tms(*this, "tms5220")
	, m_centronics(*this, "centronics")
	, m_irqs(*this, "irqs")
{
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void electron_m2105_device::device_start()
{
	address_space& space = machine().device("maincpu")->memory().space(AS_PROGRAM);
	m_slot = dynamic_cast<electron_expansion_slot_device *>(owner());

	space.install_readwrite_handler(0xfc40, 0xfc5f, READ8_DEVICE_DELEGATE(m_via6522_1, via6522_device, read), WRITE8_DEVICE_DELEGATE(m_via6522_1, via6522_device, write));
	space.install_readwrite_handler(0xfc60, 0xfc6f, READ8_DEVICE_DELEGATE(m_duart, mc68681_device, read), WRITE8_DEVICE_DELEGATE(m_duart, mc68681_device, write));
	space.install_readwrite_handler(0xfc70, 0xfc8f, READ8_DEVICE_DELEGATE(m_via6522_0, via6522_device, read), WRITE8_DEVICE_DELEGATE(m_via6522_0, via6522_device, write));
}

//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void electron_m2105_device::device_reset()
{
	machine().root_device().membank("bank2")->configure_entry(12, memregion("exp_rom")->base() + 0x0000);
	machine().root_device().membank("bank2")->configure_entry(13, memregion("exp_rom")->base() + 0x4000);
	machine().root_device().membank("bank2")->configure_entry( 0, memregion("exp_rom")->base() + 0x8000);
	machine().root_device().membank("bank2")->configure_entry( 2, memregion("exp_rom")->base() + 0xc000);
}


//**************************************************************************
//  IMPLEMENTATION
//**************************************************************************

WRITE_LINE_MEMBER(electron_m2105_device::intrq_w)
{
	m_slot->irq_w(state);
}
