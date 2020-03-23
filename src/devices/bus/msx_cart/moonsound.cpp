// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/**********************************************************************************

TODO:
- Properly hook up correct SRAM sizes for different moonsound compatible
  cartridges. (Original moonsound has 128KB SRAM)

**********************************************************************************/

#include "emu.h"
#include "moonsound.h"

#include "speaker.h"


#define VERBOSE 0
#include "logmacro.h"


DEFINE_DEVICE_TYPE(MSX_CART_MOONSOUND, msx_cart_moonsound_device, "msx_moonsound", "MSX Cartridge - MoonSound")


msx_cart_moonsound_device::msx_cart_moonsound_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, MSX_CART_MOONSOUND, tag, owner, clock)
	, msx_cart_interface(mconfig, *this)
	, m_ymf278b(*this, "ymf278b")
{
}


static ADDRESS_MAP_START( ymf278b_map, 0, 8, msx_cart_moonsound_device )
	AM_RANGE(0x000000, 0x1fffff) AM_ROM
	AM_RANGE(0x200000, 0x3fffff) AM_RAM  // 2MB sram for testing
ADDRESS_MAP_END


MACHINE_CONFIG_MEMBER( msx_cart_moonsound_device::device_add_mconfig )
	// The moonsound cartridge has a separate stereo output.
	MCFG_SPEAKER_STANDARD_STEREO("lspeaker", "rspeaker")
	MCFG_SOUND_ADD("ymf278b", YMF278B, YMF278B_STD_CLOCK)
	MCFG_DEVICE_ADDRESS_MAP(0, ymf278b_map)
	MCFG_YMF278B_IRQ_HANDLER(WRITELINE(msx_cart_moonsound_device, irq_w))
	MCFG_SOUND_ROUTE(0, "lspeaker", 0.50)
	MCFG_SOUND_ROUTE(1, "rspeaker", 0.50)
	MCFG_SOUND_ROUTE(2, "lspeaker", 0.40)
	MCFG_SOUND_ROUTE(3, "rspeaker", 0.40)
	MCFG_SOUND_ROUTE(4, "lspeaker", 0.40)
	MCFG_SOUND_ROUTE(5, "rspeaker", 0.40)
MACHINE_CONFIG_END


ROM_START( msx_cart_moonsound )
	ROM_REGION(0x400000, "ymf278b", 0)
	ROM_LOAD("yrw801.rom", 0x0, 0x200000, CRC(2a9d8d43) SHA1(32760893ce06dbe3930627755ba065cc3d8ec6ca))
ROM_END


const tiny_rom_entry *msx_cart_moonsound_device::device_rom_region() const
{
	return ROM_NAME( msx_cart_moonsound );
}


void msx_cart_moonsound_device::device_start()
{
	m_out_irq_cb.resolve_safe();

	// Install IO read/write handlers
	address_space &space = machine().device<cpu_device>("maincpu")->space(AS_IO);
	space.install_readwrite_handler(0x7e, 0x7f, read8_delegate(FUNC(msx_cart_moonsound_device::read_ymf278b_pcm), this), write8_delegate(FUNC(msx_cart_moonsound_device::write_ymf278b_pcm), this));
	space.install_readwrite_handler(0xc4, 0xc7, read8_delegate(FUNC(msx_cart_moonsound_device::read_ymf278b_fm), this), write8_delegate(FUNC(msx_cart_moonsound_device::write_ymf278b_fm), this));
	space.install_read_handler(0xc0, 0xc0, read8_delegate(FUNC(msx_cart_moonsound_device::read_c0), this));
}


void msx_cart_moonsound_device::device_reset()
{
}


WRITE_LINE_MEMBER(msx_cart_moonsound_device::irq_w)
{
	LOG("moonsound: irq state %d\n", state);
	m_out_irq_cb(state);
}


WRITE8_MEMBER(msx_cart_moonsound_device::write_ymf278b_fm)
{
	LOG("moonsound: write 0x%02x, data 0x%02x\n", 0xc4 + offset, data);
	m_ymf278b->write(space, offset, data);
}


READ8_MEMBER(msx_cart_moonsound_device::read_ymf278b_fm)
{
	LOG("moonsound: read 0x%02x\n", 0xc4 + offset);
	return m_ymf278b->read(space, offset);
}


WRITE8_MEMBER(msx_cart_moonsound_device::write_ymf278b_pcm)
{
	LOG("moonsound: write 0x%02x, data 0x%02x\n", 0x7e + offset, data);
	m_ymf278b->write(space, 4 + offset, data);
}


READ8_MEMBER(msx_cart_moonsound_device::read_ymf278b_pcm)
{
	LOG("moonsound: read 0x%02x\n", 0x7e + offset);
	return m_ymf278b->read(space, 4 + offset);
}


// For detecting presence of moonsound cartridge
READ8_MEMBER(msx_cart_moonsound_device::read_c0)
{
	LOG("moonsound: read 0xc0\n");
	return 0x00;
}
