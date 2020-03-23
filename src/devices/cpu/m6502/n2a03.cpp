// license:BSD-3-Clause
// copyright-holders:Olivier Galibert
/***************************************************************************

    n2a03.cpp

    6502, NES variant

***************************************************************************/

#include "emu.h"
#include "n2a03.h"

DEFINE_DEVICE_TYPE(N2A03, n2a03_device, "n2a03", "N2A03")

READ8_MEMBER(n2a03_device::psg1_4014_r)
{
	return m_apu->read(space, 0x14);
}

READ8_MEMBER(n2a03_device::psg1_4015_r)
{
	return m_apu->read(space, 0x15);
}

WRITE8_MEMBER(n2a03_device::psg1_4015_w)
{
	m_apu->write(space, 0x15, data);
}

WRITE8_MEMBER(n2a03_device::psg1_4017_w)
{
	m_apu->write(space, 0x17, data);
}


// on various drivers          output port 0x4014 is used for external hardware   (not used by APU?)
//                       input/output port 0x4016  ^                              (not used by APU?)
//                       input        port 0x4017  ^                              ( APU_IRQCTRL )
// is there a fall through where every write is seen by other hw, or do these addresses really not touch the APU?? APU_IRQCTRL can definitely be written by can it be read back?

static ADDRESS_MAP_START( n2a03_map, AS_PROGRAM, 8, n2a03_device )
	AM_RANGE(0x4000, 0x4013) AM_DEVREADWRITE("nesapu", nesapu_device, read, write)
	AM_RANGE(0x4014, 0x4014) AM_READ(psg1_4014_r) // AM_WRITE(sprite_dma_0_w)
	AM_RANGE(0x4015, 0x4015) AM_READWRITE(psg1_4015_r, psg1_4015_w) /* PSG status / first control register */
	//AM_RANGE(0x4016, 0x4016) AM_READWRITE(vsnes_in0_r, vsnes_in0_w)
	AM_RANGE(0x4017, 0x4017) /*AM_READ(vsnes_in1_r)*/ AM_WRITE(psg1_4017_w)
ADDRESS_MAP_END



n2a03_device::n2a03_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	m6502_device(mconfig, N2A03, tag, owner, clock),
	m_apu(*this, "nesapu")
{
	program_config.m_internal_map = ADDRESS_MAP_NAME(n2a03_map);
}

offs_t n2a03_device::disasm_disassemble(std::ostream &stream, offs_t pc, const uint8_t *oprom, const uint8_t *opram, uint32_t options)
{
	return disassemble_generic(stream, pc, oprom, opram, options, disasm_entries);
}

void n2a03_device::device_start()
{
	if(!m_apu->started())
		throw device_missing_dependencies();

	m6502_device::device_start();
}

void n2a03_device::device_clock_changed()
{
	m_apu->set_unscaled_clock(clock());
}

WRITE_LINE_MEMBER(n2a03_device::apu_irq)
{
	// games relying on the APU_IRQ don't seem to work anyway? (nes software list : timelord, mig29sf, firehawk)
	set_input_line(N2A03_APU_IRQ_LINE, state ? ASSERT_LINE : CLEAR_LINE);
}

READ8_MEMBER(n2a03_device::apu_read_mem)
{
	return mintf->program->read_byte(offset);
}

MACHINE_CONFIG_MEMBER( n2a03_device::device_add_mconfig )
	MCFG_SOUND_ADD("nesapu", NES_APU, DERIVED_CLOCK(1,1) )
	MCFG_NES_APU_IRQ_HANDLER(WRITELINE(n2a03_device, apu_irq))
	MCFG_NES_APU_MEM_READ_CALLBACK(READ8(n2a03_device, apu_read_mem))

	MCFG_SOUND_ROUTE(ALL_OUTPUTS, ":mono", 0.50)

MACHINE_CONFIG_END


#include "cpu/m6502/n2a03.hxx"
