// license:BSD-3-Clause
// copyright-holders:R. Belmont
/*********************************************************************

    pci9050.c - PLX PCI9050 PCI to 4x Local Bus Bridge

    by R. Belmont

    PCI spaces:
    0 - (config memory) not used
    1 - (config I/O) config regs
    2 - local bus 1 window
    3 - local bus 2 window
    4 - local bus 3 window
    5 - local bus 4 window

    PCI9050 is located, mapped, and initialized at BFC00700.

    The boot ROM then copies ROM to RAM, jumps to RAM, and starts trying to
    access Zeus 2 video through the mapped windows.

*********************************************************************/

#include "emu.h"
#include "pci9050.h"

DEFINE_DEVICE_TYPE(PCI9050, pci9050_device, "pci9050", "PLX PCI9050 PCI to Local Bus Bridge")

DEVICE_ADDRESS_MAP_START(map, 32, pci9050_device)
	AM_RANGE(0x00, 0x0f) AM_READWRITE(lasrr_r,   lasrr_w  )
	AM_RANGE(0x10, 0x13) AM_READWRITE(eromrr_r,  eromrr_w )
	AM_RANGE(0x14, 0x23) AM_READWRITE(lasba_r,   lasba_w  )
	AM_RANGE(0x24, 0x27) AM_READWRITE(eromba_r,  eromba_w )
	AM_RANGE(0x28, 0x37) AM_READWRITE(lasbrd_r,  lasbrd_w )
	AM_RANGE(0x38, 0x3b) AM_READWRITE(erombrd_r, erombrd_w)
	AM_RANGE(0x3c, 0x4b) AM_READWRITE(csbase_r,  csbase_w )
	AM_RANGE(0x4c, 0x4f) AM_READWRITE(intcsr_r,  intcsr_w )
	AM_RANGE(0x50, 0x53) AM_READWRITE(cntrl_r,   cntrl_w  )
ADDRESS_MAP_END

DEVICE_ADDRESS_MAP_START(empty, 32, pci9050_device)
ADDRESS_MAP_END

pci9050_device::pci9050_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: pci_device(mconfig, PCI9050, tag, owner, clock),
	m_user_input_handler(*this), m_user_output_handler(*this)
{
	for(int i=0; i<4; i++) {
		m_devices[i] = nullptr;
		m_names[i] = nullptr;
	}
}

void pci9050_device::set_map(int id, const address_map_delegate &map, device_t *device)
{
	m_maps[id] = map;
	m_devices[id] = device;
}

void pci9050_device::device_start()
{
	pci_device::device_start();

	add_map(0x80, M_MEM, FUNC(pci9050_device::map));           // map 0 is our config registers, mem space
	add_map(0x80, M_IO,  FUNC(pci9050_device::map));           // map 1 is our config registers, i/o space

	for(int i=0; i<4; i++)
		if(!m_maps[i].isnull())
			add_map(0, M_MEM | M_DISABLED, m_maps[i], m_devices[i]);
		else
			add_map(0, M_MEM | M_DISABLED, FUNC(pci9050_device::empty));

	m_user_input_handler.resolve();
	m_user_output_handler.resolve();
	// Save states
	save_item(NAME(m_lasrr));
	save_item(NAME(m_lasba));
	save_item(NAME(m_lasbrd));
	save_item(NAME(m_csbase));
	save_item(NAME(m_eromrr));
	save_item(NAME(m_eromba));
	save_item(NAME(m_erombrd));
	save_item(NAME(m_intcsr));
	save_item(NAME(m_cntrl));
	machine().save().register_postload(save_prepost_delegate(FUNC(pci9050_device::postload), this));

}

void pci9050_device::postload(void)
{
	remap_rom();
	for (int id = 0; id < 4; id++)
		remap_local(id);
}

void pci9050_device::device_reset()
{
	pci_device::device_reset();
	set_map_address(0, 0);
	set_map_address(1, 0);
	for(int i=0; i<4; i++) {
		m_lasrr[i] = i ? 0 : 0x0ff00000;
		m_lasba[i] = 0;
		m_lasbrd[i] = 0x00800000;
		m_csbase[i] = 0;
		set_map_flags(i+2, M_MEM | M_DISABLED);
	}
	m_eromrr = 0x07ff8000;
	m_eromba = 0x00080000;
	m_erombrd = 0x00800000;
	m_intcsr = 0;
	m_cntrl = 0;
}

void pci9050_device::remap_local(int id)
{
	uint32_t csbase = m_csbase[id];
	uint32_t lasrr = m_lasrr[id];
	logerror("local bus %d csbase=%08x lasrr=%08x\n", id, csbase, lasrr);

	if(!(csbase & 1)) {
		set_map_flags(id+2, M_MEM | M_DISABLED);
		return;
	}
	int lsize;
	for(lsize=1; lsize<28 && !(csbase & (1<<lsize)); lsize++) {};
	if(lsize == 28) {
		set_map_flags(id+2, M_MEM | M_DISABLED);
		return;
	}
	int size = 2 << lsize;
	// Address map is directly connected to PCI address space so post-decode mapping is not needed. (Ted Green)
	if(0 & csbase & 0x0fffffff & ~(size-1)) {
		logerror("PCI9050 local bus %d size=%08x csbase=%08X disabled due to unimplemented post-decode remapping\n", id, size, csbase);
		set_map_flags(id+2, M_MEM | M_DISABLED);
		return;
	}

	uint32_t mask = ~(size - 1);
	if(lasrr & 1)
		mask &= 0x0ffffffc;
	else
		mask &= 0x0ffffff0;

	if((lasrr & mask) != mask) {
		logerror("PCI9050 local bus %d disabled due to unimplemented pci mirroring\n", id);
		//      set_map_flags(id+2, M_MEM | M_DISABLED);
		//      return;
	}

	set_map_size(id+2, size);
	set_map_flags(id+2, lasrr & 1 ? M_IO : lasrr & 8 ? M_MEM | M_PREF : M_MEM);
}

void pci9050_device::remap_rom()
{
	switch ((m_cntrl >> 12) & 0x3) {
	case 0:
	case 3:
		set_map_flags(0, M_MEM);
		set_map_flags(1, M_IO);
		break;
	case 1:
		set_map_flags(0, M_MEM);
		set_map_flags(1, M_IO | M_DISABLED);
		break;
	case 2:
		set_map_flags(0, M_MEM | M_DISABLED);
		set_map_flags(1, M_IO);
		break;
	}
}

READ32_MEMBER (pci9050_device::lasrr_r)
{
	return m_lasrr[offset];
}

WRITE32_MEMBER(pci9050_device::lasrr_w)
{
	logerror("%06X:PCI9050 local bus %d range = %08x: %s flags %d pf %d addr bits 27-4 %08x\n", machine().device("maincpu")->safe_pc(), offset, data, (data & 1) ? "I/O" : "MEM", (data & 6)>>1, (data & 8)>>3, data & 0xfffffff);
	m_lasrr[offset] = data;
	remap_local(offset);
}

READ32_MEMBER (pci9050_device::eromrr_r)
{
	return m_eromrr;
}

WRITE32_MEMBER(pci9050_device::eromrr_w)
{
	logerror("%06X:PCI9050 ROM range = %08x: addr bits 27-11 %08x\n", machine().device("maincpu")->safe_pc(), data, data & 0xfffff800);
	m_eromrr = data;
	remap_rom();
}

READ32_MEMBER (pci9050_device::lasba_r)
{
	return m_lasba[offset];
}

WRITE32_MEMBER(pci9050_device::lasba_w)
{
	logerror("%06X:PCI9050 local bus %d base = %08x: enable %d remap %08x\n", machine().device("maincpu")->safe_pc(), offset, data, data&1, data & 0x0ffffffe);
	m_lasba[offset] = data;
	remap_local(offset);
}

READ32_MEMBER (pci9050_device::eromba_r)
{
	return m_eromba;
}

WRITE32_MEMBER(pci9050_device::eromba_w)
{
	logerror("%06X:PCI9050 ROM base = %08x: remap %08x\n", machine().device("maincpu")->safe_pc(), data, data & 0x0ffff800);
	m_eromba = data;
	remap_rom();
}

READ32_MEMBER (pci9050_device::lasbrd_r)
{
	return m_lasbrd[offset];
}

WRITE32_MEMBER(pci9050_device::lasbrd_w)
{
	logerror("%06X:PCI9050 local bus %d descriptors = %08x: burst %d prefetch %d width %d, endian %s, endian mode %d\n", machine().device("maincpu")->safe_pc(), offset, data, data&1, (data >> 5) & 1, (data >> 22) & 3, ((data >> 24) & 1) ? "BE" : "LE", (data >> 25) & 1);
	m_lasbrd[offset] = data;
	remap_local(offset);
}

READ32_MEMBER (pci9050_device::erombrd_r)
{
	return m_erombrd;
}

WRITE32_MEMBER(pci9050_device::erombrd_w)
{
	logerror("%06X:PCI9050 ROM descriptors = %08x: burst %d prefetch %d bits %d, endian %s, endian mode %d\n", machine().device("maincpu")->safe_pc(), data, data&1, (data >> 5) & 1, (data >> 22) & 3, ((data >> 24) & 1) ? "BE" : "LE", (data >> 25) & 1);
	m_erombrd = data;
	remap_rom();
}

READ32_MEMBER (pci9050_device::csbase_r)
{
	return m_csbase[offset];
}

WRITE32_MEMBER(pci9050_device::csbase_w)
{
	logerror("%06X:PCI9050 chip select %d base = %08x: enable %d size %08x\n", machine().device("maincpu")->safe_pc(), offset, data, data&1, data&0xfffffffe);
	m_csbase[offset] = data;
	remap_local(offset);
}

READ32_MEMBER (pci9050_device::intcsr_r)
{
	logerror("%06X:PCI9050 IRQ CSR read %08x\n", machine().device("maincpu")->safe_pc(), m_intcsr);
	return m_intcsr;
}

WRITE32_MEMBER(pci9050_device::intcsr_w)
{
	logerror("%06X:PCI9050 IRQ CSR write %08x\n", machine().device("maincpu")->safe_pc(), data);
	m_intcsr = data;
	remap_rom();
}

READ32_MEMBER (pci9050_device::cntrl_r)
{
	if (!m_user_input_handler.isnull())
	{
		int readData = m_user_input_handler();
		for (int userIndex = 0; userIndex < 4; userIndex++)
			if ((m_cntrl & (1 << (1 + userIndex * 3)))==0)
				m_cntrl = (m_cntrl & ~(1<<(2 + userIndex * 3))) | (((readData>>userIndex)&1) << (2 + userIndex * 3));
	}
	if (0)
		logerror("%06X:PCI9050 CNTRL read = %08x\n", machine().device("maincpu")->safe_pc(), m_cntrl);
	return m_cntrl;
}

WRITE32_MEMBER(pci9050_device::cntrl_w)
{
	if (0)
		logerror("%06X:PCI9050 CNTRL write %08x\n", machine().device("maincpu")->safe_pc(), data);
	uint32_t oldData = m_cntrl;
	m_cntrl = data;
	remap_rom();
	if ((oldData ^ m_cntrl) & 0x3000)
		remap_cb();
	if (!m_user_output_handler.isnull()) {
		int userData = 0;
		for (int userIndex = 0; userIndex < 4; userIndex++)
			userData |= ((m_cntrl >> (2 + userIndex * 3)) & 1) << userIndex;
		m_user_output_handler(userData);
	}
}
