// license:BSD-3-Clause
// copyright-holders: Aaron Giles, Ted Green
// NEC VRC 5074 System Controller

#ifndef MAME_MACHINE_VRC5074_H
#define MAME_MACHINE_VRC5074_H

#pragma once

#include "pci.h"
#include "cpu/mips/mips3.h"

#define MCFG_VRC5074_ADD(_tag, _cpu_tag) \
	MCFG_PCI_HOST_ADD(_tag, VRC5074, 0x1033005a, 0x04, 0x00000000) \
	downcast<vrc5074_device *>(device)->set_cpu_tag(_cpu_tag);

#define MCFG_VRC5074_SET_SDRAM(_index, _size) \
	downcast<vrc5074_device *>(device)->set_sdram_size(_index, _size);

#define MCFG_VRC5074_SET_CS(_cs_num, _map) \
	downcast<vrc5074_device *>(device)->set_map(_cs_num, address_map_delegate(ADDRESS_MAP_NAME(_map), #_map), owner);

class vrc5074_device : public pci_host_device {
public:
	static constexpr unsigned SYSTEM_CLOCK = 100000000;

	vrc5074_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual void reset_all_mappings() override;
	virtual void map_extra(uint64_t memory_window_start, uint64_t memory_window_end, uint64_t memory_offset, address_space *memory_space,
							uint64_t io_window_start, uint64_t io_window_end, uint64_t io_offset, address_space *io_space) override;
	void postload();

	void set_cpu_tag(const char *tag);
	void set_sdram_size(const int index, const int size) { m_sdram_size[index] = size; };

	void set_map(int id, const address_map_delegate &map, device_t *device);

	virtual DECLARE_ADDRESS_MAP(config_map, 32) override;
	DECLARE_READ32_MEMBER(sdram_addr_r);
	DECLARE_WRITE32_MEMBER(sdram_addr_w);

	// PCI interrupts
	DECLARE_WRITE_LINE_MEMBER(pci_intr_a);
	DECLARE_WRITE_LINE_MEMBER(pci_intr_b);
	DECLARE_WRITE_LINE_MEMBER(pci_intr_c);
	DECLARE_WRITE_LINE_MEMBER(pci_intr_d);
	DECLARE_WRITE_LINE_MEMBER(pci_intr_e);
	void update_pci_irq(const int index, const int state);

	//cpu bus registers
	DECLARE_READ32_MEMBER (cpu_reg_r);
	DECLARE_WRITE32_MEMBER(cpu_reg_w);
	DECLARE_READ32_MEMBER(serial_r);
	DECLARE_WRITE32_MEMBER(serial_w);
	void update_nile_irqs();

	DECLARE_READ32_MEMBER (pci0_r);
	DECLARE_WRITE32_MEMBER(pci0_w);

	DECLARE_READ32_MEMBER (pci1_r);
	DECLARE_WRITE32_MEMBER(pci1_w);

	virtual DECLARE_ADDRESS_MAP(target1_map, 32);
	DECLARE_READ32_MEMBER (target1_r);
	DECLARE_WRITE32_MEMBER(target1_w);

protected:
	address_space *m_cpu_space;
	virtual space_config_vector memory_space_config() const override;
	virtual void device_start() override;
	virtual void device_reset() override;

private:
	enum
	{
		AS_PCI_MEM = 1,
		AS_PCI_IO = 2
	};

	mips3_device *m_cpu;
	const char *cpu_tag;
	int m_sdram_size[2];

	address_space_config m_mem_config, m_io_config;

	DECLARE_ADDRESS_MAP(cpu_map, 32);
	DECLARE_ADDRESS_MAP(serial_map, 32);

	void map_cpu_space();

	emu_timer* m_dma_timer;
	TIMER_CALLBACK_MEMBER(dma_transfer);
	emu_timer *m_timer[4];
	TIMER_CALLBACK_MEMBER(nile_timer_callback);

	required_memory_region m_romRegion;
	optional_memory_region m_updateRegion;
	std::vector<uint32_t> m_sdram[2];

	// Chip Select
	device_t *m_cs_devices[7];
	address_map_delegate m_cs_maps[7];

	uint32_t m_cpu_regs[0x1ff / 4];
	uint32_t m_serial_regs[0x40 / 4];
	uint16_t m_nile_irq_state;

	void setup_pci_space();
	uint32_t m_pci_laddr[2], m_pci_mask[2], m_pci_type[2];
	uint32_t m_sdram_addr[2];

};


DECLARE_DEVICE_TYPE(VRC5074, vrc5074_device)

#endif // MAME_MACHINE_VRC5074_H
