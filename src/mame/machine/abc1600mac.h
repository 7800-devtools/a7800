// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    Luxor ABC 1600 Memory Access Controller emulation

**********************************************************************/
#ifndef MAME_MACHINE_ABC1600MAC_H
#define MAME_MACHINE_ABC1600MAC_H

#pragma once


#include "cpu/m68000/m68000.h"
#include "machine/watchdog.h"



///*************************************************************************
//  MACROS / CONSTANTS
///*************************************************************************

#define ABC1600_MAC_TAG "mac"



///*************************************************************************
//  INTERFACE CONFIGURATION MACROS
///*************************************************************************

#define MCFG_ABC1600_MAC_ADD(_cpu_tag, _program_map) \
	MCFG_DEVICE_ADD(ABC1600_MAC_TAG, ABC1600_MAC, 0) \
	MCFG_DEVICE_ADDRESS_MAP(AS_PROGRAM, _program_map) \
	downcast<abc1600_mac_device *>(device)->set_cpu_tag(_cpu_tag);



///*************************************************************************
//  TYPE DEFINITIONS
///*************************************************************************

// ======================> abc1600_mac_device

class abc1600_mac_device : public device_t,
							public device_memory_interface
{
public:
	abc1600_mac_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	void set_cpu_tag(const char *cpu_tag) { m_cpu_tag = cpu_tag; }

	virtual DECLARE_ADDRESS_MAP(map, 8);

	DECLARE_READ8_MEMBER( cause_r );
	DECLARE_WRITE8_MEMBER( task_w );
	DECLARE_READ8_MEMBER( segment_r );
	DECLARE_WRITE8_MEMBER( segment_w );
	DECLARE_READ8_MEMBER( page_r );
	DECLARE_WRITE8_MEMBER( page_w );
	DECLARE_WRITE8_MEMBER( dmamap_w );

	DECLARE_READ8_MEMBER( dma0_mreq_r ) { return dma_mreq_r(DMAMAP_R0_LO, offset); }
	DECLARE_WRITE8_MEMBER( dma0_mreq_w ) { dma_mreq_w(DMAMAP_R0_LO, offset, data); }
	DECLARE_READ8_MEMBER( dma0_iorq_r ) { return dma_iorq_r(DMAMAP_R0_LO, offset); }
	DECLARE_WRITE8_MEMBER( dma0_iorq_w ) { dma_iorq_w(DMAMAP_R0_LO, offset, data); }
	DECLARE_READ8_MEMBER( dma1_mreq_r ) { return dma_mreq_r(DMAMAP_R1_LO, offset); }
	DECLARE_WRITE8_MEMBER( dma1_mreq_w ) { dma_mreq_w(DMAMAP_R1_LO, offset, data); }
	DECLARE_READ8_MEMBER( dma1_iorq_r ) { return dma_iorq_r(DMAMAP_R1_LO, offset); }
	DECLARE_WRITE8_MEMBER( dma1_iorq_w ) { dma_iorq_w(DMAMAP_R1_LO, offset, data); }
	DECLARE_READ8_MEMBER( dma2_mreq_r ) { return dma_mreq_r(DMAMAP_R2_LO, offset); }
	DECLARE_WRITE8_MEMBER( dma2_mreq_w ) { dma_mreq_w(DMAMAP_R2_LO, offset, data); }
	DECLARE_READ8_MEMBER( dma2_iorq_r ) { return dma_iorq_r(DMAMAP_R2_LO, offset); }
	DECLARE_WRITE8_MEMBER( dma2_iorq_w ) { dma_iorq_w(DMAMAP_R2_LO, offset, data); }

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// device_memory_interface overrides
	virtual space_config_vector memory_space_config() const override;

	// optional information overrides
	virtual void device_add_mconfig(machine_config &config) override;
	virtual const tiny_rom_entry *device_rom_region() const override;

private:
	enum
	{
		DMAMAP_R2_LO = 0,
		DMAMAP_R2_HI,
		DMAMAP_R1_LO = 4,
		DMAMAP_R1_HI,
		DMAMAP_R0_LO,
		DMAMAP_R0_HI
	};

	DECLARE_READ8_MEMBER( read );
	DECLARE_WRITE8_MEMBER( write );

	int get_current_task(offs_t offset);
	offs_t get_segment_address(offs_t offset);
	offs_t get_page_address(offs_t offset, uint8_t segd);
	offs_t translate_address(offs_t offset, int *nonx, int *wp);
	uint8_t read_user_memory(offs_t offset);
	void write_user_memory(offs_t offset, uint8_t data);
	int get_fc();
	uint8_t read_supervisor_memory(address_space &space, offs_t offset);
	void write_supervisor_memory(address_space &space, offs_t offset, uint8_t data);
	offs_t get_dma_address(int index, uint16_t offset);
	uint8_t dma_mreq_r(int index, uint16_t offset);
	void dma_mreq_w(int index, uint16_t offset, uint8_t data);
	uint8_t dma_iorq_r(int index, uint16_t offset);
	void dma_iorq_w(int index, uint16_t offset, uint8_t data);

	const address_space_config m_space_config;

	required_memory_region m_rom;
	optional_shared_ptr<uint8_t> m_segment_ram;
	optional_shared_ptr<uint16_t> m_page_ram;

	required_device<watchdog_timer_device> m_watchdog;

	const char *m_cpu_tag;
	m68000_base_device *m_cpu;

	int m_ifc2;
	uint8_t m_task;
	uint8_t m_dmamap[8];
	uint8_t m_cause;
};


// device type definition
DECLARE_DEVICE_TYPE(ABC1600_MAC, abc1600_mac_device)


#endif // MAME_MACHINE_ABC1600MAC_H
