// license:BSD-3-Clause
// copyright-holders:Olivier Galibert
/***************************************************************************

    machine/lpci.h

    Legacy PCI bus

***************************************************************************/

#ifndef MAME_MACHINE_LPCI_H
#define MAME_MACHINE_LPCI_H

#pragma once

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

typedef uint32_t (*pci_read_func)(device_t *pcibus, device_t *device, int function, int reg, uint32_t mem_mask);
typedef void (*pci_write_func)(device_t *pcibus, device_t *device, int function, int reg, uint32_t data, uint32_t mem_mask);

// ======================> pci_bus_legacy_device

class pci_bus_legacy_device :  public device_t
{
public:
	// construction/destruction
	pci_bus_legacy_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	DECLARE_READ32_MEMBER( read );
	DECLARE_WRITE32_MEMBER( write );

	DECLARE_READ64_MEMBER( read_64be );
	DECLARE_WRITE64_MEMBER( write_64be );

	void set_busnum(int busnum) { m_busnum = busnum; }
	void set_father(const char *father) { m_father = father; }
	void set_device(int num, const char *tag, pci_read_func read_func, pci_write_func write_func) {
		m_devtag[num] = tag; m_read_callback[num] = read_func; m_write_callback[num] = write_func; }

	pci_bus_legacy_device *pci_search_bustree(int busnum, int devicenum, pci_bus_legacy_device *pcibus);
	void add_sibling(pci_bus_legacy_device *sibling, int busnum);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_post_load() override;

private:
	uint8_t               m_busnum;
	const char *        m_devtag[32];
	pci_read_func       m_read_callback[32];
	pci_write_func      m_write_callback[32];
	const char *        m_father;
	device_t *          m_device[32];
	pci_bus_legacy_device * m_siblings[8];
	uint8_t               m_siblings_busnum[8];
	int                 m_siblings_count;

	offs_t              m_address;
	int8_t                m_devicenum; // device number we are addressing
	int8_t                m_busnumber; // pci bus number we are addressing
	pci_bus_legacy_device * m_busnumaddr; // pci bus we are addressing
};

// device type definition
DECLARE_DEVICE_TYPE(PCI_BUS_LEGACY, pci_bus_legacy_device)


/***************************************************************************
    DEVICE CONFIGURATION MACROS
***************************************************************************/

#define MCFG_PCI_BUS_LEGACY_ADD(_tag, _busnum) \
	MCFG_DEVICE_ADD(_tag, PCI_BUS_LEGACY, 0) \
	downcast<pci_bus_legacy_device *>(device)->set_busnum(_busnum);
#define MCFG_PCI_BUS_LEGACY_DEVICE(_devnum, _devtag, _configread, _configwrite) \
	downcast<pci_bus_legacy_device *>(device)->set_device(_devnum, _devtag,_configread,_configwrite);
#define MCFG_PCI_BUS_LEGACY_SIBLING(_father_tag) \
	downcast<pci_bus_legacy_device *>(device)->set_father(_father_tag);


#endif // MAME_MACHINE_LPCI_H
