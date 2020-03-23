// license:BSD-3-Clause
// copyright-holders:Samuele Zannoli
#ifndef MAME_INCLUDES_XBOX_PCI_H
#define MAME_INCLUDES_XBOX_PCI_H

#pragma once

#include "xbox_nv2a.h"
#include "xbox_usb.h"

/*
 * Host
 */

class nv2a_host_device : public pci_host_device {
public:
	nv2a_host_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	virtual void map_extra(uint64_t memory_window_start, uint64_t memory_window_end, uint64_t memory_offset, address_space *memory_space,
		uint64_t io_window_start, uint64_t io_window_end, uint64_t io_offset, address_space *io_space) override;
	void set_cpu_tag(const char *_cpu_tag);

protected:
	virtual void device_start() override;
	virtual void device_reset() override;

private:
	const char *cpu_tag;
	cpu_device *cpu;
};

extern const device_type NV2A_HOST;

#define MCFG_NV2A_HOST_ADD(_tag, _cpu_tag)  MCFG_PCI_HOST_ADD(_tag, NV2A_HOST, 0x10de02a5, 0, 0) \
	downcast<nv2a_host_device *>(device)->set_cpu_tag(_cpu_tag);

/*
 * Ram
 */

class nv2a_ram_device : public pci_device {
public:
	nv2a_ram_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual DECLARE_ADDRESS_MAP(config_map, 32) override;

protected:
	DECLARE_READ32_MEMBER(config_register_r);
	DECLARE_WRITE32_MEMBER(config_register_w);
};

extern const device_type NV2A_RAM;

/*
 * LPC Bus
 */

class mcpx_lpc_device : public pci_device {
public:
	mcpx_lpc_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	DECLARE_READ32_MEMBER(lpc_r);
	DECLARE_WRITE32_MEMBER(lpc_w);

protected:
	virtual void device_start() override;
	virtual void device_reset() override;

private:
	DECLARE_ADDRESS_MAP(lpc_io, 32);
};

extern const device_type MCPX_LPC;

/*
 * SMBus
 */

class mcpx_smbus_device : public pci_device {
public:
	mcpx_smbus_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	void register_device(int address, std::function<int(int command, int rw, int data)> callback) { if (address < 128) smbusst.devices[address] = callback; }

	template<class _Object> static devcb_base &set_interrupt_handler(device_t &device, _Object object) { return downcast<mcpx_smbus_device &>(device).m_interrupt_handler.set_callback(object); }

	DECLARE_READ32_MEMBER(smbus_r);
	DECLARE_WRITE32_MEMBER(smbus_w);

protected:
	virtual void device_start() override;
	virtual void device_reset() override;

private:
	devcb_write_line m_interrupt_handler;
	struct smbus_state {
		int status;
		int control;
		int address;
		int data;
		int command;
		int rw;
		std::function<int(int command, int rw, int data)> devices[128];
		uint32_t words[256 / 4];
	} smbusst;
	DECLARE_ADDRESS_MAP(smbus_io0, 32);
	DECLARE_ADDRESS_MAP(smbus_io1, 32);
	DECLARE_ADDRESS_MAP(smbus_io2, 32);
};

extern const device_type MCPX_SMBUS;

#define MCFG_MCPX_SMBUS_INTERRUPT_HANDLER(_devcb) \
	devcb = &mcpx_smbus_device::set_interrupt_handler(*device, DEVCB_##_devcb);

/*
 * OHCI USB Controller
 */
class usb_function_device;
class mcpx_ohci_device : public pci_device {
public:
	mcpx_ohci_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	void set_hack_callback(std::function<void(void)> hack) { hack_callback = hack; }
	void plug_usb_device(int port, ohci_function *function);

	template<class _Object> static devcb_base &set_interrupt_handler(device_t &device, _Object object) { return downcast<mcpx_ohci_device &>(device).m_interrupt_handler.set_callback(object); }

	DECLARE_READ32_MEMBER(ohci_r);
	DECLARE_WRITE32_MEMBER(ohci_w);

protected:
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_config_complete() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

private:
	ohci_usb_controller *ohci_usb;
	devcb_write_line m_interrupt_handler;
	emu_timer *timer;
	std::function<void(void)> hack_callback;
	DECLARE_ADDRESS_MAP(ohci_mmio, 32);
	struct dev_t {
		ohci_function *dev;
		int port;
	} connecteds[4];
	int connecteds_count;
};

extern const device_type MCPX_OHCI;

#define MCFG_MCPX_OHCI_INTERRUPT_HANDLER(_devcb) \
	devcb = &mcpx_ohci_device::set_interrupt_handler(*device, DEVCB_##_devcb);

/*
 * Ethernet
 */

class mcpx_eth_device : public pci_device {
public:
	mcpx_eth_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	DECLARE_READ32_MEMBER(eth_r);
	DECLARE_WRITE32_MEMBER(eth_w);
	DECLARE_READ32_MEMBER(eth_io_r);
	DECLARE_WRITE32_MEMBER(eth_io_w);

protected:
	virtual void device_start() override;
	virtual void device_reset() override;

private:
	DECLARE_ADDRESS_MAP(eth_mmio, 32);
	DECLARE_ADDRESS_MAP(eth_io, 32);
};

extern const device_type MCPX_ETH;

/*
 * Audio Processing Unit
 */

class mcpx_apu_device : public pci_device {
public:
	mcpx_apu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	void set_cpu_tag(const char *_cpu_tag);

	DECLARE_READ32_MEMBER(apu_r);
	DECLARE_WRITE32_MEMBER(apu_w);

protected:
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

private:
	const char *cpu_tag;
	cpu_device *cpu;
	// APU contains 3 dsps: voice processor (VP) global processor (GP) encode processor (EP)
	struct apu_state {
		uint32_t memory[0x60000 / 4];
		uint32_t gpdsp_sgaddress; // global processor scatter-gather
		uint32_t gpdsp_sgblocks;
		uint32_t gpdsp_address;
		uint32_t epdsp_sgaddress; // encoder processor scatter-gather
		uint32_t epdsp_sgblocks;
		uint32_t epdsp_sgaddress2;
		uint32_t epdsp_sgblocks2;
		int voice_number;
		uint32_t voices_heap_blockaddr[1024];
		uint64_t voices_active[4]; //one bit for each voice: 1 playing 0 not
		uint32_t voicedata_address;
		int voices_frequency[256]; // sample rate
		int voices_position[256]; // position in samples * 1000
		int voices_position_start[256]; // position in samples * 1000
		int voices_position_end[256]; // position in samples * 1000
		int voices_position_increment[256]; // position increment every 1ms * 1000
		emu_timer *timer;
		address_space *space;
	} apust;
	DECLARE_ADDRESS_MAP(apu_mmio, 32);
};

extern const device_type MCPX_APU;

#define MCFG_MCPX_APU_ADD(_tag, _cpu_tag)   MCFG_PCI_DEVICE_ADD(_tag, MCPX_APU, 0x10de01b0, 0, 0, 0) \
	downcast<mcpx_apu_device *>(device)->set_cpu_tag(_cpu_tag);

/*
 * AC97 Audio Controller
 */

class mcpx_ac97_audio_device : public pci_device {
public:
	mcpx_ac97_audio_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	DECLARE_READ32_MEMBER(ac97_audio_r);
	DECLARE_WRITE32_MEMBER(ac97_audio_w);
	DECLARE_READ32_MEMBER(ac97_audio_io0_r);
	DECLARE_WRITE32_MEMBER(ac97_audio_io0_w);
	DECLARE_READ32_MEMBER(ac97_audio_io1_r);
	DECLARE_WRITE32_MEMBER(ac97_audio_io1_w);

protected:
	virtual void device_start() override;
	virtual void device_reset() override;

private:
	struct ac97_state {
		uint32_t mixer_regs[0x84 / 4];
		uint32_t controller_regs[0x40 / 4];
	} ac97st;
	DECLARE_ADDRESS_MAP(ac97_mmio, 32);
	DECLARE_ADDRESS_MAP(ac97_io0, 32);
	DECLARE_ADDRESS_MAP(ac97_io1, 32);
};

extern const device_type MCPX_AC97_AUDIO;

/*
 * AC97 Modem Controller
 */

class mcpx_ac97_modem_device : public pci_device {
public:
	mcpx_ac97_modem_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};

extern const device_type MCPX_AC97_MODEM;

/*
 * IDE Controller
 */

class mcpx_ide_device : public pci_device {
public:
	mcpx_ide_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	template<class _Object> static devcb_base &set_interrupt_handler(device_t &device, _Object object) { return downcast<mcpx_ide_device &>(device).m_interrupt_handler.set_callback(object); }

protected:
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_add_mconfig(machine_config &config) override;

private:
	devcb_write_line m_interrupt_handler;
	DECLARE_ADDRESS_MAP(mcpx_ide_io, 32);
	DECLARE_WRITE_LINE_MEMBER(ide_interrupt);
};

extern const device_type MCPX_IDE;

#define MCFG_MCPX_IDE_INTERRUPT_HANDLER(_devcb) \
	devcb = &mcpx_ide_device::set_interrupt_handler(*device, DEVCB_##_devcb);

/*
 * AGP Bridge
 */

class nv2a_agp_device : public agp_bridge_device {
public:
	nv2a_agp_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	virtual void device_start() override;
	virtual void device_reset() override;
};

DECLARE_DEVICE_TYPE(NV2A_AGP, nv2a_agp_device)

/*
 * NV2A 3D Accelerator
 */

class nv2a_gpu_device : public pci_device {
public:
	nv2a_gpu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	void set_cpu_tag(const char *_cpu_tag);
	nv2a_renderer *debug_get_renderer() { return nvidia_nv2a; }

	template<class _Object> static devcb_base &set_interrupt_handler(device_t &device, _Object object) { return downcast<nv2a_gpu_device &>(device).m_interrupt_handler.set_callback(object); }

	DECLARE_READ32_MEMBER(geforce_r);
	DECLARE_WRITE32_MEMBER(geforce_w);
	DECLARE_READ32_MEMBER(nv2a_mirror_r);
	DECLARE_WRITE32_MEMBER(nv2a_mirror_w);

protected:
	virtual void device_start() override;
	virtual void device_reset() override;

private:
	nv2a_renderer *nvidia_nv2a;
	const char *cpu_tag;
	devcb_write_line m_interrupt_handler;
	address_space *m_program;
	DECLARE_ADDRESS_MAP(nv2a_mmio, 32);
	DECLARE_ADDRESS_MAP(nv2a_mirror, 32);
};

DECLARE_DEVICE_TYPE(NV2A_GPU, nv2a_gpu_device)

#define MCFG_MCPX_NV2A_GPU_CPU(_cpu_tag) \
	downcast<nv2a_gpu_device *>(device)->set_cpu_tag(_cpu_tag);
#define MCFG_MCPX_NV2A_GPU_INTERRUPT_HANDLER(_devcb) \
	devcb = &nv2a_gpu_device::set_interrupt_handler(*device, DEVCB_##_devcb);

#endif // MAME_INCLUDES_XBOX_PCI_H
