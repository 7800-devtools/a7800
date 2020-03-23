// license:BSD-3-Clause
// copyright-holders:Samuele Zannoli
#ifndef MAME_INCLUDES_XBOX_H
#define MAME_INCLUDES_XBOX_H

#pragma once

#include "xbox_nv2a.h"
#include "xbox_usb.h"

#include "machine/idectrl.h"
#include "machine/pic8259.h"
#include "machine/pci.h"

class xbox_base_state : public driver_device
{
public:
	xbox_base_state(const machine_config &mconfig, device_type type, const char *tag) :
		driver_device(mconfig, type, tag),
		nvidia_nv2a(nullptr),
		debug_irq_active(false),
		debug_irq_number(0),
		m_maincpu(*this, "maincpu"),
		debugc_bios(nullptr) { }

	DECLARE_READ8_MEMBER(superio_read);
	DECLARE_WRITE8_MEMBER(superio_write);
	DECLARE_READ8_MEMBER(superiors232_read);
	DECLARE_WRITE8_MEMBER(superiors232_write);

	int smbus_pic16lc(int command, int rw, int data);
	int smbus_cx25871(int command, int rw, int data);
	int smbus_eeprom(int command, int rw, int data);
	void debug_generate_irq(int irq, bool active);
	virtual void hack_eeprom() {};
	virtual void hack_usb() {};

	DECLARE_WRITE_LINE_MEMBER(vblank_callback);
	uint32_t screen_update_callback(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);

	virtual void machine_start() override;
	DECLARE_WRITE_LINE_MEMBER(xbox_pic8259_1_set_int_line);
	DECLARE_READ8_MEMBER(get_slave_ack);
	DECLARE_WRITE_LINE_MEMBER(xbox_pit8254_out0_changed);
	DECLARE_WRITE_LINE_MEMBER(xbox_pit8254_out2_changed);
	DECLARE_WRITE_LINE_MEMBER(xbox_ohci_usb_interrupt_changed);
	DECLARE_WRITE_LINE_MEMBER(xbox_smbus_interrupt_changed);
	DECLARE_WRITE_LINE_MEMBER(xbox_nv2a_interrupt_changed);
	IRQ_CALLBACK_MEMBER(irq_callback);

	struct xbox_devices {
		pic8259_device    *pic8259_1;
		pic8259_device    *pic8259_2;
		bus_master_ide_controller_device    *ide;
	} xbox_base_devs;
	struct superio_state
	{
		bool configuration_mode;
		int index;
		int selected;
		uint8_t registers[16][256]; // 256 registers for up to 16 devices, registers 0-0x2f common to all
	} superiost;
	uint8_t pic16lc_buffer[0xff];
	nv2a_renderer *nvidia_nv2a;
	bool debug_irq_active;
	int debug_irq_number;
	required_device<cpu_device> m_maincpu;
	static const struct debugger_constants
	{
		uint32_t id;
		uint32_t parameter[8]; // c c c ? ? ? x x
	} debugp[];
	const debugger_constants *debugc_bios;

private:
	void dump_string_command(int ref, const std::vector<std::string> &params);
	void dump_process_command(int ref, const std::vector<std::string> &params);
	void dump_list_command(int ref, const std::vector<std::string> &params);
	void dump_dpc_command(int ref, const std::vector<std::string> &params);
	void dump_timer_command(int ref, const std::vector<std::string> &params);
	void curthread_command(int ref, const std::vector<std::string> &params);
	void threadlist_command(int ref, const std::vector<std::string> &params);
	void generate_irq_command(int ref, const std::vector<std::string> &params);
	void nv2a_combiners_command(int ref, const std::vector<std::string> &params);
	void nv2a_wclipping_command(int ref, const std::vector<std::string> &params);
	void waitvblank_command(int ref, const std::vector<std::string> &params);
	void grab_texture_command(int ref, const std::vector<std::string> &params);
	void grab_vprog_command(int ref, const std::vector<std::string> &params);
	void vprogdis_command(int ref, const std::vector<std::string> &params);
	void help_command(int ref, const std::vector<std::string> &params);
	void xbox_debug_commands(int ref, const std::vector<std::string> &params);
	int find_bios_index(running_machine &mach);
	bool find_bios_hash(running_machine &mach, int bios, uint32_t &crc32);
	void find_debug_params(running_machine &mach);
};

ADDRESS_MAP_EXTERN(xbox_base_map, 32);
ADDRESS_MAP_EXTERN(xbox_base_map_io, 32);
MACHINE_CONFIG_EXTERN(xbox_base);

#endif // MAME_INCLUDES_XBOX_H
