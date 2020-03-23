// license:BSD-3-Clause
// copyright-holders:David Haywood, Charles MacDonald
#ifndef MAME_MACHINE_DECO146_H
#define MAME_MACHINE_DECO146_H

#pragma once

#include "machine/gen_latch.h"

typedef device_delegate<uint16_t (int unused)> deco146_port_read_cb;
typedef device_delegate<void (address_space &space, uint16_t data, uint16_t mem_mask)> deco146_port_write_cb;


#define MCFG_DECO146_SET_PORTA_CALLBACK( _class, _method) \
	deco_146_base_device::set_port_a_cb(*device, deco146_port_read_cb(&_class::_method, #_class "::" #_method, nullptr, (_class *)nullptr));

#define MCFG_DECO146_SET_PORTB_CALLBACK( _class, _method) \
	deco_146_base_device::set_port_b_cb(*device, deco146_port_read_cb(&_class::_method, #_class "::" #_method, nullptr, (_class *)nullptr));

#define MCFG_DECO146_SET_PORTC_CALLBACK( _class, _method) \
	deco_146_base_device::set_port_c_cb(*device, deco146_port_read_cb(&_class::_method, #_class "::" #_method, nullptr, (_class *)nullptr));

#define MCFG_DECO146_SET_SOUNDLATCH_CALLBACK( _class, _method) \
	deco_146_base_device::set_soundlatch_cb(*device, deco146_port_write_cb(&_class::_method, #_class "::" #_method, nullptr, (_class *)nullptr));


// there are some standard ways the chip gets hooked up, so have them here ready to use
#define MCFG_DECO146_SET_INTERFACE_SCRAMBLE( a9,a8,a7,a6,a5,a4,a3,a2,a1,a0 ) \
	deco_146_base_device::set_interface_scramble(*device, a9,a8,a7,a6,a5,a4,a3,a2,a1,a0);

#define MCFG_DECO146_SET_INTERFACE_SCRAMBLE_REVERSE \
	deco_146_base_device::set_interface_scramble(*device, 0,1,2,3,4,5,6,7,8,9);

#define MCFG_DECO146_SET_INTERFACE_SCRAMBLE_INTERLEAVE \
	deco_146_base_device::set_interface_scramble(*device, 4,5,3,6,2,7,1,8,0,9 );

#define MCFG_DECO146_SET_USE_MAGIC_ADDRESS_XOR \
	deco_146_base_device::set_use_magic_read_address_xor(*device, 1 );




#define BLK (0xff)
#define INPUT_PORT_A (-1)
#define INPUT_PORT_B (-2)
#define INPUT_PORT_C (-3)


#define NIB3__ 0xc, 0xd, 0xe, 0xf
#define NIB3R1 0xd, 0xe, 0xf, 0xc
#define NIB3R2 0xe, 0xf, 0xc, 0xd
#define NIB3R3 0xf, 0xc, 0xd, 0xe

#define NIB2__ 0x8, 0x9, 0xa, 0xb
#define NIB2R1 0x9, 0xa, 0xb, 0x8
#define NIB2R2 0xa, 0xb, 0x8, 0x9
#define NIB2R3 0xb, 0x8, 0x9, 0xa

#define NIB1__ 0x4, 0x5, 0x6, 0x7
#define NIB1R1 0x5, 0x6, 0x7, 0x4
#define NIB1R2 0x6, 0x7, 0x4, 0x5
#define NIB1R3 0x7, 0x4, 0x5, 0x6

#define NIB0__ 0x0, 0x1, 0x2, 0x3
#define NIB0R1 0x1, 0x2, 0x3, 0x0
#define NIB0R2 0x2, 0x3, 0x0, 0x1
#define NIB0R3 0x3, 0x0, 0x1, 0x2

#define BLANK_ BLK, BLK, BLK, BLK

struct deco146port_xx
{
	int write_offset;
	uint8_t mapping[16];
	int use_xor;
	int use_nand;
};



/* Data East 146 protection chip */

class deco_146_base_device : public device_t
{
public:
	void write_data(address_space &space, uint16_t address, uint16_t data, uint16_t mem_mask, uint8_t &csflags);
	uint16_t read_data(uint16_t address, uint16_t mem_mask, uint8_t &csflags);

	static void set_port_a_cb(device_t &device,deco146_port_read_cb port_cb);
	static void set_port_b_cb(device_t &device,deco146_port_read_cb port_cb);
	static void set_port_c_cb(device_t &device,deco146_port_read_cb port_cb);
	static void set_soundlatch_cb(device_t &device,deco146_port_write_cb port_cb);
	static void set_interface_scramble(device_t &device,uint8_t a9, uint8_t a8, uint8_t a7, uint8_t a6, uint8_t a5, uint8_t a4, uint8_t a3,uint8_t a2,uint8_t a1,uint8_t a0);
	static void set_use_magic_read_address_xor(device_t &device, int use_xor);





	deco146_port_read_cb m_port_a_r;
	deco146_port_read_cb m_port_b_r;
	deco146_port_read_cb m_port_c_r;
	deco146_port_write_cb m_soundlatch_w;

	uint16_t port_a_default(int unused);
	uint16_t port_b_default(int unused);
	uint16_t port_c_default(int unused);
	uint16_t port_dummy_cb(int unused);
	void soundlatch_default(address_space &space, uint16_t data, uint16_t mem_mask);
	void soundlatch_dummy(address_space &space, uint16_t data, uint16_t mem_mask);

	uint8_t m_bankswitch_swap_read_address;
	uint16_t m_magic_read_address_xor;
	int m_magic_read_address_xor_enabled;
	uint8_t m_xor_port;
	uint8_t m_mask_port;
	uint8_t m_soundlatch_port;


	uint8_t m_external_addrswap[10];

	deco146port_xx const *m_lookup_table;

protected:
	deco_146_base_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	virtual void device_start() override;
	virtual void device_reset() override;

	uint16_t read_protport(uint16_t address, uint16_t mem_mask);
	virtual void write_protport(address_space &space, uint16_t address, uint16_t data, uint16_t mem_mask);
	virtual uint16_t read_data_getloc(uint16_t address, int& location);

	uint16_t m_rambank0[0x80];
	uint16_t m_rambank1[0x80];

	int m_current_rambank;


	uint16_t m_nand;
	uint16_t m_xor;
	uint16_t m_soundlatch;

	uint16_t m_latchaddr;
	uint16_t m_latchdata;

	uint8_t m_configregion; // which value of upper 4 address lines accesses the config region
	int m_latchflag;
private:
	uint8_t region_selects[6];
	optional_device<generic_latch_8_device> m_sound_latch;

};

class deco146_device : public deco_146_base_device
{
public:
	deco146_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};

extern const device_type DECO146PROT;
DECLARE_DEVICE_TYPE(DECO146PROT, deco146_device)

#define MCFG_DECO146_ADD(_tag) \
	MCFG_DEVICE_ADD(_tag, DECO146PROT, 0)

#endif // MAME_MACHINE_DECO146_H
