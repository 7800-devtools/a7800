// license:BSD-3-Clause
// copyright-holders:David Haywood
#ifndef MAME_MACHINE_SEGACRP2_DEVICE_H
#define MAME_MACHINE_SEGACRP2_DEVICE_H

#pragma once


#include "cpu/z80/z80.h"


#define MCFG_SEGAZ80_SET_DECRYPTED_TAG(_tag) \
	segacrp2_z80_device::set_decrypted_tag(*device, _tag);

// base class
class segacrp2_z80_device : public z80_device
{
public:
	static void set_decrypted_tag(device_t &device, const char* decrypted_tag);
	const char*         m_decrypted_tag;
protected:
	segacrp2_z80_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void decrypt();
};



// actual encrypted CPUs
class sega_315_5179_device : public segacrp2_z80_device
{
public:
	sega_315_5179_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t);
protected:
	virtual void decrypt() override;
};


class sega_315_5178_device : public segacrp2_z80_device
{
public:
	sega_315_5178_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t);
protected:
	virtual void decrypt() override;
};

class sega_315_5177_device : public segacrp2_z80_device
{
public:
	sega_315_5177_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t);
protected:
	virtual void decrypt() override;
};

class sega_315_5176_device : public segacrp2_z80_device
{
public:
	sega_315_5176_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t);
protected:
	virtual void decrypt() override;
};

class sega_315_5162_device : public segacrp2_z80_device
{
public:
	sega_315_5162_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t);
protected:
	virtual void decrypt() override;
};

class sega_317_0004_device : public segacrp2_z80_device
{
public:
	sega_317_0004_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t);
protected:
	virtual void decrypt() override;
};


class sega_317_0005_device : public segacrp2_z80_device
{
public:
	sega_317_0005_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t);
protected:
	virtual void decrypt() override;
};


class sega_317_0006_device : public segacrp2_z80_device
{
public:
	sega_317_0006_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t);
protected:
	virtual void decrypt() override;
};

class sega_317_0007_device : public segacrp2_z80_device
{
public:
	sega_317_0007_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t);
protected:
	virtual void decrypt() override;
};


DECLARE_DEVICE_TYPE(SEGA_315_5179, sega_315_5179_device)
DECLARE_DEVICE_TYPE(SEGA_315_5178, sega_315_5178_device)
DECLARE_DEVICE_TYPE(SEGA_315_5177, sega_315_5177_device)
DECLARE_DEVICE_TYPE(SEGA_315_5176, sega_315_5176_device)
DECLARE_DEVICE_TYPE(SEGA_315_5162, sega_315_5162_device)

DECLARE_DEVICE_TYPE(SEGA_317_0004, sega_317_0004_device)
DECLARE_DEVICE_TYPE(SEGA_317_0005, sega_317_0005_device)
DECLARE_DEVICE_TYPE(SEGA_317_0006, sega_317_0006_device)
DECLARE_DEVICE_TYPE(SEGA_317_0007, sega_317_0007_device)


#endif // MAME_MACHINE_SEGACRP2_DEVICE_H
