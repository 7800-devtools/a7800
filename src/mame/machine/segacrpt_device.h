// license:BSD-3-Clause
// copyright-holders:David Haywood

#ifndef MAME_MACHINE_SEGACRYPT_DEVICE_H
#define MAME_MACHINE_SEGACRYPT_DEVICE_H

#pragma once

#include "cpu/z80/z80.h"


#define MCFG_SEGACRPT_SET_DECRYPTED_TAG(_tag) \
	segacrpt_z80_device::set_decrypted_tag(*device, _tag);

#define MCFG_SEGACRPT_SET_DECRYPTED_PTR(_tag) \
	segacrpt_z80_device::set_decrypted_ptr(*device, _ptr);

#define MCFG_SEGACRPT_SET_SIZE(_size) \
	segacrpt_z80_device::set_size(*device, _size);

#define MCFG_SEGACRPT_SET_NUMBANKS(_numbanks) \
	segacrpt_z80_device::set_numbanks(*device, _numbanks);

#define MCFG_SEGACRPT_SET_BANKSIZE(_banksize) \
	segacrpt_z80_device::set_banksize(*device, _banksize);


// base class
class segacrpt_z80_device : public z80_device
{
public:
	static void set_decrypted_tag(device_t &device, const char* decrypted_tag);
	static void set_decrypted_ptr(device_t &device, uint8_t* ptr); // toprollr
	static void set_size(device_t &device, int size);
	static void set_numbanks(device_t &device, int _numbanks);
	static void set_banksize(device_t &device, int _banksize);

	void set_decrypted_p(uint8_t* ptr);
	void set_region_p(uint8_t* ptr);

protected:
	segacrpt_z80_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void decrypt() = 0;

	const char*         m_decrypted_tag;
	uint8_t* m_decrypted_ptr;
	uint8_t* m_region_ptr;
	int m_decode_size;
	int m_numbanks;
	int m_banksize;

private:
	bool m_decryption_done;
};


// actual encrypted CPUs
class sega_315_5132_device : public segacrpt_z80_device
{
public:
	sega_315_5132_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t);
protected:
	virtual void decrypt() override;
};

class sega_315_5155_device : public segacrpt_z80_device
{
public:
	sega_315_5155_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t);
protected:
	virtual void decrypt() override;
};

class sega_315_5110_device : public segacrpt_z80_device
{
public:
	sega_315_5110_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t);
protected:
	virtual void decrypt() override;
};

class sega_315_5135_device : public segacrpt_z80_device
{
public:
	sega_315_5135_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t);
protected:
	virtual void decrypt() override;
};

class sega_315_5051_device : public segacrpt_z80_device
{
public:
	sega_315_5051_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t);
protected:
	virtual void decrypt() override;
};

class sega_315_5098_device : public segacrpt_z80_device
{
public:
	sega_315_5098_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t);
protected:
	virtual void decrypt() override;
};

class sega_315_5102_device : public segacrpt_z80_device
{
public:
	sega_315_5102_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t);
protected:
	virtual void decrypt() override;
};

class sega_315_5065_device : public segacrpt_z80_device
{
public:
	sega_315_5065_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t);
protected:
	virtual void decrypt() override;
};


class sega_315_5064_device : public segacrpt_z80_device
{
public:
	sega_315_5064_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t);
protected:
	virtual void decrypt() override;
};


class sega_315_5033_device : public segacrpt_z80_device
{
public:
	sega_315_5033_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t);
protected:
	virtual void decrypt() override;
};

class sega_315_5041_device : public segacrpt_z80_device
{
public:
	sega_315_5041_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t);
protected:
	virtual void decrypt() override;
};

class sega_315_5048_device : public segacrpt_z80_device
{
public:
	sega_315_5048_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t);
	sega_315_5048_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

protected:
	virtual void decrypt() override;
};

class sega_315_5093_device : public segacrpt_z80_device
{
public:
	sega_315_5093_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t);
protected:
	virtual void decrypt() override;
};

class sega_315_5099_device : public segacrpt_z80_device
{
public:
	sega_315_5099_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t);
protected:
	virtual void decrypt() override;
};

class sega_315_spat_device : public segacrpt_z80_device
{
public:
	sega_315_spat_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t);
protected:
	virtual void decrypt() override;
};

class sega_315_5015_device : public segacrpt_z80_device
{
public:
	sega_315_5015_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t);
protected:
	virtual void decrypt() override;
};


class sega_315_5133_device : public sega_315_5048_device
{
public:
	sega_315_5133_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t);
};

class sega_315_5014_device : public segacrpt_z80_device
{
public:
	sega_315_5014_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t);
protected:
	virtual void decrypt() override;
};

class sega_315_5013_device : public segacrpt_z80_device
{
public:
	sega_315_5013_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t);
protected:
	virtual void decrypt() override;
};

class sega_315_5061_device : public segacrpt_z80_device
{
public:
	sega_315_5061_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t);
protected:
	virtual void decrypt() override;
};



class sega_315_5018_device : public segacrpt_z80_device
{
public:
	sega_315_5018_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t);
protected:
	virtual void decrypt() override;
};

class sega_315_5010_device : public segacrpt_z80_device
{
public:
	sega_315_5010_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t);
protected:
	virtual void decrypt() override;
};



class sega_cpu_pbactio4_device : public segacrpt_z80_device
{
public:
	sega_cpu_pbactio4_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t);
protected:
	virtual void decrypt() override;
};

class sega_315_5028_device : public segacrpt_z80_device
{
public:
	sega_315_5028_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t);
protected:
	virtual void decrypt() override;
};

class sega_315_5084_device : public segacrpt_z80_device
{
public:
	sega_315_5084_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t);
protected:
	virtual void decrypt() override;
};


DECLARE_DEVICE_TYPE(SEGA_315_5132,     sega_315_5132_device)
DECLARE_DEVICE_TYPE(SEGA_315_5155,     sega_315_5155_device)
DECLARE_DEVICE_TYPE(SEGA_315_5110,     sega_315_5110_device)
DECLARE_DEVICE_TYPE(SEGA_315_5135,     sega_315_5135_device)
DECLARE_DEVICE_TYPE(SEGA_315_5051,     sega_315_5051_device)
DECLARE_DEVICE_TYPE(SEGA_315_5098,     sega_315_5098_device)
DECLARE_DEVICE_TYPE(SEGA_315_5102,     sega_315_5102_device)
DECLARE_DEVICE_TYPE(SEGA_315_5065,     sega_315_5065_device)
DECLARE_DEVICE_TYPE(SEGA_315_5064,     sega_315_5064_device)
DECLARE_DEVICE_TYPE(SEGA_315_5033,     sega_315_5033_device)
DECLARE_DEVICE_TYPE(SEGA_315_5041,     sega_315_5041_device)
DECLARE_DEVICE_TYPE(SEGA_315_5048,     sega_315_5048_device)
DECLARE_DEVICE_TYPE(SEGA_315_5093,     sega_315_5093_device)
DECLARE_DEVICE_TYPE(SEGA_315_5099,     sega_315_5099_device)
DECLARE_DEVICE_TYPE(SEGA_315_SPAT,     sega_315_spat_device)
DECLARE_DEVICE_TYPE(SEGA_315_5015,     sega_315_5015_device)
DECLARE_DEVICE_TYPE(SEGA_315_5133,     sega_315_5133_device)
DECLARE_DEVICE_TYPE(SEGA_315_5014,     sega_315_5014_device)
DECLARE_DEVICE_TYPE(SEGA_315_5013,     sega_315_5013_device)
DECLARE_DEVICE_TYPE(SEGA_315_5061,     sega_315_5061_device)
DECLARE_DEVICE_TYPE(SEGA_315_5018,     sega_315_5018_device)
DECLARE_DEVICE_TYPE(SEGA_315_5010,     sega_315_5010_device)
DECLARE_DEVICE_TYPE(SEGA_CPU_PBACTIO4, sega_cpu_pbactio4_device)
DECLARE_DEVICE_TYPE(SEGA_315_5028,     sega_315_5028_device)
DECLARE_DEVICE_TYPE(SEGA_315_5084,     sega_315_5084_device)


#endif // MAME_MACHINE_SEGACRYPT_DEVICE_H
