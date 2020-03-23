// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/***************************************************************************

    Sega 16-bit common hardware

***************************************************************************/

#ifndef MAME_MACHINE_SEGAIC16_H
#define MAME_MACHINE_SEGAIC16_H

#pragma once

#include "cpu/m68000/m68000.h"
#include "machine/fd1089.h"
#include "machine/fd1094.h"
#include "screen.h"


//**************************************************************************
//  INTERFACE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_SEGA_315_5195_MAPPER_ADD(_tag, _cputag, _class, _mapper, _read, _write) \
	MCFG_DEVICE_ADD(_tag, SEGA_315_5195_MEM_MAPPER, 0) \
	sega_315_5195_mapper_device::static_set_cputag(*device, "^" _cputag); \
	sega_315_5195_mapper_device::static_set_mapper(*device, sega_315_5195_mapper_device::mapper_delegate(&_class::_mapper, #_class "::" #_mapper, nullptr, (_class *)nullptr)); \
	sega_315_5195_mapper_device::static_set_sound_readwrite(*device, sega_315_5195_mapper_device::sound_read_delegate(&_class::_read, #_class "::" #_read, nullptr, (_class *)nullptr), sega_315_5195_mapper_device::sound_write_delegate(&_class::_write, #_class "::" #_write, nullptr, (_class *)nullptr));

#define MCFG_SEGA_315_5248_MULTIPLIER_ADD(_tag) \
	MCFG_DEVICE_ADD(_tag, SEGA_315_5248_MULTIPLIER, 0)

#define MCFG_SEGA_315_5249_DIVIDER_ADD(_tag) \
	MCFG_DEVICE_ADD(_tag, SEGA_315_5249_DIVIDER, 0)

#define MCFG_SEGA_315_5250_COMPARE_TIMER_ADD(_tag) \
	MCFG_DEVICE_ADD(_tag, SEGA_315_5250_COMPARE_TIMER, 0)
#define MCFG_SEGA_315_5250_TIMER_ACK(_class, _func) \
	sega_315_5250_compare_timer_device::static_set_timer_ack(*device, sega_315_5250_compare_timer_device::timer_ack_delegate(&_class::_func, #_class "::" #_func, nullptr, (_class *)nullptr));
#define MCFG_SEGA_315_5250_SOUND_WRITE(_class, _func) \
	sega_315_5250_compare_timer_device::static_set_sound_write(*device, sega_315_5250_compare_timer_device::sound_write_delegate(&_class::_func, #_class "::" #_func, nullptr, (_class *)nullptr));


//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************


// ======================> sega_16bit_common_base

class sega_16bit_common_base : public driver_device
{
public:
	// open bus read helpers
	DECLARE_READ16_MEMBER( open_bus_r );

	// palette helpers
	DECLARE_WRITE16_MEMBER( paletteram_w );
	DECLARE_WRITE16_MEMBER( philko_paletteram_w );

protected:
	// construction/destruction
	sega_16bit_common_base(const machine_config &mconfig, device_type type, const char *tag);

	// internal helpers
	void palette_init();

public: // -- stupid system16.c
	// memory pointers
	required_shared_ptr<uint16_t> m_paletteram;
protected:

	// internal state
	bool        m_open_bus_recurse;         // flag to track recursion through open_bus_r
	uint32_t      m_palette_entries;          // number of palette entries
	uint8_t       m_palette_normal[32];       // RGB translations for normal pixels
	uint8_t       m_palette_shadow[32];       // RGB translations for shadowed pixels
	uint8_t       m_palette_hilight[32];      // RGB translations for hilighted pixels
	required_device<screen_device> m_screen;
	required_device<palette_device> m_palette;
};


// ======================> sega_315_5195_mapper_device

class sega_315_5195_mapper_device : public device_t
{
public:
	typedef device_delegate<void (sega_315_5195_mapper_device &, uint8_t)> mapper_delegate;
	typedef device_delegate<uint8_t ()> sound_read_delegate;
	typedef device_delegate<void (uint8_t)> sound_write_delegate;

	// construction/destruction
	sega_315_5195_mapper_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// static configuration helpers
	static void static_set_cputag(device_t &device, const char *cpu);
	static void static_set_mapper(device_t &device, mapper_delegate callback);
	static void static_set_sound_readwrite(device_t &device, sound_read_delegate read, sound_write_delegate write);

	// public interface
	DECLARE_READ8_MEMBER( read );
	DECLARE_WRITE8_MEMBER( write );

	// mapping helpers
	void map_as_rom(uint32_t offset, uint32_t length, offs_t mirror, const char *bank_name, const char *decrypted_bank_name, offs_t rgnoffset, write16_delegate whandler);
	void map_as_ram(uint32_t offset, uint32_t length, offs_t mirror, const char *bank_share_name, write16_delegate whandler);
	void map_as_handler(uint32_t offset, uint32_t length, offs_t mirror, read16_delegate rhandler, write16_delegate whandler);

	// perform an explicit configuration (for bootlegs with hard-coded mappings)
	void configure_explicit(const uint8_t *map_data);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

private:
	// internal region struct
	struct region_info
	{
		offs_t  size_mask;
		offs_t  base;
		offs_t  mirror;
		offs_t  start;
		offs_t  end;
	};

	// helper class for tracking banks mapped to ROM regions
	class decrypt_bank
	{
	public:
		// construction/destruction
		decrypt_bank();
		~decrypt_bank();

		// configuration
		void set_decrypt(fd1089_base_device *fd1089);
		void set_decrypt(fd1094_device *fd1094);
		void clear() { set(nullptr, nullptr, 0, 0, ~0, nullptr); }
		void set(memory_bank *bank, memory_bank *decrypted_bank, offs_t start, offs_t end, offs_t rgnoffs, uint8_t *src);

		// updating
		void update();
		void reset() { m_fd1089_decrypted.clear(); if (m_fd1094_cache != nullptr) m_fd1094_cache->reset(); }

	private:
		// internal state
		memory_bank *           m_bank;
		memory_bank *           m_decrypted_bank;
		offs_t                  m_start;
		offs_t                  m_end;
		offs_t                  m_rgnoffs;
		uint8_t *                 m_srcptr;
		fd1089_base_device *    m_fd1089;
		std::vector<uint16_t>   m_fd1089_decrypted;
		std::unique_ptr<fd1094_decryption_cache> m_fd1094_cache;
	};

	// internal helpers
	void compute_region(region_info &info, uint8_t index, uint32_t length, uint32_t mirror, uint32_t offset = 0);
	void update_mapping();
	void fd1094_state_change(uint8_t state);

	// configuration
	required_device<m68000_device> m_cpu;
	required_memory_region      m_cpuregion;
	mapper_delegate             m_mapper;
	sound_read_delegate         m_sound_read;
	sound_write_delegate        m_sound_write;

	// internal state
	address_space *             m_space;
	address_space *             m_decrypted_space;
	uint8_t                       m_regs[0x20];
	uint8_t                       m_curregion;
	decrypt_bank                m_banks[8];
};


// ======================> sega_315_5248_multiplier_device

class sega_315_5248_multiplier_device : public device_t
{
public:
	// construction/destruction
	sega_315_5248_multiplier_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// public interface
	DECLARE_READ16_MEMBER( read );
	DECLARE_WRITE16_MEMBER( write );

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

private:
	// internal state
	uint16_t                      m_regs[4];
};


// ======================> sega_315_5249_divider_device

class sega_315_5249_divider_device : public device_t
{
public:
	// construction/destruction
	sega_315_5249_divider_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// public interface
	DECLARE_READ16_MEMBER( read );
	DECLARE_WRITE16_MEMBER( write );

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

private:
	// internal helpers
	void execute(int mode);

	// internal state
	uint16_t                      m_regs[8];
};


// ======================> sega_315_5250_compare_timer_device

class sega_315_5250_compare_timer_device : public device_t
{
public:
	typedef device_delegate<void (uint8_t)> sound_write_delegate;
	typedef device_delegate<void ()> timer_ack_delegate;

	// construction/destruction
	sega_315_5250_compare_timer_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// static configuration helpers
	static void static_set_timer_ack(device_t &device, timer_ack_delegate callback);
	static void static_set_sound_write(device_t &device, sound_write_delegate write);

	// public interface
	bool clock();
	DECLARE_READ16_MEMBER( read );
	DECLARE_WRITE16_MEMBER( write );

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

private:
	// internal helpers
	void execute(bool update_history = false);
	void interrupt_ack() { if (!m_timer_ack.isnull()) m_timer_ack(); }

	// configuration
	timer_ack_delegate          m_timer_ack;
	sound_write_delegate        m_sound_write;

	// internal state
	uint16_t                      m_regs[16];
	uint16_t                      m_counter;
	uint8_t                       m_bit;
};


// device type definition
DECLARE_DEVICE_TYPE(SEGA_315_5195_MEM_MAPPER,    sega_315_5195_mapper_device)
DECLARE_DEVICE_TYPE(SEGA_315_5248_MULTIPLIER,    sega_315_5248_multiplier_device)
DECLARE_DEVICE_TYPE(SEGA_315_5249_DIVIDER,       sega_315_5249_divider_device)
DECLARE_DEVICE_TYPE(SEGA_315_5250_COMPARE_TIMER, sega_315_5250_compare_timer_device)


#endif // MAME_MACHINE_SEGAIC16_H
