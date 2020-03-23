// license:GPL-2.0+
// copyright-holders:Kevin Thacker,Sandro Ronco
/*****************************************************************************
 *
 * includes/z88.h
 *
 ****************************************************************************/

#ifndef MAME_INCLUDES_Z88_H
#define MAME_INCLUDES_Z88_H

#include "cpu/z80/z80.h"
#include "machine/ram.h"
#include "machine/upd65031.h"
#include "sound/spkrdev.h"

#include "bus/z88/flash.h"
#include "bus/z88/ram.h"
#include "bus/z88/rom.h"
#include "bus/z88/z88.h"

#include "rendlay.h"


#define Z88_NUM_COLOURS 3

#define Z88_SCREEN_WIDTH        640
#define Z88_SCREEN_HEIGHT       64

#define Z88_SCR_HW_REV  (1<<4)
#define Z88_SCR_HW_HRS  (1<<5)
#define Z88_SCR_HW_UND  (1<<1)
#define Z88_SCR_HW_FLS  (1<<3)
#define Z88_SCR_HW_GRY  (1<<2)
#define Z88_SCR_HW_CURS (Z88_SCR_HW_HRS|Z88_SCR_HW_FLS|Z88_SCR_HW_REV)
#define Z88_SCR_HW_NULL (Z88_SCR_HW_HRS|Z88_SCR_HW_GRY|Z88_SCR_HW_REV)

enum
{
	Z88_BANK_ROM = 1,
	Z88_BANK_RAM,
	Z88_BANK_CART,
	Z88_BANK_UNMAP
};


class z88_state : public driver_device
{
public:
	z88_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
			m_maincpu(*this, "maincpu"),
			m_ram(*this, RAM_TAG),
			m_palette(*this, "palette")
	{ }

	required_device<cpu_device> m_maincpu;
	required_device<ram_device> m_ram;
	required_device<palette_device> m_palette;

	virtual void machine_start() override;
	virtual void machine_reset() override;
	DECLARE_READ8_MEMBER(kb_r);
	UPD65031_MEMORY_UPDATE(bankswitch_update);
	UPD65031_SCREEN_UPDATE(lcd_update);

	// cartridges read/write
	DECLARE_READ8_MEMBER(bank0_cart_r);
	DECLARE_READ8_MEMBER(bank1_cart_r);
	DECLARE_READ8_MEMBER(bank2_cart_r);
	DECLARE_READ8_MEMBER(bank3_cart_r);
	DECLARE_WRITE8_MEMBER(bank0_cart_w);
	DECLARE_WRITE8_MEMBER(bank1_cart_w);
	DECLARE_WRITE8_MEMBER(bank2_cart_w);
	DECLARE_WRITE8_MEMBER(bank3_cart_w);

	// defined in video/z88.c
	inline void plot_pixel(bitmap_ind16 &bitmap, int x, int y, uint16_t color);
	inline uint8_t* convert_address(uint32_t offset);
	void vh_render_8x8(bitmap_ind16 &bitmap, int x, int y, uint16_t pen0, uint16_t pen1, uint8_t *gfx);
	void vh_render_6x8(bitmap_ind16 &bitmap, int x, int y, uint16_t pen0, uint16_t pen1, uint8_t *gfx);
	void vh_render_line(bitmap_ind16 &bitmap, int x, int y, uint16_t pen);

	struct
	{
		uint8_t slot;
		uint8_t page;
	} m_bank[4];

	int                   m_bank_type[4];
	uint8_t *               m_bios;
	uint8_t *               m_ram_base;
	z88cart_slot_device * m_carts[4];
	DECLARE_PALETTE_INIT(z88);
};

#endif /* MAME_INCLUDES_Z88_H */
