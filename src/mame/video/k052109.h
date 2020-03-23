// license:BSD-3-Clause
// copyright-holders:Fabio Priuli,Acho A. Tang, R. Belmont
#ifndef MAME_VIDEO_K052109_H
#define MAME_VIDEO_K052109_H

#pragma once

#include "screen.h"

typedef device_delegate<void (int layer, int bank, int *code, int *color, int *flags, int *priority)> k052109_cb_delegate;
#define K052109_CB_MEMBER(_name)   void _name(int layer, int bank, int *code, int *color, int *flags, int *priority)

#define MCFG_K052109_CB(_class, _method) \
	k052109_device::set_k052109_callback(*device, k052109_cb_delegate(&_class::_method, #_class "::" #_method, downcast<_class *>(owner)));

#define MCFG_K052109_CHARRAM(_ram) \
	k052109_device::set_ram(*device, _ram);

#define MCFG_K052109_SCREEN_TAG(_tag) \
	k052109_device::set_screen_tag(*device, "^" _tag);

#define MCFG_K052109_IRQ_HANDLER(_devcb) \
	devcb = &k052109_device::set_irq_handler(*device, DEVCB_##_devcb);


class k052109_device : public device_t, public device_gfx_interface
{
	static const gfx_layout charlayout;
	static const gfx_layout charlayout_ram;
	DECLARE_GFXDECODE_MEMBER(gfxinfo);
	DECLARE_GFXDECODE_MEMBER(gfxinfo_ram);

public:
	k052109_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	~k052109_device() {}

	template <class Object> static devcb_base &set_irq_handler(device_t &device, Object &&cb)
	{ return downcast<k052109_device &>(device).m_irq_handler.set_callback(std::forward<Object>(cb)); }

	static void set_k052109_callback(device_t &device, k052109_cb_delegate callback) { downcast<k052109_device &>(device).m_k052109_cb = callback; }
	static void set_ram(device_t &device, bool ram);
	static void set_screen_tag(device_t &device, const char *tag);

	/*
	The callback is passed:
	- layer number (0 = FIX, 1 = A, 2 = B)
	- bank (range 0-3, output of the pins CAB1 and CAB2)
	- code (range 00-FF, output of the pins VC3-VC10)
	NOTE: code is in the range 0000-FFFF for X-Men, which uses extra RAM
	- color (range 00-FF, output of the pins COL0-COL7)
	The callback must put:
	- in code the resulting tile number
	- in color the resulting color index
	- if necessary, put flags and/or priority for the TileMap code in the tile_info
	structure (e.g. TILE_FLIPX). Note that TILE_FLIPY is handled internally by the
	chip so it must not be set by the callback.
	*/

	DECLARE_READ8_MEMBER( read );
	DECLARE_WRITE8_MEMBER( write );
	DECLARE_READ16_MEMBER( word_r );
	DECLARE_WRITE16_MEMBER( word_w );
	DECLARE_READ16_MEMBER( lsb_r );
	DECLARE_WRITE16_MEMBER( lsb_w );

	void set_rmrd_line(int state);
	int get_rmrd_line();
	void tilemap_update();
	int is_irq_enabled();
	void tilemap_mark_dirty(int tmap_num);
	void tilemap_draw(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect, int tmap_num, uint32_t flags, uint8_t priority);

	void vblank_callback(screen_device &screen, bool state);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

private:
	// internal state
	std::unique_ptr<uint8_t[]>   m_ram;
	uint8_t    *m_videoram_F;
	uint8_t    *m_videoram_A;
	uint8_t    *m_videoram_B;
	uint8_t    *m_videoram2_F;
	uint8_t    *m_videoram2_A;
	uint8_t    *m_videoram2_B;
	uint8_t    *m_colorram_F;
	uint8_t    *m_colorram_A;
	uint8_t    *m_colorram_B;

	tilemap_t  *m_tilemap[3];
	int      m_tileflip_enable;
	uint8_t    m_charrombank[4];
	uint8_t    m_charrombank_2[4];
	uint8_t    m_has_extra_video_ram;
	int32_t    m_rmrd_line;
	uint8_t    m_irq_enabled;
	uint8_t    m_romsubbank, m_scrollctrl;

	optional_region_ptr<uint8_t> m_char_rom;

	optional_device<screen_device> m_screen;

	k052109_cb_delegate m_k052109_cb;

	devcb_write_line m_irq_handler;
	devcb_write_line m_firq_handler;
	devcb_write_line m_nmi_handler;

	TILE_GET_INFO_MEMBER(get_tile_info0);
	TILE_GET_INFO_MEMBER(get_tile_info1);
	TILE_GET_INFO_MEMBER(get_tile_info2);

	void get_tile_info( tile_data &tileinfo, int tile_index, int layer, uint8_t *cram, uint8_t *vram1, uint8_t *vram2 );
	void tileflip_reset();
};

DECLARE_DEVICE_TYPE(K052109, k052109_device)

#endif // MAME_VIDEO_K052109_H
