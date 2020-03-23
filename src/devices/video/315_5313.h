// license:BSD-3-Clause
// copyright-holders:David Haywood
/* Sega Megadrive / Genesis VDP */
#ifndef MAME_VIDEO_315_5313_H
#define MAME_VIDEO_315_5313_H

#pragma once

#include "video/315_5124.h"
#include "cpu/m68000/m68000.h"


#define MCFG_SEGA315_5313_IS_PAL(_bool) \
	sega315_5313_device::set_signal_type(*device, _bool);

#define MCFG_SEGA315_5313_INT_CB(_devcb) \
	devcb = &sega315_5313_device::set_int_callback(*device, DEVCB_##_devcb);

#define MCFG_SEGA315_5313_PAUSE_CB(_devcb) \
	devcb = &sega315_5313_device::set_pause_callback(*device, DEVCB_##_devcb);

#define MCFG_SEGA315_5313_SND_IRQ_CALLBACK(_write) \
	devcb = &sega315_5313_device::set_sndirqline_callback(*device, DEVCB_##_write);

#define MCFG_SEGA315_5313_LV6_IRQ_CALLBACK(_write) \
	devcb = &sega315_5313_device::set_lv6irqline_callback(*device, DEVCB_##_write);

#define MCFG_SEGA315_5313_LV4_IRQ_CALLBACK(_write) \
	devcb = &sega315_5313_device::set_lv4irqline_callback(*device, DEVCB_##_write);

#define MCFG_SEGA315_5313_ALT_TIMING(_data) \
	sega315_5313_device::set_alt_timing(*device, _data);

#define MCFG_SEGA315_5313_PAL_WRITE_BASE(_data) \
	sega315_5313_device::set_palwrite_base(*device, _data);

#define MCFG_SEGA315_5313_PALETTE(_palette_tag) \
	sega315_5313_device::static_set_palette_tag(*device, "^" _palette_tag);


// Temporary solution while 32x VDP mixing and scanline interrupting is moved outside MD VDP
#define MCFG_SEGA315_5313_32X_SCANLINE_CB(_class, _method) \
	sega315_5313_device::set_md_32x_scanline(*device, sega315_5313_device::md_32x_scanline_delegate(&_class::_method, #_class "::" #_method, downcast<_class *>(owner)));

#define MCFG_SEGA315_5313_32X_INTERRUPT_CB(_class, _method) \
	sega315_5313_device::set_md_32x_interrupt(*device, sega315_5313_device::md_32x_interrupt_delegate(&_class::_method, #_class "::" #_method, downcast<_class *>(owner)));

#define MCFG_SEGA315_5313_32X_SCANLINE_HELPER_CB(_class, _method) \
	sega315_5313_device::set_md_32x_scanline_helper(*device, sega315_5313_device::md_32x_scanline_helper_delegate(&_class::_method, #_class "::" #_method, downcast<_class *>(owner)));


class sega315_5313_device : public sega315_5124_device
{
public:
	typedef device_delegate<void (int x, uint32_t priority, uint16_t &lineptr)> md_32x_scanline_delegate;
	typedef device_delegate<void (int scanline, int irq6)> md_32x_interrupt_delegate;
	typedef device_delegate<void (int scanline)> md_32x_scanline_helper_delegate;

	sega315_5313_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	template <class Object> static devcb_base &set_sndirqline_callback(device_t &device, Object &&cb) { return downcast<sega315_5313_device &>(device).m_sndirqline_callback.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_lv6irqline_callback(device_t &device, Object &&cb) { return downcast<sega315_5313_device &>(device).m_lv6irqline_callback.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_lv4irqline_callback(device_t &device, Object &&cb) { return downcast<sega315_5313_device &>(device).m_lv4irqline_callback.set_callback(std::forward<Object>(cb)); }
	static void set_alt_timing(device_t &device, int use_alt_timing);
	static void set_palwrite_base(device_t &device, int palwrite_base);
	static void static_set_palette_tag(device_t &device, const char *tag);

	static void set_md_32x_scanline(device_t &device, md_32x_scanline_delegate &&cb) { downcast<sega315_5313_device &>(device).m_32x_scanline_func = std::move(cb); }
	static void set_md_32x_interrupt(device_t &device, md_32x_interrupt_delegate &&cb) { downcast<sega315_5313_device &>(device).m_32x_interrupt_func = std::move(cb); }
	static void set_md_32x_scanline_helper(device_t &device, md_32x_scanline_helper_delegate &&cb) { downcast<sega315_5313_device &>(device).m_32x_scanline_helper_func = std::move(cb); }

	int m_use_alt_timing; // use MAME scanline timer instead, render only one scanline to a single line buffer, to be rendered by a partial update call.. experimental

	int m_palwrite_base; // if we want to write to the actual MAME palette..

	DECLARE_READ16_MEMBER( vdp_r );
	DECLARE_WRITE16_MEMBER( vdp_w );

	int get_scanline_counter();

	TIMER_CALLBACK_MEMBER(render_scanline);
	void vdp_handle_scanline_callback(int scanline);
	TIMER_CALLBACK_MEMBER(irq6_on_timer_callback);
	TIMER_CALLBACK_MEMBER(irq4_on_timer_callback);
	void vdp_handle_eof();
	void device_reset_old();
	void vdp_clear_irq6_pending() { m_irq6_pending = 0; };
	void vdp_clear_irq4_pending() { m_irq4_pending = 0; };

	// set some VDP variables at start (shall be moved to a device interface?)
	void set_scanline_counter(int scanline) { m_scanline_counter = scanline; }
	void set_total_scanlines(int total) { m_base_total_scanlines = total; }
	void set_framerate(int rate) { m_framerate = rate; }
	void set_vdp_pal(bool pal) { m_vdp_pal = pal ? 1 : 0; }
	void set_use_cram(int cram) { m_use_cram = cram; }
	void set_dma_delay(int delay) { m_dma_delay = delay; }
	int get_framerate() { return m_framerate; }
	int get_imode() { return m_imode; }


	void vdp_clear_bitmap()
	{
		if (m_render_bitmap)
			m_render_bitmap->fill(0);
	}

	std::unique_ptr<bitmap_ind16> m_render_bitmap;
	std::unique_ptr<uint16_t[]> m_render_line;
	std::unique_ptr<uint16_t[]> m_render_line_raw;

	TIMER_DEVICE_CALLBACK_MEMBER( megadriv_scanline_timer_callback_alt_timing );
	TIMER_DEVICE_CALLBACK_MEMBER( megadriv_scanline_timer_callback );
	timer_device* m_megadriv_scanline_timer;

	inline uint16_t vdp_get_word_from_68k_mem(uint32_t source);

protected:
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_add_mconfig(machine_config &config) override;

	// called when we hit 240 and 241 (used to control the z80 irq line on genesis, or the main irq on c2)
	devcb_write_line m_sndirqline_callback;
	devcb_write_line m_lv6irqline_callback;
	devcb_write_line m_lv4irqline_callback;

	md_32x_scanline_delegate m_32x_scanline_func;
	md_32x_interrupt_delegate m_32x_interrupt_func;
	md_32x_scanline_helper_delegate m_32x_scanline_helper_func;

private:
	int m_command_pending; // 2nd half of command pending..
	uint16_t m_command_part1;
	uint16_t m_command_part2;
	uint8_t  m_vdp_code;
	uint16_t m_vdp_address;
	uint8_t m_vram_fill_pending;
	uint16_t m_vram_fill_length;
	int m_irq4counter;
	int m_imode_odd_frame;
	int m_sprite_collision;
	int m_irq6_pending;
	int m_irq4_pending;
	int m_scanline_counter;
	int m_vblank_flag;

	int m_imode;

	int m_visible_scanlines;
	int m_irq6_scanline;
	int m_z80irq_scanline;
	int m_total_scanlines;
	// this is only set at init: 262 for PAL, 313 for NTSC
	int m_base_total_scanlines;

	int m_framerate;
	int m_vdp_pal;
	int m_use_cram; // c2 uses it's own palette ram, so it sets this to 0
	int m_dma_delay;    // SVP and SegaCD have some 'lag' in DMA transfers

	std::unique_ptr<uint16_t[]> m_regs;
	std::unique_ptr<uint16_t[]> m_vram;
	std::unique_ptr<uint16_t[]> m_cram;
	std::unique_ptr<uint16_t[]> m_vsram;
	/* The VDP keeps a 0x400 byte on-chip cache of the Sprite Attribute Table
	   to speed up processing, Castlevania Bloodlines abuses this on the upside down level */
	std::unique_ptr<uint16_t[]> m_internal_sprite_attribute_table;

	// these are used internally by the VDP to schedule when after the start of a scanline
	// to trigger the various interrupts / rendering to our bitmap, bit of a hack really
	emu_timer* m_irq6_on_timer;
	emu_timer* m_irq4_on_timer;
	emu_timer* m_render_timer;

	uint16_t vdp_vram_r(void);
	uint16_t vdp_vsram_r(void);
	uint16_t vdp_cram_r(void);

	void insta_68k_to_cram_dma(uint32_t source,uint16_t length);
	void insta_68k_to_vsram_dma(uint32_t source,uint16_t length);
	void insta_68k_to_vram_dma(uint32_t source,int length);
	void insta_vram_copy(uint32_t source, uint16_t length);

	void vdp_vram_write(uint16_t data);
	void vdp_cram_write(uint16_t data);
	void write_cram_value(int offset, int data);
	void vdp_vsram_write(uint16_t data);

	void vdp_set_register(int regnum, uint8_t value);

	void handle_dma_bits();

	uint16_t get_hposition();
	uint16_t megadriv_read_hv_counters();

	uint16_t ctrl_port_r();
	uint16_t data_port_r();
	void data_port_w(int data);
	void ctrl_port_w(int data);
	void update_code_and_address(void);


	void render_spriteline_to_spritebuffer(int scanline);
	void render_videoline_to_videobuffer(int scanline);
	void render_videobuffer_to_screenbuffer(int scanline);

	/* variables used during emulation - not saved */
	std::unique_ptr<uint8_t[]> m_sprite_renderline;
	std::unique_ptr<uint8_t[]> m_highpri_renderline;
	std::unique_ptr<uint32_t[]> m_video_renderline;
	std::unique_ptr<uint16_t[]> m_palette_lookup;
	std::unique_ptr<uint16_t[]> m_palette_lookup_sprite; // for C2
	std::unique_ptr<uint16_t[]> m_palette_lookup_shadow;
	std::unique_ptr<uint16_t[]> m_palette_lookup_highlight;

	address_space *m_space68k;
	m68000_base_device* m_cpu68k;
};


DECLARE_DEVICE_TYPE(SEGA315_5313, sega315_5313_device)

#endif // MAME_VIDEO_315_5313_H
