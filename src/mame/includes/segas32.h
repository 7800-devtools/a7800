// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/***************************************************************************

    Sega System 32/Multi 32 hardware

***************************************************************************/
#ifndef MAME_INCLUDES_SEGAS32_H
#define MAME_INCLUDES_SEGAS32_H

#pragma once

#include "sound/multipcm.h"
#include "machine/s32comm.h"
#include "screen.h"




class segas32_state : public device_t
{
public:
	segas32_state(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	required_shared_ptr<uint8_t> m_z80_shared_ram;
	optional_shared_ptr<uint16_t> m_system32_workram;
	required_shared_ptr<uint16_t> m_system32_videoram;
	required_shared_ptr<uint16_t> m_system32_spriteram;
	optional_shared_ptr_array<uint16_t, 2> m_system32_paletteram;

	required_device<cpu_device> m_maincpu;
	required_device<cpu_device> m_soundcpu;
	optional_device<multipcm_device> m_multipcm;
	required_device<gfxdecode_device> m_gfxdecode;
	required_device<screen_device> m_screen;
	required_device<palette_device> m_palette;

	required_device<timer_device> m_irq_timer_0;
	required_device<timer_device> m_irq_timer_1;
	optional_device<s32comm_device> m_s32comm;

	typedef void (segas32_state::*sys32_output_callback)(int which, uint16_t data);

	struct layer_info
	{
		bitmap_ind16 *bitmap;
		uint8_t* transparent;
	};

	struct extents_list
	{
		uint8_t                   scan_extent[256];
		uint16_t                  extent[32][16];
	};


	struct cache_entry
	{
		struct cache_entry *    next;
		tilemap_t *             tmap;
		uint8_t                   page;
		uint8_t                   bank;
	};

	uint8_t m_v60_irq_control[0x10];
	timer_device *m_v60_irq_timer[2];
	uint8_t m_sound_irq_control[4];
	uint8_t m_sound_irq_input;
	uint8_t m_sound_dummy_value;
	uint16_t m_sound_bank;
	sys32_output_callback m_sw1_output;
	sys32_output_callback m_sw2_output;
	sys32_output_callback m_sw3_output;
	std::unique_ptr<uint16_t[]> m_system32_protram;
	uint16_t m_system32_displayenable[2];
	uint16_t m_system32_tilebank_external;
	uint16_t m_arescue_dsp_io[6];
	uint8_t m_is_multi32;
	struct cache_entry *m_cache_head;
	struct layer_info m_layer_data[11];
	uint16_t m_mixer_control[2][0x40];
	std::unique_ptr<uint16_t[]> m_solid_0000;
	std::unique_ptr<uint16_t[]> m_solid_ffff;
	uint8_t m_sprite_render_count;
	uint8_t m_sprite_control_latched[8];
	uint8_t m_sprite_control[8];
	std::unique_ptr<uint32_t[]> m_spriteram_32bit;
	typedef void (segas32_state::*prot_vblank_func)();
	prot_vblank_func m_system32_prot_vblank;
	int m_print_count;
	emu_timer *m_vblank_end_int_timer;
	emu_timer *m_update_sprites_timer;
	DECLARE_WRITE16_MEMBER(sonic_level_load_protection);
	DECLARE_READ16_MEMBER(brival_protection_r);
	DECLARE_WRITE16_MEMBER(brival_protection_w);
	DECLARE_WRITE16_MEMBER(darkedge_protection_w);
	DECLARE_READ16_MEMBER(darkedge_protection_r);
	DECLARE_WRITE16_MEMBER(dbzvrvs_protection_w);
	DECLARE_READ16_MEMBER(dbzvrvs_protection_r);
	DECLARE_READ16_MEMBER(arf_wakeup_protection_r);
	DECLARE_WRITE16_MEMBER(jleague_protection_w);
	DECLARE_READ16_MEMBER(arescue_dsp_r);
	DECLARE_WRITE16_MEMBER(arescue_dsp_w);
	DECLARE_READ16_MEMBER(system32_paletteram_r);
	DECLARE_WRITE16_MEMBER(system32_paletteram_w);
	DECLARE_READ32_MEMBER(multi32_paletteram_0_r);
	DECLARE_WRITE32_MEMBER(multi32_paletteram_0_w);
	DECLARE_READ32_MEMBER(multi32_paletteram_1_r);
	DECLARE_WRITE32_MEMBER(multi32_paletteram_1_w);
	DECLARE_READ16_MEMBER(system32_videoram_r);
	DECLARE_WRITE16_MEMBER(system32_videoram_w);
	DECLARE_READ32_MEMBER(multi32_videoram_r);
	DECLARE_WRITE32_MEMBER(multi32_videoram_w);
	DECLARE_READ16_MEMBER(system32_sprite_control_r);
	DECLARE_WRITE16_MEMBER(system32_sprite_control_w);
	DECLARE_READ32_MEMBER(multi32_sprite_control_r);
	DECLARE_WRITE32_MEMBER(multi32_sprite_control_w);
	DECLARE_READ16_MEMBER(system32_spriteram_r);
	DECLARE_WRITE16_MEMBER(system32_spriteram_w);
	DECLARE_READ32_MEMBER(multi32_spriteram_r);
	DECLARE_WRITE32_MEMBER(multi32_spriteram_w);
	DECLARE_READ16_MEMBER(system32_mixer_r);
	DECLARE_WRITE16_MEMBER(system32_mixer_w);
	DECLARE_WRITE32_MEMBER(multi32_mixer_0_w);
	DECLARE_WRITE32_MEMBER(multi32_mixer_1_w);
	DECLARE_READ16_MEMBER(interrupt_control_16_r);
	DECLARE_WRITE16_MEMBER(interrupt_control_16_w);
	DECLARE_READ32_MEMBER(interrupt_control_32_r);
	DECLARE_WRITE32_MEMBER(interrupt_control_32_w);
	DECLARE_WRITE8_MEMBER(misc_output_0_w);
	DECLARE_WRITE8_MEMBER(misc_output_1_w);
	DECLARE_WRITE8_MEMBER(sw2_output_0_w);
	DECLARE_WRITE8_MEMBER(sw2_output_1_w);
	DECLARE_WRITE8_MEMBER(tilebank_external_w);
	DECLARE_WRITE_LINE_MEMBER(display_enable_0_w);
	DECLARE_WRITE_LINE_MEMBER(display_enable_1_w);
	DECLARE_WRITE16_MEMBER(random_number_16_w);
	DECLARE_READ16_MEMBER(random_number_16_r);
	DECLARE_WRITE32_MEMBER(random_number_32_w);
	DECLARE_READ32_MEMBER(random_number_32_r);
	DECLARE_READ16_MEMBER(shared_ram_16_r);
	DECLARE_WRITE16_MEMBER(shared_ram_16_w);
	DECLARE_READ32_MEMBER(shared_ram_32_r);
	DECLARE_WRITE32_MEMBER(shared_ram_32_w);
	DECLARE_WRITE8_MEMBER(sound_int_control_lo_w);
	DECLARE_WRITE8_MEMBER(sound_int_control_hi_w);
	DECLARE_WRITE8_MEMBER(sound_bank_lo_w);
	DECLARE_WRITE8_MEMBER(sound_bank_hi_w);
	DECLARE_READ8_MEMBER(sound_dummy_r);
	DECLARE_WRITE8_MEMBER(sound_dummy_w);

	DECLARE_WRITE8_MEMBER(multipcm_bank_w);
	DECLARE_WRITE8_MEMBER(scross_bank_w);

	TILE_GET_INFO_MEMBER(get_tile_info);
	uint32_t screen_update_system32(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);
	uint32_t screen_update_multi32_left(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);
	uint32_t screen_update_multi32_right(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);
	INTERRUPT_GEN_MEMBER(start_of_vblank_int);
	TIMER_CALLBACK_MEMBER(end_of_vblank_int);
	TIMER_CALLBACK_MEMBER(update_sprites);
	TIMER_DEVICE_CALLBACK_MEMBER(signal_v60_irq_callback);
	void common_start(int multi32);
	void system32_set_vblank(int state);
	inline uint16_t xBBBBBGGGGGRRRRR_to_xBGRBBBBGGGGRRRR(uint16_t value);
	inline uint16_t xBGRBBBBGGGGRRRR_to_xBBBBBGGGGGRRRRR(uint16_t value);
	inline void update_color(int offset, uint16_t data);
	inline uint16_t common_paletteram_r(address_space &space, int which, offs_t offset);
	void common_paletteram_w(address_space &space, int which, offs_t offset, uint16_t data, uint16_t mem_mask);
	tilemap_t *find_cache_entry(int page, int bank);
	inline void get_tilemaps(int bgnum, tilemap_t **tilemaps);
	uint8_t update_tilemaps(screen_device &screen, const rectangle &cliprect);
	void sprite_erase_buffer();
	void sprite_swap_buffers();
	int draw_one_sprite(uint16_t *data, int xoffs, int yoffs, const rectangle &clipin, const rectangle &clipout);
	void sprite_render_list();
	inline uint8_t compute_color_offsets(int which, int layerbit, int layerflag);
	inline uint16_t compute_sprite_blend(uint8_t encoding);
	inline uint16_t *get_layer_scanline(int layer, int scanline);
	void mix_all_layers(int which, int xoffs, bitmap_rgb32 &bitmap, const rectangle &cliprect, uint8_t enablemask);
	void print_mixer_data(int which);
	uint32_t multi32_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect, int index);
	void update_irq_state();
	void signal_v60_irq(int which);
	void int_control_w(address_space &space, int offset, uint8_t data);
	void update_sound_irq_state();
	void segas32_common_init();
	void radm_sw1_output( int which, uint16_t data );
	void radm_sw2_output( int which, uint16_t data );
	void radr_sw2_output( int which, uint16_t data );
	void alien3_sw1_output( int which, uint16_t data );
	void arescue_sw1_output( int which, uint16_t data );
	void f1lap_sw1_output( int which, uint16_t data );
	void jpark_sw1_output( int which, uint16_t data );
	void orunners_sw1_output( int which, uint16_t data );
	void orunners_sw2_output( int which, uint16_t data );
	void harddunk_sw1_output( int which, uint16_t data );
	void harddunk_sw2_output( int which, uint16_t data );
	void harddunk_sw3_output( int which, uint16_t data );
	void titlef_sw1_output( int which, uint16_t data );
	void titlef_sw2_output( int which, uint16_t data );
	void scross_sw1_output( int which, uint16_t data );
	void scross_sw2_output( int which, uint16_t data );
	int compute_clipping_extents(screen_device &screen, int enable, int clipout, int clipmask, const rectangle &cliprect, struct extents_list *list);
	void compute_tilemap_flips(int bgnum, int &flipx, int &flipy);
	void update_tilemap_zoom(screen_device &screen, struct layer_info *layer, const rectangle &cliprect, int bgnum);
	void update_tilemap_rowscroll(screen_device &screen, struct layer_info *layer, const rectangle &cliprect, int bgnum);
	void update_tilemap_text(screen_device &screen, struct layer_info *layer, const rectangle &cliprect);
	void update_bitmap(screen_device &screen, struct layer_info *layer, const rectangle &cliprect);
	void update_background(struct layer_info *layer, const rectangle &cliprect);
	DECLARE_WRITE_LINE_MEMBER(ym3438_irq_handler);
	void signal_sound_irq(int which);
	void clear_sound_irq(int which);
	void darkedge_fd1149_vblank();
	void f1lap_fd1149_vblank();

	void init_alien3(void);
	void init_arescue(int m_hasdsp);
	void init_arabfgt(void);
	void init_brival(void);
	void init_darkedge(void);
	void init_dbzvrvs(void);
	void init_f1en(void);
	void init_f1lap(void);
	void init_ga2(void);
	void init_harddunk(void);
	void init_holo(void);
	void init_jpark(void);
	void init_orunners(void);
	void init_radm(void);
	void init_radr(void);
	void init_scross(void);
	void init_slipstrm(void);
	void init_sonic(void);
	void init_sonicp(void);
	void init_spidman(void);
	void init_svf(void);
	void init_jleague(void);
	void init_titlef(void);

protected:
	segas32_state(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	virtual void device_add_mconfig(machine_config &config) override;
	virtual void device_start() override;
	virtual void device_reset() override;
};

class segas32_regular_state : public segas32_state
{
public:
	segas32_regular_state(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
//  virtual void device_start() override;
//  virtual void device_reset() override;
};

class segas32_analog_state : public segas32_state
{
public:
	segas32_analog_state(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	segas32_analog_state(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	virtual void device_add_mconfig(machine_config &config) override;
//  virtual void device_start() override;
//  virtual void device_reset() override;
};

class segas32_trackball_state : public segas32_state
{
public:
	segas32_trackball_state(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	virtual void device_add_mconfig(machine_config &config) override;
	virtual void device_start() override;
};

class segas32_4player_state : public segas32_state
{
public:
	segas32_4player_state(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	segas32_4player_state(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	virtual void device_add_mconfig(machine_config &config) override;
//  virtual void device_start() override;
//  virtual void device_reset() override;
};

class segas32_v25_state : public segas32_4player_state
{
public:
	segas32_v25_state(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	static const uint8_t arf_opcode_table[256];
	static const uint8_t ga2_opcode_table[256];

protected:
	virtual void device_add_mconfig(machine_config &config) override;
	virtual void device_start() override;
//  virtual void device_reset() override;

private:
	void decrypt_protrom();
};

class segas32_upd7725_state : public segas32_analog_state
{
public:
	segas32_upd7725_state(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	virtual void device_add_mconfig(machine_config &config) override;
	virtual void device_start() override;
//  virtual void device_reset() override;
};

class segas32_cd_state : public segas32_state
{
public:
	segas32_cd_state(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	DECLARE_WRITE8_MEMBER(lamps1_w);
	DECLARE_WRITE8_MEMBER(lamps2_w);
	DECLARE_WRITE_LINE_MEMBER(scsi_irq_w);
	DECLARE_WRITE_LINE_MEMBER(scsi_drq_w);

protected:
	virtual void device_add_mconfig(machine_config &config) override;
	virtual void device_start() override;
//  virtual void device_reset() override;
};

class sega_multi32_state : public segas32_state
{
public:
	sega_multi32_state(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	sega_multi32_state(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	virtual void device_add_mconfig(machine_config &config) override;
	virtual void device_start() override;
//  virtual void device_reset() override;
};

class sega_multi32_analog_state : public sega_multi32_state
{
public:
	sega_multi32_analog_state(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	ioport_value in2_analog_read();
	ioport_value in3_analog_read();
	DECLARE_WRITE8_MEMBER(analog_bank_w);

protected:
	virtual void device_add_mconfig(machine_config &config) override;
	virtual void device_start() override;
//  virtual void device_reset() override;

private:
	optional_ioport_array<8> m_analog_ports;
	uint8_t m_analog_bank;
};

class sega_multi32_6player_state : public sega_multi32_state
{
public:
	sega_multi32_6player_state(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	virtual void device_add_mconfig(machine_config &config) override;
	virtual void device_start() override;
//  virtual void device_reset() override;
};

DECLARE_DEVICE_TYPE(SEGA_S32_PCB, segas32_state)

#endif // MAME_INCLUDES_SEGAS32_H
