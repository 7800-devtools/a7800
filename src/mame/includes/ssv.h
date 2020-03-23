// license:BSD-3-Clause
// copyright-holders:Luca Elia
#include "cpu/upd7725/upd7725.h"
#include "video/st0020.h"
#include "machine/eepromser.h"
#include "machine/upd4701.h"
#include "sound/es5506.h"
#include "screen.h"

class ssv_state : public driver_device
{
public:
	ssv_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_ensoniq(*this, "ensoniq"),
		m_eeprom(*this, "eeprom"),
		m_dsp(*this, "dsp"),
		m_upd4701(*this, "upd4701"),
		m_mainram(*this, "mainram"),
		m_spriteram(*this, "spriteram"),
		m_scroll(*this, "scroll"),
		m_irq_vectors(*this, "irq_vectors"),
		m_gdfs_tmapram(*this, "gdfs_tmapram"),
		m_gdfs_tmapscroll(*this, "gdfs_tmapscroll"),
		m_gdfs_st0020(*this, "st0020_spr"),
		m_input_sel(*this, "input_sel"),
		m_io_gun(*this, {"GUNX1", "GUNY1", "GUNX2", "GUNY2"}),
		m_io_key0(*this, "KEY0"),
		m_io_key1(*this, "KEY1"),
		m_io_key2(*this, "KEY2"),
		m_io_key3(*this, "KEY3"),
		m_io_service(*this, "SERVICE"),
		m_io_paddle(*this, "PADDLE"),
		m_gfxdecode(*this, "gfxdecode"),
		m_screen(*this, "screen"),
		m_palette(*this, "palette")
	{ }

	required_device<cpu_device> m_maincpu;
	required_device<es5506_device> m_ensoniq;
	optional_device<eeprom_serial_93cxx_device> m_eeprom;
	optional_device<upd96050_device> m_dsp;
	optional_device<upd4701_device> m_upd4701;

	required_shared_ptr<uint16_t> m_mainram;
	required_shared_ptr<uint16_t> m_spriteram;
	required_shared_ptr<uint16_t> m_scroll;
	required_shared_ptr<uint16_t> m_irq_vectors;
	optional_shared_ptr<uint16_t> m_gdfs_tmapram;
	optional_shared_ptr<uint16_t> m_gdfs_tmapscroll;
	optional_device<st0020_device> m_gdfs_st0020;
	optional_shared_ptr<uint16_t> m_input_sel;

	int m_tile_code[16];
	int m_enable_video;
	int m_shadow_pen_mask;
	int m_shadow_pen_shift;
	uint8_t m_requested_int;
	uint16_t m_irq_enable;
	std::unique_ptr<uint16_t[]> m_eaglshot_gfxram;
	tilemap_t *m_gdfs_tmap;
	int m_interrupt_ultrax;
	int m_gdfs_lightgun_select;
	uint16_t m_sxyreact_serial;
	int m_sxyreact_dial;
	uint16_t m_gdfs_eeprom_old;
	uint32_t m_latches[8];
	uint8_t m_trackball_select;

	DECLARE_WRITE16_MEMBER(irq_ack_w);
	DECLARE_WRITE16_MEMBER(irq_enable_w);
	DECLARE_WRITE16_MEMBER(lockout_w);
	DECLARE_WRITE16_MEMBER(lockout_inv_w);
	DECLARE_READ16_MEMBER(dsp_dr_r);
	DECLARE_WRITE16_MEMBER(dsp_dr_w);
	DECLARE_READ16_MEMBER(dsp_r);
	DECLARE_WRITE16_MEMBER(dsp_w);
	DECLARE_READ16_MEMBER(drifto94_unknown_r);
	DECLARE_READ16_MEMBER(hypreact_input_r);
	DECLARE_READ16_MEMBER(mainram_r);
	DECLARE_WRITE16_MEMBER(mainram_w);
	DECLARE_READ16_MEMBER(srmp4_input_r);
	DECLARE_READ16_MEMBER(srmp7_irqv_r);
	DECLARE_WRITE16_MEMBER(srmp7_sound_bank_w);
	DECLARE_READ16_MEMBER(srmp7_input_r);
	DECLARE_READ16_MEMBER(sxyreact_ballswitch_r);
	DECLARE_READ16_MEMBER(sxyreact_dial_r);
	DECLARE_WRITE16_MEMBER(sxyreact_dial_w);
	DECLARE_WRITE16_MEMBER(sxyreact_motor_w);
	DECLARE_READ32_MEMBER(latch32_r);
	DECLARE_WRITE32_MEMBER(latch32_w);
	DECLARE_READ16_MEMBER(latch16_r);
	DECLARE_WRITE16_MEMBER(latch16_w);
	DECLARE_WRITE8_MEMBER(eaglshot_gfxrom_bank_w);
	DECLARE_WRITE8_MEMBER(eaglshot_trackball_w);
	DECLARE_READ16_MEMBER(eaglshot_gfxram_r);
	DECLARE_WRITE16_MEMBER(eaglshot_gfxram_w);
	DECLARE_WRITE16_MEMBER(gdfs_tmapram_w);
	DECLARE_READ16_MEMBER(vblank_r);
	DECLARE_WRITE16_MEMBER(scroll_w);
	DECLARE_READ16_MEMBER(gdfs_eeprom_r);
	DECLARE_WRITE16_MEMBER(gdfs_eeprom_w);

	TILE_GET_INFO_MEMBER(get_tile_info_0);

	DECLARE_DRIVER_INIT(gdfs);
	DECLARE_DRIVER_INIT(sxyreac2);
	DECLARE_DRIVER_INIT(hypreac2);
	DECLARE_DRIVER_INIT(hypreact);
	DECLARE_DRIVER_INIT(dynagear);
	DECLARE_DRIVER_INIT(eaglshot);
	DECLARE_DRIVER_INIT(srmp4);
	DECLARE_DRIVER_INIT(srmp7);
	DECLARE_DRIVER_INIT(keithlcy);
	DECLARE_DRIVER_INIT(meosism);
	DECLARE_DRIVER_INIT(vasara);
	DECLARE_DRIVER_INIT(cairblad);
	DECLARE_DRIVER_INIT(sxyreact);
	DECLARE_DRIVER_INIT(janjans1);
	DECLARE_DRIVER_INIT(ryorioh);
	DECLARE_DRIVER_INIT(drifto94);
	DECLARE_DRIVER_INIT(survarts);
	DECLARE_DRIVER_INIT(ultrax);
	DECLARE_DRIVER_INIT(stmblade);
	DECLARE_DRIVER_INIT(jsk);
	DECLARE_DRIVER_INIT(twineag2);
	DECLARE_DRIVER_INIT(mslider);
	virtual void machine_reset() override;
	virtual void video_start() override;
	DECLARE_VIDEO_START(gdfs);
	DECLARE_VIDEO_START(eaglshot);

	uint32_t screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	uint32_t screen_update_gdfs(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	uint32_t screen_update_eaglshot(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);

	TIMER_DEVICE_CALLBACK_MEMBER(interrupt);
	TIMER_DEVICE_CALLBACK_MEMBER(gdfs_interrupt);
	void update_irq_state();
	IRQ_CALLBACK_MEMBER(irq_callback);

	void drawgfx(bitmap_ind16 &bitmap, const rectangle &cliprect, gfx_element *gfx,uint32_t code,uint32_t color,int flipx,int flipy,int x0,int y0,int shadow);
	void draw_row(bitmap_ind16 &bitmap, const rectangle &cliprect, int sx, int sy, int scroll);
	void draw_layer(bitmap_ind16 &bitmap, const rectangle &cliprect, int  nr);
	void draw_sprites(bitmap_ind16 &bitmap, const rectangle &cliprect);
	void enable_video(int enable);
	void init(int interrupt_ultrax);
	void init_hypreac2_common();
	void init_eaglshot_banking();
	void init_st010();

protected:
	optional_ioport_array<4> m_io_gun;
	optional_ioport m_io_key0;
	optional_ioport m_io_key1;
	optional_ioport m_io_key2;
	optional_ioport m_io_key3;
	optional_ioport m_io_service;
	optional_ioport m_io_paddle;
	required_device<gfxdecode_device> m_gfxdecode;
	required_device<screen_device> m_screen;
	required_device<palette_device> m_palette;
};
