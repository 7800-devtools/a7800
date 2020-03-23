// license:GPL-2.0+
// copyright-holders:Jarek Burczynski, Phil Stroffolino, Tomasz Slanina

#include "cpu/m6805/m68705.h"
#include "screen.h"


class changela_state : public driver_device
{
public:
	changela_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_spriteram(*this, "spriteram")
		, m_videoram(*this, "videoram")
		, m_colorram(*this, "colorram")
		, m_mcu(*this, "mcu")
		, m_maincpu(*this, "maincpu")
		, m_screen(*this, "screen")
		, m_palette(*this, "palette")
		, m_port_a_out(0xff)
		, m_port_c_out(0xff)
		, m_mcu_out(0xff)
		, m_mcu_in(0xff)
	{
	}

	/* video-related */
	bitmap_ind16 m_obj0_bitmap;
	bitmap_ind16 m_river_bitmap;
	bitmap_ind16 m_tree0_bitmap;
	bitmap_ind16 m_tree1_bitmap;
	std::unique_ptr<uint8_t[]>   m_tree_ram;
	std::unique_ptr<uint8_t[]>   m_memory_devices;
	uint32_t   m_mem_dev_selected;    /* an offset within memory_devices area */
	uint32_t   m_slopeROM_bank;
	uint8_t    m_tree_en;
	uint8_t    m_horizon;
	uint8_t    m_v_count_river;
	uint8_t    m_v_count_tree;
	int      m_tree_on[2];
	emu_timer* m_scanline_timer;

	/* misc */
	uint8_t    m_tree0_col;
	uint8_t    m_tree1_col;
	uint8_t    m_left_bank_col;
	uint8_t    m_right_bank_col;
	uint8_t    m_boat_shore_col;
	uint8_t    m_collision_reset;
	uint8_t    m_tree_collision_reset;
	uint8_t    m_prev_value_31;
	int      m_dir_31;

	/* devices */
	DECLARE_READ8_MEMBER(mcu_r);
	DECLARE_WRITE8_MEMBER(mcu_w);
	DECLARE_WRITE8_MEMBER(changela_68705_port_a_w);
	DECLARE_WRITE8_MEMBER(changela_68705_port_c_w);
	DECLARE_READ8_MEMBER(changela_24_r);
	DECLARE_READ8_MEMBER(changela_25_r);
	DECLARE_READ8_MEMBER(changela_30_r);
	DECLARE_READ8_MEMBER(changela_31_r);
	DECLARE_READ8_MEMBER(changela_2c_r);
	DECLARE_READ8_MEMBER(changela_2d_r);
	DECLARE_WRITE8_MEMBER(mcu_pc_0_w);
	DECLARE_WRITE8_MEMBER(changela_collision_reset_0);
	DECLARE_WRITE8_MEMBER(changela_collision_reset_1);
	DECLARE_WRITE8_MEMBER(changela_coin_counter_w);
	DECLARE_WRITE8_MEMBER(changela_colors_w);
	DECLARE_WRITE8_MEMBER(changela_mem_device_select_w);
	DECLARE_WRITE8_MEMBER(changela_mem_device_w);
	DECLARE_READ8_MEMBER(changela_mem_device_r);
	DECLARE_WRITE8_MEMBER(changela_slope_rom_addr_hi_w);
	DECLARE_WRITE8_MEMBER(changela_slope_rom_addr_lo_w);
	virtual void machine_start() override;
	virtual void machine_reset() override;
	virtual void video_start() override;
	uint32_t screen_update_changela(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	INTERRUPT_GEN_MEMBER(chl_mcu_irq);
	TIMER_CALLBACK_MEMBER(changela_scanline_callback);
	TIMER_DEVICE_CALLBACK_MEMBER(changela_scanline);
	void draw_obj0( bitmap_ind16 &bitmap, int sy );
	void draw_obj1( bitmap_ind16 &bitmap );
	void draw_river( bitmap_ind16 &bitmap, int sy );
	void draw_tree( bitmap_ind16 &bitmap, int sy, int tree_num );

protected:
	// memory pointers
	required_shared_ptr<u8>         m_spriteram;
	required_shared_ptr<u8>         m_videoram;
	required_shared_ptr<u8>         m_colorram;

	// devices
	required_device<m68705p_device> m_mcu;
	required_device<cpu_device>     m_maincpu;
	required_device<screen_device>  m_screen;
	required_device<palette_device> m_palette;

	// mcu-related
	u8  m_port_a_out;
	u8  m_port_c_out;
	u8  m_mcu_out;
	u8  m_mcu_in;
};
