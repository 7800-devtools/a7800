// license:BSD-3-Clause
// copyright-holders:Stefan Jokisch
#include "sound/discrete.h"
#include "screen.h"

class sprint8_state : public driver_device
{
public:
	sprint8_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_gfxdecode(*this, "gfxdecode"),
		m_screen(*this, "screen"),
		m_palette(*this, "palette"),
		m_discrete(*this, "discrete"),
		m_video_ram(*this, "video_ram"),
		m_pos_h_ram(*this, "pos_h_ram"),
		m_pos_v_ram(*this, "pos_v_ram"),
		m_pos_d_ram(*this, "pos_d_ram"),
		m_team(*this, "team") { }

	required_device<cpu_device> m_maincpu;
	required_device<gfxdecode_device> m_gfxdecode;
	required_device<screen_device> m_screen;
	required_device<palette_device> m_palette;
	required_device<discrete_device> m_discrete;

	required_shared_ptr<uint8_t> m_video_ram;
	required_shared_ptr<uint8_t> m_pos_h_ram;
	required_shared_ptr<uint8_t> m_pos_v_ram;
	required_shared_ptr<uint8_t> m_pos_d_ram;
	required_shared_ptr<uint8_t> m_team;

	int m_steer_dir[8];
	int m_steer_flag[8];
	int m_collision_reset;
	int m_collision_index;
	uint8_t m_dial[8];
	tilemap_t* m_tilemap1;
	tilemap_t* m_tilemap2;
	bitmap_ind16 m_helper1;
	bitmap_ind16 m_helper2;
	emu_timer *m_collision_timer;

	DECLARE_READ8_MEMBER(collision_r);
	DECLARE_READ8_MEMBER(input_r);
	DECLARE_WRITE8_MEMBER(lockout_w);
	DECLARE_WRITE8_MEMBER(int_reset_w);
	DECLARE_WRITE8_MEMBER(video_ram_w);
	DECLARE_WRITE8_MEMBER(crash_w);
	DECLARE_WRITE8_MEMBER(screech_w);
	DECLARE_WRITE8_MEMBER(attract_w);
	DECLARE_WRITE8_MEMBER(motor_w);

	virtual void machine_start() override;
	virtual void machine_reset() override;
	virtual void video_start() override;
	DECLARE_PALETTE_INIT(sprint8);

	TILE_GET_INFO_MEMBER(get_tile_info1);
	TILE_GET_INFO_MEMBER(get_tile_info2);
	uint32_t screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	DECLARE_WRITE_LINE_MEMBER(screen_vblank);
	TIMER_CALLBACK_MEMBER(collision_callback);
	TIMER_DEVICE_CALLBACK_MEMBER(input_callback);

	void set_pens();
	void draw_sprites(bitmap_ind16 &bitmap, const rectangle &cliprect);
	void set_collision(int n);
};

/*----------- defined in audio/sprint8.c -----------*/
DISCRETE_SOUND_EXTERN( sprint8 );
