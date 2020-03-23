// license:BSD-3-Clause
// copyright-holders:Luca Elia

/* TODO: some variables are per-game specifics */

#include "sound/okim6295.h"
#include "machine/gen_latch.h"
#include "machine/ticket.h"
#include "machine/watchdog.h"
#include "video/ms1_tmap.h"
#include "screen.h"

class cischeat_state : public driver_device
{
public:
	cischeat_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_tmap(*this, "scroll%u", 0),
		m_ram(*this, "ram"),
		m_roadram(*this, "roadram.%u", 0),
		m_f1gpstr2_ioready(*this, "ioready"),
		m_maincpu(*this, "maincpu"),
		m_cpu1(*this, "cpu1"),
		m_cpu2(*this, "cpu2"),
		m_cpu3(*this, "cpu3"),
		m_cpu5(*this, "cpu5"),
		m_soundcpu(*this, "soundcpu"),
		m_screen(*this, "screen"),
		m_watchdog(*this, "watchdog"),
		m_oki1(*this, "oki1"),
		m_oki2(*this, "oki2"),
		m_gfxdecode(*this, "gfxdecode"),
		m_palette(*this, "palette"),
		m_soundlatch(*this, "soundlatch"),
		m_soundlatch2(*this, "soundlatch2"),
		m_captflag_hopper(*this, "hopper"),
		m_captflag_motor_left(*this, "motor_left"),
		m_captflag_motor_right(*this, "motor_right"),
		m_oki1_bank(*this, "oki1_bank"),
		m_oki2_bank(*this, "oki2_bank")
		{
			for (int side = 0; side < 2; ++side)
				m_captflag_motor_command[side] = m_captflag_motor_pos[side] = 0;
			m_captflag_leds = 0;
		}

	optional_device_array<megasys1_tilemap_device, 3> m_tmap;
	required_shared_ptr<uint16_t> m_ram;
	optional_shared_ptr_array<uint16_t,2> m_roadram;
	optional_shared_ptr<uint16_t> m_f1gpstr2_ioready;

	uint16_t *m_objectram;
	uint16_t m_active_layers;

	int m_prev;
	int m_armold;
	uint16_t m_scudhamm_motor_command;
	int m_ip_select;
	uint16_t m_wildplt_output;
	uint8_t m_drawmode_table[16];
	int m_debugsprites;
	int m_show_unknown;
	uint16_t *m_spriteram;

	uint8_t m_motor_value;
	uint8_t m_io_value;

	DECLARE_WRITE16_MEMBER(scudhamm_motor_command_w);
	DECLARE_WRITE16_MEMBER(scudhamm_leds_w);
	DECLARE_WRITE16_MEMBER(scudhamm_enable_w);
	DECLARE_WRITE16_MEMBER(scudhamm_oki_bank_w);
	DECLARE_READ16_MEMBER(armchmp2_motor_status_r);
	DECLARE_WRITE16_MEMBER(armchmp2_motor_command_w);
	DECLARE_READ16_MEMBER(armchmp2_analog_r);
	DECLARE_READ16_MEMBER(armchmp2_buttons_r);
	DECLARE_WRITE16_MEMBER(armchmp2_leds_w);
	DECLARE_WRITE16_MEMBER(bigrun_soundbank_w);
	DECLARE_READ16_MEMBER(scudhamm_motor_status_r);
	DECLARE_READ16_MEMBER(scudhamm_motor_pos_r);
	DECLARE_READ16_MEMBER(scudhamm_analog_r);
	DECLARE_READ16_MEMBER(bigrun_ip_select_r);
	DECLARE_WRITE16_MEMBER(leds_out_w);
	DECLARE_WRITE16_MEMBER(unknown_out_w);
	DECLARE_WRITE16_MEMBER(motor_out_w);
	DECLARE_WRITE16_MEMBER(wheel_out_w);
	DECLARE_WRITE16_MEMBER(ip_select_w);
	DECLARE_WRITE16_MEMBER(ip_select_plus1_w);
	DECLARE_WRITE16_MEMBER(bigrun_comms_w);
	DECLARE_WRITE16_MEMBER(active_layers_w);
	DECLARE_READ16_MEMBER(cischeat_ip_select_r);
	DECLARE_WRITE16_MEMBER(cischeat_soundlatch_w);
	DECLARE_WRITE16_MEMBER(cischeat_comms_w);
	DECLARE_READ16_MEMBER(f1gpstar_wheel_r);
	DECLARE_READ16_MEMBER(f1gpstr2_ioready_r);
	DECLARE_READ16_MEMBER(wildplt_xy_r);
	DECLARE_READ16_MEMBER(wildplt_mux_r);
	DECLARE_WRITE16_MEMBER(wildplt_mux_w);
	DECLARE_WRITE16_MEMBER(f1gpstar_motor_w);
	DECLARE_WRITE16_MEMBER(f1gpstar_soundint_w);
	DECLARE_WRITE16_MEMBER(f1gpstar_comms_w);
	DECLARE_WRITE16_MEMBER(f1gpstr2_io_w);
	DECLARE_WRITE16_MEMBER(cischeat_soundbank_1_w);
	DECLARE_WRITE16_MEMBER(cischeat_soundbank_2_w);
	DECLARE_DRIVER_INIT(cischeat);
	DECLARE_DRIVER_INIT(bigrun);
	DECLARE_DRIVER_INIT(f1gpstar);
	virtual void video_start() override;
	uint32_t screen_update_bigrun(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	uint32_t screen_update_scudhamm(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	uint32_t screen_update_cischeat(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	uint32_t screen_update_f1gpstar(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	TIMER_DEVICE_CALLBACK_MEMBER(bigrun_scanline);
	TIMER_DEVICE_CALLBACK_MEMBER(scudhamm_scanline);
	TIMER_DEVICE_CALLBACK_MEMBER(armchamp2_scanline);
	void prepare_shadows();
	void cischeat_draw_road(bitmap_ind16 &bitmap, const rectangle &cliprect, int road_num, int priority1, int priority2, int transparency);
	void f1gpstar_draw_road(bitmap_ind16 &bitmap, const rectangle &cliprect, int road_num, int priority1, int priority2, int transparency);
	void cischeat_draw_sprites(bitmap_ind16 &bitmap , const rectangle &cliprect, int priority1, int priority2);
	void bigrun_draw_sprites(bitmap_ind16 &bitmap , const rectangle &cliprect, int priority1, int priority2);
	void cischeat_untangle_sprites(const char *region);
	optional_device<cpu_device> m_maincpu; // some are called cpu1
	optional_device<cpu_device> m_cpu1;
	optional_device<cpu_device> m_cpu2;
	optional_device<cpu_device> m_cpu3;
	optional_device<cpu_device> m_cpu5;
	optional_device<cpu_device> m_soundcpu;
	required_device<screen_device> m_screen;
	optional_device<watchdog_timer_device> m_watchdog;
	required_device<okim6295_device> m_oki1;
	required_device<okim6295_device> m_oki2;
	required_device<gfxdecode_device> m_gfxdecode;
	required_device<palette_device> m_palette;
	optional_device<generic_latch_16_device> m_soundlatch;
	optional_device<generic_latch_16_device> m_soundlatch2;

	// captflag
	optional_device<ticket_dispenser_device> m_captflag_hopper;

	optional_device<timer_device> m_captflag_motor_left;
	optional_device<timer_device> m_captflag_motor_right;
	uint16_t m_captflag_motor_command[2];
	uint16_t m_captflag_motor_pos[2];

	DECLARE_WRITE16_MEMBER(captflag_motor_command_right_w);
	DECLARE_WRITE16_MEMBER(captflag_motor_command_left_w);
	void captflag_motor_move(int side, uint16_t data);
	DECLARE_CUSTOM_INPUT_MEMBER(captflag_motor_busy_r);
	DECLARE_CUSTOM_INPUT_MEMBER(captflag_motor_pos_r);

	optional_memory_bank m_oki1_bank;
	optional_memory_bank m_oki2_bank;
	DECLARE_WRITE16_MEMBER(captflag_oki_bank_w);

	uint16_t m_captflag_leds;
	DECLARE_WRITE16_MEMBER(captflag_leds_w);

	DECLARE_DRIVER_INIT(captflag);
	TIMER_DEVICE_CALLBACK_MEMBER(captflag_scanline);
};
