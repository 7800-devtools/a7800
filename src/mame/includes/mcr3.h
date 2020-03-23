// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/*************************************************************************

    Midway MCR-3 system

**************************************************************************/
#include "screen.h"

class mcr3_state : public mcr_state
{
public:
	mcr3_state(const machine_config &mconfig, device_type type, const char *tag)
		: mcr_state(mconfig, type, tag),
		m_spyhunt_alpharam(*this, "spyhunt_alpha"),
		m_screen(*this, "screen")
	{ }

	optional_shared_ptr<uint8_t> m_spyhunt_alpharam;
	required_device<screen_device> m_screen;

	uint8_t m_input_mux;
	uint8_t m_latched_input;
	uint8_t m_last_op4;
	uint8_t m_maxrpm_adc_control;
	uint8_t m_maxrpm_adc_select;
	uint8_t m_maxrpm_last_shift;
	int8_t m_maxrpm_p1_shift;
	int8_t m_maxrpm_p2_shift;
	uint8_t m_spyhunt_sprite_color_mask;
	int16_t m_spyhunt_scroll_offset;
	int16_t m_spyhunt_scrollx;
	int16_t m_spyhunt_scrolly;
	tilemap_t *m_bg_tilemap;
	tilemap_t *m_alpha_tilemap;

	DECLARE_WRITE8_MEMBER(mcr3_videoram_w);
	DECLARE_WRITE8_MEMBER(spyhunt_videoram_w);
	DECLARE_WRITE8_MEMBER(spyhunt_alpharam_w);
	DECLARE_WRITE8_MEMBER(spyhunt_scroll_value_w);
	DECLARE_WRITE8_MEMBER(mcrmono_control_port_w);
	DECLARE_READ8_MEMBER(demoderm_ip1_r);
	DECLARE_READ8_MEMBER(demoderm_ip2_r);
	DECLARE_WRITE8_MEMBER(demoderm_op6_w);
	DECLARE_READ8_MEMBER(maxrpm_ip1_r);
	DECLARE_READ8_MEMBER(maxrpm_ip2_r);
	DECLARE_WRITE8_MEMBER(maxrpm_op5_w);
	DECLARE_WRITE8_MEMBER(maxrpm_op6_w);
	DECLARE_READ8_MEMBER(rampage_ip4_r);
	DECLARE_WRITE8_MEMBER(rampage_op6_w);
	DECLARE_READ8_MEMBER(powerdrv_ip2_r);
	DECLARE_WRITE8_MEMBER(powerdrv_op5_w);
	DECLARE_WRITE8_MEMBER(powerdrv_op6_w);
	DECLARE_READ8_MEMBER(stargrds_ip0_r);
	DECLARE_WRITE8_MEMBER(stargrds_op5_w);
	DECLARE_WRITE8_MEMBER(stargrds_op6_w);
	DECLARE_READ8_MEMBER(spyhunt_ip1_r);
	DECLARE_READ8_MEMBER(spyhunt_ip2_r);
	DECLARE_WRITE8_MEMBER(spyhunt_op4_w);
	DECLARE_READ8_MEMBER(turbotag_ip2_r);
	DECLARE_READ8_MEMBER(turbotag_kludge_r);
	DECLARE_DRIVER_INIT(crater);
	DECLARE_DRIVER_INIT(demoderm);
	DECLARE_DRIVER_INIT(turbotag);
	DECLARE_DRIVER_INIT(powerdrv);
	DECLARE_DRIVER_INIT(stargrds);
	DECLARE_DRIVER_INIT(maxrpm);
	DECLARE_DRIVER_INIT(rampage);
	DECLARE_DRIVER_INIT(spyhunt);
	DECLARE_DRIVER_INIT(sarge);
	TILE_GET_INFO_MEMBER(mcrmono_get_bg_tile_info);
	TILEMAP_MAPPER_MEMBER(spyhunt_bg_scan);
	TILE_GET_INFO_MEMBER(spyhunt_get_bg_tile_info);
	TILE_GET_INFO_MEMBER(spyhunt_get_alpha_tile_info);
	DECLARE_VIDEO_START(mcrmono);
	DECLARE_VIDEO_START(spyhunt);
	DECLARE_PALETTE_INIT(spyhunt);
	uint32_t screen_update_mcr3(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	uint32_t screen_update_spyhunt(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	void mcr3_update_sprites(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect, int color_mask, int code_xor, int dx, int dy, int interlaced);
	void mcr_common_init();

};
