// license:BSD-3-Clause
// copyright-holders:Philip Bennett
/*************************************************************************

     Microprose Games 3D hardware

*************************************************************************/
#ifndef MAME_INCLUDES_MICRO3D_H
#define MAME_INCLUDES_MICRO3D_H

#pragma once

#include "cpu/tms34010/tms34010.h"
#include "cpu/mcs51/mcs51.h"
#include "sound/upd7759.h"
#include "machine/mc68681.h"


#define HOST_MONITOR_DISPLAY        0
#define VGB_MONITOR_DISPLAY         0
#define DRMATH_MONITOR_DISPLAY      0


struct micro3d_vtx
{
	int32_t x, y, z;
};

enum planes
{
		CLIP_Z_MIN,
		CLIP_Z_MAX,
		CLIP_X_MIN,
		CLIP_X_MAX,
		CLIP_Y_MIN,
		CLIP_Y_MAX
};

class micro3d_sound_device;

class micro3d_state : public driver_device
{
public:
	enum
	{
		TIMER_MAC_DONE,
		TIMER_ADC_DONE
	};

	micro3d_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_audiocpu(*this, "audiocpu"),
		m_upd7759(*this, "upd7759"),
		m_drmath(*this, "drmath"),
		m_vgb(*this, "vgb"),
		m_palette(*this, "palette"),
		m_duart68681(*this, "duart68681"),
		m_noise_1(*this, "noise_1"),
		m_noise_2(*this, "noise_2"),
		m_vertex(*this, "vertex"),
		m_sound_sw(*this, "SOUND_SW"),
		m_volume(*this, "VOLUME"),
		m_joystick_x(*this, "JOYSTICK_X"),
		m_joystick_y(*this, "JOYSTICK_Y"),
		m_throttle(*this, "THROTTLE"),
		m_shared_ram(*this, "shared_ram"),
		m_mac_sram(*this, "mac_sram"),
		m_sprite_vram(*this, "sprite_vram") { }

	required_device<cpu_device> m_maincpu;
	required_device<i8051_device> m_audiocpu;
	required_device<upd7759_device> m_upd7759;
	required_device<cpu_device> m_drmath;
	required_device<tms34010_device> m_vgb;
	required_device<palette_device> m_palette;
	required_device<mc68681_device> m_duart68681;
	required_device<micro3d_sound_device> m_noise_1;
	required_device<micro3d_sound_device> m_noise_2;
	required_memory_region m_vertex;

	required_ioport m_sound_sw;
	required_ioport m_volume;
	optional_ioport m_joystick_x;
	optional_ioport m_joystick_y;
	optional_ioport m_throttle;

	required_shared_ptr<uint16_t> m_shared_ram;
	uint8_t               m_m68681_tx0;

	/* Sound */
	uint8_t               m_sound_port_latch[4];

	/* TI UART */
	uint8_t               m_ti_uart[9];
	int                 m_ti_uart_mode_cycle;
	int                 m_ti_uart_sync_cycle;

	/* ADC */
	uint8_t               m_adc_val;

	/* Hardware version-check latch for BOTSS 1.1a */
	uint8_t               m_botss_latch;

	/* MAC */
	required_shared_ptr<uint32_t> m_mac_sram;
	uint32_t              m_sram_r_addr;
	uint32_t              m_sram_w_addr;
	uint32_t              m_vtx_addr;
	uint32_t              m_mrab11;
	uint32_t              m_mac_stat;
	uint32_t              m_mac_inst;

	/* 2D video */
	required_shared_ptr<uint16_t> m_sprite_vram;
	uint16_t              m_creg;
	uint16_t              m_xfer3dk;

	/* 3D pipeline */
	uint32_t              m_pipe_data;
	uint32_t              m_pipeline_state;
	int32_t               m_vtx_fifo[512];
	uint32_t              m_fifo_idx;
	uint32_t              m_draw_cmd;
	int                 m_draw_state;
	int32_t               m_x_min;
	int32_t               m_x_max;
	int32_t               m_y_min;
	int32_t               m_y_max;
	int32_t               m_z_min;
	int32_t               m_z_max;
	int32_t               m_x_mid;
	int32_t               m_y_mid;
	int                 m_dpram_bank;
	uint32_t              m_draw_dpram[1024];
	std::unique_ptr<uint16_t[]>              m_frame_buffers[2];
	std::unique_ptr<uint16_t[]>              m_tmp_buffer;
	int                 m_drawing_buffer;
	int                 m_display_buffer;

	DECLARE_WRITE16_MEMBER(micro3d_ti_uart_w);
	DECLARE_READ16_MEMBER(micro3d_ti_uart_r);
	DECLARE_WRITE32_MEMBER(micro3d_scc_w);
	DECLARE_READ32_MEMBER(micro3d_scc_r);
	DECLARE_WRITE32_MEMBER(micro3d_mac1_w);
	DECLARE_READ32_MEMBER(micro3d_mac2_r);
	DECLARE_WRITE32_MEMBER(micro3d_mac2_w);
	DECLARE_READ16_MEMBER(micro3d_encoder_h_r);
	DECLARE_READ16_MEMBER(micro3d_encoder_l_r);
	DECLARE_READ16_MEMBER(micro3d_adc_r);
	DECLARE_WRITE16_MEMBER(micro3d_adc_w);
	DECLARE_READ16_MEMBER(botss_140000_r);
	DECLARE_READ16_MEMBER(botss_180000_r);
	DECLARE_WRITE16_MEMBER(micro3d_reset_w);
	DECLARE_WRITE16_MEMBER(host_drmath_int_w);
	DECLARE_WRITE32_MEMBER(micro3d_shared_w);
	DECLARE_READ32_MEMBER(micro3d_shared_r);
	DECLARE_WRITE32_MEMBER(drmath_int_w);
	DECLARE_WRITE32_MEMBER(drmath_intr2_ack);
	DECLARE_WRITE16_MEMBER(micro3d_creg_w);
	DECLARE_WRITE16_MEMBER(micro3d_xfer3dk_w);
	DECLARE_WRITE32_MEMBER(micro3d_fifo_w);
	DECLARE_WRITE32_MEMBER(micro3d_alt_fifo_w);
	DECLARE_READ32_MEMBER(micro3d_pipe_r);
	DECLARE_CUSTOM_INPUT_MEMBER(botss_hwchk_r);
	DECLARE_WRITE8_MEMBER(micro3d_snd_dac_a);
	DECLARE_WRITE8_MEMBER(micro3d_snd_dac_b);
	DECLARE_WRITE8_MEMBER(micro3d_sound_io_w);
	DECLARE_READ8_MEMBER(micro3d_sound_io_r);
	DECLARE_DRIVER_INIT(micro3d);
	DECLARE_DRIVER_INIT(botss);
	virtual void machine_reset() override;
	virtual void video_start() override;
	virtual void video_reset() override;
	INTERRUPT_GEN_MEMBER(micro3d_vblank);
	TIMER_CALLBACK_MEMBER(mac_done_callback);
	TIMER_CALLBACK_MEMBER(adc_done_callback);
	DECLARE_WRITE8_MEMBER(micro3d_upd7759_w);
	DECLARE_WRITE8_MEMBER(data_from_i8031);
	DECLARE_READ8_MEMBER(data_to_i8031);
	DECLARE_WRITE_LINE_MEMBER(duart_irq_handler);
	DECLARE_READ8_MEMBER(duart_input_r);
	DECLARE_WRITE8_MEMBER(duart_output_w);
	DECLARE_WRITE_LINE_MEMBER(duart_txb);
	DECLARE_WRITE_LINE_MEMBER(tms_interrupt);
	TMS340X0_SCANLINE_IND16_CB_MEMBER(scanline_update);

	/* 3D graphics */
	int inside(micro3d_vtx *v, enum planes plane);
	micro3d_vtx intersect(micro3d_vtx *v1, micro3d_vtx *v2, enum planes plane);
	inline void write_span(uint32_t y, uint32_t x);
	void draw_line(uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2);
	void rasterise_spans(uint32_t min_y, uint32_t max_y, uint32_t attr);
	int clip_triangle(micro3d_vtx *v, micro3d_vtx *vout, int num_vertices, enum planes plane);
	void draw_triangles(uint32_t attr);


protected:
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;
};

#endif // MAME_INCLUDES_MICRO3D_H
