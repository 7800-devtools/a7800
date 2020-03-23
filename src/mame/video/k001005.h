// license:BSD-3-Clause
// copyright-holders:David Haywood
#ifndef MAME_VIDEO_K001005_H
#define MAME_VIDEO_K001005_H

#pragma once

#include "video/poly.h"
#include "cpu/sharc/sharc.h"

#include <float.h>


struct k001005_polydata
{
	uint32_t color;
	int texture_x, texture_y;
	int texture_width, texture_height;
	int texture_page;
	int texture_palette;
	int texture_mirror_x;
	int texture_mirror_y;
	int light_r, light_g, light_b;
	int ambient_r, ambient_g, ambient_b;
	int fog_r, fog_g, fog_b;
	uint32_t flags;
};

enum k001005_param
{
	K001005_LIGHT_R,
	K001005_LIGHT_G,
	K001005_LIGHT_B,
	K001005_AMBIENT_R,
	K001005_AMBIENT_G,
	K001005_AMBIENT_B,
	K001005_FOG_R,
	K001005_FOG_G,
	K001005_FOG_B,
	K001005_FAR_Z
};


class k001005_renderer : public poly_manager<float, k001005_polydata, 8, 50000>
{
public:
	k001005_renderer(device_t &parent, screen_device &screen, device_t *k001006);

	void reset();
	void push_data(uint32_t data);
	void render_polygons();
	void swap_buffers();
	bool fifo_filled();
	void draw(bitmap_rgb32 &bitmap, const rectangle &cliprect);
	void set_param(k001005_param param, uint32_t value);

	void draw_scanline_2d(int32_t scanline, const extent_t &extent, const k001005_polydata &extradata, int threadid);
	void draw_scanline_2d_tex(int32_t scanline, const extent_t &extent, const k001005_polydata &extradata, int threadid);
	void draw_scanline(int32_t scanline, const extent_t &extent, const k001005_polydata &extradata, int threadid);
	void draw_scanline_tex(int32_t scanline, const extent_t &extent, const k001005_polydata &extradata, int threadid);
	void draw_scanline_gouraud_blend(int32_t scanline, const extent_t &extent, const k001005_polydata &extradata, int threadid);

	static constexpr int POLY_Z = 0;
	static constexpr int POLY_FOG = 1;
	static constexpr int POLY_BRI = 2;
	static constexpr int POLY_U = 3;
	static constexpr int POLY_V = 4;
	static constexpr int POLY_W = 5;
	static constexpr int POLY_A = 2;
	static constexpr int POLY_R = 3;
	static constexpr int POLY_G = 4;
	static constexpr int POLY_B = 5;

private:
	std::unique_ptr<bitmap_rgb32> m_fb[2];
	std::unique_ptr<bitmap_ind32> m_zb;
	rectangle m_cliprect;
	int m_fb_page;

	std::unique_ptr<uint32_t[]> m_3dfifo;
	int m_3dfifo_ptr;

	vertex_t m_prev_v[4];

	uint32_t m_light_r;
	uint32_t m_light_g;
	uint32_t m_light_b;
	uint32_t m_ambient_r;
	uint32_t m_ambient_g;
	uint32_t m_ambient_b;
	uint32_t m_fog_r;
	uint32_t m_fog_g;
	uint32_t m_fog_b;
	float m_far_z;

	device_t *m_k001006;

	std::unique_ptr<int[]> m_tex_mirror_table[2][8];
};


class k001005_device : public device_t, public device_video_interface
{
public:
	k001005_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	static void set_texel_chip(device_t &device, const char *tag);

	void draw(bitmap_rgb32 &bitmap, const rectangle &cliprect);
	void swap_buffers();

	DECLARE_READ32_MEMBER( read );
	DECLARE_WRITE32_MEMBER( write );

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_stop() override;
	virtual void device_reset() override;

private:
	// internal state
	device_t *m_k001006;
	const char *m_k001006_tag;

	std::unique_ptr<uint16_t[]>    m_ram[2];
	std::unique_ptr<uint32_t[]>     m_fifo;
	uint32_t m_status;

	int m_ram_ptr;
	int m_fifo_read_ptr;
	int m_fifo_write_ptr;
	uint32_t m_reg_far_z;


	k001005_renderer *m_renderer;
};

DECLARE_DEVICE_TYPE(K001005, k001005_device)


#define MCFG_K001005_TEXEL_CHIP(_tag) \
	k001005_device::set_texel_chip(*device, _tag);

#endif // MAME_VIDEO_K001005_H
