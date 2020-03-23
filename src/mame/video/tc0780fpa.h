// license:LGPL-2.1+
// copyright-holders:Ville Linde, Angelo Salese, hap

#ifndef MAME_VIDEO_TC0780FPA_H
#define MAME_VIDEO_TC0780FPA_H

#pragma once

#include "video/poly.h"


struct tc0780fpa_polydata
{
	int tex_base_x;
	int tex_base_y;
	int tex_wrap_x;
	int tex_wrap_y;
};


class tc0780fpa_renderer : public poly_manager<float, tc0780fpa_polydata, 6, 10000>
{
public:
	tc0780fpa_renderer(device_t &parent, screen_device &screen, const uint8_t *texture_ram);

	void render_solid_scan(int32_t scanline, const extent_t &extent, const tc0780fpa_polydata &extradata, int threadid);
	void render_shade_scan(int32_t scanline, const extent_t &extent, const tc0780fpa_polydata &extradata, int threadid);
	void render_texture_scan(int32_t scanline, const extent_t &extent, const tc0780fpa_polydata &extradata, int threadid);

	void render(uint16_t *polygon_fifo, int length);
	void draw(bitmap_ind16 &bitmap, const rectangle &cliprect);
	void swap_buffers();

private:
	std::unique_ptr<bitmap_ind16> m_fb[2];
	std::unique_ptr<bitmap_ind16> m_zb;
	const uint8_t *m_texture;

	rectangle m_cliprect;

	int m_current_fb;
};


class tc0780fpa_device : public device_t, public device_video_interface
{
public:
	tc0780fpa_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	void draw(bitmap_ind16 &bitmap, const rectangle &cliprect);

	DECLARE_READ16_MEMBER(tex_addr_r);
	DECLARE_WRITE16_MEMBER(tex_addr_w);
	DECLARE_WRITE16_MEMBER(tex_w);
	DECLARE_WRITE16_MEMBER(poly_fifo_w);
	DECLARE_WRITE16_MEMBER(render_w);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_stop() override;
	virtual void device_reset() override;

private:
	std::unique_ptr<uint8_t[]> m_texture;
	std::unique_ptr<uint16_t[]> m_poly_fifo;
	int m_poly_fifo_ptr;

	std::unique_ptr<tc0780fpa_renderer> m_renderer;

	uint16_t m_tex_address;
	uint16_t m_tex_offset;
	int m_texbase_x;
	int m_texbase_y;
};

DECLARE_DEVICE_TYPE(TC0780FPA, tc0780fpa_device)

#endif // MAME_VIDEO_TC0780FPA_H
