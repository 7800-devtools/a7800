// license:BSD-3-Clause
// copyright-holders:David Haywood

#include "emu.h"
#include "tecmo_mix.h"


DEFINE_DEVICE_TYPE(TECMO_MIXER, tecmo_mix_device, "tecmo_mix", "Tecmo 16-bit Mixer")

tecmo_mix_device::tecmo_mix_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, TECMO_MIXER, tag, owner, clock),
		device_video_interface(mconfig, *this),
		m_sprpri_shift(0),
		m_sprbln_shift(0),
		m_sprcol_shift(0),

		m_spblend_source(0),
		m_fgblend_source(0),

		m_bgblend_comp(0),
		m_fgblend_comp(0),
		m_txblend_comp(0),
		m_spblend_comp(0),

		m_bgregular_comp(0),
		m_fgregular_comp(0),
		m_txregular_comp(0),
		m_spregular_comp(0),

		m_revspritetile(0),
		m_bgpen(0)

{
}


void tecmo_mix_device::device_start()
{
}

void tecmo_mix_device::device_reset()
{
}



void tecmo_mix_device::set_mixer_shifts(device_t &device, int sprpri_shift, int sprbln_shift, int sprcol_shift)
{
	tecmo_mix_device &dev = downcast<tecmo_mix_device &>(device);
	dev.m_sprpri_shift = sprpri_shift;
	dev.m_sprbln_shift = sprbln_shift;
	dev.m_sprcol_shift = sprcol_shift;
}

void tecmo_mix_device::set_blendcols(device_t &device, int bgblend_comp, int fgblend_comp, int txblend_comp, int spblend_comp)
{
	tecmo_mix_device &dev = downcast<tecmo_mix_device &>(device);
	dev.m_bgblend_comp = bgblend_comp;
	dev.m_fgblend_comp = fgblend_comp;
	dev.m_txblend_comp = txblend_comp;
	dev.m_spblend_comp = spblend_comp;
}

void tecmo_mix_device::set_regularcols(device_t &device, int bgregular_comp, int fgregular_comp, int txregular_comp, int spregular_comp)
{
	tecmo_mix_device &dev = downcast<tecmo_mix_device &>(device);
	dev.m_bgregular_comp = bgregular_comp;
	dev.m_fgregular_comp = fgregular_comp;
	dev.m_txregular_comp = txregular_comp;
	dev.m_spregular_comp = spregular_comp;
}

void tecmo_mix_device::set_blendsource(device_t &device, int spblend_source, int fgblend_source)
{
	tecmo_mix_device &dev = downcast<tecmo_mix_device &>(device);
	dev.m_spblend_source = spblend_source;
	dev.m_fgblend_source = fgblend_source;
}

void tecmo_mix_device::set_revspritetile(device_t &device)
{
	tecmo_mix_device &dev = downcast<tecmo_mix_device &>(device);
	dev.m_revspritetile = 3;
}

void tecmo_mix_device::set_bgpen(device_t &device, int bgpen)
{
	tecmo_mix_device &dev = downcast<tecmo_mix_device &>(device);
	dev.m_bgpen = bgpen;
}

uint32_t tecmo_mix_device::sum_colors(const pen_t *pal, int c1_idx, int c2_idx)
{
	const pen_t c1 = pal[c1_idx];
	const pen_t c2 = pal[c2_idx];

	const int c1_a = (c1 >> 24) & 0xFF;
	const int c1_r = (c1 >> 16) & 0xFF;
	const int c1_g = (c1 >> 8)  & 0xFF;
	const int c1_b = c1 & 0xFF;

	const int c2_a = (c2 >> 24) & 0xFF;
	const int c2_r = (c2 >> 16) & 0xFF;
	const int c2_g = (c2 >> 8)  & 0xFF;
	const int c2_b = c2 & 0xFF;

	const uint8_t a = (std::min)(0xFF, c1_a + c2_a);
	const uint8_t r = (std::min)(0xFF, c1_r + c2_r);
	const uint8_t g = (std::min)(0xFF, c1_g + c2_g);
	const uint8_t b = (std::min)(0xFF, c1_b + c2_b);

	return ((a << 24) | (r << 16) | (g << 8) | b);
}

void tecmo_mix_device::mix_bitmaps(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect, palette_device &palette, bitmap_ind16* bitmap_bg, bitmap_ind16* bitmap_fg, bitmap_ind16* bitmap_tx, bitmap_ind16* bitmap_sp)
{
	//int frame = (screen.frame_number()) & 1;
	// note this game has no tx layer, comments relate to other drivers

	int y, x;
	const pen_t *paldata = palette.pens();

	for (y = cliprect.min_y; y <= cliprect.max_y; y++)
	{
		uint32_t *dd = &bitmap.pix32(y);
		uint16_t *sd2 = &bitmap_sp->pix16(y);
		uint16_t *fg = &bitmap_fg->pix16(y);
		uint16_t *bg = &bitmap_bg->pix16(y);

		for (x = cliprect.min_x; x <= cliprect.max_x; x++)
		{
			uint16_t sprpixel = (sd2[x]);

			uint16_t m_sprpri = (sprpixel >> m_sprpri_shift) & 0x3;
			uint16_t m_sprbln = (sprpixel >> m_sprbln_shift) & 0x1;
			uint16_t m_sprcol = (sprpixel >> m_sprcol_shift) & 0xf;

			sprpixel = (sprpixel & 0xf) | (m_sprcol << 4);

			//sprpixel &= 0xff;

			uint16_t fgpixel = (fg[x]);
			uint16_t fgbln = (fgpixel & 0x0100) >> 8;
			fgpixel &= 0xff;

			uint16_t bgpixel = (bg[x]);
			bgpixel &= 0xff;

			if (sprpixel&0xf)
			{
				if (m_sprpri == (0 ^ m_revspritetile)) // behind all
				{
					if (fgpixel & 0xf) // is the fg used?
					{
						if (fgbln)
						{
							dd[x] = machine().rand();
						}
						else
						{
							// solid FG
							dd[x] = paldata[fgpixel + m_fgregular_comp];
						}
					}
					else if (bgpixel & 0x0f)
					{
						// solid BG
						dd[x] = paldata[bgpixel + m_bgregular_comp];
					}
					else
					{
						if (m_sprbln)
						{ // sprite is blended with bgpen?
							dd[x] = machine().rand();
						}
						else
						{
							// solid sprite
							dd[x] = paldata[sprpixel + m_spregular_comp];
						}
					}
				}
				else  if (m_sprpri == (1 ^ m_revspritetile)) // above bg, behind tx, fg
				{
					if (fgpixel & 0xf) // is the fg used?
					{
						if (fgbln)
						{
							if (m_sprbln)
							{
								// needs if bgpixel & 0xf check?

								// fg is used and blended with sprite, sprite is used and blended with bg?  -- used on 'trail' of ball when ball is under the transparent area
								dd[x] = sum_colors(paldata, bgpixel + m_bgblend_comp, sprpixel + m_spblend_source); // WRONG??
							}
							else
							{
								// fg is used and blended with opaque sprite
								dd[x] = sum_colors(paldata, fgpixel + m_fgblend_source, sprpixel + m_spblend_comp);
							}
						}
						else
						{
							// fg is used and opaque
							dd[x] = paldata[fgpixel + m_fgregular_comp];
						}
					}
					else
					{
						if (m_sprbln)
						{
							// needs if bgpixel & 0xf check?

							//fg isn't used, sprite is used and blended with bg? -- used on trail of ball / flippers (looks odd)  -- some ninja gaiden enemy deaths (when behind fg) (looks ok?)  (maybe we need to check for colour saturation?)
							dd[x] = sum_colors(paldata, bgpixel + m_bgblend_comp, sprpixel + m_spblend_source);
						}
						else
						{
							// fg isn't used, sprite is used and is opaque
							dd[x] = paldata[sprpixel + m_spregular_comp];
						}
					}
				}
				else if (m_sprpri == (2 ^ m_revspritetile)) // above bg,fg, behind tx
				{
					if (m_sprbln)
					{
						if (fgpixel & 0xf) // is the fg used?
						{
							if (fgbln)
							{
								// blended sprite over blended fg pixel?
								dd[x] = machine().rand();
							}
							else
							{
								// blended sprite over solid fgpixel?
								dd[x] = sum_colors(paldata, fgpixel + m_fgblend_comp, sprpixel + m_spblend_source);
							}
						}
						else // needs if bgpixel & 0xf check?
						{
							// blended sprite over solid bg pixel
							dd[x] = sum_colors(paldata, bgpixel + m_bgblend_comp, sprpixel + m_spblend_source);
							//  dd[x] = machine().rand();
						}
					}
					else
					{
						dd[x] = paldata[sprpixel + m_spregular_comp];
						//dd[x] = machine().rand();
						// the bad tiles on the wildfang map (shown between levels) are drawn here.. why? looks like they should be transparent?
						// most wildfang sprites use this and are fine, so what's going wrong?
					}
				}

				else if (m_sprpri == (3 ^ m_revspritetile)) // above all?
				{
					if (m_sprbln)
					{
						// unusued by this game?
						dd[x] = machine().rand();
					}
					else
					{
						dd[x] = paldata[sprpixel + m_spregular_comp];
					}

				}
			}
			else // NON SPRITE CASES
			{
				if (fgpixel & 0x0f)
				{
					if (fgbln)
					{
						// needs if bgpixel & 0xf check?
						dd[x] = sum_colors(paldata, fgpixel + m_fgblend_source, bgpixel + m_bgblend_comp);

					}
					else
					{
						dd[x] = paldata[fgpixel + m_fgregular_comp];
					}
				}
				else if (bgpixel & 0x0f)
				{
					dd[x] = paldata[bgpixel + m_bgregular_comp];
				}
				else
				{
					dd[x] = paldata[m_bgpen];// pen 0x200 on raiga  0xb00 on spbactn
				}
			}
		}
	}
}
