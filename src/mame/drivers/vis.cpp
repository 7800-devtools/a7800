// license:BSD-3-Clause
// copyright-holders:Carl

#include "emu.h"
#include "bus/isa/isa_cards.h"
#include "cpu/i86/i286.h"
#include "machine/8042kbdc.h"
#include "machine/at.h"
#include "sound/262intf.h"
#include "sound/dac.h"
#include "sound/volt_reg.h"
#include "video/pc_vga.h"
#include "speaker.h"


class vis_audio_device : public device_t,
						 public device_isa16_card_interface
{
public:
	vis_audio_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	DECLARE_READ8_MEMBER(pcm_r);
	DECLARE_WRITE8_MEMBER(pcm_w);
protected:
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;
	virtual void dack16_w(int line, uint16_t data) override;
	virtual void device_add_mconfig(machine_config &config) override;
private:
	required_device<dac_word_interface> m_rdac;
	required_device<dac_word_interface> m_ldac;
	uint16_t m_count;
	uint16_t m_sample[2];
	uint8_t m_index[2]; // unknown indexed registers, volume?
	uint8_t m_data[2][16];
	uint8_t m_mode;
	uint8_t m_stat;
	uint8_t m_dmalen;
	unsigned int m_sample_byte;
	unsigned int m_samples;
	emu_timer *m_pcm;
};

DEFINE_DEVICE_TYPE(VIS_AUDIO, vis_audio_device, "vis_pcm", "vis_pcm")

vis_audio_device::vis_audio_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, VIS_AUDIO, tag, owner, clock),
	device_isa16_card_interface(mconfig, *this),
	m_rdac(*this, "rdac"),
	m_ldac(*this, "ldac")
{
}

void vis_audio_device::device_start()
{
	set_isa_device();
	m_isa->set_dma_channel(7, this, false);
	m_isa->install_device(0x0220, 0x022f, read8_delegate(FUNC(vis_audio_device::pcm_r), this), write8_delegate(FUNC(vis_audio_device::pcm_w), this));
	m_isa->install_device(0x0388, 0x038b, read8_delegate(FUNC(ymf262_device::read), subdevice<ymf262_device>("ymf262")), write8_delegate(FUNC(ymf262_device::write), subdevice<ymf262_device>("ymf262")));
	m_pcm = timer_alloc();
	m_pcm->adjust(attotime::never);
}

void vis_audio_device::device_reset()
{
	m_count = 0xffff;
	m_sample_byte = 0;
	m_samples = 0;
	m_mode = 0;
	m_index[0] = m_index[1] = 0;
	m_stat = 0;
}

void vis_audio_device::dack16_w(int line, uint16_t data)
{
	m_sample[m_samples++] = data;
	if(m_samples >= ((m_dmalen & 2) ? 1 : 2))
		m_isa->drq7_w(CLEAR_LINE);
}

void vis_audio_device::device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr)
{
	if(m_count == 0xffff)
	{
		m_isa->drq7_w(ASSERT_LINE);
		m_samples = 0;
		return;
	}
	switch(m_mode & 0x88)
	{
		case 0x80: // 8bit mono
		{
			uint8_t sample = m_sample[m_sample_byte >> 1] >> ((m_sample_byte & 1) * 8);
			m_ldac->write(sample << 8);
			m_rdac->write(sample << 8);
			m_sample_byte++;
			break;
		}
		case 0x00: // 8bit stereo
			m_ldac->write(m_sample[m_sample_byte >> 1] << 8);
			m_rdac->write(m_sample[m_sample_byte >> 1] & 0xff00);
			m_sample_byte += 2;
			break;
		case 0x88: // 16bit mono
			m_ldac->write(m_sample[m_sample_byte >> 1] ^ 0x8000);
			m_rdac->write(m_sample[m_sample_byte >> 1] ^ 0x8000);
			m_sample_byte += 2;
			break;
		case 0x08: // 16bit stereo
			m_ldac->write(m_sample[0] ^ 0x8000);
			m_rdac->write(m_sample[1] ^ 0x8000);
			m_sample_byte += 4;
			break;
	}

	if(m_sample_byte >= ((m_dmalen & 2) ? 2 : 4))
	{
		m_sample_byte = 0;
		m_samples = 0;
		if(m_count)
			m_isa->drq7_w(ASSERT_LINE);
		else
		{
			m_ldac->write(0x8000);
			m_rdac->write(0x8000);
			m_stat = 4;
			m_isa->irq7_w(ASSERT_LINE);
		}
		m_count--;
	}
}

MACHINE_CONFIG_MEMBER( vis_audio_device::device_add_mconfig )
	MCFG_SPEAKER_STANDARD_STEREO("lspeaker", "rspeaker")
	MCFG_SOUND_ADD("ymf262", YMF262, XTAL_14_31818MHz)
	MCFG_SOUND_ROUTE(0, "lspeaker", 1.00)
	MCFG_SOUND_ROUTE(1, "rspeaker", 1.00)
	MCFG_SOUND_ROUTE(2, "lspeaker", 1.00)
	MCFG_SOUND_ROUTE(3, "rspeaker", 1.00)
	MCFG_SOUND_ADD("ldac", DAC_16BIT_R2R, 0) MCFG_SOUND_ROUTE(ALL_OUTPUTS, "lspeaker", 1.0) // unknown DAC
	MCFG_SOUND_ADD("rdac", DAC_16BIT_R2R, 0) MCFG_SOUND_ROUTE(ALL_OUTPUTS, "rspeaker", 1.0) // unknown DAC
	MCFG_DEVICE_ADD("vref", VOLTAGE_REGULATOR, 0) MCFG_VOLTAGE_REGULATOR_OUTPUT(5.0)
	MCFG_SOUND_ROUTE_EX(0, "ldac", 1.0, DAC_VREF_POS_INPUT) MCFG_SOUND_ROUTE_EX(0, "ldac", -1.0, DAC_VREF_NEG_INPUT)
	MCFG_SOUND_ROUTE_EX(0, "rdac", 1.0, DAC_VREF_POS_INPUT) MCFG_SOUND_ROUTE_EX(0, "rdac", -1.0, DAC_VREF_NEG_INPUT)
MACHINE_CONFIG_END

READ8_MEMBER(vis_audio_device::pcm_r)
{
	switch(offset)
	{
		case 0x00:
			return m_mode;
		case 0x02:
			return m_data[0][m_index[0]];
		case 0x04:
			return m_data[1][m_index[1]];
		case 0x09:
			m_isa->irq7_w(CLEAR_LINE);
			return m_stat;
		case 0x0c:
			return m_count & 0xff;
		case 0x0e:
			return m_count >> 8;
		case 0x0f:
			//cdrom related?
			break;
		default:
			logerror("unknown pcm read %04x\n", offset);
			break;
	}
	return 0;
}

WRITE8_MEMBER(vis_audio_device::pcm_w)
{
	switch(offset)
	{
		case 0x00:
			m_mode = data;
			break;
		case 0x02:
			m_data[0][m_index[0] & 0xf] = data;
			return;
		case 0x03:
			m_index[0] = data;
			return;
		case 0x04:
			m_data[1][m_index[1] & 0xf] = data;
			return;
		case 0x05:
			m_index[1] = data;
			return;
		case 0x09:
			m_dmalen = data;
			return;
		case 0x0c:
			m_count = (m_count & 0xff00) | data;
			break;
		case 0x0e:
			m_count = (m_count & 0xff) | (data << 8);
			break;
		case 0x0f:
			//cdrom related?
			return;
		default:
			logerror("unknown pcm write %04x %02x\n", offset, data);
			return;
	}
	if((m_mode & 0x10) && (m_count != 0xffff))
	{
		const int rates[] = {44100, 22050, 11025, 5512};
		m_samples = 0;
		m_sample_byte = 0;
		m_stat = 0;
		m_isa->drq7_w(ASSERT_LINE);
		attotime rate = attotime::from_hz(rates[(m_mode >> 5) & 3]);
		m_pcm->adjust(rate, 0, rate);
	}
	else if(!(m_mode & 0x10))
		m_pcm->adjust(attotime::never);
}

class vis_vga_device : public svga_device,
					   public device_isa16_card_interface
{
public:
	vis_vga_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	DECLARE_READ8_MEMBER(vga_r);
	DECLARE_WRITE8_MEMBER(vga_w);
	DECLARE_READ8_MEMBER(visvgamem_r);
	DECLARE_WRITE8_MEMBER(visvgamem_w);
protected:
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_add_mconfig(machine_config &config) override;
	virtual void recompute_params() override;
private:
	void vga_vh_yuv8(bitmap_rgb32 &bitmap, const rectangle &cliprect);
	rgb_t yuv_to_rgb(int y, int u, int v) const;
	virtual uint32_t screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect) override;

	int m_extcnt;
	uint8_t m_extreg;
	uint8_t m_interlace;
	uint16_t m_wina, m_winb;
	uint8_t m_shift256, m_dw, m_8bit_640;
	uint8_t m_crtc_regs[0x31];
};

DEFINE_DEVICE_TYPE(VIS_VGA, vis_vga_device, "vis_vga", "vis_vga")

vis_vga_device::vis_vga_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: svga_device(mconfig, VIS_VGA, tag, owner, clock),
	device_isa16_card_interface(mconfig, *this)
{
	m_palette.set_tag("palette");
	m_screen.set_tag("screen");
}

MACHINE_CONFIG_MEMBER( vis_vga_device::device_add_mconfig )
	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_RAW_PARAMS(XTAL_25_1748MHz,900,0,640,526,0,480)
	MCFG_SCREEN_UPDATE_DEVICE(DEVICE_SELF, vis_vga_device, screen_update)
	MCFG_PALETTE_ADD("palette", 0x100)
MACHINE_CONFIG_END

void vis_vga_device::recompute_params()
{
	int vblank_period,hblank_period;
	attoseconds_t refresh;
	uint8_t hclock_m = (!vga.gc.alpha_dis) ? (vga.sequencer.data[1]&1)?8:9 : 8;
	int pixel_clock;
	int xtal = (vga.miscellaneous_output & 0xc) ? XTAL_28_63636MHz : XTAL_25_1748MHz;
	int divisor = 1; // divisor is 2 for 15/16 bit rgb/yuv modes and 3 for 24bit

	/* safety check */
	if(!vga.crtc.horz_disp_end || !vga.crtc.vert_disp_end || !vga.crtc.horz_total || !vga.crtc.vert_total)
		return;

	rectangle visarea(0, ((vga.crtc.horz_disp_end + 1) * ((float)(hclock_m)/divisor))-1, 0, vga.crtc.vert_disp_end);

	vblank_period = (vga.crtc.vert_total + 2);
	hblank_period = ((vga.crtc.horz_total + 5) * ((float)(hclock_m)/divisor));

	/* TODO: 10b and 11b settings aren't known */
	pixel_clock = xtal / (((vga.sequencer.data[1]&8) >> 3) + 1);

	refresh  = HZ_TO_ATTOSECONDS(pixel_clock) * (hblank_period) * vblank_period;
	machine().first_screen()->configure((hblank_period), (vblank_period), visarea, refresh );
	//popmessage("%d %d\n",vga.crtc.horz_total * 8,vga.crtc.vert_total);
	m_vblank_timer->adjust( machine().first_screen()->time_until_pos((vga.crtc.vert_blank_start + vga.crtc.vert_blank_end)) );
}

rgb_t vis_vga_device::yuv_to_rgb(int y, int u, int v) const
{
	u -= 128;
	v -= 128;
	double r = y + v * 1.371;
	double g = y - u * 0.337 - v * 0.698;
	double b = y + u * 1.733;

	return rgb_t(rgb_t::clamp(r), rgb_t::clamp(g), rgb_t::clamp(b));
}

void vis_vga_device::vga_vh_yuv8(bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
	const uint32_t IV = 0xff000000;
	int height = vga.crtc.maximum_scan_line * (vga.crtc.scan_doubling + 1);
	int pos, line, column, col, addr, curr_addr = 0;
	const uint8_t decode_tbl[] = {0, 1, 2, 3, 4, 5, 6, 9, 12, 17, 22, 29, 38, 50, 66, 91, 128, 165, 190,
		206, 218, 227, 234, 239, 244, 247, 250, 251, 252, 253, 254, 255};

	for (addr = vga.crtc.start_addr, line=0; line<(vga.crtc.vert_disp_end+1); line+=height, addr+=offset(), curr_addr+=offset())
	{
		for(int yi = 0;yi < height; yi++)
		{
			uint8_t ydelta = 0, ua = 0, va = 0;
			if((line + yi) < (vga.crtc.line_compare & 0x3ff))
				curr_addr = addr;
			if((line + yi) == (vga.crtc.line_compare & 0x3ff))
				curr_addr = 0;
			for (pos=curr_addr, col=0, column=0; column<(vga.crtc.horz_disp_end+1); column++, col+=8, pos+=8)
			{
				if(pos + 0x08 > 0x80000)
					return;

				for(int xi=0;xi<8;xi+=4)
				{
					if(!m_screen->visible_area().contains(col+xi, line + yi))
						continue;
					uint8_t a = vga.memory[pos + xi], b = vga.memory[pos + xi + 1];
					uint8_t c = vga.memory[pos + xi + 2], d = vga.memory[pos + xi + 3];
					uint8_t y[4], ub, vb, trans;
					uint16_t u, v;
					if(col || xi)
					{
						y[0] = decode_tbl[a & 0x1f] + ydelta;
						ub = decode_tbl[((a >> 5) & 3) | ((b >> 3) & 0x1c)] + ua;
						vb = decode_tbl[((c >> 5) & 3) | ((d >> 3) & 0x1c)] + va;
					}
					else
					{
						y[0] = (a & 0x1f) << 3;
						ua = ub = ((a >> 2) & 0x18) | (b & 0xe0);
						va = vb = ((c >> 2) & 0x18) | (d & 0xe0);
					}
					y[1] = decode_tbl[b & 0x1f] + y[0];
					y[2] = decode_tbl[c & 0x1f] + y[1];
					y[3] = decode_tbl[d & 0x1f] + y[2];
					trans = (a >> 7) | ((c >> 6) & 2);
					u = ua;
					v = va;
					for(int i = 0; i < 4; i++)
					{
						if(i == trans)
						{
							u = (ua + ub) >> 1;
							v = (va + vb) >> 1;
						}
						else if(i == (trans + 1))
						{
							u = ub;
							v = vb;
						}
						bitmap.pix32(line + yi, col + xi + i) = IV | (uint32_t)yuv_to_rgb(y[i], u, v);
					}
					ua = ub;
					va = vb;
					ydelta = y[3];
				}
			}
		}
	}
}

void vis_vga_device::device_start()
{
	if(!m_palette->started())
		throw device_missing_dependencies();
	set_isa_device();
	m_isa->install_device(0x03b0, 0x03df, read8_delegate(FUNC(vis_vga_device::vga_r), this), write8_delegate(FUNC(vis_vga_device::vga_w), this));
	m_isa->install_memory(0x0a0000, 0x0bffff, read8_delegate(FUNC(vis_vga_device::visvgamem_r), this), write8_delegate(FUNC(vis_vga_device::visvgamem_w), this));
	svga_device::device_start();
	vga.svga_intf.seq_regcount = 0x2d;
}

void vis_vga_device::device_reset()
{
	m_extcnt = 0;
	m_extreg = 0;
	m_wina = 0;
	m_winb = 0;
	m_interlace = 0;
	m_shift256 = 0;
	m_dw = 0;
	m_8bit_640 = 0;
}

uint32_t vis_vga_device::screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
	if(!BIT(m_extreg, 7))
		return svga_device::screen_update(screen, bitmap, cliprect);

	if((m_extreg & 0xc0) == 0x80)
	{
		svga_vh_rgb15(bitmap, cliprect);
		return 0;
	}

	switch(m_extreg & 0xc7)
	{
		case 0xc0:
		case 0xc1:
			svga_vh_rgb16(bitmap, cliprect);
			break;
		case 0xc2:
			popmessage("Border encoded 8-bit mode");
			break;
		case 0xc3:
			popmessage("YUV 422 mode");
			break;
		case 0xc4:
			vga_vh_yuv8(bitmap, cliprect);
			break;
		case 0xc5:
			svga_vh_rgb24(bitmap, cliprect);
			break;
		case 0xc6:
			popmessage("DAC off");
			break;
	}
	return 0;
}

READ8_MEMBER(vis_vga_device::visvgamem_r)
{
	if(!(vga.sequencer.data[0x25] & 0x40))
		return mem_r(space, offset, mem_mask);
	u16 win = (vga.sequencer.data[0x1e] & 0x0f) == 3 ? m_wina : m_winb; // this doesn't seem quite right
	return mem_linear_r(space, (offset + (win * 64)) & 0x3ffff, mem_mask);
}

WRITE8_MEMBER(vis_vga_device::visvgamem_w)
{
	if(!(vga.sequencer.data[0x25] & 0x40))
		return mem_w(space, offset, data, mem_mask);
	return mem_linear_w(space, (offset + (m_wina * 64)) & 0x3ffff, data, mem_mask);
}

READ8_MEMBER(vis_vga_device::vga_r)
{
	switch(offset)
	{
		case 0x16:
			if(m_extcnt == 4)
			{
				m_extcnt = 0;
				return m_extreg;
			}
			m_extcnt++;
			break;
		case 0x05:
		case 0x25:
			return m_crtc_regs[vga.crtc.index];
	}
	if(offset < 0x10)
		return port_03b0_r(space, offset, mem_mask);
	else if(offset < 0x20)
		return port_03c0_r(space, offset - 0x10, mem_mask);
	else
		return port_03d0_r(space, offset - 0x20, mem_mask);
}

WRITE8_MEMBER(vis_vga_device::vga_w)
{
	switch(offset)
	{
		case 0x15:
			switch(vga.sequencer.index)
			{
				case 0x01:
					m_8bit_640 = (data & 8) ? 0 : 1;
					if(m_8bit_640 && (vga.sequencer.data[0x1f] & 0x10))
						svga.rgb8_en = 1;
					else
						svga.rgb8_en = 0;
					break;
				case 0x19:
					m_wina = (m_wina & 0xff00) | data;
					break;
				case 0x18:
					m_wina = (m_wina & 0xff) | (data << 8);
					break;
				case 0x1d:
					m_winb = (m_winb & 0xff00) | data;
					break;
				case 0x1c:
					m_winb = (m_winb & 0xff) | (data << 8);
					break;
				case 0x1f:
					if(data & 0x10)
					{
						vga.gc.shift256 = 1;
						vga.crtc.dw = 1;
						if(m_8bit_640)
							svga.rgb8_en = 1;
						else
							svga.rgb8_en = 0;
					}
					else
					{
						vga.gc.shift256 = m_shift256;
						vga.crtc.dw = m_dw;
						svga.rgb8_en = 0;
					}
					break;
			}
			break;
		case 0x16:
			if(m_extcnt == 4)
			{
				if((data & 0xc7) != 0xc7)
					m_extreg = data;
				m_extcnt = 0;
				return;
			}
			break;
		case 0x1f:
			if(vga.gc.index == 0x05)
			{
				m_shift256 = data & 0x40 ? 1 : 0;
				if(vga.sequencer.data[0x1f] & 0x10)
					data |= 0x40;
			}
			break;
		case 0x05:
		case 0x25:
			m_crtc_regs[vga.crtc.index] = data;
			switch(vga.crtc.index)
			{
				case 0x00:
					if(vga.crtc.protect_enable)
						return;
					vga.crtc.horz_total = data / (m_interlace && !(vga.sequencer.data[0x25] & 0x20) ? 2 : 1);
					recompute_params();
					return;
				case 0x01:
					if(vga.crtc.protect_enable)
						return;
					vga.crtc.horz_disp_end = data / (m_interlace && !(vga.sequencer.data[0x25] & 0x20) ? 2 : 1);
					recompute_params();
					return;
				case 0x02:
					if(vga.crtc.protect_enable)
						return;
					vga.crtc.horz_blank_start = data / (m_interlace && !(vga.sequencer.data[0x25] & 0x20) ? 2 : 1);
					return;
				case 0x03:
					if(vga.crtc.protect_enable)
						return;
					vga.crtc.horz_blank_end = (m_crtc_regs[0x05] & 0x80) >> 2;
					vga.crtc.horz_blank_end |= data & 0x1f;
					vga.crtc.horz_blank_end /= (m_interlace && !(vga.sequencer.data[0x25] & 0x20) ? 2 : 1);
					vga.crtc.disp_enable_skew = (data & 0x60) >> 5;
					vga.crtc.evra = (data & 0x80) >> 7;
					return;
				case 0x04:
					if(vga.crtc.protect_enable)
						return;
					vga.crtc.horz_retrace_start = data / (m_interlace && !(vga.sequencer.data[0x25] & 0x20) ? 2 : 1);
					return;
				case 0x05:
					if(vga.crtc.protect_enable)
						return;
					vga.crtc.horz_blank_end = m_crtc_regs[0x05] & 0x1f;
					vga.crtc.horz_blank_end |= ((data & 0x80) >> 2);
					vga.crtc.horz_blank_end /= (m_interlace && !(vga.sequencer.data[0x25] & 0x20) ? 2 : 1);
					vga.crtc.horz_retrace_skew = ((data & 0x60) >> 5);
					vga.crtc.horz_retrace_end = data & 0x1f;
					return;
				case 0x06:
					if(vga.crtc.protect_enable)
						return;
					vga.crtc.vert_total = ((m_crtc_regs[0x07] & 0x20) << (9-6)) | ((m_crtc_regs[0x07] & 0x01) << (8-0));
					vga.crtc.vert_total |= data;
					vga.crtc.vert_total *= (m_interlace ? 2 : 1);
					recompute_params();
					return;
				case 0x07: // Overflow Register
					vga.crtc.line_compare       &= ~0x100;
					vga.crtc.line_compare       |= ((data & 0x10) << (8-4));
					if(vga.crtc.protect_enable)
						return;
					vga.crtc.vert_total         = m_crtc_regs[0x05];
					vga.crtc.vert_retrace_start = m_crtc_regs[0x10];
					vga.crtc.vert_disp_end      = m_crtc_regs[0x12];
					vga.crtc.vert_blank_start   = m_crtc_regs[0x15] | ((m_crtc_regs[0x09] & 0x20) << (9-5));
					vga.crtc.vert_retrace_start |= ((data & 0x80) << (9-7));
					vga.crtc.vert_disp_end      |= ((data & 0x40) << (9-6));
					vga.crtc.vert_total         |= ((data & 0x20) << (9-5));
					vga.crtc.vert_blank_start   |= ((data & 0x08) << (8-3));
					vga.crtc.vert_retrace_start |= ((data & 0x04) << (8-2));
					vga.crtc.vert_disp_end      |= ((data & 0x02) << (8-1));
					vga.crtc.vert_total         |= ((data & 0x01) << (8-0));
					vga.crtc.vert_total *= (m_interlace ? 2 : 1);
					vga.crtc.vert_blank_start *= (m_interlace ? 2 : 1);
					vga.crtc.vert_retrace_start *= (m_interlace ? 2 : 1);
					vga.crtc.vert_disp_end *= (m_interlace ? 2 : 1);
					recompute_params();
					return;
				case 0x09: // Maximum Scan Line Register
					vga.crtc.line_compare      &= ~0x200;
					vga.crtc.vert_blank_start  = m_crtc_regs[0x15] | ((m_crtc_regs[0x07] & 0x02) << (8-3));
					vga.crtc.scan_doubling      = ((data & 0x80) >> 7);
					vga.crtc.line_compare      |= ((data & 0x40) << (9-6));
					vga.crtc.vert_blank_start  |= ((data & 0x20) << (9-5));
					vga.crtc.maximum_scan_line  = (data & 0x1f) + 1;
					vga.crtc.vert_blank_start *= (m_interlace ? 2 : 1);
					return;
				case 0x10:
					vga.crtc.vert_retrace_start = ((m_crtc_regs[0x07] & 0x80) << (9-7)) | ((m_crtc_regs[0x07] & 0x40) << (8-2));
					vga.crtc.vert_retrace_start |= data;
					vga.crtc.vert_retrace_start *= (m_interlace ? 2 : 1);
					return;
				case 0x12:
					vga.crtc.vert_disp_end = ((m_crtc_regs[0x07] & 0x40) << (9-6)) | ((m_crtc_regs[0x07] & 0x02) << (8-1));
					vga.crtc.vert_disp_end |= data;
					vga.crtc.vert_disp_end *= (m_interlace ? 2 : 1);
					recompute_params();
					return;
				case 0x15:
					vga.crtc.vert_blank_start = ((m_crtc_regs[0x09] & 0x20) << (9-5)) | ((m_crtc_regs[0x07] & 0x02) << (8-3));
					vga.crtc.vert_blank_start |= data;
					vga.crtc.vert_blank_start *= (m_interlace ? 2 : 1);
					return;
				case 0x16:
					vga.crtc.vert_blank_end = (data & 0x7f) * (m_interlace ? 2 : 1);
					return;
				case 0x14:
					m_dw = data & 0x40 ? 1 : 0;
					if(vga.sequencer.data[0x1f] & 0x10)
						data |= 0x40;
					break;
				case 0x30:
					if(data && !m_interlace)
					{
						if(!(vga.sequencer.data[0x25] & 0x20))
						{
							vga.crtc.horz_total /= 2;
							vga.crtc.horz_disp_end /= 2;
							vga.crtc.horz_blank_end /= 2;
							vga.crtc.horz_retrace_start /= 2;
							vga.crtc.horz_retrace_end /= 2;
						}
						vga.crtc.vert_total *= 2;
						vga.crtc.vert_retrace_start *= 2;
						vga.crtc.vert_disp_end *= 2;
						vga.crtc.vert_blank_start *= 2;
						vga.crtc.vert_blank_end *= 2;
						recompute_params();
					}
					else if(!data && m_interlace)
					{
						if(!(vga.sequencer.data[0x25] & 0x20))
						{
							vga.crtc.horz_total *= 2;
							vga.crtc.horz_disp_end *= 2;
							vga.crtc.horz_blank_end *= 2;
							vga.crtc.horz_retrace_start *= 2;
							vga.crtc.horz_retrace_end *= 2;
						}
						vga.crtc.vert_total /= 2;
						vga.crtc.vert_retrace_start /= 2;
						vga.crtc.vert_disp_end /= 2;
						vga.crtc.vert_blank_start /= 2;
						vga.crtc.vert_blank_end /= 2;
						recompute_params();
					}
					m_interlace = data;
					return;
			}
			break;
	}
	if(offset < 0x10)
		port_03b0_w(space, offset, data, mem_mask);
	else if(offset < 0x20)
		port_03c0_w(space, offset - 0x10, data, mem_mask);
	else
		port_03d0_w(space, offset - 0x20, data, mem_mask);
}

class vis_state : public driver_device
{
public:
	vis_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_pic1(*this, "mb:pic8259_master"),
		m_pic2(*this, "mb:pic8259_slave"),
		m_pad(*this, "PAD")
		{ }
	required_device<cpu_device> m_maincpu;
	required_device<pic8259_device> m_pic1;
	required_device<pic8259_device> m_pic2;
	required_ioport m_pad;

	DECLARE_READ8_MEMBER(sysctl_r);
	DECLARE_WRITE8_MEMBER(sysctl_w);
	DECLARE_READ8_MEMBER(unk_r);
	DECLARE_WRITE8_MEMBER(unk_w);
	DECLARE_READ8_MEMBER(unk2_r);
	DECLARE_READ8_MEMBER(unk3_r);
	DECLARE_READ16_MEMBER(pad_r);
	DECLARE_WRITE16_MEMBER(pad_w);
	DECLARE_READ8_MEMBER(unk1_r);
	DECLARE_WRITE8_MEMBER(unk1_w);
	DECLARE_INPUT_CHANGED_MEMBER(update);
protected:
	void machine_reset() override;
private:
	uint8_t m_sysctl;
	uint8_t m_unkidx;
	uint8_t m_unk[16];
	uint8_t m_unk1[4];
	uint16_t m_padctl, m_padstat;
};

void vis_state::machine_reset()
{
	m_sysctl = 0;
	m_padctl = 0;
}

INPUT_CHANGED_MEMBER(vis_state::update)
{
	m_pic1->ir3_w(ASSERT_LINE);
	m_padstat = 0x80;
}

//chipset registers?
READ8_MEMBER(vis_state::unk_r)
{
	if(offset)
		return m_unk[m_unkidx];
	return 0;
}

WRITE8_MEMBER(vis_state::unk_w)
{
	if(offset)
		m_unk[m_unkidx] = data;
	else
		m_unkidx = data & 0xf;
}

READ8_MEMBER(vis_state::unk2_r)
{
	return 0x40;
}

//memory card reader?
READ8_MEMBER(vis_state::unk3_r)
{
	return 0x00;
}

READ16_MEMBER(vis_state::pad_r)
{
	uint16_t ret = 0;
	switch(offset)
	{
		case 0:
			ret = m_pad->read();
			if(m_padctl != 0x18)
				ret |= 0x400;
			else
				m_padctl = 0;
			m_padstat = 0;
			m_pic1->ir3_w(CLEAR_LINE);
			break;
		case 1:
			ret = m_padstat;
	}
	return ret;
}

WRITE16_MEMBER(vis_state::pad_w)
{
	switch(offset)
	{
		case 1:
			m_padctl = data;
			break;
	}
}

READ8_MEMBER(vis_state::unk1_r)
{
	if(offset == 2)
		return 0xde;
	return 0;
}

WRITE8_MEMBER(vis_state::unk1_w)
{
	switch(offset)
	{
		case 1:
			if(data == 0x10)
				m_pic2->ir1_w(CLEAR_LINE);
			else if(data == 0x16)
				m_pic2->ir1_w(ASSERT_LINE);
	}
	m_unk1[offset] = data;
}

READ8_MEMBER(vis_state::sysctl_r)
{
	return m_sysctl;
}

WRITE8_MEMBER(vis_state::sysctl_w)
{
	if(BIT(data, 0) && !BIT(m_sysctl, 0))
		m_maincpu->set_input_line(INPUT_LINE_RESET, PULSE_LINE);
	//m_maincpu->set_input_line(INPUT_LINE_A20, BIT(data, 1) ? CLEAR_LINE : ASSERT_LINE);
	m_sysctl = data;
}

static ADDRESS_MAP_START( at16_map, AS_PROGRAM, 16, vis_state )
	ADDRESS_MAP_UNMAP_HIGH
	AM_RANGE(0x000000, 0x09ffff) AM_RAM
	AM_RANGE(0x0d8000, 0x0fffff) AM_ROM AM_REGION("bios", 0xd8000)
	AM_RANGE(0x100000, 0x15ffff) AM_RAM
	AM_RANGE(0x300000, 0x3fffff) AM_ROM AM_REGION("bios", 0)
	AM_RANGE(0xff0000, 0xffffff) AM_ROM AM_REGION("bios", 0xf0000)
ADDRESS_MAP_END

static ADDRESS_MAP_START( at16_io, AS_IO, 16, vis_state )
	ADDRESS_MAP_UNMAP_HIGH
	AM_RANGE(0x0000, 0x001f) AM_DEVREADWRITE8("mb:dma8237_1", am9517a_device, read, write, 0xffff)
	AM_RANGE(0x0026, 0x0027) AM_READWRITE8(unk_r, unk_w, 0xffff)
	AM_RANGE(0x0020, 0x003f) AM_DEVREADWRITE8("mb:pic8259_master", pic8259_device, read, write, 0xffff)
	AM_RANGE(0x0040, 0x005f) AM_DEVREADWRITE8("mb:pit8254", pit8254_device, read, write, 0xffff)
	AM_RANGE(0x0060, 0x0065) AM_DEVREADWRITE8("kbdc", kbdc8042_device, data_r, data_w, 0xffff)
	AM_RANGE(0x006a, 0x006b) AM_READ8(unk2_r, 0x00ff)
	AM_RANGE(0x0092, 0x0093) AM_READWRITE8(sysctl_r, sysctl_w, 0x00ff)
	AM_RANGE(0x0080, 0x009f) AM_DEVREADWRITE8("mb", at_mb_device, page8_r, page8_w, 0xffff)
	AM_RANGE(0x00a0, 0x00bf) AM_DEVREADWRITE8("mb:pic8259_slave", pic8259_device, read, write, 0xffff)
	AM_RANGE(0x00c0, 0x00df) AM_DEVREADWRITE8("mb:dma8237_2", am9517a_device, read, write, 0x00ff)
	AM_RANGE(0x00e0, 0x00e1) AM_NOP
	AM_RANGE(0x023c, 0x023f) AM_READWRITE8(unk1_r, unk1_w, 0xffff)
	AM_RANGE(0x0268, 0x026f) AM_READWRITE(pad_r, pad_w)
	AM_RANGE(0x031a, 0x031b) AM_READ8(unk3_r, 0x00ff)
ADDRESS_MAP_END

static SLOT_INTERFACE_START(vis_cards)
	SLOT_INTERFACE("visaudio", VIS_AUDIO)
	SLOT_INTERFACE("visvga", VIS_VGA)
SLOT_INTERFACE_END

// TODO: other buttons
static INPUT_PORTS_START(vis)
	PORT_START("PAD")
	PORT_BIT( 0x0001, IP_ACTIVE_HIGH, IPT_JOYSTICK_DOWN ) PORT_CHANGED_MEMBER(DEVICE_SELF, vis_state, update, 0)
	PORT_BIT( 0x0002, IP_ACTIVE_HIGH, IPT_UNKNOWN ) PORT_CHANGED_MEMBER(DEVICE_SELF, vis_state, update, 0)
	PORT_BIT( 0x0004, IP_ACTIVE_HIGH, IPT_BUTTON1 ) PORT_NAME("A") PORT_CHANGED_MEMBER(DEVICE_SELF, vis_state, update, 0)
	PORT_BIT( 0x0008, IP_ACTIVE_HIGH, IPT_UNKNOWN ) PORT_CHANGED_MEMBER(DEVICE_SELF, vis_state, update, 0)
	PORT_BIT( 0x0010, IP_ACTIVE_HIGH, IPT_UNKNOWN ) PORT_CHANGED_MEMBER(DEVICE_SELF, vis_state, update, 0)
	PORT_BIT( 0x0020, IP_ACTIVE_HIGH, IPT_BUTTON2 ) PORT_NAME("B") PORT_CHANGED_MEMBER(DEVICE_SELF, vis_state, update, 0)
	PORT_BIT( 0x0040, IP_ACTIVE_HIGH, IPT_UNKNOWN ) PORT_CHANGED_MEMBER(DEVICE_SELF, vis_state, update, 0)
	PORT_BIT( 0x0080, IP_ACTIVE_HIGH, IPT_JOYSTICK_LEFT ) PORT_CHANGED_MEMBER(DEVICE_SELF, vis_state, update, 0)
	PORT_BIT( 0x0100, IP_ACTIVE_HIGH, IPT_JOYSTICK_UP ) PORT_CHANGED_MEMBER(DEVICE_SELF, vis_state, update, 0)
	PORT_BIT( 0x0200, IP_ACTIVE_HIGH, IPT_JOYSTICK_RIGHT ) PORT_CHANGED_MEMBER(DEVICE_SELF, vis_state, update, 0)
	PORT_BIT( 0x0400, IP_ACTIVE_HIGH, IPT_UNKNOWN ) PORT_CHANGED_MEMBER(DEVICE_SELF, vis_state, update, 0)
	PORT_BIT( 0x0800, IP_ACTIVE_HIGH, IPT_UNKNOWN ) PORT_CHANGED_MEMBER(DEVICE_SELF, vis_state, update, 0)
	PORT_BIT( 0x1000, IP_ACTIVE_HIGH, IPT_UNKNOWN ) PORT_CHANGED_MEMBER(DEVICE_SELF, vis_state, update, 0)
	PORT_BIT( 0x2000, IP_ACTIVE_HIGH, IPT_UNKNOWN ) PORT_CHANGED_MEMBER(DEVICE_SELF, vis_state, update, 0)
	PORT_BIT( 0x4000, IP_ACTIVE_HIGH, IPT_UNKNOWN ) PORT_CHANGED_MEMBER(DEVICE_SELF, vis_state, update, 0)
	PORT_BIT( 0x8000, IP_ACTIVE_HIGH, IPT_UNKNOWN ) PORT_CHANGED_MEMBER(DEVICE_SELF, vis_state, update, 0)
INPUT_PORTS_END

static MACHINE_CONFIG_START( vis )
	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", I80286, XTAL_12MHz )
	MCFG_CPU_PROGRAM_MAP(at16_map)
	MCFG_CPU_IO_MAP(at16_io)
	MCFG_CPU_IRQ_ACKNOWLEDGE_DEVICE("mb:pic8259_master", pic8259_device, inta_cb)
	MCFG_80286_SHUTDOWN(DEVWRITELINE("mb", at_mb_device, shutdown))

	MCFG_DEVICE_ADD("mb", AT_MB, 0)
	// this doesn't have a real keyboard controller
	MCFG_DEVICE_REMOVE("mb:keybc")
	MCFG_DEVICE_REMOVE("mb:pc_kbdc")

	MCFG_DEVICE_ADD("kbdc", KBDC8042, 0)
	MCFG_KBDC8042_KEYBOARD_TYPE(KBDC8042_AT386)
	MCFG_KBDC8042_SYSTEM_RESET_CB(INPUTLINE("maincpu", INPUT_LINE_RESET))
	MCFG_KBDC8042_GATE_A20_CB(INPUTLINE("maincpu", INPUT_LINE_A20))
	MCFG_KBDC8042_INPUT_BUFFER_FULL_CB(DEVWRITELINE("mb:pic8259_master", pic8259_device, ir1_w))

	MCFG_ISA16_SLOT_ADD("mb:isabus", "mcd", pc_isa16_cards, "mcd", true)
	MCFG_ISA16_SLOT_ADD("mb:isabus", "visaudio", vis_cards, "visaudio", true)
	MCFG_ISA16_SLOT_ADD("mb:isabus", "visvga", vis_cards, "visvga", true)
MACHINE_CONFIG_END

ROM_START(vis)
	ROM_REGION(0x100000,"bios", 0)
	ROM_LOAD( "p513bk0b.bin", 0x00000, 0x80000, CRC(364e3f74) SHA1(04260ef1e65e482c9c49d25ace40e22487d6aab9))
	ROM_LOAD( "p513bk1b.bin", 0x80000, 0x80000, CRC(e18239c4) SHA1(a0262109e10a07a11eca43371be9978fff060bc5))
ROM_END

COMP ( 1992, vis,  0, 0, vis, vis, vis_state, 0, "Tandy/Memorex", "Video Information System MD-2500", MACHINE_IMPERFECT_GRAPHICS | MACHINE_IMPERFECT_SOUND )
