// license:LGPL-2.1+
// copyright-holders:Angelo Salese, Tomasz Slanina, David Haywood, Luca Elia
/********************************************************************************************

    Gunpey (c) 2000 Banpresto

    TODO:
    - compression scheme used by the Axell video chip, game is playable but several gfxs are
      still broken.

=============================================================================================
ASM code study:
0x84718 main code
0x8472c call 0x81f34 reading of dip-switches
    0x5e62:
0x84731 call 0x81f5c move dip-switches settings to work RAM
0x84736 call 0x80ae4 writes to i/os 0x7fc0-0x7fff:
    0x7fc0: 0x00
    0x7fc1: 0x00
    0x7fc2: 0x40
    0x7fc3: 0x03
    0x7fc4: 0x72
    0x7fc5: 0x72
    0x7fc6: 0x90
    0x7fc7: 0x01
    0x7fc8: 0x55 irq mask?
    (note: skips irq ack)
    0x7fca: 0x00
    0x7fcb: 0x00
    0x7fcc: 0x00
    0x7fcd: 0x00
    0x7fce: 0x00
    0x7fcf: 0x07
    0x7fd0: 0x00
    0x7fd1: 0x00
    0x7fd2: 0x00
    0x7fd3: 0x8c
    0x7fd4-0x7fde: 0x00
    (skips 0x7fdf)
    0x7fe0: 0x40
    0x7fe1: 0x02
    0x7fe2: 0x00
    0x7fe3: 0x00
    0x7fe4: 0x60
    0x7fe5: 0x07
    0x7fe6: 0x88
    0x7fe7: 0x07
    0x7fe8: 0x9f
    0x7fe9: 0x08
    0x7fea: 0x77
    0x7feb: 0x08
    (here the code writes 16-bit words, read from 0x500e ...)
    0x7fec: 0x00
    0x7fed: 0x84
    (read from 0x5012)
    0x7fee: 0x00
    0x7fef: 0x88
    (returns to byte filling)
    0x7ff0: 0x84
    0x7ff1: 0x00
    0x7ff2: 0x00
    0x7ff3: 0x01
    0x7ff4: 0x00
    0x7ff5: 0x00
    0x7ff6: 0xc7
    0x7ff7: 0x56
    0x7ff8: 0x3f
    0x7ff9: 0x48
    0x7ffa: 0x9a
    0x7ffb: 0x22
    0x7ffc: 0x0e
    0x7ffd: 0x43
    0x7ffe: 0xf0
    0x7fff: 0x15
    Then, it fills the following work RAMs (with 0 except where noted):
    0x5e36, 0x5e38, 0x5e44, 0x5e46 (4), 0x5e48, 0x5e4a, 0x5e40, 0x5e42, 0x5b68 (0x9c8), 0x5b6a (0x9c8), 0x59c4 (1)
0x8473b: call 0x81415
    reads [0x500e] -> [0xf0bc]
    reads [0x5010] -> [0xf0be]
    (does it twice, shrug -.-")
    reads 0xf0bc / 0xf0be
    AW = 0xb3c9, CW = 0x10
    0x81447: call 0xb3aa0
        moves RAM from 0xb3c90-0xb3c9f to RAM 0x400-0x40f
    writes 0x800 to [0xf0b8] writes 0 to [0xf0ba]
    loops while writing to [0xf0b8] [0xf0ba] for 0x200 times, filling a table at [0x800][0x27f0] with a 1
    writes 0x800 to [0xf0b8] writes 0 to [0xf0ba] again
    reads the [0xf0b8][0xf0ba]
    0x81484: call 0xb3aa0
        moves RAM from 0xb3ca0-0xb3caf to RAM 0x800-0x80f
    writes 0x2800 to [0xf0b8] writes 0 to [0xf0ba]
    loops while writing to [0xf0b8] [0xf0ba] for 0x200 times, filling a table at [0x2800][0x47f0] with a 1
    0x81484: call 0xb3aa0
        moves RAM from 0xb3ca0-0xb3caf to RAM 0x2800-0x280f
0x84740: call 0x821cd
    fills 0x5c88-0x5c9a with this pattern (byte writes):
    [+0x00] 0x00
    [+0x02] 0x00
    [+0x08] 0x00
    [+0x04] 0x00
    [+0x0c] 0x00
    [+0x0e] 0x0c
    [+0x06] 0x00
    [+0x10] 0x00
    [+0x12] 0x0c
    does the same with 0x5c89-0x5c9b
0x84745: call 0x82026
    checks if 0x5e62 is 0 (bit 6 of 0x5c80, i/o 0x7f41)
    ...
0x8474a: call 0xa7f53 sound init
...



=============================================================================================

Gunpey
Banpresto, 2000

The hardware looks very Raizing/8ing -ish, especially the ROM stickers and PCB silk-screening
which are identical to those on Brave Blade and Battle Bakraid ;-)

PCB Layout
----------

VG-10
|-------------------------------------|
|        M6295  ROM5                  |
|        YMZ280B-F      ROM4   ROM3   |
|  YAC516      16.93MHz               |
|                       61256  ROM1   |
|                       61256  ROM2   |
|J                                    |
|A             PAL       XILINX       |
|M                       XC95108      |
|M             57.2424MHz             |
|A                            V30     |
|                                     |
|              |-------|              |
|              |AXELL  |              |
|       DSW1   |AG-1   |              |
|       DSW2   |AX51101|  T2316162    |
|              |-------|  T2316162    |
|-------------------------------------|

Notes:
      V30 clock: 14.3106MHz (= 57.2424 / 4)
      YMZ280B clock: 16.93MHz
      OKI M6295 clock: 2.11625MHz (= 16.93 / 8), sample rate = clock / 165
      VSync: 60Hz
      HSync: 15.79kHz

      ROMs:
           GP_ROM1.021 \
           GP_ROM2.022 / 27C040, Main program

           GP_ROM3.025 \
           GP_ROM4.525 / SOP44 32M MASK, Graphics

           GP_ROM5.622   SOP44 32M MASK, OKI samples


AX51101 gfx chip:

Axell Corporation is a Japanese company that specialises in Sound and Graphics (Amusement Graphics) LSI.
The AG-1 is a sprite system controller meant for the amusement industry, such as pachi-slot machines,
it has reached end-of-life in about 2005.
These are the specifications of the Axell AG-1 AX51102A, it should be very similar to the AX51101.
(excuse the strange grammar, it is a JP->EN translation)

Drawing technique:               Sprite system
Buffer drawing method:           Double frame buffer
Configuration of the character:  Configured from 2 or more cells, a cell can be set in units of each dot a horizontal, vertical from 1 to 256 dots
Maximum character size:          4096 x 4096 dot
Display the number of sprite:    Up to 127 sheets (register 2KB)
Maximum drawing speed:           Dot sec / 2500-35000000 highest
Color depth:                     32,768 colors (5 bits for each RGB)
Color scheme:                    Cell character unit can be specified in 256 colors of palettes, and 16 colors
Scaling:                         256 times the resolution of 1/64 to 4 cell character unit
Semi-transparent processing:     Gradation in the unit cell or 32 character
Intensity modulation:            Gradation in the unit cell or 32 character
Other Features:                  Rotation, DMA, BitBLT, Built-in flip
Display resolution:              100 to 600 dots horizontal, 120 to 800 dots vertical
Virtual screen size:             Up to 4096 x 4096 dot
CGRAM space:                     4M-bit minimum, 32M-bit maximum
Operating frequency:             Up to 76MHz
Release:                         November 1999

********************************************************************************************/

#include "emu.h"
#include "cpu/nec/nec.h"
#include "sound/okim6295.h"
#include "sound/ymz280b.h"
#include "screen.h"
#include "speaker.h"


class gunpey_state : public driver_device
{
public:
	gunpey_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_oki(*this, "oki"),
		m_wram(*this, "wram"),
		m_palette(*this, "palette")
		{ }

	required_device<cpu_device> m_maincpu;
	required_device<okim6295_device> m_oki;
	required_shared_ptr<uint16_t> m_wram;
	required_device<palette_device> m_palette;

	std::unique_ptr<uint16_t[]> m_blit_buffer;
	uint16_t m_blit_ram[0x10];
	uint8_t m_irq_cause, m_irq_mask;
	DECLARE_WRITE8_MEMBER(status_w);
	DECLARE_READ8_MEMBER(status_r);
	DECLARE_READ8_MEMBER(inputs_r);
	DECLARE_WRITE8_MEMBER(blitter_w);
	DECLARE_WRITE8_MEMBER(blitter_upper_w);
	DECLARE_WRITE8_MEMBER(blitter_upper2_w);
	DECLARE_WRITE8_MEMBER(output_w);
	DECLARE_WRITE16_MEMBER(vram_bank_w);
	DECLARE_WRITE16_MEMBER(vregs_addr_w);
	DECLARE_DRIVER_INIT(gunpey);
	virtual void video_start() override;
	uint32_t screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	TIMER_DEVICE_CALLBACK_MEMBER(scanline);
	TIMER_CALLBACK_MEMBER(blitter_end);
	void irq_check(uint8_t irq_type);
	uint8_t draw_gfx(bitmap_ind16 &bitmap,const rectangle &cliprect,int count,uint8_t scene_gradient);
	uint16_t m_vram_bank;
	uint16_t m_vreg_addr;

	uint8_t* m_blit_rom;
	uint8_t* m_blit_rom2;

	uint8_t* m_vram;

	emu_timer *m_blitter_end_timer;

	// work variables for the decompression
	int m_srcx;
	int m_srcxbase;
	int m_scrxcount;
	int m_srcy;
	int m_srcycount;
	uint8_t m_sourcewide;
	int m_ysize;
	int m_xsize;
	int m_dstx;
	int m_dsty;
	int m_dstxbase;
	int m_dstxcount;
	int m_dstycount;
	bool m_out_of_data;

	int m_latched_bits_left;
	uint8_t m_latched_byte;
	int m_zero_bit_count;

	void get_stream_next_byte(void);
	int get_stream_bit(void);
	uint32_t get_stream_bits(int bits);

	int write_dest_byte(uint8_t usedata);
	//uint16_t main_m_vram[0x800][0x800];
};


void gunpey_state::video_start()
{
	m_blit_buffer = std::make_unique<uint16_t[]>(512*512);

	m_blitter_end_timer = machine().scheduler().timer_alloc(timer_expired_delegate(FUNC(gunpey_state::blitter_end), this));
}

uint8_t gunpey_state::draw_gfx(bitmap_ind16 &bitmap,const rectangle &cliprect,int count,uint8_t scene_gradient)
{
	int x,y;
	int bpp_sel;
	int color;

	// there doesn't seem to be a specific bit to mark compressed sprites (we currently have a hack to look at the first byte of the data)
	// do they get decompressed at blit time instead? of are there other registers we need to look at

	// +0                    +1                    +2                    +3                    +4                    +5                    +6                    +7
	// cccc cccc e--b b--- | xxxx x--- u--t tttt | yyyy yy-- --XX XXXX | nnnn nnnn ---Y YYYY | mmmm mmmm -MMM -NNN | hhhh hhhh wwww wwww | ---- ---- oooo oooo | pppp pppp ---- ---- |

	// c = color palette
	// e = END marker
	// b = bpp select
	// x = LSB x source
	// X = MSB x source
	// y = LSB y source
	// Y = MSB y source
	// n = LSB X DRAW position
	// N = MSB X DRAW position
	// m = LSB Y DRAW position
	// M = MSB Y DRAW position
	// h = height
	// w = width
	// u = unknown, set on text, maybe 'solid' ?
	// o = zoom width
	// p = zoom height
	// t = transparency / alpha related? (0x10 on player cursor, 0xf when swapping, other values at other times..)
	const int debug = 0;

	if(!(m_wram[count+0] & 1))
	{
		x = (m_wram[count+3] >> 8) | ((m_wram[count+4] & 0x03) << 8);
		y = (m_wram[count+4] >> 8) | ((m_wram[count+4] & 0x30) << 4);
		uint8_t zoomheight = (m_wram[count+5] >> 8);
		uint8_t zoomwidth = (m_wram[count+5] & 0xff);
		bpp_sel = (m_wram[count+0] & 0x18);
		color = (m_wram[count+0] >> 8);

		x-=0x160;
		y-=0x188;

		uint8_t sourcewidth  = (m_wram[count+6] & 0xff);
		uint8_t sourceheight = (m_wram[count+7] >> 8);
		int xsource = ((m_wram[count+2] & 0x003f) << 5) | ((m_wram[count+1] & 0xf800) >> 11);
		int ysource = ((m_wram[count+3] & 0x001f) << 6) | ((m_wram[count+2] & 0xfc00) >> 10);

		int alpha =  m_wram[count+1] & 0x1f;


		uint16_t unused;
		if (debug) printf("sprite %04x %04x %04x %04x %04x %04x %04x %04x\n", m_wram[count+0], m_wram[count+1], m_wram[count+2], m_wram[count+3], m_wram[count+4], m_wram[count+5], m_wram[count+6], m_wram[count+7]);

		unused = m_wram[count+0]&~0xff98; if (unused) printf("unused bits set in word 0 - %04x\n", unused);
		unused = m_wram[count+1]&~0xf89f; if (unused) printf("unused bits set in word 1 - %04x\n", unused);
		unused = m_wram[count+2]&~0xfc3f; if (unused) printf("unused bits set in word 2 - %04x\n", unused);
		unused = m_wram[count+3]&~0xff1f; if (unused) printf("unused bits set in word 3 - %04x\n", unused);
		unused = m_wram[count+4]&~0xff77; if (unused) printf("unused bits set in word 4 - %04x\n", unused);
		unused = m_wram[count+5]&~0xffff; if (unused) printf("unused bits set in word 5 - %04x\n", unused);
		unused = m_wram[count+6]&~0x00ff; if (unused) printf("unused bits set in word 6 - %04x\n", unused);
		unused = m_wram[count+7]&~0xff00; if (unused) printf("unused bits set in word 7 - %04x\n", unused);

		if ((zoomwidth != sourcewidth) || (zoomheight != sourceheight))
		{
			//printf("zoomed widths %02x %02x heights %02x %02x\n", sourcewidth, zoomwidth, sourceheight, zoomheight);
		}

		if(bpp_sel == 0x00)  // 4bpp
		{
			for(int yi=0;yi<sourceheight;yi++)
			{
				for(int xi=0;xi<sourcewidth/2;xi++)
				{
					uint8_t data = m_vram[((((ysource+yi)&0x7ff)*0x800) + ((xsource+xi)&0x7ff))];
					uint8_t pix;
					uint32_t col_offs;
					uint16_t color_data;

					pix = (data & 0x0f);
					col_offs = ((pix + color*0x10) & 0xff) << 1;
					col_offs+= ((pix + color*0x10) >> 8)*0x800;
					color_data = (m_vram[col_offs])|(m_vram[col_offs+1]<<8);

					if(!(color_data & 0x8000))
					{
						if(scene_gradient & 0x40)
						{
							int r,g,b;

							r = (color_data & 0x7c00) >> 10;
							g = (color_data & 0x03e0) >> 5;
							b = (color_data & 0x001f) >> 0;
							r-= (scene_gradient & 0x1f);
							g-= (scene_gradient & 0x1f);
							b-= (scene_gradient & 0x1f);
							if(r < 0) r = 0;
							if(g < 0) g = 0;
							if(b < 0) b = 0;

							color_data = (color_data & 0x8000) | (r << 10) | (g << 5) | (b << 0);
						}

						if(cliprect.contains(x+(xi*2), y+yi))
						{
							if (alpha==0x00) // a value of 0x00 is solid
							{
								bitmap.pix16(y+yi, x+(xi*2)) = color_data & 0x7fff;
							}
							else
							{
								uint16_t basecolor = bitmap.pix16(y+yi, x+(xi*2));
								int base_r = ((basecolor >> 10)&0x1f)*alpha;
								int base_g = ((basecolor >> 5)&0x1f)*alpha;
								int base_b = ((basecolor >> 0)&0x1f)*alpha;
								int r = ((color_data & 0x7c00) >> 10)*(0x1f-alpha);
								int g = ((color_data & 0x03e0) >> 5)*(0x1f-alpha);
								int b = ((color_data & 0x001f) >> 0)*(0x1f-alpha);
								r = (base_r+r)/0x1f;
								g = (base_g+g)/0x1f;
								b = (base_b+b)/0x1f;
								color_data = (color_data & 0x8000) | (r << 10) | (g << 5) | (b << 0);
								bitmap.pix16(y+yi, x+(xi*2)) = color_data & 0x7fff;
							}
						}
					}

					pix = (data & 0xf0)>>4;
					col_offs = ((pix + color*0x10) & 0xff) << 1;
					col_offs+= ((pix + color*0x10) >> 8)*0x800;
					color_data = (m_vram[col_offs])|(m_vram[col_offs+1]<<8);

					if(!(color_data & 0x8000))
					{
						if(scene_gradient & 0x40)
						{
							int r,g,b;

							r = (color_data & 0x7c00) >> 10;
							g = (color_data & 0x03e0) >> 5;
							b = (color_data & 0x001f) >> 0;
							r-= (scene_gradient & 0x1f);
							g-= (scene_gradient & 0x1f);
							b-= (scene_gradient & 0x1f);
							if(r < 0) r = 0;
							if(g < 0) g = 0;
							if(b < 0) b = 0;

							color_data = (color_data & 0x8000) | (r << 10) | (g << 5) | (b << 0);
						}

						if(cliprect.contains(x+1+(xi*2),y+yi))
						{
							if (alpha==0x00) // a value of 0x00 is solid
							{
								bitmap.pix16(y+yi, x+1+(xi*2)) = color_data & 0x7fff;
							}
							else
							{
								uint16_t basecolor = bitmap.pix16(y+yi, x+1+(xi*2));
								int base_r = ((basecolor >> 10)&0x1f)*alpha;
								int base_g = ((basecolor >> 5)&0x1f)*alpha;
								int base_b = ((basecolor >> 0)&0x1f)*alpha;
								int r = ((color_data & 0x7c00) >> 10)*(0x1f-alpha);
								int g = ((color_data & 0x03e0) >> 5)*(0x1f-alpha);
								int b = ((color_data & 0x001f) >> 0)*(0x1f-alpha);
								r = (base_r+r)/0x1f;
								g = (base_g+g)/0x1f;
								b = (base_b+b)/0x1f;
								color_data = (color_data & 0x8000) | (r << 10) | (g << 5) | (b << 0);
								bitmap.pix16(y+yi, x+1+(xi*2)) = color_data & 0x7fff;
							}

						}

					}
				}
			}
		}
		else if(bpp_sel == 0x08) // 6bpp
		{
			printf("6bpp\n");
			#if 0
			for(int yi=0;yi<sourceheight;yi++)
			{
				for(int xi=0;xi<sourcewidth;xi++)
				{
					uint8_t data = m_vram[((((ysource+yi)&0x7ff)*0x800) + ((xsource+xi)&0x7ff))];
					uint8_t pix;
					uint32_t col_offs;
					uint16_t color_data;

					pix = (data & 0x3f);
					if(cliprect.contains(x+xi, y+yi))
						bitmap.pix16(y+yi, x+xi) = pix + color*64;
				}
			}
			#endif
		}
		else if(bpp_sel == 0x10) // 8bpp
		{
			for(int yi=0;yi<sourceheight;yi++)
			{
				for(int xi=0;xi<sourcewidth;xi++)
				{
					uint8_t data = m_vram[((((ysource+yi)&0x7ff)*0x800) + ((xsource+xi)&0x7ff))];
					uint8_t pix;
					uint32_t col_offs;
					uint16_t color_data;

					pix = (data & 0xff);
					col_offs = ((pix + color*0x100) & 0xff) << 1;
					col_offs+= ((pix + color*0x100) >> 8)*0x800;
					color_data = (m_vram[col_offs])|(m_vram[col_offs+1]<<8);

					if(!(color_data & 0x8000))
					{
						if(scene_gradient & 0x40)
						{
							int r,g,b;

							r = (color_data & 0x7c00) >> 10;
							g = (color_data & 0x03e0) >> 5;
							b = (color_data & 0x001f) >> 0;
							r-= (scene_gradient & 0x1f);
							g-= (scene_gradient & 0x1f);
							b-= (scene_gradient & 0x1f);
							if(r < 0) r = 0;
							if(g < 0) g = 0;
							if(b < 0) b = 0;

							color_data = (color_data & 0x8000) | (r << 10) | (g << 5) | (b << 0);
						}



						if(cliprect.contains(x+xi,y+yi))
						{
							if (alpha==0x00) // a value of 0x00 is solid
							{
								bitmap.pix16(y+yi, x+xi) = color_data & 0x7fff;
							}
							else
							{
								uint16_t basecolor = bitmap.pix16(y+yi, x+xi);
								int base_r = ((basecolor >> 10)&0x1f)*alpha;
								int base_g = ((basecolor >> 5)&0x1f)*alpha;
								int base_b = ((basecolor >> 0)&0x1f)*alpha;
								int r = ((color_data & 0x7c00) >> 10)*(0x1f-alpha);
								int g = ((color_data & 0x03e0) >> 5)*(0x1f-alpha);
								int b = ((color_data & 0x001f) >> 0)*(0x1f-alpha);
								r = (base_r+r)/0x1f;
								g = (base_g+g)/0x1f;
								b = (base_b+b)/0x1f;
								color_data = (color_data & 0x8000) | (r << 10) | (g << 5) | (b << 0);
								bitmap.pix16(y+yi, x+xi) = color_data & 0x7fff;
							}

						}

					}
				}
			}
		}
		else if(bpp_sel == 0x18) // RGB32k
		{
			printf("32k\n");
			// ...
		}
	}

	return m_wram[count+0] & 0x80;
}

uint32_t gunpey_state::screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	//uint16_t *blit_buffer = m_blit_buffer;
	uint16_t vram_bank = m_vram_bank & 0x7fff;
	uint16_t vreg_addr = m_vreg_addr & 0x7fff;
	uint8_t end_mark;
	int count;
	int scene_index;

	bitmap.fill(m_palette->pen(0), cliprect); //black pen

	if((!(m_vreg_addr & 0x8000)) || (!(m_vram_bank & 0x8000)))
		return 0;

	for(scene_index = vreg_addr/2;scene_index<(vreg_addr+0x400)/2;scene_index+=0x10/2)
	{
		uint16_t start_offs;
		uint16_t end_offs;
		uint8_t scene_end_mark;
		uint8_t scene_enabled;
		uint8_t scene_gradient;

		start_offs = (vram_bank+(m_wram[scene_index+5] << 8))/2;
		end_offs = (vram_bank+(m_wram[scene_index+5] << 8)+0x1000)/2; //safety check
		scene_end_mark = m_wram[scene_index+0] & 0x80;
		scene_enabled = m_wram[scene_index+0] & 0x01;
		scene_gradient = m_wram[scene_index+1] & 0xff;

//      printf("%08x: %08x %08x %08x %08x | %08x %08x %08x %08x\n",scene_index,m_wram[scene_index+0],m_wram[scene_index+1],m_wram[scene_index+2],m_wram[scene_index+3],
//                                                  m_wram[scene_index+4],m_wram[scene_index+5],m_wram[scene_index+6],m_wram[scene_index+7]);

		if(scene_enabled)
		{
			for(count = start_offs;count<end_offs;count+=0x10/2)
			{
				end_mark = draw_gfx(bitmap,cliprect,count,scene_gradient);

				if(end_mark == 0x80)
					break;
			}
		}

		if(scene_end_mark == 0x80)
			break;
	}

	return 0;
}

void gunpey_state::irq_check(uint8_t irq_type)
{
	m_irq_cause |= irq_type;

	if(m_irq_cause & m_irq_mask)
		m_maincpu->set_input_line_and_vector(0, HOLD_LINE, 0x200/4);
	else
		m_maincpu->set_input_line_and_vector(0, CLEAR_LINE, 0x200/4);
}

WRITE8_MEMBER(gunpey_state::status_w)
{
	if(offset == 1)
	{
		m_irq_cause &= ~data;
		irq_check(0);
	}

	if(offset == 0)
	{
		m_irq_mask = data;
		irq_check(0);
	}
}

READ8_MEMBER(gunpey_state::status_r)
{
	if(offset == 1)
		return m_irq_cause;

	return m_irq_mask;
}

READ8_MEMBER(gunpey_state::inputs_r)
{
	switch(offset+0x7f40)
	{
		case 0x7f40: return ioport("DSW1")->read();
		case 0x7f41: return ioport("DSW2")->read();
		case 0x7f42: return ioport("P1")->read();
		case 0x7f43: return ioport("P2")->read();
		case 0x7f44: return ioport("SYSTEM")->read();
	}

	return 0xff;
}

TIMER_CALLBACK_MEMBER(gunpey_state::blitter_end)
{
	irq_check(4);
}

void gunpey_state::get_stream_next_byte(void)
{
	// check if we need to move on to the next row of the source bitmap
	// to get the data requested

	if (m_scrxcount==m_sourcewide)
	{
		m_scrxcount = 0;
		m_srcx = m_srcxbase;
		m_srcy++; m_srcycount++;
	}

	m_latched_byte = m_blit_rom[(((m_srcy)&0x7ff)*0x800)+((m_srcx)&0x7ff)];
	if (!m_out_of_data) m_blit_rom2[(((m_srcy)&0x7ff)*0x800)+((m_srcx)&0x7ff)] = 0x77; // debug

	m_latched_bits_left = 8;

	// increase counters
	m_srcx++; m_scrxcount++;
}

int gunpey_state::get_stream_bit(void)
{
	if (m_latched_bits_left==0)
	{
		get_stream_next_byte();
	}

	m_latched_bits_left--;

	int bit = (m_latched_byte >> (7-m_latched_bits_left))&1;

	if (bit==0) m_zero_bit_count++;
	else m_zero_bit_count=0;

	return bit;
}

uint32_t gunpey_state::get_stream_bits(int bits)
{
	uint32_t output = 0;
	for (int i=0;i<bits;i++)
	{
		output = output<<1;
		output |= get_stream_bit();
	}

	return output;
}

int gunpey_state::write_dest_byte(uint8_t usedata)
{
	// write the byte we and to destination and increase our counters
	m_vram[(((m_dsty)&0x7ff)*0x800)+((m_dstx)&0x7ff)] = usedata;

	// increase destination counter and check if we've filled our destination rectangle
	m_dstx++; m_dstxcount++;
	if (m_dstxcount==m_xsize)
	{
		m_dstxcount = 0;
		m_dstx = m_dstxbase;
		m_dsty++; m_dstycount++;
		if (m_dstycount==m_ysize)
		{
			return -1;
		}
	}

	return 1;
}

/*
the very first transfer, which might be 100% blank data looks like this.. are the 239 repeats significant (our screen height is 240..  239 is 16x15 - 1)
 and this same data block occurs 6 times in the transfer.  Maximum draw size is 256x256.. (but I'm not sure this gets drawn)
 the value that repeats (110010110) is quite common in other blocks of more noisy data too.

this is sprite 959 in test mode, as you can see below the sprite size is 27 ef (== 0x50 0xf0)


02 08 00 8c|6c 06 73 06|80 03 00 00|27 00 ef 00
data:
 000101010 001011011 1111111

 (this is 239 (0xef) repeats)
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101

 101001 1001 0000

 (this is 239 repeats)
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101


 101001 1001 0000

 (this is 239 repeats)
 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101

 101001 1001 0000

 (this is 239 repeats)
 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101

 101001 1001 0000

 (this is 239 repeats)
 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101

 101001 1001 0000

 (this is 239 repeats)

 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101

 (then what happens?! is this part of the same sprite or just running past the end of the compressed source block?)
 (this is also a repeat of 239 values, but this time 11 bit ones)

 101001110000000

 10101000100 10101000100 10101000100 10101000100 10101000100 10101000100
 10101000100 10101000100 10101000100 10101000100 10101000100 10101000100
 10101000100 10101000100 10101000100 10101000100 10101000100 10101000100
 10101000100 10101000100 10101000100 10101000100 10101000100 10101000100
 10101000100 10101000100 10101000100 10101000100 10101000100 10101000100
 10101000100 10101000100 10101000100 10101000100 10101000100 10101000100
 10101000100 10101000100 10101000100 10101000100 10101000100 10101000100
 10101000100 10101000100 10101000100 10101000100 10101000100 10101000100
 10101000100 10101000100 10101000100 10101000100 10101000100 10101000100
 10101000100 10101000100 10101000100 10101000100 10101000100 10101000100
 10101000100 10101000100 10101000100 10101000100 10101000100 10101000100
 10101000100 10101000100 10101000100 10101000100 10101000100 10101000100
 10101000100 10101000100 10101000100 10101000100 10101000100 10101000100
 10101000100 10101000100 10101000100 10101000100 10101000100 10101000100
 10101000100 10101000100 10101000100 10101000100 10101000100 10101000100
 10101000100 10101000100 10101000100 10101000100 10101000100 10101000100
 10101000100 10101000100 10101000100 10101000100 10101000100 10101000100
 10101000100 10101000100 10101000100 10101000100 10101000100 10101000100
 10101000100 10101000100 10101000100 10101000100 10101000100 10101000100
 10101000100 10101000100 10101000100 10101000100 10101000100 10101000100
 10101000100 10101000100 10101000100 10101000100 10101000100 10101000100
 10101000100 10101000100 10101000100 10101000100 10101000100 10101000100
 10101000100 10101000100 10101000100 10101000100 10101000100 10101000100
 10101000100 10101000100 10101000100 10101000100 10101000100 10101000100
 10101000100 10101000100 10101000100 10101000100 10101000100 10101000100
 10101000100 10101000100 10101000100 10101000100 10101000100 10101000100
 10101000100 10101000100 10101000100 10101000100 10101000100 10101000100
 10101000100 10101000100 10101000100 10101000100 10101000100 10101000100
 10101000100 10101000100 10101000100 10101000100 10101000100 10101000100
 10101000100 10101000100 10101000100 10101000100 10101000100 10101000100
 10101000100 10101000100 10101000100 10101000100 10101000100 10101000100
 10101000100 10101000100 10101000100 10101000100 10101000100 10101000100
 10101000100 10101000100 10101000100 10101000100 10101000100 10101000100
 10101000100 10101000100 10101000100 10101000100 10101000100 10101000100
 10101000100 10101000100 10101000100 10101000100 10101000100 10101000100
 10101000100 10101000100 10101000100 10101000100 10101000100 10101000100
 10101000100 10101000100 10101000100 10101000100 10101000100 10101000100
 10101000100 10101000100 10101000100 10101000100 10101000100 10101000100
 10101000100 10101000100 10101000100 10101000100 10101000100 10101000100
 10101000100 10101000100 10101000100 10101000100 10101000100



 219 in char test

02 08 00 8c|8a 03 78 00|90 01 e0 02|17 00 27 00
data: 000101 010 001011 010 000011

(40 (0x28) 'repeat' values, last one is different)
 1101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 10110010
 1101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 10110010
 1101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 10110010
 1101100101 101100101 101100101 101100101 101100101 101100101

 10100110010000

 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101

 101001 1001 0000 << this string again before repeating data..
 (39 (0x27) repeat)
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101

 101001 1001 0000 << this string again before repeating data..
 (39 (0x27) 'repeat' values, last one is different)
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101

 000000 000000


 378 - might be interesting, is text, not pasting here.


 703 - another (mostly) blank?

 02 08 00 8c|1a 00 3f 03|90 01 e0 02|23 00 67 00
data: 000101 001100 10000

(0x67 repeats...)
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101

 101001 1001 0000   << this.. again

 (0x67 repeats...)
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101

 101001 1001 0000

 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101

 1010011001 0000

 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101

 10100111111111000000101000110000110010100010100111110100010110100100101101101010101100000101111100110010101110001010100011001000110010101011
 101100101
 10101000111001100101010111010011111001001001000011001010001010011111010001011010010010 110110100 110010000

 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101

 10100110010000

 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101

 101001 1001 0000

 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 101100101 101100101 101100101 101100101 101100101 101100101 101100101
 000000 000000



  thoughts:

  where there are repeating values of 101100101 they tend to be in blocks the same (or a very similar) size to the HEIGHT (y-size) of the object with a different (often
  common '101001100 10000') value between them.. which can throw out the stream alignment.. this separator CAN vary in both content and length..

 not all data is 9-bit, some blocks are made of 11-bit repeating values, see first example.



02 08 00 8c|90 03 78 00|90 01 e0 02|17 00 27 00
data:
00010101 00010110 10000 0111
101100101  101100101  101100101  101100101  101100101  101100101  101100101  101100101
101100101  101100101  101100101  101100101  101100101  101100101  101100101  101100101
101100101  101100101  101100101  101100101  101100101  101100101  101100101  101100101
101100101  101100101  101100101  101100101  101100101  101100101  101100101  101100101
101100101  101100101  101100101  101100101  101100101  101100101  101100101
101001100 10000
101100101  101100101  101100101  101100101  101100101  101100101  101100101  101100101
101100101  101100101  101100101  101100101  101100101  101100101  101100101  101100101
101100101  101100101  101100101  101100101  101100101  101100101  101100101  101100101
101100101  101100101  101100101  101100101  101100101  101100101  101100101  101100101
101100101  101100101  101100101  101100101  101100101  101100101  101100101
101001100 10000
101100101  101100101  101100101  101100101  101100101  101100101  101100101  101100101
101100101  101100101  101100101  101100101  101100101  101100101  101100101  101100101
101100101  101100101  101100101  101100101  101100101  101100101  101100101  101100101
101100101  101100101  101100101  101100101  101100101  101100101  101100101  101100101
101100101  101100101  101100101  101100101  101100101  101100101  101100101
101001100 10000
101100101  101100101  101100101  101100101  101100101  101100101  101100101  101100101
101100101  101100101  101100101  101100101  101100101  101100101  101100101  101100101
101100101  101100101  101100101  101100101  101100101  101100101  101100101  101100101
101100101  101100101  101100101  101100101  101100101  101100101  101100101  101100101
101100101  101100101  101100101  101100101  101100101  101100101  101100101
00000000000000


02 08 00 8c|18 04 f0 00|90 01 e0 02|17 00 27 00
data:
00010101 00010110 10111 1011
(then same as above)



02 08 00 8c|a2 03 d0 04|90 01 e0 02|03 00 07 00
data:

000101010 010101110 101101001 101001100 010001011 111110100 101110000 110001000
101101101 101010001 101011100 110111110 011111100 101101011 010110111 011000110
000011110 101100110 000011011 101100011 100100001 111111000 111111001 101001011
111101101 011100001 100001011 000110011 101010110 010001011 101001100 110111001
011100011 101100000 001101000 111001011 001111110 011110011 111110101 110110110
111111111 011010000 010101101 110010101 111100001 110101000 110111100 001100011
001101111 001001000 000000000
ooutcount was 51


 */


//#define SHOW_COMPRESSED_DATA_DEBUG



WRITE8_MEMBER(gunpey_state::blitter_w)
{
	uint16_t *blit_ram = m_blit_ram;

	//printf("gunpey_blitter_w offset %01x data %02x\n", offset,data);

	blit_ram[offset] = data;

	if(offset == 0 && data == 2) // blitter trigger, 0->1 transition
	{
		m_srcx = blit_ram[0x04]+(blit_ram[0x05]<<8);
		m_srcy = blit_ram[0x06]+(blit_ram[0x07]<<8);
		m_dstx = blit_ram[0x08]+(blit_ram[0x09]<<8);
		m_dsty = blit_ram[0x0a]+(blit_ram[0x0b]<<8);
		m_xsize = blit_ram[0x0c]+1;
		m_ysize = blit_ram[0x0e]+1;
		int rle = blit_ram[0x01];

		m_dstx<<=1;
		m_xsize<<=1;

		if(rle)
		{
			if(rle == 8)
			{
				// compressed stream format:
				//
				// byte 0 = source width   (data is often stored in fairly narrow columns)

#ifdef SHOW_COMPRESSED_DATA_DEBUG
				printf("%02x %02x %02x %02x|%02x %02x %02x %02x|%02x %02x %02x %02x|%02x %02x %02x %02x\n"
					,blit_ram[0],blit_ram[1],blit_ram[2],blit_ram[3]
					,blit_ram[4],blit_ram[5],blit_ram[6],blit_ram[7]
					,blit_ram[8],blit_ram[9],blit_ram[0xa],blit_ram[0xb]
					,blit_ram[0xc],blit_ram[0xd],blit_ram[0xe],blit_ram[0xf]);
				int count = 0;
				printf("data: ");
				int linespacer = 0;
#endif


				m_dstxbase = m_dstx;
				m_dstxcount = 0;
				m_dstycount = 0;
				m_srcxbase = m_srcx;
				m_scrxcount = 0;
				m_srcycount = 0;

				m_sourcewide = m_blit_rom[(((m_srcy)&0x7ff)*0x800)+((m_srcx)&0x7ff)]+1;
				m_srcx++;m_scrxcount++; // we don't want to decode the width as part of the data stream..
				m_latched_bits_left = 0;
				m_zero_bit_count = 0;

				m_out_of_data = false;

				for (;;)
				{
					int test = get_stream_bits(2);
					int data;
					int getbits = 1;
					// don't think this is right.. just keeps some streams in alignment, see 959 in char test for example
					if (test==0x0)
					{
						getbits = 4;
					}
					else if (test==0x1)
					{
						getbits = 1;
					}
					else if (test==0x2)
					{
						getbits = 2;
					}
					else if (test==0x3)
					{
						getbits = 7;
					}
					data = get_stream_bits(getbits);

					// hack, really I imagine there is exactly enough compressed data to fill the dest bitmap area when decompressed, but to stop us
					// overrunning into reading other data we terminate on a 0000, which doesn't seem likely to be compressed data.
					if (m_zero_bit_count>=16)
						m_out_of_data = true;



					uint8_t usedata = 0xff;
					if (!m_out_of_data)
					{
						#ifdef SHOW_COMPRESSED_DATA_DEBUG
						//if (count<512)
						{
							{
								if (test==0x0)
								{
									printf("00");
								}
								else if (test==0x1)
								{
									printf("01");
								}
								else if (test==0x2)
								{
									printf("10");
								}
								else if (test==0x3)
								{
									printf("11");
								}

								for (int z=0;z<getbits;z++)
								{
									printf("%d", (data>>((getbits-1)-z))&1);
								}

								linespacer++;
								if ((linespacer%16) == 0) printf("\n");

								printf(" ");

							}
							count++;
						}
						#endif

						usedata = data;
					}
					else
						usedata = 0x44;

					if ((write_dest_byte(usedata))==-1)
						break;

				}

#ifdef SHOW_COMPRESSED_DATA_DEBUG
				printf("\n");
#endif

			}
			else
				printf("unknown RLE mode %02x\n",rle);
		}
		else
		{
			m_dstxbase = m_dstx;
			m_dstxcount = 0;
			m_dstycount = 0;
			m_srcxbase = m_srcx;
			m_scrxcount = 0;
			m_srcycount = 0;

			for (;;)
			{
				uint8_t usedata = m_blit_rom[(((m_srcy)&0x7ff)*0x800)+((m_srcx)&0x7ff)];
				m_blit_rom2[(((m_srcy)&0x7ff)*0x800)+((m_srcx)&0x7ff)] = 0x44; // debug
				m_srcx++; m_scrxcount++;
				if (m_scrxcount==m_xsize)
				{
					m_scrxcount = 0;
					m_srcx = m_srcxbase;
					m_srcy++; m_srcycount++;
				}

				if ((write_dest_byte(usedata))==-1)
					break;
			}
		}

		m_blitter_end_timer->adjust(m_maincpu->cycles_to_attotime(m_xsize*m_ysize));


/*

*/
	}
}

WRITE8_MEMBER(gunpey_state::blitter_upper_w)
{
	//printf("gunpey_blitter_upper_w %02x %02x\n", offset, data);

}

WRITE8_MEMBER(gunpey_state::blitter_upper2_w)
{
	//printf("gunpey_blitter_upper2_w %02x %02x\n", offset, data);

}


WRITE8_MEMBER(gunpey_state::output_w)
{
	//bit 0 is coin counter
//  popmessage("%02x",data);

	m_oki->set_rom_bank((data & 0x70) >> 4);
}

WRITE16_MEMBER(gunpey_state::vram_bank_w)
{
	COMBINE_DATA(&m_vram_bank);
}

WRITE16_MEMBER(gunpey_state::vregs_addr_w)
{
	COMBINE_DATA(&m_vreg_addr);
}

/***************************************************************************************/

static ADDRESS_MAP_START( mem_map, AS_PROGRAM, 16, gunpey_state )
	AM_RANGE(0x00000, 0x0ffff) AM_RAM AM_SHARE("wram")
//  AM_RANGE(0x50000, 0x500ff) AM_RAM
//  AM_RANGE(0x50100, 0x502ff) AM_NOP
	AM_RANGE(0x80000, 0xfffff) AM_ROM
ADDRESS_MAP_END

static ADDRESS_MAP_START( io_map, AS_IO, 16, gunpey_state )
	AM_RANGE(0x7f40, 0x7f45) AM_READ8(inputs_r,0xffff)

	AM_RANGE(0x7f48, 0x7f49) AM_WRITE8(output_w,0x00ff)
	AM_RANGE(0x7f80, 0x7f81) AM_DEVREADWRITE8("ymz", ymz280b_device, read, write, 0xffff)

	AM_RANGE(0x7f88, 0x7f89) AM_DEVREADWRITE8("oki", okim6295_device, read, write, 0x00ff)

	AM_RANGE(0x7fc8, 0x7fc9) AM_READWRITE8(status_r, status_w, 0xffff )
	AM_RANGE(0x7fd0, 0x7fdf) AM_WRITE8(blitter_w, 0xffff )
	AM_RANGE(0x7fe0, 0x7fe5) AM_WRITE8(blitter_upper_w, 0xffff )
	AM_RANGE(0x7ff0, 0x7ff5) AM_WRITE8(blitter_upper2_w, 0xffff )

	//AM_RANGE(0x7FF0, 0x7FF1) AM_RAM
	AM_RANGE(0x7fec, 0x7fed) AM_WRITE(vregs_addr_w)
	AM_RANGE(0x7fee, 0x7fef) AM_WRITE(vram_bank_w)

ADDRESS_MAP_END


/***************************************************************************************/

static INPUT_PORTS_START( gunpey )
	PORT_START("DSW1")  // IN0 - 7f40
	PORT_DIPNAME( 0x03, 0x00, DEF_STR( Difficulty ) )   PORT_DIPLOCATION("SW1:1,2")
	PORT_DIPSETTING(    0x01, DEF_STR( Easy ) )
	PORT_DIPSETTING(    0x00, DEF_STR( Normal ) )
	PORT_DIPSETTING(    0x02, DEF_STR( Hard ) )
	PORT_DIPSETTING(    0x03, DEF_STR( Hardest ) )
	PORT_DIPNAME( 0x0c, 0x00, "Difficulty (vs. mode)" ) PORT_DIPLOCATION("SW1:3,4")
	PORT_DIPSETTING(    0x04, DEF_STR( Easy ) )
	PORT_DIPSETTING(    0x00, DEF_STR( Normal ) )
	PORT_DIPSETTING(    0x08, DEF_STR( Hard ) )
	PORT_DIPSETTING(    0x0c, DEF_STR( Hardest ) )
	PORT_DIPNAME( 0x30, 0x00, "Matches (vs. mode)?" )   PORT_DIPLOCATION("SW1:5,6")
	PORT_DIPSETTING(    0x00, "1" )
	PORT_DIPSETTING(    0x10, "2" )
	PORT_DIPSETTING(    0x20, "3" )
	PORT_DIPSETTING(    0x30, "5" )
	PORT_DIPNAME( 0x40, 0x00, DEF_STR( Demo_Sounds ) )  PORT_DIPLOCATION("SW1:7")
	PORT_DIPSETTING(    0x40, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x80, 0x00, DEF_STR( Flip_Screen ) )  PORT_DIPLOCATION("SW1:8")
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x80, DEF_STR( On ) )

	PORT_START("DSW2")  // IN1 - 7f41
	PORT_DIPNAME( 0x07, 0x00, DEF_STR( Coin_A ) )   PORT_DIPLOCATION("SW2:1,2,3")
	PORT_DIPSETTING(    0x07, DEF_STR( 4C_1C ) )
	PORT_DIPSETTING(    0x05, DEF_STR( 3C_1C ) )
	PORT_DIPSETTING(    0x03, DEF_STR( 2C_1C ) )
	PORT_DIPSETTING(    0x06, DEF_STR( 3C_2C ) )
	PORT_DIPSETTING(    0x00, DEF_STR( 1C_1C ) )
	PORT_DIPSETTING(    0x04, DEF_STR( 2C_3C ) )
	PORT_DIPSETTING(    0x01, DEF_STR( 1C_2C ) )
	PORT_DIPSETTING(    0x02, DEF_STR( 1C_3C ) )
	PORT_DIPNAME( 0x38, 0x00, DEF_STR( Coin_B ) )   PORT_DIPLOCATION("SW2:4,5,6")
	PORT_DIPSETTING(    0x38, DEF_STR( 4C_1C ) )
	PORT_DIPSETTING(    0x28, DEF_STR( 3C_1C ) )
	PORT_DIPSETTING(    0x18, DEF_STR( 2C_1C ) )
	PORT_DIPSETTING(    0x30, DEF_STR( 3C_2C ) )
	PORT_DIPSETTING(    0x00, DEF_STR( 1C_1C ) )
	PORT_DIPSETTING(    0x20, DEF_STR( 2C_3C ) )
	PORT_DIPSETTING(    0x08, DEF_STR( 1C_2C ) )
	PORT_DIPSETTING(    0x10, DEF_STR( 1C_3C ) )
	PORT_DIPNAME( 0x40, 0x00, DEF_STR( Free_Play ) )    PORT_DIPLOCATION("SW2:7")
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x40, DEF_STR( On ) )
	PORT_DIPNAME( 0x80, 0x00, DEF_STR( Allow_Continue ) )   PORT_DIPLOCATION("SW2:8")
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x80, DEF_STR( On ) )

	PORT_START("P1")    // IN2 - 7f42
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_JOYSTICK_UP    ) PORT_PLAYER(1)
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_JOYSTICK_DOWN  ) PORT_PLAYER(1)
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_JOYSTICK_LEFT  ) PORT_PLAYER(1)
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_JOYSTICK_RIGHT ) PORT_PLAYER(1)
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_BUTTON1        ) PORT_PLAYER(1)
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_BUTTON2        ) PORT_PLAYER(1)
	PORT_BIT( 0x40, IP_ACTIVE_HIGH, IPT_BUTTON3        ) PORT_PLAYER(1)
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_UNKNOWN )

	PORT_START("P2")    // IN3 - 7f43
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_JOYSTICK_UP    ) PORT_PLAYER(2)
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_JOYSTICK_DOWN  ) PORT_PLAYER(2)
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_JOYSTICK_LEFT  ) PORT_PLAYER(2)
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_JOYSTICK_RIGHT ) PORT_PLAYER(2)
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_BUTTON1        ) PORT_PLAYER(2)
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_BUTTON2        ) PORT_PLAYER(2)
	PORT_BIT( 0x40, IP_ACTIVE_HIGH, IPT_BUTTON3        ) PORT_PLAYER(2)
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_UNKNOWN )

	PORT_START("SYSTEM")    // IN4 - 7f44
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_SERVICE1 )
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_UNKNOWN )
	PORT_SERVICE( 0x04, IP_ACTIVE_HIGH )   // TEST!!
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_COIN1 )
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_COIN2 )
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_START1 )
	PORT_BIT( 0x40, IP_ACTIVE_HIGH, IPT_START2 )
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_UNKNOWN )
INPUT_PORTS_END

/***************************************************************************************/

/*:
0x01
0x04 blitter ready
0x10 vblank too? (otherwise you'll get various hangs/inputs stop to work)
0x40 almost certainly vblank (reads inputs)
0x80
*/
TIMER_DEVICE_CALLBACK_MEMBER(gunpey_state::scanline)
{
	int scanline = param;

	if(scanline == 240)
	{
		//printf("frame\n");
		irq_check(0x50);
	}
}





// this isn't a real decode as such, but the graphic data is all stored in pages 2048 bytes wide at varying BPP levels, some (BG data) compressed with what is likely a lossy scheme
// palette data is in here too, the blocks at the bottom right of all this?
static GFXLAYOUT_RAW( gunpey, 2048, 1, 2048*8, 2048*8 )

static GFXDECODE_START( gunpey )
	GFXDECODE_ENTRY( "blit_data", 0, gunpey,     0x0000, 0x1 )
	GFXDECODE_ENTRY( "blit_data2", 0, gunpey,     0x0000, 0x1 )
	GFXDECODE_ENTRY( "vram", 0, gunpey,     0x0000, 0x1 )
GFXDECODE_END



/***************************************************************************************/
static MACHINE_CONFIG_START( gunpey )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", V30, 57242400 / 4)
	MCFG_CPU_PROGRAM_MAP(mem_map)
	MCFG_CPU_IO_MAP(io_map)
	MCFG_TIMER_DRIVER_ADD_SCANLINE("scantimer", gunpey_state, scanline, "screen", 0, 1)

	/* video hardware */
	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_RAW_PARAMS(57242400/8, 442, 0, 320, 264, 0, 240) /* just to get ~60 Hz */
	MCFG_SCREEN_UPDATE_DRIVER(gunpey_state, screen_update)
	MCFG_SCREEN_PALETTE("palette")

	MCFG_PALETTE_ADD_RRRRRGGGGGBBBBB("palette")
	MCFG_GFXDECODE_ADD("gfxdecode", "palette", gunpey)

	MCFG_SPEAKER_STANDARD_STEREO("lspeaker","rspeaker")

	MCFG_OKIM6295_ADD("oki", XTAL_16_9344MHz / 8, PIN7_LOW)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "lspeaker", 0.25)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "rspeaker", 0.25)

	MCFG_SOUND_ADD("ymz", YMZ280B, XTAL_16_9344MHz)
	MCFG_SOUND_ROUTE(0, "lspeaker", 0.25)
	MCFG_SOUND_ROUTE(1, "rspeaker", 0.25)
MACHINE_CONFIG_END


/***************************************************************************************/

ROM_START( gunpey )
	ROM_REGION( 0x100000, "maincpu", 0 ) /* V30 code */
	ROM_LOAD16_BYTE( "gp_rom1.021",  0x00000, 0x80000, CRC(07a589a7) SHA1(06c4140ffd5f74b3d3ddfc424f43fcd08d903490) )
	ROM_LOAD16_BYTE( "gp_rom2.022",  0x00001, 0x80000, CRC(f66bc4cf) SHA1(54931d878d228c535b9e2bf22a0a3e41756f0fe5) )

	ROM_REGION( 0x400000, "blit_data", 0 )
	ROM_LOAD( "gp_rom3.025",  0x00000, 0x400000,  CRC(f2d1f9f0) SHA1(0d20301fd33892074508b9d127456eae80cc3a1c) )
	ROM_REGION( 0x400000, "blit_data2", 0 ) // debug test
	ROM_COPY( "blit_data", 0x00000, 0x00000, 0x400000 )


	ROM_REGION( 0x400000, "vram", ROMREGION_ERASEFF )

	ROM_REGION( 0x400000, "ymz", 0 )
	ROM_LOAD( "gp_rom4.525",  0x000000, 0x400000, CRC(78dd1521) SHA1(91d2046c60e3db348f29f776def02e3ef889f2c1) ) // 11xxxxxxxxxxxxxxxxxxxx = 0xFF

	ROM_REGION( 0x400000, "oki", 0 )
	ROM_LOAD( "gp_rom5.622",  0x000000, 0x400000,  CRC(f79903e0) SHA1(4fd50b4138e64a48ec1504eb8cd172a229e0e965)) // 1xxxxxxxxxxxxxxxxxxxxx = 0xFF
ROM_END



DRIVER_INIT_MEMBER(gunpey_state,gunpey)
{
	m_blit_rom = memregion("blit_data")->base();
	m_blit_rom2 = memregion("blit_data2")->base();

	m_vram = memregion("vram")->base();
	// ...
}

GAME( 2000, gunpey, 0, gunpey, gunpey, gunpey_state, gunpey,    ROT0, "Banpresto", "Gunpey (Japan)", MACHINE_NOT_WORKING | MACHINE_IMPERFECT_GRAPHICS )
