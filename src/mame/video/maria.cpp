// license:BSD-3-Clause
// copyright-holders:Dan Boris, Fabio Priuli, Mike Saarna, Robert Tuccitto
/***************************************************************************

Atari MARIA video emulation.

Dan Boris

History:

2002/05/12 kubecj
        -Added cases for 0x01-160A, 0x05-160B as stated by docs.

2002/05/13 kubecj
        -Fixed 320C mode - displayed 2 pixels instead of one!
        -Noticed that Jinks uses 0x02-320D mode, implemented the
         mode - completely unsure if good!
        -Implemented some Maria CTRL variables.

2002/05/14 kubecj
        -Vblank dma stop fix.

2003/06/23 ericball
        -Kangaroo mode & 320 mode & other stuff.

2013/05/08 huygens
        -Rewrite to emulate line ram buffers (mostly fixes Kung-Fu Master).
        -Started DMA cycle stealing implementation.

2014/03/24 Mike Saarna
        -Fixed DMA regarding startup, shutdown and cycle stealing.

2014/05/06 Mike Saarna
        -Added interrupts to DMA cycle eating.
        -Updates to LL, OL, and spin accounting for HALT behavior.

2014/08/26 Fabio Priuli
        -Converted to device.

2014/08/29 Mike Saarna
        -Timeout rendering added.

2014/09/03 Mike Saarna/Robert Tuccitto
        -Reorganized DMA penalties to support new rendering timeout code.

2014/10/05 Mike Saarna/Robert Tuccitto
        -Last Line DMA value corrected to 6. GCC and Atari docs both show
         a difference between Other Line and Last Line as +6 at the lowest
         part of the range.
        -Blank scanlines are drawn when DMA is off, like real hardware.
        -If MARIA hits the DMA limit, the CPU doesn't run until the next 
         scanline.

2014/12/01 Mike Saarna/Robert Tuccitto
        -Implemented "colorburst kill" bit of the MARIA CTRL register.

***************************************************************************/

#include "emu.h"
#include "maria.h"
#include "screen.h"


#define TRIGGER_HSYNC   64717

//#define READ_MEM(x) space.read_byte(x)
#define READ_MEM(x) (((x)<0x20) ? 0:space.read_byte(x)) //TODO: track down spurious writes to TIA

DEFINE_DEVICE_TYPE(ATARI_MARIA, atari_maria_device, "atari_maria", "Atari MARIA")



atari_maria_device::atari_maria_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, ATARI_MARIA, tag, owner, clock)
{
}


void atari_maria_device::device_start()
{
	m_cpu = machine().device<cpu_device>(m_cpu_tag);
	m_screen = machine().first_screen();
	m_screen->register_screen_bitmap(m_bitmap);

	save_item(NAME(m_maria_palette));
	save_item(NAME(m_line_ram));
	save_item(NAME(m_active_buffer));
	save_item(NAME(m_write_mode));
	save_item(NAME(m_dll));
	save_item(NAME(m_dl));
	save_item(NAME(m_holey));
	save_item(NAME(m_offset));
	save_item(NAME(m_vblank));
	save_item(NAME(m_dmaon));
	save_item(NAME(m_dpp));
	save_item(NAME(m_wsync));
	save_item(NAME(m_color_kill));
	save_item(NAME(m_cwidth));
	save_item(NAME(m_bcntl));
	save_item(NAME(m_kangaroo));
	save_item(NAME(m_rm));
	save_item(NAME(m_nmi));
	save_item(NAME(m_charbase));
}

void atari_maria_device::device_reset()
{
	for (auto & elem : m_maria_palette)
		elem = 0;

	for (auto & elem : m_line_ram)
		for (int j = 0; j < 160; j++)
			elem[j] = 0;

	m_active_buffer = 0;

	m_write_mode = 0;
	m_dmaon = 0;
	m_vblank = 0x80;
	m_dll = 0;
	m_wsync = 0;

	m_color_kill = 0;
	m_cwidth = 0;
	m_bcntl = 0;
	m_kangaroo = 0;
	m_rm = 0;

	m_dl = 0;
	m_holey = 0;
	m_offset = 0;
	m_dpp = 0;
	m_nmi = 0;
	m_charbase = 0;
}

/***************************************************************************

  Stop the video hardware emulation.

***************************************************************************/

int atari_maria_device::is_holey(unsigned int addr)
{
	if (((m_holey & 0x02) && ((addr & 0x9000) == 0x9000)) || ( (m_holey & 0x01) && ((addr & 0x8800) == 0x8800)))
		return 1;
	else
		return 0;
}

int atari_maria_device::write_line_ram(int addr, uint8_t offset, int pal)
{
	address_space& space = m_cpu->space(AS_PROGRAM);
	int c;

	int data = READ_MEM(addr);
	pal <<= 2;

	if (m_write_mode)
	{
		c = (pal & 0x10) | (data & 0x0c) | (data >> 6); // P2 D3 D2 D7 D6
		if (((c & 3) || m_kangaroo) && (offset < 160))
			m_line_ram[m_active_buffer][offset] = c;
		offset++;

		c = (pal & 0x10) | ((data & 0x03) << 2) | ((data & 0x30) >> 4); // P2 D1 D0 D5 D4
		if (((c & 3) || m_kangaroo) && (offset < 160))
			m_line_ram[m_active_buffer][offset] = c;
	}
	else
	{
		for (int i = 0; i < 4; i++, offset++)
		{
			c = pal | ((data >> (6 - 2 * i)) & 0x03);
			if (((c & 3) || m_kangaroo) && (offset < 160))
				m_line_ram[m_active_buffer][offset] = c;
		}
	}
	return m_write_mode ? 2 : 4;
}


void atari_maria_device::draw_scanline()
{
	address_space& space = m_cpu->space(AS_PROGRAM);
	uint16_t graph_adr, data_addr;
	int width, pal, ind;
	uint8_t hpos;
	uint16_t dl;
	int c, cells;
	int maria_cycles;
	int dma_hole_known;

	for (int i = 0; i < 160; i++) // working buffer cleared
		m_line_ram[m_active_buffer][i] = 0;

	cells = 0;

	if(m_dmaon)
	{
		// All lines in a zone have the same initial DMA startup time. We'll adjust
		// cycles for the special last zone line later, as those penalties happen after
		// MARIA is done rendering, or after its hit the maximum rendering time.
		maria_cycles = 16;


		/* Process this DLL entry */
		dl = m_dl;

		/* DMA */
		#define DMALIMIT 430

		while (((READ_MEM(dl + 1) & 0x5f) != 0) && (maria_cycles<DMALIMIT))
		{
			/* Extended header */
			if (!(READ_MEM(dl + 1) & 0x1f))
			{
				graph_adr = (READ_MEM(dl + 2) << 8) | READ_MEM(dl);
				width = ((READ_MEM(dl + 3) ^ 0xff) & 0x1f) + 1;
				hpos = READ_MEM(dl + 4);
				pal = READ_MEM(dl + 3) >> 5;
				m_write_mode = (READ_MEM(dl + 1) & 0x80) >> 5;
				ind = READ_MEM(dl + 1) & 0x20;
				dl += 5;
				maria_cycles += 10;
			}
			/* Normal header */
			else
			{
				graph_adr = (READ_MEM(dl + 2) << 8) | READ_MEM(dl);
				width = ((READ_MEM(dl + 1) ^ 0xff) & 0x1f) + 1;
				hpos = READ_MEM(dl + 3);
				pal = READ_MEM(dl + 1) >> 5;
				ind = 0x00;
				dl += 4;
				maria_cycles += 8;
			}

			dma_hole_known=0;

			/*logerror("%x DL: ADR=%x  width=%x  hpos=%x  pal=%x  mode=%x  ind=%x\n", m_screen->vpos(), graph_adr, width, hpos, pal, m_write_mode, ind);*/

			for (int x = 0; x < width; x++)
			{
				if (maria_cycles >= DMALIMIT) // ensure we haven't overrun the maximum render time
					break;

				/* Do indirect mode */
				if (ind)
				{
					c = READ_MEM(graph_adr + x) & 0xff;
					data_addr = (m_charbase | c) + (m_offset << 8);
					if (is_holey(data_addr))
					{	
						if(!dma_hole_known)
							maria_cycles += 3;
						dma_hole_known=1;
						continue;
					}
					maria_cycles += 3;
					if (m_cwidth) // two data bytes per map byte
					{
						cells = write_line_ram(data_addr, hpos, pal);
						hpos += cells;
						cells = write_line_ram(data_addr+1, hpos, pal);
						hpos += cells;
						maria_cycles += 6;
					}
					else
					{
						cells = write_line_ram(data_addr, hpos, pal);
						hpos += cells;
						maria_cycles += 3;
					}
				}
				else // direct mode
				{
					data_addr = graph_adr + x + (m_offset  << 8);
					if (is_holey(data_addr))
					{	
						if(!dma_hole_known)
							maria_cycles += 3;
						dma_hole_known=1;
						continue;
					}
					cells = write_line_ram(data_addr, hpos, pal);
					hpos += cells;
					maria_cycles += 3;
				}
			}
		}

		// Last Line post-render DMA cycle penalties...
		if (m_offset == 0)
		{
			maria_cycles += 8; // extra shutdown time

			if (READ_MEM(m_dll + 3) & 0x80)
				maria_cycles += 17; // interrupt overhead

		}

		if (m_devmode_flag)
		{
			if (maria_cycles>=DMALIMIT)
			{
				// if we hit DMA limits, use a yellow+wide indicator
				m_devmode_dmacolor=0x1f;
				m_devmode_dmacolorwidth=8;
			}
			else
			{
				// otherwise use a regular red gradient, with brightness indicating % of DMA used
				m_devmode_dmacolor = ((maria_cycles*0x0f)/428);
				if (m_devmode_dmacolor>0x0f)
					m_devmode_dmacolor=0x0f;
				m_devmode_dmacolor|=0x40;
				m_devmode_dmacolorwidth=4;
			}
		}

		// If MARIA used up all of the DMA time then the CPU can't run until next line...
		if (maria_cycles>=DMALIMIT)
		{
			m_cpu->spin_until_trigger(TRIGGER_HSYNC);
			m_wsync = 1;
		}

		// Spin the CPU for Maria DMA, if it's not already spinning for WSYNC.
		// MARIA generates the 6502 clock by dividing its own clock by 4. It needs to HALT and unHALT
		// the 6502 on ths same clock phase, so MARIA will wait until its clock divides evenly by 4.
		// To spin until an even divisor, we just round-up any would-be truncations by adding 3.
		if (!m_wsync)
			m_cpu->spin_until_time(m_cpu->cycles_to_attotime((maria_cycles+3)/4));
	} // if m_dmaon
}

void atari_maria_device::display_visible(int pixelx)
{

	// draw line buffer to screen
	uint16_t *scanline;
	scanline = &m_bitmap.pix16(m_screen->vpos());

	int d, pixel_cell;

	int standby_buffer = !m_active_buffer; // display using the non-active buffer

	if (m_devmode_flag && (pixelx<m_devmode_dmacolorwidth))
	{
		scanline[2 * pixelx]      = m_devmode_dmacolor;
		scanline[2 * pixelx + 1 ] = m_devmode_dmacolor;
		return;
	}

	switch (m_rm)
	{

		case 0x00:  /* 160A, 160B */
		case 0x01:  /* 160A, 160B */
			pixel_cell =  m_line_ram[standby_buffer][pixelx];
			scanline[2 * pixelx] = m_maria_palette[pixel_cell];
			scanline[2 * pixelx + 1] = m_maria_palette[pixel_cell];
			break;
		case 0x02: /* 320B, 320D */
			pixel_cell = m_line_ram[standby_buffer][pixelx];
			d = (pixel_cell & 0x10) | (pixel_cell & 0x02) | ((pixel_cell >> 3) & 1); // b4 0 0 b1 b3
			scanline[2 * pixelx] = m_maria_palette[d];
			d = (pixel_cell & 0x10) | ((pixel_cell << 1) & 0x02) | ((pixel_cell >> 2) & 1); // b4 0 0 b0 b2
			scanline[2 * pixelx + 1] = m_maria_palette[d];
			break;

		case 0x03:  /* 320A, 320C */
			pixel_cell = m_line_ram[standby_buffer][pixelx];
			d = (pixel_cell & 0x1c) | (pixel_cell & 0x02); // b4 b3 b2 b1 0
			scanline[2 * pixelx] = m_maria_palette[d];
			d = (pixel_cell & 0x1c) | ((pixel_cell << 1) & 0x02); // b4 b3 b2 b0 0
			scanline[2 * pixelx + 1] = m_maria_palette[d];
			break;
	}

	if(m_color_kill) //remove color if there's no colorburst signal
	{
		scanline[2 * pixelx] &= 0x0f;
		scanline[2 * pixelx + 1] &= 0x0f;
	}

}


void atari_maria_device::interrupt(int lines)
{
	if (m_wsync)
	{
		machine().scheduler().trigger(TRIGGER_HSYNC);
		m_wsync = 0;
	}

	int frame_scanline = m_screen->vpos() % (lines + 1);
	if (frame_scanline == 16)
		m_vblank = 0x00;

	if (frame_scanline == (lines - 5))
		m_vblank = 0x80;
}


void atari_maria_device::startdma(int lines)
{
	address_space& space = m_cpu->space(AS_PROGRAM);
	int maria_scanline = m_screen->vpos();
	int frame_scanline = maria_scanline % (lines + 1);

	m_active_buffer = !m_active_buffer; // switch active buffer at start of DMA

	if ((frame_scanline == 16) && m_dmaon)
	{
		/* end of vblank */
		m_dll = m_dpp; // currently only handle changes to dll during vblank
		m_dl = (READ_MEM(m_dll + 1) << 8) | READ_MEM(m_dll+2);
		m_offset = READ_MEM(m_dll) & 0x0f;
		m_holey = (READ_MEM(m_dll) & 0x60) >> 5;
		m_nmi = READ_MEM(m_dll) & 0x80;
		/*  logerror("DLL=%x\n",m_dll); */
	}

	if ((frame_scanline > 15) && (frame_scanline < (lines - 5)))
		draw_scanline();

	if ((frame_scanline > 16) && (frame_scanline < (lines - 5)) && m_dmaon)
	{
		if (m_offset == 0)
		{
			m_dll += 3;
			m_dl = (READ_MEM(m_dll + 1) << 8) | READ_MEM(m_dll + 2);
			m_offset = READ_MEM(m_dll) & 0x0f;
			m_holey = (READ_MEM(m_dll) & 0x60) >> 5;
			if (READ_MEM(m_dll & 0x10))
				logerror("dll bit 5 set!\n");
			m_nmi = READ_MEM(m_dll) & 0x80;
		}
		else
		{
			m_offset--;
		}
	}

	if (m_nmi)
	{
		m_cpu->set_input_line(INPUT_LINE_NMI, PULSE_LINE);
		m_nmi = 0;
	}
}

/***************************************************************************

  Refresh the video screen

***************************************************************************/

/* This routine is called at the start of vblank to refresh the screen */
uint32_t atari_maria_device::screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	copybitmap(bitmap, m_bitmap, 0, 0, 0, 0, cliprect);
	return 0;
}


READ8_MEMBER(atari_maria_device::read)
{
	switch (offset)
	{
		case 0x08:
			return m_vblank;

		default:
			logerror("undefined MARIA read %x\n",offset);
			return 0x00; // don't know if this should be 0x00 or 0xff
	}
}

WRITE8_MEMBER(atari_maria_device::write)
{
	if ((offset & 3) != 0)
		m_maria_palette[offset] = data;

	switch (offset)
	{
		case 0x00:
			// all color ram addresses with 00 for their least significant bits point to the background color
			for (int i = 0; i < 8; i++)
				m_maria_palette[4 * i] = data;
			break;
		case 0x04:
			m_cpu->spin_until_trigger(TRIGGER_HSYNC);
			m_wsync = 1;
			break;
		case 0x0C: // DPPH
			m_dpp = (m_dpp & 0x00ff) | (data << 8);
			break;
		case 0x10: // DPPL
			m_dpp = (m_dpp & 0xff00) | data;
			break;
		case 0x14:
			m_charbase = (data << 8);
			break;
		case 0x1C:
			/*logerror("MARIA CTRL=%x\n",data);*/
			m_color_kill = data & 0x80;
			switch ((data >> 5) & 3)
			{
				case 0x00:
				case 01:
					logerror("dma test mode, do not use.\n");
					break;

				case 0x02:
					m_dmaon = 1;
					break;

				case 0x03:
					m_dmaon = 0;
					break;
			}
			m_cwidth = data & 0x10;
			m_bcntl = data & 0x08; // Currently unimplemented as we don't display the border
			m_kangaroo = data & 0x04;
			m_rm = data & 0x03;

			/*logerror( "MARIA CTRL: CK:%d DMA:%d CW:%d BC:%d KM:%d RM:%d\n",
			        m_color_kill ? 1 : 0,
			        (data & 0x60) >> 5,
			        m_cwidth ? 1 : 0,
			        m_bcntl ? 1 : 0,
			        m_kangaroo ? 1 : 0,
			        m_rm );*/

			break;
	}
}
