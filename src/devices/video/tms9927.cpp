// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/**********************************************************************

    TI TMS9927 and compatible CRT controller emulation

**********************************************************************/

#include "emu.h"
#include "video/tms9927.h"

#include "screen.h"


static constexpr uint8_t chars_per_row_value[8] = { 20, 32, 40, 64, 72, 80, 96, 132 };
static constexpr uint8_t skew_bits_value[4] = { 0, 1, 2, 2 };


#define HCOUNT               (m_reg[0] + 1)
#define INTERLACED           ((m_reg[1] >> 7) & 0x01)
#define HSYNC_WIDTH          (((m_reg[1] >> 3) & 0x0f) + 1)
#define HSYNC_DELAY          (((m_reg[1] >> 0) & 0x07) + 1)
#define SCANS_PER_DATA_ROW   (((m_reg[2] >> 3) & 0x0f) + 1)
#define CHARS_PER_DATA_ROW   (chars_per_row_value[(m_reg[2] >> 0) & 0x07])
#define SKEW_BITS            (skew_bits_value[(m_reg[3] >> 6) & 0x03])
#define DATA_ROWS_PER_FRAME  ((m_reg[3] & 0x3f) + 1)
#define SCAN_LINES_PER_FRAME ((m_reg[4] * 2) + 256 + (((m_reg[1] >> 7) & 0x01) * 257))
#define VERTICAL_DATA_START  (m_reg[5])
#define LAST_DISP_DATA_ROW   (m_reg[6] & 0x3f)
#define CURSOR_CHAR_ADDRESS  (m_reg[7])
#define CURSOR_ROW_ADDRESS   (m_reg[8] & 0x3f)


DEFINE_DEVICE_TYPE(TMS9927, tms9927_device, "tms9927", "TMS9927 VTC")
DEFINE_DEVICE_TYPE(CRT5027, crt5027_device, "crt5027", "CRT5027")
DEFINE_DEVICE_TYPE(CRT5037, crt5037_device, "crt5037", "CRT5037")
DEFINE_DEVICE_TYPE(CRT5057, crt5057_device, "crt5057", "CRT5057")

tms9927_device::tms9927_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: tms9927_device(mconfig, TMS9927, tag, owner, clock)
{
}

tms9927_device::tms9927_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, type, tag, owner, clock)
	, device_video_interface(mconfig, *this)
	, m_write_vsyn(*this)
	, m_hpixels_per_column(0)
	, m_overscan_left(0)
	, m_overscan_right(0)
	, m_overscan_top(0)
	, m_overscan_bottom(0)
	, m_selfload(*this, finder_base::DUMMY_TAG)
	, m_reset(0)
{
	std::fill(std::begin(m_reg), std::end(m_reg), 0x00);
}

crt5027_device::crt5027_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: tms9927_device(mconfig, CRT5027, tag, owner, clock)
{
}

crt5037_device::crt5037_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: tms9927_device(mconfig, CRT5037, tag, owner, clock)
{
}

crt5057_device::crt5057_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: tms9927_device(mconfig, CRT5057, tag, owner, clock)
{
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void tms9927_device::device_start()
{
	assert(clock() > 0);
	if (!(m_hpixels_per_column > 0)) fatalerror("TMS9927: number of pixels per column must be explicitly set using MCFG_TMS9927_CHAR_WIDTH()!\n");

	/* copy the initial parameters */
	m_clock = clock();

	// resolve callbacks
	m_write_vsyn.resolve_safe();

	// allocate timers
	m_vsync_timer = timer_alloc(TIMER_VSYNC);

	/* register for state saving */
	machine().save().register_postload(save_prepost_delegate(FUNC(tms9927_device::state_postload), this));

	save_item(NAME(m_reg));
	save_item(NAME(m_start_datarow));
	save_item(NAME(m_reset));
}

//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void tms9927_device::device_reset()
{
	m_start_datarow = 0;
}

//-------------------------------------------------
//  device_stop - device-specific stop
//-------------------------------------------------

void tms9927_device::device_stop()
{
	osd_printf_debug("TMS9927: Final params: (%d, %d, %d, %d, %d, %d, %d)\n",
						m_clock,
						m_total_hpix,
						0, m_visible_hpix,
						m_total_vpix,
						0, m_visible_vpix);
}



//-------------------------------------------------
//  device_timer - handle timer events
//-------------------------------------------------

void tms9927_device::device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr)
{
	switch (id)
	{
	case TIMER_VSYNC:
		m_vsyn = !m_vsyn;

		m_write_vsyn(m_vsyn);

		if (m_vsyn)
		{
			m_vsync_timer->adjust(m_screen->time_until_pos(3));
		}
		else
		{
			m_vsync_timer->adjust(m_screen->time_until_pos(0));
		}
		break;
	}
}

void tms9927_device::state_postload()
{
	recompute_parameters(true);
}


void tms9927_device::generic_access(address_space &space, offs_t offset)
{
	switch (offset)
	{
		case 0x07:  /* Processor Self Load */
		case 0x0f:  /* Non-processor self-load */
			if (m_selfload.found())
			{
				for (int cur = 0; cur < 7; cur++)
					write(space, cur, m_selfload[cur]);
				for (int cur = 0; cur < 1; cur++)
					write(space, cur + 0xc, m_selfload[cur + 7]);
			}
			else
				popmessage("tms9927: self-load initiated with no PROM!");

			/* processor self-load waits with reset enabled;
			   non-processor just goes ahead */
			m_reset = (offset == 0x07);
			break;

		case 0x0a:  /* Reset */
			if (!m_reset)
			{
				m_screen->update_now();
				m_reset = true;
			}
			break;

		case 0x0b:  /* Up scroll */
			m_screen->update_now();
			m_start_datarow = (m_start_datarow + 1) % DATA_ROWS_PER_FRAME;
			break;

		case 0x0e:  /* Start timing chain */
			if (m_reset)
			{
				m_screen->update_now();
				m_reset = false;
				recompute_parameters(false);
			}
			break;
	}
}

WRITE8_MEMBER( tms9927_device::write )
{
	switch (offset)
	{
		case 0x00:  /* HORIZONTAL CHARACTER COUNT */
		case 0x01:  /* INTERLACED / HSYNC WIDTH / HSYNC DELAY */
		case 0x02:  /* SCANS PER DATA ROW / CHARACTERS PER DATA ROW */
		case 0x03:  /* SKEW BITS / DATA ROWS PER FRAME */
		case 0x04:  /* SCAN LINES / FRAME */
		case 0x05:  /* VERTICAL DATA START */
		case 0x06:  /* LAST DISPLAYED DATA ROW */
			m_reg[offset] = data;
			recompute_parameters(false);
			break;

		case 0x0c:  /* LOAD CURSOR CHARACTER ADDRESS */
		case 0x0d:  /* LOAD CURSOR ROW ADDRESS */
osd_printf_debug("Cursor address changed\n");
			m_reg[offset - 0x0c + 7] = data;
			/* Recomputing parameters here will break the scrollup on the Attachè
			   and probably other machines due to m_start_datarow being reset ! */
			//recompute_parameters(false);
			break;

		default:
			generic_access(space, offset);
			break;
	}
}


READ8_MEMBER( tms9927_device::read )
{
	switch (offset)
	{
		case 0x08:  /* READ CURSOR CHARACTER ADDRESS */
		case 0x09:  /* READ CURSOR ROW ADDRESS */
			return m_reg[offset - 0x08 + 7];

		default:
			generic_access(space, offset);
			break;
	}
	return 0xff;
}


int tms9927_device::screen_reset()
{
	return m_reset;
}


int tms9927_device::upscroll_offset()
{
	return m_start_datarow;
}


int tms9927_device::cursor_bounds(rectangle &bounds)
{
	int cursorx = CURSOR_CHAR_ADDRESS;
	int cursory = CURSOR_ROW_ADDRESS;

	bounds.min_x = cursorx * m_hpixels_per_column;
	bounds.max_x = bounds.min_x + m_hpixels_per_column - 1;
	bounds.min_y = cursory * SCANS_PER_DATA_ROW;
	bounds.max_y = bounds.min_y + SCANS_PER_DATA_ROW - 1;

	return (cursorx < HCOUNT && cursory <= LAST_DISP_DATA_ROW);
}


void tms9927_device::recompute_parameters(bool postload)
{
	attoseconds_t refresh;
	rectangle visarea;

	if (m_reset)
		return;

	/* compute the screen sizes */
	m_total_hpix = HCOUNT * m_hpixels_per_column;
	m_total_vpix = SCAN_LINES_PER_FRAME;

	/* determine the visible area, avoid division by 0 */
	m_visible_hpix = CHARS_PER_DATA_ROW * m_hpixels_per_column;
	m_visible_vpix = DATA_ROWS_PER_FRAME * SCANS_PER_DATA_ROW;

	m_start_datarow = (LAST_DISP_DATA_ROW + 1) % DATA_ROWS_PER_FRAME;

	osd_printf_debug("TMS9927: Total = %dx%d, Visible = %dx%d, Skew=%d, Upscroll=%d\n", m_total_hpix, m_total_vpix, m_visible_hpix, m_visible_vpix, SKEW_BITS, m_start_datarow);

	/* see if it all makes sense */
	m_valid_config = true;
	if ( (m_visible_hpix > m_total_hpix || m_visible_vpix > m_total_vpix) || (((m_visible_hpix-1)<=0) || ((m_visible_vpix-1)<=0)) || ((m_total_hpix * m_total_vpix) == 0) )
	{
		m_valid_config = false;
		logerror("tms9927: invalid visible size (%dx%d) versus total size (%dx%d)\n", m_visible_hpix, m_visible_vpix, m_total_hpix, m_total_vpix);
	}

	if (m_clock == 0)
	{
		m_valid_config = false;
		// TODO: make the screen refresh never, and disable the vblank and odd/even interrupts here!
		logerror("tms9927: invalid clock rate of zero defined!\n");
	}

	/* update */
	if (!m_valid_config)
		return;

	/* create a visible area */
	visarea.set(0, m_overscan_left + m_visible_hpix + m_overscan_right - 1,
				0, m_overscan_top + m_visible_vpix + m_overscan_bottom - 1);

	refresh = HZ_TO_ATTOSECONDS(m_clock) * m_total_hpix * m_total_vpix;

	m_screen->configure(m_total_hpix, m_total_vpix, visarea, refresh);

	m_vsyn = 0;
	m_vsync_timer->adjust(m_screen->time_until_pos(0, 0));

}
