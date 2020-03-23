// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    RCA CDP1869/1870/1876 Video Interface System (VIS) emulation

**********************************************************************
                            _____   _____
                   TPA   1 |*    \_/     | 40  Vdd
                   TPB   2 |             | 39  PMSEL
                  _MRD   3 |             | 38  _PMWR
                  _MWR   4 |             | 37  CMSEL
                 MA0/8   5 |             | 36  _CMWR
                 MA1/9   6 |             | 35  PMA0
                MA2/10   7 |             | 34  PMA1
                MA3/11   8 |             | 33  PMA2
                MA4/12   9 |             | 32  PMA3
                MA5/13  10 |             | 31  PMA4
                MA6/14  11 |   CDP1869   | 30  PMA5
                MA7/15  12 |             | 29  PMA6
                    N0  13 |             | 28  PMA7
                    N1  14 |             | 27  PMA8
                    N2  15 |             | 26  PMA9
               _H SYNC  16 |             | 25  CMA3/PMA10
              _DISPLAY  17 |             | 24  CMA2
              _ADDRSTB  18 |             | 23  CMA1
                 SOUND  19 |             | 22  CMA0
                   Vss  20 |_____________| 21  _N=3

                            _____   _____
           _PREDISPLAY   1 |*    \_/     | 40  Vdd
              _DISPLAY   2 |             | 39  PAL/_NTSC
                   PCB   3 |             | 38  CPUCLK
                  CCB1   4 |             | 37  XTAL (DOT)
                  BUS7   5 |             | 36  _XTAL (DOT)
                  CCB0   6 |             | 35  _ADDRSTB
                  BUS6   7 |             | 34  _MRD
                  CDB5   8 |             | 33  TPB
                  BUS5   9 |             | 32  CMSEL
                  CDB4  10 |             | 31  BURST
                  BUS4  11 |   CDP1870   | 30  _H SYNC
                  CDB3  12 |             | 29  _COMPSYNC
                  BUS3  13 |             | 28  LUM
                  CDB2  14 |             | 27  PAL CHROM
                  BUS2  15 |             | 26  NTSC CHROM
                  CDB1  16 |             | 25  _XTAL (CHROM)
                  BUS1  17 |             | 24  XTAL (CHROM)
                  CDB0  18 |             | 23  _EMS
                  BUS0  19 |             | 22  _EVS
                   Vss  20 |_____________| 21  _N=3

                            _____   _____
           _PREDISPLAY   1 |*    \_/     | 40  Vdd
              _DISPLAY   2 |             | 39  PAL/_NTSC
                   PCB   3 |             | 38  CPUCLK
                  CCB1   4 |             | 37  XTAL (DOT)
                  BUS7   5 |             | 36  _XTAL (DOT)
                  CCB0   6 |             | 35  _ADDRSTB
                  BUS6   7 |             | 34  _MRD
                  CDB5   8 |             | 33  TPB
                  BUS5   9 |             | 32  CMSEL
                  CDB4  10 |             | 31  BURST
                  BUS4  11 |   CDP1876   | 30  _H SYNC
                  CDB3  12 |             | 29  _COMPSYNC
                  BUS3  13 |             | 28  RED
                  CDB2  14 |             | 27  BLUE
                  BUS2  15 |             | 26  GREEN
                  CDB1  16 |             | 25  _XTAL (CHROM)
                  BUS1  17 |             | 24  XTAL (CHROM)
                  CDB0  18 |             | 23  _EMS
                  BUS0  19 |             | 22  _EVS
                   Vss  20 |_____________| 21  _N=3

**********************************************************************/

#ifndef MAME_SOUND_CDP1869_H
#define MAME_SOUND_CDP1869_H

#pragma once

#include "screen.h"



//**************************************************************************
//  INTERFACE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_CDP1869_ADD(_tag, _pixclock, _map) \
		MCFG_DEVICE_ADD(_tag, CDP1869, _pixclock) \
		MCFG_DEVICE_ADDRESS_MAP(0, _map)

#define MCFG_CDP1869_SCREEN_PAL_ADD(_cdptag, _tag, _clock) \
		MCFG_SCREEN_ADD(_tag, RASTER) \
		MCFG_SCREEN_UPDATE_DEVICE(_cdptag, cdp1869_device, screen_update) \
		MCFG_SCREEN_RAW_PARAMS(_clock, cdp1869_device::SCREEN_WIDTH, cdp1869_device::HBLANK_END, cdp1869_device::HBLANK_START, cdp1869_device::TOTAL_SCANLINES_PAL, cdp1869_device::SCANLINE_VBLANK_END_PAL, cdp1869_device::SCANLINE_VBLANK_START_PAL)

#define MCFG_CDP1869_SCREEN_NTSC_ADD(_cdptag, _tag, _clock) \
		MCFG_SCREEN_ADD(_tag, RASTER) \
		MCFG_SCREEN_UPDATE_DEVICE(_cdptag, cdp1869_device, screen_update) \
		MCFG_SCREEN_RAW_PARAMS(_clock, cdp1869_device::SCREEN_WIDTH, cdp1869_device::HBLANK_END, cdp1869_device::HBLANK_START, cdp1869_device::TOTAL_SCANLINES_NTSC, cdp1869_device::SCANLINE_VBLANK_END_NTSC, cdp1869_device::SCANLINE_VBLANK_START_NTSC)

#define MCFG_CDP1869_SET_SCREEN MCFG_VIDEO_SET_SCREEN

#define CDP1869_CHAR_RAM_READ_MEMBER(name) uint8_t name(uint16_t pma, uint8_t cma, uint8_t pmd)
#define CDP1869_CHAR_RAM_WRITE_MEMBER(name) void name(uint16_t pma, uint8_t cma, uint8_t pmd, uint8_t data)
#define CDP1869_PCB_READ_MEMBER(name) int name(uint16_t pma, uint8_t cma, uint8_t pmd)

#define MCFG_CDP1869_PAL_NTSC_CALLBACK(_read) \
		devcb = &cdp1869_device::set_pal_ntsc_rd_callback(*device, DEVCB_##_read);

#define MCFG_CDP1869_PRD_CALLBACK(_write) \
		devcb = &cdp1869_device::set_prd_wr_callback(*device, DEVCB_##_write);

#define MCFG_CDP1869_COLOR_CLOCK(_clk) \
		cdp1869_device::static_set_color_clock(*device, _clk);

#define MCFG_CDP1869_CHAR_RAM_READ_OWNER(_class, _method) \
		cdp1869_device::static_set_char_ram_read(*device, cdp1869_device::char_ram_read_delegate(&_class::_method, #_class "::" #_method, downcast<_class *>(owner)));

#define MCFG_CDP1869_CHAR_RAM_WRITE_OWNER(_class, _method) \
		cdp1869_device::static_set_char_ram_write(*device, cdp1869_device::char_ram_write_delegate(&_class::_method, #_class "::" #_method, downcast<_class *>(owner)));

#define MCFG_CDP1869_CHAR_PCB_READ_OWNER(_class, _method) \
		cdp1869_device::static_set_pcb_read(*device, cdp1869_device::pcb_read_delegate(&_class::_method, #_class "::" #_method, downcast<_class *>(owner)));


//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> cdp1869_device

class cdp1869_device :  public device_t,
						public device_sound_interface,
						public device_video_interface,
						public device_memory_interface
{
public:
	static constexpr float DOT_CLK_PAL         = XTAL_5_626MHz;
	static constexpr float DOT_CLK_NTSC        = XTAL_5_67MHz;
	static constexpr float COLOR_CLK_PAL       = XTAL_8_867236MHz;
	static constexpr float COLOR_CLK_NTSC      = XTAL_7_15909MHz;

	static constexpr float CPU_CLK_PAL         = DOT_CLK_PAL / 2;
	static constexpr float CPU_CLK_NTSC        = DOT_CLK_NTSC / 2;

	static constexpr unsigned CHAR_WIDTH          = 6;

	static constexpr unsigned HSYNC_START         = 56 * CHAR_WIDTH;
	static constexpr unsigned HSYNC_END           = 60 * CHAR_WIDTH;
	static constexpr unsigned HBLANK_START        = 54 * CHAR_WIDTH;
	static constexpr unsigned HBLANK_END          =  5 * CHAR_WIDTH;
	static constexpr unsigned SCREEN_START_PAL    =  9 * CHAR_WIDTH;
	static constexpr unsigned SCREEN_START_NTSC   = 10 * CHAR_WIDTH;
	static constexpr unsigned SCREEN_START        = 10 * CHAR_WIDTH;
	static constexpr unsigned SCREEN_END          = 50 * CHAR_WIDTH;
	static constexpr unsigned SCREEN_WIDTH        = 60 * CHAR_WIDTH;

	static constexpr unsigned TOTAL_SCANLINES_PAL             = 312;
	static constexpr unsigned SCANLINE_VBLANK_START_PAL       = 304;
	static constexpr unsigned SCANLINE_VBLANK_END_PAL         = 10;
	static constexpr unsigned SCANLINE_VSYNC_START_PAL        = 308;
	static constexpr unsigned SCANLINE_VSYNC_END_PAL          = 312;
	static constexpr unsigned SCANLINE_DISPLAY_START_PAL      = 44;
	static constexpr unsigned SCANLINE_DISPLAY_END_PAL        = 260;
	static constexpr unsigned SCANLINE_PREDISPLAY_START_PAL   = 43;
	static constexpr unsigned SCANLINE_PREDISPLAY_END_PAL     = 260;
	static constexpr unsigned VISIBLE_SCANLINES_PAL           = SCANLINE_DISPLAY_END_PAL - SCANLINE_DISPLAY_START_PAL;

	static constexpr unsigned TOTAL_SCANLINES_NTSC            = 262;
	static constexpr unsigned SCANLINE_VBLANK_START_NTSC      = 252;
	static constexpr unsigned SCANLINE_VBLANK_END_NTSC        = 10;
	static constexpr unsigned SCANLINE_VSYNC_START_NTSC       = 258;
	static constexpr unsigned SCANLINE_VSYNC_END_NTSC         = 262;
	static constexpr unsigned SCANLINE_DISPLAY_START_NTSC     = 36;
	static constexpr unsigned SCANLINE_DISPLAY_END_NTSC       = 228;
	static constexpr unsigned SCANLINE_PREDISPLAY_START_NTSC  = 35;
	static constexpr unsigned SCANLINE_PREDISPLAY_END_NTSC    = 228;
	static constexpr unsigned VISIBLE_SCANLINES_NTSC          = SCANLINE_DISPLAY_END_NTSC - SCANLINE_DISPLAY_START_NTSC;

	static constexpr unsigned PALETTE_LENGTH  = 8+64;

	typedef device_delegate<uint8_t (uint16_t pma, uint8_t cma, uint8_t pmd)> char_ram_read_delegate;
	typedef device_delegate<void (uint16_t pma, uint8_t cma, uint8_t pmd, uint8_t data)> char_ram_write_delegate;
	typedef device_delegate<int (uint16_t pma, uint8_t cma, uint8_t pmd)> pcb_read_delegate;

	// construction/destruction
	cdp1869_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	template <class Object> static devcb_base &set_pal_ntsc_rd_callback(device_t &device, Object &&cb) { return downcast<cdp1869_device &>(device).m_read_pal_ntsc.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_prd_wr_callback(device_t &device, Object &&cb) { return downcast<cdp1869_device &>(device).m_write_prd.set_callback(std::forward<Object>(cb)); }
	static void static_set_char_ram_read(device_t &device, char_ram_read_delegate &&cb) { downcast<cdp1869_device &>(device).m_in_char_ram_func = std::move(cb); }
	static void static_set_char_ram_write(device_t &device, char_ram_write_delegate &&cb) { downcast<cdp1869_device &>(device).m_out_char_ram_func = std::move(cb); }
	static void static_set_pcb_read(device_t &device, pcb_read_delegate &&cb) { downcast<cdp1869_device &>(device).m_in_pcb_func = std::move(cb); }
	static void static_set_color_clock(device_t &device, int color_clock) { downcast<cdp1869_device &>(device).m_color_clock = color_clock; }

	virtual DECLARE_ADDRESS_MAP(io_map, 8);
	virtual DECLARE_ADDRESS_MAP(char_map, 8);
	virtual DECLARE_ADDRESS_MAP(page_map, 8);

	DECLARE_WRITE8_MEMBER( out3_w );
	DECLARE_WRITE8_MEMBER( out4_w );
	DECLARE_WRITE8_MEMBER( out5_w );
	DECLARE_WRITE8_MEMBER( out6_w );
	DECLARE_WRITE8_MEMBER( out7_w );

	DECLARE_READ8_MEMBER( char_ram_r );
	DECLARE_WRITE8_MEMBER( char_ram_w );

	DECLARE_READ8_MEMBER( page_ram_r );
	DECLARE_WRITE8_MEMBER( page_ram_w );

	DECLARE_READ_LINE_MEMBER( predisplay_r );
	DECLARE_READ_LINE_MEMBER( pal_ntsc_r );

	uint32_t screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);

protected:
	// device-level overrides
	virtual void device_add_mconfig(machine_config &config) override;
	virtual void device_start() override;
	virtual void device_post_load() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

	// device_memory_interface overrides
	virtual space_config_vector memory_space_config() const override;

	// device_sound_interface callbacks
	virtual void sound_stream_update(sound_stream &stream, stream_sample_t **inputs, stream_sample_t **outputs, int samples) override;

	inline bool is_ntsc();
	inline uint8_t read_page_ram_byte(offs_t address);
	inline void write_page_ram_byte(offs_t address, uint8_t data);
	inline uint8_t read_char_ram_byte(offs_t pma, offs_t cma, uint8_t pmd);
	inline void write_char_ram_byte(offs_t pma, offs_t cma, uint8_t pmd, uint8_t data);
	inline int read_pcb(offs_t pma, offs_t cma, uint8_t pmd);
	inline void update_prd_changed_timer();
	inline rgb_t get_rgb(int i, int c, int l);
	inline int get_lines();
	inline uint16_t get_pmemsize(int cols, int rows);
	inline uint16_t get_pma();
	inline int get_pen(int ccb0, int ccb1, int pcb);

	void draw_line(bitmap_rgb32 &bitmap, const rectangle &rect, int x, int y, uint8_t data, int color);
	void draw_char(bitmap_rgb32 &bitmap, const rectangle &rect, int x, int y, uint16_t pma);

private:
	devcb_read_line        m_read_pal_ntsc;
	devcb_write_line       m_write_prd;
	pcb_read_delegate           m_in_pcb_func;
	char_ram_read_delegate      m_in_char_ram_func;
	char_ram_write_delegate     m_out_char_ram_func;
	int m_color_clock;

	//address_space *m_page_ram;
	emu_timer *m_prd_timer;
	sound_stream *m_stream;
	required_device<palette_device> m_palette;
	const address_space_config      m_space_config;

	// video state
	int m_prd;                      // predisplay
	int m_dispoff;                  // display off
	int m_fresvert;                 // full resolution vertical
	int m_freshorz;                 // full resolution horizontal
	int m_cmem;                     // character memory access mode
	int m_dblpage;                  // double page mode
	int m_line16;                   // 16-line hi-res mode
	int m_line9;                    // 9 line mode
	int m_cfc;                      // color format control
	uint8_t m_col;                    // character color control
	uint8_t m_bkg;                    // background color
	uint16_t m_pma;                   // page memory address
	uint16_t m_hma;                   // home memory address

	// sound state
	int16_t m_signal;                 // current signal
	int m_incr;                     // initial wave state
	int m_toneoff;                  // tone off
	int m_wnoff;                    // white noise off
	uint8_t m_tonediv;                // tone divisor
	uint8_t m_tonefreq;               // tone range select
	uint8_t m_toneamp;                // tone output amplitude
	uint8_t m_wnfreq;                 // white noise range select
	uint8_t m_wnamp;                  // white noise output amplitude

	DECLARE_PALETTE_INIT(cdp1869);
};


// device type definition
DECLARE_DEVICE_TYPE(CDP1869, cdp1869_device)

#endif // MAME_SOUND_CDP1869_H
