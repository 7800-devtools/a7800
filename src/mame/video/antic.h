// license:GPL-2.0+
// copyright-holders:Juergen Buchmueller
/***************************************************************************

    Atari 400/800

    ANTIC video controller

    Juergen Buchmueller, June 1998

***************************************************************************/

#ifndef MAME_VIDEO_ANTIC_H
#define MAME_VIDEO_ANTIC_H

#pragma once

#include "video/gtia.h"


class antic_device : public device_t, public device_video_interface
{
private:
	static constexpr unsigned   VBL_END             = 8;    // vblank ends in this scanline
	static constexpr unsigned   VDATA_START         = 11;   // video display begins in this scanline
	static constexpr unsigned   VDATA_END           = 244;  // video display ends in this scanline
	static constexpr unsigned   VBL_START           = 248;  // vblank starts in this scanline

public:
	// total number of lines per frame (incl. vblank)
	static constexpr unsigned   TOTAL_LINES_60HZ    = 262;
	static constexpr unsigned   TOTAL_LINES_50HZ    = 312;

	// frame rates
	static constexpr double     FRAME_RATE_50HZ     = 1789790.0 / 114 / TOTAL_LINES_50HZ;
	static constexpr double     FRAME_RATE_60HZ     = 1789790.0 / 114 / TOTAL_LINES_60HZ;

	static constexpr unsigned   HWIDTH              = 48;   // total characters per line
	static constexpr unsigned   VHEIGHT             = 32;
	static constexpr unsigned   MIN_X               = ((HWIDTH - 42) / 2) * 8;
	static constexpr unsigned   MAX_X               = MIN_X + (42 * 8) - 1;
	static constexpr unsigned   MIN_Y               = VDATA_START;
	static constexpr unsigned   MAX_Y               = VDATA_END - 8 - 1;

	// construction/destruction
	antic_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

	static void set_gtia_tag(device_t &device, const char *tag) { downcast<antic_device &>(device).m_gtia_tag = tag; }

	DECLARE_READ8_MEMBER( read );
	DECLARE_WRITE8_MEMBER( write );

	uint32_t screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	void generic_interrupt(int button_count);

private:
	static constexpr device_timer_id TIMER_CYCLE_STEAL = 0;
	static constexpr device_timer_id TIMER_ISSUE_DLI = 1;
	static constexpr device_timer_id TIMER_LINE_REND = 2;
	static constexpr device_timer_id TIMER_LINE_DONE = 3;

	static constexpr unsigned   HCHARS              = 44;   // visible characters per line
	static constexpr unsigned   VCHARS              = (VDATA_END - VDATA_START + 7) / 8;
	static constexpr unsigned   BUF_OFFS0           = (HWIDTH - HCHARS) / 2;

	struct ANTIC_R {
		uint8_t   antic00;    /* 00 nothing */
		uint8_t   antic01;    /* 01 nothing */
		uint8_t   antic02;    /* 02 nothing */
		uint8_t   antic03;    /* 03 nothing */
		uint8_t   antic04;    /* 04 nothing */
		uint8_t   antic05;    /* 05 nothing */
		uint8_t   antic06;    /* 06 nothing */
		uint8_t   antic07;    /* 07 nothing */
		uint8_t   antic08;    /* 08 nothing */
		uint8_t   antic09;    /* 09 nothing */
		uint8_t   antic0a;    /* 0a nothing */
		uint8_t   vcount;     /* 0b vertical (scanline) counter */
		uint8_t   penh;       /* 0c light pen horizontal pos */
		uint8_t   penv;       /* 0d light pen vertical pos */
		uint8_t   antic0e;    /* 0e nothing */
		uint8_t   nmist;      /* 0f NMI status */
	};  /* read registers */

	struct ANTIC_W {
		uint8_t   dmactl;     /* 00 write DMA control */
		uint8_t   chactl;     /* 01 write character control */
		uint8_t   dlistl;     /* 02 display list low */
		uint8_t   dlisth;     /* 03 display list high */
		uint8_t   hscrol;     /* 04 horz scroll */
		uint8_t   vscrol;     /* 05 vert scroll */
		uint8_t   pmbasl;     /* 06 player/missile base addr low */
		uint8_t   pmbash;     /* 07 player/missile base addr high */
		uint8_t   chbasl;     /* 08 character generator base addr low */
		uint8_t   chbash;     /* 09 character generator base addr high */
		uint8_t   wsync;      /* 0a wait for hsync */
		uint8_t   antic0b;    /* 0b nothing */
		uint8_t   antic0c;    /* 0c nothing */
		uint8_t   antic0d;    /* 0d nothing */
		uint8_t   nmien;      /* 0e NMI enable */
		uint8_t   nmires;     /* 0f NMI reset */
	};  /* write registers */

	/* per scanline buffer for video data (and optimization variables) */
	struct VIDEO {
		uint32_t  cmd;                /* antic command for this scanline */
		uint16_t  data[HWIDTH];       /* graphics data buffer (text through chargen) */
	};


	const char *m_gtia_tag;
	gtia_device  *m_gtia;
	required_device<cpu_device> m_maincpu;
	optional_ioport m_djoy_b;
	optional_ioport m_artifacts;

	uint32_t m_tv_artifacts;
	int m_render1, m_render2, m_render3;

	inline void mode_0(address_space &space, VIDEO *video);
	inline void mode_2(address_space &space, VIDEO *video, int bytes, int erase);
	inline void mode_3(address_space &space, VIDEO *video, int bytes, int erase);
	inline void mode_4(address_space &space, VIDEO *video, int bytes, int erase);
	inline void mode_5(address_space &space, VIDEO *video, int bytes, int erase);
	inline void mode_6(address_space &space, VIDEO *video, int bytes, int erase);
	inline void mode_7(address_space &space, VIDEO *video, int bytes, int erase);
	inline void mode_8(address_space &space, VIDEO *video, int bytes, int erase);
	inline void mode_9(address_space &space, VIDEO *video, int bytes, int erase);
	inline void mode_a(address_space &space, VIDEO *video, int bytes, int erase);
	inline void mode_b(address_space &space, VIDEO *video, int bytes, int erase);
	inline void mode_c(address_space &space, VIDEO *video, int bytes, int erase);
	inline void mode_d(address_space &space, VIDEO *video, int bytes, int erase);
	inline void mode_e(address_space &space, VIDEO *video, int bytes, int erase);
	inline void mode_f(address_space &space, VIDEO *video, int bytes, int erase);
	inline void mode_gtia1(address_space &space, VIDEO *video, int bytes, int erase);
	inline void mode_gtia2(address_space &space, VIDEO *video, int bytes, int erase);
	inline void mode_gtia3(address_space &space, VIDEO *video, int bytes, int erase);

	uint32_t  m_cmd;                /* currently executed display list command */
	uint32_t  m_steal_cycles;       /* steal how many cpu cycles for this line ? */
	uint32_t  m_vscrol_old;         /* old vscrol value */
	uint32_t  m_hscrol_old;         /* old hscrol value */
	int32_t   m_modelines;          /* number of lines for current ANTIC mode */
	uint32_t  m_chbase;             /* character mode source base */
	uint32_t  m_chand;              /* character and mask (chactl) */
	uint32_t  m_chxor;              /* character xor mask (chactl) */
	uint32_t  m_scanline;           /* current scan line */
	uint32_t  m_pfwidth;            /* playfield width */
	uint32_t  m_dpage;              /* display list address page */
	uint32_t  m_doffs;              /* display list offset into page */
	uint32_t  m_vpage;              /* video data source page */
	uint32_t  m_voffs;              /* video data offset into page */
	uint32_t  m_pmbase_s;           /* p/m graphics single line source base */
	uint32_t  m_pmbase_d;           /* p/m graphics double line source base */
	ANTIC_R m_r;                  /* ANTIC read registers */
	ANTIC_W m_w;                  /* ANTIC write registers */
	uint8_t   m_cclock[256+32];     /* color clock buffer filled by ANTIC */
	uint8_t   m_pmbits[256+32];     /* player missile buffer filled by GTIA */
	std::unique_ptr<uint8_t[]>   m_prio_table[64];    /* player/missile priority tables */
	VIDEO   *m_video[312];        /* video buffer */
	std::unique_ptr<uint32_t[]>  m_cclk_expand;       /* shared buffer for the following: */
	uint32_t  *m_pf_21;             /* 1cclk 2 color txt 2,3 */
	uint32_t  *m_pf_x10b;           /* 1cclk 4 color txt 4,5, gfx D,E */
	uint32_t  *m_pf_3210b2;         /* 1cclk 5 color txt 6,7, gfx 9,B,C */
	uint32_t  *m_pf_210b4;          /* 4cclk 4 color gfx 8 */
	uint32_t  *m_pf_210b2;          /* 2cclk 4 color gfx A */
	uint32_t  *m_pf_1b;             /* 1cclk hires gfx F */
	uint32_t  *m_pf_gtia1;          /* 1cclk gtia mode 1 */
	uint32_t  *m_pf_gtia2;          /* 1cclk gtia mode 2 */
	uint32_t  *m_pf_gtia3;          /* 1cclk gtia mode 3 */
	std::unique_ptr<uint8_t[]>   m_used_colors;       /* shared buffer for the following: */
	uint8_t   *m_uc_21;             /* used colors for txt (2,3) */
	uint8_t   *m_uc_x10b;           /* used colors for txt 4,5, gfx D,E */
	uint8_t   *m_uc_3210b2;         /* used colors for txt 6,7, gfx 9,B,C */
	uint8_t   *m_uc_210b4;          /* used colors for gfx 8 */
	uint8_t   *m_uc_210b2;          /* used colors for gfx A */
	uint8_t   *m_uc_1b;             /* used colors for gfx F */
	uint8_t   *m_uc_g1;             /* used colors for gfx GTIA 1 */
	uint8_t   *m_uc_g2;             /* used colors for gfx GTIA 2 */
	uint8_t   *m_uc_g3;             /* used colors for gfx GTIA 3 */
	std::unique_ptr<bitmap_ind16> m_bitmap;

	void prio_init();
	void cclk_init();

	void artifacts_gfx(uint8_t *src, uint8_t *dst, int width);
	void artifacts_txt(uint8_t *src, uint8_t *dst, int width);

	void linerefresh();
	int cycle();

	TIMER_CALLBACK_MEMBER( issue_dli );
	TIMER_CALLBACK_MEMBER( line_done );
	TIMER_CALLBACK_MEMBER( steal_cycles );
	TIMER_CALLBACK_MEMBER( scanline_render );

	void render(address_space &space, int param1, int param2, int param3);

	inline void LMS(int new_cmd);
	void scanline_dma(int param);
};


// device type definition
DECLARE_DEVICE_TYPE(ATARI_ANTIC, antic_device)



#define MCFG_ANTIC_GTIA(_tag) \
		antic_device::set_gtia_tag(*device, _tag);

#define MCFG_SCREEN_VISIBLE_AREA_ANTIC() \
		MCFG_SCREEN_VISIBLE_AREA(antic_device::MIN_X, antic_device::MAX_X, antic_device::MIN_Y, antic_device::MAX_Y)

#define MCFG_SCREEN_REFRESH_RATE_ANTIC_50HZ() \
		MCFG_SCREEN_REFRESH_RATE(antic_device::FRAME_RATE_50HZ)

#define MCFG_SCREEN_REFRESH_RATE_ANTIC_60HZ() \
		MCFG_SCREEN_REFRESH_RATE(antic_device::FRAME_RATE_60HZ)

#define MCFG_SCREEN_SIZE_ANTIC_50HZ() \
		MCFG_SCREEN_SIZE(antic_device::HWIDTH * 8, antic_device::TOTAL_LINES_50HZ)

#define MCFG_SCREEN_SIZE_ANTIC_60HZ() \
		MCFG_SCREEN_SIZE(antic_device::HWIDTH * 8, antic_device::TOTAL_LINES_60HZ)


#endif // MAME_VIDEO_ANTIC_H
