// license:BSD-3-Clause
// copyright-holders:Nathan Woods, Peter Trauner, Angelo Salese
/***************************************************************************

    pc_vga.h

    PC standard VGA adaptor

***************************************************************************/

#ifndef MAME_VIDEO_PC_VGA_H
#define MAME_VIDEO_PC_VGA_H

#include "screen.h"

MACHINE_CONFIG_EXTERN( pcvideo_vga );
MACHINE_CONFIG_EXTERN( pcvideo_trident_vga );
MACHINE_CONFIG_EXTERN( pcvideo_gamtor_vga );
MACHINE_CONFIG_EXTERN( pcvideo_s3_vga );

// ======================> vga_device

class vga_device : public device_t
{
	friend class ibm8514a_device;

public:
	// construction/destruction
	vga_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual void zero();
	virtual uint32_t screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);

	virtual READ8_MEMBER(port_03b0_r);
	virtual WRITE8_MEMBER(port_03b0_w);
	virtual READ8_MEMBER(port_03c0_r);
	virtual WRITE8_MEMBER(port_03c0_w);
	virtual READ8_MEMBER(port_03d0_r);
	virtual WRITE8_MEMBER(port_03d0_w);
	virtual READ8_MEMBER(mem_r);
	virtual WRITE8_MEMBER(mem_w);
	virtual READ8_MEMBER(mem_linear_r);
	virtual WRITE8_MEMBER(mem_linear_w);
	virtual TIMER_CALLBACK_MEMBER(vblank_timer_cb);

protected:
	enum
	{
		SCREEN_OFF = 0,
		TEXT_MODE,
		VGA_MODE,
		EGA_MODE,
		CGA_MODE,
		MONO_MODE,
		RGB8_MODE,
		RGB15_MODE,
		RGB16_MODE,
		RGB24_MODE,
		RGB32_MODE
	};

	vga_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	void vga_vh_text(bitmap_rgb32 &bitmap, const rectangle &cliprect);
	void vga_vh_ega(bitmap_rgb32 &bitmap,  const rectangle &cliprect);
	void vga_vh_vga(bitmap_rgb32 &bitmap, const rectangle &cliprect);
	void vga_vh_cga(bitmap_rgb32 &bitmap, const rectangle &cliprect);
	void vga_vh_mono(bitmap_rgb32 &bitmap, const rectangle &cliprect);
	virtual uint8_t pc_vga_choosevideomode();
	void recompute_params_clock(int divisor, int xtal);
	uint8_t crtc_reg_read(uint8_t index);
	virtual void recompute_params();
	void crtc_reg_write(uint8_t index, uint8_t data);
	void seq_reg_write(uint8_t index, uint8_t data);
	uint8_t vga_vblank();
	READ8_MEMBER(vga_crtc_r);
	WRITE8_MEMBER(vga_crtc_w);
	uint8_t gc_reg_read(uint8_t index);
	void attribute_reg_write(uint8_t index, uint8_t data);
	void gc_reg_write(uint8_t index,uint8_t data);
	virtual uint16_t offset();
	inline uint8_t vga_latch_write(int offs, uint8_t data);
	inline uint8_t rotate_right(uint8_t val) { return (val >> vga.gc.rotate_count) | (val << (8 - vga.gc.rotate_count)); }
	inline uint8_t vga_logical_op(uint8_t data, uint8_t plane, uint8_t mask)
	{
		uint8_t res = 0;

		switch (vga.gc.logical_op & 3)
		{
		case 0: /* NONE */
			res = (data & mask) | (vga.gc.latch[plane] & ~mask);
			break;
		case 1: /* AND */
			res = (data | ~mask) & (vga.gc.latch[plane]);
			break;
		case 2: /* OR */
			res = (data & mask) | (vga.gc.latch[plane]);
			break;
		case 3: /* XOR */
			res = (data & mask) ^ (vga.gc.latch[plane]);
			break;
		}

		return res;
	}


	struct
	{
		read8_delegate read_dipswitch;
		struct
		{
			size_t vram_size;
			int seq_regcount;
			int crtc_regcount;
		} svga_intf;

		std::vector<uint8_t> memory;
		uint32_t pens[16]; /* the current 16 pens */

		uint8_t miscellaneous_output;
		uint8_t feature_control;

		struct
		{
			uint8_t index;
			uint8_t data[0x100];
			uint8_t map_mask;
			struct
			{
				uint8_t A, B;
			}char_sel;
		} sequencer;

		/* An empty comment at the start of the line indicates that register is currently unused */
		struct
		{
			uint8_t index;
			uint8_t data[0x100];
			uint16_t horz_total;
			uint16_t horz_disp_end;
	/**/    uint8_t horz_blank_start;
	/**/    uint8_t horz_blank_end;
	/**/    uint8_t horz_retrace_start;
	/**/    uint8_t horz_retrace_skew;
	/**/    uint8_t horz_retrace_end;
	/**/    uint8_t disp_enable_skew;
	/**/    uint8_t evra;
			uint16_t vert_total;
			uint16_t vert_disp_end;
	/**/    uint16_t vert_retrace_start;
	/**/    uint8_t vert_retrace_end;
			uint16_t vert_blank_start;
			uint16_t line_compare;
			uint32_t cursor_addr;
	/**/    uint8_t byte_panning;
			uint8_t preset_row_scan;
			uint8_t scan_doubling;
			uint8_t maximum_scan_line;
			uint8_t cursor_enable;
			uint8_t cursor_scan_start;
	/**/    uint8_t cursor_skew;
			uint8_t cursor_scan_end;
			uint32_t start_addr;
			uint32_t start_addr_latch;
			uint8_t protect_enable;
	/**/    uint8_t bandwidth;
			uint16_t offset;
			uint8_t word_mode;
			uint8_t dw;
	/**/    uint8_t div4;
	/**/    uint8_t underline_loc;
			uint16_t vert_blank_end;
			uint8_t sync_en;
	/**/    uint8_t aw;
			uint8_t div2;
	/**/    uint8_t sldiv;
	/**/    uint8_t map14;
	/**/    uint8_t map13;
	/**/    uint8_t irq_clear;
	/**/    uint8_t irq_disable;
		} crtc;

		struct
		{
			uint8_t index;
			uint8_t latch[4];
			uint8_t set_reset;
			uint8_t enable_set_reset;
			uint8_t color_compare;
			uint8_t logical_op;
			uint8_t rotate_count;
			uint8_t shift256;
			uint8_t shift_reg;
			uint8_t read_map_sel;
			uint8_t read_mode;
			uint8_t write_mode;
			uint8_t color_dont_care;
			uint8_t bit_mask;
			uint8_t alpha_dis;
			uint8_t memory_map_sel;
			uint8_t host_oe;
			uint8_t chain_oe;
		} gc;

		struct
		{
			uint8_t index, data[0x15]; int state;
			uint8_t prot_bit;
			uint8_t pel_shift;
			uint8_t pel_shift_latch;
		} attribute;

		struct {
			uint8_t read_index, write_index, mask;
			int read;
			int state;
			uint8_t color[0x300]; /* flat RGB triplets */
			int dirty;
		} dac;

		struct {
			uint8_t visible;
		} cursor;

		/* oak vga */
		struct { uint8_t reg; } oak;
	} vga;

	emu_timer *m_vblank_timer;
	required_device<palette_device> m_palette;
	required_device<screen_device> m_screen;
};


// device type definition
DECLARE_DEVICE_TYPE(VGA, vga_device)

// ======================> svga_device

class svga_device :  public vga_device
{
public:
	virtual void zero() override;
	virtual uint32_t screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect) override;

protected:
	// construction/destruction
	svga_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	void svga_vh_rgb8(bitmap_rgb32 &bitmap, const rectangle &cliprect);
	void svga_vh_rgb15(bitmap_rgb32 &bitmap, const rectangle &cliprect);
	void svga_vh_rgb16(bitmap_rgb32 &bitmap, const rectangle &cliprect);
	void svga_vh_rgb24(bitmap_rgb32 &bitmap, const rectangle &cliprect);
	void svga_vh_rgb32(bitmap_rgb32 &bitmap, const rectangle &cliprect);
	virtual uint8_t pc_vga_choosevideomode() override;
	virtual void device_start() override;
	struct
	{
		uint8_t bank_r,bank_w;
		uint8_t rgb8_en;
		uint8_t rgb15_en;
		uint8_t rgb16_en;
		uint8_t rgb24_en;
		uint8_t rgb32_en;
		uint8_t id;
	} svga;
};

// ======================> ibm8514_device

class ibm8514a_device : public device_t
{
public:
	ibm8514a_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	void set_vga(const char* tag) { m_vga_tag.assign(tag); }
	void set_vga_owner() { m_vga = dynamic_cast<vga_device*>(owner()); }

	void enabled();

	READ16_MEMBER(ibm8514_gpstatus_r);
	WRITE16_MEMBER(ibm8514_cmd_w);
	READ16_MEMBER(ibm8514_line_error_r);
	WRITE16_MEMBER(ibm8514_line_error_w);
	READ16_MEMBER(ibm8514_status_r);
	WRITE16_MEMBER(ibm8514_htotal_w);
	READ16_MEMBER(ibm8514_substatus_r);
	WRITE16_MEMBER(ibm8514_subcontrol_w);
	READ16_MEMBER(ibm8514_subcontrol_r);
	READ16_MEMBER(ibm8514_htotal_r);
	READ16_MEMBER(ibm8514_vtotal_r);
	WRITE16_MEMBER(ibm8514_vtotal_w);
	READ16_MEMBER(ibm8514_vdisp_r);
	WRITE16_MEMBER(ibm8514_vdisp_w);
	READ16_MEMBER(ibm8514_vsync_r);
	WRITE16_MEMBER(ibm8514_vsync_w);
	READ16_MEMBER(ibm8514_desty_r);
	WRITE16_MEMBER(ibm8514_desty_w);
	READ16_MEMBER(ibm8514_destx_r);
	WRITE16_MEMBER(ibm8514_destx_w);
	READ16_MEMBER(ibm8514_ssv_r);
	WRITE16_MEMBER(ibm8514_ssv_w);
	READ16_MEMBER(ibm8514_currentx_r);
	WRITE16_MEMBER(ibm8514_currentx_w);
	READ16_MEMBER(ibm8514_currenty_r);
	WRITE16_MEMBER(ibm8514_currenty_w);
	READ16_MEMBER(ibm8514_width_r);
	WRITE16_MEMBER(ibm8514_width_w);
	READ16_MEMBER(ibm8514_fgcolour_r);
	WRITE16_MEMBER(ibm8514_fgcolour_w);
	READ16_MEMBER(ibm8514_bgcolour_r);
	WRITE16_MEMBER(ibm8514_bgcolour_w);
	READ16_MEMBER(ibm8514_multifunc_r);
	WRITE16_MEMBER(ibm8514_multifunc_w);
	READ16_MEMBER(ibm8514_backmix_r);
	WRITE16_MEMBER(ibm8514_backmix_w);
	READ16_MEMBER(ibm8514_foremix_r);
	WRITE16_MEMBER(ibm8514_foremix_w);
	READ16_MEMBER(ibm8514_pixel_xfer_r);
	WRITE16_MEMBER(ibm8514_pixel_xfer_w);
	READ16_MEMBER(ibm8514_read_mask_r);
	WRITE16_MEMBER(ibm8514_read_mask_w);
	READ16_MEMBER(ibm8514_write_mask_r);
	WRITE16_MEMBER(ibm8514_write_mask_w);
	void ibm8514_wait_draw();
	struct
	{
		uint16_t htotal;  // Horizontal total (9 bits)
		uint16_t vtotal;  // Vertical total adjust (3 bits), Vertical total base (9 bit)
		uint16_t vdisp;
		uint16_t vsync;
		uint16_t subctrl;
		uint16_t substatus;
		uint16_t ssv;
		uint16_t ec0;
		uint16_t ec1;
		uint16_t ec2;
		uint16_t ec3;
		bool gpbusy;
		bool data_avail;
		int16_t dest_x;
		int16_t dest_y;
		int16_t curr_x;
		int16_t curr_y;
		int16_t prev_x;
		int16_t prev_y;
		int16_t line_axial_step;
		int16_t line_diagonal_step;
		int16_t line_errorterm;
		uint16_t current_cmd;
		uint16_t src_x;
		uint16_t src_y;
		int16_t scissors_left;
		int16_t scissors_right;
		int16_t scissors_top;
		int16_t scissors_bottom;
		uint16_t rect_width;
		uint16_t rect_height;
		uint32_t fgcolour;
		uint32_t bgcolour;
		uint16_t fgmix;
		uint16_t bgmix;
		uint32_t pixel_xfer;
		uint16_t pixel_control;
		uint8_t bus_size;
		uint8_t multifunc_sel;
		uint16_t multifunc_misc;
		uint32_t read_mask;
		uint32_t write_mask;

		int state;
		uint8_t wait_vector_len;
		uint8_t wait_vector_dir;
		bool wait_vector_draw;
		uint8_t wait_vector_count;

	} ibm8514;

protected:
	ibm8514a_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	virtual void device_start() override;
	virtual void device_config_complete() override;
	vga_device* m_vga;  // for pass-through
	std::string m_vga_tag;  // pass-through device tag
private:
	void ibm8514_draw_vector(uint8_t len, uint8_t dir, bool draw);
	void ibm8514_wait_draw_ssv();
	void ibm8514_draw_ssv(uint8_t data);
	void ibm8514_wait_draw_vector();
	void ibm8514_write_fg(uint32_t offset);
	void ibm8514_write_bg(uint32_t offset);
	void ibm8514_write(uint32_t offset, uint32_t src);

	//uint8_t* m_vram;  // the original 8514/A has it's own VRAM, but most VGA+8514 combination cards will have
					// only one set of VRAM, so this will only be needed in standalone 8514/A cards
	//uint32_t m_vramsize;
};

// device type definition
DECLARE_DEVICE_TYPE(IBM8514A, ibm8514a_device)

#define MCFG_8514A_ADD(_tag, _param) \
		MCFG_DEVICE_ADD(_tag, IBM8514A, 0) \
		downcast<ibm8514a_device*>(device)->set_vga(_param);

#define MCFG_8514A_ADD_OWNER(_tag) \
		MCFG_DEVICE_ADD(_tag, IBM8514A, 0) \
		downcast<ibm8514a_device*>(device)->set_vga_owner();


class mach8_device : public ibm8514a_device
{
public:
	mach8_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	READ16_MEMBER(mach8_ec0_r);
	WRITE16_MEMBER(mach8_ec0_w);
	READ16_MEMBER(mach8_ec1_r);
	WRITE16_MEMBER(mach8_ec1_w);
	READ16_MEMBER(mach8_ec2_r);
	WRITE16_MEMBER(mach8_ec2_w);
	READ16_MEMBER(mach8_ec3_r);
	WRITE16_MEMBER(mach8_ec3_w);
	READ16_MEMBER(mach8_ext_fifo_r);
	WRITE16_MEMBER(mach8_linedraw_index_w);
	READ16_MEMBER(mach8_bresenham_count_r);
	WRITE16_MEMBER(mach8_bresenham_count_w);
	WRITE16_MEMBER(mach8_linedraw_w);
	READ16_MEMBER(mach8_linedraw_r);
	READ16_MEMBER(mach8_scratch0_r);
	WRITE16_MEMBER(mach8_scratch0_w);
	READ16_MEMBER(mach8_scratch1_r);
	WRITE16_MEMBER(mach8_scratch1_w);
	READ16_MEMBER(mach8_config1_r);
	READ16_MEMBER(mach8_config2_r);
	READ16_MEMBER(mach8_sourcex_r);
	READ16_MEMBER(mach8_sourcey_r);
	WRITE16_MEMBER(mach8_ext_leftscissor_w);
	WRITE16_MEMBER(mach8_ext_topscissor_w);
	READ16_MEMBER(mach8_clksel_r) { return mach8.clksel; }

protected:
	mach8_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);
	virtual void device_start() override;
	struct
	{
		uint16_t scratch0;
		uint16_t scratch1;
		uint16_t linedraw;
		uint16_t clksel;
	} mach8;
};

// device type definition
DECLARE_DEVICE_TYPE(MACH8, mach8_device)

#define MCFG_MACH8_ADD(_tag, _param) \
		MCFG_DEVICE_ADD(_tag, MACH8, 0) \
		downcast<mach8_device*>(device)->set_vga(_param);

#define MCFG_MACH8_ADD_OWNER(_tag) \
		MCFG_DEVICE_ADD(_tag, MACH8, 0) \
		downcast<mach8_device*>(device)->set_vga_owner();

// ======================> tseng_vga_device

class tseng_vga_device :  public svga_device
{
public:
	// construction/destruction
	tseng_vga_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual READ8_MEMBER(port_03b0_r) override;
	virtual WRITE8_MEMBER(port_03b0_w) override;
	virtual READ8_MEMBER(port_03c0_r) override;
	virtual WRITE8_MEMBER(port_03c0_w) override;
	virtual READ8_MEMBER(port_03d0_r) override;
	virtual WRITE8_MEMBER(port_03d0_w) override;
	virtual READ8_MEMBER(mem_r) override;
	virtual WRITE8_MEMBER(mem_w) override;

protected:
	virtual void device_start() override;

private:
	void tseng_define_video_mode();
	uint8_t tseng_crtc_reg_read(uint8_t index);
	void tseng_crtc_reg_write(uint8_t index, uint8_t data);
	uint8_t tseng_seq_reg_read(uint8_t index);
	void tseng_seq_reg_write(uint8_t index, uint8_t data);
	void tseng_attribute_reg_write(uint8_t index, uint8_t data);

	struct
	{
		uint8_t reg_3d8;
		uint8_t dac_ctrl;
		uint8_t dac_state;
		uint8_t horz_overflow;
		uint8_t aux_ctrl;
		bool ext_reg_ena;
		uint8_t misc1;
		uint8_t misc2;
	}et4k;

};


// device type definition
DECLARE_DEVICE_TYPE(TSENG_VGA, tseng_vga_device)


// ======================> ati_vga_device

class ati_vga_device : public svga_device
{
public:
	// construction/destruction
	ati_vga_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual READ8_MEMBER(mem_r) override;
	virtual WRITE8_MEMBER(mem_w) override;

	// VGA registers
	virtual READ8_MEMBER(port_03c0_r) override;
	READ8_MEMBER(ati_port_ext_r);
	WRITE8_MEMBER(ati_port_ext_w);

	virtual uint16_t offset() override;

	mach8_device* get_8514() { return m_8514; }
protected:
	ati_vga_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	virtual void device_start() override;
	virtual void device_add_mconfig(machine_config &config) override;

private:
	void ati_define_video_mode();
	struct
	{
		uint8_t ext_reg[64];
		uint8_t ext_reg_select;
		uint8_t vga_chip_id;
	} ati;
	mach8_device* m_8514;
};

// device type definition
DECLARE_DEVICE_TYPE(ATI_VGA, ati_vga_device)


// ======================> s3_vga_device

class s3_vga_device : public ati_vga_device
{
public:
	// construction/destruction
	s3_vga_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual READ8_MEMBER(port_03b0_r) override;
	virtual WRITE8_MEMBER(port_03b0_w) override;
	virtual READ8_MEMBER(port_03c0_r) override;
	virtual WRITE8_MEMBER(port_03c0_w) override;
	virtual READ8_MEMBER(port_03d0_r) override;
	virtual WRITE8_MEMBER(port_03d0_w) override;
	virtual READ8_MEMBER(mem_r) override;
	virtual WRITE8_MEMBER(mem_w) override;

	virtual uint32_t screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect) override;

	virtual TIMER_CALLBACK_MEMBER(vblank_timer_cb) override;

	ibm8514a_device* get_8514() { return m_8514; }

protected:
	s3_vga_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_add_mconfig(machine_config &config) override;

	struct
	{
		uint8_t memory_config;
		uint8_t ext_misc_ctrl_2;
		uint8_t crt_reg_lock;
		uint8_t reg_lock1;
		uint8_t reg_lock2;
		uint8_t enable_8514;
		uint8_t enable_s3d;
		uint8_t cr3a;
		uint8_t cr42;
		uint8_t cr43;
		uint8_t cr53;
		uint8_t id_high;
		uint8_t id_low;
		uint8_t revision;
		uint8_t id_cr30;
		uint32_t strapping;  // power-on strapping bits
		uint8_t sr10;   // MCLK PLL
		uint8_t sr11;   // MCLK PLL
		uint8_t sr12;   // DCLK PLL
		uint8_t sr13;   // DCLK PLL
		uint8_t sr15;   // CLKSYN control 2
		uint8_t sr17;   // CLKSYN test
		uint8_t clk_pll_r;  // individual DCLK PLL values
		uint8_t clk_pll_m;
		uint8_t clk_pll_n;

		// data for memory-mapped I/O
		uint16_t mmio_9ae8;
		uint16_t mmio_bee8;
		uint16_t mmio_96e8;

		// hardware graphics cursor
		uint8_t cursor_mode;
		uint16_t cursor_x;
		uint16_t cursor_y;
		uint16_t cursor_start_addr;
		uint8_t cursor_pattern_x;  // cursor pattern origin
		uint8_t cursor_pattern_y;
		uint8_t cursor_fg[4];
		uint8_t cursor_bg[4];
		uint8_t cursor_fg_ptr;
		uint8_t cursor_bg_ptr;
		uint8_t extended_dac_ctrl;
	} s3;
	virtual uint16_t offset() override;

private:
	uint8_t s3_crtc_reg_read(uint8_t index);
	void s3_define_video_mode(void);
	void s3_crtc_reg_write(uint8_t index, uint8_t data);
	uint8_t s3_seq_reg_read(uint8_t index);
	void s3_seq_reg_write(uint8_t index, uint8_t data);
	ibm8514a_device* m_8514;
};

// device type definition
DECLARE_DEVICE_TYPE(S3_VGA, s3_vga_device)

// ======================> gamtor_vga_device

class gamtor_vga_device :  public svga_device
{
public:
	// construction/destruction
	gamtor_vga_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);


	virtual READ8_MEMBER(port_03b0_r) override;
	virtual WRITE8_MEMBER(port_03b0_w) override;
	virtual READ8_MEMBER(port_03c0_r) override;
	virtual WRITE8_MEMBER(port_03c0_w) override;
	virtual READ8_MEMBER(port_03d0_r) override;
	virtual WRITE8_MEMBER(port_03d0_w) override;
	virtual READ8_MEMBER(mem_r) override;
	virtual WRITE8_MEMBER(mem_w) override;
};


// device type definition
DECLARE_DEVICE_TYPE(GAMTOR_VGA, gamtor_vga_device)

/*
  pega notes (paradise)
  build in amstrad pc1640

  ROM_LOAD("40100", 0xc0000, 0x8000, CRC(d2d1f1ae))

  4 additional dipswitches
  seems to have emulation modes at register level
  (mda/hgc lines bit 8 not identical to ega/vga)

  standard ega/vga dipswitches
  00000000  320x200
  00000001  640x200 hanging
  00000010  640x200 hanging
  00000011  640x200 hanging

  00000100  640x350 hanging
  00000101  640x350 hanging EGA mono
  00000110  320x200
  00000111  640x200

  00001000  640x200
  00001001  640x200
  00001010  720x350 partial visible
  00001011  720x350 partial visible

  00001100  320x200
  00001101  320x200
  00001110  320x200
  00001111  320x200

*/

/*
  oak vga (oti 037 chip)
  (below bios patch needed for running)

  ROM_LOAD("oakvga.bin", 0xc0000, 0x8000, CRC(318c5f43))
*/


#endif // MAME_VIDEO_PC_VGA_H
