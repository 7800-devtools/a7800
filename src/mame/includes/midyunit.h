// license:BSD-3-Clause
// copyright-holders:Alex Pasadyn, Zsolt Vasvari, Ernesto Corvi, Aaron Giles
// thanks-to:Kurt Mahan
/*************************************************************************

    Williams/Midway Y/Z-unit system

**************************************************************************/

#include "audio/williams.h"

#include "cpu/tms34010/tms34010.h"
#include "machine/gen_latch.h"
#include "machine/gen_latch.h"
#include "machine/nvram.h"
#include "sound/okim6295.h"

/* protection data types */
struct protection_data
{
	uint16_t  reset_sequence[3];
	uint16_t  data_sequence[100];
};

struct dma_state_t
{
	uint32_t      offset;         /* source offset, in bits */
	int32_t       rowbytes;       /* source bytes to skip each row */
	int32_t       xpos;           /* x position, clipped */
	int32_t       ypos;           /* y position, clipped */
	int32_t       width;          /* horizontal pixel count */
	int32_t       height;         /* vertical pixel count */
	uint16_t      palette;        /* palette base */
	uint16_t      color;          /* current foreground color with palette */
};


class midyunit_state : public driver_device
{
public:
	enum
	{
		TIMER_DMA,
		TIMER_AUTOERASE_LINE
	};

	midyunit_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_audiocpu(*this, "audiocpu")
		, m_oki(*this, "oki")
		, m_palette(*this, "palette")
		, m_narc_sound(*this, "narcsnd")
		, m_cvsd_sound(*this, "cvsd")
		, m_adpcm_sound(*this, "adpcm")
		, m_soundlatch(*this, "soundlatch")
		, m_generic_paletteram_16(*this, "paletteram")
		, m_gfx_rom(*this, "gfx_rom", 16)
		, m_mainram(*this, "mainram")
		, m_ports(*this, { { "IN0", "IN1", "IN2", "DSW", "UNK0", "UNK1" } })
	{
	}


	required_device<cpu_device> m_maincpu;
	optional_device<cpu_device> m_audiocpu;
	optional_device<okim6295_device> m_oki;
	required_device<palette_device> m_palette;
	optional_device<williams_narc_sound_device> m_narc_sound;
	optional_device<williams_cvsd_sound_device> m_cvsd_sound;
	optional_device<williams_adpcm_sound_device> m_adpcm_sound;
	optional_device<generic_latch_8_device> m_soundlatch;

	required_shared_ptr<uint16_t> m_generic_paletteram_16;
	optional_shared_ptr<uint8_t> m_gfx_rom;
	required_shared_ptr<uint16_t> m_mainram;
	optional_ioport_array<6> m_ports;

	std::unique_ptr<uint16_t[]> m_cmos_ram;
	uint32_t m_cmos_page;
	uint16_t m_prot_result;
	uint16_t m_prot_sequence[3];
	uint8_t m_prot_index;
	uint8_t m_term2_analog_select;
	const struct protection_data *m_prot_data;
	uint8_t m_cmos_w_enable;
	uint8_t m_chip_type;
	uint16_t *m_t2_hack_mem;
	uint8_t *m_cvsd_protection_base;
	uint8_t m_autoerase_enable;
	uint32_t m_palette_mask;
	std::unique_ptr<pen_t[]> m_pen_map;
	std::unique_ptr<uint16_t[]>   m_local_videoram;
	uint8_t m_videobank_select;
	uint8_t m_yawdim_dma;
	uint16_t m_dma_register[16];
	dma_state_t m_dma_state;
	emu_timer *m_dma_timer;
	emu_timer *m_autoerase_line_timer;
	DECLARE_WRITE16_MEMBER(midyunit_cmos_w);
	DECLARE_READ16_MEMBER(midyunit_cmos_r);
	DECLARE_WRITE16_MEMBER(midyunit_cmos_enable_w);
	DECLARE_READ16_MEMBER(midyunit_protection_r);
	DECLARE_READ16_MEMBER(midyunit_input_r);
	DECLARE_WRITE16_MEMBER(midyunit_sound_w);
	DECLARE_READ16_MEMBER(term2_input_r);
	DECLARE_WRITE16_MEMBER(term2_sound_w);
	DECLARE_WRITE16_MEMBER(term2_hack_w);
	DECLARE_WRITE16_MEMBER(term2la3_hack_w);
	DECLARE_WRITE16_MEMBER(term2la2_hack_w);
	DECLARE_WRITE16_MEMBER(term2la1_hack_w);
	DECLARE_WRITE8_MEMBER(cvsd_protection_w);
	DECLARE_READ16_MEMBER(mkturbo_prot_r);
	DECLARE_READ16_MEMBER(midyunit_gfxrom_r);
	DECLARE_WRITE16_MEMBER(midyunit_vram_w);
	DECLARE_READ16_MEMBER(midyunit_vram_r);
	DECLARE_WRITE16_MEMBER(midyunit_control_w);
	DECLARE_WRITE16_MEMBER(midyunit_paletteram_w);
	DECLARE_READ16_MEMBER(midyunit_dma_r);
	DECLARE_WRITE16_MEMBER(midyunit_dma_w);
	DECLARE_CUSTOM_INPUT_MEMBER(narc_talkback_strobe_r);
	DECLARE_CUSTOM_INPUT_MEMBER(narc_talkback_data_r);
	DECLARE_CUSTOM_INPUT_MEMBER(adpcm_irq_state_r);
	DECLARE_WRITE8_MEMBER(yawdim_oki_bank_w);
	TMS340X0_TO_SHIFTREG_CB_MEMBER(to_shiftreg);
	TMS340X0_FROM_SHIFTREG_CB_MEMBER(from_shiftreg);
	TMS340X0_SCANLINE_IND16_CB_MEMBER(scanline_update);
	DECLARE_DRIVER_INIT(smashtv);
	DECLARE_DRIVER_INIT(strkforc);
	DECLARE_DRIVER_INIT(narc);
	DECLARE_DRIVER_INIT(term2);
	DECLARE_DRIVER_INIT(term2la1);
	DECLARE_DRIVER_INIT(term2la3);
	DECLARE_DRIVER_INIT(mkyunit);
	DECLARE_DRIVER_INIT(trog);
	DECLARE_DRIVER_INIT(totcarn);
	DECLARE_DRIVER_INIT(mkyawdim);
	DECLARE_DRIVER_INIT(shimpact);
	DECLARE_DRIVER_INIT(hiimpact);
	DECLARE_DRIVER_INIT(mkyturbo);
	DECLARE_DRIVER_INIT(term2la2);
	DECLARE_MACHINE_RESET(midyunit);
	DECLARE_VIDEO_START(midzunit);
	DECLARE_VIDEO_START(midyunit_4bit);
	DECLARE_VIDEO_START(midyunit_6bit);
	DECLARE_VIDEO_START(mkyawdim);
	DECLARE_VIDEO_START(common);
	TIMER_CALLBACK_MEMBER(dma_callback);
	TIMER_CALLBACK_MEMBER(autoerase_line);

protected:
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;
	void dma_draw(uint16_t command);
	void init_generic(int bpp, int sound, int prot_start, int prot_end);
	void term2_init_common(write16_delegate hack_w);
};
