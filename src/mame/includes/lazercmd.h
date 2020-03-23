// license:GPL-2.0+
// copyright-holders:Juergen Buchmueller
#include "cpu/s2650/s2650.h"
#include "sound/dac.h"

#define HORZ_RES        32
#define VERT_RES        24
#define HORZ_CHR        8
#define VERT_CHR        10
#define VERT_FNT        8

#define HORZ_BAR        0x40
#define VERT_BAR        0x80

#define MARKER_ACTIVE_L 0x03
#define MARKER_ACTIVE_R 0x04
#define MARKER_VERT_R   0x0a
#define MARKER_HORZ_L   0x0b
#define MARKER_VERT_L   0x0c
#define MARKER_HORZ_R   0x0d

#define MARKER_HORZ_ADJ -1
#define MARKER_VERT_ADJ -10

class lazercmd_state : public driver_device
{
public:
	lazercmd_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_dac0(*this, "dac0"),
		m_dac1(*this, "dac1"),
		m_dac2(*this, "dac2"),
		m_dac3(*this, "dac3"),
		m_videoram(*this, "videoram"),
		m_gfxdecode(*this, "gfxdecode"),
		m_palette(*this, "palette")
	{ }

	/* device */
	required_device<s2650_device> m_maincpu;
	optional_device<dac_bit_interface> m_dac0;
	optional_device<dac_bit_interface> m_dac1;
	required_device<dac_bit_interface> m_dac2;
	required_device<dac_bit_interface> m_dac3;
	/* memory pointers */
	required_shared_ptr<uint8_t> m_videoram;

	required_device<gfxdecode_device> m_gfxdecode;
	required_device<palette_device> m_palette;

	/* video-related */
	uint8_t m_marker_x;
	uint8_t m_marker_y;

	/* misc */
	int m_timer_count;
	uint8_t m_sense_state;
	uint8_t m_attract;

	DECLARE_WRITE8_MEMBER(lazercmd_ctrl_port_w);
	DECLARE_READ8_MEMBER(lazercmd_ctrl_port_r);
	DECLARE_WRITE8_MEMBER(lazercmd_data_port_w);
	DECLARE_READ8_MEMBER(lazercmd_data_port_r);
	DECLARE_WRITE8_MEMBER(lazercmd_hardware_w);
	DECLARE_WRITE8_MEMBER(medlanes_hardware_w);
	DECLARE_WRITE8_MEMBER(bbonk_hardware_w);
	DECLARE_READ8_MEMBER(lazercmd_hardware_r);
	DECLARE_DRIVER_INIT(lazercmd);
	virtual void machine_start() override;
	virtual void machine_reset() override;
	DECLARE_PALETTE_INIT(lazercmd);
	uint32_t screen_update_lazercmd(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	TIMER_DEVICE_CALLBACK_MEMBER(lazercmd_timer);
	TIMER_DEVICE_CALLBACK_MEMBER(bbonk_timer);
	int vert_scale(int data);
	void plot_pattern( bitmap_ind16 &bitmap, int x, int y );
};
