// license:BSD-3-Clause
// copyright-holders:Dan Boris
/***************************************************************************

    Sun Electronics Arabian hardware

    driver by Dan Boris

***************************************************************************/

class arabian_state : public driver_device
{
public:
	arabian_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_custom_cpu_ram(*this, "custom_cpu_ram"),
		m_blitter(*this, "blitter"),
		m_maincpu(*this, "maincpu"),
		m_mcu(*this, "mcu"),
		m_palette(*this, "palette") { }

	/* memory pointers */
	required_shared_ptr<uint8_t> m_custom_cpu_ram;
	required_shared_ptr<uint8_t> m_blitter;

	std::unique_ptr<uint8_t[]>  m_main_bitmap;
	std::unique_ptr<uint8_t[]>  m_converted_gfx;

	/* video-related */
	uint8_t    m_video_control;
	uint8_t    m_flip_screen;

	/* MCU */
	uint8_t    m_mcu_port_o;
	uint8_t    m_mcu_port_p;
	uint8_t    m_mcu_port_r[4];
	DECLARE_READ8_MEMBER(mcu_port_r0_r);
	DECLARE_READ8_MEMBER(mcu_port_r1_r);
	DECLARE_READ8_MEMBER(mcu_port_r2_r);
	DECLARE_READ8_MEMBER(mcu_port_r3_r);
	DECLARE_WRITE8_MEMBER(mcu_port_r0_w);
	DECLARE_WRITE8_MEMBER(mcu_port_r1_w);
	DECLARE_WRITE8_MEMBER(mcu_port_r2_w);
	DECLARE_WRITE8_MEMBER(mcu_port_r3_w);
	DECLARE_READ8_MEMBER(mcu_portk_r);
	DECLARE_WRITE8_MEMBER(mcu_port_o_w);
	DECLARE_WRITE8_MEMBER(mcu_port_p_w);
	DECLARE_WRITE8_MEMBER(arabian_blitter_w);
	DECLARE_WRITE8_MEMBER(arabian_videoram_w);
	DECLARE_WRITE8_MEMBER(ay8910_porta_w);
	DECLARE_WRITE8_MEMBER(ay8910_portb_w);
	virtual void machine_start() override;
	virtual void machine_reset() override;
	virtual void video_start() override;
	DECLARE_PALETTE_INIT(arabian);
	uint32_t screen_update_arabian(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	void blit_area( uint8_t plane, uint16_t src, uint8_t x, uint8_t y, uint8_t sx, uint8_t sy );
	required_device<cpu_device> m_maincpu;
	required_device<cpu_device> m_mcu;
	required_device<palette_device> m_palette;
};
