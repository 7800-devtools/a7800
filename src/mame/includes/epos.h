// license:BSD-3-Clause
// copyright-holders:Zsolt Vasvari
/*************************************************************************

    Epos games

**************************************************************************/

class epos_state : public driver_device
{
public:
	epos_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_videoram(*this, "videoram"),
		m_inputs(*this, { "INPUTS", "INPUTS2" }),
		m_maincpu(*this, "maincpu"),
		m_palette(*this, "palette")
		{ }

	/* memory pointers */
	required_shared_ptr<uint8_t> m_videoram;
	optional_ioport_array<2> m_inputs;

	/* video-related */
	uint8_t    m_palette_bank;

	/* misc */
	int      m_counter;
	int      m_input_multiplex;
	bool     m_ay_porta_multiplex;
	DECLARE_WRITE8_MEMBER(dealer_decrypt_rom);
	DECLARE_WRITE8_MEMBER(port_1_w);
	DECLARE_READ8_MEMBER(i8255_porta_r);
	DECLARE_WRITE8_MEMBER(i8255_portc_w);
	DECLARE_READ8_MEMBER(ay_porta_mpx_r);
	DECLARE_WRITE8_MEMBER(flip_screen_w);
	DECLARE_WRITE8_MEMBER(dealer_pal_w);
	DECLARE_DRIVER_INIT(dealer);
	virtual void machine_reset() override;
	DECLARE_MACHINE_START(epos);
	DECLARE_MACHINE_START(dealer);
	DECLARE_PALETTE_INIT(epos);
	uint32_t screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);
	void set_pal_color( uint8_t offset, uint8_t data );
	required_device<cpu_device> m_maincpu;
	required_device<palette_device> m_palette;
};
