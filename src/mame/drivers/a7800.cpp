// license:BSD-3-Clause
// copyright-holders:Dan Boris, Fabio Priuli, Mike Saarna, Robert Tuccitto
/***************************************************************************

  a7800.c

  Driver file to handle emulation of the Atari 7800.

  Dan Boris

    2002/05/13 kubecj   added more banks for bankswitching
                        added PAL machine description
                        changed clock to be precise
                        improved cart emulation (in machine/)

    2012/10/25 Robert Tuccitto  NTSC Color Generator utilized for
                color palette with hue shift/start
                based on observation of several
                systems across multiple displays

    2012/11/09 Robert Tuccitto  Fixed 3 degree hue begin point
                miscalculation of color palette

    2012/12/05 Robert Tuccitto  Implemented proper IRE and phase
               value to the palette

    2012/12/14 Robert Tuccitto  Adjusted colorburst/tint/hue of entire
               palette to closer reflect default hardware configuration
               setting of ~180 degrees.  Palette settings now correspond
               documented and calculated settings as follows:

               Contrast = 0.0526 --> 0.05
               Brightness = 0.0 --> 0.00
               Color = 0.2162 --> 0.22
               Phase = 25.714 --> 25.7
               Colorburst/Hue = 180 degrees

    2013/02/27 Robert Tuccitto  Palette rebuild due to misaligned
               data references.  Corrected PAL color sequence order.

    2013/03/19 Robert Tuccitto  Stripped palette to raw video output
               values removing YIQ/YUV infer data.

    2013/04/02 Robert Tuccitto  Corrected rotation values and errors
               including duplicate entries for palette.

    2013/04/07 Robert Tuccitto  Address map locations for the XBOARD
               added.

    2013/05/01 Robert Tuccitto  Red and Blue miscalculated proportions
               fixed.

    2013/08/04 Robert Tuccitto  Green miscalculated proportions fixed.

    2013/08/13 Robert Tuccitto  Normalized contrast and brightness,
               providing a standardize grayscale and adjusted color values.

    2013/09/02 Robert Tuccitto  Stored data for 26.7 & 27.7 phase shifts
               with corrections and label for 25.7 values. Made 26.7
               (medium) default. Phase shifting falls outside the realm of
               video controls and hope to implement a selectable toggle
               hardware option similar to Donkey Kong TKG02/TKG04.

    2013/09/19 Robert Tuccitto  Cleanup of Address Maps, high score maps
               added.

    2013/10/16 Robert Tuccitto  Added Phase Shifts 24.7, 25.2, 26.2, 27.2.
               Phase Shifts 24.7 through 27.7 degrees with 0.5 degree
               increments documented. Phase Shift 26.2 degrees made active.
               Fixed typo under 26.7 7$.

    2013/10/27 Robert Tuccitto  Modernized screen parameters for NTSC & PAL.

    2013/11/03 Robert Tuccitto  Fixed correctly typo under 26.7 7$.

    2013/11/23 Robert Tuccitto  Added NTSC Palette Notes.

    2014/01/02 Robert Tuccitto  Corrected joystick buttons assignment & minor
                                palette notes cleanup.

    2014/01/09 Robert Tuccitto  Positional description for difficulty
                                switches added.

    2014/02/15 Robert Tuccitto  Added more details and clarification
                                regarding the potentiometer.

    2014/03/25 Mike Saarna  Fixed Riot Timer

    2014/04/04 Mike Saarna  Fix to controller button RIOT behavior

    2014/05/06 Mike Saarna/Robert Tuccitto Brought initial Maria cycle counts
               inline from measurements taken with logic analyzer and tests.

    2014/08/25 Fabio Priuli Converted carts to be slot devices and cleaned
               up the driver (removed the pokey, cleaned up rom regions, etc.)

    2017/07/14 Mike Saarna/Robert Tuccito   Converted to slotted controls and
               added support for proline joysticks, vcs joysticks, paddles,
               lightguns, keypads, driving wheels, cx22 trakballs, amiga mice, 
               st mice/cx80. 

***************************************************************************/

#include "emu.h"
#include "cpu/m6502/m6502.h"
#include "sound/tiaintf.h"
#include "sound/tiasound.h"
#include "machine/6532riot.h"
#include "video/maria.h"
#include "bus/a7800_ctrl/ctrl.h"
#include "bus/a7800/a78_carts.h"
#include "screen.h"
#include "softlist.h"
#include "speaker.h"


#define A7800_NTSC_Y1   XTAL_14_31818MHz
#define CLK_PAL 1773447

// FIXME: global used to pass info between a7800 driver and bus devices
int m_dmaactive; 

class a7800_state : public driver_device
{
public:
	a7800_state(const machine_config &mconfig, device_type type, const char *tag)
	: driver_device(mconfig, type, tag),
	m_maincpu(*this, "maincpu"),
	m_tia(*this, "tia"),
	m_maria(*this, "maria"),
	m_riot(*this, "riot"),
	m_joy1(*this, "joy1"),
	m_joy2(*this, "joy2"),
	m_io_console_buttons(*this, "console_buttons"),
	m_cart(*this, "cartslot"),
	m_screen(*this, "screen"),
	m_bios(*this, "maincpu") { }


	int m_lines;
	int m_ispal;

	int m_ctrl_lock;
	int m_ctrl_reg;
	int m_maria_flag;
	int m_p1_one_button;
	int m_p2_one_button;
	int m_bios_enabled;


	DECLARE_READ8_MEMBER(bios_or_cart_r);
	DECLARE_READ8_MEMBER(tia_r);
	DECLARE_WRITE8_MEMBER(tia_w);
	DECLARE_DRIVER_INIT(a7800_ntsc);
	DECLARE_DRIVER_INIT(a7800u1_ntsc);
	DECLARE_DRIVER_INIT(a7800u2_ntsc);
	DECLARE_DRIVER_INIT(a7800_pal);
	DECLARE_DRIVER_INIT(a7800u1_pal);
	DECLARE_DRIVER_INIT(a7800u2_pal);
	virtual void machine_start() override;
	virtual void machine_reset() override;
	DECLARE_PALETTE_INIT(a7800);
	DECLARE_PALETTE_INIT(a7800u1);
	DECLARE_PALETTE_INIT(a7800u2);
	DECLARE_PALETTE_INIT(a7800p);
	DECLARE_PALETTE_INIT(a7800pu1);
	DECLARE_PALETTE_INIT(a7800pu2);
	TIMER_DEVICE_CALLBACK_MEMBER(interrupt);
	TIMER_CALLBACK_MEMBER(maria_startdma);
	DECLARE_READ8_MEMBER(riot_joystick_r);
	DECLARE_WRITE8_MEMBER(riot_joystick_w);
	DECLARE_READ8_MEMBER(riot_console_button_r);
	DECLARE_WRITE8_MEMBER(riot_button_pullup_w);


protected:
	required_device<cpu_device> m_maincpu;
	required_device<tia_device> m_tia;
	required_device<atari_maria_device> m_maria;
	required_device<riot6532_device> m_riot;
	required_device<a7800_control_port_device> m_joy1;
	required_device<a7800_control_port_device> m_joy2;
	required_ioport m_io_console_buttons;
	required_device<a78_cart_slot_device> m_cart;
	required_device<screen_device> m_screen;
	required_region_ptr<uint8_t> m_bios;
	uint64_t paddle_start;
	uint8_t  tia_delay;
};


/***************************************************************************
 MEMORY HANDLERS
 ***************************************************************************/

// RIOT
READ8_MEMBER(a7800_state::riot_joystick_r)
{
	uint8_t val = 0;


	/* Left controller port PINs 1-4 ( 4321 ) */
	val |= ( m_joy1->joy_r() & 0x0F ) << 4;

	/* Right controller port PINs 1-4 ( 4321 ) */
	val |= m_joy2->joy_r() & 0x0F;

	return val;
}

WRITE8_MEMBER(a7800_state::riot_joystick_w)
{

	/* Left controller port */
	m_joy1->joy_w( data >> 4 );

	/* Right controller port */
	m_joy2->joy_w( data & 0x0f );

}

READ8_MEMBER(a7800_state::riot_console_button_r)
{
	return m_io_console_buttons->read();
}

WRITE8_MEMBER(a7800_state::riot_button_pullup_w)
{
	if(m_maincpu->space(AS_PROGRAM).read_byte(0x283) & 0x04)
		m_p1_one_button = data & 0x04; // pin 6 of the controller port is held high by the riot chip when reading two-button controllers (from schematic)
	if(m_maincpu->space(AS_PROGRAM).read_byte(0x283) & 0x10)
		m_p2_one_button = data & 0x10;
}

READ8_MEMBER(a7800_state::tia_r)
{

	uint64_t elapsed; 
	int16_t cpu_x, cpu_y;
	int16_t gun_x, gun_y;

	/* For now we only apply the 0.5 cycle penalty to lightgun games, where it's required.
	   Other games (mostly paddle) may jitter from frame to frame, because we can't actually 
	   wait 0.5 cycles, and instead wait for 1 cycle every 2x TIA accesses. Without this fix
	   our 6502 on-screen position is severely skewed. */
	if ((m_joy1->is_lightgun())||(m_joy2->is_lightgun()))
	{
		tia_delay++;
		if(tia_delay==2)
		{
			tia_delay=0;
			//+3 gives us +1 cpu cycle, because we're partway through an instruction.
			m_maincpu->spin_until_time(m_maincpu->cycles_to_attotime(3));
		}
	}

	switch (offset & 0x0f)
	{
		case 0x00:
		case 0x01:
		case 0x02:
		case 0x03:
		case 0x04:
		case 0x05:
		case 0x06:
		case 0x07:
			/* Even though the 7800 doesn't use the TIA graphics the collision registers should
			   still return a reasonable value */
			return 0x00;
		case 0x08: //INPT0
			if ( m_joy1->is_paddle() )
			{
				/* Scale the elapsed cycles to approximately fit the digital paddle-scale. This
				   is always a bit different from game to game, so we'll invariably wind up with
				   some wasted digital paddle-range and an in-game non-centered center point. */
				elapsed = (machine().device<cpu_device>("maincpu")->total_cycles() - paddle_start)/97;
				if ( elapsed > (m_joy1->pot_x_r()^0xff) )
					return 0x80;
				else
					return 0x00;
			}
			if(m_joy1->has_pot_x())
				return m_joy1->pot_x_r();
			return ((m_joy1->joy_r() & 0x10)<<3);
		case 0x09: //INPT1
			if ( m_joy1->is_paddle() )
			{
				/* Scale the elapsed cycles to approximately fit the digital paddle-scale. This
				   is always a bit different from game to game, so we'll invariably wind up with
				   some wasted digital paddle-range and an in-game non-centered center point. */
				elapsed = (machine().device<cpu_device>("maincpu")->total_cycles() - paddle_start)/97;
				if ( elapsed > (m_joy1->pot_y_r()^0xff) )
					return 0x80;
				else
					return 0x00;
			}
			if(m_joy1->has_pot_y())
				return m_joy1->pot_y_r();
			return ( m_joy1->joy_r() & 0x40 ) ? 0x80 : 0x00;
		case 0x0a: //INPT2
			if ( m_joy2->is_paddle() )
			{
				/* Scale the elapsed cycles to approximately fit the digital paddle-scale. This
				   is always a bit different from game to game, so we'll invariably wind up with
				   some wasted digital paddle-range and an in-game non-centered center point. */
				elapsed = (machine().device<cpu_device>("maincpu")->total_cycles() - paddle_start)/97;
				if ( elapsed > (m_joy2->pot_x_r()^0xff) )
					return 0x80;
				else
					return 0x00;
			}
			if(m_joy2->has_pot_x())
				return m_joy2->pot_x_r();
			return ( m_joy2->joy_r() & 0x10 ) ? 0x80 : 0x00;
		case 0x0b: //INPT3
			if ( m_joy2->is_paddle() )
			{
				/* Scale the elapsed cycles to approximately fit the digital paddle-scale. This
				   is always a bit different from game to game, so we'll invariably wind up with
				   some wasted digital paddle-range and an in-game non-centered center point. */
				elapsed = (machine().device<cpu_device>("maincpu")->total_cycles() - paddle_start)/97;
				if ( elapsed > (m_joy2->pot_y_r()^0xff) )
					return 0x80;
				else
					return 0x00;
			}
			if(m_joy2->has_pot_y())
				return m_joy2->pot_y_r();
			return ( m_joy2->joy_r() & 0x40 ) ? 0x80 : 0x00;
		case 0x0c: //INPT4
			if ( m_joy1->has_pro_buttons() && m_p1_one_button && (m_joy1->joy_r() & 0x50) )
				return 0x00; // A and B buttons activate the vcs button line in 1 button mode
			if (m_joy1->is_lightgun())
			{
				/* To support 7800 lightguns, the 6502 races the beam 2600 style, checking
				   the INPT4 light sensor. If it's being checked, we need to find out
				   the current cpu position, and compare it to the GUI gunsight. If the
				   gunsight is nearby, we need to return a "lit-up" sensor value. */

				cpu_x=m_screen->hpos()/2;
				cpu_y=(m_screen->vpos() % m_lines); 

				// Scale the gun X coordinates to the screen. Offset and wrap the X.
				gun_x=((((m_joy1->light_x_r())*160)/255)+95)%227;

				// Scale the gun Y coordinates to the visible screen length. Offset the Y.
				if(m_ispal)
					gun_y=(((m_joy1->light_y_r())*260)/255)+24;
				else //ntsc
					gun_y=(((m_joy1->light_y_r())*228)/255)+16;

				/* Calculate the distance between the gunsight and beam. If it's within 8 pixels,
				   light up the sensor. We use 8 squared instead of 8 for the comparison, so that
				   we can skip the square root on the distance calculation. */
				if( (((gun_x-cpu_x)*(gun_x-cpu_x))+((gun_y-cpu_y)*(gun_y-cpu_y))) < 64 )
					return 0x00;
				else
					return 0x80;
			}
			else
				return ( m_joy1->joy_r() & 0x20 ) ? 0x80 : 0x00;

		case 0x0d: //INPT5
			if ( m_joy2->has_pro_buttons() && m_p2_one_button && (m_joy2->joy_r() & 0x50) )
				return 0x00; // A and B buttons activate the vcs button line in 1 button mode
			if (m_joy2->is_lightgun())
			{
				/* To support 7800 lightguns, the 6502 races the beam 2600 style, checking
				   the INPT5 light sensor. If it's being checked, we need to find out
				   the current cpu position, and compare it to the GUI gunsight. If the
				   gunsight is nearby, we need to return a "lit-up" sensor value. */

				cpu_x=m_screen->hpos()/2;
				cpu_y=(m_screen->vpos() % m_lines); 

				// Scale the gun X coordinates to the screen. Offset and wrap the X.
				gun_x=((((m_joy2->light_x_r())*160)/255)+95)%227;

				// Scale the gun Y coordinates to the visible screen length. Offset the Y.
				if(m_ispal)
					gun_y=(((m_joy2->light_y_r())*260)/255)+24;
				else //ntsc
					gun_y=(((m_joy2->light_y_r())*228)/255)+16;

				/* Calculate the distance between the gunsight and beam. If it's within 8 pixels,
				   light up the sensor. We use 8 squared instead of 8 for the comparison, so we 
				   can skip the square root on the distance calculation. */
				if( (((gun_x-cpu_x)*(gun_x-cpu_x))+((gun_y-cpu_y)*(gun_y-cpu_y))) < 64 )
					return 0x00;
				else
					return 0x80;
			}
			else
				return ( m_joy2->joy_r() & 0x20 ) ? 0x80 : 0x00;
		default:
			logerror("undefined TIA read %x\n",offset);

	}
	return 0xff;
}

// TIA
WRITE8_MEMBER(a7800_state::tia_w)
{
	static uint8_t paddle_is_grounded = 0;

	/* For now we only apply the 0.5 cycle penalty to lightgun games, where it's required.
	   Other games (mostly paddle) may jitter from frame to frame, because we can't actually 
	   wait 0.5 cycles, and instead wait for 1 cycle every 2x TIA accesses. Without this fix
	   our 6502 on-screen position is severely skewed. */
	if ((m_joy1->is_lightgun())||(m_joy2->is_lightgun()))
	{
		tia_delay++;
		if(tia_delay==2)
		{
			tia_delay=0;
			//+3 gives us +1 cpu cycle, because we're partway through an instruction.
			m_maincpu->spin_until_time(m_maincpu->cycles_to_attotime(3));
		}
	}

	if (offset < 0x20)
	{ //INPTCTRL covers TIA registers 0x00-0x1F until locked
		if (data & 0x01)
		{
			if (m_ctrl_lock && offset == 0x01)
				m_maria_flag = 1;
			else if (!m_ctrl_lock)
				m_maria_flag = 1;
		}
		if (!m_ctrl_lock)
		{
			m_ctrl_lock = data & 0x01;
			m_ctrl_reg = data;
		}
	}
	if(offset==1) // VBLANK
	{
		// paddle_start is when the paddles have been grounded and released from ground.
		if(data & 0x80) 
			paddle_is_grounded=1;
		else if (paddle_is_grounded)
		{
			paddle_is_grounded=0;
			paddle_start = machine().device<cpu_device>("maincpu")->total_cycles();
		}
	}
	else if ((m_ctrl_reg & 3)==3)
		m_tia->tia_sound_w(space, offset, data);

}


// TIMERS

TIMER_DEVICE_CALLBACK_MEMBER(a7800_state::interrupt)
{
	// DMA Begins 7 cycles after hblank
	machine().scheduler().timer_set(m_maincpu->cycles_to_attotime(7), timer_expired_delegate(FUNC(a7800_state::maria_startdma),this));
	m_maria->interrupt(m_lines);
}

TIMER_CALLBACK_MEMBER(a7800_state::maria_startdma)
{
	m_dmaactive = 1;
	m_maria->startdma(m_lines);
	m_dmaactive = 0;
}



// ROM
READ8_MEMBER(a7800_state::bios_or_cart_r)
{
	if (!(m_ctrl_reg & 0x04))
		return m_bios[offset];
	else
		return m_cart->read_40xx(space, offset + 0x8000);
}

/***************************************************************************
    ADDRESS MAPS
***************************************************************************/

static ADDRESS_MAP_START( a7800_mem, AS_PROGRAM, 8, a7800_state )
	AM_RANGE(0x0000, 0x001f) AM_MIRROR(0x300) AM_READWRITE(tia_r, tia_w)
	AM_RANGE(0x0020, 0x003f) AM_MIRROR(0x300) AM_DEVREADWRITE("maria", atari_maria_device, read, write)
	AM_RANGE(0x0040, 0x00ff) AM_RAMBANK("zpmirror") // mirror of 0x2040-0x20ff, for zero page
	AM_RANGE(0x0140, 0x01ff) AM_RAMBANK("spmirror") // mirror of 0x2140-0x21ff, for stack page
	AM_RANGE(0x0280, 0x029f) AM_MIRROR(0x160) AM_DEVREADWRITE("riot", riot6532_device, read, write)
	AM_RANGE(0x0480, 0x04ff) AM_MIRROR(0x100) AM_RAM AM_SHARE("riot_ram")
	AM_RANGE(0x1800, 0x1fff) AM_RAM AM_SHARE("6116_1")
	AM_RANGE(0x2000, 0x27ff) AM_RAM AM_SHARE("6116_2")
								// According to the official Software Guide, the RAM at 0x2000 is
								// repeatedly mirrored up to 0x3fff, but this is evidently incorrect
								// because the High Score Cartridge maps ROM at 0x3000-0x3fff
								// Hardware tests show that only the page at 0x2700 appears at
								// 0x2800, and only on some hardware (MARIA? motherboard?) revisions,
								// and even then with inconsistent and unreliable results.
	AM_RANGE(0x4000, 0xffff) AM_DEVWRITE("cartslot", a78_cart_slot_device, write_40xx)
	AM_RANGE(0x4000, 0xbfff) AM_DEVREAD("cartslot", a78_cart_slot_device, read_40xx)
	AM_RANGE(0xc000, 0xffff) AM_READ(bios_or_cart_r)    // here also the BIOS can be accessed
ADDRESS_MAP_END


/***************************************************************************
    INPUT PORTS
***************************************************************************/

static INPUT_PORTS_START( a7800 )

	PORT_START("console_buttons")
	//PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_OTHER)  PORT_NAME("Reset")         PORT_CODE(KEYCODE_R)
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_OTHER)  PORT_NAME("Reset")         PORT_CODE(KEYCODE_T)
	//PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_OTHER)  PORT_NAME("Select")        PORT_CODE(KEYCODE_S)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_OTHER)  PORT_NAME("Select")        PORT_CODE(KEYCODE_E)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_OTHER)  PORT_NAME(DEF_STR(Pause))  PORT_CODE(KEYCODE_O)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_DIPNAME(0x40, 0x40, "Left Difficulty Switch")
	PORT_DIPSETTING(0x40, "A - Right Position" )
	PORT_DIPSETTING(0x00, "B - Left Position" )
	PORT_DIPNAME(0x80, 0x80, "Right Difficulty Switch")
	PORT_DIPSETTING(0x80, "A - Right Position" )
	PORT_DIPSETTING(0x00, "B - Left Position" )
INPUT_PORTS_END

/***************************************************************************
Atari 7800 NTSC Palette Notes:

Palette on a modern flat panel display (LCD, LED, Plasma, etc.) appears
different from a traditional CRT. The most outstanding difference is Hue 1x,
the hue begin point. Hue 1x looks very 'green' (~-60 to -45 degrees -
depending on how poor or well it handles the signal conversion and its
calibration) on a modern flat panel display, as opposed to 'gold' (~-33
degrees) on a CRT.

The potentiometer (pot adjustment) for the 7800 modifies the delay line
regarding colors it will exhibit and is extremely sensitive.  The slightest
turn can have a significant impact.

A system whose potentiometer is not properly calibrated via
'CPS 7800 Diagnostic Test Cartridge' or/and just slightly off from the
desired factory settings may exhibit consequences such as too much blue in
place of green (Pot adjusted slightly too far left) or washed out reddish
tones in place of where most other systems display a darker reddish tones
(Pot adjusted slightly too far right).

This is a result of the phase shifting of lesser degrees (Pot adjusted more
to the left) or phase shifting of greater degrees (Pot adjusted more to the
right).

Turning the pot adjustment to the right, it can be observed that the values
of the higher end of the scale will match the lower end of the scale.
For example, after some turning to the right, the values of Dx, Ex, Fx,
can be set to match 1x, 2x, 3x.

After further turning to the right, now the palette can be brought to make
Ax, Bx, Cx, Dx, Ex, Fx will match 1x, 2x, 3x, 4x, 5x, 6x.

Ultimately though, too much turning to the right results in all colors being
wiped from the scale, excluding the hue begin point 1x (Which remains
unchanged while tweaking the potentiometer either left or right).

Continuously turning the pot adjustment to the left, red and blue become the
most dominant two colors encompassing the palette with only a slight
influence of green at the highest end of the palette (Fx), once turned all
the way leftward.

The degree range for adjustment of the phase shifting on the 7800 appears
to be as low as approximately 15 degrees when tuned all the way left, and
seems to be able to achieve as high as approximately 45 degrees when turned
right before losing all color (Excluding 1x) from the palette scale.

For even a properly calibrated system at power on, the system's phase
shift appears as low as ~23 degrees and after a considerable consistent
runtime ('warm-up'), can be as high as ~28 degrees.

In general, the low end of ~23 degrees lasts for maybe several seconds,
whereas higher values such as ~25-27 degrees is the most dominant during
system run time.  180 degrees colorburst takes place at ~25.7 degrees (A
near exact match of Hue 1x and 15x - To the naked eye they appear to be
the same).

However, consistent system run time causes Hue 15x (F$) to become
stronger/darker gold (More brown then ultimately red-brown); as well
as leans Hue 14x (E$) more brown than green.  Once achieving a phase shift
of 27.7, Hue 14x (E$) and Hue 15x (F$) near-exact match Hue 1x and 2x
respectively.

Therefore, an ideal phase shift while accounting for the reality of
shifting while warming up, as well as maintaining differences between 1x,
2x and 14x, 15x, would likely fall between a 25.7 and 27.7. Phase shifts
26.2 degrees and 26.7 degrees places Hue 15x (F$) between Hue 1x and
Hue 2x, having 26.2 degrees leaning closer to Hue 1x and 26.7 degrees
leaning closer to Hue 2x.

The above notion would also harmonize with what has been documented for
the colors of 1x, 2x, 14x, 15x on the 7800.  1x = Gold, 2x = Orange,
14x (E$) = Orange-Green. 15x (F$) = Light Orange.  Color descriptions are
best measured in the middle of the brightness scale.  It should be
mentioned that Green-Yellow is referenced at Hue 13x (D$), nowhere near
Hue 1x.  A Green-Yellow Hue 1x is how the palette is manipulated and
modified (in part) under a modern flat panel display.

Additionally, the blue to red (And consequently blue to green) ratio
proportions may appear different on a modern flat panel display than a CRT
in some instances for the Atari 7800 system.  Furthermore, you may have
some variation of proportions even within the same display type.

One side effect of this on the console's palette is that some values of
red may appear too pinkish - Too much blue to red.  This is not the same
as a traditional tint-hue control adjustment; rather, can be demonstrated
by changing the blue ratio values via MESS HLSL settings.

Lastly, the Atari 2600 & 5200 NTSC color palettes hold the same hue
structure order and have similar appearance differences that are dependent
upon display type.
***************************************************************************/

/***************************************************************************
    PALETTE - 25.7 PHASE SHIFT
***************************************************************************/

#define NTSC_HUE0_PAL_HUE0 \
    rgb_t(0x00,0x00,0x00), rgb_t(0x06,0x06,0x06), rgb_t(0x1C,0x1C,0x1C), rgb_t(0x2E,0x2E,0x2E), \
    rgb_t(0x40,0x40,0x40), rgb_t(0x53,0x53,0x53), rgb_t(0x69,0x69,0x69), rgb_t(0x7B,0x7B,0x7B), \
    rgb_t(0x84,0x84,0x84), rgb_t(0x96,0x96,0x96), rgb_t(0xAC,0xAC,0xAC), rgb_t(0xBF,0xBF,0xBF), \
    rgb_t(0xD1,0xD1,0xD1), rgb_t(0xE3,0xE3,0xE3), rgb_t(0xF9,0xF9,0xF9), rgb_t(0xFF,0xFF,0xFF   )

#define NTSC_HUE1_PAL_HUE2 \
	rgb_t(0x31,0x00,0x00), rgb_t(0x46,0x19,0x00), rgb_t(0x5C,0x34,0x00), rgb_t(0x6F,0x48,0x00), \
    rgb_t(0x81,0x5B,0x00), rgb_t(0x92,0x6E,0x00), rgb_t(0xA6,0x83,0x00), rgb_t(0xB7,0x94,0x00), \
    rgb_t(0xBF,0x9C,0x00), rgb_t(0xCF,0xAD,0x00), rgb_t(0xE2,0xC1,0x00), rgb_t(0xF1,0xD1,0x00), \
    rgb_t(0xFF,0xE1,0x00), rgb_t(0xFF,0xF0,0x10), rgb_t(0xFF,0xFF,0x2C), rgb_t(0xFF,0xFF,0x41   )

#define PAL_HUE1 \
	rgb_t(0x00,0x1A,0x00), rgb_t(0x00,0x30,0x00), rgb_t(0x1E,0x48,0x00), rgb_t(0x35,0x5C,0x00), \
	rgb_t(0x49,0x6E,0x00), rgb_t(0x5C,0x80,0x00), rgb_t(0x72,0x95,0x00), rgb_t(0x84,0xA6,0x00), \
	rgb_t(0x8C,0xAE,0x00), rgb_t(0x9D,0xBE,0x00), rgb_t(0xB1,0xD1,0x00), rgb_t(0xC1,0xE1,0x00), \
	rgb_t(0xD1,0xF1,0x00), rgb_t(0xE1,0xFF,0x04), rgb_t(0xF4,0xFF,0x23), rgb_t(0xFF,0xFF,0x39   )

#define NTSC_HUE2_PAL_HUE3 \
	rgb_t(0x63,0x00,0x00), rgb_t(0x75,0x00,0x00), rgb_t(0x8A,0x1A,0x00), rgb_t(0x9B,0x31,0x00), \
    rgb_t(0xAC,0x45,0x00), rgb_t(0xBD,0x59,0x00), rgb_t(0xD0,0x6F,0x00), rgb_t(0xE0,0x81,0x00), \
    rgb_t(0xE7,0x89,0x00), rgb_t(0xF7,0x9A,0x00), rgb_t(0xFF,0xAE,0x09), rgb_t(0xFF,0xBE,0x22), \
    rgb_t(0xFF,0xCF,0x38), rgb_t(0xFF,0xDE,0x4C), rgb_t(0xFF,0xF1,0x63), rgb_t(0xFF,0xFF,0x75   )

#define NTSC_HUE3_PAL_HUE4 \
	rgb_t(0x7E,0x00,0x00), rgb_t(0x90,0x00,0x00), rgb_t(0xA4,0x00,0x00), rgb_t(0xB5,0x19,0x00), \
    rgb_t(0xC5,0x30,0x00), rgb_t(0xD5,0x45,0x00), rgb_t(0xE8,0x5C,0x20), rgb_t(0xF7,0x6E,0x36), \
    rgb_t(0xFF,0x77,0x3F), rgb_t(0xFF,0x88,0x53), rgb_t(0xFF,0x9D,0x69), rgb_t(0xFF,0xAE,0x7B), \
    rgb_t(0xFF,0xBE,0x8D), rgb_t(0xFF,0xCE,0x9E), rgb_t(0xFF,0xE1,0xB2), rgb_t(0xFF,0xF1,0xC2   )

#define NTSC_HUE4_PAL_HUE5 \
	rgb_t(0x82,0x00,0x08), rgb_t(0x93,0x00,0x22), rgb_t(0xA7,0x00,0x3B), rgb_t(0xB8,0x07,0x4F), \
	rgb_t(0xC8,0x21,0x62), rgb_t(0xD8,0x37,0x74), rgb_t(0xEB,0x4E,0x89), rgb_t(0xFA,0x61,0x9A), \
	rgb_t(0xFF,0x6A,0xA3), rgb_t(0xFF,0x7C,0xB3), rgb_t(0xFF,0x91,0xC6), rgb_t(0xFF,0xA2,0xD7), \
	rgb_t(0xFF,0xB2,0xE6), rgb_t(0xFF,0xC3,0xF6), rgb_t(0xFF,0xD6,0xFF), rgb_t(0xFF,0xE6,0xFF   )

#define NTSC_HUE5_PAL_HUE6 \
	rgb_t(0x6D,0x00,0x76), rgb_t(0x7F,0x00,0x88), rgb_t(0x94,0x00,0x9C), rgb_t(0xA5,0x00,0xAD), \
	rgb_t(0xB5,0x1B,0xBD), rgb_t(0xC5,0x31,0xCD), rgb_t(0xD8,0x4A,0xE0), rgb_t(0xE8,0x5D,0xF0), \
	rgb_t(0xF0,0x66,0xF7), rgb_t(0xFF,0x78,0xFF), rgb_t(0xFF,0x8D,0xFF), rgb_t(0xFF,0x9E,0xFF), \
	rgb_t(0xFF,0xAF,0xFF), rgb_t(0xFF,0xBF,0xFF), rgb_t(0xFF,0xD2,0xFF), rgb_t(0xFF,0xE2,0xFF   )

#define NTSC_HUE6_PAL_HUE7 \
	rgb_t(0x41,0x00,0xBB), rgb_t(0x55,0x00,0xCB), rgb_t(0x6B,0x00,0xDE), rgb_t(0x7D,0x07,0xEE), \
	rgb_t(0x8F,0x21,0xFD), rgb_t(0xA0,0x37,0xFF), rgb_t(0xB3,0x4F,0xFF), rgb_t(0xC4,0x62,0xFF), \
	rgb_t(0xCB,0x6B,0xFF), rgb_t(0xDB,0x7D,0xFF), rgb_t(0xEE,0x91,0xFF), rgb_t(0xFD,0xA2,0xFF), \
	rgb_t(0xFF,0xB3,0xFF), rgb_t(0xFF,0xC3,0xFF), rgb_t(0xFF,0xD6,0xFF), rgb_t(0xFF,0xE6,0xFF   )

#define NTSC_HUE7_PAL_HUE8 \
	rgb_t(0x00,0x00,0xDB), rgb_t(0x16,0x00,0xEB), rgb_t(0x31,0x00,0xFD), rgb_t(0x46,0x1B,0xFF), \
	rgb_t(0x59,0x31,0xFF), rgb_t(0x6C,0x46,0xFF), rgb_t(0x81,0x5D,0xFF), rgb_t(0x92,0x6F,0xFF), \
	rgb_t(0x9B,0x78,0xFF), rgb_t(0xAB,0x89,0xFF), rgb_t(0xBF,0x9E,0xFF), rgb_t(0xCF,0xAE,0xFF), \
	rgb_t(0xDF,0xBF,0xFF), rgb_t(0xEF,0xCF,0xFF), rgb_t(0xFF,0xE2,0xFF), rgb_t(0xFF,0xF1,0xFF   )

#define NTSC_HUE8_PAL_HUE9 \
	rgb_t(0x00,0x00,0xD5), rgb_t(0x00,0x00,0xE5), rgb_t(0x00,0x1C,0xF7), rgb_t(0x00,0x32,0xFF), \
	rgb_t(0x1A,0x47,0xFF), rgb_t(0x30,0x5A,0xFF), rgb_t(0x49,0x70,0xFF), rgb_t(0x5C,0x82,0xFF), \
	rgb_t(0x65,0x8A,0xFF), rgb_t(0x77,0x9B,0xFF), rgb_t(0x8C,0xAF,0xFF), rgb_t(0x9D,0xBF,0xFF), \
	rgb_t(0xAE,0xD0,0xFF), rgb_t(0xBE,0xDF,0xFF), rgb_t(0xD1,0xF2,0xFF), rgb_t(0xE1,0xFF,0xFF   )

#define NTSC_HUE9_PAL_HUE10 \
	rgb_t(0x00,0x00,0xA8), rgb_t(0x00,0x1B,0xB9), rgb_t(0x00,0x35,0xCC), rgb_t(0x00,0x49,0xDC), \
	rgb_t(0x00,0x5D,0xEB), rgb_t(0x00,0x6F,0xFB), rgb_t(0x11,0x84,0xFF), rgb_t(0x29,0x95,0xFF), \
	rgb_t(0x33,0x9E,0xFF), rgb_t(0x48,0xAE,0xFF), rgb_t(0x5E,0xC2,0xFF), rgb_t(0x71,0xD2,0xFF), \
	rgb_t(0x83,0xE2,0xFF), rgb_t(0x94,0xF1,0xFF), rgb_t(0xA8,0xFF,0xFF), rgb_t(0xB8,0xFF,0xFF   )

#define NTSC_HUE10_PAL_HUE11 \
	rgb_t(0x00,0x1B,0x57), rgb_t(0x00,0x32,0x6A), rgb_t(0x00,0x4A,0x7F), rgb_t(0x00,0x5D,0x91), \
	rgb_t(0x00,0x6F,0xA2), rgb_t(0x00,0x81,0xB2), rgb_t(0x00,0x96,0xC6), rgb_t(0x01,0xA7,0xD6), \
	rgb_t(0x10,0xAF,0xDD), rgb_t(0x28,0xBF,0xED), rgb_t(0x41,0xD2,0xFF), rgb_t(0x55,0xE2,0xFF), \
	rgb_t(0x67,0xF2,0xFF), rgb_t(0x7A,0xFF,0xFF), rgb_t(0x8E,0xFF,0xFF), rgb_t(0x9F,0xFF,0xFF   )

#define NTSC_HUE11_PAL_HUE12 \
	rgb_t(0x00,0x2B,0x00), rgb_t(0x00,0x40,0x00), rgb_t(0x00,0x57,0x12), rgb_t(0x00,0x6A,0x29), \
	rgb_t(0x00,0x7C,0x3F), rgb_t(0x00,0x8D,0x52), rgb_t(0x00,0xA2,0x69), rgb_t(0x00,0xB2,0x7B), \
	rgb_t(0x0B,0xBA,0x83), rgb_t(0x24,0xCA,0x94), rgb_t(0x3D,0xDD,0xA8), rgb_t(0x51,0xED,0xB9), \
	rgb_t(0x64,0xFC,0xC9), rgb_t(0x76,0xFF,0xD9), rgb_t(0x8B,0xFF,0xEC), rgb_t(0x9C,0xFF,0xFB   )

#define NTSC_HUE12_PAL_HUE13 \
	rgb_t(0x00,0x30,0x00), rgb_t(0x00,0x45,0x00), rgb_t(0x00,0x5C,0x00), rgb_t(0x00,0x6E,0x00), \
	rgb_t(0x00,0x80,0x00), rgb_t(0x00,0x92,0x00), rgb_t(0x00,0xA6,0x00), rgb_t(0x1D,0xB6,0x10), \
	rgb_t(0x28,0xBE,0x1C), rgb_t(0x3D,0xCE,0x32), rgb_t(0x54,0xE1,0x4A), rgb_t(0x67,0xF1,0x5D), \
	rgb_t(0x79,0xFF,0x70), rgb_t(0x8B,0xFF,0x82), rgb_t(0x9F,0xFF,0x96), rgb_t(0xB0,0xFF,0xA7   )

#define NTSC_HUE13_PAL_HUE14 \
	rgb_t(0x00,0x2A,0x00), rgb_t(0x00,0x3F,0x00), rgb_t(0x00,0x57,0x00), rgb_t(0x00,0x69,0x00), \
	rgb_t(0x05,0x7B,0x00), rgb_t(0x1F,0x8D,0x00), rgb_t(0x39,0xA1,0x00), rgb_t(0x4D,0xB2,0x00), \
	rgb_t(0x56,0xBA,0x00), rgb_t(0x69,0xCA,0x00), rgb_t(0x7E,0xDD,0x00), rgb_t(0x90,0xEC,0x04), \
	rgb_t(0xA1,0xFC,0x1F), rgb_t(0xB2,0xFF,0x35), rgb_t(0xC5,0xFF,0x4D), rgb_t(0xD5,0xFF,0x60   )

#define NTSC_HUE14_PAL_HUE15 \
	rgb_t(0x00,0x1A,0x00), rgb_t(0x00,0x30,0x00), rgb_t(0x1E,0x48,0x00), rgb_t(0x35,0x5C,0x00), \
	rgb_t(0x49,0x6E,0x00), rgb_t(0x5C,0x80,0x00), rgb_t(0x72,0x95,0x00), rgb_t(0x84,0xA6,0x00), \
	rgb_t(0x8C,0xAE,0x00), rgb_t(0x9D,0xBE,0x00), rgb_t(0xB1,0xD1,0x00), rgb_t(0xC1,0xE1,0x00), \
	rgb_t(0xD1,0xF1,0x00), rgb_t(0xE1,0xFF,0x04), rgb_t(0xF4,0xFF,0x23), rgb_t(0xFF,0xFF,0x39   )

#define NTSC_HUE15 \
	rgb_t(0x32,0x00,0x00), rgb_t(0x46,0x19,0x00), rgb_t(0x5D,0x33,0x00), rgb_t(0x6F,0x48,0x00), \
	rgb_t(0x81,0x5B,0x00), rgb_t(0x93,0x6E,0x00), rgb_t(0xA7,0x83,0x00), rgb_t(0xB7,0x94,0x00), \
	rgb_t(0xBF,0x9C,0x00), rgb_t(0xCF,0xAD,0x00), rgb_t(0xE2,0xC0,0x00), rgb_t(0xF2,0xD1,0x00), \
	rgb_t(0xFF,0xE0,0x00), rgb_t(0xFF,0xF0,0x11), rgb_t(0xFF,0xFF,0x2D), rgb_t(0xFF,0xFF,0x41   )

static const rgb_t a7800_palette[256*3] =
{
	NTSC_HUE0_PAL_HUE0,
	NTSC_HUE1_PAL_HUE2,
	NTSC_HUE2_PAL_HUE3,
	NTSC_HUE3_PAL_HUE4,
	NTSC_HUE4_PAL_HUE5,
	NTSC_HUE5_PAL_HUE6,
	NTSC_HUE6_PAL_HUE7,
	NTSC_HUE7_PAL_HUE8,
	NTSC_HUE8_PAL_HUE9,
	NTSC_HUE9_PAL_HUE10,
	NTSC_HUE10_PAL_HUE11,
	NTSC_HUE11_PAL_HUE12,
	NTSC_HUE12_PAL_HUE13,
	NTSC_HUE13_PAL_HUE14,
	NTSC_HUE14_PAL_HUE15,
	NTSC_HUE15
};

static const rgb_t a7800p_palette[256*3] =
{
	NTSC_HUE0_PAL_HUE0,
	PAL_HUE1,
	NTSC_HUE1_PAL_HUE2,
	NTSC_HUE2_PAL_HUE3,
	NTSC_HUE3_PAL_HUE4,
	NTSC_HUE4_PAL_HUE5,
	NTSC_HUE5_PAL_HUE6,
	NTSC_HUE6_PAL_HUE7,
	NTSC_HUE7_PAL_HUE8,
	NTSC_HUE8_PAL_HUE9,
	NTSC_HUE9_PAL_HUE10,
	NTSC_HUE10_PAL_HUE11,
	NTSC_HUE11_PAL_HUE12,
	NTSC_HUE12_PAL_HUE13,
	NTSC_HUE13_PAL_HUE14,
	NTSC_HUE14_PAL_HUE15
};


/* Initialise the palette */
PALETTE_INIT_MEMBER(a7800_state, a7800)
{
	palette.set_pen_colors(0, a7800_palette, ARRAY_LENGTH(a7800_palette));
}


PALETTE_INIT_MEMBER(a7800_state,a7800p)
{
	palette.set_pen_colors(0, a7800p_palette, ARRAY_LENGTH(a7800p_palette));
}


/***************************************************************************
    PALETTE - 26.7 PHASE SHIFT
***************************************************************************/

#define NTSC_HUE0_PAL_HUE0_U1 \
    rgb_t(0x00,0x00,0x00), rgb_t(0x06,0x06,0x06), rgb_t(0x1C,0x1C,0x1C), rgb_t(0x2E,0x2E,0x2E), \
    rgb_t(0x40,0x40,0x40), rgb_t(0x53,0x53,0x53), rgb_t(0x69,0x69,0x69), rgb_t(0x7B,0x7B,0x7B), \
    rgb_t(0x84,0x84,0x84), rgb_t(0x96,0x96,0x96), rgb_t(0xAC,0xAC,0xAC), rgb_t(0xBF,0xBF,0xBF), \
    rgb_t(0xD1,0xD1,0xD1), rgb_t(0xE3,0xE3,0xE3), rgb_t(0xF9,0xF9,0xF9), rgb_t(0xFF,0xFF,0xFF   )

#define NTSC_HUE1_PAL_HUE2_U1 \
	rgb_t(0x31,0x00,0x00), rgb_t(0x46,0x19,0x00), rgb_t(0x5C,0x34,0x00), rgb_t(0x6F,0x48,0x00), \
    rgb_t(0x81,0x5B,0x00), rgb_t(0x92,0x6E,0x00), rgb_t(0xA6,0x83,0x00), rgb_t(0xB7,0x94,0x00), \
    rgb_t(0xBF,0x9C,0x00), rgb_t(0xCF,0xAD,0x00), rgb_t(0xE2,0xC1,0x00), rgb_t(0xF1,0xD1,0x00), \
    rgb_t(0xFF,0xE1,0x00), rgb_t(0xFF,0xF0,0x10), rgb_t(0xFF,0xFF,0x2C), rgb_t(0xFF,0xFF,0x41   )

#define PAL_HUE1_U1 \
	rgb_t(0x00,0x1A,0x00), rgb_t(0x00,0x30,0x00), rgb_t(0x1E,0x48,0x00), rgb_t(0x35,0x5C,0x00), \
	rgb_t(0x49,0x6E,0x00), rgb_t(0x5C,0x80,0x00), rgb_t(0x72,0x95,0x00), rgb_t(0x84,0xA6,0x00), \
	rgb_t(0x8C,0xAE,0x00), rgb_t(0x9D,0xBE,0x00), rgb_t(0xB1,0xD1,0x00), rgb_t(0xC1,0xE1,0x00), \
	rgb_t(0xD1,0xF1,0x00), rgb_t(0xE1,0xFF,0x04), rgb_t(0xF4,0xFF,0x23), rgb_t(0xFF,0xFF,0x39   )

#define NTSC_HUE2_PAL_HUE3_U1 \
	rgb_t(0x64,0x00,0x00), rgb_t(0x77,0x00,0x00), rgb_t(0x8B,0x19,0x00), rgb_t(0x9D,0x30,0x00), \
    rgb_t(0xAD,0x44,0x00), rgb_t(0xBE,0x58,0x00), rgb_t(0xD1,0x6E,0x00), rgb_t(0xE1,0x80,0x00), \
    rgb_t(0xE8,0x88,0x00), rgb_t(0xF8,0x99,0x00), rgb_t(0xFF,0xAD,0x0D), rgb_t(0xFF,0xBE,0x25), \
    rgb_t(0xFF,0xCE,0x3B), rgb_t(0xFF,0xDE,0x4F), rgb_t(0xFF,0xF0,0x65), rgb_t(0xFF,0xFF,0x77   )

#define NTSC_HUE3_PAL_HUE4_U1 \
	rgb_t(0x7F,0x00,0x00), rgb_t(0x91,0x00,0x00), rgb_t(0xA5,0x00,0x00), rgb_t(0xB6,0x18,0x00), \
    rgb_t(0xC6,0x2F,0x00), rgb_t(0xD6,0x44,0x0B), rgb_t(0xE9,0x5A,0x28), rgb_t(0xF8,0x6D,0x3D), \
    rgb_t(0xFF,0x76,0x47), rgb_t(0xFF,0x87,0x5A), rgb_t(0xFF,0x9C,0x70), rgb_t(0xFF,0xAC,0x82), \
    rgb_t(0xFF,0xBD,0x93), rgb_t(0xFF,0xCD,0xA4), rgb_t(0xFF,0xE0,0xB8), rgb_t(0xFF,0xF0,0xC8   )

#define NTSC_HUE4_PAL_HUE5_U1 \
	rgb_t(0x81,0x00,0x18), rgb_t(0x92,0x00,0x2F), rgb_t(0xA6,0x00,0x47), rgb_t(0xB7,0x05,0x5B), \
	rgb_t(0xC7,0x1F,0x6D), rgb_t(0xD7,0x35,0x7F), rgb_t(0xEA,0x4D,0x94), rgb_t(0xF9,0x60,0xA5), \
	rgb_t(0xFF,0x69,0xAD), rgb_t(0xFF,0x7B,0xBD), rgb_t(0xFF,0x90,0xD0), rgb_t(0xFF,0xA1,0xE0), \
	rgb_t(0xFF,0xB2,0xF0), rgb_t(0xFF,0xC2,0xFF), rgb_t(0xFF,0xD5,0xFF), rgb_t(0xFF,0xE5,0xFF   )

#define NTSC_HUE5_PAL_HUE6_U1 \
	rgb_t(0x68,0x00,0x82), rgb_t(0x7A,0x00,0x93), rgb_t(0x8F,0x00,0xA7), rgb_t(0xA0,0x00,0xB8), \
	rgb_t(0xB1,0x1B,0xC8), rgb_t(0xC1,0x32,0xD8), rgb_t(0xD4,0x4A,0xEB), rgb_t(0xE4,0x5D,0xFA), \
	rgb_t(0xEB,0x66,0xFF), rgb_t(0xFB,0x78,0xFF), rgb_t(0xFF,0x8D,0xFF), rgb_t(0xFF,0x9E,0xFF), \
	rgb_t(0xFF,0xAF,0xFF), rgb_t(0xFF,0xBF,0xFF), rgb_t(0xFF,0xD2,0xFF), rgb_t(0xFF,0xE2,0xFF   )

#define NTSC_HUE6_PAL_HUE7_U1 \
	rgb_t(0x37,0x00,0xC4), rgb_t(0x4B,0x00,0xD4), rgb_t(0x62,0x00,0xE6), rgb_t(0x74,0x0A,0xF6), \
	rgb_t(0x86,0x24,0xFF), rgb_t(0x97,0x39,0xFF), rgb_t(0xAB,0x51,0xFF), rgb_t(0xBC,0x64,0xFF), \
	rgb_t(0xC3,0x6C,0xFF), rgb_t(0xD3,0x7E,0xFF), rgb_t(0xE6,0x93,0xFF), rgb_t(0xF6,0xA4,0xFF), \
	rgb_t(0xFF,0xB5,0xFF), rgb_t(0xFF,0xC5,0xFF), rgb_t(0xFF,0xD8,0xFF), rgb_t(0xFF,0xE8,0xFF   )

#define NTSC_HUE7_PAL_HUE8_U1 \
	rgb_t(0x00,0x00,0xDD), rgb_t(0x03,0x00,0xED), rgb_t(0x22,0x05,0xFF), rgb_t(0x38,0x20,0xFF), \
	rgb_t(0x4C,0x36,0xFF), rgb_t(0x5F,0x4A,0xFF), rgb_t(0x75,0x61,0xFF), rgb_t(0x87,0x73,0xFF), \
	rgb_t(0x8F,0x7B,0xFF), rgb_t(0xA0,0x8D,0xFF), rgb_t(0xB4,0xA1,0xFF), rgb_t(0xC4,0xB2,0xFF), \
	rgb_t(0xD4,0xC2,0xFF), rgb_t(0xE4,0xD2,0xFF), rgb_t(0xF6,0xE5,0xFF), rgb_t(0xFF,0xF5,0xFF   )

#define NTSC_HUE8_PAL_HUE9_U1 \
	rgb_t(0x00,0x00,0xCD), rgb_t(0x00,0x03,0xDD), rgb_t(0x00,0x22,0xEF), rgb_t(0x00,0x38,0xFF), \
	rgb_t(0x06,0x4C,0xFF), rgb_t(0x20,0x5F,0xFF), rgb_t(0x3A,0x75,0xFF), rgb_t(0x4E,0x87,0xFF), \
	rgb_t(0x57,0x8F,0xFF), rgb_t(0x6A,0xA0,0xFF), rgb_t(0x7F,0xB4,0xFF), rgb_t(0x91,0xC4,0xFF), \
	rgb_t(0xA2,0xD4,0xFF), rgb_t(0xB2,0xE4,0xFF), rgb_t(0xC6,0xF6,0xFF), rgb_t(0xD6,0xFF,0xFF   )

#define NTSC_HUE9_PAL_HUE10_U1 \
	rgb_t(0x00,0x08,0x94), rgb_t(0x00,0x22,0xA5), rgb_t(0x00,0x3C,0xB9), rgb_t(0x00,0x50,0xC9), \
	rgb_t(0x00,0x62,0xD9), rgb_t(0x00,0x75,0xE9), rgb_t(0x00,0x8A,0xFB), rgb_t(0x1C,0x9B,0xFF), \
	rgb_t(0x27,0xA3,0xFF), rgb_t(0x3C,0xB4,0xFF), rgb_t(0x54,0xC7,0xFF), rgb_t(0x66,0xD7,0xFF), \
	rgb_t(0x79,0xE7,0xFF), rgb_t(0x8A,0xF6,0xFF), rgb_t(0x9E,0xFF,0xFF), rgb_t(0xAF,0xFF,0xFF   )

#define NTSC_HUE10_PAL_HUE11_U1 \
	rgb_t(0x00,0x21,0x35), rgb_t(0x00,0x37,0x49), rgb_t(0x00,0x4F,0x60), rgb_t(0x00,0x62,0x72), \
	rgb_t(0x00,0x74,0x84), rgb_t(0x00,0x86,0x95), rgb_t(0x00,0x9A,0xA9), rgb_t(0x00,0xAB,0xBA), \
	rgb_t(0x0A,0xB3,0xC2), rgb_t(0x23,0xC3,0xD2), rgb_t(0x3D,0xD6,0xE4), rgb_t(0x51,0xE6,0xF4), \
	rgb_t(0x63,0xF6,0xFF), rgb_t(0x76,0xFF,0xFF), rgb_t(0x8B,0xFF,0xFF), rgb_t(0x9C,0xFF,0xFF   )

#define NTSC_HUE11_PAL_HUE12_U1 \
	rgb_t(0x00,0x2E,0x00), rgb_t(0x00,0x43,0x00), rgb_t(0x00,0x5A,0x00), rgb_t(0x00,0x6D,0x00), \
	rgb_t(0x00,0x7E,0x12), rgb_t(0x00,0x90,0x2A), rgb_t(0x00,0xA4,0x42), rgb_t(0x05,0xB5,0x56), \
	rgb_t(0x12,0xBC,0x5F), rgb_t(0x2A,0xCD,0x71), rgb_t(0x43,0xDF,0x86), rgb_t(0x56,0xEF,0x98), \
	rgb_t(0x69,0xFF,0xA9), rgb_t(0x7B,0xFF,0xB9), rgb_t(0x90,0xFF,0xCC), rgb_t(0xA1,0xFF,0xDC   )

#define NTSC_HUE12_PAL_HUE13_U1 \
	rgb_t(0x00,0x2F,0x00), rgb_t(0x00,0x44,0x00), rgb_t(0x00,0x5B,0x00), rgb_t(0x00,0x6D,0x00), \
	rgb_t(0x00,0x7F,0x00), rgb_t(0x00,0x91,0x00), rgb_t(0x18,0xA5,0x00), rgb_t(0x2F,0xB5,0x00), \
	rgb_t(0x39,0xBD,0x00), rgb_t(0x4D,0xCD,0x06), rgb_t(0x63,0xE0,0x24), rgb_t(0x76,0xF0,0x3A), \
	rgb_t(0x87,0xFF,0x4E), rgb_t(0x99,0xFF,0x61), rgb_t(0xAC,0xFF,0x77), rgb_t(0xBD,0xFF,0x88   )

#define NTSC_HUE13_PAL_HUE14_U1 \
	rgb_t(0x00,0x24,0x00), rgb_t(0x00,0x3A,0x00), rgb_t(0x00,0x51,0x00), rgb_t(0x0C,0x64,0x00), \
	rgb_t(0x25,0x77,0x00), rgb_t(0x3A,0x88,0x00), rgb_t(0x52,0x9D,0x00), rgb_t(0x65,0xAD,0x00), \
	rgb_t(0x6D,0xB5,0x00), rgb_t(0x7F,0xC6,0x00), rgb_t(0x94,0xD8,0x00), rgb_t(0xA5,0xE8,0x00), \
	rgb_t(0xB6,0xF8,0x00), rgb_t(0xC6,0xFF,0x1B), rgb_t(0xD9,0xFF,0x36), rgb_t(0xE9,0xFF,0x4A   )

#define NTSC_HUE14_PAL_HUE15_U1 \
	rgb_t(0x0C,0x0E,0x00), rgb_t(0x25,0x26,0x00), rgb_t(0x3E,0x3F,0x00), rgb_t(0x52,0x53,0x00), \
	rgb_t(0x65,0x66,0x00), rgb_t(0x77,0x78,0x00), rgb_t(0x8C,0x8D,0x00), rgb_t(0x9D,0x9E,0x00), \
	rgb_t(0xA5,0xA6,0x00), rgb_t(0xB6,0xB7,0x00), rgb_t(0xC9,0xCA,0x00), rgb_t(0xD9,0xDA,0x00), \
	rgb_t(0xE9,0xE9,0x00), rgb_t(0xF8,0xF9,0x04), rgb_t(0xFF,0xFF,0x21), rgb_t(0xFF,0xFF,0x37   )

#define NTSC_HUE15_U1 \
	rgb_t(0x4E,0x00,0x00), rgb_t(0x61,0x09,0x00), rgb_t(0x76,0x27,0x00), rgb_t(0x88,0x3C,0x00), \
	rgb_t(0x99,0x50,0x00), rgb_t(0xAA,0x63,0x00), rgb_t(0xBE,0x78,0x00), rgb_t(0xCE,0x8A,0x00), \
	rgb_t(0xD5,0x92,0x00), rgb_t(0xE5,0xA3,0x00), rgb_t(0xF8,0xB7,0x00), rgb_t(0xFF,0xC7,0x00), \
	rgb_t(0xFF,0xD7,0x14), rgb_t(0xFF,0xE7,0x2C), rgb_t(0xFF,0xF9,0x44), rgb_t(0xFF,0xFF,0x58   )

static const rgb_t a7800u1_palette[256*3] =
{
	NTSC_HUE0_PAL_HUE0_U1,
	NTSC_HUE1_PAL_HUE2_U1,
	NTSC_HUE2_PAL_HUE3_U1,
	NTSC_HUE3_PAL_HUE4_U1,
	NTSC_HUE4_PAL_HUE5_U1,
	NTSC_HUE5_PAL_HUE6_U1,
	NTSC_HUE6_PAL_HUE7_U1,
	NTSC_HUE7_PAL_HUE8_U1,
	NTSC_HUE8_PAL_HUE9_U1,
	NTSC_HUE9_PAL_HUE10_U1,
	NTSC_HUE10_PAL_HUE11_U1,
	NTSC_HUE11_PAL_HUE12_U1,
	NTSC_HUE12_PAL_HUE13_U1,
	NTSC_HUE13_PAL_HUE14_U1,
	NTSC_HUE14_PAL_HUE15_U1,
	NTSC_HUE15_U1
};

static const rgb_t a7800pu1_palette[256*3] =
{
	NTSC_HUE0_PAL_HUE0_U1,
	PAL_HUE1_U1,
	NTSC_HUE1_PAL_HUE2_U1,
	NTSC_HUE2_PAL_HUE3_U1,
	NTSC_HUE3_PAL_HUE4_U1,
	NTSC_HUE4_PAL_HUE5_U1,
	NTSC_HUE5_PAL_HUE6_U1,
	NTSC_HUE6_PAL_HUE7_U1,
	NTSC_HUE7_PAL_HUE8_U1,
	NTSC_HUE8_PAL_HUE9_U1,
	NTSC_HUE9_PAL_HUE10_U1,
	NTSC_HUE10_PAL_HUE11_U1,
	NTSC_HUE11_PAL_HUE12_U1,
	NTSC_HUE12_PAL_HUE13_U1,
	NTSC_HUE13_PAL_HUE14_U1,
	NTSC_HUE14_PAL_HUE15_U1
};


/* Initialise the palette */
PALETTE_INIT_MEMBER(a7800_state, a7800u1)
{
	palette.set_pen_colors(0, a7800u1_palette, ARRAY_LENGTH(a7800u1_palette));
}


PALETTE_INIT_MEMBER(a7800_state,a7800pu1)
{
	palette.set_pen_colors(0, a7800pu1_palette, ARRAY_LENGTH(a7800pu1_palette));
}


/***************************************************************************
    PALETTE - 27.7 PHASE SHIFT via Commondore 1702
***************************************************************************/

#define NTSC_HUE0_PAL_HUE0_U2 \
    rgb_t(0x00,0x00,0x00), rgb_t(0x06,0x06,0x06), rgb_t(0x1C,0x1C,0x1C), rgb_t(0x2E,0x2E,0x2E), \
    rgb_t(0x40,0x40,0x40), rgb_t(0x53,0x53,0x53), rgb_t(0x69,0x69,0x69), rgb_t(0x7B,0x7B,0x7B), \
    rgb_t(0x84,0x84,0x84), rgb_t(0x96,0x96,0x96), rgb_t(0xAC,0xAC,0xAC), rgb_t(0xBF,0xBF,0xBF), \
    rgb_t(0xD1,0xD1,0xD1), rgb_t(0xE3,0xE3,0xE3), rgb_t(0xF9,0xF9,0xF9), rgb_t(0xFF,0xFF,0xFF   )

#define NTSC_HUE1_PAL_HUE2_U2 \
	rgb_t(0x31,0x00,0x00), rgb_t(0x46,0x19,0x00), rgb_t(0x5C,0x34,0x00), rgb_t(0x6F,0x48,0x00), \
    rgb_t(0x81,0x5B,0x00), rgb_t(0x92,0x6E,0x00), rgb_t(0xA6,0x83,0x00), rgb_t(0xB7,0x94,0x00), \
    rgb_t(0xBF,0x9C,0x00), rgb_t(0xCF,0xAD,0x00), rgb_t(0xE2,0xC1,0x00), rgb_t(0xF1,0xD1,0x00), \
    rgb_t(0xFF,0xE1,0x00), rgb_t(0xFF,0xF0,0x10), rgb_t(0xFF,0xFF,0x2C), rgb_t(0xFF,0xFF,0x41   )

#define PAL_HUE1_U2 \
	rgb_t(0x00,0x1A,0x00), rgb_t(0x00,0x30,0x00), rgb_t(0x1E,0x48,0x00), rgb_t(0x35,0x5C,0x00), \
	rgb_t(0x49,0x6E,0x00), rgb_t(0x5C,0x80,0x00), rgb_t(0x72,0x95,0x00), rgb_t(0x84,0xA6,0x00), \
	rgb_t(0x8C,0xAE,0x00), rgb_t(0x9D,0xBE,0x00), rgb_t(0xB1,0xD1,0x00), rgb_t(0xC1,0xE1,0x00), \
	rgb_t(0xD1,0xF1,0x00), rgb_t(0xE1,0xFF,0x04), rgb_t(0xF4,0xFF,0x23), rgb_t(0xFF,0xFF,0x39   )

#define NTSC_HUE2_PAL_HUE3_U2 \
	rgb_t(0x66,0x00,0x00), rgb_t(0x78,0x00,0x00), rgb_t(0x8D,0x18,0x00), rgb_t(0x9E,0x2F,0x00), \
    rgb_t(0xAF,0x43,0x00), rgb_t(0xBF,0x57,0x00), rgb_t(0xD2,0x6D,0x00), rgb_t(0xE2,0x7F,0x00), \
    rgb_t(0xEA,0x87,0x00), rgb_t(0xF9,0x99,0x00), rgb_t(0xFF,0xAC,0x11), rgb_t(0xFF,0xBD,0x29), \
    rgb_t(0xFF,0xCD,0x3E), rgb_t(0xFF,0xDD,0x52), rgb_t(0xFF,0xF0,0x68), rgb_t(0xFF,0xFF,0x7A   )

#define NTSC_HUE3_PAL_HUE4_U2 \
	rgb_t(0x81,0x00,0x00), rgb_t(0x92,0x00,0x00), rgb_t(0xA6,0x00,0x00), rgb_t(0xB7,0x16,0x00), \
    rgb_t(0xC7,0x2D,0x00), rgb_t(0xD7,0x42,0x17), rgb_t(0xEA,0x59,0x32), rgb_t(0xF9,0x6C,0x46), \
    rgb_t(0xFF,0x74,0x50), rgb_t(0xFF,0x86,0x63), rgb_t(0xFF,0x9A,0x78), rgb_t(0xFF,0xAB,0x8A), \
    rgb_t(0xFF,0xBC,0x9B), rgb_t(0xFF,0xCC,0xAC), rgb_t(0xFF,0xDF,0xBF), rgb_t(0xFF,0xEF,0xCF   )

#define NTSC_HUE4_PAL_HUE5_U2 \
	rgb_t(0x7F,0x00,0x28), rgb_t(0x91,0x00,0x3D), rgb_t(0xA5,0x00,0x55), rgb_t(0xB5,0x03,0x67), \
	rgb_t(0xC6,0x1E,0x79), rgb_t(0xD6,0x34,0x8B), rgb_t(0xE8,0x4C,0x9F), rgb_t(0xF8,0x5F,0xB0), \
	rgb_t(0xFF,0x68,0xB8), rgb_t(0xFF,0x7A,0xC8), rgb_t(0xFF,0x8F,0xDB), rgb_t(0xFF,0xA0,0xEB), \
	rgb_t(0xFF,0xB1,0xFA), rgb_t(0xFF,0xC1,0xFF), rgb_t(0xFF,0xD4,0xFF), rgb_t(0xFF,0xE4,0xFF   )

#define NTSC_HUE5_PAL_HUE6_U2 \
	rgb_t(0x62,0x00,0x8F), rgb_t(0x74,0x00,0xA0), rgb_t(0x89,0x00,0xB4), rgb_t(0x9A,0x00,0xC4), \
	rgb_t(0xAB,0x1C,0xD4), rgb_t(0xBC,0x32,0xE4), rgb_t(0xCF,0x4A,0xF7), rgb_t(0xDF,0x5D,0xFF), \
	rgb_t(0xE6,0x66,0xFF), rgb_t(0xF6,0x78,0xFF), rgb_t(0xFF,0x8D,0xFF), rgb_t(0xFF,0x9E,0xFF), \
	rgb_t(0xFF,0xAF,0xFF), rgb_t(0xFF,0xBF,0xFF), rgb_t(0xFF,0xD3,0xFF), rgb_t(0xFF,0xE2,0xFF   )

#define NTSC_HUE6_PAL_HUE7_U2 \
	rgb_t(0x2A,0x00,0xCC), rgb_t(0x3F,0x00,0xDC), rgb_t(0x57,0x00,0xEF), rgb_t(0x69,0x0E,0xFE), \
	rgb_t(0x7B,0x27,0xFF), rgb_t(0x8D,0x3C,0xFF), rgb_t(0xA1,0x53,0xFF), rgb_t(0xB2,0x66,0xFF), \
	rgb_t(0xBA,0x6F,0xFF), rgb_t(0xCA,0x81,0xFF), rgb_t(0xDD,0x95,0xFF), rgb_t(0xEC,0xA6,0xFF), \
	rgb_t(0xFC,0xB7,0xFF), rgb_t(0xFF,0xC7,0xFF), rgb_t(0xFF,0xDA,0xFF), rgb_t(0xFF,0xEA,0xFF   )

#define NTSC_HUE7_PAL_HUE8_U2 \
	rgb_t(0x00,0x00,0xDD), rgb_t(0x00,0x00,0xED), rgb_t(0x0F,0x0D,0xFF), rgb_t(0x28,0x25,0xFF), \
	rgb_t(0x3D,0x3B,0xFF), rgb_t(0x51,0x4F,0xFF), rgb_t(0x67,0x65,0xFF), rgb_t(0x79,0x77,0xFF), \
	rgb_t(0x82,0x80,0xFF), rgb_t(0x93,0x91,0xFF), rgb_t(0xA7,0xA5,0xFF), rgb_t(0xB8,0xB6,0xFF), \
	rgb_t(0xC8,0xC6,0xFF), rgb_t(0xD8,0xD6,0xFF), rgb_t(0xEA,0xE9,0xFF), rgb_t(0xFA,0xF8,0xFF   )

#define NTSC_HUE8_PAL_HUE9_U2 \
	rgb_t(0x00,0x00,0xC1), rgb_t(0x00,0x0D,0xD1), rgb_t(0x00,0x2A,0xE4), rgb_t(0x00,0x3F,0xF4), \
	rgb_t(0x00,0x53,0xFF), rgb_t(0x0D,0x66,0xFF), rgb_t(0x2A,0x7B,0xFF), rgb_t(0x3F,0x8C,0xFF), \
	rgb_t(0x49,0x95,0xFF), rgb_t(0x5C,0xA6,0xFF), rgb_t(0x72,0xB9,0xFF), rgb_t(0x83,0xC9,0xFF), \
	rgb_t(0x95,0xD9,0xFF), rgb_t(0xA6,0xE9,0xFF), rgb_t(0xB9,0xFC,0xFF), rgb_t(0xCA,0xFF,0xFF   )

#define NTSC_HUE9_PAL_HUE10_U2 \
	rgb_t(0x00,0x12,0x7A), rgb_t(0x00,0x2A,0x8C), rgb_t(0x00,0x43,0xA0), rgb_t(0x00,0x56,0xB1), \
	rgb_t(0x00,0x69,0xC1), rgb_t(0x00,0x7B,0xD1), rgb_t(0x00,0x90,0xE4), rgb_t(0x0F,0xA1,0xF3), \
	rgb_t(0x1B,0xA9,0xFB), rgb_t(0x31,0xB9,0xFF), rgb_t(0x4A,0xCC,0xFF), rgb_t(0x5D,0xDC,0xFF), \
	rgb_t(0x6F,0xEC,0xFF), rgb_t(0x81,0xFC,0xFF), rgb_t(0x96,0xFF,0xFF), rgb_t(0xA7,0xFF,0xFF   )

#define NTSC_HUE10_PAL_HUE11_U2 \
	rgb_t(0x00,0x27,0x02), rgb_t(0x00,0x3D,0x1E), rgb_t(0x00,0x54,0x38), rgb_t(0x00,0x67,0x4C), \
	rgb_t(0x00,0x79,0x5F), rgb_t(0x00,0x8B,0x71), rgb_t(0x00,0x9F,0x86), rgb_t(0x00,0xB0,0x98), \
	rgb_t(0x08,0xB7,0xA0), rgb_t(0x22,0xC8,0xB0), rgb_t(0x3B,0xDB,0xC4), rgb_t(0x4F,0xEA,0xD4), \
	rgb_t(0x62,0xFA,0xE4), rgb_t(0x75,0xFF,0xF3), rgb_t(0x89,0xFF,0xFF), rgb_t(0x9B,0xFF,0xFF   )

#define NTSC_HUE11_PAL_HUE12_U2 \
	rgb_t(0x00,0x30,0x00), rgb_t(0x00,0x45,0x00), rgb_t(0x00,0x5C,0x00), rgb_t(0x00,0x6E,0x00), \
	rgb_t(0x00,0x80,0x00), rgb_t(0x00,0x91,0x00), rgb_t(0x00,0xA5,0x12), rgb_t(0x14,0xB6,0x2A), \
	rgb_t(0x1F,0xBE,0x34), rgb_t(0x35,0xCE,0x49), rgb_t(0x4D,0xE1,0x5F), rgb_t(0x60,0xF0,0x72), \
	rgb_t(0x73,0xFF,0x83), rgb_t(0x84,0xFF,0x95), rgb_t(0x99,0xFF,0xA9), rgb_t(0xAA,0xFF,0xB9   )

#define NTSC_HUE12_PAL_HUE13_U2 \
	rgb_t(0x00,0x2C,0x00), rgb_t(0x00,0x41,0x00), rgb_t(0x00,0x58,0x00), rgb_t(0x00,0x6B,0x00), \
	rgb_t(0x00,0x7D,0x00), rgb_t(0x16,0x8E,0x00), rgb_t(0x31,0xA2,0x00), rgb_t(0x46,0xB3,0x00), \
	rgb_t(0x4F,0xBB,0x00), rgb_t(0x62,0xCB,0x00), rgb_t(0x78,0xDE,0x00), rgb_t(0x89,0xED,0x12), \
	rgb_t(0x9A,0xFD,0x2A), rgb_t(0xAB,0xFF,0x3F), rgb_t(0xBF,0xFF,0x56), rgb_t(0xCF,0xFF,0x69   )

#define NTSC_HUE13_PAL_HUE14_U2 \
	rgb_t(0x00,0x1B,0x00), rgb_t(0x00,0x32,0x00), rgb_t(0x19,0x4A,0x00), rgb_t(0x30,0x5D,0x00), \
	rgb_t(0x45,0x6F,0x00), rgb_t(0x58,0x81,0x00), rgb_t(0x6E,0x96,0x00), rgb_t(0x80,0xA7,0x00), \
	rgb_t(0x88,0xAF,0x00), rgb_t(0x9A,0xBF,0x00), rgb_t(0xAD,0xD2,0x00), rgb_t(0xBE,0xE2,0x00), \
	rgb_t(0xCE,0xF2,0x00), rgb_t(0xDE,0xFF,0x06), rgb_t(0xF1,0xFF,0x25), rgb_t(0xFF,0xFF,0x3A   )

#define NTSC_HUE14_PAL_HUE15_U2 \
	rgb_t(0x32,0x00,0x00), rgb_t(0x47,0x18,0x00), rgb_t(0x5E,0x33,0x00), rgb_t(0x70,0x48,0x00), \
	rgb_t(0x82,0x5B,0x00), rgb_t(0x93,0x6D,0x00), rgb_t(0xA7,0x83,0x00), rgb_t(0xB8,0x94,0x00), \
	rgb_t(0xC0,0x9C,0x00), rgb_t(0xD0,0xAD,0x00), rgb_t(0xE3,0xC0,0x00), rgb_t(0xF2,0xD0,0x00), \
	rgb_t(0xFF,0xE0,0x00), rgb_t(0xFF,0xF0,0x11), rgb_t(0xFF,0xFF,0x2D), rgb_t(0xFF,0xFF,0x42   )

#define  NTSC_HUE15_U2 \
	rgb_t(0x67,0x00,0x00), rgb_t(0x79,0x00,0x00), rgb_t(0x8E,0x17,0x00), rgb_t(0x9F,0x2E,0x00), \
	rgb_t(0xAF,0x43,0x00), rgb_t(0xC0,0x57,0x00), rgb_t(0xD3,0x6D,0x00), rgb_t(0xE3,0x7F,0x00), \
	rgb_t(0xEA,0x87,0x00), rgb_t(0xFA,0x98,0x00), rgb_t(0xFF,0xAC,0x13), rgb_t(0xFF,0xBD,0x2B), \
	rgb_t(0xFF,0xCD,0x40), rgb_t(0xFF,0xDD,0x54), rgb_t(0xFF,0xEF,0x6A), rgb_t(0xFF,0xFF,0x7C   )

static const rgb_t a7800u2_palette[256*3] =
{
	NTSC_HUE0_PAL_HUE0_U2,
	NTSC_HUE1_PAL_HUE2_U2,
	NTSC_HUE2_PAL_HUE3_U2,
	NTSC_HUE3_PAL_HUE4_U2,
	NTSC_HUE4_PAL_HUE5_U2,
	NTSC_HUE5_PAL_HUE6_U2,
	NTSC_HUE6_PAL_HUE7_U2,
	NTSC_HUE7_PAL_HUE8_U2,
	NTSC_HUE8_PAL_HUE9_U2,
	NTSC_HUE9_PAL_HUE10_U2,
	NTSC_HUE10_PAL_HUE11_U2,
	NTSC_HUE11_PAL_HUE12_U2,
	NTSC_HUE12_PAL_HUE13_U2,
	NTSC_HUE13_PAL_HUE14_U2,
	NTSC_HUE14_PAL_HUE15_U2,
	NTSC_HUE15_U2
};

static const rgb_t a7800pu2_palette[256*3] =
{
	NTSC_HUE0_PAL_HUE0_U2,
	PAL_HUE1_U2,
	NTSC_HUE1_PAL_HUE2_U2,
	NTSC_HUE2_PAL_HUE3_U2,
	NTSC_HUE3_PAL_HUE4_U2,
	NTSC_HUE4_PAL_HUE5_U2,
	NTSC_HUE5_PAL_HUE6_U2,
	NTSC_HUE6_PAL_HUE7_U2,
	NTSC_HUE7_PAL_HUE8_U2,
	NTSC_HUE8_PAL_HUE9_U2,
	NTSC_HUE9_PAL_HUE10_U2,
	NTSC_HUE10_PAL_HUE11_U2,
	NTSC_HUE11_PAL_HUE12_U2,
	NTSC_HUE12_PAL_HUE13_U2,
	NTSC_HUE13_PAL_HUE14_U2,
	NTSC_HUE14_PAL_HUE15_U2
};


/* Initialise the palette */
PALETTE_INIT_MEMBER(a7800_state, a7800u2)
{
	palette.set_pen_colors(0, a7800u2_palette, ARRAY_LENGTH(a7800u2_palette));
}


PALETTE_INIT_MEMBER(a7800_state,a7800pu2)
{
	palette.set_pen_colors(0, a7800pu2_palette, ARRAY_LENGTH(a7800pu2_palette));
}


/***************************************************************************
    MACHINE DRIVERS
***************************************************************************/

void a7800_state::machine_start()
{
	save_item(NAME(m_p1_one_button));
	save_item(NAME(m_p2_one_button));
	save_item(NAME(m_bios_enabled));
	save_item(NAME(m_ctrl_lock));
	save_item(NAME(m_ctrl_reg));
	save_item(NAME(m_maria_flag));

	// set up RAM mirrors
	uint8_t *ram = reinterpret_cast<uint8_t *>(memshare("6116_2")->ptr());
	membank("zpmirror")->set_base(ram + 0x0040);
	membank("spmirror")->set_base(ram + 0x0140);

	// install additional handlers, if needed
	if (m_cart->exists())
	{
		switch (m_cart->get_cart_type())
		{
			case A78_HSC:
				// ROM+NVRAM accesses for HiScore
				m_maincpu->space(AS_PROGRAM).install_readwrite_handler(0x1000, 0x17ff, read8_delegate(FUNC(a78_cart_slot_device::read_10xx),(a78_cart_slot_device*)m_cart), write8_delegate(FUNC(a78_cart_slot_device::write_10xx),(a78_cart_slot_device*)m_cart));
				m_maincpu->space(AS_PROGRAM).install_readwrite_handler(0x3000, 0x3fff, read8_delegate(FUNC(a78_cart_slot_device::read_30xx),(a78_cart_slot_device*)m_cart), write8_delegate(FUNC(a78_cart_slot_device::write_30xx),(a78_cart_slot_device*)m_cart));
				break;
			case A78_TYPE0_POK450:
			case A78_TYPE1_POK450:
			case A78_TYPE6_POK450:
			case A78_TYPEA_POK450:
			case A78_VERSA_POK450:
				// POKEY and RAM regs at 0x400-0x47f
				m_maincpu->space(AS_PROGRAM).install_readwrite_handler(0x0400, 0x047f, read8_delegate(FUNC(a78_cart_slot_device::read_04xx),(a78_cart_slot_device*)m_cart), write8_delegate(FUNC(a78_cart_slot_device::write_04xx),(a78_cart_slot_device*)m_cart));
				break;
			case A78_XM_BOARD:
				// POKEY and RAM and YM regs at 0x400-0x47f
				m_maincpu->space(AS_PROGRAM).install_readwrite_handler(0x0400, 0x047f, read8_delegate(FUNC(a78_cart_slot_device::read_04xx),(a78_cart_slot_device*)m_cart), write8_delegate(FUNC(a78_cart_slot_device::write_04xx),(a78_cart_slot_device*)m_cart));
				// ROM+NVRAM accesses for HiScore
				m_maincpu->space(AS_PROGRAM).install_readwrite_handler(0x1000, 0x17ff, read8_delegate(FUNC(a78_cart_slot_device::read_10xx),(a78_cart_slot_device*)m_cart), write8_delegate(FUNC(a78_cart_slot_device::write_10xx),(a78_cart_slot_device*)m_cart));
				m_maincpu->space(AS_PROGRAM).install_readwrite_handler(0x3000, 0x3fff, read8_delegate(FUNC(a78_cart_slot_device::read_30xx),(a78_cart_slot_device*)m_cart), write8_delegate(FUNC(a78_cart_slot_device::write_30xx),(a78_cart_slot_device*)m_cart));
				break;
		}
	}
}

void a7800_state::machine_reset()
{
	m_ctrl_lock = 0;
	m_ctrl_reg = 0;
	m_maria_flag = 0;
	m_bios_enabled = 0;
	tia_delay = 0;
	if ((m_maincpu->space(AS_PROGRAM).read_byte(0xfffe) == 0xff) && (m_maincpu->space(AS_PROGRAM).read_byte(0xffff) == 0xff) )
		m_ctrl_reg = 4; // flip to cart if the bios isn't present
}

static MACHINE_CONFIG_START( a7800_ntsc )
	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", M6502, A7800_NTSC_Y1/8) /* 1.79 MHz (switches to 1.19 MHz on TIA or RIOT RAM access) */
	MCFG_CPU_PROGRAM_MAP(a7800_mem)
	MCFG_TIMER_DRIVER_ADD_SCANLINE("scantimer", a7800_state, interrupt, "screen", 0, 1)

	/* video hardware */
	MCFG_SCREEN_ADD("screen", RASTER)
	//MCFG_SCREEN_RAW_PARAMS( 7159090, 454, 0, 320, 263, 27, 27 + 192 + 32 )
	MCFG_SCREEN_RAW_PARAMS( 7159090, 454, 0, 320, 263, 16, 16 + 243 -3 )
	/*Exact Frame Timing per updated 7800 Software Guide which is 
	  verified through hardware testing.  Actual number of possible 
	  visible lines is 243, but we subtract 3 for the sake of rounding
	  to an even number that follows NTSC standard resolution*/
	MCFG_SCREEN_DEFAULT_POSITION( 0.910, 0.000, 1.072, -0.013 )
	MCFG_SCREEN_UPDATE_DEVICE("maria", atari_maria_device, screen_update)
	MCFG_SCREEN_PALETTE("palette")

	MCFG_PALETTE_ADD("palette", ARRAY_LENGTH(a7800_palette) / 3)
	MCFG_PALETTE_INIT_OWNER(a7800_state, a7800)

	MCFG_DEVICE_ADD("maria", ATARI_MARIA, 0)
	MCFG_MARIA_DMACPU("maincpu")

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("mono")
	MCFG_SOUND_TIA_ADD("tia", 31400)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 1.00)

	/* devices */
    MCFG_DEVICE_ADD("riot", RIOT6532, A7800_NTSC_Y1/8)
	MCFG_RIOT6532_IN_PA_CB(READ8(a7800_state, riot_joystick_r))
	MCFG_RIOT6532_OUT_PA_CB(WRITE8(a7800_state, riot_joystick_w))
	MCFG_RIOT6532_IN_PB_CB(READ8(a7800_state, riot_console_button_r))
	MCFG_RIOT6532_OUT_PB_CB(WRITE8(a7800_state, riot_button_pullup_w))


	MCFG_A7800_CONTROL_PORT_ADD("joy1", a7800_control_port_devices, "proline_joystick")
	MCFG_A7800_CONTROL_PORT_ADD("joy2", a7800_control_port_devices, "proline_joystick")

	MCFG_A78_CARTRIDGE_ADD("cartslot", a7800_cart, nullptr)

	/* software lists */
	MCFG_SOFTWARE_LIST_ADD("cart_list","a7800")
	MCFG_SOFTWARE_LIST_FILTER("cart_list","NTSC")
MACHINE_CONFIG_END


static MACHINE_CONFIG_DERIVED( a7800_pal, a7800_ntsc )
	/* basic machine hardware */
	MCFG_CPU_MODIFY("maincpu")
	MCFG_CPU_CLOCK(CLK_PAL)
//  MCFG_TIMER_ADD_SCANLINE("scantimer", a7800_interrupt, "screen", 0, 1)

	MCFG_SCREEN_MODIFY( "screen" )
	//MCFG_SCREEN_RAW_PARAMS( 7093788, 454, 0, 320, 313, 35, 35 + 228 + 32 )
	MCFG_SCREEN_RAW_PARAMS( 7093788, 454, 0, 320, 313, 16, 16 + 293 -5 ) 
	/*Exact Frame Timing per updated 7800 Software Guide which is 
	  verified through hardware testing.  Actual number of possible 
	  visible lines is 293, but we subtract 5 for the sake of rounding
	  to an even number that follows PAL standard resolution*/
	MCFG_SCREEN_DEFAULT_POSITION( 0.910, 0.000, 1.108, -0.013 )
	MCFG_PALETTE_MODIFY("palette")
	MCFG_PALETTE_INIT_OWNER(a7800_state, a7800p )

	/* devices */
	MCFG_DEVICE_REMOVE("riot")
	MCFG_DEVICE_ADD("riot", RIOT6532, CLK_PAL)
	MCFG_RIOT6532_IN_PA_CB(READ8(a7800_state, riot_joystick_r))
	MCFG_RIOT6532_OUT_PA_CB(WRITE8(a7800_state, riot_joystick_w))
	MCFG_RIOT6532_IN_PB_CB(READ8(a7800_state, riot_console_button_r))
	MCFG_RIOT6532_OUT_PB_CB(WRITE8(a7800_state, riot_button_pullup_w))


	/* software lists */
	MCFG_DEVICE_REMOVE("cart_list")
	MCFG_SOFTWARE_LIST_ADD("cart_list","a7800")
	MCFG_SOFTWARE_LIST_FILTER("cart_list","PAL")
MACHINE_CONFIG_END


static MACHINE_CONFIG_DERIVED( a7800u1_ntsc, a7800_ntsc )

	MCFG_PALETTE_MODIFY("palette")
	MCFG_PALETTE_INIT_OWNER(a7800_state, a7800u1 )

MACHINE_CONFIG_END


static MACHINE_CONFIG_DERIVED( a7800u2_ntsc, a7800_ntsc )

	MCFG_PALETTE_MODIFY("palette")
	MCFG_PALETTE_INIT_OWNER(a7800_state, a7800u2 )

MACHINE_CONFIG_END


static MACHINE_CONFIG_DERIVED( a7800u1_pal, a7800_ntsc )
	/* basic machine hardware */
	MCFG_CPU_MODIFY("maincpu")
	MCFG_CPU_CLOCK(CLK_PAL)
//  MCFG_TIMER_ADD_SCANLINE("scantimer", a7800_interrupt, "screen", 0, 1)

	MCFG_SCREEN_MODIFY( "screen" )
	//MCFG_SCREEN_RAW_PARAMS( 7093788, 454, 0, 320, 313, 35, 35 + 228 + 32 )
	MCFG_SCREEN_RAW_PARAMS( 7093788, 454, 0, 320, 313, 16, 16 + 293 -5 )
	MCFG_SCREEN_DEFAULT_POSITION( 0.910, 0.000, 1.108, -0.013 )

	MCFG_PALETTE_MODIFY("palette")
	MCFG_PALETTE_INIT_OWNER(a7800_state, a7800pu1 )

	/* devices */
	MCFG_DEVICE_REMOVE("riot")
	MCFG_DEVICE_ADD("riot", RIOT6532, CLK_PAL)
	MCFG_RIOT6532_IN_PA_CB(READ8(a7800_state, riot_joystick_r))
	MCFG_RIOT6532_OUT_PA_CB(WRITE8(a7800_state, riot_joystick_w))
	MCFG_RIOT6532_IN_PB_CB(READ8(a7800_state, riot_console_button_r))
	MCFG_RIOT6532_OUT_PB_CB(WRITE8(a7800_state, riot_button_pullup_w))


	/* software lists */
	MCFG_DEVICE_REMOVE("cart_list")
	MCFG_SOFTWARE_LIST_ADD("cart_list","a7800")
	MCFG_SOFTWARE_LIST_FILTER("cart_list","PAL")
MACHINE_CONFIG_END


static MACHINE_CONFIG_DERIVED( a7800u2_pal, a7800_ntsc )
	/* basic machine hardware */
	MCFG_CPU_MODIFY("maincpu")
	MCFG_CPU_CLOCK(CLK_PAL)
//  MCFG_TIMER_ADD_SCANLINE("scantimer", a7800_interrupt, "screen", 0, 1)

	MCFG_SCREEN_MODIFY( "screen" )
	//MCFG_SCREEN_RAW_PARAMS( 7093788, 454, 0, 320, 313, 35, 35 + 228 + 32 )
	MCFG_SCREEN_RAW_PARAMS( 7093788, 454, 0, 320, 313, 16, 16 + 293 -5 )
	MCFG_SCREEN_DEFAULT_POSITION( 0.910, 0.000, 1.108, -0.013 )
	MCFG_PALETTE_MODIFY("palette")
	MCFG_PALETTE_INIT_OWNER(a7800_state, a7800pu2 )

	/* devices */
	MCFG_DEVICE_REMOVE("riot")
	MCFG_DEVICE_ADD("riot", RIOT6532, CLK_PAL)
	MCFG_RIOT6532_IN_PA_CB(READ8(a7800_state, riot_joystick_r))
	MCFG_RIOT6532_OUT_PA_CB(WRITE8(a7800_state, riot_joystick_w))
	MCFG_RIOT6532_IN_PB_CB(READ8(a7800_state, riot_console_button_r))
	MCFG_RIOT6532_OUT_PB_CB(WRITE8(a7800_state, riot_button_pullup_w))


	/* software lists */
	MCFG_DEVICE_REMOVE("cart_list")
	MCFG_SOFTWARE_LIST_ADD("cart_list","a7800")
	MCFG_SOFTWARE_LIST_FILTER("cart_list","PAL")
MACHINE_CONFIG_END


/***************************************************************************
    ROM DEFINITIONS
***************************************************************************/

ROM_START( a7800 )
	ROM_REGION(0x4000, "maincpu", ROMREGION_ERASEFF)
	ROM_SYSTEM_BIOS( 0, "a7800", "Atari 7800" )
	ROMX_LOAD("7800.u7", 0x3000, 0x1000, CRC(5d13730c) SHA1(d9d134bb6b36907c615a594cc7688f7bfcef5b43), ROM_BIOS(1) | ROM_OPTIONAL)
	ROM_SYSTEM_BIOS( 1, "a7800pr", "Atari 7800 (prototype with Asteroids)" )
	ROMX_LOAD("c300558-001a.u7", 0x0000, 0x4000, CRC(a0e10edf) SHA1(14584b1eafe9721804782d4b1ac3a4a7313e455f), ROM_BIOS(2) | ROM_OPTIONAL)
ROM_END

ROM_START( a7800u1 )
	ROM_REGION(0x4000, "maincpu", ROMREGION_ERASEFF)
	ROM_SYSTEM_BIOS( 0, "a7800", "Atari 7800" )
	ROMX_LOAD("7800.u7", 0x3000, 0x1000, CRC(5d13730c) SHA1(d9d134bb6b36907c615a594cc7688f7bfcef5b43), ROM_BIOS(1) | ROM_OPTIONAL)
	ROM_SYSTEM_BIOS( 1, "a7800pr", "Atari 7800 (prototype with Asteroids)" )
	ROMX_LOAD("c300558-001a.u7", 0x0000, 0x4000, CRC(a0e10edf) SHA1(14584b1eafe9721804782d4b1ac3a4a7313e455f), ROM_BIOS(2) | ROM_OPTIONAL)
ROM_END

ROM_START( a7800u2 )
	ROM_REGION(0x4000, "maincpu", ROMREGION_ERASEFF)
	ROM_SYSTEM_BIOS( 0, "a7800", "Atari 7800" )
	ROMX_LOAD("7800.u7", 0x3000, 0x1000, CRC(5d13730c) SHA1(d9d134bb6b36907c615a594cc7688f7bfcef5b43), ROM_BIOS(1) | ROM_OPTIONAL)
	ROM_SYSTEM_BIOS( 1, "a7800pr", "Atari 7800 (prototype with Asteroids)" )
	ROMX_LOAD("c300558-001a.u7", 0x0000, 0x4000, CRC(a0e10edf) SHA1(14584b1eafe9721804782d4b1ac3a4a7313e455f), ROM_BIOS(2) | ROM_OPTIONAL)
ROM_END

ROM_START( a7800p )
	ROM_REGION(0x4000, "maincpu", ROMREGION_ERASEFF)
	ROM_LOAD_OPTIONAL("7800pal.rom", 0x0000, 0x4000, CRC(d5b61170) SHA1(5a140136a16d1d83e4ff32a19409ca376a8df874))
ROM_END

ROM_START( a7800pu1 )
	ROM_REGION(0x4000, "maincpu", ROMREGION_ERASEFF)
	ROM_LOAD_OPTIONAL("7800pal.rom", 0x0000, 0x4000, CRC(d5b61170) SHA1(5a140136a16d1d83e4ff32a19409ca376a8df874))
ROM_END

ROM_START( a7800pu2 )
	ROM_REGION(0x4000, "maincpu", ROMREGION_ERASEFF)
	ROM_LOAD_OPTIONAL("7800pal.rom", 0x0000, 0x4000, CRC(d5b61170) SHA1(5a140136a16d1d83e4ff32a19409ca376a8df874))
ROM_END

/***************************************************************************
 DRIVER INIT
 ***************************************************************************/

DRIVER_INIT_MEMBER(a7800_state,a7800_ntsc)
{
	m_ispal = false;
	m_lines = 263;
	m_p1_one_button = 1;
	m_p2_one_button = 1;
}


DRIVER_INIT_MEMBER(a7800_state,a7800u1_ntsc)
{
	m_ispal = false;
	m_lines = 263;
	m_p1_one_button = 1;
	m_p2_one_button = 1;
}


DRIVER_INIT_MEMBER(a7800_state,a7800u2_ntsc)
{
	m_ispal = false;
	m_lines = 263;
	m_p1_one_button = 1;
	m_p2_one_button = 1;
}


DRIVER_INIT_MEMBER(a7800_state,a7800_pal)
{
	m_ispal = true;
	m_lines = 313;
	m_p1_one_button = 1;
	m_p2_one_button = 1;
}


DRIVER_INIT_MEMBER(a7800_state,a7800u1_pal)
{
	m_ispal = true;
	m_lines = 313;
	m_p1_one_button = 1;
	m_p2_one_button = 1;
}


DRIVER_INIT_MEMBER(a7800_state,a7800u2_pal)
{
	m_ispal = true;
	m_lines = 313;
	m_p1_one_button = 1;
	m_p2_one_button = 1;
}
/***************************************************************************
    GAME DRIVERS
***************************************************************************/

/*    YEAR  NAME      PARENT    COMPAT  MACHINE			INPUT  STATE         INIT			COMPANY   FULLNAME */
CONS( 1986, a7800,    0,        0,      a7800_ntsc,		a7800, a7800_state,  a7800_ntsc,	"Atari",  "Atari 7800 (NTSC) Cool", MACHINE_SUPPORTS_SAVE )
CONS( 1986, a7800u1,  a7800,    0,      a7800u1_ntsc,   a7800, a7800_state,  a7800u1_ntsc,	"Atari",  "Atari 7800 (NTSC) Warm", MACHINE_SUPPORTS_SAVE )
CONS( 1986, a7800u2,  a7800,    0,      a7800u2_ntsc,   a7800, a7800_state,  a7800u2_ntsc,	"Atari",  "Atari 7800 (NTSC) Hot", MACHINE_SUPPORTS_SAVE )
CONS( 1986, a7800p,   a7800,    0,      a7800_pal,		a7800, a7800_state,  a7800_pal,		"Atari",  "Atari 7800 (PAL) Cool", MACHINE_SUPPORTS_SAVE )
CONS( 1986, a7800pu1, a7800,    0,      a7800u1_pal,	a7800, a7800_state,  a7800u1_pal,	"Atari",  "Atari 7800 (PAL) Warm", MACHINE_SUPPORTS_SAVE )
CONS( 1986, a7800pu2, a7800,    0,      a7800u2_pal,	a7800, a7800_state,  a7800u2_pal,	"Atari",  "Atari 7800 (PAL) Hot", MACHINE_SUPPORTS_SAVE )
