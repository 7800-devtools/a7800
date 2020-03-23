// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/***************************************************************************

    Namco 54XX

    This custom chip is a Fujitsu MB8844 MCU programmed to act as a noise
    generator. It is used for explosions, the shoot sound in Bosconian,
    and the tire screech sound in Pole Position.

    CMD = command from main CPU
    OUTn = sound outputs (3 channels)

    The chip reads the command when the /IRQ is pulled down.

                       +------+
                     EX|1   28|Vcc
                      X|2   27|K3 (CMD7)
                 /RESET|3   26|K2 (CMD6)
            (OUT0.0) O0|4   25|K1 (CMD5)
            (OUT0.1) O1|5   24|K0 (CMD4)
            (OUT0.2) O2|6   23|R10/IRQ
            (OUT0.3) O3|7   22|R9/TC
            (OUT1.0) O4|8   21|R8
            (OUT1.1) O5|9   20|R7 (OUT2.3)
            (OUT1.2) O6|10  19|R6 (OUT2.2)
            (OUT1.3) O7|11  18|R5 (OUT2.1)
              (CMD0) R0|12  17|R4 (OUT2.0)
              (CMD1) R1|13  16|R3 (CMD3)
                    GND|14  15|R2 (CMD2)
                       +------+

    [1] The RNG that drives the type A output is output on pin 21, and
    the one that drives the type B output is output on pin 22, but those
    pins are not connected on the board.


    The command format is very simple:

    0x: nop
    1x: play sound type A
    2x: play sound type B
    3x: set parameters (type A) (followed by 4 bytes)
    4x: set parameters (type B) (followed by 4 bytes)
    5x: play sound type C
    6x: set parameters (type C) (followed by 5 bytes)
    7x: set volume for sound type C to x
    8x-Fx: nop

***************************************************************************/

#include "emu.h"
#include "namco54.h"

TIMER_CALLBACK_MEMBER( namco_54xx_device::latch_callback )
{
	m_latched_cmd = param;
}

READ8_MEMBER( namco_54xx_device::K_r )
{
	return m_latched_cmd >> 4;
}

READ8_MEMBER( namco_54xx_device::R0_r )
{
	return m_latched_cmd & 0x0f;
}

WRITE8_MEMBER( namco_54xx_device::O_w )
{
	uint8_t out = (data & 0x0f);
	if (data & 0x10)
		m_discrete->write(space, NAMCO_54XX_1_DATA(m_basenode), out);
	else
		m_discrete->write(space, NAMCO_54XX_0_DATA(m_basenode), out);
}

WRITE8_MEMBER( namco_54xx_device::R1_w )
{
	uint8_t out = (data & 0x0f);

	m_discrete->write(space, NAMCO_54XX_2_DATA(m_basenode), out);
}


TIMER_CALLBACK_MEMBER( namco_54xx_device::irq_clear )
{
	m_cpu->set_input_line(0, CLEAR_LINE);
}

WRITE8_MEMBER( namco_54xx_device::write )
{
	machine().scheduler().synchronize(timer_expired_delegate(FUNC(namco_54xx_device::latch_callback),this), data);

	m_cpu->set_input_line(0, ASSERT_LINE);

	// The execution time of one instruction is ~4us, so we must make sure to
	// give the cpu time to poll the /IRQ input before we clear it.
	// The input clock to the 06XX interface chip is 64H, that is
	// 18432000/6/64 = 48kHz, so it makes sense for the irq line to be
	// asserted for one clock cycle ~= 21us.
	machine().scheduler().timer_set(attotime::from_usec(21), timer_expired_delegate(FUNC(namco_54xx_device::irq_clear),this), 0);
}


/***************************************************************************
    DEVICE INTERFACE
***************************************************************************/

ROM_START( namco_54xx )
	ROM_REGION( 0x400, "mcu", 0 )
	ROM_LOAD( "54xx.bin",     0x0000, 0x0400, CRC(ee7357e0) SHA1(01bdf984a49e8d0cc8761b2cc162fd6434d5afbe) )
ROM_END

DEFINE_DEVICE_TYPE(NAMCO_54XX, namco_54xx_device, "namco54", "Namco 54xx")

namco_54xx_device::namco_54xx_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, NAMCO_54XX, tag, owner, clock),
	m_cpu(*this, "mcu"),
	m_discrete(*this, finder_base::DUMMY_TAG),
	m_basenode(0),
	m_latched_cmd(0)
{
}
//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void namco_54xx_device::device_start()
{
}

//-------------------------------------------------
// device_add_mconfig - add device configuration
//-------------------------------------------------

MACHINE_CONFIG_MEMBER( namco_54xx_device::device_add_mconfig )
	MCFG_CPU_ADD("mcu", MB8844, DERIVED_CLOCK(1,1))     /* parent clock, internally divided by 6 */
	MCFG_MB88XX_READ_K_CB(READ8(namco_54xx_device, K_r))
	MCFG_MB88XX_WRITE_O_CB(WRITE8(namco_54xx_device, O_w))
	MCFG_MB88XX_READ_R0_CB(READ8(namco_54xx_device, R0_r))
	MCFG_MB88XX_WRITE_R1_CB(WRITE8(namco_54xx_device, R1_w))
MACHINE_CONFIG_END

//-------------------------------------------------
//  device_rom_region - return a pointer to the
//  the device's ROM definitions
//-------------------------------------------------

const tiny_rom_entry *namco_54xx_device::device_rom_region() const
{
	return ROM_NAME(namco_54xx );
}
