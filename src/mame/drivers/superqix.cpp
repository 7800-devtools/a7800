// license:BSD-3-Clause
// copyright-holders:Mirko Buffoni, Nicola Salmoria, Tomasz Slanina
/***************************************************************************

Prebillian        (c) 1986 Taito
Hot Smash         (c) 1987 Taito
Super Qix         (c) 1987 Taito
Perestroika Girls (c) 1994 Promat (hack of Super Qix)

driver by Mirko Buffoni, Nicola Salmoria, Tomasz Slanina

Super Qix is a later revision of the hardware, featuring a bitmap layer that
is not present in the earlier games. It also has two 8910, while the earlier
games have one 8910 + a sample player.

Notes:
- All versions of the hardware have four 64kx4 DRAMs near the video roms;
  the gfx hardware draws to this as a backing framebuffer for sprites and
  background layer during display. This ram is not tested, or even testable
  by the z80. See http://www.jammarcade.net/tag/super-qix/
  Super Qix: 6S and 6R are the BG framebuffer
             5P and 5M are the Sprite framebuffer

Super Qix:
- The sq07.ic108 ROM came from a bootleg where the 8751 MCU was replaced by an
  8031 MCU plus an external ROM (i.e. the sqixb1 romset). The 8031 ROM was bad
  (bit 3 was stuck high). It was originally recovered by carefully checking
  the disassembly, and this repair was later verified from another dump of an
  intact chip to be correct. The majority of the bootleg 8031 MCU code matches
  the deprotected sqix b03__03.l2 MCU code, implying the sq07.ic108 8031 MCU
  code ROM was derived from code dumped from an original Taito b03__03.l2 8751
  MCU somehow.
  The bootleg MCU code is different from the original b03__03.l2 MCU since
  an 8031 when running in external ROM mode cannot use ports 0 or 2, hence
  the code was extensively patched by the bootleggers to avoid use of those
  ports, by adding an additional multiplexer to port 1, and moving various
  read and write pins around.
  An important note about the sqixb1 bootleg pcb: the SOCKET on the pcb for
  sq07.ic108 is populated backwards compared to the way the ROM will fit into
  it! This is probably the cause of the bad bit 3 in the original dump in MAME
  (due to someone inserting and powering the chip backwards) and is definitely
  the cause of at least one other ROM failure during a repair. Be aware of
  this, if you find or own one of these PCBs!

- The MCU sends some ID to the Z80 on startup, but the Z80 happily ignores it.
  This happens in all sets. There appears to be code that would check part of
  the MCU init sequence ($5973 onwards), but it doesn't seem to be called.

- sqixr0 (World/Japan V1.0) (and sqixb1 which is the exact same ROMs but the
  8751 MCU replaced with an 8031) has a bug with coin lockout: it is activated
  after inserting 10 coins instead of 9.
  This is fixed in World/Japan V1.1, V1.2 and the US set.
  In addition, the polarity of the coin lockout on V1.0 (and sqixb1) is
  flat-out reversed, so the pcb will not work in a standard JAMMA harness with
  coin lockouts without inverting JAMMA pins K and/or 9. The hack below on V1.0
  pcbs with the two wires connecting to IC 7H may have been a workaround which
  involved a customized JAMMA connector/harness. How exactly is unclear.

- All Taito Super Qix PCBS are part M6100237A, and have a wiring hack on top of
  component 7H (a 74LS86 Quad XOR gate):
  (reference: 74LS86 pins 4, 5 and 12 are 2A, 2B and 4A respectively, none are
  outputs)
  The V1.0 PCBs have two greenwires running on the back of the pcb, connected
  from 7H pins 4 and 5, to JAMMA pins e (GND) and d (unused) respectively.
  This implies there was a hack done on the JAMMA harness connector itself
  (possibly to invert the coin lockout value using one of the XOR gates
  at 7H (or perhaps 7H controls the coin lockouts themselves?)) but what
  exactly the hack does is unclear without further tracing.

  The V1.1, V1.2 and US PCBS have two resistors from VCC to GND forming a
  voltage divider on top of 7H, the resistor from VCC/Pin 14 to Common is
  22KOhms, the other resistor is also 22KOhms and seems to connect to
  GND/Pin 7. The center of the two resistors connects to one end of a 0.1uf
  capacitor and also to 7H pin 4, the other end of the capacitor connects to
  7H pin 12.
  This implies some sort of brief/reset pulse generation or filter on pin 12,
  or more likely some abuse of the TTL input hysteresis of the 74LS86 IC itself
  such that transitions of pin 12 cause transitions on pin 4 as well, or similar.
  Again, what exactly this accomplishes is unclear without further tracing.

- sqixb2 is a bootleg of sqixb1, with the MCU removed.

- Prebillian controls: (from the Japanese flyer):
  - pullout plunger for shot power (there's no on-screen power indicator in the game)
  - dial for aiming
  - button (fire M powerup, high score initials)


TODO:
- The way we generate NMI in sqix doesn't make much sense, but is a workaround
  for the slow gameplay you would otherwise get. Some interaction with vblank?

- I'm not sure about the NMI ack at 0418 in the original sqix, but the game hangs
  at the end of a game without it. Note that the bootleg replaces that call with
  something else. That something else is actually reading the system/Coin/Start
  inputs from 0418, which the MCU normally reads from its port 0, hence...
- Given the behavior of prebillian and hotsmash, I'm guessing 0418 resetting the
  NMI latch (i.e. NMI ACK) is correct. [LN]


Prebillian :
------------

PCB Layout (Prebillian)

 M6100211A
 -------------------------------------------------------------------
 |                    HM50464                                       |
 |  6                 HM50464                                       |
 |  5                 HM50464                               6116    |
 |  4                 HM50464                                       |
 |                                                                  |
 |                                                                  |
 |                                                               J  |
 |                                            68705P5 SW1(8)        |
 |               6264                                            A  |
 |                                              3     SW2(8)        |
 |                                                               M  |
 |                                                                  |
 |                                                               M  |
 |                                                                  |
 |                                   2                           A  |
 |                                                                  |
 |                                   1                              |
 |                                                                  |
 |                                   Z80B            AY-3-8910      |
 | 12MHz                                                            |
 --------------------------------------------------------------------

Notes:
       Vertical Sync: 60Hz
         Horiz. Sync: 15.67kHz
         Z80B Clock : 5.995MHz
     AY-3-8910 Clock: 1.499MHz



Hot (Vs) Smash :
----------------

Dips (not verified):

DSW1 stored @ $f236
76------ coin a
--54---- coin b
----3--- stored @ $f295 , tested @ $2a3b
------1- code @ $03ed, stored @ $f253 (flip screen)

DSW2 stored @ $f237
---4---- code @ $03b4, stored @ $f290
----32-- code @ $03d8, stored @ $f293 (3600/5400/2400/1200  -> bonus  ?)
------10 code @ $03be, stored @ $f291/92 (8,8/0,12/16,6/24,4 -> difficulty ? )

hotsmash notes for 408-41f area, related to above
code at z80:0070:
 set bit 3 at ram address f253 (was 0x00, now 0x08)
 read ram address f253 to 'a' register
 set bc to 0410, write 'a' register (0x08) to bc

code at z80:0093:
 set bc to 0418, read from bc and ignore result
 set bit 4 at ram address f253 (was 0x08, now 0x18)
 read ram address f253 to 'a' register
 set bc to 0410, write 'a' register (0x18) to bc


***************************************************************************/

#include "emu.h"
#include "includes/superqix.h"

#include "cpu/z80/z80.h"
#include "cpu/mcs51/mcs51.h"
#include "sound/ay8910.h"
#include "screen.h"
#include "speaker.h"


SAMPLES_START_CB_MEMBER(hotsmash_state::pbillian_sh_start)
{
	// convert 8-bit unsigned samples to 8-bit signed
	m_samplebuf = std::make_unique<int16_t[]>(m_samples_region.length());
	for (unsigned i = 0; i < m_samples_region.length(); i++)
		m_samplebuf[i] = s8(m_samples_region[i] ^ 0x80) * 256;
}

WRITE8_MEMBER(hotsmash_state::pbillian_sample_trigger_w)
{
	//logerror("sample trigger write of %02x\n", data);

	// look for end of sample marker
	unsigned start = data << 7;
	unsigned end = start;
	while ((end < m_samples_region.length()) && (m_samples_region[end] != 0xff))
		end++;

	m_samples->start_raw(0, m_samplebuf.get() + start, end - start, XTAL_12MHz/3072); // needs verification, could be 2048 and 4096 alternating every sample
}

/**************************************************************************

Super Qix Z80 <-> 8751 communication

This is quite hackish, because the communication protocol is not very clear.

The Z80 acts this way:
- wait for 8910 #0 port B, bit 6 to be 0
- write command for MCU to 8910 #1 port B
- read port 0408
- wait for 8910 #0 port B, bit 6 to be 1
- read answer from MCU from 8910 #1 port B
- read port 0408

also, in other places it waits for 8910 #0 port B, bit 7 to be 0

The MCU acts this way:
- write FF to latch
- fiddle with port 1
- wait for IN2 bit 7 to be 1
- read command from latch
- process command
- fiddle with port 1
- write answer to latch
- wait for IN2 bit 7 to be 1

**************************************************************************/

CUSTOM_INPUT_MEMBER(superqix_state_base::superqix_semaphore_input_r) // similar to pbillian_semaphore_input_r below, but reverse order and polarity
{
	int res = 0;

	if (m_MCUHasWritten)
		res |= 0x01;

	if (m_Z80HasWritten)
		res |= 0x02;

	return res;
}

READ8_MEMBER(superqix_state_base::in4_mcu_r)
{
//  logerror("%04x: in4_mcu_r\n",space.device().safe_pc());
	//logerror("%04x: ay_port_b_r and MCUHasWritten is %d and Z80HasWritten is %d: ",static_cast<device_t &>(*m_maincpu).safe_pc(),m_MCUHasWritten, m_Z80HasWritten);
	uint8_t temp = ioport("P2")->read();
	//logerror("returning %02X\n", temp);
	return temp;
}

READ8_MEMBER(superqix_state_base::sqix_from_mcu_r)
{
//  logerror("%04x: read mcu answer (%02x)\n",space.device().safe_pc(),m_fromMCU);
	return m_fromMCU;
}

TIMER_CALLBACK_MEMBER(superqix_state::mcu_acknowledge_callback)
{
	/* if we're on a set with no mcu, namely sqixb2, perestro or perestrof,
	   do not set the mcu flags since at least a few checks in sqixb2 were
	   not patched out by the bootleggers nor the read from the
	   mcu_acknowledge_r register which sets the m_Z80HasWritten semaphore,
	   hence the semaphore flags must both be hard-wired inactive on the pcb,
	   or else it will never boot to the title screen.
	   perestro and perestrof seem to completely ignore the semaphores.
	 */
	if (m_mcu.found()) m_Z80HasWritten = 1; // only set this if we have an actual mcu
	m_fromZ80 = m_fromZ80pending;
//  logerror("Z80->MCU %02x\n",m_fromZ80);
}

READ8_MEMBER(superqix_state::mcu_acknowledge_r)
{
	if(!machine().side_effect_disabled())
	{
		machine().scheduler().synchronize(timer_expired_delegate(FUNC(superqix_state::mcu_acknowledge_callback), this));
	}
	return 0;
}

WRITE8_MEMBER(superqix_state_base::sqix_z80_mcu_w)
{
//  logerror("%04x: sqix_z80_mcu_w %02x\n",space.device().safe_pc(),data);
	m_fromZ80pending = data;
}

WRITE8_MEMBER(superqix_state_base::bootleg_mcu_p1_w)
{
	switch ((data & 0x0e) >> 1)
	{
		case 0:
			// ???
			break;
		case 1:
			machine().bookkeeping().coin_counter_w(0,data & 1);
			break;
		case 2:
			machine().bookkeeping().coin_counter_w(1,data & 1);
			break;
		case 3:
			machine().bookkeeping().coin_lockout_global_w((data & 1) ^ m_invert_coin_lockout);
			break;
		case 4:
			flip_screen_set(data & 1);
			break;
		case 5:
			m_port1 = data;
			if ((m_port1 & 0x80) == 0)
			{
				m_port3_latch = m_port3;
			}
			break;
		case 6:
			m_MCUHasWritten = 0; // ????
			break;
		case 7:
			if ((data & 1) == 0)
			{
//              logerror("%04x: MCU -> Z80 %02x\n",space.device().safe_pc(),m_port3);
				m_fromMCU = m_port3_latch;
				m_MCUHasWritten = 1;
				m_Z80HasWritten = 0; // ????
			}
			break;
	}
}

WRITE8_MEMBER(superqix_state_base::mcu_p3_w)
{
	m_port3 = data;
}

READ8_MEMBER(superqix_state_base::bootleg_mcu_p3_r)
{
	if ((m_port1 & 0x10) == 0)
	{
		return ioport("DSW1")->read();
	}
	else if ((m_port1 & 0x20) == 0)
	{
		return sqix_system_status_r(space, 0);
	}
	else if ((m_port1 & 0x40) == 0)
	{
		if(!machine().side_effect_disabled())
		{
			//logerror("%04x: read Z80 command %02x\n",space.device().safe_pc(),m_fromZ80);
			m_Z80HasWritten = 0;
		}
		return m_fromZ80;
	}
	return 0;
}

READ8_MEMBER(superqix_state_base::sqix_system_status_r)
{
	return ioport("SYSTEM")->read();
}

WRITE8_MEMBER(superqix_state_base::sqixu_mcu_p2_w)
{
	// bit 0 = enable latch for bits 1-6 below on high level or falling edge (doesn't particularly matter which, either one works)

	// bit 1 = coin cointer 1
	machine().bookkeeping().coin_counter_w(0,data & 2);

	// bit 2 = coin counter 2
	machine().bookkeeping().coin_counter_w(1,data & 4);

	// bit 3 = coin lockout
	machine().bookkeeping().coin_lockout_global_w(((data & 8)>>3) ^ m_invert_coin_lockout);

	// bit 4 = flip screen
	flip_screen_set(data & 0x10);

	// bit 5 = unknown (set on startup)

	// bit 6 = unknown
	if ((data & 0x40) == 0)
		m_MCUHasWritten = 0; // ????

	// bit 7 = clock latch from port 3 to Z80
	if ((m_port2 & 0x80) != 0 && (data & 0x80) == 0)
	{
//      logerror("%04x: MCU -> Z80 %02x\n",space.device().safe_pc(),m_port3);
		m_fromMCU = m_port3;
		m_MCUHasWritten = 1;
		m_Z80HasWritten = 0; // ????
	}

	m_port2 = data;
}

READ8_MEMBER(superqix_state_base::sqixu_mcu_p3_r)
{
//  logerror("%04x: read Z80 command %02x\n",space.device().safe_pc(),m_fromZ80);
	if(!machine().side_effect_disabled())
	{
		m_Z80HasWritten = 0;
	}
	return m_fromZ80;
}


READ8_MEMBER(superqix_state_base::nmi_ack_r)
{
	if(!machine().side_effect_disabled())
	{
		m_maincpu->set_input_line(INPUT_LINE_NMI, CLEAR_LINE);
	}
	return sqix_system_status_r(space, 0);
}

READ8_MEMBER(superqix_state_base::bootleg_in0_r)
{
	return BITSWAP8(ioport("DSW1")->read(), 0,1,2,3,4,5,6,7);
}

WRITE8_MEMBER(superqix_state_base::bootleg_flipscreen_w)
{
	flip_screen_set(~data & 1);
}


/***************************************************************************

 Hot Smash Z80 <-> 68705 protection interface

 High level commands; these commands are parsed by the MCU from the z80->mcu
 register when the MCU's /INT pin is activated, which seems to occur on a
 write to the Z80->MCU register by the Z80.

 MCU Commands Legend (hotsmash)
 0x00 - Reset MCU (jumps to reset vector; does not return anything or set mcu->z80 semaphore)
 0x01 - Read Spinner Position Counter for Player 1 (p1, bits 2 and 3 quadrature, counter range is clamped to 00-7f) OR Protection Read
        Protection reads are reads of a variable length of a rom extending from MCU rom 0x80 to 0xff, and are only every even byte;
        the strange values returned by the protection functions below are actually the raw rom offset in mcu rom where the reads will come from.
        The first byte of each read is checked if it is >= or < 0x32, if it is >= it is thrown out and the next byte is ignored. if it is <, it is thrown out,
        and the next byte is returned instead of reading spinner 1. In the case where the byte of the rom WOULD BE 0xFF, instead based on the LSB of the spinner
        counter value (effectively a random coin flip) 0x8A or 0x8B is returned. This protection value might actually be the ball speed or AI aggressiveness
        per level after a certain number of ball hits/points scored, as it always increases, to a limit.
 0x02 - Read Spinner Position Counter for Player 2 (p2, bits 3 and 2 quadrature, counter range is clamped to 00-7f)
 0x04 - Read dipswitch array sw1 and send to z80
 0x08 - Read dipswitch array sw2 and send to z80 and also write them to $3b
 0x20 - Reset quadrature counters to 0x38, clears protection read enable flag, return 0x38; unlike all other return values, this is not an offset into mcu rom.
 0x40 - Reset score and start 1P vs CPU game; returns number of points per game win based on sw2:3; clears counters and clears $3a
 0x41 - Reset score and start 2P game; returns number of points per game win based on sw2:3; clears counters and sets $3a bit 0
 0x80 - Increment score for CPU/P2, reset protection read suboffset and set protection read enable flag
        If in a 2P game, if p2 scored more than sw2.5?4:3 points, and won 2 matches, clear matches won by both players and return 0xb3
        If in a 2P game, if p2 scored more than sw2.5?4:3 points, and did not yet win 2 matches, return 0x9d
        If in a 2P game, if p2 did not yet score more than sw2.5?4:3 points, return 0x89
        If in a 1P game, if cpu scored more than sw2.5?4:3 points, return 0xee
        If in a 1P game, if cpu did not yet score more than sw2.5?4:3 points, return 0xd9
 0x81 - Increment score for P1, reset protection read suboffset and set protection read enable flag
        If in a 2P game, if p1 scored more than sw2.5?4:3 points, and won 2 matches, clear matches won by both players and return 0xa8
        If in a 2P game, if p1 scored more than sw2.5?4:3 points, and did not yet win 2 matches, return 0x92
        If in a 2P game, if p1 did not yet score more than sw2.5?4:3 points, return 0x80
        If in a 1P game, if p1 scored more than sw2.5?4:3 points, and won 2 matches, clear matches won by both players and return 0xe2
        If in a 1P game, if p1 scored more than sw2.5?4:3 points, and did not yet win 2 matches, return 0xe2
        If in a 1P game, if p1 did not yet score more than sw2.5?4:3 points, return 0xd0
 0x83 - Increment score for BOTH PLAYERS, reset protection read offset and set protection read enable flag, return 0xbe
 0x84 - Reset protection read suboffset and set protection read enable flag, return 0xc9
 0xF0 - Reset protection read suboffset, return 0xf0 (sent on mcu timeout from z80 side?)
 other - Echo (returns whatever the command byte was back to the z80 immediately)

 MCU Commands Detail:
 0x00 - Reset MCU (jumps to reset vector; does not return anything or set mcu->z80 semaphore)
 0x01 - Read Spinner Position Counter for Player 1 (p1, bits 2 and 3 quadrature, counter range is clamped to 00-7f) OR Protection Read
    if $31 has bit 1 set
    jump to 239
        increment $32
        load a with $34
        add $33 to a
        transfer a to x
        load a with $x
        if a is 0
        jump to 247
            jump to 204, see below
        if a < 0x32
        jump to 24a
            increment $33
            load a with $34
            add $33 to a
            transfer a to x
            load a with $x
            if a == 0xFF
            jump to 25b
                load x with 0x10
                load a with [x+1] which is 0x11 (spinner 1 value)
                store a (value of $11) into $2a
                if $2a has bit 0 set
                jump to 269
                    load a with 0x8B
                    store a (value of $34) into $2e
                    store $2e to the mcu->z80 latch
                otherwise
                    load a with 0x8A
                    store a (value of $34) into $2e
                    store $2e to the mcu->z80 latch
            otherwise jump to 22a <- I believe this path is the 'protection succeeded' path, as it allows an offset of the first 256 bytes of the mcu rom to be read...
                store accum into $2e
                store $2e to the mcu->z80 latch
        otherwise jump to 204, see below
    204:
        load x with 0x10
        load a with [x+1] which is 0x11 (spinner 1 value)
        store a (value of $11) into $2e
        store $2e to the mcu->z80 latch
 0x02 - Read Spinner Position Counter for Player 2 (p2, bits 3 and 2 quadrature, counter range is clamped to 00-7f)
    load x with 0x1a
    load accum with [x+1] which is 0x1b (spinner 2 value)
    store accum (value of $1b) into $2e
    store $2e to the mcu->z80 latch
 0x04 - Read dipswitch array sw1 and send to z80
 0x08 - Read dipswitch array sw2 and send to z80 and also write them to $3b
    same as command 0x04, but polls the other port, and also stores the result to $3b
 0x20 - Reset quadrature counters to 0x38, clears protection read enable flag, return 0x38
    load a with 0x38
    load x with 0x10
    store a to $11
    load x with 0x1a
    store a to $1b
    clear $31
    store a into $2e
    store $2e to the mcu->z80 latch
 0x40 Reset score and start 1P vs CPU game; returns number of points per game win based on sw2:3; clears counters and clears $3a
 0x41 Reset score and start 2P game; returns number of points per game win based on sw2:3; clears counters and sets $3a bit 0
    clear $32
    clear $35
    clear $36
    clear $37
    clear $38
    if $3b has bit 4 clear (number of points per game dipswitch is set to 3)
    jump to 29c
        store 0x03 in $39
        goto in all cases:
    otherwise (number of points per game dipswitch is set to 4)
        store 0x04 in $39
        goto in all cases:
    in all cases:
****if command was 0x40: clears $31, $3a,
****if command was 0x41: clears $31, sets bit 0x01 of $3a
    store accum (value of $39) into $2e
    store $2e to the mcu->z80 latch
 0x80 - Increment score for CPU/P2, reset protection read suboffset and set protection read enable flag
    if $3a has bit 0x01 set
    jump to 2ad
        set bit 1 of $31
        clear $33
        increment $36
        check if $36 == $39, if so
        jump to 2c2
            clear $36
            clear $35
            increment $38
            check if $38 == 0x02, if so
            jump to 2d7
                clear $38
                clear $37
                store 0xb3 into $34
                clear $32
                store a (value of $34) into $2e
                store $2e to the mcu->z80 latch
            otherwise
                store 0x9d into $34
                clear $32
                store a (value of $34) into $2e
                store $2e to the mcu->z80 latch
        otherwise
            store 0x89 into $34
            clear $32
            store a (value of $34) into $2e
            store $2e to the mcu->z80 latch
    otherwise jump to 31b <- we're in a 1p game
        set bit 1 of $31
        clear $33
        increment $36
        check if $36 == $39, if so
        jump to 330
            store 0xee into $34
            clear $32
            store a (value of $34) into $2e
            store $2e to the mcu->z80 latch
        otherwise
            store 0xd9 into $34
            clear $32
            store a (value of $34) into $2e
            store $2e to the mcu->z80 latch
 0x81 - Increment score for P1, reset protection read suboffset and set protection read enable flag
    if $3a has bit 0x01 set
    jump to 2e4
        set bit 0x01 of $31
        clear $33
        increment $35
        check if $35 == $39, if so
        jump to 2f9
            clear $36
            clear $35
            increment $37
            check if $37 == 0x02, if so
            jump to 30e
                clear $38
                clear $37
                store 0xA8 into $34
                clear $32
                store a (value of $34) into $2e
                store $2e to the mcu->z80 latch
            otherwise
                store 0x92 into $34
                clear $32
                store a (value of $34) into $2e
                store $2e to the mcu->z80 latch
        otherwise
            store 0x80 to $34
            clear $32
            store a (value of $34) into $2e
            store $2e to the mcu->z80 latch
    otherwise jump to 339
        set bit 0x01 of $31
        clear $33
        increment $35
        check if $35 == $39, if so
        jump to 34e
            clear $35
            clear $36
            increment $37
            check if $37 == 0x02
            if so jump to 363
                clear $37
                clear $38
                store 0xE2 into $34
                clear $32
                store a (value of $34) into $2e
                store $2e to the mcu->z80 latch
            otherwise
                store 0xE2 into $34
                clear $32
                store a (value of $34) into $2e
                store $2e to the mcu->z80 latch
        otherwise
            store 0xd0 to $34
            clear $32
            store a (value of $34) into $2e
            store $2e to the mcu->z80 latch
 0x83 - Increment score for BOTH PLAYERS, reset protection read offset and set protection read enable flag, return 0xbe
    increment $35
    increment $36
    store 0xbe to $34
    clear $32
    clear $33
    set bit 1 of $31
    store a (value of $34) into $2e
    store $2e to the mcu->z80 latch
 0x84 - Reset protection read suboffset and set protection read enable flag, return 0xc9
    store 0xc9 to $34
    clear $32
    clear $33
    set bit 1 of $31
    store a (value of $34) into $2e
    store $2e to the mcu->z80 latch
 0xF0 - Reset protection read suboffset, return 0xf0 (sent on mcu timeout from z80 side?)
    clear $32
    clear $33
    store a (0xF0) into $2e
    store $2e to the mcu->z80 latch
 other - Echo (returns whatever the command byte was back to the z80 immediately)

 MCU idle quadrature read loop starts at 165
 MCU reset vector is 120
 The block of code between 100 and 120 is unknown.

 MCU memory addresses known:
 10 - cleared by reset, holds the player 1 raw quadrature inputs as last read in bits 2 and 3
 11 - cleared by reset, holds the player 1 spinner position counter, clamped between 0x00 and 0x7f
 13 - cleared by reset, never used by mcu code (leftover from prebillian?)
 19 - cleared by reset, never used by mcu code (leftover from prebillian?)
 1a - cleared by reset, holds the player 2 raw quadrature inputs as last read in bits 3 and 2
 1b - cleared by reset, holds the player 2 spinner position counter, clamped between 0x00 and 0x7f
 1d - cleared by reset, never used by mcu code (leftover from prebillian?)
 21 - cleared by reset, never used by mcu code (leftover from prebillian?)
 23 - cleared by reset, never used by mcu code (leftover from prebillian?)
 24 - set to 0x10 by reset, never used by mcu code (leftover from prebillian?)
 2d ? definitely used, not sure where.
 2e - temporary storage for returned 'state' values for z80
 2f - cleared by reset, holds a copy of 10 or 1a, used for quadrature decode second read
 30 - cleared by reset, holds a copy of 10 or 1a, used for quadrature decode first read
 31 - bit 1: this is only set AFTER the first ball of the game has been played, and enables protection reads
 32 - some sort of running counter; this seems to increment once every mcu poll for spinner positions; may have been intended as some sort of protection watchdog, but doesn't seem to be read anywhere?
 33 - sub-offset for protection reads (full offset is created by adding 0x33 and 0x34, when protection read enable flag $31.1 is set); incremented twice per read
 34 - holds 'state' response for most commands, actually an offset into the rom at 0x80-0xFF and is the primary offset for protection reads
 35 - number of points scored by Player 1
 36 - number of points scored by Player 2/CPU
 37 - number of matches won by Player 1 in a 1P/CPU game or VS Game
 38 - number of matches won by Player 2 in a VS Game
 39 - number of points being played for in total (3 or 4, based on sw2:5 dipswitch)
 3a - bit 0: if set: 2P/VS game; if clear: 1P/CPU game
 3b - contents of dipswitch 2; bit 0x10 (switch 5?) affects the value loaded to 39

The Prebillian/Hotsmash hardware seems to be an evolution of the arkanoid hardware in regards to the mcu:
arkanoid:
Port A[7:0] <> bidir comms with z80
Port B[7:0] <- input MUX (where does the paddle select bit come from??? port a bit 0?)
PortC[0] <- m_Z80HasWritten
PortC[1] <- m_MCUHasWritten
PortC[2] -> high - clear m_Z80HasWritten and deassert MCU /INT; low - allow m_fromZ80 to be read at port A
PortC[3] -> high - latch port A contents into m_fromMCU and set m_MCUHasWritten; low - do nothing.

hotsmash/prebillian:
PortA[] <- input MUX
PortB[] -> output MUX
PortC[3:0] -> select one of 8 MUX selects for m_porta_in and m_portb_out
PortC[4] -> activates m_porta_in latch (active low)

 *  Port C connections:
 *
 *  0-2 W  select I/O; inputs are read from port A, outputs are written to port B
 *         000  dsw A (I)
 *         001  dsw B (I)
 *         010  not used
 *         011  from Z80 (I)
 *         100  not used
 *         101  to Z80 (O)
 *         110  P1 dial input (I)
 *         111  P2 dial input (I)
 *  3   W  clocks the active latch (active low)
 *  4-7 W  nonexistent on 68705p5

 ***************************************************************************/

/**************************************************************************

 Prebillian MCU info

Seems to act like an older version of hotsmash mcu code, the quadrature code is much messier here than in hotsmash

 MCU Commands Legend (prebillian)
 0x00 - Reset MCU
 0x01 - Read Plunger Position Counter for Player 1 or 2 (p1 plunger, bits UNKNOWN (2 and 3?) quadrature) OR (p2 plunger, bits UNKNOWN (0 and 1?) quadrature); counter range is 00-FF???
 0x02 - Read Spinner Position Counter for Player 1 or 2 (p1 spinner, bits UNKNOWN (3 and 2?) quadrature) OR (p2 spinner, bits UNKNOWN (0 and 1?) quadrature); counter range is 00-FF and wraps
 0x04 - Read dipswitch array sw1 and send to z80
 0x08 - Read dipswitch array sw2 and send to z80
 0x80 - Set commands 00 and 01 to return player 1 controls, return nothing
 0x81 - Set commands 00 and 01 to return player 2 controls, return nothing
 other - do nothing, return nothing
 Disabled/dead code MCU commands (can be enabled by patching MCU rom 0x1BA to 0x9D)
  0x03 - return mcu timer, and latch the current command (0x03) (or another byte if you write one VERY fast) to add to an accumulator
  0x0A - return the accumulator from command 0x03
  0x13 - return currently selected player number (bit0=0 for player 1, bit0=1 for player 2; upper 7 bits are a counter of how many times more or less command 80 or 81 was run; 80 increments, 81 decrements)
  0x10 - protection scramble; immediately latch the current command (0x10) (or another byte if you write one VERY fast) and do some rotates and scrambling of the value an XORing it against the prior value, and return it. This is affected by the carry flag if something else set it.

**************************************************************************/

/*
 * This wrapper routine is necessary because the dial is not connected to an
 * hardware counter as usual, but the DIR and CLOCK inputs are directly
 * connected to the 68705 which acts as a counter.
 * both hotsmash and prebillian have two dials connected this way.
 * on hotsmash only, the second player dial has the DIR and CLOCK inputs swapped
 * prebillian also has two plungers, which are connected via a standard quadrature hookup.
 * though the plungers are spring-loaded and return to one extreme when released.
 * prebillian also has a launch button which will instantly launch the ball;
 * Whether this is a secondary trigger at the innermost position of the plunger
 * (in case the quadrature is fouled and/or the plunger is slammed inward), a separate
 * panel button, or a debug button left over from development is up to debate.
 */

int hotsmash_state::read_inputs(int player) // if called with player=1, we're mux port 7, otherwise mux port 6
{
	// get the new position and adjust the result
	// dials use DIR and CLOCK?
	int const newpos_dial = m_dials[player]->read();
	// get the launch button state
	int const launchbtn_state = m_launchbtns[player]->read()&1;
	if (newpos_dial != m_dial_oldpos[player])
	{
		m_dial_sign[player] = ((newpos_dial - m_dial_oldpos[player]) & 0x80) >> 7;
		m_dial_oldpos[player] = newpos_dial;
	}
	// plungers use a plain old quadrature
	// quad1 = plunger bit 1
	// quad2 = plunger bit 0 XOR plunger bit 1
	int const newpos_plunger = m_plungers[player]->read();

	if ((player == 0) || (m_invert_p2_spinner == false))
		return (launchbtn_state<<4 | ((m_dial_oldpos[player] & 1) << 2) | (m_dial_sign[player] << 3) | (newpos_plunger&2) | ((newpos_plunger^(newpos_plunger>>1))&1) );
	else    // (player == 1) && (m_invert_p2_spinner == true)
		return (launchbtn_state<<4 | ((m_dial_oldpos[player] & 1) << 3) | (m_dial_sign[player] << 2) | (newpos_plunger&2) | ((newpos_plunger^(newpos_plunger>>1))&1) );
}

WRITE8_MEMBER(hotsmash_state::hotsmash_68705_portB_w)
{
	m_portB_out = data;
}

WRITE8_MEMBER(hotsmash_state::hotsmash_68705_portC_w)
{
	u8 const changed_m_portC_out = m_portC_out ^ data;
	m_portC_out = data;
	//logerror("%04x: MCU setting MUX port to %d\n", space.device().safe_pc(), m_portC_out & 0x07);
	// maybe on the RISING edge of the latch bit, the semaphores are updated, like TaitoSJ?
	/*if (BIT(changed_m_portC_out, 3) && BIT(m_portC_out, 3))
	{
	    switch (m_portC_out & 0x07)
	    {
	    case 0x03:
	        m_Z80HasWritten = 0;
	        break;
	    case 0x05:
	        m_MCUHasWritten = 1;
	        break;
	    default:
	        break;
	    }
	}*/
	// on the falling edge of the latch bit, update port A and (if applicable) m_portB_out latches
	if (BIT(changed_m_portC_out, 3) && !BIT(m_portC_out, 3))
	{
		switch (m_portC_out & 0x07)
		{
		case 0x0:   // dsw A
		case 0x1:   // dsw B
			m_mcu->pa_w(space, 0, m_dsw[m_portC_out & 0x01]->read());
			break;

		case 0x3:   // Read command from Z80 to MCU, the z80->mcu semaphore is cleared on the rising edge
			//logerror("%04x: command %02x read by MCU; Z80HasWritten: %d (and will be 0 after this); MCUHasWritten: %d\n",space.device().safe_pc(),m_fromZ80,m_Z80HasWritten, m_MCUHasWritten);
			m_mcu->set_input_line(M68705_IRQ_LINE, CLEAR_LINE);
			m_mcu->pa_w(space, 0, m_fromZ80);
			m_Z80HasWritten = 0;
			break;

		case 0x5:   // latch response from MCU to Z80; the mcu->z80 semaphore is set on the rising edge
			m_fromMCU = m_portB_out;
			//logerror("%04x: response %02x written by MCU; Z80HasWritten: %d; MCUHasWritten: %d (and will be 1 after this)\n",space.device().safe_pc(),m_fromMCU,m_Z80HasWritten, m_MCUHasWritten);
			m_MCUHasWritten = 1;
			break;

		case 0x6:
		case 0x7:
			m_mcu->pa_w(space, 0, read_inputs(m_portC_out & 0x01));
			break;

		default: // cases 2 and 4 presumably latch open bus/0xFF; implication from the superqix bootleg is that reading port 4 may clear the m_MCUHasWritten flag, but the hotsmash MCU never touches it. Needs hardware tests/tracing to prove.
			logerror("%04x: MCU attempted to read mux port %d which is invalid!\n", space.device().safe_pc(), m_portC_out & 0x07);
			m_mcu->pa_w(space, 0, 0xff);
			break;
		}
		//if ((m_portC_out & 0x07) < 6) logerror("%04x: MCU latched %02x from mux input %d m_portA_in\n", space.device().safe_pc(), m_portA_in, m_portC_out & 0x07);
	}
}

WRITE8_MEMBER(hotsmash_state::hotsmash_Z80_mcu_w)
{
	m_fromZ80 = data;
	//if ((m_fromZ80 != 0x04) && (m_fromZ80 != 0x08))
	//  logerror("%04x: z80 write to MCU %02x; Z80HasWritten: %d (and will be 1 after this); MCUHasWritten: %d\n",space.device().safe_pc(),m_fromZ80, m_Z80HasWritten, m_MCUHasWritten);
	m_Z80HasWritten = 1; // set the semaphore, and assert interrupt on the mcu
	machine().scheduler().boost_interleave(attotime::zero, attotime::from_usec(250)); //boost the interleave temporarily, or the game will crash.
	m_mcu->set_input_line(M68705_IRQ_LINE, ASSERT_LINE);
}

READ8_MEMBER(hotsmash_state::hotsmash_Z80_mcu_r)
{
	if(!machine().side_effect_disabled())
	{
		//if ((m_fromZ80 != 0x04) && (m_fromZ80 != 0x08))
		//  logerror("%04x: z80 read from MCU %02x; Z80HasWritten: %d; MCUHasWritten: %d (and will be 0 after this)\n",space.device().safe_pc(),m_fromMCU, m_Z80HasWritten, m_MCUHasWritten);
		m_MCUHasWritten = 0;
	}
	// return the last value the 68705 wrote, but do not mark that we've read it
	return m_fromMCU;
}

CUSTOM_INPUT_MEMBER(hotsmash_state::pbillian_semaphore_input_r)
{
	ioport_value res = 0;
	// bit 0x40 is PROBABLY latch 1 on 74ls74.7c, is high if m_Z80HasWritten is clear
	if (!m_Z80HasWritten)
		res |= 0x01;

	// bit 0x80 is PROBABLY latch 2 on 74ls74.7c, is high if m_MCUHasWritten is clear
	// prebillian code at 0x6771 will wait in a loop reading ay port E forever and waiting
	// for bit 7 to be clear before it will read from the mcu
	if (!m_MCUHasWritten)
		res |= 0x02;
	return res;
}


void superqix_state_base::machine_init_common()
{
	// MCU HLE and/or 8751 related
	save_item(NAME(m_port1));
	save_item(NAME(m_port2));
	save_item(NAME(m_port3));
	save_item(NAME(m_port3_latch));
	save_item(NAME(m_fromZ80pending));

	// commmon 68705/8751/HLE
	save_item(NAME(m_MCUHasWritten));
	save_item(NAME(m_Z80HasWritten));
	save_item(NAME(m_fromMCU));
	save_item(NAME(m_fromZ80));

	//general machine stuff
	save_item(NAME(m_invert_coin_lockout));
	save_item(NAME(m_invert_p2_spinner));
	save_item(NAME(m_nmi_mask));

	// superqix specific stuff
	save_item(NAME(m_gfxbank));
	save_item(NAME(m_show_bitmap));
	// the following are saved in VIDEO_START_MEMBER(superqix_state,superqix):
	//save_item(NAME(*m_fg_bitmap[0]));
	//save_item(NAME(*m_fg_bitmap[1]));
}

void hotsmash_state::machine_init_common()
{
	superqix_state_base::machine_init_common();

	// 68705 related
	save_item(NAME(m_portB_out));
	save_item(NAME(m_portC_out));

	// spinner quadrature stuff
	save_item(NAME(m_dial_oldpos));
	save_item(NAME(m_dial_sign));
}

MACHINE_START_MEMBER(superqix_state_base, superqix)
{
	/* configure the banks */
	membank("bank1")->configure_entries(0, 4, memregion("maincpu")->base() + 0x10000, 0x4000);

	machine_init_common();
}

MACHINE_START_MEMBER(hotsmash_state, pbillian)
{
	/* configure the banks */
	membank("bank1")->configure_entries(0, 2, memregion("maincpu")->base() + 0x10000, 0x4000);

	machine_init_common();
}


static ADDRESS_MAP_START( main_map, AS_PROGRAM, 8, superqix_state_base )
	AM_RANGE(0x0000, 0x7fff) AM_ROM
	AM_RANGE(0x8000, 0xbfff) AM_ROMBANK("bank1")
	// the following four ranges are part of a single 6264 64Kibit SRAM chip, called 'VRAM' in POST
	AM_RANGE(0xe000, 0xe0ff) AM_RAM AM_SHARE("spriteram")
	AM_RANGE(0xe100, 0xe7ff) AM_RAM
	AM_RANGE(0xe800, 0xefff) AM_RAM_WRITE(superqix_videoram_w) AM_SHARE("videoram")
	AM_RANGE(0xf000, 0xffff) AM_RAM
ADDRESS_MAP_END

static ADDRESS_MAP_START( pbillian_port_map, AS_IO, 8, hotsmash_state ) // used by both pbillian and hotsmash
	AM_RANGE(0x0000, 0x01ff) AM_RAM_DEVWRITE("palette", palette_device, write) AM_SHARE("palette") // 6116 sram near the jamma connector, "COLOR RAM" during POST
	//AM_RANGE(0x0200, 0x03ff) AM_RAM // looks like leftover crap from a dev board which had double the color ram? zeroes written here, never read.
	AM_RANGE(0x0401, 0x0401) AM_DEVREAD("aysnd", ay8910_device, data_r) // ay i/o ports connect to "SYSTEM" and "BUTTONS" inputs which includes mcu semaphore flags
	AM_RANGE(0x0402, 0x0403) AM_DEVWRITE("aysnd", ay8910_device, data_address_w)
	AM_RANGE(0x0408, 0x0408) AM_READWRITE(hotsmash_Z80_mcu_r, hotsmash_Z80_mcu_w)
	AM_RANGE(0x0410, 0x0410) AM_WRITE(pbillian_0410_w) /* Coin Counters, ROM bank, NMI enable, Flipscreen */
	AM_RANGE(0x0418, 0x0418) AM_READ(nmi_ack_r)
	AM_RANGE(0x0419, 0x0419) AM_WRITENOP // ??? is this a watchdog, or something else? manual reset of mcu semaphores? manual nmi TRIGGER? used by prebillian
	AM_RANGE(0x041a, 0x041a) AM_WRITE(pbillian_sample_trigger_w)
	AM_RANGE(0x041b, 0x041b) AM_READNOP  // input related? but probably not used, may be 'sample has stopped playing' flag? used by prebillian
ADDRESS_MAP_END

static ADDRESS_MAP_START( sqix_port_map, AS_IO, 8, superqix_state )
	AM_RANGE(0x0000, 0x00ff) AM_RAM_DEVWRITE("palette", palette_device, write) AM_SHARE("palette")
	AM_RANGE(0x0401, 0x0401) AM_DEVREAD("ay1", ay8910_device, data_r)
	AM_RANGE(0x0402, 0x0403) AM_DEVWRITE("ay1", ay8910_device, data_address_w)
	AM_RANGE(0x0405, 0x0405) AM_DEVREAD("ay2", ay8910_device, data_r)
	AM_RANGE(0x0406, 0x0407) AM_DEVWRITE("ay2", ay8910_device, data_address_w)
	AM_RANGE(0x0408, 0x0408) AM_READ(mcu_acknowledge_r)
	AM_RANGE(0x0410, 0x0410) AM_WRITE(superqix_0410_w)  /* ROM bank, NMI enable, tile bank, bitmap bank */
	AM_RANGE(0x0418, 0x0418) AM_READ(nmi_ack_r)
	// following two ranges are made of two 64x4 4464 DRAM chips at 9L and 9M, "GRAPHICS RAM" or "GRP BIT" if there is an error in POST
	AM_RANGE(0x0800, 0x77ff) AM_RAM_WRITE(superqix_bitmapram_w) AM_SHARE("bitmapram")
	AM_RANGE(0x8800, 0xf7ff) AM_RAM_WRITE(superqix_bitmapram2_w) AM_SHARE("bitmapram2")
	//AM_RANGE(0xf970, 0xfa6f) AM_RAM // this is probably a portion of the remainder of the chips at 9L and 9M which isn't used or tested for graphics ram
ADDRESS_MAP_END


/* I8751 memory handlers */

static ADDRESS_MAP_START( sqix_8031_mcu_io_map, AS_IO, 8, superqix_state )
	AM_RANGE(MCS51_PORT_P1, MCS51_PORT_P1) AM_WRITE(bootleg_mcu_p1_w)
	AM_RANGE(MCS51_PORT_P3, MCS51_PORT_P3) AM_READWRITE(bootleg_mcu_p3_r, mcu_p3_w)
ADDRESS_MAP_END

static ADDRESS_MAP_START( sqix_mcu_io_map, AS_IO, 8, superqix_state )
	AM_RANGE(MCS51_PORT_P0, MCS51_PORT_P0) AM_READ(sqix_system_status_r)
	AM_RANGE(MCS51_PORT_P1, MCS51_PORT_P1) AM_READ_PORT("DSW1")
	AM_RANGE(MCS51_PORT_P2, MCS51_PORT_P2) AM_WRITE(sqixu_mcu_p2_w)
	AM_RANGE(MCS51_PORT_P3, MCS51_PORT_P3) AM_READWRITE(sqixu_mcu_p3_r, mcu_p3_w)
ADDRESS_MAP_END



static INPUT_PORTS_START( pbillian )
	PORT_START("DSW1")
	PORT_DIPNAME( 0x07, 0x07, DEF_STR( Coin_A ) )           PORT_DIPLOCATION("SW1:1,2,3")
	PORT_DIPSETTING(    0x03, DEF_STR( 5C_1C ) )
	PORT_DIPSETTING(    0x04, DEF_STR( 4C_1C ) )
	PORT_DIPSETTING(    0x05, DEF_STR( 3C_1C ) )
	PORT_DIPSETTING(    0x06, DEF_STR( 2C_1C ) )
	PORT_DIPSETTING(    0x07, DEF_STR( 1C_1C ) )
	PORT_DIPSETTING(    0x00, DEF_STR( 2C_3C ) )
	PORT_DIPSETTING(    0x02, DEF_STR( 1C_2C ) )
	PORT_DIPSETTING(    0x01, DEF_STR( 1C_3C ) )
	PORT_DIPNAME( 0x38, 0x38, DEF_STR( Coin_B ) )           PORT_DIPLOCATION("SW1:4,5,6")
	PORT_DIPSETTING(    0x18, DEF_STR( 5C_1C ) )
	PORT_DIPSETTING(    0x20, DEF_STR( 4C_1C ) )
	PORT_DIPSETTING(    0x28, DEF_STR( 3C_1C ) )
	PORT_DIPSETTING(    0x30, DEF_STR( 2C_1C ) )
	PORT_DIPSETTING(    0x38, DEF_STR( 1C_1C ) )
	PORT_DIPSETTING(    0x00, DEF_STR( 2C_3C ) )
	PORT_DIPSETTING(    0x10, DEF_STR( 1C_2C ) )
	PORT_DIPSETTING(    0x08, DEF_STR( 1C_3C ) )
	PORT_DIPNAME( 0x40, 0x00, DEF_STR( Allow_Continue ) )   PORT_DIPLOCATION("SW1:7")
	PORT_DIPSETTING(    0x40, DEF_STR( No ) )
	PORT_DIPSETTING(    0x00, DEF_STR( Yes ) )
	PORT_DIPNAME( 0x80, 0x80, "Freeze" )                    PORT_DIPLOCATION("SW1:8")
	PORT_DIPSETTING(    0x80, DEF_STR( No ) )
	PORT_DIPSETTING(    0x00, DEF_STR( Yes ) )

	PORT_START("DSW2")
	PORT_DIPNAME( 0x03, 0x03, DEF_STR( Lives ) )            PORT_DIPLOCATION("SW2:1,2")
	PORT_DIPSETTING(    0x03, "2" )
	PORT_DIPSETTING(    0x02, "3" )
	PORT_DIPSETTING(    0x01, "4" )
	PORT_DIPSETTING(    0x00, "5" )
	PORT_DIPNAME( 0x0c, 0x0c, DEF_STR( Bonus_Life ) )       PORT_DIPLOCATION("SW2:3,4")
	PORT_DIPSETTING(    0x0c, "10/20/300K Points" )
	PORT_DIPSETTING(    0x00, "10/30/500K Points" )
	PORT_DIPSETTING(    0x08, "20/30/400K Points" )
	PORT_DIPSETTING(    0x04, "30/40/500K Points" )
	PORT_DIPNAME( 0x30, 0x10, DEF_STR( Difficulty ) )       PORT_DIPLOCATION("SW2:5,6")
	PORT_DIPSETTING(    0x00, DEF_STR( Easy ) )
	PORT_DIPSETTING(    0x10, DEF_STR( Normal ) )
	PORT_DIPSETTING(    0x20, DEF_STR( Hard ) )
	PORT_DIPSETTING(    0x30, DEF_STR( Very_Hard ) )
	PORT_DIPNAME( 0x40, 0x00, DEF_STR( Cabinet ) )          PORT_DIPLOCATION("SW2:7")
	PORT_DIPSETTING(    0x00, DEF_STR( Upright ) )
	PORT_DIPSETTING(    0x40, DEF_STR( Cocktail ) )
	PORT_DIPNAME( 0x80, 0x80, DEF_STR( Flip_Screen ) )      PORT_DIPLOCATION("SW2:8")
	PORT_DIPSETTING(    0x80, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )

	PORT_START("SYSTEM") // ay port B (register F)
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_SERVICE1 )
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_START2 )
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_START1 )
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_COIN2 )
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_COIN1 )
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_UNKNOWN ) // hblank?
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_CUSTOM ) PORT_VBLANK("screen")

	PORT_START("BUTTONS") // ay port A (register E)
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_UNUSED )     // N/C
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_BUTTON2 )    // P1 fire (M powerup) + high score initials
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_UNUSED )     // N/C
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_COCKTAIL  // P2 fire (M powerup) + high score initials
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0xc0, IP_ACTIVE_HIGH, IPT_SPECIAL ) PORT_CUSTOM_MEMBER(DEVICE_SELF, hotsmash_state, pbillian_semaphore_input_r, nullptr)  /* Z80 and MCU Semaphores */

	PORT_START("PLUNGER1")
	PORT_BIT( 0xff, 0x00, IPT_PEDAL ) PORT_MINMAX(0x00, 0xff) PORT_SENSITIVITY(100) PORT_KEYDELTA(16)

	PORT_START("DIAL1")
	PORT_BIT( 0xff, 0x00, IPT_DIAL ) PORT_SENSITIVITY(20) PORT_KEYDELTA(8)

	PORT_START("LAUNCH1")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_BUTTON1 )
	PORT_BIT( 0xfe, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_START("PLUNGER2")
	PORT_BIT( 0xff, 0x00, IPT_PEDAL ) PORT_MINMAX(0x00, 0xff) PORT_SENSITIVITY(100) PORT_KEYDELTA(16) PORT_COCKTAIL

	PORT_START("DIAL2")
	PORT_BIT( 0xff, 0x00, IPT_DIAL ) PORT_SENSITIVITY(20) PORT_KEYDELTA(8) PORT_COCKTAIL

	PORT_START("LAUNCH2")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_COCKTAIL
	PORT_BIT( 0xfe, IP_ACTIVE_LOW, IPT_UNUSED )

INPUT_PORTS_END

static INPUT_PORTS_START( hotsmash )
	PORT_START("DSW1")
	PORT_DIPNAME( 0x01, 0x01, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x01, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x02, 0x02, DEF_STR( Flip_Screen ) )
	PORT_DIPSETTING(    0x02, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x04, 0x04, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x04, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x08, 0x08, DEF_STR( Demo_Sounds ) )
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x08, DEF_STR( On ) )
	PORT_DIPNAME( 0x30, 0x30, DEF_STR( Coin_A ) )
	PORT_DIPSETTING(    0x10, DEF_STR( 2C_1C ) )
	PORT_DIPSETTING(    0x30, DEF_STR( 1C_1C ) )
	PORT_DIPSETTING(    0x00, DEF_STR( 2C_3C ) )
	PORT_DIPSETTING(    0x20, DEF_STR( 1C_2C ) )
	PORT_DIPNAME( 0xc0, 0xc0, DEF_STR( Coin_B ) )
	PORT_DIPSETTING(    0x40, DEF_STR( 2C_1C ) )
	PORT_DIPSETTING(    0xc0, DEF_STR( 1C_1C ) )
	PORT_DIPSETTING(    0x00, DEF_STR( 2C_3C ) )
	PORT_DIPSETTING(    0x80, DEF_STR( 1C_2C ) )

	PORT_START("DSW2")
	PORT_DIPNAME( 0x03, 0x03, "Difficulty vs. CPU" )
	PORT_DIPSETTING(    0x02, DEF_STR( Easy ) )
	PORT_DIPSETTING(    0x03, DEF_STR( Normal ) )
	PORT_DIPSETTING(    0x01, DEF_STR( Hard ) )
	PORT_DIPSETTING(    0x00, DEF_STR( Hardest ) )
	PORT_DIPNAME( 0x0c, 0x0c, "Difficulty vs. 2P" )
	PORT_DIPSETTING(    0x08, DEF_STR( Easy ) )
	PORT_DIPSETTING(    0x0c, DEF_STR( Normal ) )
	PORT_DIPSETTING(    0x04, DEF_STR( Hard ) )
	PORT_DIPSETTING(    0x00, DEF_STR( Hardest ) )
	PORT_DIPNAME( 0x10, 0x10, "Points per game" )
	PORT_DIPSETTING(    0x00, "3" )
	PORT_DIPSETTING(    0x10, "4" )
	PORT_DIPNAME( 0x20, 0x20, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x20, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x40, 0x40, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x40, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x80, 0x80, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x80, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )

	PORT_START("SYSTEM") // ay port B (register F)
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_SERVICE1 )
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_START2 )
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_START1 )
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_COIN2 )//$49c
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_COIN1 )//$42d
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_UNKNOWN ) // hblank?
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_CUSTOM ) PORT_VBLANK("screen")

	PORT_START("BUTTONS") // ay port A (register E)
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_BUTTON2 ) // p1 button 2, unused on this game?
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_COCKTAIL  // p2 button 2, unused on this game?
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0xc0, IP_ACTIVE_HIGH, IPT_SPECIAL ) PORT_CUSTOM_MEMBER(DEVICE_SELF, hotsmash_state, pbillian_semaphore_input_r, nullptr)  /* Z80 and MCU Semaphores */

	PORT_START("PLUNGER1")  // plunger isn't present on hotsmash though the pins exist for it
	PORT_BIT( 0xff, IP_ACTIVE_HIGH, IPT_UNUSED ) PORT_PLAYER(1)

	PORT_START("DIAL1")
	PORT_BIT( 0xff, 0x00, IPT_DIAL ) PORT_SENSITIVITY(15) PORT_KEYDELTA(30) PORT_CENTERDELTA(0) PORT_PLAYER(1)

	PORT_START("LAUNCH1")  // launch button isn't present on hotsmash
	PORT_BIT( 0xff, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_START("PLUNGER2")  // plunger isn't present on hotsmash though the pins exist for it
	PORT_BIT( 0xff, IP_ACTIVE_HIGH, IPT_UNUSED ) PORT_PLAYER(2)

	PORT_START("DIAL2")
	PORT_BIT( 0xff, 0x00, IPT_DIAL ) PORT_SENSITIVITY(15) PORT_KEYDELTA(30) PORT_CENTERDELTA(0) PORT_PLAYER(2)

	PORT_START("LAUNCH2")  // launch button isn't present on hotsmash
	PORT_BIT( 0xff, IP_ACTIVE_LOW, IPT_UNUSED ) PORT_PLAYER(2)

INPUT_PORTS_END


static INPUT_PORTS_START( superqix )
	PORT_START("DSW1")  /* DSW1 */
	PORT_DIPNAME( 0x01, 0x00, DEF_STR( Cabinet ) )          PORT_DIPLOCATION("SW1:1")
	PORT_DIPSETTING(    0x00, DEF_STR( Upright ) )
	PORT_DIPSETTING(    0x01, DEF_STR( Cocktail ) )
	PORT_DIPNAME( 0x02, 0x02, DEF_STR( Flip_Screen ) )      PORT_DIPLOCATION("SW1:2")
	PORT_DIPSETTING(    0x02, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x04, 0x04, "Freeze" )                    PORT_DIPLOCATION("SW1:3")
	PORT_DIPSETTING(    0x04, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x08, 0x00, DEF_STR( Allow_Continue ) )   PORT_DIPLOCATION("SW1:4")
	PORT_DIPSETTING(    0x08, DEF_STR( No ) )
	PORT_DIPSETTING(    0x00, DEF_STR( Yes ) )
	PORT_DIPNAME( 0x30, 0x30, DEF_STR( Coin_A ) )           PORT_DIPLOCATION("SW1:5,6")
	PORT_DIPSETTING(    0x10, DEF_STR( 2C_1C ) )
	PORT_DIPSETTING(    0x30, DEF_STR( 1C_1C ) )
	PORT_DIPSETTING(    0x00, DEF_STR( 2C_3C ))
	PORT_DIPSETTING(    0x20, DEF_STR( 1C_2C ) )
	PORT_DIPNAME( 0xc0, 0xc0, DEF_STR( Coin_B ) )           PORT_DIPLOCATION("SW1:7,8")
	PORT_DIPSETTING(    0x40, DEF_STR( 2C_1C ) )
	PORT_DIPSETTING(    0xc0, DEF_STR( 1C_1C ) )
	PORT_DIPSETTING(    0x00, DEF_STR( 2C_3C ))
	PORT_DIPSETTING(    0x80, DEF_STR( 1C_2C ) )

	PORT_START("DSW2")  /* DSW2 */
	PORT_DIPNAME( 0x03, 0x03, DEF_STR( Difficulty ) )       PORT_DIPLOCATION("SW2:1,2")
	PORT_DIPSETTING(    0x02, DEF_STR( Easy ) )
	PORT_DIPSETTING(    0x03, DEF_STR( Normal ) )
	PORT_DIPSETTING(    0x01, DEF_STR( Hard ) )
	PORT_DIPSETTING(    0x00, DEF_STR( Hardest ) )
	PORT_DIPNAME( 0x0c, 0x0c, DEF_STR( Bonus_Life ) )       PORT_DIPLOCATION("SW2:3,4")
	PORT_DIPSETTING(    0x08, "20000 50000" )
	PORT_DIPSETTING(    0x0c, "30000 100000" )
	PORT_DIPSETTING(    0x04, "50000 100000" )
	PORT_DIPSETTING(    0x00, DEF_STR( None ) )
	PORT_DIPNAME( 0x30, 0x30, DEF_STR( Lives ) )            PORT_DIPLOCATION("SW2:5,6")
	PORT_DIPSETTING(    0x20, "2" )
	PORT_DIPSETTING(    0x30, "3" )
	PORT_DIPSETTING(    0x10, "4" )
	PORT_DIPSETTING(    0x00, "5" )
	PORT_DIPNAME( 0xc0, 0xc0, "Fill Area" )                 PORT_DIPLOCATION("SW2:7,8")
	PORT_DIPSETTING(    0x80, "70%" )
	PORT_DIPSETTING(    0xc0, "75%" )
	PORT_DIPSETTING(    0x40, "80%" )
	PORT_DIPSETTING(    0x00, "85%" )

	PORT_START("SYSTEM") /* Port 0 of MCU, might also be readable by z80 at io 0x0418 (nmi ack read port) */
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_COIN1 )
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_COIN2 )
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_START1 )
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_START2 )
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_SERVICE1 )   // doesn't work in bootleg
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0xc0, IP_ACTIVE_HIGH, IPT_SPECIAL ) PORT_CUSTOM_MEMBER(DEVICE_SELF, superqix_state, superqix_semaphore_input_r, nullptr)  /* Z80 and MCU Semaphores */
	/* The bits 0xc0 above is known to be WRONG from tracing:
	bit 6 connects to whatever bit 7 is connected to on AY-3-8910 #1 @3P Port A
	bit 7 connects to whatever bit 7 is connected to on AY-3-8910 #1 @3P Port B
	however what those ay bits actually each connect to (semaphores? service button?) is currently unknown
	*/

	PORT_START("P1") /* AY-3-8910 #1 @3P Port A */
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_JOYSTICK_UP ) PORT_4WAY
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN ) PORT_4WAY
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT ) PORT_4WAY
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT ) PORT_4WAY
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_BUTTON1 )
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_CUSTOM ) PORT_VBLANK("screen")   /* ??? */
	PORT_SERVICE( 0x80, IP_ACTIVE_LOW )

	PORT_START("P2") /* AY-3-8910 #1 @3P Port B */
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_JOYSTICK_UP ) PORT_4WAY PORT_COCKTAIL
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN ) PORT_4WAY PORT_COCKTAIL
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT ) PORT_4WAY PORT_COCKTAIL
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT ) PORT_4WAY PORT_COCKTAIL
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_COCKTAIL
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0xc0, IP_ACTIVE_HIGH, IPT_SPECIAL ) PORT_CUSTOM_MEMBER(DEVICE_SELF, superqix_state, superqix_semaphore_input_r, nullptr)  /* Z80 and MCU Semaphores */

INPUT_PORTS_END



static const gfx_layout pbillian_charlayout =
{
	8,8,
	0x800,  /* doesn't use the whole ROM space */
	4,
	{ 0, 1, 2, 3 },
	{ 0*4, 1*4, 2*4, 3*4, 4*4, 5*4, 6*4, 7*4 },
	{ 0*32, 1*32, 2*32, 3*32, 4*32, 5*32, 6*32, 7*32 },
	32*8
};

static const gfx_layout sqix_charlayout =
{
	8,8,
	RGN_FRAC(1,1),
	4,
	{ 0, 1, 2, 3 },
	{ 0*4, 1*4, 2*4, 3*4, 4*4, 5*4, 6*4, 7*4 },
	{ 0*32, 1*32, 2*32, 3*32, 4*32, 5*32, 6*32, 7*32 },
	32*8
};

static const gfx_layout spritelayout =
{
	16,16,
	RGN_FRAC(1,1),
	4,
	{ 0, 1, 2, 3 },
	{ 0*4, 1*4, 2*4, 3*4, 4*4, 5*4, 6*4, 7*4,
			32*8+0*4, 32*8+1*4, 32*8+2*4, 32*8+3*4, 32*8+4*4, 32*8+5*4, 32*8+6*4, 32*8+7*4 },
	{ 0*32, 1*32, 2*32, 3*32, 4*32, 5*32, 6*32, 7*32,
			16*32, 17*32, 18*32, 19*32, 20*32, 21*32, 22*32, 23*32 },
	128*8
};


static GFXDECODE_START( pbillian )
	GFXDECODE_ENTRY( "gfx1", 0, pbillian_charlayout, 16*16, 16 )
	GFXDECODE_ENTRY( "gfx1", 0, spritelayout,            0, 16 )
GFXDECODE_END

static GFXDECODE_START( sqix )
	GFXDECODE_ENTRY( "gfx1", 0x00000, sqix_charlayout,   0, 16 )    /* Chars */
	GFXDECODE_ENTRY( "gfx2", 0x00000, sqix_charlayout,   0, 16 )    /* Background tiles */
	GFXDECODE_ENTRY( "gfx3", 0x00000, spritelayout,      0, 16 )    /* Sprites */
GFXDECODE_END


INTERRUPT_GEN_MEMBER(hotsmash_state::vblank_irq)
{
	if (m_nmi_mask)
		device.execute().set_input_line(INPUT_LINE_NMI, ASSERT_LINE);
}

INTERRUPT_GEN_MEMBER(superqix_state_base::sqix_timer_irq)
{
	if (m_nmi_mask)
		device.execute().set_input_line(INPUT_LINE_NMI, ASSERT_LINE);
}



static MACHINE_CONFIG_START( pbillian )
	MCFG_CPU_ADD("maincpu", Z80,XTAL_12MHz/2)      /* 6 MHz, ROHM Z80B */
	MCFG_CPU_PROGRAM_MAP(main_map)
	MCFG_CPU_IO_MAP(pbillian_port_map)
	MCFG_CPU_VBLANK_INT_DRIVER("screen", hotsmash_state, vblank_irq)

	MCFG_CPU_ADD("mcu", M68705P5, XTAL_12MHz/4) /* 3mhz???? */
	MCFG_M68705_PORTB_W_CB(WRITE8(hotsmash_state, hotsmash_68705_portB_w))
	MCFG_M68705_PORTC_W_CB(WRITE8(hotsmash_state, hotsmash_68705_portC_w))

	//MCFG_QUANTUM_PERFECT_CPU("maincpu")
	MCFG_MACHINE_START_OVERRIDE(hotsmash_state, pbillian)

	/* video hardware */
	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(60)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(0))
	MCFG_SCREEN_SIZE(256, 256)
	MCFG_SCREEN_VISIBLE_AREA(0*8, 32*8-1, 2*8, 30*8-1)
	MCFG_SCREEN_UPDATE_DRIVER(hotsmash_state, screen_update_pbillian)
	MCFG_SCREEN_PALETTE("palette")

	MCFG_GFXDECODE_ADD("gfxdecode", "palette", pbillian)
	MCFG_PALETTE_ADD("palette", 512)
	MCFG_PALETTE_FORMAT_CLASS(1, superqix_state, BBGGRRII)

	MCFG_VIDEO_START_OVERRIDE(hotsmash_state, pbillian)

	MCFG_SPEAKER_STANDARD_MONO("mono")

	MCFG_SOUND_ADD("aysnd", AY8910, XTAL_12MHz/8) // AY-3-8910A
	MCFG_AY8910_PORT_A_READ_CB(IOPORT("BUTTONS"))
	MCFG_AY8910_PORT_B_READ_CB(IOPORT("SYSTEM"))
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.30)

	MCFG_SOUND_ADD("samples", SAMPLES, 0)
	MCFG_SAMPLES_CHANNELS(1)
	MCFG_SAMPLES_START_CB(hotsmash_state, pbillian_sh_start)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.50)
MACHINE_CONFIG_END

static MACHINE_CONFIG_START( sqix )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", Z80, XTAL_12MHz/2)    /* 6 MHz */
	MCFG_CPU_PROGRAM_MAP(main_map)
	MCFG_CPU_IO_MAP(sqix_port_map)
	MCFG_CPU_PERIODIC_INT_DRIVER(superqix_state, sqix_timer_irq,  4*60) /* ??? */

	MCFG_CPU_ADD("mcu", I8751, XTAL_12MHz/3)  /* TODO: VERIFY DIVISOR, is this 3mhz or 4mhz? */
	MCFG_CPU_IO_MAP(sqix_mcu_io_map)

	MCFG_QUANTUM_PERFECT_CPU("maincpu")

	MCFG_MACHINE_START_OVERRIDE(superqix_state,superqix)

	/* video hardware */
	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(60)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(2500) /* not accurate */)
	MCFG_SCREEN_SIZE(32*8, 32*8)
	MCFG_SCREEN_VISIBLE_AREA(0*8, 32*8-1, 2*8, 30*8-1)
	MCFG_SCREEN_UPDATE_DRIVER(superqix_state, screen_update_superqix)
	MCFG_SCREEN_PALETTE("palette")

	MCFG_GFXDECODE_ADD("gfxdecode", "palette", sqix)
	MCFG_PALETTE_ADD("palette", 256)
	MCFG_PALETTE_FORMAT_CLASS(1, superqix_state, BBGGRRII)

	MCFG_VIDEO_START_OVERRIDE(superqix_state,superqix)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("mono")

	MCFG_SOUND_ADD("ay1", AY8910, XTAL_12MHz/8) // AY-3-8910 @3P, outputs directly tied together
	MCFG_AY8910_OUTPUT_TYPE(AY8910_SINGLE_OUTPUT)
	MCFG_AY8910_PORT_A_READ_CB(IOPORT("P1"))
	MCFG_AY8910_PORT_B_READ_CB(READ8(superqix_state, in4_mcu_r)) /* port Bread */
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.25)

	MCFG_SOUND_ADD("ay2", AY8910, XTAL_12MHz/8) // AY-3-8910 @3M, outputs directly tied together
	MCFG_AY8910_OUTPUT_TYPE(AY8910_SINGLE_OUTPUT)
	MCFG_AY8910_PORT_A_READ_CB(IOPORT("DSW2"))
	MCFG_AY8910_PORT_B_READ_CB(READ8(superqix_state, sqix_from_mcu_r)) /* port Bread */
	MCFG_AY8910_PORT_B_WRITE_CB(WRITE8(superqix_state,sqix_z80_mcu_w)) /* port Bwrite */
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.25)
MACHINE_CONFIG_END


static MACHINE_CONFIG_DERIVED( sqix_8031, sqix )
	MCFG_CPU_MODIFY("mcu")
	MCFG_CPU_IO_MAP(sqix_8031_mcu_io_map)
MACHINE_CONFIG_END


static MACHINE_CONFIG_START( sqix_nomcu )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", Z80, 12000000/2)    /* 6 MHz */
	MCFG_CPU_PROGRAM_MAP(main_map)
	MCFG_CPU_IO_MAP(sqix_port_map)
	MCFG_CPU_PERIODIC_INT_DRIVER(superqix_state, sqix_timer_irq,  4*60) /* ??? */

	MCFG_MACHINE_START_OVERRIDE(superqix_state,superqix)

	/* video hardware */
	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(60)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(2500) /* not accurate */)
	MCFG_SCREEN_SIZE(32*8, 32*8)
	MCFG_SCREEN_VISIBLE_AREA(0*8, 32*8-1, 2*8, 30*8-1)
	MCFG_SCREEN_UPDATE_DRIVER(superqix_state, screen_update_superqix)
	MCFG_SCREEN_PALETTE("palette")

	MCFG_GFXDECODE_ADD("gfxdecode", "palette", sqix)
	MCFG_PALETTE_ADD("palette", 256)
	MCFG_PALETTE_FORMAT_CLASS(1, superqix_state, BBGGRRII)

	MCFG_VIDEO_START_OVERRIDE(superqix_state,superqix)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("mono")

	MCFG_SOUND_ADD("ay1", AY8910, 12000000/8)
	MCFG_AY8910_OUTPUT_TYPE(AY8910_SINGLE_OUTPUT) // ?
	MCFG_AY8910_PORT_A_READ_CB(IOPORT("P1"))
	MCFG_AY8910_PORT_B_READ_CB(READ8(superqix_state, in4_mcu_r))
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.25)

	MCFG_SOUND_ADD("ay2", AY8910, 12000000/8)
	MCFG_AY8910_OUTPUT_TYPE(AY8910_SINGLE_OUTPUT) // ?
	MCFG_AY8910_PORT_A_READ_CB(IOPORT("DSW2"))
	MCFG_AY8910_PORT_B_READ_CB(READ8(superqix_state, bootleg_in0_r)) /* port Bread */
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.25)
MACHINE_CONFIG_END



/***************************************************************************

  Game driver(s)

***************************************************************************/

/* Prebillian pcbs do not have the usual Taito letter-number pair code on the
labels on the roms/MCU, they only have the mitsubishi electric logo and a
single number.
The PCB has a label which says "M6100211A // プレビリアン" (PuReBiRiAN)
*/
ROM_START( pbillian )
	ROM_REGION( 0x018000, "maincpu", 0 )
	ROM_LOAD( "mitsubishi__electric__1.m5l27256k.6bc",  0x00000, 0x08000, CRC(d379fe23) SHA1(e147a9151b1cdeacb126d9713687bd0aa92980ac) )
	ROM_LOAD( "mitsubishi__electric__2.m5l27128k.6d",  0x14000, 0x04000, CRC(1af522bc) SHA1(83e002dc831bfcedbd7096b350c9b34418b79674) )

	ROM_REGION( 0x0800, "mcu", 0 )
	ROM_LOAD( "mitsubishi__electric__7.mc68705p5s.7k", 0x0000, 0x0800, CRC(03de0c74) SHA1(ee2bc8be9bab9557c6776b996b85ed6f32300b47) )

	ROM_REGION( 0x8000, "samples", 0 )
	ROM_LOAD( "mitsubishi__electric__3.m5l27256k.7h",  0x0000, 0x08000, CRC(3f9bc7f1) SHA1(0b0c2ec3bea6a7f3fc6c0c8b750318f3f9ec3d1f) )

	ROM_REGION( 0x018000, "gfx1", 0 )
	ROM_LOAD( "mitsubishi__electric__4.m5l27256k.1n",  0x00000, 0x08000, CRC(9c08a072) SHA1(25f31fcf72216cf42528b07ad8c09113aa69861a) )
	ROM_LOAD( "mitsubishi__electric__5.m5l27256k.1r",  0x08000, 0x08000, CRC(2dd5b83f) SHA1(b05e3a008050359d0207757b9cbd8cee87abc697) )
	ROM_LOAD( "mitsubishi__electric__6.m5l27256k.1t",  0x10000, 0x08000, CRC(33b855b0) SHA1(5a1df4f82fc0d6f78883b759fd61f395942645eb) )
ROM_END

ROM_START( hotsmash )
	ROM_REGION( 0x018000, "maincpu", 0 )
	ROM_LOAD( "b18-04",  0x00000, 0x08000, CRC(981bde2c) SHA1(ebcc901a036cde16b33d534d423500d74523b781) )

	ROM_REGION( 0x0800, "mcu", 0 )
	ROM_LOAD( "b18-06.mcu", 0x0000, 0x0800, CRC(67c0920a) SHA1(23a294892823d1d9216ea8ddfa9df1c8af149477) ) // has valid reset vector and int vector in it, SWI and TIMER vectors are NOPs

	ROM_REGION( 0x8000, "samples", 0 )
	ROM_LOAD( "b18-05",  0x0000, 0x08000, CRC(dab5e718) SHA1(6cf6486f283f5177dfdc657b1627fbfa3f0743e8) )

	ROM_REGION( 0x018000, "gfx1", 0 )
	ROM_LOAD( "b18-01",  0x00000, 0x08000, CRC(870a4c04) SHA1(a029108bcda40755c8320d2ee297f42d816aa7c0) )
	ROM_LOAD( "b18-02",  0x08000, 0x08000, CRC(4e625cac) SHA1(2c21b32240eaada9a5f909a2ec5b335372c8c994) )
	ROM_LOAD( "b18-03",  0x14000, 0x04000, CRC(1c82717d) SHA1(6942c8877e24ac51ed71036e771a1655d82f3491) )
ROM_END

ROM_START( sqix ) // It is unclear what this set fixes vs 1.1 below, but the 'rug pattern' on the bitmap test during POST has the left edge entirely black, unlike v1.0 or v1.1, but like sqixu
	ROM_REGION( 0x20000, "maincpu", 0 )
	ROM_LOAD( "b03__01-2.ef3",  0x00000, 0x08000, CRC(5ded636b) SHA1(827954001b4617b3bd439be75094d8dca06ea32b) )
	ROM_LOAD( "b03__02.h3",     0x10000, 0x10000, CRC(9c23cb64) SHA1(7e04cb18cabdc0031621162cbc228cd95875a022) )

	ROM_REGION( 0x1000, "mcu", 0 )  /* I8751 code */
	ROM_LOAD( "b03__03.l2",     0x00000, 0x1000, CRC(f0c3af2b) SHA1(6dce2175011b5c8d0f1bce433c53979841d5d1a4) ) /* B03 // 03 C8751-88 MCU, verified from deprotected part */

	ROM_REGION( 0x08000, "gfx1", 0 )
	ROM_LOAD( "b03__04.s8",    0x00000, 0x08000, CRC(f815ef45) SHA1(4189d455b6ccf3ae922d410fb624c4665203febf) )

	ROM_REGION( 0x20000, "gfx2", 0 )
	ROM_LOAD( "taito_sq-iu3__lh231041__sharp_japan__8709_d.p8",    0x00000, 0x20000, CRC(b8d0c493) SHA1(ef5d62ef3835c7ae088a7aa98945f747130fe0ec) ) /* Sharp LH231041 28 pin 128K x 8bit mask rom */

	ROM_REGION( 0x10000, "gfx3", 0 )
	ROM_LOAD( "b03__05.t8",    0x00000, 0x10000, CRC(df326540) SHA1(1fe025edcd38202e24c4e1005f478b6a88533453) )
ROM_END

ROM_START( sqixr1 ) // This set has the coin lockout polarity inverted, and also fixes the 10 vs 9 lockout bug
	ROM_REGION( 0x20000, "maincpu", 0 )
	ROM_LOAD( "b03__01-1.ef3",  0x00000, 0x08000, CRC(ad614117) SHA1(c461f00a2aecde1bc3860c15a3c31091b14665a2) )
	ROM_LOAD( "b03__02.h3",     0x10000, 0x10000, CRC(9c23cb64) SHA1(7e04cb18cabdc0031621162cbc228cd95875a022) )

	ROM_REGION( 0x1000, "mcu", 0 )  /* I8751 code */
	ROM_LOAD( "b03__03.l2",     0x00000, 0x1000, CRC(f0c3af2b) SHA1(6dce2175011b5c8d0f1bce433c53979841d5d1a4) ) /* B03 // 03 C8751-88 MCU, verified from deprotected part */

	ROM_REGION( 0x08000, "gfx1", 0 )
	ROM_LOAD( "b03__04.s8",    0x00000, 0x08000, CRC(f815ef45) SHA1(4189d455b6ccf3ae922d410fb624c4665203febf) )

	ROM_REGION( 0x20000, "gfx2", 0 )
	ROM_LOAD( "taito_sq-iu3__lh231041__sharp_japan__8709_d.p8",    0x00000, 0x20000, CRC(b8d0c493) SHA1(ef5d62ef3835c7ae088a7aa98945f747130fe0ec) ) /* Sharp LH231041 28 pin 128K x 8bit mask rom */

	ROM_REGION( 0x10000, "gfx3", 0 )
	ROM_LOAD( "b03__05.t8",    0x00000, 0x10000, CRC(df326540) SHA1(1fe025edcd38202e24c4e1005f478b6a88533453) )
ROM_END

ROM_START( sqixr0 ) // This set is older than the above two: it has the coin lockout only trigger after 10 coins (causing the last coin to be lost), and the coin lockout polarity is not inverted
	ROM_REGION( 0x20000, "maincpu", 0 )
	ROM_LOAD( "b03__01.ef3",    0x00000, 0x08000, CRC(0888b7de) SHA1(de3e4637436de185f43d2ad4186d4cfdcd4d33d9) )
	ROM_LOAD( "b03__02.h3",     0x10000, 0x10000, CRC(9c23cb64) SHA1(7e04cb18cabdc0031621162cbc228cd95875a022) )

	ROM_REGION( 0x1000, "mcu", 0 )  /* I8751 code */
	ROM_LOAD( "b03__03.l2",     0x00000, 0x1000, CRC(f0c3af2b) SHA1(6dce2175011b5c8d0f1bce433c53979841d5d1a4) ) /* B03 // 03 C8751-88 MCU, verified from deprotected part */

	ROM_REGION( 0x08000, "gfx1", 0 )
	ROM_LOAD( "b03__04.s8",    0x00000, 0x08000, CRC(f815ef45) SHA1(4189d455b6ccf3ae922d410fb624c4665203febf) )

	ROM_REGION( 0x20000, "gfx2", 0 )
	ROM_LOAD( "taito_sq-iu3__lh231041__sharp_japan__8709_d.p8",    0x00000, 0x20000, CRC(b8d0c493) SHA1(ef5d62ef3835c7ae088a7aa98945f747130fe0ec) ) /* Sharp LH231041 28 pin 128K x 8bit mask rom */

	ROM_REGION( 0x10000, "gfx3", 0 )
	ROM_LOAD( "b03__05.t8",    0x00000, 0x10000, CRC(df326540) SHA1(1fe025edcd38202e24c4e1005f478b6a88533453) )
ROM_END

ROM_START( sqixu )
	ROM_REGION( 0x20000, "maincpu", 0 )
	ROM_LOAD( "b03__06.ef3",   0x00000, 0x08000, CRC(4f59f7af) SHA1(6ea627ea8505cf8d1a5a1350258180c61fbd1ed9) )
	ROM_LOAD( "b03__07.h3",    0x10000, 0x10000, CRC(4c417d4a) SHA1(de46551da1b27312dca40240a210e77595cf9dbd) )

	ROM_REGION( 0x1000, "mcu", 0 )  /* I8751 code */
	ROM_LOAD( "b03__08.l2",    0x00000, 0x01000, CRC(7c338c0f) SHA1(b91468c881641f807067835b2dd490cd3e3c577e) ) /* B03 // 08 C8751-88 MCU, verified from deprotected part, 3 bytes different from B03 // 03 */

	ROM_REGION( 0x08000, "gfx1", 0 )
	ROM_LOAD( "b03__04.s8",    0x00000, 0x08000, CRC(f815ef45) SHA1(4189d455b6ccf3ae922d410fb624c4665203febf) )

	ROM_REGION( 0x20000, "gfx2", 0 )
	ROM_LOAD( "taito_sq-iu3__lh231041__sharp_japan__8709_d.p8",    0x00000, 0x20000, CRC(b8d0c493) SHA1(ef5d62ef3835c7ae088a7aa98945f747130fe0ec) ) /* Sharp LH231041 28 pin 128K x 8bit mask rom */

	ROM_REGION( 0x10000, "gfx3", 0 )
	ROM_LOAD( "b03__09.t8",    0x00000, 0x10000, CRC(69d2a84a) SHA1(b461d8a01f73c6aaa4aac85602c688c111bdca5d) )
ROM_END

/* this is a bootleg with an 8031+external rom in place of the 8751 of the
   original board; The MCU code is extensively hacked to avoid use of ports 0
   and 2, which are used as the rom data and address buses, using a multiplexed
   latch on the other ports instead. This bootleg MCU is based on a dump of the
   original b03__03.l2 code, obtained by the pirates through unknown means.
   Barring the bootleg MCU, the actual rom set is an exact copy of sqixr0 above. */
ROM_START( sqixb1 ) // formerly 'sqixa'
	ROM_REGION( 0x20000, "maincpu", 0 )
	ROM_LOAD( "sq01.97",       0x00000, 0x08000, CRC(0888b7de) SHA1(de3e4637436de185f43d2ad4186d4cfdcd4d33d9) ) // == b03__01.ef3
	ROM_LOAD( "b03__02.h3",     0x10000, 0x10000, CRC(9c23cb64) SHA1(7e04cb18cabdc0031621162cbc228cd95875a022) ) // actual label is something different on the bootleg

	ROM_REGION( 0x10000, "mcu", 0 ) /* I8031 code */
	ROM_LOAD( "sq07.ic108",     0x00000, 0x1000, CRC(d11411fb) SHA1(31183f433596c4d2503c01f6dc8d91024f2cf5de) ) // actual label is something different on the bootleg

	ROM_REGION( 0x08000, "gfx1", 0 )
	ROM_LOAD( "b03__04.s8",    0x00000, 0x08000, CRC(f815ef45) SHA1(4189d455b6ccf3ae922d410fb624c4665203febf) ) // actual label is something different on the bootleg

	ROM_REGION( 0x20000, "gfx2", 0 )
	ROM_LOAD( "b03-03",       0x00000, 0x10000, CRC(6e8b6a67) SHA1(c71117cc880a124c46397c446d1edc1cbf681200) ) /* == 1st half of taito_sq-iu3__lh231041__sharp_japan__8709_d.p8, fake label */
	ROM_LOAD( "b03-06",       0x10000, 0x10000, CRC(38154517) SHA1(703ad4cfe54a4786c67aedcca5998b57f39fd857) ) /* == 2nd half of taito_sq-iu3__lh231041__sharp_japan__8709_d.p8, fake label */

	ROM_REGION( 0x10000, "gfx3", 0 )
	ROM_LOAD( "b03__05.t8",    0x00000, 0x10000, CRC(df326540) SHA1(1fe025edcd38202e24c4e1005f478b6a88533453) ) // actual label is something different on the bootleg
ROM_END

ROM_START( sqixb2 ) // this bootleg set has been extensively hacked to avoid using the MCU at all, though a few checks for the semaphore flags were never patched out
	ROM_REGION( 0x20000, "maincpu", 0 )
	ROM_LOAD( "cpu.2",         0x00000, 0x08000, CRC(682e28e3) SHA1(fe9221d26d7397be5a0fc8fdc51672b5924f3cf2) )
	ROM_LOAD( "b03__02.h3",     0x10000, 0x10000, CRC(9c23cb64) SHA1(7e04cb18cabdc0031621162cbc228cd95875a022) ) // actual label is something different on the bootleg

	ROM_REGION( 0x08000, "gfx1", 0 )
	ROM_LOAD( "b03__04.s8",    0x00000, 0x08000, CRC(f815ef45) SHA1(4189d455b6ccf3ae922d410fb624c4665203febf) ) // actual label is something different on the bootleg

	ROM_REGION( 0x20000, "gfx2", 0 )
	ROM_LOAD( "b03-03",       0x00000, 0x10000, CRC(6e8b6a67) SHA1(c71117cc880a124c46397c446d1edc1cbf681200) ) /* == 1st half of taito_sq-iu3__lh231041__sharp_japan__8709_d.p8, fake label */
	ROM_LOAD( "b03-06",       0x10000, 0x10000, CRC(38154517) SHA1(703ad4cfe54a4786c67aedcca5998b57f39fd857) ) /* == 2nd half of taito_sq-iu3__lh231041__sharp_japan__8709_d.p8, fake label */

	ROM_REGION( 0x10000, "gfx3", 0 )
	ROM_LOAD( "b03__05.t8",    0x00000, 0x10000, CRC(df326540) SHA1(1fe025edcd38202e24c4e1005f478b6a88533453) ) // actual label is something different on the bootleg
ROM_END

ROM_START( perestrof )
	ROM_REGION( 0x20000, "maincpu", 0 )
	/* 0x8000 - 0x10000 in the rom is empty anyway */
	ROM_LOAD( "rom1.bin",        0x00000, 0x20000, CRC(0cbf96c1) SHA1(cf2b1367887d1b8812a56aa55593e742578f220c) )

	ROM_REGION( 0x10000, "gfx1", 0 )
	ROM_LOAD( "rom4.bin",       0x00000, 0x10000, CRC(c56122a8) SHA1(1d24b2f0358e14aca5681f92175869224584a6ea) ) /* both halves identical */

	ROM_REGION( 0x20000, "gfx2", 0 )
	ROM_LOAD( "rom2.bin",       0x00000, 0x20000, CRC(36f93701) SHA1(452cb23efd955c6c155cef2b1b650e253e195738) )

	ROM_REGION( 0x10000, "gfx3", 0 )
	ROM_LOAD( "rom3.bin",       0x00000, 0x10000, CRC(00c91d5a) SHA1(fdde56d3689a47e6bfb296e442207b93b887ec7a) )
ROM_END

ROM_START( perestro )
	ROM_REGION( 0x20000, "maincpu", 0 )
	/* 0x8000 - 0x10000 in the rom is empty anyway */
	ROM_LOAD( "rom1.bin",        0x00000, 0x20000, CRC(0cbf96c1) SHA1(cf2b1367887d1b8812a56aa55593e742578f220c) )

	ROM_REGION( 0x10000, "gfx1", 0 )
	ROM_LOAD( "rom4.bin",       0x00000, 0x10000, CRC(c56122a8) SHA1(1d24b2f0358e14aca5681f92175869224584a6ea) ) /* both halves identical */

	ROM_REGION( 0x20000, "gfx2", 0 )
	ROM_LOAD( "rom2.bin",       0x00000, 0x20000, CRC(36f93701) SHA1(452cb23efd955c6c155cef2b1b650e253e195738) )

	ROM_REGION( 0x10000, "gfx3", 0 )
	ROM_LOAD( "rom3a.bin",       0x00000, 0x10000, CRC(7a2a563f) SHA1(e3654091b858cc80ec1991281447fc3622a0d4f9) )
ROM_END

DRIVER_INIT_MEMBER(superqix_state_base, sqix)
{
	m_invert_coin_lockout = true;
}

DRIVER_INIT_MEMBER(superqix_state_base, sqixr0)
{
	m_invert_coin_lockout = false;
}

DRIVER_INIT_MEMBER(superqix_state_base, perestro)
{
	uint8_t *src;
	int len;
	uint8_t temp[16];
	int i,j;

	/* decrypt program code; the address lines are shuffled around in a non-trivial way */
	src = memregion("maincpu")->base();
	len = memregion("maincpu")->bytes();
	for (i = 0;i < len;i += 16)
	{
		memcpy(temp,&src[i],16);
		for (j = 0;j < 16;j++)
		{
			static const int convtable[16] =
			{
				0xc, 0x9, 0xb, 0xa,
				0x8, 0xd, 0xf, 0xe,
				0x4, 0x1, 0x3, 0x2,
				0x0, 0x5, 0x7, 0x6
			};

			src[i+j] = temp[convtable[j]];
		}
	}

	/* decrypt gfx ROMs; simple bit swap on the address lines */
	src = memregion("gfx1")->base();
	len = memregion("gfx1")->bytes();
	for (i = 0;i < len;i += 16)
	{
		memcpy(temp,&src[i],16);
		for (j = 0;j < 16;j++)
		{
			src[i+j] = temp[BITSWAP8(j,7,6,5,4,3,2,0,1)];
		}
	}

	src = memregion("gfx2")->base();
	len = memregion("gfx2")->bytes();
	for (i = 0;i < len;i += 16)
	{
		memcpy(temp,&src[i],16);
		for (j = 0;j < 16;j++)
		{
			src[i+j] = temp[BITSWAP8(j,7,6,5,4,0,1,2,3)];
		}
	}

	src = memregion("gfx3")->base();
	len = memregion("gfx3")->bytes();
	for (i = 0;i < len;i += 16)
	{
		memcpy(temp,&src[i],16);
		for (j = 0;j < 16;j++)
		{
			src[i+j] = temp[BITSWAP8(j,7,6,5,4,1,0,3,2)];
		}
	}
}

DRIVER_INIT_MEMBER(superqix_state_base, pbillian)
{
	m_invert_p2_spinner = false;
}

DRIVER_INIT_MEMBER(superqix_state_base, hotsmash)
{
	m_invert_p2_spinner = true;
}


GAME( 1986, pbillian, 0,        pbillian,   pbillian, hotsmash_state, pbillian, ROT0,  "Kaneko / Taito", "Prebillian", MACHINE_SUPPORTS_SAVE )
GAME( 1987, hotsmash, 0,        pbillian,   hotsmash, hotsmash_state, hotsmash, ROT90, "Kaneko / Taito", "Vs. Hot Smash", MACHINE_SUPPORTS_SAVE )
GAME( 1987, sqix,     0,        sqix,       superqix, superqix_state, sqix,     ROT90, "Kaneko / Taito", "Super Qix (World/Japan, V1.2)", MACHINE_SUPPORTS_SAVE )
GAME( 1987, sqixr1,   sqix,     sqix,       superqix, superqix_state, sqix,     ROT90, "Kaneko / Taito", "Super Qix (World/Japan, V1.1)", MACHINE_SUPPORTS_SAVE )
GAME( 1987, sqixr0,   sqix,     sqix,       superqix, superqix_state, sqixr0,   ROT90, "Kaneko / Taito", "Super Qix (World/Japan, V1.0)", MACHINE_SUPPORTS_SAVE )
GAME( 1987, sqixu,    sqix,     sqix,       superqix, superqix_state, sqix,     ROT90, "Kaneko / Taito (Romstar License)", "Super Qix (US)", MACHINE_SUPPORTS_SAVE )
GAME( 1987, sqixb1,   sqix,     sqix_8031,  superqix, superqix_state, sqixr0,   ROT90, "bootleg", "Super Qix (bootleg of V1.0, 8031 MCU)", MACHINE_SUPPORTS_SAVE ) // bootleg of World, Rev 1
GAME( 1987, sqixb2,   sqix,     sqix_nomcu, superqix, superqix_state, sqix,     ROT90, "bootleg", "Super Qix (bootleg, No MCU)", MACHINE_SUPPORTS_SAVE ) // bootleg of World, Rev 1
GAME( 1994, perestro, 0,        sqix_nomcu, superqix, superqix_state, perestro, ROT90, "Promat", "Perestroika Girls", MACHINE_SUPPORTS_SAVE )
GAME( 1993, perestrof,perestro, sqix_nomcu, superqix, superqix_state, perestro, ROT90, "Promat (Fuuki license)", "Perestroika Girls (Fuuki license)", MACHINE_SUPPORTS_SAVE )
