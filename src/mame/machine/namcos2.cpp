// license:BSD-3-Clause
// copyright-holders:K.Wilkins
/***************************************************************************

Namco System II

  machine.c

  Functions to emulate general aspects of the machine (RAM, ROM, interrupts,
  I/O ports)

***************************************************************************/

#include "emu.h"
#include "cpu/m6809/m6809.h"
#include "cpu/m6805/m6805.h"
#include "includes/namcos2.h"
#include "machine/nvram.h"


void (*namcos2_kickstart)(running_machine &machine, int internal);


READ16_MEMBER( namcos2_state::namcos2_finallap_prot_r )
{
	static const uint16_t table0[8] = { 0x0000,0x0040,0x0440,0x2440,0x2480,0xa080,0x8081,0x8041 };
	static const uint16_t table1[8] = { 0x0040,0x0060,0x0060,0x0860,0x0864,0x08e4,0x08e5,0x08a5 };
	uint16_t data;

	switch( offset )
	{
	case 0:
		data = 0x0101;
		break;

	case 1:
		data = 0x3e55;
		break;

	case 2:
		data = table1[m_finallap_prot_count&7];
		data = (data&0xff00)>>8;
		break;

	case 3:
		data = table1[m_finallap_prot_count&7];
		m_finallap_prot_count++;
		data = data&0x00ff;
		break;

	case 0x3fffc/2:
		data = table0[m_finallap_prot_count&7];
		data = data&0xff00;
		break;

	case 0x3fffe/2:
		data = table0[m_finallap_prot_count&7];
		m_finallap_prot_count++;
		data = (data&0x00ff)<<8;
		break;

	default:
		data = 0;
	}
	return data;
}

/*************************************************************/
/* Perform basic machine initialisation                      */
/*************************************************************/

#define m_eeprom_size 0x2000

WRITE8_MEMBER(namcos2_shared_state::sound_reset_w)
{
	address_space &masterspace = m_maincpu->space(AS_PROGRAM);

	if (data & 0x01)
	{
		/* Resume execution */
		m_audiocpu->set_input_line(INPUT_LINE_RESET, CLEAR_LINE);
		masterspace.device().execute().yield();
	}
	else
	{
		/* Suspend execution */
		m_audiocpu->set_input_line(INPUT_LINE_RESET, ASSERT_LINE);
	}

	if (namcos2_kickstart != nullptr)
	{
		//printf( "dspkick=0x%x\n", data );
		if (data & 0x04)
		{
			(*namcos2_kickstart)(space.machine(), 1);
		}
	}
}

// TODO:
WRITE8_MEMBER(namcos2_shared_state::system_reset_w)
{
	reset_all_subcpus(data & 1 ? CLEAR_LINE : ASSERT_LINE);

	if (data & 0x01)
	{
		address_space &masterspace = m_maincpu->space(AS_PROGRAM);
		masterspace.device().execute().yield();
	}
}

void namcos2_shared_state::reset_all_subcpus(int state)
{
	m_slave->set_input_line(INPUT_LINE_RESET, state);
	if (m_c68)
	{
		m_c68->set_input_line(INPUT_LINE_RESET, state);
	}
	else
	{
		m_mcu->set_input_line(INPUT_LINE_RESET, state);
	}
	switch( m_gametype )
	{
	case NAMCOS21_SOLVALOU:
	case NAMCOS21_STARBLADE:
	case NAMCOS21_AIRCOMBAT:
	case NAMCOS21_CYBERSLED:
		m_dspmaster->set_input_line(INPUT_LINE_RESET, state);
		m_dspslave->set_input_line(INPUT_LINE_RESET, state);
		break;

//  case NAMCOS21_WINRUN91:
//  case NAMCOS21_DRIVERS_EYES:
	default:
		break;
	}
}

MACHINE_START_MEMBER(namcos2_shared_state,namcos2)
{
	namcos2_kickstart = nullptr;
	m_eeprom = std::make_unique<uint8_t[]>(m_eeprom_size);
	machine().device<nvram_device>("nvram")->set_base(m_eeprom.get(), m_eeprom_size);
}

MACHINE_RESET_MEMBER(namcos2_shared_state, namcos2)
{
//  address_space &space = m_maincpu->space(AS_PROGRAM);
	address_space &audio_space = m_audiocpu->space(AS_PROGRAM);

	m_mcu_analog_ctrl = 0;
	m_mcu_analog_data = 0xaa;
	m_mcu_analog_complete = 0;

	/* Initialise the bank select in the sound CPU */
	namcos2_sound_bankselect_w(audio_space, 0, 0); /* Page in bank 0 */

	m_audiocpu->set_input_line(INPUT_LINE_RESET, ASSERT_LINE );

	/* Place CPU2 & CPU3 into the reset condition */
	reset_all_subcpus(ASSERT_LINE);

	m_player_mux = 0;
}

/*************************************************************/
/* EEPROM Load/Save and read/write handling                  */
/*************************************************************/

WRITE8_MEMBER( namcos2_shared_state::namcos2_68k_eeprom_w )
{
	m_eeprom[offset] = data;
}

READ8_MEMBER( namcos2_shared_state::namcos2_68k_eeprom_r )
{
	return m_eeprom[offset];
}

/*************************************************************/
/* 68000 Shared protection/random key generator              */
/*************************************************************

Custom chip ID numbers:

Game        Year    ID (dec)    ID (hex)
--------    ----    ---         -----
finallap    1987
assault     1988    unused
metlhawk    1988
ordyne      1988    176         $00b0
mirninja    1988    177         $00b1
phelios     1988    178         $00b2   readme says 179
dirtfoxj    1989    180         $00b4
fourtrax    1989
valkyrie    1989
finehour    1989    188         $00bc
burnforc    1989    189         $00bd
marvland    1989    190         $00be
kyukaidk    1990    191         $00bf
dsaber      1990    192         $00c0
finalap2    1990    318         $013e
rthun2      1990    319         $013f
gollygho    1990                $0143
cosmogng    1991    330         $014a
sgunner2    1991    346         $015a   ID out of order; gfx board is not standard
finalap3    1992    318         $013e   same as finalap2
suzuka8h    1992
sws92       1992    331         $014b
sws92g      1992    332         $014c
suzuk8h2    1993
sws93       1993    334         $014e
 *************************************************************/

READ16_MEMBER( namcos2_state::namcos2_68k_key_r )
{
	switch (m_gametype)
	{
	case NAMCOS2_ORDYNE:
		switch(offset)
		{
		case 2: return 0x1001;
		case 3: return 0x1;
		case 4: return 0x110;
		case 5: return 0x10;
		case 6: return 0xB0;
		case 7: return 0xB0;
		}
		break;

	case NAMCOS2_STEEL_GUNNER_2:
		switch( offset )
		{
			case 4: return 0x15a;
		}
		break;

	case NAMCOS2_MIRAI_NINJA:
		switch(offset)
		{
		case 7: return 0xB1;
		}
		break;

	case NAMCOS2_PHELIOS:
		switch(offset)
		{
		case 0: return 0xF0;
		case 1: return 0xFF0;
		case 2: return 0xB2;
		case 3: return 0xB2;
		case 4: return 0xF;
		case 5: return 0xF00F;
		case 7: return 0xB2;
		}
		break;

	case NAMCOS2_DIRT_FOX_JP:
		switch(offset)
		{
		case 1: return 0xB4;
		}
		break;

	case NAMCOS2_FINEST_HOUR:
		switch(offset)
		{
		case 7: return 0xBC;
		}
		break;

	case NAMCOS2_BURNING_FORCE:
		switch(offset)
		{
		case 1: return 0xBD;
		}
		break;

	case NAMCOS2_MARVEL_LAND:
		switch(offset)
		{
		case 0: return 0x10;
		case 1: return 0x110;
		case 4: return 0xBE;
		case 6: return 0x1001;
		case 7: return (m_sendval==1)?0xBE:1;
		}
		break;

	case NAMCOS2_DRAGON_SABER:
		switch(offset)
		{
		case 2: return 0xC0;
		}
		break;

	case NAMCOS2_ROLLING_THUNDER_2:
		switch(offset)
		{
		case 4:
			if( m_sendval == 1 ){
				m_sendval = 0;
				return 0x13F;
			}
			break;
		case 7:
			if( m_sendval == 1 ){
				m_sendval = 0;
				return 0x13F;
			}
			break;
		case 2: return 0;
		}
		break;

	case NAMCOS2_COSMO_GANG:
		switch(offset)
		{
		case 3: return 0x14A;
		}
		break;

	case NAMCOS2_SUPER_WSTADIUM:
		switch(offset)
		{
	//  case 3: return 0x142;
		case 4: return 0x142;
	//  case 3: popmessage("blah %08x",space.device().safe_pc());
		default: return space.machine().rand();
		}

	case NAMCOS2_SUPER_WSTADIUM_92:
		switch(offset)
		{
		case 3: return 0x14B;
		}
		break;

	case NAMCOS2_SUPER_WSTADIUM_92T:
		switch(offset)
		{
		case 3: return 0x14C;
		}
		break;

	case NAMCOS2_SUPER_WSTADIUM_93:
		switch(offset)
		{
		case 3: return 0x14E;
		}
		break;

	case NAMCOS2_SUZUKA_8_HOURS_2:
		switch(offset)
		{
		case 3: return 0x14D;
		case 2: return 0;
		}
		break;

	case NAMCOS2_GOLLY_GHOST:
		switch(offset)
		{
		case 0: return 2;
		case 1: return 2;
		case 2: return 0;
		case 4: return 0x143;
		}
		break;


	case NAMCOS2_BUBBLE_TROUBLE:
		switch(offset)
		{
		case 0: return 2; // not verified
		case 1: return 2; // not verified
		case 2: return 0; // not verified
		case 4: return 0x141;
		}
		break;
	}



	return space.machine().rand()&0xffff;
}

WRITE16_MEMBER( namcos2_state::namcos2_68k_key_w )
{
	int gametype = m_gametype;
	if( gametype == NAMCOS2_MARVEL_LAND && offset == 5 )
	{
		if (data == 0x615E) m_sendval = 1;
	}
	if( gametype == NAMCOS2_ROLLING_THUNDER_2 && offset == 4 )
	{
		if (data == 0x13EC) m_sendval = 1;
	}
	if( gametype == NAMCOS2_ROLLING_THUNDER_2 && offset == 7 )
	{
		if (data == 0x13EC) m_sendval = 1;
	}
	if( gametype == NAMCOS2_MARVEL_LAND && offset == 6 )
	{
		if (data == 0x1001) m_sendval = 0;
	}
}

/*************************************************************/
/* 68000 Interrupt/IO Handlers - CUSTOM 148 - NOT SHARED     */
/*************************************************************/

#define NO_OF_LINES     256
#define FRAME_TIME      (1.0/60.0)
#define LINE_LENGTH     (FRAME_TIME/NO_OF_LINES)


bool namcos2_shared_state::is_system21()
{
	switch (m_gametype)
	{
		case NAMCOS21_AIRCOMBAT:
		case NAMCOS21_STARBLADE:
		case NAMCOS21_CYBERSLED:
		case NAMCOS21_SOLVALOU:
		case NAMCOS21_WINRUN91:
		case NAMCOS21_DRIVERS_EYES:
			return 1;
		default:
			return 0;
	}
}


/**************************************************************/
/*  Sound sub-system                                          */
/**************************************************************/

WRITE8_MEMBER( namcos2_shared_state::namcos2_sound_bankselect_w )
{
	uint8_t *RAM= memregion("audiocpu")->base();
	uint32_t max = (memregion("audiocpu")->bytes() - 0x10000) / 0x4000;
	int bank = ( data >> 4 ) % max; /* 991104.CAB */
	membank(BANKED_SOUND_ROM)->set_base(&RAM[ 0x10000 + ( 0x4000 * bank ) ] );
}

/**************************************************************/
/*                                                            */
/*  68705 IO CPU Support functions                            */
/*                                                            */
/**************************************************************/

WRITE8_MEMBER( namcos2_shared_state::namcos2_mcu_analog_ctrl_w )
{
	m_mcu_analog_ctrl = data & 0xff;

	/* Check if this is a start of conversion */
	/* Input ports 2 through 9 are the analog channels */

	if(data & 0x40)
	{
	/* Set the conversion complete flag */
		m_mcu_analog_complete = 2;
		/* We convert instantly, good eh! */
		switch((data>>2) & 0x07)
		{
		case 0:
			m_mcu_analog_data=ioport("AN0")->read();
			break;
		case 1:
			m_mcu_analog_data=ioport("AN1")->read();
			break;
		case 2:
			m_mcu_analog_data=ioport("AN2")->read();
			break;
		case 3:
			m_mcu_analog_data=ioport("AN3")->read();
			break;
		case 4:
			m_mcu_analog_data=ioport("AN4")->read();
			break;
		case 5:
			m_mcu_analog_data=ioport("AN5")->read();
			break;
		case 6:
			m_mcu_analog_data=ioport("AN6")->read();
			break;
		case 7:
			m_mcu_analog_data=ioport("AN7")->read();
			break;
		default:
			output().set_value("anunk",data);
		}
#if 0
		/* Perform the offset handling on the input port */
		/* this converts it to a twos complement number */
		if( m_gametype == NAMCOS2_DIRT_FOX ||
			m_gametype == NAMCOS2_DIRT_FOX_JP )
		{
			m_mcu_analog_data ^= 0x80;
		}
#endif
		/* If the interrupt enable bit is set trigger an A/D IRQ */
		if(data & 0x20)
		{
			generic_pulse_irq_line(*m_mcu, HD63705_INT_ADCONV, 1);
		}
	}
}

READ8_MEMBER( namcos2_shared_state::namcos2_mcu_analog_ctrl_r )
{
	int data=0;

	/* ADEF flag is only cleared AFTER a read from control THEN a read from DATA */
	if(m_mcu_analog_complete==2) m_mcu_analog_complete=1;
	if(m_mcu_analog_complete) data|=0x80;

	/* Mask on the lower 6 register bits, Irq EN/Channel/Clock */
	data|=m_mcu_analog_ctrl&0x3f;
	/* Return the value */
	return data;
}

WRITE8_MEMBER( namcos2_shared_state::namcos2_mcu_analog_port_w )
{
}

READ8_MEMBER( namcos2_shared_state::namcos2_mcu_analog_port_r )
{
	if(m_mcu_analog_complete==1) m_mcu_analog_complete=0;
	return m_mcu_analog_data;
}

WRITE8_MEMBER( namcos2_shared_state::namcos2_mcu_port_d_w )
{
	/* Undefined operation on write */
}

READ8_MEMBER( namcos2_shared_state::namcos2_mcu_port_d_r )
{
	/* Provides a digital version of the analog ports */
	int threshold = 0x7f;
	int data = 0;

	/* Read/convert the bits one at a time */
	if(ioport("AN0")->read() > threshold) data |= 0x01;
	if(ioport("AN1")->read() > threshold) data |= 0x02;
	if(ioport("AN2")->read() > threshold) data |= 0x04;
	if(ioport("AN3")->read() > threshold) data |= 0x08;
	if(ioport("AN4")->read() > threshold) data |= 0x10;
	if(ioport("AN5")->read() > threshold) data |= 0x20;
	if(ioport("AN6")->read() > threshold) data |= 0x40;
	if(ioport("AN7")->read() > threshold) data |= 0x80;

	/* Return the result */
	return data;
}
