// license:LGPL-2.1+
// copyright-holders:Angelo Salese, R. Belmont
/************************************************************************************

Sega Saturn SMPC - System Manager and Peripheral Control MCU simulation

The SMPC is actually a 4-bit Hitachi HD404920FS MCU, labeled with a Sega custom
315-5744 (that needs decapping)

MCU simulation by Angelo Salese & R. Belmont

TODO:
- timings;
- fix intback issue with inputs (according to the docs, it should fall in between
  VBLANK-IN and OUT, for obvious reasons);
- clean-ups;

*************************************************************************************/
/* SMPC Addresses

00
01 -w  Input Register 0 (IREG)
02
03 -w  Input Register 1
04
05 -w  Input Register 2
06
07 -w  Input Register 3
08
09 -w  Input Register 4
0a
0b -w  Input Register 5
0c
0d -w  Input Register 6
0e
0f
10
11
12
13
14
15
16
17
18
19
1a
1b
1c
1d
1e
1f -w  Command Register (COMREG)
20
21 r-  Output Register 0 (OREG)
22
23 r-  Output Register 1
24
25 r-  Output Register 2
26
27 r-  Output Register 3
28
29 r-  Output Register 4
2a
2b r-  Output Register 5
2c
2d r-  Output Register 6
2e
2f r-  Output Register 7
30
31 r-  Output Register 8
32
33 r-  Output Register 9
34
35 r-  Output Register 10
36
37 r-  Output Register 11
38
39 r-  Output Register 12
3a
3b r-  Output Register 13
3c
3d r-  Output Register 14
3e
3f r-  Output Register 15
40
41 r-  Output Register 16
42
43 r-  Output Register 17
44
45 r-  Output Register 18
46
47 r-  Output Register 19
48
49 r-  Output Register 20
4a
4b r-  Output Register 21
4c
4d r-  Output Register 22
4e
4f r-  Output Register 23
50
51 r-  Output Register 24
52
53 r-  Output Register 25
54
55 r-  Output Register 26
56
57 r-  Output Register 27
58
59 r-  Output Register 28
5a
5b r-  Output Register 29
5c
5d r-  Output Register 30
5e
5f r-  Output Register 31
60
61 r-  SR
62
63 rw  SF
64
65
66
67
68
69
6a
6b
6c
6d
6e
6f
70
71
72
73
74
75 rw PDR1
76
77 rw PDR2
78
79 -w DDR1
7a
7b -w DDR2
7c
7d -w IOSEL2/1
7e
7f -w EXLE2/1
*/

#include "emu.h"
#include "machine/smpc.h"
#include "includes/saturn.h" // FIXME: this is a dependency from devices on MAME

#include "machine/eepromser.h"
#include "screen.h"

#include "coreutil.h"


#define LOG_SMPC 0
#define LOG_PAD_CMD 0


/********************************************
 *
 * Bankswitch code for ST-V Multi Cart mode
 *
 *******************************************/

void saturn_state::stv_select_game(int gameno)
{
	if (m_prev_bankswitch != gameno)
	{
		if (m_cart_reg[gameno] && m_cart_reg[gameno]->base())
			memcpy(memregion("abus")->base(), m_cart_reg[gameno]->base(), 0x3000000);
		else
			memset(memregion("abus")->base(), 0x00, 0x3000000); // TODO: 1-filled?

		m_prev_bankswitch = gameno;
	}
}

/********************************************
 *
 * Command functions
 *
 *******************************************/

void saturn_state::smpc_master_on()
{
	m_maincpu->set_input_line(INPUT_LINE_RESET, CLEAR_LINE);
}

TIMER_CALLBACK_MEMBER( saturn_state::smpc_slave_enable )
{
	m_slave->set_input_line(INPUT_LINE_RESET, param ? ASSERT_LINE : CLEAR_LINE);
	m_smpc.OREG[31] = param + 0x02; //read-back for last command issued
	m_smpc.SF = 0x00; //clear hand-shake flag
	m_smpc.slave_on = param;
//  printf("%d %d\n",machine().first_screen()->hpos(),machine().first_screen()->vpos());
}

TIMER_CALLBACK_MEMBER( saturn_state::smpc_sound_enable )
{
	m_audiocpu->set_input_line(INPUT_LINE_RESET, param ? ASSERT_LINE : CLEAR_LINE);
	m_en_68k = param ^ 1;
	m_smpc.OREG[31] = param + 0x06; //read-back for last command issued
	m_smpc.SF = 0x00; //clear hand-shake flag
}

TIMER_CALLBACK_MEMBER( saturn_state::smpc_cd_enable )
{
	m_smpc.OREG[31] = param + 0x08; //read-back for last command issued
	m_smpc.SF = 0x08; //clear hand-shake flag (TODO: diagnostic wants this to have bit 3 high)
}

void saturn_state::smpc_system_reset()
{
	/*Only backup ram and SMPC ram are retained after that this command is issued.*/
	memset(m_scu_regs.get() ,0x00,0x000100);
	memset(m_scsp_regs.get(),0x00,0x001000);
	memset(m_sound_ram,0x00,0x080000);
	memset(m_workram_h,0x00,0x100000);
	memset(m_workram_l,0x00,0x100000);
	memset(m_vdp2_regs.get(),0x00,0x040000);
	memset(m_vdp2_vram.get(),0x00,0x100000);
	memset(m_vdp2_cram.get(),0x00,0x080000);
	memset(m_vdp1_vram.get(),0x00,0x100000);
	//A-Bus

	m_maincpu->set_input_line(INPUT_LINE_RESET, PULSE_LINE);
}

TIMER_CALLBACK_MEMBER( saturn_state::smpc_change_clock )
{
	uint32_t xtal;

	if(LOG_SMPC) printf ("Clock change execute at (%d %d)\n",machine().first_screen()->hpos(),machine().first_screen()->vpos());

	xtal = param ? MASTER_CLOCK_320 : MASTER_CLOCK_352;

	machine().device("maincpu")->set_unscaled_clock(xtal/2);
	machine().device("slave")->set_unscaled_clock(xtal/2);

	m_vdp2.dotsel = param ^ 1;
	stv_vdp2_dynamic_res_change();

	m_maincpu->set_input_line(INPUT_LINE_HALT, CLEAR_LINE);
	if(!m_NMI_reset)
		m_maincpu->set_input_line(INPUT_LINE_NMI, PULSE_LINE);
	m_slave->set_input_line(INPUT_LINE_HALT, CLEAR_LINE);
	m_slave->set_input_line(INPUT_LINE_RESET, ASSERT_LINE);
	m_audiocpu->set_input_line(INPUT_LINE_HALT, CLEAR_LINE);

	/* put issued command in OREG31 */
	m_smpc.OREG[31] = 0x0e + param;
	/* clear hand-shake flag */
	m_smpc.SF = 0x00;

	/* TODO: VDP1 / VDP2 / SCU / SCSP default power ON values? */
}

TIMER_CALLBACK_MEMBER( saturn_state::stv_intback_peripheral )
{
	if (m_smpc.intback_stage == 2)
	{
		m_smpc.SR = (0x80 | m_smpc.pmode);    // pad 2, no more data, echo back pad mode set by intback
		m_smpc.intback_stage = 0;
	}
	else
	{
		m_smpc.SR = (0xc0 | m_smpc.pmode);    // pad 1, more data, echo back pad mode set by intback
		m_smpc.intback_stage ++;
	}

	if(!(m_scu.ism & IRQ_SMPC))
		m_maincpu->set_input_line_and_vector(8, HOLD_LINE, 0x47);
	else
		m_scu.ist |= (IRQ_SMPC);

	m_smpc.OREG[31] = 0x10; /* callback for last command issued */
	m_smpc.SF = 0x00;    /* clear hand-shake flag */
}


TIMER_CALLBACK_MEMBER( saturn_state::stv_smpc_intback )
{
	int i;

//  printf("%02x %02x %02x\n",m_smpc.intback_buf[0],m_smpc.intback_buf[1],m_smpc.intback_buf[2]);

	if(m_smpc.intback_buf[0] != 0)
	{
		m_smpc.OREG[0] = (0x80) | ((m_NMI_reset & 1) << 6);

		for(i=0;i<7;i++)
			m_smpc.OREG[1+i] = m_smpc.rtc_data[i];

		m_smpc.OREG[8]=0x00;  // CTG0 / CTG1?

		m_smpc.OREG[9]=0x00;  // TODO: system region on Saturn

		m_smpc.OREG[10]= 0 << 7 |
								m_vdp2.dotsel << 6 |
								1 << 5 |
								1 << 4 |
								0 << 3 | //MSHNMI
								1 << 2 |
								0 << 1 | //SYSRES
								0 << 0;  //SOUNDRES
		m_smpc.OREG[11]= 0 << 6; //CDRES

		for(i=0;i<4;i++)
			m_smpc.OREG[12+i]=m_smpc.SMEM[i];

		for(i=0;i<15;i++)
			m_smpc.OREG[16+i]=0xff; // undefined

		m_smpc.intback_stage = (m_smpc.intback_buf[1] & 8) >> 3; // first peripheral
		m_smpc.SR = 0x40 | m_smpc.intback_stage << 5;
		m_smpc.pmode = m_smpc.intback_buf[0]>>4;

		//  /*This is for RTC,cartridge code and similar stuff...*/
		//if(LOG_SMPC) printf ("Interrupt: System Manager (SMPC) at scanline %04x, Vector 0x47 Level 0x08\n",scanline);
		if(!(m_scu.ism & IRQ_SMPC))
			m_maincpu->set_input_line_and_vector(8, HOLD_LINE, 0x47);
		else
			m_scu.ist |= (IRQ_SMPC);

		/* put issued command in OREG31 */
		m_smpc.OREG[31] = 0x10; // TODO: doc says 0?
		/* clear hand-shake flag */
		m_smpc.SF = 0x00;
	}
	else if(m_smpc.intback_buf[1] & 8)
	{
		m_smpc.intback_stage = (m_smpc.intback_buf[1] & 8) >> 3; // first peripheral
		m_smpc.SR = 0x40;
		m_smpc.OREG[31] = 0x10;
		machine().scheduler().timer_set(attotime::from_usec(0), timer_expired_delegate(FUNC(saturn_state::stv_intback_peripheral),this),0);
	}
	else
	{
		/* Shienryu calls this, it would be plainly illegal on Saturn, I'll just return the command and clear the hs flag for now. */
		m_smpc.OREG[31] = 0x10;
		m_smpc.SF = 0x00;
	}
}

/*
    [0] port status:
        0x04 Sega-tap
        0x16 Multi-tap
        0x2x clock serial peripheral
        0xf0 peripheral isn't connected
        0xf1 peripheral is connected
    [1] Peripheral ID (note: lowest four bits determines the size of the input packet)
        0x02 digital pad
        0x25 (tested by Game Basic?)
        0x34 keyboard

 Lower 4 bits of the port status tell the number of controllers to check for the port
 Lower 4 bits of the peripheral ID tell the number of registers used by each controller
 For multitap / segatap, we have implemented the following logic:
 SMPC reads in sequence
 - status for port 1
 - ID first controller, followed by the number of reads needed by the plugged controller
 - ID second controller, followed by the number of reads needed by the plugged controller
 - and so on... until the 4th (for SegaTap) or 6th (for Multitap) controller is read
 TODO: how does the multitap check if a controller is connected? does it ask for the
 controller status of each subport? how does this work exactly?
 currently, there is a small problem in some specific controller config which seems to
 lose track of one controller. E.g. if I put multitap in port2 with inserted joy1, joy2 and joy4
 it does not see joy4 controller, but if I put joy1, joy2, joy4 and joy5 it sees
 all four of them. The same happens if I skip controllers with id = 0xff...
 how did a real unit behave in this case?
*/

TIMER_CALLBACK_MEMBER( saturn_state::intback_peripheral )
{
//  if (LOG_SMPC) logerror("SMPC: providing PAD data for intback, pad %d\n", intback_stage-2);

	// doesn't work?
	//pad_num = m_smpc.intback_stage - 1;

	if(LOG_PAD_CMD) printf("%d %d %d\n", m_smpc.intback_stage - 1, machine().first_screen()->vpos(), (int)machine().first_screen()->frame_number());

	uint8_t status1 = m_ctrl1 ? m_ctrl1->read_status() : 0xf0;
	uint8_t status2 = m_ctrl2 ? m_ctrl2->read_status() : 0xf0;

	uint8_t reg_offset = 0;
	uint8_t ctrl1_offset = 0;     // this is used when there is segatap or multitap connected
	uint8_t ctrl2_offset = 0;     // this is used when there is segatap or multitap connected

	m_smpc.OREG[reg_offset++] = status1;
	// read ctrl1
	for (int i = 0; i < (status1 & 0xf); i++)
	{
		uint8_t id = m_ctrl1->read_id(i);
		m_smpc.OREG[reg_offset++] = id;
		for (int j = 0; j < (id & 0xf); j++)
			m_smpc.OREG[reg_offset++] = m_ctrl1->read_ctrl(j + ctrl1_offset);
		ctrl1_offset += (id & 0xf);
	}
	m_smpc.OREG[reg_offset++] = status2;
	// read ctrl2
	for (int i = 0; i < (status2 & 0xf); i++)
	{
		uint8_t id = m_ctrl2->read_id(i);
		m_smpc.OREG[reg_offset++] = id;
		for (int j = 0; j < (id & 0xf); j++)
			m_smpc.OREG[reg_offset++] = m_ctrl2->read_ctrl(j + ctrl2_offset);
		ctrl2_offset += (id & 0xf);
	}

	if (m_smpc.intback_stage == 2)
	{
		m_smpc.SR = (0x80 | m_smpc.pmode);    // pad 2, no more data, echo back pad mode set by intback
		m_smpc.intback_stage = 0;
	}
	else
	{
		m_smpc.SR = (0xc0 | m_smpc.pmode);    // pad 1, more data, echo back pad mode set by intback
		m_smpc.intback_stage ++;
	}

	if(!(m_scu.ism & IRQ_SMPC))
		m_maincpu->set_input_line_and_vector(8, HOLD_LINE, 0x47);
	else
		m_scu.ist |= (IRQ_SMPC);

	m_smpc.OREG[31] = 0x10; /* callback for last command issued */
	m_smpc.SF = 0x00;    /* clear hand-shake flag */
}

TIMER_CALLBACK_MEMBER( saturn_state::saturn_smpc_intback )
{
	if(m_smpc.intback_buf[0] != 0)
	{
		{
			int i;

			m_smpc.OREG[0] = (0x80) | ((m_NMI_reset & 1) << 6); // bit 7: SETTIME (RTC isn't setted up properly)

			for(i=0;i<7;i++)
				m_smpc.OREG[1+i] = m_smpc.rtc_data[i];

			m_smpc.OREG[8]=0x00;  //Cartridge code?

			m_smpc.OREG[9] = m_saturn_region;

			m_smpc.OREG[10]= 0 << 7 |
										m_vdp2.dotsel << 6 |
										1 << 5 |
										1 << 4 |
										0 << 3 | //MSHNMI
										1 << 2 |
										0 << 1 | //SYSRES
										0 << 0;  //SOUNDRES
			m_smpc.OREG[11]= 0 << 6; //CDRES

			for(i=0;i<4;i++)
				m_smpc.OREG[12+i]=m_smpc.SMEM[i];

			for(i=0;i<15;i++)
				m_smpc.OREG[16+i]=0xff; // undefined
		}

		m_smpc.intback_stage = (m_smpc.intback_buf[1] & 8) >> 3; // first peripheral
		m_smpc.SR = 0x40 | m_smpc.intback_stage << 5;
		m_smpc.pmode = m_smpc.intback_buf[0]>>4;

		if(!(m_scu.ism & IRQ_SMPC))
			m_maincpu->set_input_line_and_vector(8, HOLD_LINE, 0x47);
		else
			m_scu.ist |= (IRQ_SMPC);

		/* put issued command in OREG31 */
		m_smpc.OREG[31] = 0x10;
		/* clear hand-shake flag */
		m_smpc.SF = 0x00;
	}
	else if(m_smpc.intback_buf[1] & 8)
	{
		m_smpc.intback_stage = (m_smpc.intback_buf[1] & 8) >> 3; // first peripheral
		m_smpc.SR = 0x40;
		m_smpc.OREG[31] = 0x10;
		machine().scheduler().timer_set(attotime::from_usec(0), timer_expired_delegate(FUNC(saturn_state::intback_peripheral),this),0);
	}
	else
	{
		printf("SMPC intback bogus behaviour called %02x %02x\n",m_smpc.IREG[0],m_smpc.IREG[1]);
	}

}

void saturn_state::smpc_rtc_write()
{
	int i;

	for(i=0;i<7;i++)
		m_smpc.rtc_data[i] = m_smpc.IREG[i];
}

void saturn_state::smpc_memory_setting()
{
	int i;

	for(i=0;i<4;i++)
		m_smpc.SMEM[i] = m_smpc.IREG[i];
}

void saturn_state::smpc_nmi_req()
{
	/*NMI is unconditionally requested */
	m_maincpu->set_input_line(INPUT_LINE_NMI, PULSE_LINE);
}

TIMER_CALLBACK_MEMBER( saturn_state::smpc_nmi_set )
{
//  printf("%d %d\n",machine().first_screen()->hpos(),machine().first_screen()->vpos());

	m_NMI_reset = param;
	/* put issued command in OREG31 */
	m_smpc.OREG[31] = 0x19 + param;
	/* clear hand-shake flag */
	m_smpc.SF = 0x00;

	//m_smpc.OREG[0] = (0x80) | ((m_NMI_reset & 1) << 6);
}


TIMER_CALLBACK_MEMBER( saturn_state::smpc_audio_reset_line_pulse )
{
	m_audiocpu->set_input_line(INPUT_LINE_RESET, PULSE_LINE);
}

/********************************************
 *
 * COMREG sub-routine
 *
 *******************************************/

void saturn_state::smpc_comreg_exec(address_space &space, uint8_t data, uint8_t is_stv)
{
	switch (data)
	{
		case 0x00:
			if(LOG_SMPC) printf ("SMPC: Master ON\n");
			smpc_master_on();
			break;
		//case 0x01: Master OFF?
		case 0x02:
		case 0x03:
			if(LOG_SMPC) printf ("SMPC: Slave %s %d %d\n",(data & 1) ? "off" : "on",machine().first_screen()->hpos(),machine().first_screen()->vpos());
			machine().scheduler().timer_set(attotime::from_usec(15), timer_expired_delegate(FUNC(saturn_state::smpc_slave_enable),this),data & 1);
			break;
		case 0x06:
		case 0x07:
			if(LOG_SMPC) printf ("SMPC: Sound %s\n",(data & 1) ? "off" : "on");

			if(!is_stv)
				machine().scheduler().timer_set(attotime::from_usec(15), timer_expired_delegate(FUNC(saturn_state::smpc_sound_enable),this),data & 1);
			break;
		/*CD (SH-1) ON/OFF */
		case 0x08:
		case 0x09:
			printf ("SMPC: CD %s\n",(data & 1) ? "off" : "on");
			machine().scheduler().timer_set(attotime::from_usec(20), timer_expired_delegate(FUNC(saturn_state::smpc_cd_enable),this),data & 1);
			break;
		case 0x0a:
		case 0x0b:
			popmessage ("SMPC: NETLINK %s, contact MAMEdev",(data & 1) ? "off" : "on");
			break;      case 0x0d:
			if(LOG_SMPC) printf ("SMPC: System Reset\n");
			smpc_system_reset();
			break;
		case 0x0e:
		case 0x0f:
			if(LOG_SMPC) printf ("SMPC: Change Clock to %s (%d %d)\n",data & 1 ? "320" : "352",machine().first_screen()->hpos(),machine().first_screen()->vpos());

			/* on ST-V timing of this is pretty fussy, you get 2 credits at start-up otherwise
			   My current theory is that SMPC first stops all CPUs until it executes the whole snippet for this,
			   and restarts them when the screen is again ready for use. I really don't think that the system
			   can do an usable mid-frame clock switching anyway.
			   */

			m_maincpu->set_input_line(INPUT_LINE_HALT, ASSERT_LINE);
			m_slave->set_input_line(INPUT_LINE_HALT, ASSERT_LINE);
			m_audiocpu->set_input_line(INPUT_LINE_HALT, ASSERT_LINE);

			machine().scheduler().timer_set(machine().first_screen()->time_until_pos(get_vblank_start_position()*get_ystep_count(), 0), timer_expired_delegate(FUNC(saturn_state::smpc_change_clock),this),data & 1);
			break;
		/*"Interrupt Back"*/
		case 0x10:
			if(0)
			{
				printf ("SMPC: Status Acquire %02x %02x %02x %d\n",m_smpc.IREG[0],m_smpc.IREG[1],m_smpc.IREG[2],machine().first_screen()->vpos());
			}

			int timing;

			timing = 8;

			if(m_smpc.IREG[0] != 0) // non-peripheral data
				timing += 8;

			/* TODO: At vblank-out actually ... */
			if(m_smpc.IREG[1] & 8) // peripheral data
				timing += 700;

			/* TODO: check if IREG[2] is setted to 0xf0 */
			{
				int i;

				for(i=0;i<3;i++)
					m_smpc.intback_buf[i] = m_smpc.IREG[i];
			}

			if(is_stv)
			{
				machine().scheduler().timer_set(attotime::from_usec(timing), timer_expired_delegate(FUNC(saturn_state::stv_smpc_intback),this),0); //TODO: variable time
			}
			else
			{
				if(LOG_PAD_CMD) printf("INTBACK %02x %02x %d %d\n",m_smpc.IREG[0],m_smpc.IREG[1],machine().first_screen()->vpos(),(int)machine().first_screen()->frame_number());
				machine().scheduler().timer_set(attotime::from_usec(timing), timer_expired_delegate(FUNC(saturn_state::saturn_smpc_intback),this),0); //TODO: is variable time correct?
			}
			break;
		/* RTC write*/
		case 0x16:
			if(LOG_SMPC) printf("SMPC: RTC write\n");
			smpc_rtc_write();
			break;
		/* SMPC memory setting*/
		case 0x17:
			if(LOG_SMPC) printf ("SMPC: memory setting\n");
			smpc_memory_setting();
			break;
		case 0x18:
			if(LOG_SMPC) printf ("SMPC: NMI request\n");
			smpc_nmi_req();
			break;
		case 0x19:
		case 0x1a:
			/* TODO: timing */
			if(LOG_SMPC) printf ("SMPC: NMI %sable %d %d\n",data & 1 ? "Dis" : "En",machine().first_screen()->hpos(),machine().first_screen()->vpos());
			machine().scheduler().timer_set(attotime::from_usec(100), timer_expired_delegate(FUNC(saturn_state::smpc_nmi_set),this),data & 1);
			break;
		default:
			printf ("cpu '%s' (PC=%08X) SMPC: undocumented Command %02x\n", space.device().tag(), space.device().safe_pc(), data);
	}
}

/********************************************
 *
 * ST-V handlers
 *
 *******************************************/

READ8_MEMBER( saturn_state::stv_SMPC_r )
{
	int return_data = 0;

	if(!(offset & 1))
		return 0;

	if(offset >= 0x21 && offset <= 0x5f)
		return_data = m_smpc.OREG[(offset-0x21) >> 1];

	if (offset == 0x61) // TODO: SR
		return_data = m_smpc.SR;

	if (offset == 0x63)
		return_data = m_smpc.SF;

	if (offset == 0x75)//PDR1 read
		return_data = ioport("DSW1")->read();

	if (offset == 0x77)//PDR2 read
		return_data = (0xfe | m_eeprom->do_read());

	return return_data;
}

WRITE8_MEMBER( saturn_state::stv_SMPC_w )
{
	if (!(offset & 1)) // avoid writing to even bytes
		return;

//  if(LOG_SMPC) printf ("8-bit SMPC Write to Offset %02x with Data %02x\n", offset, data);

	if(offset >= 1 && offset <= 0xd)
		m_smpc.IREG[offset >> 1] = data;

	if(offset == 1) //IREG0, check if a BREAK / CONTINUE request for INTBACK command
	{
		if(m_smpc.intback_stage)
		{
			if(data & 0x40)
			{
				if(LOG_PAD_CMD) printf("SMPC: BREAK request\n");
				m_smpc.SR &= 0x0f;
				m_smpc.intback_stage = 0;
			}
			else if(data & 0x80)
			{
				if(LOG_PAD_CMD) printf("SMPC: CONTINUE request\n");
				machine().scheduler().timer_set(attotime::from_usec(700), timer_expired_delegate(FUNC(saturn_state::stv_intback_peripheral),this),0); /* TODO: is timing correct? */
				m_smpc.OREG[31] = 0x10;
				m_smpc.SF = 0x01; //TODO: set hand-shake flag?
			}
		}
	}

	if (offset == 0x1f) // COMREG
	{
		smpc_comreg_exec(space,data,1);

		// we've processed the command, clear status flag
		if(data != 0x10 && data != 0x02 && data != 0x03 && data != 0x08 && data != 0x09 && data != 0xe && data != 0xf && data != 0x19 && data != 0x1a)
		{
			m_smpc.OREG[31] = data; //read-back command
			m_smpc.SF = 0x00;
		}
		/*TODO:emulate the timing of each command...*/
	}

	if(offset == 0x63)
		m_smpc.SF = data & 1;

	if(offset == 0x75)
	{
		/*
		-xx- ---- PDR1
		---x ---- EEPROM write bit
		---- x--- EEPROM CLOCK line
		---- -x-- EEPROM CS line
		---- --xx A-Bus bank bits
		*/
		m_eeprom->clk_write((data & 0x08) ? ASSERT_LINE : CLEAR_LINE);
		m_eeprom->di_write((data >> 4) & 1);
		m_eeprom->cs_write((data & 0x04) ? ASSERT_LINE : CLEAR_LINE);
		m_stv_multi_bank = data & 3;

		stv_select_game(m_stv_multi_bank);

		m_smpc.PDR1 = (data & 0x60);
	}

	if(offset == 0x77)
	{
		/*
		    -xx- ---- PDR2
		    ---x ---- Enable Sound System (ACTIVE LOW)
		*/
		//popmessage("PDR2 = %02x",m_smpc_ram[0x77]);

		if(LOG_SMPC) printf("SMPC: M68k %s\n",(data & 0x10) ? "off" : "on");
		//machine().scheduler().timer_set(attotime::from_usec(100), timer_expired_delegate(FUNC(saturn_state::smpc_sound_enable),this),(m_smpc_ram[0x77] & 0x10) >> 4);
		m_audiocpu->set_input_line(INPUT_LINE_RESET, (data & 0x10) ? ASSERT_LINE : CLEAR_LINE);
		m_en_68k = ((data & 0x10) >> 4) ^ 1;

		//if(LOG_SMPC) printf("SMPC: ram [0x77] = %02x\n",data);
		m_smpc.PDR2 = (data & 0x60);
	}

	if(offset == 0x7d)
	{
		/*
		---- --x- IOSEL2 direct (1) / control mode (0) port select
		---- ---x IOSEL1 direct (1) / control mode (0) port select
		*/
		m_smpc.IOSEL1 = (data & 1) >> 0;
		m_smpc.IOSEL2 = (data & 2) >> 1;
	}

	if(offset == 0x7f)
	{
		//enable PAD irq & VDP2 external latch for port 1/2
		m_smpc.EXLE1 = (data & 1) >> 0;
		m_smpc.EXLE2 = (data & 2) >> 1;
	}
}

/********************************************
 *
 * Saturn handlers
 *
 *******************************************/

uint8_t saturn_state::smpc_th_control_mode(uint8_t pad_n)
{
	uint8_t res = 0;
	int th = (pad_n == 0) ? ((m_smpc.PDR1 >> 5) & 3) : ((m_smpc.PDR2 >> 5) & 3);

	uint16_t ctrl_read = 0;
	if (m_ctrl1 && pad_n == 0)
		ctrl_read = m_ctrl1->read_direct();
	if (m_ctrl2 && pad_n == 1)
		ctrl_read = m_ctrl2->read_direct();

	if (LOG_SMPC) printf("SMPC: SH-2 TH control mode, returning pad data %d for phase %d\n", pad_n + 1, th);

	switch (th)
	{
		/* TODO: 3D Lemmings bogusly enables TH Control mode, wants this to return the ID, needs HW tests.  */
		case 3:
			res = th << 6;
			res |= 0x14;
			res |= (ctrl_read & 8); // L
			break;
		case 2:
			res = th << 6;
			//  1 C B Right Left Down Up
			//  WHP actually has a very specific code at 0x6015f30, doesn't like bits 0-1 active here ...
			res|= ((ctrl_read >>  4) & 0x30); // C & B
			res|= ((ctrl_read >> 12) & 0xc);
			break;
		case 1:
			res = th << 6;
			res |= 0x10;
			res |= ((ctrl_read >> 4) & 0xf); // R, X, Y, Z
			break;
		case 0:
			res = th << 6;
			//  0 Start A 0 0    Down Up
			res |= ((ctrl_read >>  6) & 0x30); // Start & A
			res |= ((ctrl_read >> 12) & 0x03);
			//  ... and it actually wants bits 2 - 3 active here.
			res |= 0xc;
			break;
	}

	return res;
}

uint8_t saturn_state::smpc_direct_mode(uint8_t pad_n)
{
	int hshake = (pad_n == 0) ? ((m_smpc.PDR1 >> 5) & 3) : ((m_smpc.PDR2 >> 5) & 3);
	const int shift_bit[4] = { 4, 12, 8, 0 };

	uint16_t ctrl_read = 0;
	if (m_ctrl1 && pad_n == 0)
		ctrl_read = m_ctrl1->read_direct();
	if (m_ctrl2 && pad_n == 1)
		ctrl_read = m_ctrl2->read_direct();

	if (LOG_SMPC) logerror("SMPC: SH-2 direct mode, returning data for phase %d\n", hshake);

	return 0x80 | 0x10 | ((ctrl_read >> shift_bit[hshake]) & 0xf);
}

READ8_MEMBER( saturn_state::saturn_SMPC_r )
{
	uint8_t return_data = 0;

	if (!(offset & 1)) // avoid reading to even bytes (TODO: is it 0s or 1s?)
		return 0x00;

	if(offset >= 0x21 && offset <= 0x5f)
		return_data = m_smpc.OREG[(offset-0x21) >> 1];

	if (offset == 0x61)
		return_data = m_smpc.SR;

	if (offset == 0x63)
	{
		//printf("SF %d %d\n",machine().first_screen()->hpos(),machine().first_screen()->vpos());
		return_data = m_smpc.SF;
	}

	if (offset == 0x75 || offset == 0x77)//PDR1/2 read
	{
		if ((m_smpc.IOSEL1 && offset == 0x75) || (m_smpc.IOSEL2 && offset == 0x77))
		{
			uint8_t cur_ddr;

			if (((m_ctrl1 && m_ctrl1->read_id(0) != 0x02) || (m_ctrl2 && m_ctrl2->read_id(0) != 0x02)) && !(machine().side_effect_disabled()))
			{
				popmessage("Warning: read with SH-2 direct mode with a non-pad device");
				return 0;
			}

			cur_ddr = (offset == 0x75) ? m_smpc.DDR1 : m_smpc.DDR2;

			switch(cur_ddr & 0x60)
			{
				case 0x00: break; // in diag test
				case 0x40: return_data = smpc_th_control_mode(offset == 0x77); break;
				case 0x60: return_data = smpc_direct_mode(offset == 0x77); break;
				default:
					popmessage("SMPC: unemulated control method %02x, contact MAMEdev",cur_ddr & 0x60);
					return_data = 0;
					break;
			}
		}
	}

	if (LOG_SMPC) logerror ("cpu %s (PC=%08X) SMPC: Read from Byte Offset %02x (%d) Returns %02x\n", space.device().tag(), space.device().safe_pc(), offset, offset>>1, return_data);

	return return_data;
}

WRITE8_MEMBER( saturn_state::saturn_SMPC_w )
{
	if (LOG_SMPC) logerror ("8-bit SMPC Write to Offset %02x (reg %d) with Data %02x\n", offset, offset>>1, data);

	if (!(offset & 1)) // avoid writing to even bytes
		return;

	if(offset >= 1 && offset <= 0xd)
		m_smpc.IREG[offset >> 1] = data;

	if(offset == 1) //IREG0, check if a BREAK / CONTINUE request for INTBACK command
	{
		if(m_smpc.intback_stage)
		{
			if(data & 0x40)
			{
				if(LOG_PAD_CMD) printf("SMPC: BREAK request %02x\n",data);
				m_smpc.SR &= 0x0f;
				m_smpc.intback_stage = 0;
			}
			else if(data & 0x80)
			{
				if(LOG_PAD_CMD) printf("SMPC: CONTINUE request %02x\n",data);
				machine().scheduler().timer_set(attotime::from_usec(700), timer_expired_delegate(FUNC(saturn_state::intback_peripheral),this),0); /* TODO: is timing correct? */
				m_smpc.OREG[31] = 0x10;
				m_smpc.SF = 0x01; //TODO: set hand-shake flag?
			}
		}
	}

	if (offset == 0x1f)
	{
		smpc_comreg_exec(space,data,0);

		// we've processed the command, clear status flag
		if(data != 0x10 && data != 2 && data != 3 && data != 6 && data != 7 && data != 0x08 && data != 0x09 && data != 0x0e && data != 0x0f && data != 0x19 && data != 0x1a)
		{
			m_smpc.OREG[31] = data; //read-back for last command issued
			m_smpc.SF = 0x00; //clear hand-shake flag
		}
		/*TODO:emulate the timing of each command...*/
	}

	if (offset == 0x63)
		m_smpc.SF = data & 1; // hand-shake flag

	if(offset == 0x75)  // PDR1
		m_smpc.PDR1 = data & 0x7f;

	if(offset == 0x77)  // PDR2
		m_smpc.PDR2 = data & 0x7f;

	if(offset == 0x79)
		m_smpc.DDR1 = data & 0x7f;

	if(offset == 0x7b)
		m_smpc.DDR2 = data & 0x7f;

	if(offset == 0x7d)
	{
		m_smpc.IOSEL1 = data & 1;
		m_smpc.IOSEL2 = (data & 2) >> 1;
	}

	if(offset == 0x7f)
	{
		//enable PAD irq & VDP2 external latch for port 1/2
		m_smpc.EXLE1 = (data & 1) >> 0;
		m_smpc.EXLE2 = (data & 2) >> 1;
	}
}
