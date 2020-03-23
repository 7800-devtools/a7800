// license:BSD-3-Clause
// copyright-holders:Couriersud
#include "emu.h"
#include "includes/mario.h"

#include "cpu/mcs48/mcs48.h"
#include "cpu/z80/z80.h"
#include "sound/ay8910.h"
#include "speaker.h"

#include "audio/nl_mario.h"

/****************************************************************
 *
 * Defines and Macros
 *
 ****************************************************************/

#define RUN_VCO_VOLTAGE     (0.0)   /* 5 in schematics */

#define USE_8039    (0)         /* set to 1 to try 8039 hack */

#define ACTIVELOW_PORT_BIT(P,A,D)   ((P & (~(1 << A))) | ((D ^ 1) << A))
#define ACTIVEHIGH_PORT_BIT(P,A,D)   ((P & (~(1 << A))) | (D << A))

#define I8035_T_R(M,N) ((m_soundlatch2->read(M,0) >> (N)) & 1)
#define I8035_T_W_AH(M,N,D) do { m_portT = ACTIVEHIGH_PORT_BIT(m_portT,N,D); m_soundlatch2->write(M, 0, m_portT); } while (0)

#define I8035_P1_R(M) (m_soundlatch3->read(M,0))
#define I8035_P2_R(M) (m_soundlatch4->read(M,0))
#define I8035_P1_W(M,D) m_soundlatch3->write(M,0,D)

#if (USE_8039)
#define I8035_P2_W(M,D) do { m_soundlatch4->write(M,0,D); } while (0)
#else
#define I8035_P2_W(M,D) do { set_ea(M, ((D) & 0x20) ? 0 : 1);  m_soundlatch4->write(M,0,D); } while (0)
#endif

#define I8035_P1_W_AH(M,B,D) I8035_P1_W(M,ACTIVEHIGH_PORT_BIT(I8035_P1_R(M),B,(D)))
#define I8035_P2_W_AH(M,B,D) I8035_P2_W(M,ACTIVEHIGH_PORT_BIT(I8035_P2_R(M),B,(D)))


#if !OLD_SOUND


#else
/****************************************************************
 *
 * Discrete Sound defines
 *
 ****************************************************************/

/* Discrete sound inputs */

#define DS_SOUND0_INV       NODE_01
#define DS_SOUND1_INV       NODE_02
#define DS_SOUND7_INV       NODE_05
#define DS_DAC              NODE_07

#define DS_SOUND0           NODE_208
#define DS_SOUND1           NODE_209
#define DS_SOUND7           NODE_212

#define DS_OUT_SOUND0       NODE_241
#define DS_OUT_SOUND1       NODE_242
#define DS_OUT_SOUND7       NODE_248
#define DS_OUT_DAC          NODE_250

/* Input definitions for write handlers */

#define DS_SOUND0_INP       DS_SOUND0_INV
#define DS_SOUND1_INP       DS_SOUND1_INV
#define DS_SOUND7_INP       DS_SOUND7_INV

/* General defines */

#define VSS                 5.0
#define TTL_HIGH            4.0
#define GND                 0.0

/****************************************************************
 *
 * Mario Discrete Sound Interface
 *
 * Parts verified against a real TMA1-04-CPU Board.
 ****************************************************************/

#define MR_R6       RES_K(4.7)      /* verified                             */
#define MR_R7       RES_K(4.7)      /* verified                             */
#define MR_R17      RES_K(27)       /* 20 according to parts list           */
									/* 27 verified, 30K in schematics       */
#define MR_R18      RES_K(27)       /* 20 according to parts list           */
									/* 27 verified, 30K in schematics       */
#define MR_R19      RES_K(22)       /* verified                             */
#define MR_R20      RES_K(22)       /* verified                             */
#define MR_R34      RES_M(2)        /*                             */
#define MR_R35      RES_M(1)        /*                             */
#define MR_R36      RES_M(1.8)      /*                             */
#define MR_R40      RES_K(22)       /* verified                             */
#define MR_R41      RES_K(100)      /* verified                             */
#define MR_R42      RES_K(43)       /* verified                             */
#define MR_R43      RES_K(100)      /* verified                             */
#define MR_R61      RES_K(47)       /* verified                             */
#define MR_R64      RES_K(20)       /* verified                             */
#define MR_R65      RES_K(10)       /* verified                             */

#define MR_C3       CAP_U(10)       /* verified                             */
#define MR_C4       CAP_U(4.7)      /* verified                             */
#define MR_C5       CAP_N(39)       /* verified                             */
#define MR_C6       CAP_N(3.9)      /* verified                             */
#define MR_C14      CAP_U(4.7)      /* verified                             */
#define MR_C15      CAP_U(4.7)      /* verified                             */
#define MR_C16      CAP_N(6.8)      /* verified                             */
#define MR_C17      CAP_N(22)       /* verified                             */
#define MR_C30      CAP_P(100)      /* verified                             */
#define MR_C31      CAP_U(0.022)    /* verified                             */
#define MR_C32      CAP_U(1)        /* verified                             */
#define MR_C39      CAP_N(4.7)      /* verified                             */
#define MR_C40      CAP_N(22)       /* verified                             */
#define MR_C41      CAP_U(4.7)      /* verified                             */
#define MR_C43      CAP_U(3.3)      /* verified                             */
#define MR_C44      CAP_U(3.3)      /* verified                             */

#define MR_MIXER_RPAR  RES_4_PARALLEL(MR_R20, MR_R19, MR_R41, MR_R40)


/* KT = 0.25 for diode circuit, 0.33 else */

#define DISCRETE_LS123(_N, _T, _R, _C) \
	DISCRETE_ONESHOTR(_N, 0, _T, 1, (0.25 * (_R) * (_C) * (1.0+700./(_R))), DISC_ONESHOT_RETRIG | DISC_ONESHOT_REDGE)
#define DISCRETE_LS123_INV(_N, _T, _R, _C) \
	DISCRETE_ONESHOTR(_N, 0, _T, 1, (0.25 * (_R) * (_C) * (1.0+700./(_R))), DISC_ONESHOT_RETRIG | DISC_ONESHOT_REDGE | DISC_OUT_ACTIVE_LOW)


	static const discrete_op_amp_info mario_dac_amp =
{
	DISC_OP_AMP_IS_NORTON,
	MR_R34, MR_R36, 0, MR_R35, 0,       /* r1, r2, r3, r4, c */
	0, 5                            /* vN, vP */
};

	static const discrete_mixer_desc mario_mixer =
{
	DISC_MIXER_IS_RESISTOR,
	{MR_R20, MR_R19, MR_R41, MR_R40},
	{0}, {0}, 0, 0, MR_C31, MR_C32, 0, 1    /* r_node{}, c{}, rI, rF, cF, cAmp, vRef, gain*/
};

#define LS629_FREQ_R_IN     RES_K(90)

static DISCRETE_SOUND_START(mario)

	/************************************************
	 * Input register mapping for mario
	 ************************************************/

	/* DISCRETE_INPUT_DATA */
	DISCRETE_INPUT_NOT(DS_SOUND7_INV)               /* IC 7L, pin 8 */

	/************************************************
	 * SOUND0
	 ************************************************/

	DISCRETE_TASK_START(1)
	DISCRETE_INPUT_PULSE(DS_SOUND0_INV, 1)          /* IC 4C, pin 15 */
	DISCRETE_LS123(NODE_10,                         /* IC 2H, pin 13 */
		DS_SOUND0_INV,                              /* IC 2H, pin 2 */
		MR_R17, MR_C14)

/* Breadboarded measurements IC 1J, pin 10
   D.R. Oct 2010
    V       Hz
    0.115   14470
    0.250   15190
    0.500   14980
    0.750   18150
    1.000   21690
    2.000   38790
    3.000   58580
    4.000   79890
*/

/* Breadboarded measurements IC 2J, pin 10
   D.R. Oct 2010
    V       Hz
    0.116   2458
    0.250   2593
    0.500   2540
    0.750   3081
    1.000   3676
    2.000   6590
    3.000   9974
    4.000   13620
*/

	/* covert logic to measured voltage */
	DISCRETE_XTIME_BUFFER(NODE_11,                  /* IC 1H, pin 10 */
		NODE_10,                                    /* IC 1H, pin 11 */
		0.115, 4.0)                                 /* measured Low/High */
	/* work out cap charge of RC in parallel with 2 74LS629s */
	DISCRETE_RCFILTER(NODE_12, NODE_11, RES_3_PARALLEL(MR_R6, LS629_FREQ_R_IN, LS629_FREQ_R_IN), MR_C3 )
	/* work out voltage drop of RC in parallel with 2 74LS629s */
	DISCRETE_GAIN(NODE_13, NODE_12, RES_VOLTAGE_DIVIDER(MR_R6, RES_2_PARALLEL(LS629_FREQ_R_IN, LS629_FREQ_R_IN)))
	DISCRETE_74LS624(NODE_14,                       /* IC 1J, pin 10 */
		1,                                          /* ENAB */
		NODE_13, 5,                                 /* VMOD -  IC 1J, pin 1; VRNG */
		MR_C6, 0, 0, 0,                             /* C; R_FREQ_IN; C_FREQ_IN; R_RNG_IN */
		DISC_LS624_OUT_LOGIC_X)
	DISCRETE_74LS624(NODE_15,                       /* IC 2J, pin 10 */
		1,                                          /* ENAB */
		NODE_13, 5,                                 /* VMOD - IC 2J, pin 1; VRNG */
		MR_C17, 0, 0, 0,                            /* C; R_FREQ_IN; C_FREQ_IN; R_RNG_IN */
		DISC_LS624_OUT_LOGIC_X)
	DISCRETE_XTIME_XOR(NODE_16,                     /* IC IC 1K, pin 6 */
		NODE_14, NODE_15,                           /* IC 1K, pin 5; pin 4 */
		0, 0)                                       /* use x_time logic */
	DISCRETE_XTIME_AND(DS_OUT_SOUND0,               /* IC 2K, pin 6 */
		NODE_10, NODE_16,                           /* IC 2K, pin 5; pin 4 */
		0.066, 3.8)                                 /* LOW; HIGH (varies due to load 3.7 - 4.4) */
	DISCRETE_TASK_END()

	/************************************************
	 * SOUND1
	 ************************************************/

	DISCRETE_TASK_START(1)
	DISCRETE_INPUT_PULSE(DS_SOUND1_INV, 1)          /* IC 4C, pin 14 */
	DISCRETE_LS123(NODE_20,                         /* IC 2H, pin 5 */
		DS_SOUND1_INV,                              /* IC 2H, pin 10 */
		MR_R18, MR_C15)

/* Breadboarded measurements IC 1J, pin 7
   D.R. Oct 2010
    V       Hz
    0.116   1380
    0.250   1448
    0.500   1419
    0.750   1717
    1.000   2053
    2.000   3677
    3.000   5561
    4.000   7610
*/

/* Breadboarded measurements IC 2J, pin 7
   D.R. Oct 2010
    V       Hz
    0.112   8030
    0.250   8490
    0.500   8326
    0.750   10030
    1.000   12000
    2.000   21460
    3.000   32540
    4.000   44300
*/

	/* covert logic to measured voltage */
	DISCRETE_XTIME_BUFFER(NODE_21,                  /* IC 1H, pin 8 */
		NODE_20,                                    /* IC 1H, pin 9 */
		0.115, 4.0)                                 /* measured Low/High */
	/* work out cap charge of RC in parallel with 2 74LS629s */
	DISCRETE_RCFILTER(NODE_22, NODE_21, RES_3_PARALLEL(MR_R7, LS629_FREQ_R_IN, LS629_FREQ_R_IN), MR_C4 )
	/* work out voltage drop of RC in parallel with 2 74LS629s */
	DISCRETE_GAIN(NODE_23, NODE_22, RES_VOLTAGE_DIVIDER(MR_R7, RES_2_PARALLEL(LS629_FREQ_R_IN, LS629_FREQ_R_IN)))
	DISCRETE_74LS624(NODE_24,                       /* IC 1J, pin 7 */
		1,                                          /* ENAB */
		NODE_23, 5,                                 /* VMOD, VRNG */
		MR_C5, 0, 0, 0,                             /* C; R_FREQ_IN; C_FREQ_IN; R_RNG_IN */
		DISC_LS624_OUT_LOGIC_X)
	DISCRETE_74LS624(NODE_25,                       /* IC 2J, pin 7 */
		1,                                          /* ENAB */
		NODE_23, 5,                                 /* VMOD, VRNG */
		MR_C16, 0, 0, 0,                            /* C; R_FREQ_IN; C_FREQ_IN; R_RNG_IN */
		DISC_LS624_OUT_LOGIC_X)
	DISCRETE_XTIME_XOR(NODE_26,                     /* IC IC 1K, pin 3 */
		NODE_24, NODE_25,                           /* IC 1K, pin 1; pin 2 */
		0, 0)                                       /* use x_time logic */
	DISCRETE_XTIME_AND(DS_OUT_SOUND1,               /* IC 2K, pin 3 */
		NODE_20, NODE_26,                           /* IC 2K, pin 2; pin 1 */
		0.066, 3.8)                                 /* LOW; HIGH (varies due to load 3.7 - 4.4) */
	DISCRETE_TASK_END()

	/************************************************
	 * SOUND7
	 ************************************************/

	DISCRETE_TASK_START(1)
	DISCRETE_COUNTER(NODE_100,                      /* IC 3H */
		1, 0,                                       /* ENAB; RESET */
		NODE_118,                                   /* CLK - IC 3H, pin 10 */
		0, 0x3FFF, DISC_COUNT_UP, 0, DISC_CLK_BY_COUNT | DISC_OUT_HAS_XTIME)
	DISCRETE_BIT_DECODE(NODE_102,                   /* IC 3H, pin 7 */
		NODE_100,  3, 0)                            /* output x_time logic */
	DISCRETE_BIT_DECODE(NODE_104,                   /* IC 3H, pin 1 */
		NODE_100,  11, 0)                           /* output x_time logic */

	DISCRETE_LS123(NODE_110,                        /* IC 4L, pin 13 */
		DS_SOUND7_INV,                              /* IC 4L, pin 2 */
		MR_R61, MR_C41)
	DISCRETE_XTIME_INVERTER(NODE_111,               /* IC 4J, pin 8 */
		NODE_110,                                   /* IC 4J, pin 9 */
		0.151, 4.14)                                /* measured Low/High */

/* Breadboarded measurements IC 4K, pin 10
   D.R. Oct 2010
    V       Hz
    0.151   3139
    0.25    2883
    0.5     2820
    0.75    3336
    1       3805
    2       6498
    3       9796
    4       13440
    4.14    13980
*/

	DISCRETE_74LS624(NODE_113,                      /* IC 4K, pin 10 */
		1,                                          /* ENAB */
		NODE_111, 5,                                /* VMOD - IC 4K, pin 1; VRNG */
		MR_C40, MR_R65, MR_C44, 0,                  /* C; R_FREQ_IN; C_FREQ_IN; R_RNG_IN */
		DISC_LS624_OUT_LOGIC_X)

	DISCRETE_XTIME_XOR(NODE_115,                    /* IC 6N, pin 3 */
		NODE_113, NODE_102,                         /* IC 6N, pin 1; pin 2 */
		0, 0)                                       /* use x_time logic */

/* Breadboarded measurements IC 4K, pin 7
   D.R. Oct 2010
    V       Hz
    0.135   14450
    0.25    13320
    0.5     12980
    0.75    15150
    1       17270
    2       28230
    3       41910
    4       56950
    4.15    59400
*/

	DISCRETE_XTIME_INVERTER(NODE_117,               /* IC 4J, pin 4 */
		NODE_104,                                   /* IC 4J, pin 3 */
		0.135, 4.15)                                /* measured Low/High */
	DISCRETE_74LS624(NODE_118,                      /* IC 4K, pin 7 */
		1,                                          /* ENAB */
		NODE_117, 5,                                /* VMOD - IC 4K, pin 2; VRNG */
		MR_C39, MR_R64, MR_C43, 0,                  /* C; R_FREQ_IN; C_FREQ_IN; R_RNG_IN */
		DISC_LS624_OUT_LOGIC_X)

	DISCRETE_XTIME_AND(DS_OUT_SOUND7,               /* IC 2K, pin 11 */
		NODE_110, NODE_115,                         /* IC 2K, pin 12; pin 13 */
		0.066, 4.07)                                /* LOW; HIGH (varies due to load 4.07 is lowest) */
	DISCRETE_TASK_END()

	/************************************************
	 * DAC
	 ************************************************/

	/* following the resistor DAC are two opamps. The first is a 1:1 amplifier, the second
	 * is a filter circuit. Simulation in LTSPICE shows, that the following is equivalent:
	 */

	DISCRETE_TASK_START(1)
	DISCRETE_INPUT_BUFFER(DS_DAC, 0)
	DISCRETE_MULTIPLY(NODE_170, DS_DAC, TTL_HIGH / 256.0)       /* MXR1 */
	/* this stage reduces the gain of the DAC by 50%, so yes the volume is much lower then the walk sound */
	DISCRETE_OP_AMP(NODE_171,                                   /* IC 3M, pin 5 */
		1,                                                      /* ENAB */
		NODE_170, 5,                                            /* IN0 - IC 3M, pin 6; IN1 - IC 3M, pin 1 */
		&mario_dac_amp)
	/* This provides a close simulation of the IC 3M, pin 10 filter circuit */
	/* The Measured and SPICEd low freq gain is 1, it then has a high frequency
	 * drop close to the following RC filter. */
	DISCRETE_RCFILTER_VREF(DS_OUT_DAC, NODE_171, RES_K(750), CAP_P(180), 2.5)
	DISCRETE_TASK_END()


	/************************************************
	 * MIXER
	 ************************************************/

	DISCRETE_TASK_START(2)
	DISCRETE_MIXER4(NODE_297,
		1,                                                          /* ENAB */
		DS_OUT_SOUND0, DS_OUT_SOUND1, DS_OUT_SOUND7, DS_OUT_DAC,
		&mario_mixer)
	/* approx -0.625V to 0.980V when playing, but turn on sound peaks at 2.38V */
	/* we will set the full wav range to 1.19V which will cause clipping on the turn on
	 * sound.  The real game would do this when the volume is turned up too.
	 * Reducing MAME's master volume to 50% will provide full unclipped volume.
	 */
	DISCRETE_OUTPUT(NODE_297, 32767.0/1.19)
	DISCRETE_TASK_END()

DISCRETE_SOUND_END
#endif
/****************************************************************
 *
 * EA / Banking
 *
 ****************************************************************/

static void set_ea(address_space &space, int ea)
{
	mario_state *state = space.machine().driver_data<mario_state>();
	//printf("ea: %d\n", ea);
	//machine.device("audiocpu")->execute().set_input_line(MCS48_INPUT_EA, (ea) ? ASSERT_LINE : CLEAR_LINE);
	if (state->m_eabank != nullptr)
		state->membank(state->m_eabank)->set_entry(ea);
}

/****************************************************************
 *
 * Initialization
 *
 ****************************************************************/

void mario_state::sound_start()
{
	device_t *audiocpu = machine().device("audiocpu");

#if USE_8039
	uint8_t *SND = memregion("audiocpu")->base();

	SND[0x1001] = 0x01;
#endif

	m_eabank = nullptr;
	if (audiocpu != nullptr && audiocpu->type() != Z80)
	{
		m_eabank = "bank1";
		audiocpu->memory().space(AS_PROGRAM).install_read_bank(0x000, 0x7ff, "bank1");
		membank("bank1")->configure_entry(0, memregion("audiocpu")->base());
		membank("bank1")->configure_entry(1, memregion("audiocpu")->base() + 0x1000);
	}

	save_item(NAME(m_last));
	save_item(NAME(m_portT));
}

void mario_state::sound_reset()
{
	address_space &space = machine().device("audiocpu")->memory().space(AS_PROGRAM);

#if USE_8039
	set_ea(space, 1);
#endif

	/* FIXME: convert to latch8 */
	m_soundlatch->clear_w(space, 0, 0);
	if (m_soundlatch2) m_soundlatch2->clear_w(space, 0, 0);
	if (m_soundlatch3) m_soundlatch3->clear_w(space, 0, 0);
	if (m_soundlatch4) m_soundlatch4->clear_w(space, 0, 0);
	if (m_soundlatch3) I8035_P1_W(space, 0x00); /* Input port */
	if (m_soundlatch4) I8035_P2_W(space, 0xff); /* Port is in high impedance state after reset */

	m_last = 0;
}

/****************************************************************
 *
 * I/O Handlers - static
 *
 ****************************************************************/

READ8_MEMBER(mario_state::mario_sh_p1_r)
{
	return I8035_P1_R(space);
}

READ8_MEMBER(mario_state::mario_sh_p2_r)
{
	return I8035_P2_R(space) & 0xEF; /* Bit 4 connected to GND! */
}

READ_LINE_MEMBER(mario_state::mario_sh_t0_r)
{
	return I8035_T_R(machine().dummy_space(), 0);
}

READ_LINE_MEMBER(mario_state::mario_sh_t1_r)
{
	return I8035_T_R(machine().dummy_space(), 1);
}

READ8_MEMBER(mario_state::mario_sh_tune_r)
{
	uint8_t *SND = memregion("audiocpu")->base();
	uint16_t mask = memregion("audiocpu")->bytes()-1;
	uint8_t p2 = I8035_P2_R(space);

	if ((p2 >> 7) & 1)
		return m_soundlatch->read(space, offset);
	else
		return (SND[(0x1000 + (p2 & 0x0f) * 256 + offset) & mask]);
}

WRITE8_MEMBER(mario_state::mario_sh_sound_w)
{
#if OLD_SOUND
	m_discrete->write(space, DS_DAC, data);
#else
	m_audio_dac->write(data);
#endif
}

WRITE8_MEMBER(mario_state::mario_sh_p1_w)
{
	I8035_P1_W(space, data);
}

WRITE8_MEMBER(mario_state::mario_sh_p2_w)
{
	I8035_P2_W(space, data);
}

/****************************************************************
 *
 * I/O Handlers - global
 *
 ****************************************************************/

WRITE8_MEMBER(mario_state::masao_sh_irqtrigger_w)
{
	if (m_last == 1 && data == 0)
	{
		/* setting bit 0 high then low triggers IRQ on the sound CPU */
		m_audiocpu->set_input_line_and_vector(0, HOLD_LINE, 0xff);
	}

	m_last = data;
}

WRITE8_MEMBER(mario_state::mario_sh_tuneselect_w)
{
	m_soundlatch->write(space, offset, data);
}

/* Sound 0 and 1 are pulsed !*/

/* Mario running sample */
WRITE8_MEMBER(mario_state::mario_sh1_w)
{
#if OLD_SOUND
	m_discrete->write(space, DS_SOUND0_INP, 0);
#else
	m_audio_snd0->write(data);
#endif
}

/* Luigi running sample */
WRITE8_MEMBER(mario_state::mario_sh2_w)
{
#if OLD_SOUND
	m_discrete->write(space, DS_SOUND1_INP, 0);
#else
	m_audio_snd1->write(data);
#endif
}

/* Misc samples */
WRITE8_MEMBER(mario_state::mario_sh3_w)
{
	switch (offset)
	{
		case 0: /* death */
			if (data)
				m_audiocpu->set_input_line(0,ASSERT_LINE);
			else
				m_audiocpu->set_input_line(0,CLEAR_LINE);
			break;
		case 1: /* get coin */
			I8035_T_W_AH(space, 0,data & 1);
			break;
		case 2: /* ice */
			I8035_T_W_AH(space, 1, data & 1);
			break;
		case 3: /* crab */
			I8035_P1_W_AH(space, 0, data & 1);
			break;
		case 4: /* turtle */
			I8035_P1_W_AH(space, 1, data & 1);
			break;
		case 5: /* fly */
			I8035_P1_W_AH(space, 2, data & 1);
			break;
		case 6: /* coin */
			I8035_P1_W_AH(space, 3, data & 1);
			break;
		case 7: /* skid */
#if OLD_SOUND
			machine().device<discrete_device>("discrete")->write(space, DS_SOUND7_INP, data & 1);
#else
			m_audio_snd7->write((data & 1) ^ 1);
#endif
			break;
	}
}

/*************************************
 *
 *  Sound CPU memory handlers
 *
 *************************************/

static ADDRESS_MAP_START( mario_sound_map, AS_PROGRAM, 8, mario_state )
	AM_RANGE(0x0000, 0x07ff) AM_ROMBANK("bank1") AM_REGION("audiocpu", 0)
	AM_RANGE(0x0800, 0x0fff) AM_ROM
ADDRESS_MAP_END

static ADDRESS_MAP_START( mario_sound_io_map, AS_IO, 8, mario_state )
	AM_RANGE(0x00, 0xff) AM_READ(mario_sh_tune_r) AM_WRITE(mario_sh_sound_w)
ADDRESS_MAP_END

static ADDRESS_MAP_START( masao_sound_map, AS_PROGRAM, 8, mario_state )
	AM_RANGE(0x0000, 0x0fff) AM_ROM
	AM_RANGE(0x2000, 0x23ff) AM_RAM
	AM_RANGE(0x4000, 0x4000) AM_DEVREADWRITE("aysnd", ay8910_device, data_r, data_w)
	AM_RANGE(0x6000, 0x6000) AM_DEVWRITE("aysnd", ay8910_device, address_w)
ADDRESS_MAP_END


/*************************************
 *
 *  Machine driver
 *
 *************************************/

MACHINE_CONFIG_START( mario_audio )

#if USE_8039
	MCFG_CPU_ADD("audiocpu", I8039, I8035_CLOCK)         /* 730 kHz */
#else
	MCFG_CPU_ADD("audiocpu", M58715, I8035_CLOCK)        /* 730 kHz */
#endif
	MCFG_CPU_PROGRAM_MAP(mario_sound_map)
	MCFG_CPU_IO_MAP(mario_sound_io_map)
	MCFG_MCS48_PORT_P1_IN_CB(READ8(mario_state, mario_sh_p1_r))
	MCFG_MCS48_PORT_P1_OUT_CB(WRITE8(mario_state, mario_sh_p1_w))
	MCFG_MCS48_PORT_P2_IN_CB(READ8(mario_state, mario_sh_p2_r))
	MCFG_MCS48_PORT_P2_OUT_CB(WRITE8(mario_state, mario_sh_p2_w))
	MCFG_MCS48_PORT_T0_IN_CB(READLINE(mario_state, mario_sh_t0_r))
	MCFG_MCS48_PORT_T1_IN_CB(READLINE(mario_state, mario_sh_t1_r))

	MCFG_SPEAKER_STANDARD_MONO("mono")

	MCFG_GENERIC_LATCH_8_ADD("soundlatch")
	MCFG_GENERIC_LATCH_8_ADD("soundlatch2")
	MCFG_GENERIC_LATCH_8_ADD("soundlatch3")
	MCFG_GENERIC_LATCH_8_ADD("soundlatch4")

#if OLD_SOUND
	MCFG_SOUND_ADD("discrete", DISCRETE, 0)
	MCFG_DISCRETE_INTF(mario)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 1)
#else
	MCFG_SOUND_ADD("snd_nl", NETLIST_SOUND, 48000)
	MCFG_NETLIST_SETUP(mario)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 1.0)

	MCFG_NETLIST_LOGIC_INPUT("snd_nl", "snd0", "SOUND0.IN", 0)
	MCFG_NETLIST_LOGIC_INPUT("snd_nl", "snd1", "SOUND1.IN", 0)
	MCFG_NETLIST_LOGIC_INPUT("snd_nl", "snd7", "SOUND7.IN", 0)
	MCFG_NETLIST_INT_INPUT("snd_nl", "dac", "DAC.VAL", 0, 255)

	MCFG_NETLIST_STREAM_OUTPUT("snd_nl", 0, "ROUT.1")
	MCFG_NETLIST_ANALOG_MULT_OFFSET(150000.0, 0.0)
#endif

MACHINE_CONFIG_END

MACHINE_CONFIG_START( masao_audio )

	MCFG_CPU_ADD("audiocpu", Z80, 24576000/16)  /* ???? */
	MCFG_CPU_PROGRAM_MAP(masao_sound_map)

	MCFG_SPEAKER_STANDARD_MONO("mono")

	MCFG_GENERIC_LATCH_8_ADD("soundlatch")

	MCFG_SOUND_ADD("aysnd", AY8910, 14318000/6)
	MCFG_AY8910_PORT_A_READ_CB(DEVREAD8("soundlatch", generic_latch_8_device, read))
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.50)

MACHINE_CONFIG_END
