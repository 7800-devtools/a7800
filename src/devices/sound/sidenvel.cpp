// license:BSD-3-Clause
// copyright-holders:Peter Trauner
/*========================================================================= */
/* This source implements the ADSR volume envelope of the SID-chip. */
/* Two different envelope shapes are implemented, an exponential */
/* approximation and the linear shape, which can easily be determined */
/* by reading the registers of the third SID operator. */
/* */
/* Accurate volume envelope times as of November 1994 are used */
/* courtesy of George W. Taylor <aa601@cfn.cs.dal.ca>, <yurik@io.org> */
/* They are slightly modified. */
/* */
/* To use the rounded envelope times from the C64 Programmers Reference */
/* Book define SID_REFTIMES at the Makefile level. */
/* */
/* To perform realtime calculations with floating point precision define */
/* SID_FPUENVE at the Makefile level. On high-end FPUs (not Pentium !), */
/* this can result in speed improvement. Default is integer fixpoint. */
/* */
/* Global Makefile definables: */
/* */
/*   DIRECT_FIXPOINT - use a union to access integer fixpoint operands */
/*                     in memory. This makes an assumption about the */
/*                     hardware and software architecture and therefore */
/*                     is considered a hack ! */
/* */
/* Local (or Makefile) definables: */
/* */
/*   SID_REFTIMES - use rounded envelope times */
/*   SID_FPUENVE  - use floating point precision for calculations */
/*                  (will override the global DIRECT_FIXPOINT setting !) */
/* */
/*========================================================================= */

#include "emu.h"
#include "sidenvel.h"

#include "sid.h"
#include "side6581.h"
#include "sidvoice.h"


const uint8_t masterVolumeLevels[16] =
{
	0,  17,  34,  51,  68,  85, 102, 119,
	136, 153, 170, 187, 204, 221, 238, 255
};

static uint16_t masterAmplModTable[16*256];

static const float attackTimes[16] =
{
	/* milliseconds */
#if defined(SID_REFTIMES)
	2.0f,   8.0f,  16.0f,  24.0f,   38.0f,   56.0f,   68.0f,   80.0f,
	100.0f, 250.0f, 500.0f, 800.0f, 1000.0f, 3000.0f, 5000.0f, 8000.0f
#else
	2.2528606f,  8.0099577f, 15.7696042f, 23.7795619f, 37.2963655f, 55.0684591f,
	66.8330845f, 78.3473987f,
	98.1219818f, 244.554021f, 489.108042f, 782.472742f, 977.715461f, 2933.64701f,
	4889.07793f, 7822.72493f
#endif
};

static const float decayReleaseTimes[16] =
{
	/* milliseconds */
#if defined(SID_REFTIMES)
	8.0f,  24.0f,   48.0f,   72.0f,  114.0f,  168.0f,   204.0f,   240.0f,
	300.0f, 750.0f, 1500.0f, 2400.0f, 3000.0f, 9000.0f, 15000.0f, 24000.0f
#else
	8.91777693f,  24.594051f, 48.4185907f, 73.0116639f, 114.512475f, 169.078356f,
	205.199432f, 240.551975f,
	301.266125f, 750.858245f, 1501.71551f, 2402.43682f, 3001.89298f, 9007.21405f,
	15010.998f, 24018.2111f
#endif
};

#ifdef SID_FPUENVE
	static float attackRates[16];
	static float decayReleaseRates[16];
#elif defined(DIRECT_FIXPOINT)
	static uint32_t attackRates[16];
	static uint32_t decayReleaseRates[16];
#else
	static uint32_t attackRates[16];
	static uint32_t attackRatesP[16];
	static uint32_t decayReleaseRates[16];
	static uint32_t decayReleaseRatesP[16];
#endif

static const uint32_t attackTabLen = 255;
static uint32_t releaseTabLen;
static uint32_t releasePos[256];


void enveEmuInit( uint32_t updateFreq, int measuredValues )
{
	uint32_t i, j, k;

	releaseTabLen = sizeof(releaseTab);
	for ( i = 0; i < 256; i++ )
	{
		j = 0;
		while (( j < releaseTabLen ) && (releaseTab[j] > i) )
		{
			j++;
		}
		if ( j < releaseTabLen )
		{
			releasePos[i] = j;
		}
		else
		{
			releasePos[i] = releaseTabLen -1;
		}
	}

	k = 0;
	for ( i = 0; i < 16; i++ )
	{
		for ( j = 0; j < 256; j++ )
		{
			uint16_t tmpVol = j;
			if (measuredValues)
			{
				tmpVol = (uint16_t) ((293.0*(1-exp(j/-130.0)))+4.0);
				if (j == 0)
					tmpVol = 0;
				if (tmpVol > 255)
					tmpVol = 255;
			}
			/* Want the modulated volume value in the high byte. */
			masterAmplModTable[k++] = ((tmpVol * masterVolumeLevels[i]) / 255) << 8;
		}
	}

	for ( i = 0; i < 16; i++ )
	{
#ifdef SID_FPUENVE
		double scaledenvelen = floor(( attackTimes[i] * updateFreq ) / 1000UL );
		if (scaledenvelen == 0)
			scaledenvelen = 1;
		attackRates[i] = attackTabLen / scaledenvelen;

		scaledenvelen = floor(( decayReleaseTimes[i] * updateFreq ) / 1000UL );
		if (scaledenvelen == 0)
			scaledenvelen = 1;
		decayReleaseRates[i] = releaseTabLen / scaledenvelen;
#elif defined(DIRECT_FIXPOINT)
		uint32_t scaledenvelen = (uint32_t)floor(( attackTimes[i] * updateFreq ) / 1000UL );
		if (scaledenvelen == 0)
			scaledenvelen = 1;
		attackRates[i] = (attackTabLen << 16) / scaledenvelen;

		scaledenvelen = (uint32_t)floor(( decayReleaseTimes[i] * updateFreq ) / 1000UL );
		if (scaledenvelen == 0)
			scaledenvelen = 1;
		decayReleaseRates[i] = (releaseTabLen << 16) / scaledenvelen;
#else
		uint32_t scaledenvelen = (uint32_t)(/*floor*/(( attackTimes[i] * updateFreq ) / 1000UL ));

		if (scaledenvelen == 0)
			scaledenvelen = 1;
		attackRates[i] = attackTabLen / scaledenvelen;
		attackRatesP[i] = (( attackTabLen % scaledenvelen ) * 65536UL ) / scaledenvelen;

		scaledenvelen = (uint32_t)(/*floor*/(( decayReleaseTimes[i] * updateFreq ) / 1000UL ));
		if (scaledenvelen == 0)
			scaledenvelen = 1;
		decayReleaseRates[i] = releaseTabLen / scaledenvelen;
		decayReleaseRatesP[i] = (( releaseTabLen % scaledenvelen ) * 65536UL ) / scaledenvelen;
#endif
	}
}

/* Reset op. */

void enveEmuResetOperator(sidOperator* pVoice)
{
	/* mute, end of R-phase */
	pVoice->ADSRctrl = ENVE_MUTE;
//  pVoice->gateOnCtrl = (pVoice->gateOffCtrl = false);

#ifdef SID_FPUENVE
	pVoice->fenveStep = (pVoice->fenveStepAdd = 0);
	pVoice->enveStep = 0;
#elif defined(DIRECT_FIXPOINT)
	pVoice->enveStep.l = (pVoice->enveStepAdd.l = 0);
#else
	pVoice->enveStep = (pVoice->enveStepPnt = 0);
	pVoice->enveStepAdd = (pVoice->enveStepAddPnt = 0);
#endif
	pVoice->enveSusVol = 0;
	pVoice->enveVol = 0;
	pVoice->enveShortAttackCount = 0;
}

static inline uint16_t enveEmuStartAttack(sidOperator*);
static inline uint16_t enveEmuStartDecay(sidOperator*);
static inline uint16_t enveEmuStartRelease(sidOperator*);
static inline uint16_t enveEmuAlterAttack(sidOperator*);
static inline uint16_t enveEmuAlterDecay(sidOperator*);
static inline uint16_t enveEmuAlterSustain(sidOperator*);
static inline uint16_t enveEmuAlterSustainDecay(sidOperator*);
static inline uint16_t enveEmuAlterRelease(sidOperator*);
static inline uint16_t enveEmuAttack(sidOperator*);
static inline uint16_t enveEmuDecay(sidOperator*);
static inline uint16_t enveEmuSustain(sidOperator*);
static inline uint16_t enveEmuSustainDecay(sidOperator*);
static inline uint16_t enveEmuRelease(sidOperator*);
static inline uint16_t enveEmuMute(sidOperator*);

static inline uint16_t enveEmuStartShortAttack(sidOperator*);
static inline uint16_t enveEmuAlterShortAttack(sidOperator*);
static inline uint16_t enveEmuShortAttack(sidOperator*);


const ptr2sidUwordFunc enveModeTable[] =
{
	/* 0 */
	&enveEmuStartAttack, &enveEmuStartRelease,
	&enveEmuAttack, &enveEmuDecay, &enveEmuSustain, &enveEmuRelease,
	&enveEmuSustainDecay, &enveEmuMute,
	/* 16 */
	&enveEmuStartShortAttack,
	&enveEmuMute, &enveEmuMute, &enveEmuMute,
	&enveEmuMute, &enveEmuMute, &enveEmuMute, &enveEmuMute,
	/* 32        */
	&enveEmuStartAttack, &enveEmuStartRelease,
	&enveEmuAlterAttack, &enveEmuAlterDecay, &enveEmuAlterSustain, &enveEmuAlterRelease,
	&enveEmuAlterSustainDecay, &enveEmuMute,
	/* 48        */
	&enveEmuStartShortAttack,
	&enveEmuMute, &enveEmuMute, &enveEmuMute,
	&enveEmuMute, &enveEmuMute, &enveEmuMute, &enveEmuMute
};

/* Real-time functions. */
/* Order is important because of inline optimizations. */
/* */
/* ADSRctrl is (index*2) to enveModeTable[], because of KEY-bit. */

static inline void enveEmuEnveAdvance(sidOperator* pVoice)
{
#ifdef SID_FPUENVE
	pVoice->fenveStep += pVoice->fenveStepAdd;
#elif defined(DIRECT_FIXPOINT)
	pVoice->enveStep.l += pVoice->enveStepAdd.l;
#else
	pVoice->enveStepPnt += pVoice->enveStepAddPnt;
	pVoice->enveStep += pVoice->enveStepAdd + ( pVoice->enveStepPnt > 65535 );
	pVoice->enveStepPnt &= 0xFFFF;
#endif
}

/* */
/* Mute/Idle. */
/* */

/* Only used in the beginning. */
static inline uint16_t enveEmuMute(sidOperator* pVoice)
{
	return 0;
}

/* */
/* Release */
/* */

static inline uint16_t enveEmuRelease(sidOperator* pVoice)
{
#ifdef SID_FPUENVE
	pVoice->enveStep = (uint16_t)pVoice->fenveStep;
#endif
#if defined(DIRECT_FIXPOINT) && !defined(SID_FPUENVE)
	if ( pVoice->enveStep.w[HI] >= releaseTabLen )
#else
	if ( pVoice->enveStep >= releaseTabLen )
#endif
	{
		pVoice->enveVol = releaseTab[releaseTabLen -1];
		return masterAmplModTable[ pVoice->sid->masterVolumeAmplIndex + pVoice->enveVol ];
	}
	else
	{
#if defined(DIRECT_FIXPOINT) && !defined(SID_FPUENVE)
		pVoice->enveVol = releaseTab[pVoice->enveStep.w[HI]];
#else
		pVoice->enveVol = releaseTab[pVoice->enveStep];
#endif
		enveEmuEnveAdvance(pVoice);
		return masterAmplModTable[ pVoice->sid->masterVolumeAmplIndex + pVoice->enveVol ];
	}
}

static inline uint16_t enveEmuAlterRelease(sidOperator* pVoice)
{
	uint8_t release = pVoice->SIDSR & 0x0F;
#ifdef SID_FPUENVE
	pVoice->fenveStepAdd = decayReleaseRates[release];
#elif defined(DIRECT_FIXPOINT)
	pVoice->enveStepAdd.l = decayReleaseRates[release];
#else
	pVoice->enveStepAdd = decayReleaseRates[release];
	pVoice->enveStepAddPnt = decayReleaseRatesP[release];
#endif
	pVoice->ADSRproc = &enveEmuRelease;
	return enveEmuRelease(pVoice);
}

static inline uint16_t enveEmuStartRelease(sidOperator* pVoice)
{
	pVoice->ADSRctrl = ENVE_RELEASE;
#ifdef SID_FPUENVE
	pVoice->fenveStep = releasePos[pVoice->enveVol];
#elif defined(DIRECT_FIXPOINT)
	pVoice->enveStep.w[HI] = releasePos[pVoice->enveVol];
	pVoice->enveStep.w[LO] = 0;
#else
	pVoice->enveStep = releasePos[pVoice->enveVol];
	pVoice->enveStepPnt = 0;
#endif
	return enveEmuAlterRelease(pVoice);
}

/* */
/* Sustain */
/* */

static inline uint16_t enveEmuSustain(sidOperator* pVoice)
{
	return masterAmplModTable[pVoice->sid->masterVolumeAmplIndex+pVoice->enveVol];
}

static inline uint16_t enveEmuSustainDecay(sidOperator* pVoice)
{
#ifdef SID_FPUENVE
	pVoice->enveStep = (uint16_t)pVoice->fenveStep;
#endif
#if defined(DIRECT_FIXPOINT) && !defined(SID_FPUENVE)
	if ( pVoice->enveStep.w[HI] >= releaseTabLen )
#else
	if ( pVoice->enveStep >= releaseTabLen )
#endif
	{
		pVoice->enveVol = releaseTab[releaseTabLen-1];
		return enveEmuAlterSustain(pVoice);
	}
	else
	{
#if defined(DIRECT_FIXPOINT) && !defined(SID_FPUENVE)
		pVoice->enveVol = releaseTab[pVoice->enveStep.w[HI]];
#else
		pVoice->enveVol = releaseTab[pVoice->enveStep];
#endif
		/* Will be controlled from sidEmuSet2(). */
		if ( pVoice->enveVol <= pVoice->enveSusVol )
		{
			pVoice->enveVol = pVoice->enveSusVol;
			return enveEmuAlterSustain(pVoice);
		}
		else
		{
			enveEmuEnveAdvance(pVoice);
			return masterAmplModTable[ pVoice->sid->masterVolumeAmplIndex + pVoice->enveVol ];
		}
	}
}

/* This is the same as enveEmuStartSustainDecay(). */
static inline uint16_t enveEmuAlterSustainDecay(sidOperator* pVoice)
{
	uint8_t decay = pVoice->SIDAD & 0x0F ;
#ifdef SID_FPUENVE
	pVoice->fenveStepAdd = decayReleaseRates[decay];
#elif defined(DIRECT_FIXPOINT)
	pVoice->enveStepAdd.l = decayReleaseRates[decay];
#else
	pVoice->enveStepAdd = decayReleaseRates[decay];
	pVoice->enveStepAddPnt = decayReleaseRatesP[decay];
#endif
	pVoice->ADSRproc = &enveEmuSustainDecay;
	return enveEmuSustainDecay(pVoice);
}

/* This is the same as enveEmuStartSustain(). */
static inline uint16_t enveEmuAlterSustain(sidOperator* pVoice)
{
	if ( pVoice->enveVol > pVoice->enveSusVol )
	{
		pVoice->ADSRctrl = ENVE_SUSTAINDECAY;
		pVoice->ADSRproc = &enveEmuSustainDecay;
		return enveEmuAlterSustainDecay(pVoice);
	}
	else
	{
		pVoice->ADSRctrl = ENVE_SUSTAIN;
		pVoice->ADSRproc = &enveEmuSustain;
		return enveEmuSustain(pVoice);
	}
}

/* */
/* Decay */
/* */

static inline uint16_t enveEmuDecay(sidOperator* pVoice)
{
#ifdef SID_FPUENVE
	pVoice->enveStep = (uint16_t)pVoice->fenveStep;
#endif
#if defined(DIRECT_FIXPOINT) && !defined(SID_FPUENVE)
	if ( pVoice->enveStep.w[HI] >= releaseTabLen )
#else
	if ( pVoice->enveStep >= releaseTabLen )
#endif
	{
		pVoice->enveVol = pVoice->enveSusVol;
		return enveEmuAlterSustain(pVoice);  /* start sustain */
	}
	else
	{
#if defined(DIRECT_FIXPOINT) && !defined(SID_FPUENVE)
		pVoice->enveVol = releaseTab[pVoice->enveStep.w[HI]];
#else
		pVoice->enveVol = releaseTab[pVoice->enveStep];
#endif
		/* Will be controlled from sidEmuSet2(). */
		if ( pVoice->enveVol <= pVoice->enveSusVol )
		{
			pVoice->enveVol = pVoice->enveSusVol;
			return enveEmuAlterSustain(pVoice);  /* start sustain */
		}
		else
		{
			enveEmuEnveAdvance(pVoice);
			return masterAmplModTable[ pVoice->sid->masterVolumeAmplIndex + pVoice->enveVol ];
		}
	}
}

static inline uint16_t enveEmuAlterDecay(sidOperator* pVoice)
{
	uint8_t decay = pVoice->SIDAD & 0x0F ;
#ifdef SID_FPUENVE
	pVoice->fenveStepAdd = decayReleaseRates[decay];
#elif defined(DIRECT_FIXPOINT)
	pVoice->enveStepAdd.l = decayReleaseRates[decay];
#else
	pVoice->enveStepAdd = decayReleaseRates[decay];
	pVoice->enveStepAddPnt = decayReleaseRatesP[decay];
#endif
	pVoice->ADSRproc = &enveEmuDecay;
	return enveEmuDecay(pVoice);
}

static inline uint16_t enveEmuStartDecay(sidOperator* pVoice)
{
	pVoice->ADSRctrl = ENVE_DECAY;
#ifdef SID_FPUENVE
	pVoice->fenveStep = 0;
#elif defined(DIRECT_FIXPOINT)
	pVoice->enveStep.l = 0;
#else
	pVoice->enveStep = (pVoice->enveStepPnt = 0);
#endif
	return enveEmuAlterDecay(pVoice);
}

/* */
/* Attack */
/* */

static inline uint16_t enveEmuAttack(sidOperator* pVoice)
{
#ifdef SID_FPUENVE
	pVoice->enveStep = (uint16_t)pVoice->fenveStep;
#endif
#if defined(DIRECT_FIXPOINT) && !defined(SID_FPUENVE)
	if ( pVoice->enveStep.w[HI] > attackTabLen )
#else
	if ( pVoice->enveStep >= attackTabLen )
#endif
		return enveEmuStartDecay(pVoice);
	else
	{
#if defined(DIRECT_FIXPOINT) && !defined(SID_FPUENVE)
		pVoice->enveVol = pVoice->enveStep.w[HI];
#else
		pVoice->enveVol = pVoice->enveStep;
#endif
		enveEmuEnveAdvance(pVoice);
		return masterAmplModTable[ pVoice->sid->masterVolumeAmplIndex + pVoice->enveVol ];
	}
}

static inline uint16_t enveEmuAlterAttack(sidOperator* pVoice)
{
	uint8_t attack = pVoice->SIDAD >> 4;
#ifdef SID_FPUENVE
	pVoice->fenveStepAdd = attackRates[attack];
#elif defined(DIRECT_FIXPOINT)
	pVoice->enveStepAdd.l = attackRates[attack];
#else
	pVoice->enveStepAdd = attackRates[attack];
	pVoice->enveStepAddPnt = attackRatesP[attack];
#endif
	pVoice->ADSRproc = &enveEmuAttack;
	return enveEmuAttack(pVoice);
}

static inline uint16_t enveEmuStartAttack(sidOperator* pVoice)
{
	pVoice->ADSRctrl = ENVE_ATTACK;
#ifdef SID_FPUENVE
	pVoice->fenveStep = (float)pVoice->enveVol;
#elif defined(DIRECT_FIXPOINT)
	pVoice->enveStep.w[HI] = pVoice->enveVol;
	pVoice->enveStep.w[LO] = 0;
#else
	pVoice->enveStep = pVoice->enveVol;
	pVoice->enveStepPnt = 0;
#endif
	return enveEmuAlterAttack(pVoice);
}

/* */
/* Experimental. */
/* */

/*#include <iostream.h> */
/*#include <iomanip.h> */

static inline uint16_t enveEmuShortAttack(sidOperator* pVoice)
{
#ifdef SID_FPUENVE
	pVoice->enveStep = (uint16_t)pVoice->fenveStep;
#endif
#if defined(DIRECT_FIXPOINT) && !defined(SID_FPUENVE)
	if ((pVoice->enveStep.w[HI] > attackTabLen) ||
		(pVoice->enveShortAttackCount == 0))
#else
	if ((pVoice->enveStep >= attackTabLen) ||
		(pVoice->enveShortAttackCount == 0))
#endif
/*      return enveEmuStartRelease(pVoice); */
		return enveEmuStartDecay(pVoice);
#if defined(DIRECT_FIXPOINT) && !defined(SID_FPUENVE)
	pVoice->enveVol = pVoice->enveStep.w[HI];
#else
	pVoice->enveVol = pVoice->enveStep;
#endif
	pVoice->enveShortAttackCount--;
/*  cout << hex << pVoice->enveShortAttackCount << " / " << pVoice->enveVol << endl; */
	enveEmuEnveAdvance(pVoice);
	return masterAmplModTable[ pVoice->sid->masterVolumeAmplIndex + pVoice->enveVol ];
}

static inline uint16_t enveEmuAlterShortAttack(sidOperator* pVoice)
{
	uint8_t attack = pVoice->SIDAD >> 4;
#ifdef SID_FPUENVE
	pVoice->fenveStepAdd = attackRates[attack];
#elif defined(DIRECT_FIXPOINT)
	pVoice->enveStepAdd.l = attackRates[attack];
#else
	pVoice->enveStepAdd = attackRates[attack];
	pVoice->enveStepAddPnt = attackRatesP[attack];
#endif
	pVoice->ADSRproc = &enveEmuShortAttack;
	return enveEmuShortAttack(pVoice);
}

static inline uint16_t enveEmuStartShortAttack(sidOperator* pVoice)
{
	pVoice->ADSRctrl = ENVE_SHORTATTACK;
#ifdef SID_FPUENVE
	pVoice->fenveStep = (float)pVoice->enveVol;
#elif defined(DIRECT_FIXPOINT)
	pVoice->enveStep.w[HI] = pVoice->enveVol;
	pVoice->enveStep.w[LO] = 0;
#else
	pVoice->enveStep = pVoice->enveVol;
	pVoice->enveStepPnt = 0;
#endif
	pVoice->enveShortAttackCount = 65535;  /* unused */
	return enveEmuAlterShortAttack(pVoice);
}
