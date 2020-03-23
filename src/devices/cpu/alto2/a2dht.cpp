// license:BSD-3-Clause
// copyright-holders:Juergen Buchmueller
/*****************************************************************************
 *
 *   Xerox AltoII display horizontal task
 *
 *****************************************************************************/
#include "emu.h"
#include "alto2cpu.h"

/*
 * Copied from ALTOCODE24.MU
 *  ;Display Horizontal Task.
 *  ;11 cycles if no block change, 17 if new control block.
 *
 *  DHT:    MAR← CBA-1;
 *      L← SLC -1, BUS=0;
 *      SLC← L, :DHT0;
 *
 *  DHT0:   T← 37400;       MORE TO DO IN THIS BLOCK
 *      SINK← MD;
 *      L← T← MD AND T, SETMODE;
 *      HTAB← L LCY 8, :NORMODE;
 *
 *  NORMODE:L← T← 377 . T;
 *      AECL← L, :REST;
 *
 *  HALFMODE: L← T←  377 . T;
 *      AECL← L, :REST, T← 0;
 *
 *  REST:   L← DWA + T,TASK;    INCREMENT DWA BY 0 OR NWRDS
 *  NDNX:   DWA← L, :DHT;
 *
 *  DHT1:   L← T← MD+1, BUS=0;
 *      CBA← L, MAR← T, :MOREB;
 *
 *  NOMORE: BLOCK, :DNX;
 *  MOREB:  T← 37400;
 *      L← T← MD AND T, SETMODE;
 *      MAR← CBA+1, :NORMX, EVENFIELD;
 *
 *  NORMX:  HTAB← L LCY 8, :NODD;
 *  HALFX:  HTAB← L LCY 8, :NEVEN;
 *
 *  NODD:   L←T← 377 . T;
 *      AECL← L, :XREST;    ODD FIELD, FULL RESOLUTION
 *
 *  NEVEN:  L← 377 AND T;       EVEN FIELD OR HALF RESOLUTION
 *      AECL←L, T←0;
 *
 *  XREST:  L← MD+T;
 *      T←MD-1;
 *  DNX:    DWA←L, L←T, TASK;
 *      SLC←L, :DHT;
 */

/**
 * @brief f1_dht_block early: disable the display word task
 */
void alto2_cpu_device::f1_early_dht_block()
{
	m_dsp.dht_blocks = true;
	// clear the wakeup for the display horizontal task
	m_task_wakeup &= ~(1 << m_task);
	LOG((this,LOG_DHT,2,"    BLOCK %s\n", task_name(m_task)));
}

/**
 * @brief f2_dht_setmode late: set the next scanline's mode inverse and half clock and branch
 *
 * BUS[0] selects the pixel clock (0), or half pixel clock (1)
 * BUS[1] selects normal mode (0), or inverse mode (1)
 *
 * The current BUS[0] drives the NEXT[09] line, i.e. branches to 0 or 1
 */
void alto2_cpu_device::f2_late_dht_setmode()
{
	uint16_t r = X_RDBITS(m_bus,16,0,0);
	m_dsp.setmode = m_bus;
	LOG((this,LOG_DHT,2,"    SETMODE<- BUS (%#o), branch on BUS[0] (%#o | %#o)\n", m_bus, m_next2, r));
	m_next2 |= r;
}

/**
 * @brief called by the CPU when the display horizontal task becomes active
 */
void alto2_cpu_device::activate_dht()
{
	m_task_wakeup &= ~(1 << m_task);
}

/**
 * @brief initialize the display horizontal task
 *
 * @param task task number
 */
void alto2_cpu_device::init_dht(int task)
{
}

void alto2_cpu_device::exit_dht()
{
	// nothing to do yet
}

void alto2_cpu_device::reset_dht()
{
	m_dsp.dht_blocks = true;
	m_dsp.setmode = 0;
}
