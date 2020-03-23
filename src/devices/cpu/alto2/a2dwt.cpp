// license:BSD-3-Clause
// copyright-holders:Juergen Buchmueller
/*****************************************************************************
 *
 *   Xerox AltoII display word task
 *
 *****************************************************************************/
#include "emu.h"
#include "alto2cpu.h"

/*
 * Copied from ALTOCODE24.MU
 *
 *  ;Display Word Task
 *
 *  DWT:    T← DWA;
 *      T←-3+T+1;
 *      L← AECL+T,BUS=0,TASK;   AECL CONTAINS NWRDS AT THIS TIME
 *      AECL←L, :DWTZ;
 *
 *  DWTY:   BLOCK;
 *      TASK, :DWTF;
 *
 *  DWTZ:   L←HTAB-1, BUS=0,TASK;
 *      HTAB←L, :DOTAB;
 *
 *  DOTAB:  DDR←0, :DWTZ;
 *  NOTAB:  MAR←T←DWA;
 *      L←AECL-T-1;
 *      ALUCY, L←2+T;
 *      DWA←L, :XNOMORE;
 *
 *  DOMORE: DDR←MD, TASK;
 *      DDR←MD, :NOTAB;
 *
 *  XNOMORE:DDR← MD, BLOCK;
 *      DDR← MD, TASK;
 *
 *  DWTF:   :DWT;
 */

//! PROM a38 bit O1 is STOPWAKE' (stop DWT if bit is zero)
#define FIFO_STOPWAKE(a38) (0 == (a38 & disp_a38_STOPWAKE) ? true : false)

/**
 * @brief Block the display word task.
 */
void alto2_cpu_device::f1_early_dwt_block()
{
	m_dsp.dwt_blocks = true;

	// Clear the wakeup for the display word task
	m_task_wakeup &= ~(1 << m_task);
	LOG((this,LOG_DWT,2,"    BLOCK %s\n", task_name(m_task)));

	// Wakeup the display horizontal task, if it didn't block itself
	if (!m_dsp.dht_blocks)
		m_task_wakeup |= 1 << task_dht;
}

/**
 * @brief Load the display data register
 * Pushes the word on the bus to the display FIFO.
 * If the FIFO becomes full, the wakeup flag for task_dwt is reset.
 */
void alto2_cpu_device::f2_late_load_ddr()
{
	LOG((this,LOG_DWT,2,"    DDR<- BUS (%#o)\n", m_bus));
	m_dsp.fifo[m_dsp.wa] = m_bus;
	m_dsp.wa = (m_dsp.wa + 1) % A2_DISP_FIFO;
	uint8_t a38 = m_disp_a38[m_dsp.ra * 16 + m_dsp.wa];
	if (FIFO_STOPWAKE(a38))
		m_task_wakeup &= ~(1 << task_dwt);
	LOG((this,LOG_DWT,2, "   DWT push %04x into FIFO[%02o]%s\n",
		m_bus, (m_dsp.wa - 1) & (A2_DISP_FIFO - 1),
		FIFO_STOPWAKE(a38) ? " STOPWAKE" : ""));
}

void alto2_cpu_device::init_dwt(int task)
{
}

void alto2_cpu_device::exit_dwt()
{
	// nothing to do yet
}

void alto2_cpu_device::reset_dwt()
{
	m_dsp.dwt_blocks = false;
	memset(m_dsp.fifo, 0, sizeof(m_dsp.fifo));
	m_dsp.wa = 0;
	m_dsp.ra = 0;
}
