// license:BSD-3-Clause
// copyright-holders: Joakim Larsson Edstrom
/***************************************************************************

    Z80-SCC Serial Communications Controller emulation

    The z80scc is an updated version of the z80sio, with additional support for CRC
    checks and a number of data link layer protocols such as HDLC, SDLC and BiSync.
    (See https://en.wikipedia.org/wiki/Zilog_SCC). The variants in the SCC
    family are as follows:
           Zbus    Universal bus
    NMOS   Z8030   Z8530
    CMOS   Z80C30  Z85C30
    ESCC   Z80230  Z85230, Z8523L (L = low voltage)
    EMSCC          Z85233
    The difference between Zbus and Universal bus is mainly at hardware
    design level and suitable for Intel oriented (Zbus) or Motorola oriented
    chip designs.

TODO/
DONE (x) (p=partly)         NMOS         CMOS       ESCC      EMSCC
-----------------------------------------------------------------------
    Channels                2 FD         2 FD       2 FD      2 FD         FD = Full Duplex
    Synch data rates        2Mbps        4Mbps      5Mbps     5Mbps
                            1Mbps (FM)
                           .5Mbps (NRZI)
 ----- asynchrounous features -------------------------------------------
  p 5-8 bit per char         Y             Y          Y         Y
  p 1,1.5,2 stop bits        Y             Y          Y         Y
  x odd/even parity          Y             Y          Y         Y
  x x1,x16,x32,x64           Y             Y          Y         Y
  p break det/gen            Y             Y          Y         Y
  x parity, framing &        Y             Y          Y         Y
    overrun error det
    -- byte oriented synchrounous features -------------------------------
    Int/ext char sync        Y             Y          Y         Y
    1/2 synch chars          Y             Y          Y         Y
    Aut CRC gen/det          Y             Y          Y         Y
    -- SDLC/HDLC capabilities --------------------------------------------
    Abort seq gen/chk        Y             Y          Y         Y
    Aut zero ins/det         Y             Y          Y         Y
    Aut flag insert          Y             Y          Y         Y
    Addr field rec           Y             Y          Y         Y
    I-fld resid hand         Y             Y          Y         Y
    CRC gen/det              Y             Y          Y         Y
    SDLC loop w EOP          Y             Y          Y         Y
    --
 p  Receiver FIFO            3             3          8         8
 p  Transmitter FIFO         1             1          4         4
    NRZ, NRZI or             Y             Y          Y         Y
     FM enc/dec              Y             Y          Y         Y
    Manchester dec           Y             Y          Y         Y
 x  Baud gen per chan        Y             Y          Y         Y
    DPLL clock recov         Y             Y          Y         Y
    -- Additional features CMOS versions -----------------------------------
 x  Status FIFO              N             Y          Y         Y
 x  SWI ack feat             N             Y          Y         Y
    higher bps w ext DPLL    N           32Mbps     32Mbps    32Mbps
    -- Additional features 85C30 -------------------------------------------
 x  New WR7 feat             N           85C30        Y         Y
    Improved SDLC            N           85C30        Y         Y
    Improved reg handl       N           85C30        Y         Y
    Improved auto feat       N           85C30        Y         Y
    -- Additional features ESCC   -------------------------------------------
    Progrmbl FIFO int &
     DMA req lev             Y             Y          Y         Y
    Improved SDLC            Y             Y          Y         Y
    DPLL counter as TXc      Y             Y          Y         Y
    Improved reg handl       Y             Y          Y         Y
    -------------------------------------------------------------------------
   x/p = Features that has been implemented  n/a = features that will not
***************************************************************************/

#include "emu.h"
#include "z80scc.h"


//**************************************************************************
//  CONFIGURABLE LOGGING
//**************************************************************************

#define LOG_GENERAL (1U <<  0)
#define LOG_SETUP   (1U <<  1)
#define LOG_PRINTF  (1U <<  2)
#define LOG_READ    (1U <<  3)
#define LOG_INT     (1U <<  4)
#define LOG_CMD     (1U <<  5)
#define LOG_TX      (1U <<  6)
#define LOG_RCV     (1U <<  7)
#define LOG_CTS     (1U <<  8)
#define LOG_DCD     (1U <<  9)
#define LOG_SYNC    (1U << 10)

//#define VERBOSE (LOG_CMD|LOG_INT|LOG_SETUP|LOG_TX|LOG_RCV|LOG_READ|LOG_CTS|LOG_DCD)
//#define LOG_OUTPUT_FUNC printf
#include "logmacro.h"

#define LOGSETUP(...) LOGMASKED(LOG_SETUP,   __VA_ARGS__)
#define LOGR(...)     LOGMASKED(LOG_READ,    __VA_ARGS__)
#define LOGINT(...)   LOGMASKED(LOG_INT,     __VA_ARGS__)
#define LOGCMD(...)   LOGMASKED(LOG_CMD,     __VA_ARGS__)
#define LOGTX(...)    LOGMASKED(LOG_TX,      __VA_ARGS__)
#define LOGRCV(...)   LOGMASKED(LOG_RCV,     __VA_ARGS__)
#define LOGCTS(...)   LOGMASKED(LOG_CTS,     __VA_ARGS__)
#define LOGDCD(...)   LOGMASKED(LOG_DCD,     __VA_ARGS__)
#define LOGSYNC(...)  LOGMASKED(LOG_SYNC,    __VA_ARGS__)


//**************************************************************************
//  MACROS / CONSTANTS
//**************************************************************************

#ifdef _MSC_VER
#define FUNCNAME __func__
#define LLFORMAT "%I64d"
#else
#define FUNCNAME __PRETTY_FUNCTION__
#define LLFORMAT "%lld"
#endif

/* LOCAL _BRG is set in z80scc.h, local timer based BRG is not complete and will be removed if not needed for synchrounous mode */
#if Z80SCC_USE_LOCAL_BRG
#define START_BIT_HUNT 1
#define START_BIT_ADJUST 1
#else
#define START_BIT_HUNT 0
#define START_BIT_ADJUST 0
#endif

#define CHANA_TAG   "cha"
#define CHANB_TAG   "chb"

//**************************************************************************
//  DEVICE DEFINITIONS
//**************************************************************************
// device type definition
DEFINE_DEVICE_TYPE(Z80SCC,         z80scc_device,   "z80scc",         "Z80 SCC")
DEFINE_DEVICE_TYPE(Z80SCC_CHANNEL, z80scc_channel,  "z80scc_channel", "Z80 SCC Channel")
DEFINE_DEVICE_TYPE(SCC8030,        scc8030_device,  "scc8030",        "Zilog Z8030 SCC")
DEFINE_DEVICE_TYPE(SCC80C30,       scc80c30_device, "scc80c30",       "Zilog Z80C30 SCC")
DEFINE_DEVICE_TYPE(SCC80230,       scc80230_device, "scc80230",       "Zilog Z80230 ESCC")
DEFINE_DEVICE_TYPE(SCC8530N,       scc8530_device,  "scc8530",        "Zilog Z8530 SCC")  // remove trailing N when 8530scc.c is fully replaced and removed
DEFINE_DEVICE_TYPE(SCC85C30,       scc85c30_device, "scc85c30",       "Zilog Z85C30 SCC")
DEFINE_DEVICE_TYPE(SCC85230,       scc85230_device, "scc85230",       "Zilog Z85230 ESCC")
DEFINE_DEVICE_TYPE(SCC85233,       scc85233_device, "scc85233",       "Zilog Z85233 EMSCC")
DEFINE_DEVICE_TYPE(SCC8523L,       scc8523l_device, "scc8523l",       "Zilog Z8523L SCC")

//-------------------------------------------------
//  device_add_mconfig - add device configuration
//-------------------------------------------------
MACHINE_CONFIG_MEMBER( z80scc_device::device_add_mconfig )
	MCFG_DEVICE_ADD(CHANA_TAG, Z80SCC_CHANNEL, 0)
	MCFG_DEVICE_ADD(CHANB_TAG, Z80SCC_CHANNEL, 0)
MACHINE_CONFIG_END


//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  z80scc_device - constructor
//-------------------------------------------------

z80scc_device::z80scc_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock, uint32_t variant)
	: device_t(mconfig, type, tag, owner, clock),
	device_z80daisy_interface(mconfig, *this),
	m_chanA(*this, CHANA_TAG),
	m_chanB(*this, CHANB_TAG),
	m_rxca(0),
	m_txca(0),
	m_rxcb(0),
	m_txcb(0),
	m_out_txda_cb(*this),
	m_out_dtra_cb(*this),
	m_out_rtsa_cb(*this),
	m_out_wreqa_cb(*this),
	m_out_synca_cb(*this),
	m_out_txdb_cb(*this),
	m_out_dtrb_cb(*this),
	m_out_rtsb_cb(*this),
	m_out_wreqb_cb(*this),
	m_out_syncb_cb(*this),
	m_out_int_cb(*this),
	m_out_rxdrqa_cb(*this),
	m_out_txdrqa_cb(*this),
	m_out_rxdrqb_cb(*this),
	m_out_txdrqb_cb(*this),
	m_variant(variant),
	m_wr0_ptrbits(0)
{
	for (auto & elem : m_int_state)
		elem = 0;
}

z80scc_device::z80scc_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: z80scc_device(mconfig, Z80SCC, tag, owner, clock, TYPE_Z80SCC)
{
}

scc8030_device::scc8030_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: z80scc_device(mconfig, SCC8030, tag, owner, clock, TYPE_SCC8030)
{
}

scc80c30_device::scc80c30_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: z80scc_device(mconfig, SCC80C30, tag, owner, clock, TYPE_SCC80C30)
{
}

scc80230_device::scc80230_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: z80scc_device(mconfig, SCC80230, tag, owner, clock, TYPE_SCC80230)
{
}

scc8530_device::scc8530_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: z80scc_device(mconfig, SCC8530N, tag, owner, clock, TYPE_SCC8530)
{
}

scc85c30_device::scc85c30_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: z80scc_device(mconfig, SCC85C30, tag, owner, clock, TYPE_SCC85C30)
{
}

scc85230_device::scc85230_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: z80scc_device(mconfig, SCC85230, tag, owner, clock, TYPE_SCC85230)
{
}

scc85233_device::scc85233_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: z80scc_device(mconfig, SCC85233, tag, owner, clock, TYPE_SCC85233)
{
}

scc8523l_device::scc8523l_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: z80scc_device(mconfig, SCC8523L, tag, owner, clock, TYPE_SCC8523L)
{
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void z80scc_device::device_start()
{
	LOGSETUP("%s\n", FUNCNAME);
	// resolve callbacks
	m_out_txda_cb.resolve_safe();
	m_out_dtra_cb.resolve_safe();
	m_out_rtsa_cb.resolve_safe();
	m_out_wreqa_cb.resolve_safe();
	m_out_synca_cb.resolve_safe();
	m_out_txdb_cb.resolve_safe();
	m_out_dtrb_cb.resolve_safe();
	m_out_rtsb_cb.resolve_safe();
	m_out_wreqb_cb.resolve_safe();
	m_out_syncb_cb.resolve_safe();
	m_out_int_cb.resolve_safe();
	m_out_rxdrqa_cb.resolve_safe();
	m_out_txdrqa_cb.resolve_safe();
	m_out_rxdrqb_cb.resolve_safe();
	m_out_txdrqb_cb.resolve_safe();

	// state saving
	save_item(NAME(m_int_state));
	save_item(NAME(m_int_source));
	save_item(NAME(m_wr9));
	save_item(NAME(m_wr0_ptrbits));
	LOG(" - SCC variant %02x\n", m_variant);
}


//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void z80scc_device::device_reset()
{
	LOGSETUP("%s %s \n",tag(), FUNCNAME);

	// Do channel reset on both channels
	m_chanA->reset();
	m_chanB->reset();

	// Fix hardware reset values for registers where it differs from channel reset values
	m_wr9  &= 0x3c;
	m_wr9  |= 0xc0;
	m_chanA->m_wr10  = 0x00;
	m_chanB->m_wr10  = 0x00;
	m_chanA->m_wr11  = 0x08;
	m_chanB->m_wr11  = 0x08;
	m_chanA->m_wr14 &= 0xf0;
	m_chanA->m_wr14 |= 0x30;
	m_chanB->m_wr14 &= 0xf0;
	m_chanB->m_wr14 |= 0x30;
}

/*
 * Interrupts
Each of the SCC's two channels contain three priority levels of interrupts, making a total of six interrupt
levels. These three sources of interrupts are: 1) Receiver, 2) Transmitter, and 3) External/Status
conditions. In addition, there are four sources per channel: 0) Transmitt 1) External 2) Receiver 3) Special
which affects the way the interrupt vector is formed. The sources in more detail

INT_RECEIVE:  The sources of receive interrupts consist of Receive Character Available and Special Receive Condition.
              The Special Receive Condition can be subdivided into Receive Overrun, Framing Error (Asynchronous) or
              End of Frame (SDLC). In addition, a parity error can be a special receive condition by programming

INT_EXTERNAL: The External/status interrupts have several sources which may be individually enabled in WR15.
              The sources are zero count, /DCD, Sync/Hunt, /CTS, transmitter under-run/EOM and Break/Abort.

INT_TRANSMIT: The NMOS/CMOS version of the SCC only has a one byte deep transmit buffer. The status of the
              transmit buffer can be determined through TBE bit in RR0, bit D2, which shows whether the
              transmit buffer is empty or not. After a hardware reset (including a hardware reset by software), or
              a channel reset, this bit is set to 1.
              While transmit interrupts are enabled, the NMOS/CMOS version sets the Transmit Interrupt Pending
              (TxIP) bit whenever the transmit buffer becomes empty. This means that the transmit buffer
              must be full before the TxIP can be set. Thus, when transmit interrupts are first enabled, the TxIP
              will not be set until after the first character is written to the NMOS/CMOS.

INT_SPECIAL:  This mode allows the receiver to interrupt only on
              characters with a special receive condition. When an interrupt occurs, the data containing the error
              is held in the Receive FIFO until an Error Reset command is issued. When using this mode in conjunction
              with a DMA, the DMA is initialized and enabled before any characters have been
              received by the ESCC. This eliminates the time-critical section of code required in the Receive
              Interrupt on First Character or Special Condition mode. Hence, all data can be transferred via the
              DMA so that the CPU need not handle the first received character as a special case. In SDLC
              mode, if the SDLC Frame Status FIFO is enabled and an EOF is received, an interrupt with vector
              for receive data available is generated and the Receive FIFO is not locked.

To allow for control over the daisy chain, the SCC has a Disable Lower Chain (DLC) software command (WR9 bit 2)
that pulls IEO Low. This selectively deactivates parts of the daisy chain regardless of the interrupt status.

*/

//-------------------------------------------------
//  z80daisy_irq_state - get interrupt status
//-------------------------------------------------
int z80scc_device::z80daisy_irq_state()
{
	int state = 0;

	LOGINT("%s %s A:%d%d%d B:%d%d%d ", tag(), FUNCNAME,
			m_int_state[0], m_int_state[1], m_int_state[2],
			m_int_state[3], m_int_state[4], m_int_state[5]);

	// loop over all interrupt sources
	for (auto & elem : m_int_state)
	{
		// if we're servicing a request, don't indicate more interrupts
		if (elem & Z80_DAISY_IEO)
		{
			state |= Z80_DAISY_IEO;
			break;
		}
		state |= elem;
	}

	// Last chance to keep the control of the interrupt line
	state |= (m_wr9 & z80scc_channel::WR9_BIT_DLC) ? Z80_DAISY_IEO : 0;

	LOGINT("- Interrupt State %u\n", state);

	return state;
}


//-------------------------------------------------
//  z80daisy_irq_ack - interrupt acknowledge
//-------------------------------------------------
int z80scc_device::z80daisy_irq_ack()
{
	LOGINT("%s %s \n",tag(), FUNCNAME);
	// loop over all interrupt sources
	for (auto & elem : m_int_state)
	{
		// find the first channel with an interrupt requested
		if (elem & Z80_DAISY_INT)
		{
			elem = Z80_DAISY_IEO; // Set IUS bit (called IEO in z80 daisy lingo)
			check_interrupts();
			LOGINT(" - Found an INT request, ");
			if (m_wr9 & z80scc_channel::WR9_BIT_VIS)
			{
				LOGINT("but WR9 D1 set to use autovector, returning -1\n");
				return -1;
			}
			else
			{
				LOGINT("returning RR2: %02x\n", m_chanB->m_rr2 );
				return m_chanB->m_rr2;
			}
		}
	}

	return -1;
}


//-------------------------------------------------
//  z80daisy_irq_reti - return from interrupt
//-------------------------------------------------
/*
"SCC Interrupt Daisy-Chain Operation
In the SCC, the IUS bit normally controls the state of the IEO line. The IP bit affects the daisy
chain only during an Interrupt Acknowledge cycle. Since the IP bit is normally not part of the SCC
interrupt daisy chain, there is no need to decode the RETI instruction. To allow for control over the
daisy chain, the SCC has a Disable Lower Chain (DLC) software command (WR9 bit 2) that pulls IEO Low."
*/
void z80scc_device::z80daisy_irq_reti()
{
	LOGINT("%s %s - No RETI detection needed on SCC\n",tag(), FUNCNAME);
}


//-------------------------------------------------
//  check_interrupts -
//-------------------------------------------------

void z80scc_device::check_interrupts()
{
	int state = (z80daisy_irq_state() & Z80_DAISY_INT) ? ASSERT_LINE : CLEAR_LINE;
	LOGINT("%s %s \n",tag(), FUNCNAME);
	m_out_int_cb(state);
}


//-------------------------------------------------
//  reset_interrupts -
//-------------------------------------------------

void z80scc_device::reset_interrupts()
{
	LOGINT("%s %s \n",tag(), FUNCNAME);
	// reset internal interrupt sources
	for (auto & elem : m_int_state)
	{
		elem = 0;
	}

	// check external interrupt sources
	check_interrupts();
}

uint8_t z80scc_device::modify_vector(uint8_t vec, int i, uint8_t src)
{
	/*
	  Interrupt Vector Modification
	  V3 V2 V1 Status High/Status Low =0
	  V4 V5 V6 Status High/Status Low =1
	  0  0  0 Ch B Transmit Buffer Empty
	  0  0  1 Ch B External/Status Change
	  0  1  0 Ch B Receive Char. Available
	  0  1  1 Ch B Special Receive Condition
	  1  0  0 Ch A Transmit Buffer Empty
	  1  0  1 Ch A External/Status Change
	  1  1  0 Ch A Receive Char. Available
	  1  1  1 Ch A Special Receive Condition
	*/

	// Add channel offset according to table above
	src &= 3;
	src |= (i == CHANNEL_A ? 0x04 : 0x00 );

	// Modify vector according to Hi/lo bit of WR9
	if (m_wr9 & z80scc_channel::WR9_BIT_SHSL) // Affect V4-V6
	{
		vec &= 0x8f;
		vec |= src << 4;
	}
	else              // Affect V1-V3
	{
		vec &= 0xf1;
		vec |= src << 1;
	}
	return vec;
}

int z80scc_device::get_extint_priority(int type)
{
	int prio = 0;

	switch(type)
	 {
	case z80scc_channel::INT_RECEIVE:  prio = z80scc_channel::INT_RECEIVE_PRIO;  break;
	case z80scc_channel::INT_EXTERNAL: prio = z80scc_channel::INT_EXTERNAL_PRIO; break;
	case z80scc_channel::INT_TRANSMIT: prio = z80scc_channel::INT_TRANSMIT_PRIO; break;
	case z80scc_channel::INT_SPECIAL:  prio = z80scc_channel::INT_SPECIAL_PRIO;  break;
	default: logerror("Bad interrupt source being prioritized!");
	}
	return prio;
}

//-------------------------------------------------
//  trigger_interrupt -
//-------------------------------------------------
void z80scc_device::trigger_interrupt(int index, int type)
{
	uint8_t vector = m_chanA->m_rr2;
	uint8_t source = 0;
	int priority;

	int prio_level = 0;

	LOGINT("%s %s:%c %02x \n",FUNCNAME, tag(), 'A' + index, type);

	/* The Master Interrupt Enable (MIE) bit, WR9 D3, must be set to enable the SCC to generate interrupts.*/
	if (!(m_wr9 & z80scc_channel::WR9_BIT_MIE))
	{
		LOGINT("Master Interrupt Enable is not set, blocking attempt to interrupt\n");
		return;
	}

	source = type;
	prio_level = get_extint_priority(type);
	if (source < z80scc_channel::INT_TRANSMIT || source > z80scc_channel::INT_SPECIAL || prio_level < 0 || prio_level > 2)
	{
		logerror("Attempt to trigger interrupt of unknown origin blocked: %02x/%02x on channel %c\n", source, prio_level, 'A' + index);
		return;
	}
	// Vector modification requested?
	if (m_wr9 & z80scc_channel::WR9_BIT_VIS)
	{
		vector = modify_vector(vector, index, source);
	}

	LOGINT("   Interrupt Request fired of type %u and vector %02x\n", type, vector);

	// update vector register
	m_chanB->m_rr2 = vector;

	/* Check the interrupt source and build the vector modification */
	/*Interrupt Source Priority order
	  Channel A Receive
	  Channel A Transmit
	  Channel A External/Status
	  Channel B Receive
	  Channel B Transmit
	  Channel B External/Status
	*/
	// Add channel offset to priority according to table above
	priority = prio_level + (index == CHANNEL_A ? 0 : 3 );

	// trigger interrupt
	m_int_state[priority] |= Z80_DAISY_INT;

	// remember the source
	m_int_source[priority] = source;

	// Based on the fact that prio levels are aligned with the bitorder of rr3 we can do this...
	m_chanA->m_rr3 |=  ((1 << prio_level) + (index == CHANNEL_A ? 3 : 0 ));

	// check for interrupt
	check_interrupts();
}

int z80scc_device::update_extint(int index)
{
	int ret = 1; // Assume there is more external/status interrupts to serve
	uint8_t rr0  = (index == CHANNEL_A ? m_chanA->m_rr0  : m_chanB->m_rr0);
	uint8_t wr15 = (index == CHANNEL_A ? m_chanA->m_wr15 : m_chanB->m_wr15);
	uint8_t lrr0 = (index == CHANNEL_A ? m_chanA->m_extint_states : m_chanB->m_extint_states);

	LOGINT("%s(%02x)\n", FUNCNAME, index);
	// Check if any of the enabled external interrupt sources has changed and requiresd service TODO: figure out Zero Count
	if ( ((lrr0 & wr15 & 0xf8) ^ (rr0 & wr15 & 0xf8)) == 0 ) // mask off disabled and non relevant bits
	{
		LOGINT(" - All interrupts serviced\n");

		// Reset IP bit for external interrupts in both internal structure and rr3
		// - External and Special interripts has the same prio, just add channel offset
		m_int_state[z80scc_channel::INT_EXTERNAL_PRIO + (index == CHANNEL_A ? 0 : 3 )] = 0;
		// Based on the fact that prio levels are aligned with the bitorder of rr3 we can do this...
		m_chanA->m_rr3 &=  ~((1 << z80scc_channel::INT_EXTERNAL_PRIO) + (index == CHANNEL_A ? 3 : 0 ));
		ret = 0; // indicate that we are done
	}
	else
	{
		LOGINT(" - More external/status interrupts to serve: %02x\n", ((lrr0 & wr15 & 0xf8) ^ (rr0 & wr15 & 0xf8)));
	}
	return ret;
}

//-------------------------------------------------
//  m1_r - interrupt acknowledge
//-------------------------------------------------

int z80scc_device::m1_r()
{
	return z80daisy_irq_ack();
}


//-------------------------------------------------
//  zbus_r - Z-Bus read
//-------------------------------------------------
READ8_MEMBER( z80scc_device::zbus_r )
{
	int ba = 0;
	int reg = 0x20; // Default point to a non register number
	uint8_t data = 0;

	/* Expell non- Z-Bus variants */
	if ( !(m_variant & SET_Z80X30))
	{
		logerror(" zbus_r not supported by this device variant, you should probably use the universal bus variants  c*_r/w and d*_r/w (see z80scc.h)\n");
		return data;
	}

	switch ((m_chanB->m_wr0) & 7)
	{
	case z80scc_channel::WR0_Z_SEL_SHFT_LEFT:  ba = offset & 0x01; reg = (offset >> 1) & 0x0f; break; /* Shift Left mode */
	case z80scc_channel::WR0_Z_SEL_SHFT_RIGHT: ba = offset & 0x10; reg = (offset >> 1) & 0x0f; break; /* Shift Right mode */
	default:
		logerror("Malformed Z-bus SCC read: offset %02x WR0 bits %02x\n", offset, m_chanB->m_wr0);
		LOG("Malformed Z-bus SCC read: offset %02x WR0 bits %02x\n", offset, m_chanB->m_wr0);
		return data;
	}

	if (ba == 0)
		data = m_chanB->scc_register_read(reg);
	else
		data = m_chanA->scc_register_read(reg);

	return data;
}

//-------------------------------------------------
//  zbus_w - Z-Bus write
//-------------------------------------------------
WRITE8_MEMBER( z80scc_device::zbus_w )
{
	int ba = 0;
	int reg = 0x20; // Default point to a non register number

	/* Expell non- Z-Bus variants */
	if ( !(m_variant & SET_Z80X30))
	{
		logerror(" zbus_w not supported by this device variant, you should probably use the universal bus variants  c*_r/w and d*_r/w (see z80scc.h)\n");
		return;
	}

	switch ((m_chanB->m_wr0) & 7)
	{
	case z80scc_channel::WR0_Z_SEL_SHFT_LEFT:  ba = offset & 0x01; reg = (offset >> 1) & 0x0f; break; /* Shift Left mode */
	case z80scc_channel::WR0_Z_SEL_SHFT_RIGHT: ba = offset & 0x10; reg = (offset >> 1) & 0x0f; break; /* Shift Right mode */
	default:
		logerror("Malformed Z-bus SCC write: offset %02x WR0 bits %02x\n", offset, m_chanB->m_wr0);
		LOG("Malformed Z-bus SCC write: offset %02x WR0 bits %02x\n", offset, m_chanB->m_wr0);
	}

	if (ba == 0)
		m_chanB->scc_register_write(reg, data);
	else
		m_chanA->scc_register_write(reg, data);

	return;
}

//-------------------------------------------------
//  cd_ab_r - Universal Bus read
//-------------------------------------------------
READ8_MEMBER( z80scc_device::cd_ab_r )
{
	int ba = BIT(offset, 0);
	int cd = BIT(offset, 1);
	z80scc_channel *channel = ba ? m_chanA : m_chanB;

	/* Expell non-Universal Bus variants */
	if ( !(m_variant & SET_Z85X3X))
	{
		logerror(" cd_ab_r not supported by this device variant, you should probably use combinations of c*_r/w and d*_r/w (see z80scc.h)\n");
		return 0;
	}

	//    LOG("z80scc_device::cd_ba_r ba:%02x cd:%02x\n", ba, cd);
	return cd ? channel->data_read() : channel->control_read();
}

//-------------------------------------------------
//  cd_ab_w - Universal Bus write
//-------------------------------------------------
WRITE8_MEMBER( z80scc_device::cd_ab_w )
{
	int ba = BIT(offset, 0);
	int cd = BIT(offset, 1);
	z80scc_channel *channel = ba ? m_chanA : m_chanB;

	/* Expell non-Universal Bus variants */
	if ( !(m_variant & SET_Z85X3X) )
	{
		logerror(" cd_ab_w not supported by this device variant, you should probably use combinations of c*_r/w and d*_r/w (see z80scc.h)\n");
		return;
	}

	LOG(" cd_ab_w %02x => ba:%02x cd:%02x (ofs %d)\n", data, ba, cd, offset&3);
	if (cd)
		channel->data_write(data);
	else
		channel->control_write(data);
}

//-------------------------------------------------
//  cd_ba_r - Universal Bus read
//-------------------------------------------------
READ8_MEMBER( z80scc_device::cd_ba_r )
{
	int ba = BIT(offset, 0);
	int cd = BIT(offset, 1);
	z80scc_channel *channel = ba ? m_chanB : m_chanA;

	/* Expell non-Universal Bus variants */
	if ( !(m_variant & SET_Z85X3X))
	{
		logerror(" cd_ba_r not supported by this device variant, you should probably use combinations of c*_r/w and d*_r/w (see z80scc.h)\n");
		return 0;
	}

	//    LOG("z80scc_device::cd_ba_r ba:%02x cd:%02x\n", ba, cd);
	return cd ? channel->control_read() : channel->data_read();
}

//-------------------------------------------------
//  cd_ba_w - Universal Bus write
//-------------------------------------------------
WRITE8_MEMBER( z80scc_device::cd_ba_w )
{
	int ba = BIT(offset, 0);
	int cd = BIT(offset, 1);
	z80scc_channel *channel = ba ? m_chanB : m_chanA;

	/* Expell non-Universal Bus variants */
	if ( !(m_variant & SET_Z85X3X) )
	{
		logerror(" cd_ba_w not supported by this device variant, you should probably use combinations of c*_r/w and d*_r/w (see z80scc.h)\n");
		return;
	}

	//    LOG("z80scc_device::cd_ba_w ba:%02x cd:%02x\n", ba, cd);
	if (cd)
		channel->control_write(data);
	else
		channel->data_write(data);
}


//-------------------------------------------------
//  ba_cd_r - Universal Bus read
//-------------------------------------------------

READ8_MEMBER( z80scc_device::ba_cd_r )
{
	int ba = BIT(offset, 1);
	int cd = BIT(offset, 0);
	z80scc_channel *channel = ba ? m_chanB : m_chanA;

	/* Expell non-Universal Bus variants */
	if ( !(m_variant & SET_Z85X3X) )
	{
		logerror(" ba_cd_r not supported by this device variant, you should probably use combinations of c*_r/w and d*_r/w (see z80scc.h)\n");
		return 0;
	}

	//    LOG("z80scc_device::ba_cd_r ba:%02x cd:%02x\n", ba, cd);
	return cd ? channel->control_read() : channel->data_read();
}


//-------------------------------------------------
//  ba_cd_w - Universal Bus write
//-------------------------------------------------

WRITE8_MEMBER( z80scc_device::ba_cd_w )
{
	int ba = BIT(offset, 1);
	int cd = BIT(offset, 0);
	z80scc_channel *channel = ba ? m_chanB : m_chanA;

	/* Expell non-Universal Bus variants */
	if ( !(m_variant & SET_Z85X3X) )
	{
		logerror(" ba_cd_w not supported by this device variant, you should probably use combinations of c*_r/w and d*_r/w (see z80scc.h)\n");
		return;
	}

	LOG("z80scc_device::ba_cd_w ba:%02x cd:%02x\n", ba, cd);

	if (cd)
		channel->control_write(data);
	else
		channel->data_write(data);
}

//-------------------------------------------------
//  ba_cd_inv_r - Universal Bus read
//-------------------------------------------------

READ8_MEMBER( z80scc_device::ba_cd_inv_r )
{
	int ba = BIT(offset, 1);
	int cd = BIT(offset, 0);
	z80scc_channel *channel = ba ? m_chanA : m_chanB;

	/* Expell non-Universal Bus variants */
	if ( !(m_variant & SET_Z85X3X) )
	{
		logerror(" ba_cd_inv_r not supported by this device variant, you should probably use combinations of c*_r/w and d*_r/w (see z80scc.h)\n");
		return 0;
	}

	//    LOG("z80scc_device::ba_cd_inv_r ba:%02x cd:%02x\n", ba, cd);
	return cd ? channel->data_read() : channel->control_read();
}


//-------------------------------------------------
//  ba_cd_inv_w - Universal Bus read
//-------------------------------------------------

WRITE8_MEMBER( z80scc_device::ba_cd_inv_w )
{
	int ba = BIT(offset, 1);
	int cd = BIT(offset, 0);
	z80scc_channel *channel = ba ? m_chanA : m_chanB;

	/* Expell non-Universal Bus variants */
	if ( !(m_variant & SET_Z85X3X) )
	{
		logerror(" ba_cd_inv_w not supported by this device variant, you should probably use combinations of c*_r/w and d*_r/w (see z80scc.h)\n");
		return;
	}

	LOG("z80scc_device::ba_cd_inv_w ba:%02x cd:%02x\n", ba, cd);

	if (cd)
		channel->data_write(data);
	else
		channel->control_write(data);
}

//**************************************************************************
//  SCC CHANNEL
//**************************************************************************

//-------------------------------------------------
//  SCC_channel - constructor
//-------------------------------------------------

z80scc_channel::z80scc_channel(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, Z80SCC_CHANNEL, tag, owner, clock),
		device_serial_interface(mconfig, *this),
#if Z80SCC_USE_LOCAL_BRG
		m_brg_counter(0),
#else
		m_brg_rate(0),
#endif
		m_delayed_tx_brg_change(0),
		m_rx_error(0),
		m_rx_clock(0),
		m_rx_first(0),
		m_rx_break(0),
		m_extint_latch(0),
		m_extint_states(0),
		m_rxd(0),
		m_tx_clock(0),
		m_tx_int_disarm(0),
		m_dtr(0),
		m_rts(0),
		m_sync_pattern(0)
#if START_BIT_HUNT
		,m_rcv_mode(RCV_IDLE)
#endif
{
	LOG("%s\n",FUNCNAME);

	// Reset all registers
	m_rr0 = m_rr1 = m_rr2 = m_rr3  = m_rr4  = m_rr5  = m_rr6  = m_rr7 = m_rr7p = m_rr8
		= m_rr9 = m_rr10 = m_rr11 = m_rr12 = m_rr13 = m_rr14 = m_rr15 = 0;
	m_wr0 = m_wr1 = m_wr2 = m_wr3  = m_wr4  = m_wr5  = m_wr6  = m_wr7 = m_wr7p = m_wr8
		= m_wr10 = m_wr11 = m_wr12 = m_wr13 = m_wr14 = m_wr15 = 0;

	for (auto & elem : m_rx_data_fifo)
		elem = 0;
	for (auto & elem : m_rx_error_fifo)  // TODO: Status FIFO needs to be fixed
		elem = 0;
	for (auto & elem : m_tx_data_fifo)
		elem = 0;
	for (auto & elem : m_tx_error_fifo) //  TODO: Status FIFO needs to be fixed
		elem = 0;
}


//-------------------------------------------------
//  start - channel startup
//-------------------------------------------------

void z80scc_channel::device_start()
{
	LOGSETUP("%s\n", FUNCNAME);
	m_uart = downcast<z80scc_device *>(owner());
	m_index = m_uart->get_channel_index(this);

	m_uart->m_wr0_ptrbits = 0;

	m_rx_fifo_sz = (m_uart->m_variant & z80scc_device::SET_ESCC) ? 8 : 3;
	m_rx_fifo_wp = m_rx_fifo_rp = 0;

	m_tx_fifo_sz = (m_uart->m_variant & z80scc_device::SET_ESCC) ? 4 : 1;
	m_tx_fifo_wp = m_tx_fifo_rp = 0;

#if Z80SCC_USE_LOCAL_BRG
	// baudrate clocks and timers
	baudtimer = timer_alloc(TIMER_ID_BAUD);
#endif

	// state saving
	save_item(NAME(m_rr0));
	save_item(NAME(m_rr1));
	save_item(NAME(m_rr2));
	save_item(NAME(m_rr3));
	save_item(NAME(m_rr4));
	save_item(NAME(m_rr5));
	save_item(NAME(m_rr6));
	save_item(NAME(m_rr7));
	save_item(NAME(m_rr7p));
	save_item(NAME(m_rr8));
	save_item(NAME(m_rr9));
	save_item(NAME(m_rr10));
	save_item(NAME(m_rr11));
	save_item(NAME(m_rr12));
	save_item(NAME(m_rr13));
	save_item(NAME(m_rr14));
	save_item(NAME(m_rr15));
	save_item(NAME(m_wr0));
	save_item(NAME(m_wr1));
	save_item(NAME(m_wr2));
	save_item(NAME(m_wr3));
	save_item(NAME(m_wr4));
	save_item(NAME(m_wr5));
	save_item(NAME(m_wr6));
	save_item(NAME(m_wr7));
	save_item(NAME(m_wr7p));
	save_item(NAME(m_wr8));
	save_item(NAME(m_wr10));
	save_item(NAME(m_wr11));
	save_item(NAME(m_wr12));
	save_item(NAME(m_wr13));
	save_item(NAME(m_wr14));
	save_item(NAME(m_wr15));
	save_item(NAME(m_tx_data_fifo));
	save_item(NAME(m_tx_error_fifo)); //  TODO: Status FIFO needs to be fixed
	save_item(NAME(m_tx_fifo_rp));
	save_item(NAME(m_tx_fifo_wp));
	save_item(NAME(m_tx_fifo_sz));
	save_item(NAME(m_rx_data_fifo));
	save_item(NAME(m_rx_error_fifo)); //  TODO: Status FIFO needs to be fixed
	save_item(NAME(m_rx_fifo_rp));
	save_item(NAME(m_rx_fifo_wp));
	save_item(NAME(m_rx_fifo_sz));
	save_item(NAME(m_rx_clock));
	save_item(NAME(m_rx_first));
	save_item(NAME(m_rx_break));
	save_item(NAME(m_extint_latch));
	save_item(NAME(m_extint_states));
	save_item(NAME(m_ri));
	save_item(NAME(m_tx_clock));
	save_item(NAME(m_dtr));
	save_item(NAME(m_rts));
	save_item(NAME(m_tx_int_disarm));
	save_item(NAME(m_sync_pattern));
}


//-------------------------------------------------
//  reset - reset channel status
//-------------------------------------------------

void z80scc_channel::device_reset()
{
	LOGSETUP("%s\n", FUNCNAME);

	// Reset RS232 emulation
	receive_register_reset();
	transmit_register_reset();

	// Soft/Channel Reset values (mostly) according to SCC users manual
	m_wr0   = 0x00;
	m_wr1  &= 0x24;
	m_wr3  &= 0x01;
	m_wr4   = 0x04;
	m_wr5   = 0x00;
	if (m_uart->m_variant & (z80scc_device::TYPE_SCC85C30 | z80scc_device::SET_ESCC))
		m_wr7 = 0x20;
	//  WR9,WR10,WR11 and WR14 has a different hard reset (see z80scc_device::device_reset()) values
	m_uart->m_wr9 &= 0xdf;
	m_wr10 &= 0x60;
	m_wr11 &= 0xff;
	m_wr14 &= 0xc3;
	m_wr14 |= 0x20;
	m_wr15  = 0xf8;
	m_rr0  &= 0xfc;
	m_rr0  |= 0x44;
	m_rr1  &= 0x07;
	m_rr1  |= 0x06;         //  Required reset value
	m_rr1  |= RR1_ALL_SENT; // It is a don't care in the SCC user manual but drivers hangs without it set
	m_rr3   = 0x00;
	m_rr10 &= 0x40;

	// reset external lines
	set_rts(m_wr5 & WR5_RTS ? 0 : 1);
	set_dtr(m_wr14 & WR14_DTR_REQ_FUNC ? 0 : (m_wr5 & WR5_DTR ? 0 : 1));

	// reset interrupts
	if (m_index == z80scc_device::CHANNEL_A)
	{
		m_uart->reset_interrupts();
	}
	m_extint_states = m_rr0;
}

void z80scc_channel::device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr)
{
//  LOG("%s %d\n", FUNCNAME, id);

#if Z80SCC_USE_LOCAL_BRG
	switch(id)
	{
	case TIMER_ID_BAUD:
		{
			//int brconst = m_wr13 << 8 | m_wr12 | 1; // If the counter is 1 the effect is passthrough ehh?! To avoid div0...
			if (m_wr14 & WR14_BRG_ENABLE)
			{
				//  int rate = owner()->clock() / brconst;
				//  attotime attorate = attotime::from_hz(rate);
				//  timer.adjust(attorate, id, attorate);
				txc_w(m_brg_counter & 1);
				rxc_w(m_brg_counter & 1);
				m_brg_counter++; // Will just keep track of state in timer mode, not hardware counter value.
			}
			else
			{
				LOG(" - turning off Baudrate timer\n");
				timer.adjust(attotime::never, 0, attotime::never);
			}
		}
		break;
	default:
		logerror("Spurious timer %d event\n", id);
	}
#endif
}


//-------------------------------------------------
//  tra_callback -
//-------------------------------------------------

void z80scc_channel::tra_callback()
{
	if (!(m_wr5 & WR5_TX_ENABLE))
	{
		LOG(LLFORMAT " %s() \"%s \"Channel %c transmit mark 1 m_wr5:%02x\n", machine().firstcpu->total_cycles(), FUNCNAME, owner()->tag(), 'A' + m_index, m_wr5);
		// transmit mark
		if (m_index == z80scc_device::CHANNEL_A)
			m_uart->m_out_txda_cb(1);
		else
			m_uart->m_out_txdb_cb(1);
	}
	else if (m_wr5 & WR5_SEND_BREAK)
	{
		LOG(LLFORMAT " %s() \"%s \"Channel %c send break 1 m_wr5:%02x\n", machine().firstcpu->total_cycles(), FUNCNAME, owner()->tag(), 'A' + m_index, m_wr5);
		// transmit break
		if (m_index == z80scc_device::CHANNEL_A)
			m_uart->m_out_txda_cb(0);
		else
			m_uart->m_out_txdb_cb(0);
	}
	else if (!is_transmit_register_empty())
	{
		int db = transmit_register_get_data_bit();

		LOG(LLFORMAT " %s() \"%s \"Channel %c transmit data bit %d m_wr5:%02x\n", machine().firstcpu->total_cycles(), FUNCNAME, owner()->tag(), 'A' + m_index, db, m_wr5);
		// transmit data
		if (m_index == z80scc_device::CHANNEL_A)
			m_uart->m_out_txda_cb(db);
		else
			m_uart->m_out_txdb_cb(db);
	}
	else
	{
		LOG(LLFORMAT " %s() \"%s \"Channel %c Failed to transmit m_wr5:%02x\n", machine().firstcpu->total_cycles(), FUNCNAME, owner()->tag(), 'A' + m_index, m_wr5);
		logerror("%s \"%s \"Channel %c Failed to transmit\n", FUNCNAME, owner()->tag(), 'A' + m_index);
	}
}


//-------------------------------------------------
//  tra_complete -
//-------------------------------------------------

void z80scc_channel::tra_complete()
{
	// Delayed baudrate change according to SCC specs
	if ( m_delayed_tx_brg_change == 1)
	{
		m_delayed_tx_brg_change = 0;
		set_tra_rate(m_brg_rate);
		LOG("Delayed Init - Baud Rate Generator: %d mode: %dx\n", m_brg_rate, get_clock_mode() );
	}

	if ((m_wr5 & WR5_TX_ENABLE) && !(m_wr5 & WR5_SEND_BREAK))
	{
		if ( (m_rr0 & RR0_TX_BUFFER_EMPTY) == 0 || // Takes care of the NMOS/CMOS 1 slot TX FIFO
			 m_tx_fifo_rp != m_tx_fifo_wp) // or there are more characters to send in a longer FIFO.
		{
			LOGTX(" %s() %s %c done sending, loading data from fifo:%02x '%c'\n", FUNCNAME, owner()->tag(), 'A' + m_index,
				   m_tx_data_fifo[m_tx_fifo_rp], isascii(m_tx_data_fifo[m_tx_fifo_rp]) ? m_tx_data_fifo[m_tx_fifo_rp] : ' ');
			transmit_register_setup(m_tx_data_fifo[m_tx_fifo_rp]); // Reload the shift register
			m_tx_fifo_rp_step();
			m_rr0 |= RR0_TX_BUFFER_EMPTY; // Now here is room in the tx fifo again
		}
		else
		{
			LOGTX(" %s() %s %c done sending, setting all sent bit\n", FUNCNAME, owner()->tag(), 'A' + m_index);
			m_rr1 |= RR1_ALL_SENT;

			// when the RTS bit is reset, the _RTS output goes high after the transmitter empties
			if (!m_rts) // TODO: Clean up RTS handling
				set_rts(1);
		}

		check_waitrequest();

		if (m_wr1 & WR1_TX_INT_ENABLE && m_tx_int_disarm == 0)
		{
			if ((m_uart->m_variant & z80scc_device::SET_ESCC) &&
				(m_wr7p & WR7P_TX_FIFO_EMPTY)  &&
				m_tx_fifo_wp == m_tx_fifo_rp)  // ESCC and fifo empty bit set and fifo is completelly empty?
			{
				m_uart->trigger_interrupt(m_index, INT_TRANSMIT); // Set TXIP bit
			}
			else if(m_rr0 & RR0_TX_BUFFER_EMPTY)  // Check TBE bit and interrupt if one or more FIFO slots available
			{
				m_uart->trigger_interrupt(m_index, INT_TRANSMIT); // Set TXIP bit
			}
		}
		/* Arm interrupts since we completed another data byte, however it may be set by the reset tx int pending
		command before the shifter is done and the disarm flag is evaluated again in tra_complete()  */
		m_tx_int_disarm = 0;
	}
	else if (m_wr5 & WR5_SEND_BREAK)
	{
		LOG(LLFORMAT " %s() \"%s \"Channel %c Transmit Break 0 m_wr5:%02x\n", machine().firstcpu->total_cycles(), FUNCNAME, owner()->tag(), 'A' + m_index, m_wr5);
		// transmit break
		if (m_index == z80scc_device::CHANNEL_A)
			m_uart->m_out_txda_cb(0);
		else
			m_uart->m_out_txdb_cb(0);
	}
	else
	{
		LOG(LLFORMAT " %s() \"%s \"Channel %c Transmit Mark 1 m_wr5:%02x\n", machine().firstcpu->total_cycles(), FUNCNAME, owner()->tag(), 'A' + m_index, m_wr5);
		// transmit mark
		if (m_index == z80scc_device::CHANNEL_A)
			m_uart->m_out_txda_cb(1);
		else
			m_uart->m_out_txdb_cb(1);
	}
}


//-------------------------------------------------
//  rcv_callback -
//-------------------------------------------------

void z80scc_channel::rcv_callback()
{
	if (m_wr3 & WR3_RX_ENABLE)
	{
		LOG(LLFORMAT " %s() \"%s \"Channel %c receive data bit %d m_wr3:%02x\n", machine().firstcpu->total_cycles(), FUNCNAME, owner()->tag(), 'A' + m_index, m_rxd, m_wr3);
		receive_register_update_bit(m_rxd);
	}
#if 1
	else
	{
		LOG(LLFORMAT " %s() \"%s \"Channel %c Received Data Bit but receiver is disabled\n", machine().firstcpu->total_cycles(), FUNCNAME, owner()->tag(), 'A' + m_index);
		logerror("%s \"%s \"Channel %c Received data dit but receiver is disabled\n", FUNCNAME, owner()->tag(), 'A' + m_index);
	}
#endif
}


//-------------------------------------------------
//  rcv_complete -
//-------------------------------------------------

void z80scc_channel::rcv_complete()
{
	uint8_t data;

	receive_register_extract();
	data = get_received_char();
	LOG(LLFORMAT " %s() \"%s \"Channel %c Received Data %c\n", machine().firstcpu->total_cycles(), FUNCNAME, owner()->tag(), 'A' + m_index, data);
	receive_data(data);
#if START_BIT_HUNT
	m_rcv_mode = RCV_SEEKING;
#endif
}


//-------------------------------------------------
//  get_clock_mode - get clock divisor
//-------------------------------------------------

int z80scc_channel::get_clock_mode()
{
	int clocks = 1;

	switch (m_wr4 & WR4_CLOCK_RATE_MASK)
	{
	case WR4_CLOCK_RATE_X1: clocks = 1; break;
	case WR4_CLOCK_RATE_X16:    clocks = 16;    break;
	case WR4_CLOCK_RATE_X32:    clocks = 32;    break;
	case WR4_CLOCK_RATE_X64:    clocks = 64;    break;
	}

	return clocks;
}

/* From Zilog SCC/ESCC USers manual, UM010902-0609:
"/RTSA, /RTSB. Request To Send (outputs, active Low). The /RTS pins can be used as generalpurpose
outputs or with the Auto Enable feature. When used with Auto Enable ON (WR3, D5=1)
in asynchronous mode, the /RTS pin goes High after the transmitter is empty. When Auto Enable
is OFF, the /RTS pins are used as general-purpose outputs, and, they strictly follow the inverse
state of WR5, bit D1.
ESCC and 85C30: In SDLC mode, the /RTS pins can be programmed to be deasserted when the closing
flag of the message clears the TxD pin, if WR7' D2 is set."
TODO:
- SDLC mode behaviour for ESCC/85C30
*/
void z80scc_channel::set_rts(int state)
{
	LOG("%s(%d) \"%s\": %c \n", FUNCNAME, state, owner()->tag(), 'A' + m_index);
	if (m_index == z80scc_device::CHANNEL_A)
		m_uart->m_out_rtsa_cb(state);
	else
		m_uart->m_out_rtsb_cb(state);
}

void z80scc_channel::update_rts()
{
//    LOG("%s(%d) \"%s\": %c \n", FUNCNAME, state, owner()->tag(), 'A' + m_index);
		if (m_wr5 & WR5_RTS)
		{
				// when the RTS bit is set, the _RTS output goes low
				set_rts(0);
				m_rts = 1;
		}
		else
		{
				// when the RTS bit is reset, the _RTS output goes high after the transmitter empties
				m_rts = 0;
		}

		// data terminal ready output follows the state programmed into the DTR bit*/
		set_dtr((m_wr5 & WR5_DTR) ? 0 : 1);
}

//-------------------------------------------------
//  get_stop_bits - get number of stop bits
//-------------------------------------------------

device_serial_interface::stop_bits_t z80scc_channel::get_stop_bits()
{
	switch (m_wr4 & WR4_STOP_BITS_MASK)
	{
	case WR4_STOP_BITS_1: return STOP_BITS_1;
	case WR4_STOP_BITS_1_5: return STOP_BITS_1_5;
	case WR4_STOP_BITS_2: return STOP_BITS_2;
	}

	return STOP_BITS_0;
}


//-------------------------------------------------
//  get_rx_word_length - get receive word length
//-------------------------------------------------

int z80scc_channel::get_rx_word_length()
{
	int bits = 5;

	switch (m_wr3 & WR3_RX_WORD_LENGTH_MASK)
	{
	case WR3_RX_WORD_LENGTH_5:  bits = 5;   break;
	case WR3_RX_WORD_LENGTH_6:  bits = 6;   break;
	case WR3_RX_WORD_LENGTH_7:  bits = 7;   break;
	case WR3_RX_WORD_LENGTH_8:  bits = 8;   break;
	}

	return bits;
}


//-------------------------------------------------
//  get_tx_word_length - get transmit word length
//-------------------------------------------------

int z80scc_channel::get_tx_word_length()
{
	int bits = 5;

	switch (m_wr5 & WR5_TX_WORD_LENGTH_MASK)
	{
	case WR5_TX_WORD_LENGTH_5:  bits = 5;   break;
	case WR5_TX_WORD_LENGTH_6:  bits = 6;   break;
	case WR5_TX_WORD_LENGTH_7:  bits = 7;   break;
	case WR5_TX_WORD_LENGTH_8:  bits = 8;   break;
	}

	return bits;
}

/*
 * This register contains the status of the receive and transmit buffers; the
 * DCD, CTS, and SYNC inputs; the Transmit Underrun/EOM latch; and the
 * Break/Abort latch. */
uint8_t z80scc_channel::do_sccreg_rr0()
{
	uint8_t rr0 = m_rr0;

	LOG("%s %c %s <- %02x\n",tag(), 'A' + m_index, FUNCNAME, m_rr0);
	if (m_extint_latch == 1)
	{
		rr0 &=                     ((~m_wr15) | WR15_WR7PRIME | WR15_STATUS_FIFO);  // clear enabled bits, saving 2 unrelated bits
		rr0 |= (m_extint_states & ~((~m_wr15) | WR15_WR7PRIME | WR15_STATUS_FIFO)); // set enabled bits to latched states
		LOGINT("- %c returning latched value RR0:%02x WR15:%02x => %02x\n", 'A' + m_index, m_rr0, m_wr15, rr0);
	}
	else
	{
		LOG("- %c returning unlatched value: %02x\n", 'A' + m_index, rr0);
	}

	return rr0;
}

/*
 * This register contains the Special Receive condition status bits and Residue
 * codes for the I-Field in the SDLC Receive Mode. */
uint8_t z80scc_channel::do_sccreg_rr1()
{
	LOGR("%s %s <- %02x\n",tag(), FUNCNAME, m_rr1);
	return m_rr1;
}

/* From Zilog SCC/ESCC USers manual, UM010902-0609:
"RR2 contains the interrupt vector written into WR2. When the register is accessed in Channel A,
the vector returned is the vector actually stored in WR2. When this register is accessed in Channel
B, the vector returned includes status information in bits 1, 2 and 3 or in bits 6, 5 and 4, depending
on the state of the Status High/Status Low bit in WR9 and independent of the state of the VIS bit
in WR9."*/
uint8_t z80scc_channel::do_sccreg_rr2()
{
	LOGINT("%s\n", FUNCNAME);

	// Assume the unmodified in polled mode
	m_rr2 = m_uart->m_chanA->m_wr2;

	// If we are chan B we have to modify the vector regardless of the VIS bit
	if (m_index == z80scc_device::CHANNEL_B)
	{
		int i = 0;

		LOGINT(" - Channel B so we might need to update the vector modification\n");
		// loop over all interrupt sources
		for (auto & elem : m_uart->m_int_state)
		{
			// find the first channel with an interrupt requested
			if (elem & Z80_DAISY_INT)
			{
				LOGINT(" - Checking an INT source %d\n", i);
				m_rr2 = m_uart->modify_vector(m_rr2, i < 3 ? z80scc_device::CHANNEL_A : z80scc_device::CHANNEL_B, m_uart->m_int_source[i] & 3);
				if ((m_uart->m_variant & (z80scc_device::SET_ESCC | z80scc_device::SET_CMOS)) && (m_uart->m_wr9 & WR9_BIT_IACK))
				{
					LOGINT(" - Found an INT request to ack while reading RR2\n");
					elem = Z80_DAISY_IEO; // Set IUS bit (called IEO in z80 daisy lingo)
					m_uart->check_interrupts();
				}
				break;
			}
			i++;
		}
	}
	return m_rr2;
}

/* From Zilog SCC/ESCC USers manual, UM010902-0609:
RR3 is the interrupt Pending register. The status of each of the interrupt Pending bits in the SCC is
reported in this register. This register exists only in Channel A. If this register is accessed in Channel
B, all 0s are returned. The two unused bits are always returned as 0. Figure displays the bit positions for RR3."

     Chan B    |Chan A    | Unused
-------------------------------------
Bit: D0  D1 D2 |D3  D4 D5 |D6 D7
     Ext Tx Rx |Ext Tx Rx | 0  0
*/
uint8_t z80scc_channel::do_sccreg_rr3()
{
	LOGR("%s(%02x)\n", FUNCNAME, m_rr3);
	return m_index == z80scc_device::CHANNEL_A ? m_rr3 & 0x3f : 0; // TODO Update all bits of this status register
}


/* (ESCC and 85C30 Only) */
/*On the ESCC, Read Register 4 reflects the contents of Write Register 4 provided the Extended
  Read option is enabled. Otherwise, this register returns an image of RR0. On the NMOS/CMOS version,
  a read to this location returns an image of RR0.*/
uint8_t z80scc_channel::do_sccreg_rr4()
{
	LOGR("%s\n", FUNCNAME);
	if (m_uart->m_variant & (z80scc_device::SET_ESCC | z80scc_device::TYPE_SCC85C30))
		return (BIT(m_wr7, 6) ? m_wr4 : m_rr0);
	else
		return m_rr0;
}

	/* (ESCC and 85C30 Only) */
/*On the ESCC, Read Register 5 reflects the contents of Write Register 5 provided the Extended
  Read option is enabled. Otherwise, this register returns an image of RR1. On the NMOS/CMOS version,
  a read to this register returns an image of RR1.*/
uint8_t z80scc_channel::do_sccreg_rr5()
{
	LOGR("%s\n", FUNCNAME);
	if (m_uart->m_variant & (z80scc_device::SET_ESCC | z80scc_device::TYPE_SCC85C30))
		return BIT(m_wr7, 6) ? m_wr5 : m_rr1;
	else
		return m_rr1;
}

/* (not on NMOS)
 On the CMOS and ESCC, Read Register 6 contains the least significant byte of the frame byte
 count that is currently at the top of the Status FIFO. RR6 is displayed in Figure on page 183. This
 register is readable only if the FIFO is enabled (refer to the description Write Register 15, bit D2,
 and SDLC Frame Status FIFO on page 126). Otherwise, this register is an image of RR2.
 On the NMOS version, a read to this register location returns an image of RR2.*/
uint8_t z80scc_channel::do_sccreg_rr6()
{
	LOGR("%s\n", FUNCNAME);
	if (m_wr15 & WR15_STATUS_FIFO)
	{
		LOGSYNC(" - Status FIFO for synchronous mode - not implemented\n");
		logerror(" - Status FIFO for synchronous mode - not implemented\n");
		return 0;
	}
	return m_rr2; /* Note that NMOS calls are redirected to do_sccreg_rr2() before getting here */
}

/* (not on NMOS)
 On the CMOS and ESCC, Read Register 7 contains the most significant six bits of the frame byte
 count that is currently at the top of the Status FIFO. Bit D7 is the FIFO Overflow Status and bit D6
 is the FIFO Data Available Status. The status indications are given in Table on page 184. RR7 is
 displayed in Figure on page 183. This register is readable only if the FIFO is enabled (refer to the
 description Write Register 15, bit D2). Otherwise this register is an image of RR3. Note, for proper
 operation of the FIFO and byte count logic, the registers should be read in the following order:
 RR7, RR6, RR1.*/
uint8_t z80scc_channel::do_sccreg_rr7()
{
	LOGR("%s\n", FUNCNAME);
	if (!(m_uart->m_variant & (z80scc_device::SET_NMOS)))
	{
		logerror(" %s() not implemented feature\n", FUNCNAME);
		return 0;
	}
		return m_rr3;
}

#if 0 // Short cutted in control_read()
/* RR8 is the Receive Data register. */
uint8_t z80scc_channel::do_sccreg_rr8()
{
	return data_read():
}
#endif

/* (ESCC and 85C30 Only)
 On the ESCC, Read Register 9 reflects the contents of Write Register 3 provided the Extended
 Read option has been enabled. On the NMOS/CMOS version, a read to this location returns an image
 of RR13. TODO: Check what is returned if Extended Read option is turned off */
uint8_t z80scc_channel::do_sccreg_rr9()
{
	LOGR("%s\n", FUNCNAME);
	if (m_uart->m_variant & (z80scc_device::SET_ESCC | z80scc_device::TYPE_SCC85C30))
		return BIT(m_wr7, 6) ? m_wr3 : m_rr13;
	else
		return m_rr13;
}

/* RR10 contains some SDLC related miscellaneous status bits. Unused bits are always 0. */
uint8_t z80scc_channel::do_sccreg_rr10()
{
	LOGR("%s\n", FUNCNAME);
	logerror("%s() not implemented feature\n", FUNCNAME);
	return m_rr10;
}

/* (ESCC and 85C30 Only)
 On the ESCC, Read Register 11 reflects the contents of Write Register 10 provided the Extended
 Read option has been enabled. Otherwise, this register returns an image of RR15.
 On the NMOS/CMOS version, a read to this location returns an image of RR15.*/
uint8_t z80scc_channel::do_sccreg_rr11()
{
	LOGR("%s\n", FUNCNAME);
	if (m_uart->m_variant & (z80scc_device::SET_ESCC | z80scc_device::TYPE_SCC85C30))
		return BIT(m_wr7, 6) ? m_wr10 : m_rr15;
	else
		return m_rr15;
}

/*
 RR12 returns the value stored in WR12, the lower byte of the time constant, for the BRG.*/
uint8_t z80scc_channel::do_sccreg_rr12()
{
	LOGR("%s\n", FUNCNAME);
	return m_wr12;
}

/*
  RR13 returns the value stored in WR13, the upper byte of the time constant for the BRG. */
uint8_t z80scc_channel::do_sccreg_rr13()
{
	LOGR("%s\n", FUNCNAME);
	return m_wr13;
}

/* (ESCC and 85C30 Only)
On the ESCC, Read Register 14 reflects the contents of Write Register 7 Prime provided the
Extended Read option has been enabled. Otherwise, this register returns an image of RR10.
On the NMOS/CMOS version, a read to this location returns an image of RR10.*/
uint8_t z80scc_channel::do_sccreg_rr14()
{
	LOGR("%s\n", FUNCNAME);
	if (m_uart->m_variant & (z80scc_device::SET_ESCC | z80scc_device::TYPE_SCC85C30))
		return BIT(m_wr7, 6) ? m_wr7 : m_rr10;
	else
		return m_rr10;
}

/*
 RR15 reflects the value stored in WR15, the External/Status IE bits. The two unused bits are
 always returned as Os. */
uint8_t z80scc_channel::do_sccreg_rr15()
{
	LOGR("%s\n", FUNCNAME);

	return m_wr15 & 0xfa; // Mask out the used bits
}

//-------------------------------------------------
//  scc_register_read - read a SCC register
//-------------------------------------------------
uint8_t z80scc_channel::scc_register_read( uint8_t reg)
{
	if (reg > 1)
		LOG("%s %02x\n", FUNCNAME, reg);
	uint8_t data = 0;
	uint8_t wreg = 0;

	/* Sort out 80X30 limitations in register access */
	if (BIT(m_wr15, 2) == 0 || m_uart->m_variant & z80scc_device::SET_NMOS)
	{
		if (reg > 3 && reg < 8) reg &= 0x03;
		else if (reg == 9) reg = 13;
		else if (reg == 11) reg = 15;
	}
	else if (BIT(m_wr15, 2) != 0)
	{
		if (m_uart->m_variant & z80scc_device::SET_ESCC && BIT(m_wr7p, 6) != 0)
		{
			if (reg > 3 && reg < 6) wreg = 1;
			else if (reg == 9)  { reg = 3;  wreg = 1; }
			else if (reg == 11) { reg = 10; wreg = 1; }
			else if (reg == 14) { reg = 7;  wreg = 1; }
		}
		else
		{
			if (reg > 3 && reg < 6) reg &= 3;
			else if (reg == 9)  { reg = 13; }
			else if (reg == 11) { reg = 15; }
		}
	}
	switch (reg)
	{
	case REG_RR0_STATUS:         data = do_sccreg_rr0(); break; // TODO: verify handling of SCC specific bits: D6 and D1
	case REG_RR1_SPEC_RCV_COND:  data = do_sccreg_rr1(); break;
	case REG_RR2_INTERRUPT_VECT: data = do_sccreg_rr2(); break; // Channel dependent and SCC specific handling compared to SIO
		/* registers 3-7 are specific to SCC. TODO: Check variant and log/stop misuse */
	case REG_RR3_INTERUPPT_PEND: data = wreg ? m_wr3 : do_sccreg_rr3(); break;
	case REG_RR4_WR4_OR_RR0:     data = wreg ? m_wr4 : do_sccreg_rr4(); break;
	case REG_RR5_WR5_OR_RR0:     data = wreg ? m_wr5 : do_sccreg_rr5(); break;
	case REG_RR6_LSB_OR_RR2:     data = do_sccreg_rr6(); break;
	case REG_RR7_MSB_OR_RR3:     data = wreg ? m_wr7p : do_sccreg_rr7(); break;
		/* registers 8-15 are specific to SCC */
	case REG_RR8_RECEIVE_DATA:   data = data_read(); break;
	case REG_RR9_WR3_OR_RR13:    data = do_sccreg_rr9(); break;
	case REG_RR10_MISC_STATUS:   data = wreg ? m_wr10 : do_sccreg_rr10(); break;
	case REG_RR11_WR10_OR_RR15:  data = do_sccreg_rr11(); break;
	case REG_RR12_LO_TIME_CONST: data = do_sccreg_rr12(); break;
	case REG_RR13_HI_TIME_CONST: data = do_sccreg_rr13(); break;
	case REG_RR14_WR7_OR_R10:    data = do_sccreg_rr14(); break;
	case REG_RR15_WR15_EXT_STAT: data = do_sccreg_rr15(); break;
	default:
		logerror(" \"%s\" %s: %c : Unsupported RRx register:%02x\n", owner()->tag(), FUNCNAME, 'A' + m_index, reg);
	}
	return data;
}

//-------------------------------------------------
//  control_read - read control register
//-------------------------------------------------
uint8_t z80scc_channel::control_read()
{
	uint8_t data = 0;
	int reg = m_uart->m_wr0_ptrbits;
	int regmask = (WR0_REGISTER_MASK | (m_uart->m_wr0_ptrbits & WR0_POINT_HIGH));

	LOGR("%s(%02x) reg %02x, regmask %02x, WR0 %02x\n", FUNCNAME, data, reg, regmask, m_wr0);
	m_uart->m_wr0_ptrbits = 0;
	reg &= regmask;

	if (reg != 0)
	{
		LOG("%s(%02x) reg %02x, regmask %02x, WR0 %02x\n", FUNCNAME, data, reg, regmask, m_wr0);
		m_wr0 &= ~regmask; // mask out register index
	}

	data = scc_register_read(reg);

	//LOG("%s \"%s\": %c : Register R%d read '%02x'\n", FUNCNAME, owner()->tag(), 'A' + m_index, reg, data);
	return data;
}

/**/
void z80scc_channel::do_sccreg_wr0(uint8_t data)
{
	m_wr0 = data;

	if (m_uart->m_variant & z80scc_device::SET_Z85X3X)
		m_uart->m_wr0_ptrbits = data & WR0_REGISTER_MASK;

	switch (data & WR0_COMMAND_MASK)
	{
	case WR0_POINT_HIGH:
		/*This command effectively adds eight to the Register Pointer (D2-D0) by allowing
		  WR8 through WR15 to be accessed. The Point High command and the Register
		  Pointer bits are written simultaneously. This command is used in the Z85X30
		  version of the SCC. Note that WR0 changes form depending upon the SCC version.
		  Register access for the Z80X30 version of the SCC is accomplished through direct
		  addressing*/
		if (m_uart->m_variant & z80scc_device::SET_Z85X3X)
		{
			LOG("%s %s: %c : - Point High command\n", FUNCNAME, owner()->tag(), 'A' + m_index);
			m_uart->m_wr0_ptrbits |= 8;
		}
		else
			LOG("%s %s: %c : - NULL command 2\n", FUNCNAME, owner()->tag(), 'A' + m_index);
		break;
	case WR0_RESET_EXT_STATUS:
		/*After an External/Status interrupt (a change on a modem line or a break condition,
		  for example), the status bits in RR0 are latched. This command re-enables the bits
		  and allows interrupts to occur again as a result of a status change. Latching the
		  status bits captures short pulses until the CPU has time to read the change.
		  The SCC contains simple queueing logic associated with most of the external status
		  bits in RR0. If another External/Status condition changes while a previous condition
		  is still pending (Reset External/Status Interrupt has not yet been issued) and this
		  condition persists until after the command is issued, this second change causes another
		  External/Status interrupt. However, if this second status change does not persist
		  (there are two transitions), another interrupt is not generated. Exceptions to this
		  rule are detailed in the RR0 description.*/

		LOGCMD("%s %c - Reset External/Status Interrupt, latch %s\n", owner()->tag(), 'A' + m_index,
			 m_extint_latch == 1? "is released" : "was already released");
		// Release latch if no other external or status sources are active
		if ((m_extint_latch = m_uart->update_extint(m_index)) == 0)
			m_uart->check_interrupts();
		break;
	case WR0_RESET_HIGHEST_IUS:
		/* This command resets the highest priority Interrupt Under Service (IUS) bit, allowing lower
		   priority conditions to request interrupts. This command allows the use of the internal
		   daisy chain (even in systems without an external daisy chain) and is the last operation in
		   an interrupt service routine. */
		if (m_uart->m_variant & z80scc_device::SET_NMOS)
		{
			logerror("WR0 SWI ack command not supported on NMOS\n");
			LOGCMD("%s: %c : WR0_RESET_HIGHEST_IUS command not available on NMOS!\n", owner()->tag(), 'A' + m_index);
		}
		else
		{
			LOGCMD("%s: %c : Reset Highest IUS\n", owner()->tag(), 'A' + m_index);
			// loop over all interrupt sources
			for (auto & elem : m_uart->m_int_state)
			{
				// find the first channel with an interrupt requested
				if (elem & Z80_DAISY_INT)
				{
					LOGCMD("- %c found IUS bit to clear\n", 'A' + m_index);
					elem = 0; // Clear IUS bit (called IEO in z80 daisy lingo)
					m_uart->check_interrupts();
					break;
				}
			}
		}
		break;
	case WR0_ERROR_RESET:
		/*Error Reset Command (110). This command resets the error bits in RR1. If interrupt on first Rx
		  Character or Interrupt on Special Condition modes is selected and a special condition exists, the
		  data with the special condition is held in the Receive FIFO until this command is issued. If either
		  of these modes is selected and this command is issued before the data has been read from the
		  Receive FIFO, the data is lost */
		LOGCMD("%s: %c : WR0_ERROR_RESET - not implemented\n", owner()->tag(), 'A' + m_index);
		m_rx_fifo_rp_step(); // Reset error state in fifo and unlock it. unlock == step to next slot in fifo.
		break;
	case WR0_SEND_ABORT: // Flush transmitter and Send 8-13 bits of '1's, used with SDLC
		LOGCMD("%s: %c : WR0_SEND_ABORT - not implemented\n", owner()->tag(), 'A' + m_index);
		break;
	case WR0_NULL: // Do nothing
		LOGCMD("%s: %c : WR0_NULL\n", owner()->tag(), 'A' + m_index);
		break;
	case WR0_ENABLE_INT_NEXT_RX: // enable interrupt on next receive character
		LOGCMD("%s: %c : WR0_ENABLE_INT_NEXT\n", owner()->tag(), 'A' + m_index);
		m_rx_first = 1;
		break;
	case WR0_RESET_TX_INT: // reset transmitter interrupt pending
		/*Reset Tx Interrupt Pending Command (101). This command is used in cases where there are no
		  more characters to be sent; e.g., at the end of a message. This command prevents further transmit
		  interrupts until after the next character has been loaded into the transmit buffer or until CRC has
		  been completely sent. This command is necessary to prevent the transmitter from requesting an
		  interrupt when the transmit buffer becomes empty (with Transmit Interrupt Enabled).*/
		m_tx_int_disarm = 1;
		LOGCMD("%s: %c : WR0_RESET_TX_INT\n", owner()->tag(), 'A' + m_index);
		m_uart->m_int_state[INT_TRANSMIT_PRIO + (m_index == z80scc_device::CHANNEL_A ? 0 : 3 )] = 0;
		// Based on the fact that prio levels are aligned with the bitorder of rr3 we can do this...
		m_uart->m_chanA->m_rr3 &=  ~((1 << INT_TRANSMIT_PRIO) + (m_index == z80scc_device::CHANNEL_A ? 3 : 0 ));
		// Update interrupt line
		m_uart->check_interrupts();
		break;
	default:
		break;
	}

	/* CRC Initialization Code handling */
	switch (data & WR0_CRC_RESET_CODE_MASK)
	{
	case WR0_CRC_RESET_NULL:
		LOGCMD(" CRC_RESET_NULL\n");
		break;
	case WR0_CRC_RESET_RX: /* In Synchronous mode: all Os (zeros) (CCITT-O CRC-16) */
		LOGCMD(" CRC_RESET_RX - not implemented\n");
		break;
	case WR0_CRC_RESET_TX: /* In HDLC mode: all 1s (ones) (CCITT-1) */
		LOGCMD(" CRC_RESET_TX - not implemented\n");
		break;
	case WR0_CRC_RESET_TX_UNDERRUN: /* Resets Tx underrun/EOM bit (D6 of the RRO register) */
		LOGCMD(" CRC_RESET_TX_UNDERRUN - not implemented\n");
		break;
	default: /* Will not happen unless someone messes with the mask */
		logerror(" Wrong CRC reset/init command:%02x\n", data & WR0_CRC_RESET_CODE_MASK);
	}

	if (m_uart->m_variant & z80scc_device::SET_Z85X3X)
	{
		m_uart->m_wr0_ptrbits &= ~WR0_REGISTER_MASK;
		m_uart->m_wr0_ptrbits |= (m_wr0 & (WR0_REGISTER_MASK));
	}
}

/* Write Register 1 is the control register for the various SCC interrupt and Wait/Request modes.*/
void z80scc_channel::do_sccreg_wr1(uint8_t data)
{
	LOG("%s(%02x) \"%s\": %c : %s - %02x\n", FUNCNAME, data, owner()->tag(), 'A' + m_index, FUNCNAME, data);
	/* TODO: Sort out SCC specific behaviours  from legacy SIO behaviours:
	   - Channel B only bits vs
	   - Parity Is Special Condition, bit2 */
	m_wr1 = data;
	LOG("- External Interrupt Enable %u\n", (data & WR1_EXT_INT_ENABLE) ? 1 : 0);
	LOG("- Transmit Interrupt Enable %u\n", (data & WR1_TX_INT_ENABLE) ? 1 : 0);
	LOG("- Parity is special condition %u\n", (data & WR1_PARITY_IS_SPEC_COND) ? 1 : 0);
	LOG("- Wait/DMA Request Enable %u\n", (data & WR1_WREQ_ENABLE) ? 1 : 0);
	LOG("- Wait/DMA Request Function %s\n", (data & WR1_WREQ_FUNCTION) ? "Request" : "Wait");
	LOG("- Wait/DMA Request on %s\n", (data & WR1_WREQ_ON_RX_TX) ? "Receive" : "Transmit");

	check_waitrequest();

	switch (data & WR1_RX_INT_MODE_MASK)
	{
	case WR1_RX_INT_DISABLE:
		LOG("- Receiver Interrupt Disabled\n");
		break;

	case WR1_RX_INT_FIRST:
		LOG("- Receiver Interrupt on First Character or Special Conditions\n");
		break;

	case WR1_RX_INT_ALL:
		LOG("- Receiver Interrupt on All Characters or Special Conditions\n");
		break;

	case WR1_RX_INT_PARITY:
		LOG("- Receiver Interrupt on Special Conditions only\n");
		break;
	}
	if ((data & WR1_RX_INT_MODE_MASK) == WR1_PARITY_IS_SPEC_COND)
		LOG("- Parity error is a Special Condition\n");
	m_uart->check_interrupts();
}

/* WR2 is the interrupt vector register. Only one vector register exists in the SCC, and it can be
   accessed through either channel. The interrupt vector can be modified by status information. This
   is controlled by the Vector Includes Status (VIS) and the Status High/Status Low bits in WR9.*/
void z80scc_channel::do_sccreg_wr2(uint8_t data)
{
	LOG("%s(%02x) Setting the interrupt vector\n", FUNCNAME, data);
	m_wr2 = data;
	m_uart->m_chanA->m_rr2 = data;
	m_uart->m_chanB->m_rr2 = data; /* TODO: Sort out the setting of ChanB depending on bits in WR9 */

	m_uart->check_interrupts();
}

/*
  In the SDLC modes, the Sync/Hunt bit is initially set by the Enter Hunt Mode command or when
  the receiver is disabled. It is reset when the opening flag of the first frame is detected by the SCC.
  An External/Status interrupt is also generated if the Sync/Hunt IE bit is set. Unlike the Monosync
  and Bisync modes, once the Sync/Hunt bit is reset in SDLC mode, it does not need to be set when
  the end of the frame is detected. The SCC automatically maintains synchronization. The only way
  the Sync/Hunt bit is set again is by the Enter Hunt Mode command or by disabling the receiver.*/
void z80scc_channel::do_sccreg_wr3(uint8_t data)
{
	LOG("%s(%02x) Setting up the receiver\n", FUNCNAME, data);
	m_wr3 = data;
	LOG("- Receiver Enable:        %u\n", (data & WR3_RX_ENABLE) ? 1 : 0);
	LOG("- Sync Char Load Inhibit  %u\n", (data & WR3_SYNC_CHAR_LOAD_INHIBIT) ? 1 : 0);
	LOG("- Address Search Mode     %u\n", (data & WR3_ADDRESS_SEARCH_MODE) ? 1 : 0);
	LOG("- Rx CRC Enable           %u\n", (data & WR3_RX_CRC_ENABLE) ? 1 : 0);
	LOG("- Enter Hunt Mode         %u\n", (data & WR3_ENTER_HUNT_MODE) ? 1 : 0);
	LOG("- Auto Enables            %u\n", (data & WR3_AUTO_ENABLES) ? 1 : 0);
	LOG("- Receiver Bits/Character %u\n", get_rx_word_length());

	if ((m_wr3 & WR3_ENTER_HUNT_MODE) || ((m_wr3 & WR3_RX_ENABLE) == 0))
	{
		m_rr0 |= RR0_SYNC_HUNT; // Set the sync/hunt bit
	}
	update_serial();
	receive_register_reset();
}

void z80scc_channel::do_sccreg_wr4(uint8_t data)
{
	LOG("%s(%02x) Setting up asynchronous frame format and clock\n", FUNCNAME, data);
	if (data == m_wr4)
	{
		logerror("- suppressing reinit of Tx as write to wr4 is identical to previous value\n");
	}
	else
	{
		m_wr4 = data;
		LOG("- Parity    : %s\n", (data & WR4_PARITY_ENABLE) ? ((data & WR4_PARITY_EVEN) ? "Even" : "Odd") : "None");
		LOG("- Stop Bits : %s\n", data & WR4_STOP_BITS_MASK ? stop_bits_tostring(get_stop_bits()) : "not used, sync modes enabled" );
		LOG("- Sync Mode : %s\n", !(data & WR4_STOP_BITS_MASK) ?
			 (data & WR4_BIT5 ?
			  (data & WR4_BIT4 ? "External Sync Mode - /SYNC is used as input!" : "SDLC - not implemented")
			  : (data & WR4_BIT4 ? "16 bit" : "8 bit"))
			 : "Disabled");
		LOG("- Clock Mode: %uX\n", get_clock_mode());
		update_serial();
		safe_transmit_register_reset();
		receive_register_reset();
	}
}

void z80scc_channel::do_sccreg_wr5(uint8_t data)
{
	LOG("%s(%02x) Setting up the transmitter\n", FUNCNAME, data);
	if (data == m_wr5)
	{
		logerror("- suppressing reinit of Tx as write to wr5 is identical to previous value\n");
	}
	else
	{
		m_wr5 = data;
		LOG("- Transmitter Enable %u\n", (data & WR5_TX_ENABLE) ? 1 : 0);
		LOG("- Transmitter Bits/Character %u\n", get_tx_word_length());
		LOG("- Send Break %u\n", (data & WR5_SEND_BREAK) ? 1 : 0);
		LOG("- Request to Send %u\n", (data & WR5_RTS) ? 1 : 0);
		LOG("- Data Terminal Ready %u\n", (data & WR5_DTR) ? 1 : 0);
		update_serial();
		safe_transmit_register_reset();
		update_rts(); // Will also update DTR accordingly
	}
}

void z80scc_channel::do_sccreg_wr6(uint8_t data)
{
	LOG("%s(%02x) Transmit sync\n", FUNCNAME, data);
	m_sync_pattern = (m_sync_pattern & 0xff00) | data;
}

void z80scc_channel::do_sccreg_wr7(uint8_t data)
{
	LOG("%s(%02x) Receive sync\n", FUNCNAME, data);
	m_sync_pattern = (data << 8) | (m_sync_pattern & 0xff);
}

/* WR8 is the transmit buffer register */
void z80scc_channel::do_sccreg_wr8(uint8_t data)
{
	LOG("%s(%02x) \"%s\": %c : Transmit Buffer write %02x\n", FUNCNAME, data, owner()->tag(), 'A' + m_index, data);
	data_write(data);
}

/*WR9 is the Master Interrupt Control register and contains the Reset command bits. Only one WR9
exists in the SCC and is accessed from either channel. The Interrupt control bits are programmed
at the same time as the Reset command, because these bits are only reset by a hardware reset
note that the Z80X30 contains only one WR2 and WR9; these registers may be written from either channel.*/
void z80scc_channel::do_sccreg_wr9(uint8_t data)
{
	m_uart->m_wr9 = data; // There is only one WR9 register and is written from both channels A and B (if exists)

	switch (data & WR9_CMD_MASK)
	{
	case WR9_CMD_NORESET:
		LOG("\"%s\": %c : Master Interrupt Control - No reset  %02x\n", owner()->tag(), 'A' + m_index, data);
		break;
	case WR9_CMD_CHNB_RESET:
		LOGINT("\"%s\": %c : Master Interrupt Control - Channel B reset  %02x\n", owner()->tag(), 'A' + m_index, data);
		m_uart->m_chanB->reset();
		break;
	case WR9_CMD_CHNA_RESET:
		LOGINT("\"%s\": %c : Master Interrupt Control - Channel A reset  %02x\n", owner()->tag(), 'A' + m_index, data);
		m_uart->m_chanA->reset();
		break;
	case WR9_CMD_HW_RESET:
		LOGINT("\"%s\": %c : Master Interrupt Control - Device reset  %02x\n", owner()->tag(), 'A' + m_index, data);
		/*"The effects of this command are identical to those of a hardware reset, except that the Shift Right/Shift Left bit is
		  not changed and the MIE, Status High/Status Low and DLC bits take the programmed values that accompany this command."
		*/
		if (m_uart->m_variant & z80scc_device::SET_Z80X30)
		{
			uint8_t tmp_wr0 = m_wr0; // Save the Shift Left/Shift Right bits
			m_uart->device_reset();
			// Restore the Shift Left/Shift Right bits
			m_wr0 &= 0xfc;
			m_wr0 |= (tmp_wr0 & 0x03);
		}
		else
		{
			m_uart->device_reset();
		}
		// Set the MIE, Status High/Status Low and DLC bits as given in this command
		m_uart->m_wr9 &=        ~(WR9_BIT_MIE | WR9_BIT_SHSL | WR9_BIT_DLC );
		m_uart->m_wr9 |= (data & (WR9_BIT_MIE | WR9_BIT_SHSL | WR9_BIT_DLC ));
		break;
	default:
		logerror("Code is broken in WR9, please report!\n");
	}
}

/* WR10 contains miscellaneous control bits for both the receiver and the transmitter.
   On the ESCC and 85C30 with the Extended Read option enabled, this register may be read as RR11.*/
void z80scc_channel::do_sccreg_wr10(uint8_t data)
{
	m_wr10 = data;
	LOG("\"%s\": %c : %s Misc Tx/Rx Control %02x - not implemented \n", owner()->tag(), 'A' + m_index, FUNCNAME, data);
	LOG("- 6/8 bit sync %d\n", data & WR10_8_6_BIT_SYNC ? 1 : 0);
	LOG("- Loop Mode %d\n", data & WR10_LOOP_MODE ? 1 : 0);
	LOG("- Abort/Flag on underrun %d\n", data & WR10_ABORT_FLAG_UNDERRUN ? 1 : 0);
	LOG("- Mark/Flag Idle line %d\n", data & WR10_MARK_FLAG_IDLE ? 1 : 0);
	LOG("- Go active on poll %d\n", data & WR10_GO_ACTIVE_ON_POLL ? 1 : 0);
	LOG("- Encoding %s\n", data & WR10_BIT6 ?
		 (data & WR10_BIT5 ? "FM0"  : "FM1") :
		 (data & WR10_BIT5 ? "NRZI" : "NRZ"));
	LOG("- CRC Preset %d\n", data & WR10_CRC_PRESET ? 1 : 0);
}

/* WR11 is the Clock Mode Control register. The bits in this register control the sources of both the
receive and transmit clocks, the type of signal on the /SYNC and /RTxC pins, and the direction of
the /TRxC pin.*/
void z80scc_channel::do_sccreg_wr11(uint8_t data)
{
	LOG("\"%s\": %c : %s Clock Mode Control %02x\n", owner()->tag(), 'A' + m_index, FUNCNAME, data);
	m_wr11 = data;
	/*Bit 7: This bit controls the type of input signal the SCC expects to see on the /RTxC pin. If this bit is set
	  to 0, the SCC expects a TTL-compatible signal as an input to this pin. If this bit is set to 1, the SCC
	  connects a high-gain amplifier between the /RTxC and /SYNC pins in expectation of a quartz
	  crystal being placed across the pins.
	  The output of this oscillator is available for use as a clocking source. In this mode of operation, the
	  /SYNC pin is unavailable for other use. The /SYNC signal is forced to zero internally. A hardware
	  reset forces /NO XTAL. (At least 20 ms should be allowed after this bit is set to allow the oscillator
	  to stabilize.)*/
	LOG("- Clock type %s\n", data & WR11_RCVCLK_TYPE ? "Crystal oscillator between RTxC and /SYNC pins" : "TTL level on RTxC pin and /SYNC can be used");
	/*Bits 6 and 5: Receiver Clock select bits 1 and 0
	  These bits determine the source of the receive clock as listed below. They do not
	  interfere with any of the modes of operation in the SCC, but simply control a multiplexer just
	  before the internal receive clock input. A hardware reset forces the receive clock to come from the
	  /RTxC pin.*/
	LOG("- Receive clock source is: ");
	switch (data & WR11_RCVCLK_SRC_MASK)
	{
	case WR11_RCVCLK_SRC_RTXC: LOG("RTxC - not implemented\n"); break;
	case WR11_RCVCLK_SRC_TRXC: LOG("TRxC - not implemented\n"); break;
	case WR11_RCVCLK_SRC_BR:   LOG("Baudrate Generator\n"); break;
	case WR11_RCVCLK_SRC_DPLL: LOG("DPLL - not implemented\n"); break;
	default: logerror("Wrong!\n");/* Will not happen unless someone messes with the mask */
	}
	/*Bits 4 and 3: Transmit Clock select bits 1 and 0.
	  These bits determine the source of the transmit clock as listed in Table . They do not interfere with
	  any of the modes of operation of the SCC, but simply control a multiplexer just before the internal
	  transmit clock input. The DPLL output that is used to feed the transmitter in FM modes lags by 90
	  degrees the output of the DPLL used by the receiver. This makes the received and transmitted bit
	  cells occur simultaneously, neglecting delays. A hardware reset selects the /TRxC pin as the
	  source of the transmit clocks.*/
	LOG("- Transmit clock source is: ");
	switch (data & WR11_TRACLK_SRC_MASK)
	{
	case WR11_TRACLK_SRC_RTXC: LOG("RTxC - not implemented\n"); break;
	case WR11_TRACLK_SRC_TRXC: LOG("TRxC - not implemented\n"); break;
	case WR11_TRACLK_SRC_BR:   LOG("Baudrate Generator\n"); break;
	case WR11_TRACLK_SRC_DPLL: LOG("DPLL - not implemented\n"); break;
	default: logerror("Wrong!\n");/* Will not happen unless someone messes with the mask */
	}
	/* Bit 2: TRxC Pin I/O control bit
	   This bit determines the direction of the /TRxC pin. If this bit is set to 1, the /TRxC pin is an output
	   and carries the signal selected by D1 and D0 of this register. However, if either the receive or the
	   transmit clock is programmed to come from the /TRxC pin, /TRxC is an input, regardless of the
	   state of this bit. The /TRxC pin is also an input if this bit is set to 0. A hardware reset forces this bit
	   to 0.*/
	LOG("- TRxC pin is %s\n", data & WR11_TRXC_DIRECTION ? "Output" : "Input");
	/*Bits 1 and 0: /TRxC Output Source select bits 1 and 0
	  These bits determine the signal to be echoed out of the SCC via the /TRxC pin as listed in Table
	  on page 167. No signal is produced if /TRxC has been programmed as the source of either the
	  receive or the transmit clock. If /TRxC O/I (bit 2) is set to 0, these bits are ignored.
	  If the XTAL oscillator output is programmed to be echoed, and the XTAL oscillator is not enabled,
	  the /TRxC pin goes High. The DPLL signal that is echoed is the DPLL signal used by the receiver.
	  Hardware reset selects the XTAL oscillator as the output source*/
	if (data & WR11_TRXC_DIRECTION)
	{
	LOG("- TRxC pin output is: ");
		switch (data & WR11_TRXSRC_SRC_MASK)
		{
		case WR11_TRXSRC_SRC_XTAL: LOG("the Oscillator - not implemented\n"); break;
		case WR11_TRXSRC_SRC_TRA:  LOG("Transmit clock - not implemented\n"); break;
		case WR11_TRXSRC_SRC_BR:   LOG("Baudrate Generator\n"); break;
		case WR11_TRXSRC_SRC_DPLL: LOG("DPLL - not implemented\n"); break;
		default: logerror("Wrong!\n");/* Will not happen unless someone messes with the mask */
		}
	}
}

/*WR12 contains the lower byte of the time constant for the baud rate generator. The time constant
  can be changed at any time, but the new value does not take effect until the next time the time constant
  is loaded into the down counter. No attempt is made to synchronize the loading of the time
  constant into WR12 and WR13 with the clock driving the down counter. For this reason, it is
  advisable to disable the baud rate generator while the new time constant is loaded into WR12 and
  WR13. Ordinarily, this is done anyway to prevent a load of the down counter between the writing
  of the upper and lower bytes of the time constant.
  The formula for determining the appropriate time constant for a given baud is shown below, with
  the desired rate in bits per second and the BR clock period in seconds. This formula is derived
  because the counter decrements from N down to zero-plus-one-cycle for reloading the time constant.
  This is then fed to a toggle flip-flop to make the output a square wave.

  Time Constant = Clock Frequency / (2 * Desired Rate * Baud Rate Clock Period) - 2

*/
void z80scc_channel::do_sccreg_wr12(uint8_t data)
{
	m_wr12 = data;
	update_serial();
	LOG("\"%s\": %c : %s  %02x Low byte of Time Constant for Baudrate generator\n", owner()->tag(), 'A' + m_index, FUNCNAME, data);
}

/* WR13 contains the upper byte of the time constant for the baud rate generator. */
void z80scc_channel::do_sccreg_wr13(uint8_t data)
{
	m_wr13 = data;
	update_serial();
	LOG("\"%s\": %c : %s  %02x  High byte of Time Constant for Baudrate generator\n", owner()->tag(), 'A' + m_index, FUNCNAME, data);
}

/*
 WR14 contains some miscellaneous control bits */
void z80scc_channel::do_sccreg_wr14(uint8_t data)
{
	switch (data & WR14_DPLL_CMD_MASK)
	{
	case WR14_CMD_NULL:
		LOG("\"%s\": %c : %s  Misc Control Bits Null Command %02x\n", owner()->tag(), 'A' + m_index, FUNCNAME, data);
		break;
	case WR14_CMD_ESM:
/* Issuing this command causes the DPLL to enter the Search mode, where the DPLL searches for a locking edge in the
   incoming data stream. The action taken by the DPLL upon receipt of this command depends on the operating mode of
   the DPLL. In NRZI mode, the output of the DPLL is High while the DPLL is waiting for an edge in the incoming data
   stream. After the Search mode is entered, the first edge the DPLL sees is assumed to be a valid data edge, and
   the DPLL begins the clock recovery operation from that point. The DPLL clock rate must be 32x the data rate in
   NRZI mode. Upon leaving the Search mode, the first sampling edge of the DPLL occurs 16 of these 32x clocks after
   the first data edge, and the second sampling occurs 48 of these 32x clocks after the first data edge. Beyond
   this point, the DPLL begins normal operation, adjusting the output to remain in sync with the incoming data.
   In FM mode, the output of the DPLL is Low while the DPLL is waiting for an edge in the incoming data stream.
   The first edge the DPLL detects is assumed to be a valid clock edge. For this to be the case, the line must
   contain only clock edges; i.e. with FM1 encoding, the line must be continuous 0s. With FM0 encoding the line must
   be continuous 1s, whereas Manchester encoding requires alternating 1s and 0s on the line. The DPLL clock rate must
   be 16 times the data rate in FM mode. The DPLL output causes the receiver to sample the data stream in the nominal
   center of the two halves of the bit to decide whether the data was a 1 or a 0. After this command is issued, as in
   NRZI mode, the DPLL starts sampling immediately after the first edge is detected. (In FM mode, the DPLL examines
   the clock edge of every other bit to decide what correction must be made to remain in sync.) If the DPLL does not
   see an edge during the expected window, the one clock missing bit in RR10 is set. If the DPLL does not see an edge
   after two successive attempts, the two clocks missing bits in RR10 are set and the DPLL automatically enters the
   Search mode. This command resets both clocks missing latches.*/
		LOG("\"%s\": %c : %s  Misc Control Bits Enter Search Mode Command - not implemented\n", owner()->tag(), 'A' + m_index, FUNCNAME);
		break;
	case WR14_CMD_RMC:
		/* Issuing this command disables the DPLL, resets the clock missing latches in RR10, and forces a continuous Search mode state */
		LOG("\"%s\": %c : %s  Misc Control Bits Reset Missing Clocks Command - not implemented\n", owner()->tag(), 'A' + m_index, FUNCNAME);
		break;
	case WR14_CMD_DISABLE_DPLL:
		/* Issuing this command disables the DPLL, resets the clock missing latches in RR10, and forces a continuous Search mode state.*/
		LOG("\"%s\": %c : %s  Misc Control Bits Disable DPLL Command - not implemented\n", owner()->tag(), 'A' + m_index, FUNCNAME);
		break;
	case WR14_CMD_SS_BRG:
		/* Issuing this command forces the clock for the DPLL to come from the output of the BRG. */
		LOG("\"%s\": %c : %s  Misc Control Bits Baudrate Generator Input DPLL Command - not implemented\n", owner()->tag(), 'A' + m_index, FUNCNAME);
		break;
	case WR14_CMD_SS_RTXC:
		/* Issuing the command forces the clock for the DPLL to come from the /RTxC pin or the crystal oscillator, depending on
		   the state of the XTAL/no XTAL bit in WR11. This mode is selected by a channel or hardware reset*/
		LOG("\"%s\": %c : %s  Misc Control Bits RTxC Input DPLL Command - not implemented\n", owner()->tag(), 'A' + m_index, FUNCNAME);
		break;
	case WR14_CMD_SET_FM:
		/* This command forces the DPLL to operate in the FM mode and is used to recover the clock from FM or Manchester-Encoded
		   data. (Manchester is decoded by placing the receiver in NRZ mode while the DPLL is in FM mode.)*/
		LOG("\"%s\": %c : %s  Misc Control Bits Set FM Mode Command - not implemented\n", owner()->tag(), 'A' + m_index, FUNCNAME);
		break;
	case WR14_CMD_SET_NRZI:
		/* Issuing this command forces the DPLL to operate in the NRZI mode. This mode is also selected by a hardware or channel reset.*/
		LOG("\"%s\": %c : %s  Mics Control Bits Set NRZI Mode Command - not implemented\n", owner()->tag(), 'A' + m_index, FUNCNAME);
		break;
	default:
		logerror("\"%s\": %c : %s Mics Control Bits command %02x - not implemented \n", owner()->tag(), 'A' + m_index, FUNCNAME, data);
	}
	/* Based on baudrate code from 8530scc.cpp */
	if ( !(m_wr14 & WR14_BRG_ENABLE) && (data & WR14_BRG_ENABLE) ) // baud rate generator being enabled?
	{
		LOG("\"%s\": %c : %s Mics Control Bits Baudrate generator enabled with ", owner()->tag(), 'A' + m_index, FUNCNAME);
		if (data & WR14_BRG_SOURCE) // Do we use the PCLK as baudrate source
		{
			LOG("   - PCLK as source\n");

#if Z80SCC_USE_LOCAL_BRG
			baudtimer->adjust(attotime::from_hz(rate), TIMER_ID_BAUD, attotime::from_hz(rate)); // Start the baudrate generator
#if START_BIT_HUNT
			m_rcv_mode = RCV_SEEKING;
#endif
#endif
		}
		else
		{
			LOG("external clock source\n");
		}
	}
	else if ( (m_wr14 & WR14_BRG_ENABLE) && !(data & WR14_BRG_ENABLE) ) // baud rate generator being disabled?
	{
#if Z80SCC_USE_LOCAL_BRG
		baudtimer->adjust(attotime::never, TIMER_ID_BAUD, attotime::never); // Stop the baudrate generator
		m_brg_counter = 0;
#endif
	}
	// TODO: Add info on the other bits of this register
	m_wr14 = data;
	update_serial();
}

/* WR15 is the External/Status Source Control register. If the External/Status interrupts are enabled
   as a group via WR1, bits in this register control which External/Status conditions cause an interrupt.
   Only the External/Status conditions that occur after the controlling bit is set to 1 cause an
   interrupt. This is true, even if an External/Status condition is pending at the time the bit is set*/
#define WR15EN "enabled"
#define WR15NO "not implemented"
void z80scc_channel::do_sccreg_wr15(uint8_t data)
{
	LOG("%s(%02x) \"%s\": %c : External/Status Control Bits\n",
			FUNCNAME, data, owner()->tag(), 'A' + m_index);
	LOG("WR7 prime ints     : %s\n", data & WR15_WR7PRIME    ? WR15NO : "disabled");
	LOG("Zero count ints    : %s\n", data & WR15_ZEROCOUNT   ? WR15NO : "disabled");
	LOG("14 bit Status FIFO : %s\n", data & WR15_STATUS_FIFO ? WR15NO : "disabled");
	LOG("DCD ints           : %s\n", data & WR15_DCD         ? WR15EN : "disabled");
	LOG("SYNC/Hunt ints     : %s\n", data & WR15_SYNC        ? WR15NO : "disabled");
	LOG("CTS ints           : %s\n", data & WR15_CTS         ? WR15EN : "disabled");
	LOG("Tx underr./EOM ints: %s\n", data & WR15_TX_EOM      ? WR15NO : "disabled");
	LOG("Break/Abort ints   : %s\n", data & WR15_BREAK_ABORT ? WR15NO : "disabled");
	m_wr15 = data;
}

void z80scc_channel::scc_register_write(uint8_t reg, uint8_t data)
{
	switch (reg)
	{
	case REG_WR0_COMMAND_REGPT:     do_sccreg_wr0(data); break;
	case REG_WR1_INT_DMA_ENABLE:    do_sccreg_wr1(data); m_uart->check_interrupts(); break;
	case REG_WR2_INT_VECTOR:        do_sccreg_wr2(data); break;
	case REG_WR3_RX_CONTROL:        do_sccreg_wr3(data); break;
	case REG_WR4_RX_TX_MODES:       do_sccreg_wr4(data); break;
	case REG_WR5_TX_CONTROL:        do_sccreg_wr5(data); break;
	case REG_WR6_SYNC_OR_SDLC_A:    do_sccreg_wr6(data); break;
	case REG_WR7_SYNC_OR_SDLC_F:    do_sccreg_wr7(data); break;
	case REG_WR8_TRANSMIT_DATA:     do_sccreg_wr8(data); break;
	case REG_WR9_MASTER_INT_CTRL:   do_sccreg_wr9(data); break;
	case REG_WR10_MSC_RX_TX_CTRL:   do_sccreg_wr10(data); break;
	case REG_WR11_CLOCK_MODES:      do_sccreg_wr11(data); break;
	case REG_WR12_LO_BAUD_GEN:      do_sccreg_wr12(data); break;
	case REG_WR13_HI_BAUD_GEN:      do_sccreg_wr13(data); break;
	case REG_WR14_MISC_CTRL:        do_sccreg_wr14(data); break;
	case REG_WR15_EXT_ST_INT_CTRL:  do_sccreg_wr15(data); break;
	default:
		logerror("\"%s\": %c : Unsupported WRx register:%02x\n", owner()->tag(), 'A' + m_index, reg);
	}
}

/*Z85X30 Register Access
 -----------------------
The registers in the Z85X30 are accessed in a two step process, using a Register Pointer to perform
the addressing. To access a particular register, the pointer bits are set by writing to WR0. The
pointer bits may be written in either channel because only one set exists in the Z85X30. After the
pointer bits are set, the next read or write cycle of the Z85X30 having D//C Low will access the
desired register. At the conclusion of this read or write cycle the pointer bits are reset to 0s, so that
the next control write is to the pointers in WR0.

A read to RR8 (the receive data FIFO) or a write to WR8 (the transmit data FIFO) is either done in
this fashion or by accessing the Z85X30 having D//C pin High. A read or write with D//C High
accesses the data registers directly, and independently of the state of the pointer bits. This allows
single-cycle access to the data registers and does not disturb the pointer bits.

The fact that the pointer bits are reset to 0, unless explicitly set otherwise, means that WR0 and
RR0 may also be accessed in a single cycle. That is, it is not necessary to write the pointer bits
with 0 before accessing WR0 or RR0.*/
//-------------------------------------------------
//  control_write - write control register
//-------------------------------------------------

void z80scc_channel::control_write(uint8_t data)
{
	uint8_t reg = m_uart->m_wr0_ptrbits; //m_wr0;
	uint8_t regmask = (WR0_REGISTER_MASK | (m_uart->m_wr0_ptrbits & WR0_POINT_HIGH));

	m_uart->m_wr0_ptrbits = 0; // The "Point High" command is only valid for one access

	reg &= regmask;

	if (reg != 0)
	{
		// mask out register index
		m_wr0 &= ~regmask;
	}

	LOGSETUP(" * %s %c Reg %02x <- %02x - %s\n", owner()->tag(), 'A' + m_index, reg, data, std::array<char const *, 16>
			 {{ "Command register",                 "Tx/Rx Interrupt and Data Transfer Modes",  "Interrupt Vector",                     "Rx Parameters and Control",
				"Tx/Rx Misc Parameters and Modes",  "Tx Parameters and Controls",               "Sync Characters or SDLC Address Field","Sync Character or SDLC Flag/Prime",
				"Tx Buffer",                        "Master Interrupt Control",                 "Miscellaneous Tx/Rx Control Bits",     "Clock Mode Control",
				"Lower Byte of BRG Time Constant",  "Upper Byte of BRg Time Constant",          "Miscellaneous Control Bits",           "External/Status Interrupt Control"}}[reg]);

	scc_register_write(reg, data);
}


//-------------------------------------------------
//  data_read - read data register from fifo
//-------------------------------------------------

uint8_t z80scc_channel::data_read()
{
	uint8_t data = 0;

	LOGRCV("%s \"%s\": %c : Data Register Read: ", FUNCNAME, owner()->tag(), 'A' + m_index);

	if (m_rx_fifo_wp != m_rx_fifo_rp)
	{
		/* Special Receive Condition interrupts are generated after the character is read from
		   the FIFO, not when the special condition is first detected. This is done so that when
		   using receive interrupt on first or Special Condition or Special Condition Only, data is
		   directly read out of the data FIFO without checking the status first. If a special condi-
		   tion interrupted the CPU when first detected, it would be necessary to read RR1
		   before each byte in the FIFO to determine which byte had the special condition.
		   Therefore, by not generating the interrupt until after the byte has been read and then
		   locking the FIFO, only one status read is necessary. A DMA can be used to do all data
		   transfers (otherwise, it would be necessary to disable the DMA to allow the CPU to
		   read the status on each byte). Consequently, since the special condition locks the
		   FIFO to preserve the status, it is necessary to issue the Error Reset command to
		   unlock it. Only the exit location of the FIFO is locked allowing more data to be
		   received into the other bytes of the Receive FIFO.*/

		// load data from the FIFO
		data = m_rx_data_fifo[m_rx_fifo_rp];

		// load error status from the FIFO
		m_rr1 = (m_rr1 & ~(RR1_CRC_FRAMING_ERROR | RR1_RX_OVERRUN_ERROR | RR1_PARITY_ERROR)) | m_rx_error_fifo[m_rx_fifo_rp];

		// trigger interrupt and lock the fifo if an error is present
		if (m_rr1 & (RR1_CRC_FRAMING_ERROR | RR1_RX_OVERRUN_ERROR | ((m_wr1 & WR1_PARITY_IS_SPEC_COND) ? RR1_PARITY_ERROR : 0)))
		{
			logerror("Rx Error %02x\n", m_rr1 & (RR1_CRC_FRAMING_ERROR | RR1_RX_OVERRUN_ERROR | RR1_PARITY_ERROR));
			m_uart->trigger_interrupt(m_index, INT_SPECIAL);
		}
		else
		{
			// decrease RX FIFO pointer
			m_rx_fifo_rp_step();

			// if RX FIFO empty reset RX interrupt status
			if (m_rx_fifo_wp == m_rx_fifo_rp)
			{
				LOGRCV("Rx FIFO empty, resetting status and interrupt state");
				m_uart->m_int_state[INT_RECEIVE_PRIO + (m_index == z80scc_device::CHANNEL_A ? 0 : 3 )] = 0;
				m_uart->m_chanA->m_rr3 &=  ~((1 << INT_RECEIVE_PRIO) + (m_index == z80scc_device::CHANNEL_A ? 3 : 0 ));
			}
		}
	}
	else
	{
		LOG("data_read: Attempt to read out character from empty FIFO\n");
		logerror("data_read: Attempt to read out character from empty FIFO\n");
	}

	LOG("  '%c' %02x\n", isascii(data) ? data : ' ', data);
	return data;
}

/* Step read pointer */
void z80scc_channel::m_rx_fifo_rp_step()
{
		m_rx_fifo_rp++;
		if (m_rx_fifo_rp >= m_rx_fifo_sz)
		{
				m_rx_fifo_rp = 0;
		}

		// check if RX FIFO is empty
		if (m_rx_fifo_rp == m_rx_fifo_wp)
		{
				// no more characters available in the FIFO
				m_rr0 &= ~ RR0_RX_CHAR_AVAILABLE;
		}
}

/* Step TX read pointer */
void z80scc_channel::m_tx_fifo_rp_step()
{
		m_tx_fifo_rp++;
		if (m_tx_fifo_rp >= m_tx_fifo_sz)
		{
				m_tx_fifo_rp = 0;
		}
}

READ8_MEMBER (z80scc_device::da_r)  { return m_chanA->data_read(); }
WRITE8_MEMBER (z80scc_device::da_w) { m_chanA->data_write(data); }
READ8_MEMBER (z80scc_device::db_r)  { return m_chanB->data_read(); }
WRITE8_MEMBER (z80scc_device::db_w) { m_chanB->data_write(data); }

//-------------------------------------------------
//  data_write - write data register
//-------------------------------------------------
void z80scc_channel::data_write(uint8_t data)
{
	/* Tx FIFO is full or...? */
	LOGTX("%s \"%s\": %c : Data Register Write: %02d '%c'\n", FUNCNAME, owner()->tag(), 'A' + m_index, data, isprint(data) ? data : ' ');

	if ( !(m_rr0 & RR0_TX_BUFFER_EMPTY) && // NMOS/CMOS 1 slot "FIFO" is controlled by the TBE bit instead of fifo logic
		( (m_tx_fifo_wp + 1 == m_tx_fifo_rp) || ( (m_tx_fifo_wp + 1 == m_tx_fifo_sz) && (m_tx_fifo_rp == 0) )))
	{
		logerror("- TX FIFO is full, discarding data\n");
		LOGTX("- TX FIFO is full, discarding data\n");
	}
	else // ..there is still room
	{
		LOGTX("- TX FIFO has room\n");
		m_tx_data_fifo[m_tx_fifo_wp++] = data;
		if (m_tx_fifo_wp >= m_tx_fifo_sz)
		{
			m_tx_fifo_wp = 0;
		}

		// Check FIFO fullness and set TBE bit accordingly
		if (m_tx_fifo_sz == 1)
		{
			LOGTX("- TX FIFO has only one slot so is now completelly filled, clearing TBE bit\n");
			m_rr0 &= ~RR0_TX_BUFFER_EMPTY; // If only one FIFO position it is full now!
		}
		else if (m_tx_fifo_wp + 1 == m_tx_fifo_rp || ( (m_tx_fifo_wp + 1 == m_tx_fifo_sz) && (m_tx_fifo_rp == 0) ))
		{
			LOGTX("- TX FIFO has filled all slots so now completelly filled, clearing TBE bit\n");
			m_rr0 &= ~RR0_TX_BUFFER_EMPTY; // Indicate that the TX fifo is full
		}
		else
		{
			LOGTX("- TX FIFO has more room, setting TBE bit\n");
			m_rr0 |= RR0_TX_BUFFER_EMPTY; // or there is a slot in the FIFO available
		}
		LOGTX("- TX FIFO now has data to send, clearing ALL_SENT bit\n");
		m_rr1 &= ~RR1_ALL_SENT; // All is definitelly not sent anymore
	}

	check_waitrequest();

	/* Transmitter enabled?  */
	if (m_wr5 & WR5_TX_ENABLE)
	{
		LOGTX("- TX is enabled\n");
		if (is_transmit_register_empty()) // Is the shift register loaded?
		{
			LOGTX("- Setting up transmitter\n");
			transmit_register_setup(m_tx_data_fifo[m_tx_fifo_rp]); // Load the shift register, reload is done in tra_complete()
			m_tx_fifo_rp_step();
			m_rr1 |= RR1_ALL_SENT; // Now stuff is on its way again
			m_rr0 |= RR0_TX_BUFFER_EMPTY; // And there is a slot in the FIFO available
		}
		else
		{
			LOGTX("- Transmitter not empty\n");
		}
	}
	else
	{
		LOGTX("- Transmitter disabled\n");
	}
	/* "While transmit interrupts are enabled, the nmos/cmos version sets the transmit interrupt pending
	   (TxIP) bit whenever the transmit buffer becomes empty. this means that the transmit buffer
	   must be full before the TxIP can be set. thus, when transmit interrupts are first enabled, the TxIP
	   will not be set until after the first character is written to the nmos/cmos." */
	// check if to fire interrupt
	LOG("- TX interrupt are %s\n", (m_wr1 & WR1_TX_INT_ENABLE) ? "enabled" : "disabled" );

	if (m_wr1 & WR1_TX_INT_ENABLE)
	{
		if ((m_uart->m_variant & z80scc_device::SET_ESCC) &&
			(m_wr7p & WR7P_TX_FIFO_EMPTY)  &&
			m_tx_fifo_wp == m_tx_fifo_rp)  // ESCC and fifo empty bit set and fifo is completelly empty?
		{
			m_uart->trigger_interrupt(m_index, INT_TRANSMIT); // Set TXIP bit
		}
		else if(m_rr0 & RR0_TX_BUFFER_EMPTY)  // Check TBE bit and interrupt if one or more FIFO slots available
		{
			m_uart->trigger_interrupt(m_index, INT_TRANSMIT); // Set TXIP bit
		}
	}
}


//-------------------------------------------------
//  receive_data - put received data word into fifo
//-------------------------------------------------

void z80scc_channel::receive_data(uint8_t data)
{
	LOGRCV("\"%s\": %c : Received Data Byte '%c'/%02x put into FIFO\n", owner()->tag(), 'A' + m_index, isprint(data) ? data : ' ', data);

	if (m_rx_fifo_wp + 1 == m_rx_fifo_rp || ( (m_rx_fifo_wp + 1 == m_rx_fifo_sz) && (m_rx_fifo_rp == 0) ))
	{
		// receive overrun error detected
		m_rx_error_fifo[m_rx_fifo_wp] |= RR1_RX_OVERRUN_ERROR;

		// store received character but do not step the fifo
		m_rx_data_fifo[m_rx_fifo_wp] = data;

		logerror("Receive_data() Error %02x\n", m_rx_error_fifo[m_rx_fifo_wp] & (RR1_CRC_FRAMING_ERROR | RR1_RX_OVERRUN_ERROR | RR1_PARITY_ERROR));
	}
	else
	{
		m_rx_error_fifo[m_rx_fifo_wp] &= ~RR1_RX_OVERRUN_ERROR;

		// store received character
		m_rx_data_fifo[m_rx_fifo_wp] = data;

		m_rx_fifo_wp++;
		if (m_rx_fifo_wp >= m_rx_fifo_sz)
		{
			m_rx_fifo_wp = 0;
		}
	}

	m_rr0 |= RR0_RX_CHAR_AVAILABLE;

	// receive interrupt on FIRST and ALL character
	switch (m_wr1 & WR1_RX_INT_MODE_MASK)
	{
	case WR1_RX_INT_FIRST:
		if (m_rx_first)
		{
			m_uart->trigger_interrupt(m_index, INT_RECEIVE);

			m_rx_first = 0;
		}
		break;

	case WR1_RX_INT_ALL:
		m_uart->trigger_interrupt(m_index, INT_RECEIVE);
		break;
	}
}


//-------------------------------------------------
//  cts_w - clear to send handler
//-------------------------------------------------

WRITE_LINE_MEMBER( z80scc_channel::cts_w )
{
	LOG("\"%s\" %s: %c : CTS %u\n", owner()->tag(), FUNCNAME, 'A' + m_index, state);

	if ((m_rr0 & RR0_CTS) != (state ? RR0_CTS : 0)) //  SCC change detection logic
	{
		// enable transmitter if in auto enables mode
		if (!state)
		{
			LOGCTS(" - CTS active\n");
			if (m_wr3 & WR3_AUTO_ENABLES)
			{
				LOGCTS(" - TX auto enabled\n");
				m_wr5 |= WR5_TX_ENABLE;
			}
		}

		if (state) m_rr0 |= RR0_CTS; else  m_rr0 &= ~RR0_CTS; // Raw pin/status value

		if (m_extint_latch == 0 && (m_wr1 & WR1_EXT_INT_ENABLE) && (m_wr15 & WR15_CTS))
		{
			// trigger interrupt
			LOGCTS(" - Trigger CTS interrupt\n");
			m_uart->trigger_interrupt(m_index, INT_EXTERNAL);

			// latch read register 0
			LOGCTS(" - Latches RR0\n");
			m_extint_latch = 1;
			m_extint_states = m_rr0;
		}
	}
}


//-------------------------------------------------
//  dcd_w - data carrier detected handler
//-------------------------------------------------
WRITE_LINE_MEMBER( z80scc_channel::dcd_w )
{
	LOGDCD("\"%s\": %c : DCD %u\n", owner()->tag(), 'A' + m_index, state);

	if ((m_rr0 & RR0_DCD) != (state ? RR0_DCD : 0)) //  SCC change detection logic
	{
		// enable transmitter if in auto enables mode
		if (!state)
		{
			LOGDCD(" - DCD active\n");
			if (m_wr3 & WR3_AUTO_ENABLES)
			{
				LOGDCD(" - RX auto enabled\n");
				m_wr3 |= WR3_RX_ENABLE;
#if START_BIT_HUNT
				m_rcv_mode = RCV_SEEKING;
#endif
			}
		}

		if (state) m_rr0 |= RR0_DCD; else  m_rr0 &= ~RR0_DCD; // Raw pin/status value

		if (m_extint_latch == 0 && (m_wr1 & WR1_EXT_INT_ENABLE) && (m_wr15 & WR15_DCD))
		{
			// latch read register 0
			LOGINT(" - Latches RR0\n");
			m_extint_latch = 1;
			m_extint_states = m_rr0;

			// trigger interrupt
			LOGINT(" - Trigger DCD interrupt\n");
			m_uart->trigger_interrupt(m_index, INT_EXTERNAL);
		}
	}
}


//-------------------------------------------------
//  ri_w - ring indicator handler
//-------------------------------------------------
WRITE_LINE_MEMBER( z80scc_channel::ri_w )
{
	LOGINT("\"%s\": %c : RI %u - not implemented\n", owner()->tag(), 'A' + m_index, state);

#if 0 // TODO: This code is inherited from another device driver and not correct for SCC
	if (m_ri != state)
	{
		// set ring indicator state
		m_ri = state;

		if (!m_rx_rr0_latch)
		{
			if (m_ri)
				m_rr0 |= RR0_RI;
			else
				m_rr0 &= ~RR0_RI;

			if (m_wr1 & WR1_EXT_INT_ENABLE)
			{
				// trigger interrupt
				m_uart->trigger_interrupt(m_index, INT_EXTERNAL);

				// latch read register 0
				m_rx_rr0_latch = 1;
			}
		}
	}
#endif
}

//-------------------------------------------------
//  sync_w - sync handler for external sync mode
//-------------------------------------------------
WRITE_LINE_MEMBER( z80scc_channel::sync_w )
{
	LOGSYNC("\"%s\": %c : SYNC %u\n", owner()->tag(), 'A' + m_index, state);
	if ((m_rr0 & RR0_SYNC_HUNT) != (state ? RR0_SYNC_HUNT : 0)) //  SCC change detection logic
	{
		if (state) m_rr0 |= RR0_SYNC_HUNT; else  m_rr0 &= ~RR0_SYNC_HUNT; // Raw pin/status value
	}
}

//-------------------------------------------------
//  rxc_w - receive clock
//-------------------------------------------------
WRITE_LINE_MEMBER( z80scc_channel::rxc_w )
{
/* Support for external clock as source for BRG yet to be finished */
#if 0
	//LOG("\"%s\": %c : Receiver Clock Pulse\n", owner()->tag(), m_index + 'A');
	if ( ((m_wr3 & WR3_RX_ENABLE) | (m_wr5 & WR5_TX_ENABLE)) && m_wr14 & WR14_BRG_ENABLE)
	{
		if (!(m_wr14 & WR14_BRG_SOURCE)) // Is the Baud rate Generator driven by RTxC?
		{
			printf("x");
			if (!m_brg_counter) // Zero crossing?!
			{
				printf(".");
				m_brg_counter =  m_wr13 << 8 | m_wr12; // Reload BRG counter
				if ((m_wr11 & WR11_TRACLK_SRC_MASK) == WR11_TRACLK_SRC_BR) // Is transmitt clock driven by BRG?
				{
					printf("+");
					txc_w(state);
				}
			}
			else
			{
				m_brg_counter--;
				if ((m_wr11 & WR11_RCVCLK_SRC_MASK) == WR11_RCVCLK_SRC_BR) // Is receive clock driven by BRG and not zero cross
					return;
			}
		}
	}
#endif

	if (m_wr3 & WR3_RX_ENABLE)
	{
		int clocks = get_clock_mode();
		if (clocks == 1)
			rx_clock_w(state);
		else if(state)
		{
			if (m_rx_clock == clocks/2 && m_rcv_mode == RCV_SAMPLING)
			rx_clock_w(m_rx_clock < clocks/2);

			m_rx_clock++;
			if (m_rx_clock == clocks)
			{
				m_rx_clock = 0;
			}
		}
	}
}

//-------------------------------------------------
//  txc_w - transmit clock
//-------------------------------------------------
WRITE_LINE_MEMBER( z80scc_channel::txc_w )
{
	//LOG("\"%s\": %c : Transmitter Clock Pulse\n", owner()->tag(), m_index + 'A');
	if (m_wr5 & WR5_TX_ENABLE)
	{
		int clocks = get_clock_mode();
		if (clocks == 1)
			tx_clock_w(state);
		else if(state)
		{
			tx_clock_w(m_tx_clock < clocks/2);

			m_tx_clock++;
			if (m_tx_clock == clocks)
				m_tx_clock = 0;
		}
	}
}

//--------------------------------------------------------------------------------------------------------
//  safe_transmit_register_reset -  wait for the transmitter shift register to be
//   emptied before apply the new value of wr5 and/or wr4. In the case of a Tx FIFO
//   the change will occur before next character is started. From the specification:
//
//   "The character length may be changed on the fly, but the desired length must be selected before the
//   character is loaded into the Transmit Shift register from the transmit data FIFO. The easiest way to
//   ensure this is to write to WR5 to change the character length before writing the data to the transmit
//   buffer."
//
//  Right now we only detect the problem and log an error
//---------------------------------------------------------------------------------------------------------
void z80scc_channel::safe_transmit_register_reset()
{
	if (!is_transmit_register_empty())
	{
		logerror("Attempt to reset transmit shift register while busy detected, please report\n");
	}
	transmit_register_reset();
}

//-------------------------------------------------
// get_brg_rate
//-------------------------------------------------
unsigned int z80scc_channel::get_brg_rate()
{
	unsigned int rate;
	unsigned int brg_const;

	brg_const = 2 + (m_wr13 << 8 | m_wr12);
	if (m_wr14 & WR14_BRG_SOURCE) // Do we use the PCLK as baudrate source
	{
		rate = owner()->clock() / (brg_const == 0 ? 1 : brg_const);
		LOG("   - Source bit rate (%d) = PCLK (%d) / (%d)\n", rate, owner()->clock(), brg_const);
	}
	else // Else we use the RTxC as BRG source
	{
		unsigned int source = (m_index == z80scc_device::CHANNEL_A) ? m_uart->m_rxca : m_uart->m_rxcb;
		rate = source / (brg_const == 0 ? 1 : brg_const);
		LOG("   - Source bit rate (%d) = RTxC (%d) / (%d)\n", rate, source, brg_const);
	}

	return (rate / (2 * get_clock_mode()));
}

//-------------------------------------------------
//  update_serial -
//-------------------------------------------------
void z80scc_channel::update_serial()
{
	int data_bit_count = get_rx_word_length();
	stop_bits_t stop_bits = get_stop_bits();
	parity_t parity;

	if (m_wr4 & WR4_PARITY_ENABLE)
	{
		if (m_wr4 & WR4_PARITY_EVEN)
			parity = PARITY_EVEN;
		else
			parity = PARITY_ODD;
	}
	else
	{
		parity = PARITY_NONE;
	}

	LOG(" %s() \"%s \"Channel %c setting data frame %d+%d%c%d\n", FUNCNAME, owner()->tag(), 'A' + m_index, 1,
		 data_bit_count, parity == PARITY_NONE ? 'N' : parity == PARITY_EVEN ? 'E' : 'O', (stop_bits + 1) / 2);

	set_data_frame(1, data_bit_count, parity, stop_bits);

#if START_BIT_HUNT
	m_rcv_mode = m_wr3 & WR3_RX_ENABLE ? RCV_SEEKING : RCV_IDLE;
#endif

	int clocks = get_clock_mode();

	if  (m_wr14 & WR14_BRG_ENABLE)
	{
		LOG("- BRG enabled\n");
		m_brg_rate = get_brg_rate();

		LOG("- BRG rate %d\n", m_brg_rate);
		set_rcv_rate(m_brg_rate);

		if (is_transmit_register_empty())
		{
			set_tra_rate(m_brg_rate);
			LOGTX("   - Baud Rate Generator: %d clock mode: %dx\n", m_brg_rate, get_clock_mode());
		}
		else
		{
			m_delayed_tx_brg_change = 1;
			LOGTX("   - Baud Rate Generator delay init: %d clock mode: %dx\n", m_brg_rate, get_clock_mode());
		}
	}
	else
	{
		LOG("- BRG disabled\n");
		set_rcv_rate(0);
		set_tra_rate(0);
	}
	// TODO: Check registers for use of RTxC and TRxC, if used as direct Tx and/or Rx clocks set them to value as programmed
	// in m_uart->txca/txcb and rxca/rxcb respectivelly
	if (m_rxc > 0)
	{
		set_rcv_rate(m_rxc / clocks); // TODO Check/Fix this to get the right tx/rx clocks, seems to be missing a divider or two
		LOG("   - Receiver clock: %d mode: %d rate: %d/%xh\n", m_rxc, clocks, m_rxc / clocks, m_rxc / clocks);
	}

	if (m_txc > 0 && !(m_wr14 & WR14_BRG_ENABLE))
	{
		set_tra_rate(m_txc / clocks);
		LOG("   - Transmit clock: %d mode: %d rate: %d/%xh\n", m_rxc, clocks, m_rxc / clocks, m_rxc / clocks);
	}
}

//-------------------------------------------------
//  set_dtr -
//-------------------------------------------------
void z80scc_channel::set_dtr(int state)
{
	LOG("%s(%d)\n", FUNCNAME, state);
	m_dtr = state;

	if (m_index == z80scc_device::CHANNEL_A)
		m_uart->m_out_dtra_cb(m_dtr);
	else
		m_uart->m_out_dtrb_cb(m_dtr);
}

//-------------------------------------------------
//  write_rx - called by terminal through rs232/diserial
//         when character is sent to board
//-------------------------------------------------
WRITE_LINE_MEMBER(z80scc_channel::write_rx)
{
#if START_BIT_HUNT
	// Check for start bit if not receiving
	if (m_rcv_mode == RCV_SEEKING && m_rxd == 1 && state == 0){
		m_rcv_mode = RCV_SAMPLING;
#if START_BIT_ADJUST
		m_rx_clock = 0;
#endif
	}
#endif

	LOGRCV("%s(%d)\n", FUNCNAME, state);
	m_rxd = state;
	//only use rx_w when self-clocked
	if(m_rxc != 0 || m_brg_rate != 0)
		device_serial_interface::rx_w(state);
}

/*
 * This is a partial implementation of the "wait/dma request" functionality of the SCC controlled by
 * bits D7, D6 and D5 in WR1. This implementation is sufficient to support DMA request on transmit
 * used by the InterPro driver.
 *
 * TODO:
 *  - wait function (D6=0)
 *  - wait/request function on receive (D5=1)
 */
void z80scc_channel::check_waitrequest()
{
	// don't do anything if wait/request function is not enabled
	if ((m_wr1 & WR1_WREQ_ENABLE) == 0)
		return;

	// wait/request function for receive not implemented
	if (m_wr1 & WR1_WREQ_ON_RX_TX)
		return;

	// if dma request function is enabled
	if (m_wr1 & WR1_WREQ_FUNCTION)
	{
		// assert /W//REQ if transmit buffer is empty, clear if it's not
		int state = (m_rr0 & RR0_TX_BUFFER_EMPTY) ? ASSERT_LINE : CLEAR_LINE;

		(m_index ? m_uart->m_out_wreqb_cb : m_uart->m_out_wreqa_cb)(state);
	}
}
