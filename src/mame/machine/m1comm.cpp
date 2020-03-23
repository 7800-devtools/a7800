// license:BSD-3-Clause
// copyright-holders:Ariane Fugmann

/*
Comm PCB
--------

MODEL-1 COMMUNICATION BD 837-8842 171-6293B (C) SEGA 1992
|--------------------------------------------------------------------------------|
|                                                                                |
|    MB89237A            MB89374                                                 |
|       JP4                                                                 LED1 |
|    15112.17            Z80                                                     |
|    JP2  JP3                                                       75179        |
|    MB8464              315-5624                                     JP6        |
|                                                       315-5547                 |
|        315-5611                                            SW1    PC910     CN4|
|                                                                                |
|                                                                   PC910     CN5|
|     MB8421             MB8431                                JP7               |
|                                                                   JP5          |
|        JP8                                                                  CN7|
|                CN1                                    CN2                      |
| |---------------------------------|   |---------------------------------|   CN6|
| |---------------------------------|   |---------------------------------|      |
|--------------------------------------------------------------------------------|
Notes:
      15112.17 - AMD AM27C100 128k x8 EPROM (DIP32, labelled 'EPR-15112')
      Z80      - Zilog Z0840008PSC Z80 CPU, running at 8.000MHz (DIP40)
      MB8464   - Fujitsu MB8464 8k x8 SRAM (DIP28)
      MB8421   - Fujitsu MB8421-12LP 2k x8 SRAM (SDIP52)
      MB8431   - Fujitsu MB8431-90LP 2k x8 SRAM (SDIP52)
      MB89237A - Fujitsu MB89237A DMA-Controller (DIP20) [most likely i8237A clone]
      MB89374  - Fujitsu MB89374 Data Link Controller (SDIP42)
      75179    - Texas Instruments SN75179 Differential Driver and Receiver Pair (DIP8)
      315-5547 - AMI 18CV8PC-25 PAL (DIP20)
      315-5624 - MMI PAL16L8BCN PAL (DIP20)
      315-5611 - Lattice GAL16V8A PAL (DIP20)
      PC910    - Sharp PC910 opto-isolator (x2, DIP8)
      SW1      - Push Button Switch (enables board)
      CN1, CN2 - Connectors to join Comm board to Video board
      CN4      - 8 pin connector (DIFFERENTIAL port)
      CN5      - 6 pin connector (SERIAL port)
      CN6, CN7 - TOSLINK-Connectors for network optical cable link
      JP2      - Jumper, set to 2-3 (connected to EPROM A15)
      JP3      - Jumper, set to 1-2 (connected to EPROM A16)
      JP4      - Jumper, set to 1-2
      JP5      - Jumper, shorted (enables TOSLINK RX channel)
      JP6      - Jumper, not shorted (enables DIFFERERENTIAL RX channel)
      JP7      - Jumper, not shorted (enables SERIAL RX channel)
      JP8      - Jumper, set to 1-2 (selects CLOCK SOURCE)
*/

#include "emu.h"
#include "emuopts.h"
#include "machine/m1comm.h"

#define Z80_TAG     "m1commcpu"

#define VERBOSE 0
#include "logmacro.h"

/*************************************
 *  M1COMM Memory Map
 *************************************/
static ADDRESS_MAP_START( m1comm_mem, AS_PROGRAM, 8, m1comm_device )
	AM_RANGE(0x0000, 0x7fff) AM_ROM
	AM_RANGE(0x8000, 0x9fff) AM_RAM
	AM_RANGE(0xc000, 0xffff) AM_READWRITE(share_r, share_w)
ADDRESS_MAP_END

/*************************************
 *  M1COMM I/O Map
 *************************************/
static ADDRESS_MAP_START( m1comm_io, AS_IO, 8, m1comm_device )
	ADDRESS_MAP_GLOBAL_MASK(0xff)
	AM_RANGE(0x00, 0x1f) AM_READWRITE(dlc_reg_r, dlc_reg_w)
	AM_RANGE(0x20, 0x2f) AM_READWRITE(dma_reg_r, dma_reg_w)
	AM_RANGE(0x40, 0x40) AM_READWRITE(syn_r, syn_w)
	AM_RANGE(0x60, 0x60) AM_READWRITE(zfg_r, zfg_w)
	AM_RANGE(0xff, 0xff) AM_RAM
ADDRESS_MAP_END


ROM_START( m1comm )
	ROM_REGION( 0x20000, Z80_TAG, ROMREGION_ERASEFF )
	ROM_LOAD( "epr-15112.17", 0x0000, 0x20000, CRC(4950e771) SHA1(99014124e0324dd114cb22f55159d18b597a155a) )
ROM_END

//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

DEFINE_DEVICE_TYPE(M1COMM, m1comm_device, "m1comm", "Model 1 Communication Board")

//-------------------------------------------------
//  device_add_mconfig - add device configuration
//-------------------------------------------------

MACHINE_CONFIG_MEMBER( m1comm_device::device_add_mconfig )
	MCFG_CPU_ADD(Z80_TAG, Z80, 8000000) /* 32 MHz / 4 */
	MCFG_CPU_PROGRAM_MAP(m1comm_mem)
	MCFG_CPU_IO_MAP(m1comm_io)
MACHINE_CONFIG_END

//-------------------------------------------------
//  rom_region - device-specific ROM region
//-------------------------------------------------

const tiny_rom_entry *m1comm_device::device_rom_region() const
{
	return ROM_NAME( m1comm );
}

//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  m1comm_device - constructor
//-------------------------------------------------

m1comm_device::m1comm_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, M1COMM, tag, owner, clock),
	m_commcpu(*this, Z80_TAG),
	m_line_rx(OPEN_FLAG_WRITE | OPEN_FLAG_CREATE ),
	m_line_tx(OPEN_FLAG_READ)
{
	// prepare localhost "filename"
	m_localhost[0] = 0;
	strcat(m_localhost, "socket.");
	strcat(m_localhost, mconfig.options().comm_localhost());
	strcat(m_localhost, ":");
	strcat(m_localhost, mconfig.options().comm_localport());

	// prepare remotehost "filename"
	m_remotehost[0] = 0;
	strcat(m_remotehost, "socket.");
	strcat(m_remotehost, mconfig.options().comm_remotehost());
	strcat(m_remotehost, ":");
	strcat(m_remotehost, mconfig.options().comm_remoteport());
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void m1comm_device::device_start()
{
}

//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void m1comm_device::device_reset()
{
	m_syn = 0;
	m_zfg = 0;
	m_cn = 0;
	m_fg = 0;

	m_commcpu->set_input_line(INPUT_LINE_RESET, ASSERT_LINE);
}

READ8_MEMBER(m1comm_device::dlc_reg_r)
{
	// dirty hack to keep Z80 in RESET state
	if (!m_cn)
	{
		device_reset();
		return 0xff;
	}
	// dirty hack to keep Z80 in RESET state

	uint8_t result = m_dlc_reg[offset];
	LOG("m1comm-dlc_reg_r: read register %02x for value %02x\n", offset, result);
	return result;
}

WRITE8_MEMBER(m1comm_device::dlc_reg_w)
{
	m_dlc_reg[offset] = data;
	LOG("m1comm-dlc_reg_w: write register %02x for value %02x\n", offset, data);
}

READ8_MEMBER(m1comm_device::dma_reg_r)
{
	uint8_t result = m_dma_reg[offset];
	LOG("m1comm-dma_reg_r: read register %02x for value %02x\n", offset, result);
	return result;
}

WRITE8_MEMBER(m1comm_device::dma_reg_w)
{
	LOG("m1comm-dma_reg_w: %02x %02x\n", offset, data);
	m_dma_reg[offset] = data;
}

READ8_MEMBER(m1comm_device::syn_r)
{
	uint8_t result = m_syn | 0xfc;
	LOG("m1comm-syn_r: read register %02x for value %02x\n", offset, result);
	return result;
}

WRITE8_MEMBER(m1comm_device::syn_w)
{
	m_syn = data & 0x03;

	switch (data & 0x02)
	{
	case 0x00:
		LOG("m1comm-syn_w: VINT disabled\n");
		break;

	case 0x02:
		LOG("m1comm-syn_w: VINT enabled\n");
		break;

	default:
		LOG("m1comm-syn_w: %02x\n", data);
		break;
	}
}

READ8_MEMBER(m1comm_device::zfg_r)
{
	uint8_t result = m_zfg | (~m_fg << 7) | 0x7e;
	LOG("m1comm-zfg_r: read register %02x for value %02x\n", offset, result);
	return result;
}

WRITE8_MEMBER(m1comm_device::zfg_w)
{
	LOG("m1comm-zfg_w: %02x\n", data);
	m_zfg = data & 0x01;
}

READ8_MEMBER(m1comm_device::share_r)
{
	return m_shared[offset];
}

WRITE8_MEMBER(m1comm_device::share_w)
{
	m_shared[offset] = data;
}

READ8_MEMBER(m1comm_device::cn_r)
{
	return m_cn | 0xfe;
}

WRITE8_MEMBER(m1comm_device::cn_w)
{
	m_cn = data & 0x01;

#ifndef M1COMM_SIMULATION
	if (!m_cn)
		device_reset();
	else
		m_commcpu->set_input_line(INPUT_LINE_RESET, CLEAR_LINE);
#else
	if (!m_cn)
	{
		// reset command
		osd_printf_verbose("M1COMM: board disabled\n");
		m_linkenable = 0x00;
		m_zfg = 0x00;
	}
	else
	{
		// init command
		osd_printf_verbose("M1COMM: board enabled\n");
		m_linkenable = 0x01;
		m_linkid = 0x00;
		m_linkalive = 0x00;
		m_linkcount = 0x00;
		m_linktimer = 0x00e8; // 58 fps * 4s
	}
#endif
}

READ8_MEMBER(m1comm_device::fg_r)
{
	return m_fg | (~m_zfg << 7) | 0x7e;
}

WRITE8_MEMBER(m1comm_device::fg_w)
{
	if (!m_cn)
		return;

	m_fg = data & 0x01;
}

void m1comm_device::check_vint_irq()
{
#ifndef M1COMM_SIMULATION
	if (m_syn & 0x02)
	{
		m_commcpu->set_input_line_and_vector(0, HOLD_LINE, 0xef);
		LOG("m1comm-INT5\n");
	}
#else
	comm_tick();
#endif
}

#ifdef M1COMM_SIMULATION
void m1comm_device::comm_tick()
{
	if (m_linkenable == 0x01)
	{
		int frameStart = 0x0010;
		int frameOffset = 0x0000;
		int frameSize = 0x01c4;
		int dataSize = frameSize + 1;
		int togo = 0;
		int recv = 0;
		int idx = 0;

		bool isMaster = (m_shared[1] == 0x01);
		bool isSlave = (m_shared[1] == 0x02);
		bool isRelay = (m_shared[1] == 0x00);

		// if link not yet established...
		if (m_linkalive == 0x00)
		{
			// waiting...
			m_shared[0] = 0x05;

			// check rx socket
			if (!m_line_rx.is_open())
			{
				osd_printf_verbose("M1COMM: listen on %s\n", m_localhost);
				m_line_rx.open(m_localhost);
			}

			// check tx socket
			if (!m_line_tx.is_open())
			{
				osd_printf_verbose("M1COMM: connect to %s\n", m_remotehost);
				m_line_tx.open(m_remotehost);
			}

			// if both sockets are there check ring
			if ((m_line_rx.is_open()) && (m_line_tx.is_open()))
			{
				// try to read one messages
				recv = m_line_rx.read(m_buffer, dataSize);
				while (recv != 0)
				{
					// check if complete message
					if (recv == dataSize)
					{
						// check if message id
						idx = m_buffer[0];

						// 0xFF - link id
						if (idx == 0xff)
						{
							if (isMaster)
							{
								// master gets first id and starts next state
								m_linkid = 0x01;
								m_linkcount = m_buffer[1];
								m_linktimer = 0x01;
							}
							else if (isSlave || isRelay)
							{
								// slave gets own id
								if (isSlave)
								{
									m_buffer[1]++;
									m_linkid = m_buffer[1];
								}

								// slave and relay forward message
								m_line_tx.write(m_buffer, dataSize);
							}
						}

						// 0xFE - link size
						else if (idx == 0xfe)
						{
							if (isSlave || isRelay)
							{
								m_linkcount = m_buffer[1];

								// slave and relay forward message
								m_line_tx.write(m_buffer, dataSize);
							}

							// consider it done
							osd_printf_verbose("M1COMM: link established - id %02x of %02x\n", m_linkid, m_linkcount);
							m_linkalive = 0x01;
							m_zfg = 0x01;

							// write to shared mem
							m_shared[0] = 0x01;
							m_shared[2] = m_linkid;
							m_shared[3] = m_linkcount;
						}
					}
					else
					{
						// got only part of a message - read the rest (and drop it)
						// TODO: combine parts and push to "ring buffer"
						togo = dataSize - recv;
						while (togo > 0){
							recv = m_line_rx.read(m_buffer, togo);
							togo -= recv;
						}
						osd_printf_verbose("M1COMM: dropped a message...\n");
					}

					if (m_linkalive == 0x00)
						recv = m_line_rx.read(m_buffer, dataSize);
					else
						recv = 0;
				}

				// if we are master and link is not yet established
				if (isMaster && (m_linkalive == 0x00))
				{
					// send first packet
					if (m_linktimer == 0x00)
					{
						m_buffer[0] = 0xff;
						m_buffer[1] = 0x01;
						m_line_tx.write(m_buffer, dataSize);
					}

					// send second packet
					else if (m_linktimer == 0x01)
					{
						m_buffer[0] = 0xfe;
						m_buffer[1] = m_linkcount;
						m_line_tx.write(m_buffer, dataSize);

						// consider it done
						osd_printf_verbose("M1COMM: link established - id %02x of %02x\n", m_linkid, m_linkcount);
						m_linkalive = 0x01;
						m_zfg = 0x01;

						// write to shared mem
						m_shared[0] = 0x01;
						m_shared[2] = m_linkid;
						m_shared[3] = m_linkcount;
					}

					else if (m_linktimer > 0x02)
					{
						// decrease delay timer
						m_linktimer--;
						if (m_linktimer == 0x02)
							m_linktimer = 0x00;
					}
				}
			}
		}

		// update "ring buffer" if link established
		if (m_linkalive == 0x01)
		{
			int togo = 0;
			// try to read one messages
			int recv = m_line_rx.read(m_buffer, dataSize);
			while (recv != 0)
			{
				// check if complete message
				if (recv == dataSize)
				{
					// check if valid id
					int idx = m_buffer[0];
					if (idx > 0 && idx <= m_linkcount) {
						// if not our own message
						if (idx != m_linkid)
						{
							// save message to "ring buffer"
							frameOffset = frameStart + (idx * frameSize);
							for (int j = 0x00 ; j < frameSize ; j++)
							{
								m_shared[frameOffset + j] = m_buffer[1 + j];
							}

							// forward message to other nodes
							m_line_tx.write(m_buffer, dataSize);
						}
					} else {
						if (!isMaster && idx == 0xf0){
							// 0xF0 - master addional bytes
							for (int j = 0x06 ; j < 0x10 ; j++)
							{
								m_shared[j] = m_buffer[1 + j];
							}

							// forward message to other nodes
							m_line_tx.write(m_buffer, dataSize);
						}
					}
				}
				else
				{
					// got only part of a message - read the rest (and drop it)
					// TODO: combine parts and push to "ring buffer"
					togo = dataSize - recv;
					while (togo > 0){
						recv = m_line_rx.read(m_buffer, togo);
						togo -= recv;
					}
					osd_printf_verbose("M1COMM: dropped a message...\n");
				}
				recv = m_line_rx.read(m_buffer, dataSize);
			}

			// update "ring buffer" if link established
			// live relay does not send data
			if (m_linkid != 0x00 && m_shared[5] != 0x00)
			{
				m_buffer[0] = m_linkid;
				frameOffset = frameStart + (m_linkid * frameSize);
				for (int j = 0x00 ; j < frameSize ; j++)
				{
					// push message to "ring buffer"
					m_shared[frameOffset + j] = m_shared[frameStart + j];
					m_buffer[1 + j] = m_shared[frameStart + j];
				}
				// push message to other nodes
				m_line_tx.write(m_buffer, dataSize);

				// master sends some additional status bytes
				if (isMaster){
					m_buffer[0] = 0xf0;
					for (int j = 0x00 ; j < frameSize ; j++)
					{
						m_buffer[1 + j] = 0x00;
					}
					for (int j = 0x06 ; j < 0x10 ; j++)
					{
						m_buffer[1 + j] = m_shared[j];
					}
					// push message to other nodes
					m_line_tx.write(m_buffer, dataSize);
				}
			}
			// clear 05
			m_shared[5] = 0x00;
		}
	}
}
#endif
