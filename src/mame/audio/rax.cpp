// license:BSD-3-Clause
// copyright-holders:Phil Bennett
/***************************************************************************

    Acclaim RAX Sound Board

****************************************************************************/

#include "emu.h"
#include "rax.h"
#include "speaker.h"


/*************************************
 *
 *  Constants
 *
 *************************************/

/* These are some of the control registers. We don't use them all */
enum
{
	IDMA_CONTROL_REG = 0,   /* 3fe0 */
	BDMA_INT_ADDR_REG,      /* 3fe1 */
	BDMA_EXT_ADDR_REG,      /* 3fe2 */
	BDMA_CONTROL_REG,       /* 3fe3 */
	BDMA_WORD_COUNT_REG,    /* 3fe4 */
	PROG_FLAG_DATA_REG,     /* 3fe5 */
	PROG_FLAG_CONTROL_REG,  /* 3fe6 */

	S1_AUTOBUF_REG = 15,    /* 3fef */
	S1_RFSDIV_REG,          /* 3ff0 */
	S1_SCLKDIV_REG,         /* 3ff1 */
	S1_CONTROL_REG,         /* 3ff2 */
	S0_AUTOBUF_REG,         /* 3ff3 */
	S0_RFSDIV_REG,          /* 3ff4 */
	S0_SCLKDIV_REG,         /* 3ff5 */
	S0_CONTROL_REG,         /* 3ff6 */
	S0_MCTXLO_REG,          /* 3ff7 */
	S0_MCTXHI_REG,          /* 3ff8 */
	S0_MCRXLO_REG,          /* 3ff9 */
	S0_MCRXHI_REG,          /* 3ffa */
	TIMER_SCALE_REG,        /* 3ffb */
	TIMER_COUNT_REG,        /* 3ffc */
	TIMER_PERIOD_REG,       /* 3ffd */
	WAITSTATES_REG,         /* 3ffe */
	SYSCONTROL_REG          /* 3fff */
};


/*************************************
 *
 *  Interface
 *
 *************************************/

WRITE16_MEMBER( acclaim_rax_device::data_w )
{
	m_data_in->write(space, 0, data, 0xffff);
	m_cpu->set_input_line(ADSP2181_IRQL0, ASSERT_LINE);
	machine().scheduler().boost_interleave(attotime::zero, attotime::from_usec(5));
}


READ16_MEMBER( acclaim_rax_device::data_r )
{
	m_adsp_snd_pf0 = 1;
	return m_data_out->read(space, 0);
}


/*************************************
 *
 *  Internal
 *
 *************************************/

READ16_MEMBER( acclaim_rax_device::adsp_control_r )
{
	uint16_t res = 0;

	switch (offset)
	{
		case PROG_FLAG_DATA_REG:
			res = m_adsp_snd_pf0;
			break;
		default:
			res = m_control_regs[offset];
	}

	return res;
}

WRITE16_MEMBER( acclaim_rax_device::adsp_control_w )
{
	m_control_regs[offset] = data;

	switch (offset)
	{
		case 0x1:
			m_control_regs[BDMA_INT_ADDR_REG] = data & 0x3fff;
			break;
		case 0x2:
			m_control_regs[BDMA_EXT_ADDR_REG] = data & 0x3fff;
			break;
		case 0x3:
			m_control_regs[BDMA_CONTROL_REG] = data & 0xff0f;
			break;

		case 0x4:
		{
			m_control_regs[BDMA_WORD_COUNT_REG] = data & 0x3fff;

			const uint8_t * adsp_rom = m_rom + m_rom_bank * 0x400000;

			uint32_t page = (m_control_regs[BDMA_CONTROL_REG] >> 8) & 0xff;
			uint32_t dir = (m_control_regs[BDMA_CONTROL_REG] >> 2) & 1;
			uint32_t type = m_control_regs[BDMA_CONTROL_REG] & 3;
			uint32_t src_addr = (page << 14) | m_control_regs[BDMA_EXT_ADDR_REG];

			uint32_t count = m_control_regs[BDMA_WORD_COUNT_REG];

			address_space* addr_space = (type == 0 ? m_program : m_data);

			if (dir == 0)
			{
				if (type == 0)
				{
					while (count)
					{
						uint32_t src_dword = (adsp_rom[src_addr + 0] << 16) | (adsp_rom[src_addr + 1] << 8) | adsp_rom[src_addr + 2];

						addr_space->write_dword(m_control_regs[BDMA_INT_ADDR_REG] * 4, src_dword);

						src_addr += 3;
						++m_control_regs[BDMA_INT_ADDR_REG];
						--count;
					}
				}
				else if (type == 1)
				{
					while (count)
					{
						uint16_t src_word = (adsp_rom[src_addr + 0] << 8) | adsp_rom[src_addr + 1];

						addr_space->write_word(m_control_regs[BDMA_INT_ADDR_REG] * 2, src_word);

						src_addr += 2;
						++m_control_regs[BDMA_INT_ADDR_REG];
						--count;
					}
				}
				else
				{
					int shift = type == 2 ? 8 : 0;

					while (count)
					{
						uint16_t src_word = adsp_rom[src_addr] << shift;

						addr_space->write_word(m_control_regs[BDMA_INT_ADDR_REG] * 2, src_word);

						++src_addr;
						++m_control_regs[BDMA_INT_ADDR_REG];
						--count;
					}
				}
			}
			else
			{
				fatalerror("DMA to byte memory!");
			}

			attotime word_period = attotime::from_hz(m_cpu->unscaled_clock());
			attotime period = word_period * (data & 0x3fff) * 1;
			m_dma_timer->adjust(period, src_addr, period);

			break;
		}

		case S1_AUTOBUF_REG:
			/* autobuffer off: nuke the timer, and disable the DAC */
			if ((data & 0x0002) == 0)
			{
				dmadac_enable(&m_dmadac[1], 1, 0);
			}
			break;

		case S0_AUTOBUF_REG:
			/* autobuffer off: nuke the timer, and disable the DAC */
			if ((data & 0x0002) == 0)
			{
				dmadac_enable(&m_dmadac[0], 1, 0);
				m_reg_timer[0]->reset();
			}
			break;

		case S1_CONTROL_REG:
			if (((data >> 4) & 3) == 2)
				fatalerror("DCS: Oh no!, the data is compressed with u-law encoding\n");
			if (((data >> 4) & 3) == 3)
				fatalerror("DCS: Oh no!, the data is compressed with A-law encoding\n");
			break;

		case PROG_FLAG_DATA_REG:
			logerror("PFLAGS: %x\n", data);
			break;
		case PROG_FLAG_CONTROL_REG:
			logerror("PFLAG CTRL: %x\n", data);
			break;
		default:
			logerror("Unhandled register: %x %x\n", 0x3fe0 + offset, data);
	}
}


TIMER_DEVICE_CALLBACK_MEMBER( acclaim_rax_device::dma_timer_callback )
{
	/* Update external address count and page */
	m_control_regs[BDMA_WORD_COUNT_REG] = 0;
	m_control_regs[BDMA_EXT_ADDR_REG] = param & 0x3fff;
	m_control_regs[BDMA_CONTROL_REG] &= ~0xff00;
	m_control_regs[BDMA_CONTROL_REG] |= ((param >> 14) & 0xff) << 8;

	if (m_control_regs[BDMA_CONTROL_REG] & 8)
		m_cpu->set_input_line(INPUT_LINE_RESET, PULSE_LINE);
	else
		m_cpu->machine().driver_data()->generic_pulse_irq_line(*m_cpu, ADSP2181_BDMA, 1);

	timer.adjust(attotime::never);
}


void acclaim_rax_device::update_data_ram_bank()
{
	if (m_dmovlay_val == 0)
		membank("databank")->set_entry(0);
	else
		membank("databank")->set_entry(1 + m_data_bank);
}

WRITE16_MEMBER( acclaim_rax_device::ram_bank_w )
{
	// Note: The PCB has two unstuffed RAM locations
	m_data_bank = data & 3;
	update_data_ram_bank();
}

WRITE16_MEMBER( acclaim_rax_device::rom_bank_w )
{
	m_rom_bank = data;
}

READ16_MEMBER( acclaim_rax_device::host_r )
{
	m_cpu->set_input_line(ADSP2181_IRQL0, CLEAR_LINE);
	return m_data_in->read(space, 0);
}

WRITE16_MEMBER( acclaim_rax_device::host_w )
{
	m_data_out->write(space, 0, data, 0xffff);
	m_adsp_snd_pf0 = 0;
}


/*************************************
 *
 *  CPU memory map & config
 *
 *************************************/

static ADDRESS_MAP_START( adsp_program_map, AS_PROGRAM, 32, acclaim_rax_device )
	ADDRESS_MAP_UNMAP_HIGH
	AM_RANGE(0x0000, 0x3fff) AM_RAM AM_SHARE("adsp_pram")
ADDRESS_MAP_END

static ADDRESS_MAP_START( adsp_data_map, AS_DATA, 16, acclaim_rax_device )
	ADDRESS_MAP_UNMAP_HIGH
	AM_RANGE(0x0000, 0x1fff) AM_RAMBANK("databank")
	AM_RANGE(0x2000, 0x3fdf) AM_RAM // Internal RAM
	AM_RANGE(0x3fe0, 0x3fff) AM_READWRITE(adsp_control_r, adsp_control_w)
ADDRESS_MAP_END

static ADDRESS_MAP_START( adsp_io_map, AS_IO, 16, acclaim_rax_device )
	ADDRESS_MAP_UNMAP_HIGH
	AM_RANGE(0x0000, 0x0000) AM_WRITE(ram_bank_w)
	AM_RANGE(0x0001, 0x0001) AM_WRITE(rom_bank_w)
	AM_RANGE(0x0003, 0x0003) AM_READWRITE(host_r, host_w)
ADDRESS_MAP_END


void acclaim_rax_device::device_start()
{
	m_rom = (uint8_t *)machine().root_device().memregion("rax")->base();

	m_program = &m_cpu->space(AS_PROGRAM);
	m_data = &m_cpu->space(AS_DATA);

	m_dmadac[0] = subdevice<dmadac_sound_device>("dacl");
	m_dmadac[1] = subdevice<dmadac_sound_device>("dacr");

	m_reg_timer[0] = subdevice<timer_device>("adsp_reg_timer0");
	m_dma_timer = subdevice<timer_device>("adsp_dma_timer");

	// 1 bank for internal
	membank("databank")->configure_entries(0, 5, auto_alloc_array(machine(), uint16_t, 0x2000 * 5), 0x2000*sizeof(uint16_t));
}

void acclaim_rax_device::device_reset()
{
	/* Load 32 program words (96 bytes) via BDMA */
	for (int i = 0; i < 32; i ++)
	{
		uint32_t word;

		word = m_rom[i*3 + 0] << 16;
		word |= m_rom[i*3 + 1] << 8;
		word |= m_rom[i*3 + 2];

		m_adsp_pram[i] = word;
	}

	m_adsp_snd_pf0 = 1;
	m_rom_bank = 0;

	/* initialize our state structure and install the transmit callback */
	m_size[0] = 0;
	m_incs[0] = 0;
	m_ireg[0] = 0;

	/* initialize the ADSP control regs */
	memset(m_control_regs, 0, sizeof(m_control_regs));

	m_dmovlay_val = 0;
	m_data_bank = 0;
	update_data_ram_bank();
}


void acclaim_rax_device::adsp_irq(int which)
{
	if (which != 0)
		return;

	/* get the index register */
	int reg = m_cpu->state_int(ADSP2100_I0 + m_ireg[which]);

	/* copy the current data into the buffer */
	int count = m_size[which] / (4 * (m_incs[which] ? m_incs[which] : 1));

	int16_t buffer[0x100];

	for (uint32_t i = 0; i < count; i++)
	{
		buffer[i] = m_data->read_word(reg * 2);
		reg += m_incs[which];
	}

	dmadac_transfer(&m_dmadac[0], 2, 1, 2, count/2, buffer);

	/* check for wrapping */
	if (reg >= m_ireg_base[which] + m_size[which])
	{
		/* reset the base pointer */
		reg = m_ireg_base[which];
	}

	m_cpu->set_state_int(ADSP2100_I0 + m_ireg[which], reg);
}

TIMER_DEVICE_CALLBACK_MEMBER( acclaim_rax_device::adsp_irq0 )
{
	adsp_irq(0);
}



void acclaim_rax_device::recompute_sample_rate(int which)
{
	/* calculate how long until we generate an interrupt */

	/* frequency the time per each bit sent */
	attotime sample_period = attotime::from_hz(m_cpu->unscaled_clock()) * (1 * (m_control_regs[which ? S1_SCLKDIV_REG : S0_SCLKDIV_REG] + 1));

	/* now put it down to samples, so we know what the channel frequency has to be */
	sample_period = sample_period * (16 * 1);
	dmadac_set_frequency(&m_dmadac[0], 2, ATTOSECONDS_TO_HZ(sample_period.attoseconds()));
	dmadac_enable(&m_dmadac[0], 2, 1);

	/* fire off a timer which will hit every half-buffer */
	if (m_incs[which])
	{
		attotime period = (sample_period * m_size[which]) / (4 * 2 * m_incs[which]);
		m_reg_timer[which]->adjust(period, 0, period);
	}
}

WRITE32_MEMBER(acclaim_rax_device::adsp_sound_tx_callback)
{
	int which = offset;

	if (which != 0)
		return;

	int autobuf_reg = which ? S1_AUTOBUF_REG : S0_AUTOBUF_REG;

	/* check if SPORT1 is enabled */
	if (m_control_regs[SYSCONTROL_REG] & (which ? 0x0800 : 0x1000)) /* bit 11 */
	{
		/* we only support autobuffer here (which is what this thing uses), bail if not enabled */
		if (m_control_regs[autobuf_reg] & 0x0002) /* bit 1 */
		{
			/* get the autobuffer registers */
			int     mreg, lreg;
			uint16_t  source;

			m_ireg[which] = (m_control_regs[autobuf_reg] >> 9) & 7;
			mreg = (m_control_regs[autobuf_reg] >> 7) & 3;
			mreg |= m_ireg[which] & 0x04; /* msb comes from ireg */
			lreg = m_ireg[which];

			/* now get the register contents in a more legible format */
			/* we depend on register indexes to be continuous (which is the case in our core) */
			source = m_cpu->state_int(ADSP2100_I0 + m_ireg[which]);
			m_incs[which] = m_cpu->state_int(ADSP2100_M0 + mreg);
			m_size[which] = m_cpu->state_int(ADSP2100_L0 + lreg);

			/* get the base value, since we need to keep it around for wrapping */
			source -= m_incs[which];

			/* make it go back one so we dont lose the first sample */
			m_cpu->set_state_int(ADSP2100_I0 + m_ireg[which], source);

			/* save it as it is now */
			m_ireg_base[which] = source;

			/* recompute the sample rate and timer */
			recompute_sample_rate(which);
			return;
		}
		else
			logerror( "ADSP SPORT1: trying to transmit and autobuffer not enabled!\n" );
	}

	/* if we get there, something went wrong. Disable playing */
	dmadac_enable(&m_dmadac[0], 2, 0);

	/* remove timer */
	m_reg_timer[which]->reset();
}

WRITE32_MEMBER(acclaim_rax_device::dmovlay_callback)
{
	if (data < 0 || data > 1)
	{
		fatalerror("dmovlay_callback: Error! dmovlay called with value = %X\n", data);
	}
	else
	{
		m_dmovlay_val = data;
		update_data_ram_bank();
	}
}


DEFINE_DEVICE_TYPE(ACCLAIM_RAX, acclaim_rax_device, "rax_audio", "Acclaim RAX")

//-------------------------------------------------
//  acclaim_rax_device - constructor
//-------------------------------------------------

acclaim_rax_device::acclaim_rax_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, ACCLAIM_RAX, tag, owner, clock)
	, m_cpu(*this, "adsp")
	, m_adsp_pram(*this, "adsp_pram")
	, m_adsp_data_bank(*this, "databank")
	, m_data_in(*this, "data_in")
	, m_data_out(*this, "data_out")
{

}

//-------------------------------------------------
// device_add_mconfig - add device configuration
//-------------------------------------------------

MACHINE_CONFIG_MEMBER( acclaim_rax_device::device_add_mconfig )
	MCFG_CPU_ADD("adsp", ADSP2181, XTAL_16_67MHz)
	MCFG_ADSP21XX_SPORT_TX_CB(WRITE32(acclaim_rax_device, adsp_sound_tx_callback))      /* callback for serial transmit */
	MCFG_ADSP21XX_DMOVLAY_CB(WRITE32(acclaim_rax_device, dmovlay_callback)) // callback for adsp 2181 dmovlay instruction
	MCFG_CPU_PROGRAM_MAP(adsp_program_map)
	MCFG_CPU_DATA_MAP(adsp_data_map)
	MCFG_CPU_IO_MAP(adsp_io_map)

	MCFG_TIMER_DEVICE_ADD("adsp_reg_timer0", DEVICE_SELF, acclaim_rax_device, adsp_irq0)
	MCFG_TIMER_DEVICE_ADD("adsp_dma_timer", DEVICE_SELF, acclaim_rax_device, dma_timer_callback)

	MCFG_GENERIC_LATCH_16_ADD("data_in")
	MCFG_GENERIC_LATCH_16_ADD("data_out")

	MCFG_SPEAKER_STANDARD_STEREO("lspeaker", "rspeaker")

	MCFG_SOUND_ADD("dacl", DMADAC, 0)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "lspeaker", 1.0)

	MCFG_SOUND_ADD("dacr", DMADAC, 0)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "rspeaker", 1.0)
MACHINE_CONFIG_END

