// license:BSD-3-Clause
// copyright-holders:Ryan Holtz, Andreas Naive, Fabio Priuli
/***********************************************************************************************************

 S-DD1 add-on chip emulation (for SNES/SFC)

 Based on Andreas Naive Public Domain code.
 Code ported by Ryan Holtz and updated to slots by Fabio Priuli.

 ***********************************************************************************************************/


#include "emu.h"
#include "sdd1.h"


#define SSD1_ADD(addr)\
	mmc[(addr >> 20) & 3] + (addr & 0x0fffff)


// Input Manager

void sns_rom_sdd1_device::SDD1_IM::IM_prepareDecomp(uint32_t in_buf)
{
	m_byte_ptr = in_buf;
	m_bit_count = 4;
}

uint8_t sns_rom_sdd1_device::SDD1_IM::IM_getCodeword(uint8_t *ROM, uint32_t *mmc, const uint8_t code_len)
{
	uint8_t codeword = ROM[SSD1_ADD(m_byte_ptr)] << m_bit_count;

	++m_bit_count;

	if (codeword & 0x80)
	{
		codeword |= ROM[SSD1_ADD((m_byte_ptr + 1))] >> (9 - m_bit_count);
		m_bit_count += code_len;
	}

	if (m_bit_count & 0x08)
	{
		m_byte_ptr++;
		m_bit_count &= 0x07;
	}

	return codeword;
}

// GCD

void sns_rom_sdd1_device::SDD1_GCD::GCD_getRunCount(uint8_t *ROM, uint32_t *mmc, uint8_t code_num, uint8_t* MPScount, uint8_t* LPSind)
{
	static constexpr uint8_t run_count[] =
	{
		0x00, 0x00, 0x01, 0x00, 0x03, 0x01, 0x02, 0x00,
		0x07, 0x03, 0x05, 0x01, 0x06, 0x02, 0x04, 0x00,
		0x0f, 0x07, 0x0b, 0x03, 0x0d, 0x05, 0x09, 0x01,
		0x0e, 0x06, 0x0a, 0x02, 0x0c, 0x04, 0x08, 0x00,
		0x1f, 0x0f, 0x17, 0x07, 0x1b, 0x0b, 0x13, 0x03,
		0x1d, 0x0d, 0x15, 0x05, 0x19, 0x09, 0x11, 0x01,
		0x1e, 0x0e, 0x16, 0x06, 0x1a, 0x0a, 0x12, 0x02,
		0x1c, 0x0c, 0x14, 0x04, 0x18, 0x08, 0x10, 0x00,
		0x3f, 0x1f, 0x2f, 0x0f, 0x37, 0x17, 0x27, 0x07,
		0x3b, 0x1b, 0x2b, 0x0b, 0x33, 0x13, 0x23, 0x03,
		0x3d, 0x1d, 0x2d, 0x0d, 0x35, 0x15, 0x25, 0x05,
		0x39, 0x19, 0x29, 0x09, 0x31, 0x11, 0x21, 0x01,
		0x3e, 0x1e, 0x2e, 0x0e, 0x36, 0x16, 0x26, 0x06,
		0x3a, 0x1a, 0x2a, 0x0a, 0x32, 0x12, 0x22, 0x02,
		0x3c, 0x1c, 0x2c, 0x0c, 0x34, 0x14, 0x24, 0x04,
		0x38, 0x18, 0x28, 0x08, 0x30, 0x10, 0x20, 0x00,
		0x7f, 0x3f, 0x5f, 0x1f, 0x6f, 0x2f, 0x4f, 0x0f,
		0x77, 0x37, 0x57, 0x17, 0x67, 0x27, 0x47, 0x07,
		0x7b, 0x3b, 0x5b, 0x1b, 0x6b, 0x2b, 0x4b, 0x0b,
		0x73, 0x33, 0x53, 0x13, 0x63, 0x23, 0x43, 0x03,
		0x7d, 0x3d, 0x5d, 0x1d, 0x6d, 0x2d, 0x4d, 0x0d,
		0x75, 0x35, 0x55, 0x15, 0x65, 0x25, 0x45, 0x05,
		0x79, 0x39, 0x59, 0x19, 0x69, 0x29, 0x49, 0x09,
		0x71, 0x31, 0x51, 0x11, 0x61, 0x21, 0x41, 0x01,
		0x7e, 0x3e, 0x5e, 0x1e, 0x6e, 0x2e, 0x4e, 0x0e,
		0x76, 0x36, 0x56, 0x16, 0x66, 0x26, 0x46, 0x06,
		0x7a, 0x3a, 0x5a, 0x1a, 0x6a, 0x2a, 0x4a, 0x0a,
		0x72, 0x32, 0x52, 0x12, 0x62, 0x22, 0x42, 0x02,
		0x7c, 0x3c, 0x5c, 0x1c, 0x6c, 0x2c, 0x4c, 0x0c,
		0x74, 0x34, 0x54, 0x14, 0x64, 0x24, 0x44, 0x04,
		0x78, 0x38, 0x58, 0x18, 0x68, 0x28, 0x48, 0x08,
		0x70, 0x30, 0x50, 0x10, 0x60, 0x20, 0x40, 0x00,
	};

	uint8_t codeword = m_IM->IM_getCodeword(ROM, mmc, code_num);

	if (codeword & 0x80)
	{
		*LPSind = 1;
		*MPScount = run_count[codeword >> (code_num ^ 0x07)];
	}
	else
	{
		*MPScount = (1 << code_num);
	}
}

// BG

void sns_rom_sdd1_device::SDD1_BG::BG_prepareDecomp()
{
	m_MPScount = 0;
	m_LPSind = 0;
}

uint8_t sns_rom_sdd1_device::SDD1_BG::BG_getBit(uint8_t *ROM, uint32_t *mmc, uint8_t* endOfRun)
{
	uint8_t bit;

	if (!(m_MPScount || m_LPSind))
	{
		m_GCD->GCD_getRunCount(ROM, mmc, m_code_num, &(m_MPScount), &(m_LPSind));
	}

	if (m_MPScount)
	{
		bit = 0;
		m_MPScount--;
	}
	else
	{
		bit = 1;
		m_LPSind = 0;
	}

	if (m_MPScount || m_LPSind)
	{
		(*endOfRun) = 0;
	}
	else
	{
		(*endOfRun) = 1;
	}

	return bit;
}

// PEM

struct SDD1_PEM_state
{
	uint8_t code_num;
	uint8_t nextIfMPS;
	uint8_t nextIfLPS;
};

static constexpr SDD1_PEM_state PEM_evolution_table[33] =
{
	{ 0,25,25},
	{ 0, 2, 1},
	{ 0, 3, 1},
	{ 0, 4, 2},
	{ 0, 5, 3},
	{ 1, 6, 4},
	{ 1, 7, 5},
	{ 1, 8, 6},
	{ 1, 9, 7},
	{ 2,10, 8},
	{ 2,11, 9},
	{ 2,12,10},
	{ 2,13,11},
	{ 3,14,12},
	{ 3,15,13},
	{ 3,16,14},
	{ 3,17,15},
	{ 4,18,16},
	{ 4,19,17},
	{ 5,20,18},
	{ 5,21,19},
	{ 6,22,20},
	{ 6,23,21},
	{ 7,24,22},
	{ 7,24,23},
	{ 0,26, 1},
	{ 1,27, 2},
	{ 2,28, 4},
	{ 3,29, 8},
	{ 4,30,12},
	{ 5,31,16},
	{ 6,32,18},
	{ 7,24,22}
};

void sns_rom_sdd1_device::SDD1_PEM::PEM_prepareDecomp()
{
	for (auto & elem : m_contextInfo)
	{
		elem.status = 0;
		elem.MPS = 0;
	}
}

uint8_t sns_rom_sdd1_device::SDD1_PEM::PEM_getBit(uint8_t *ROM, uint32_t *mmc, uint8_t context)
{
	uint8_t endOfRun;
	uint8_t bit;

	SDD1_PEM_ContextInfo *pContInfo = &(m_contextInfo)[context];
	uint8_t currStatus = pContInfo->status;
	const SDD1_PEM_state* pState = &(PEM_evolution_table[currStatus]);
	uint8_t currentMPS = pContInfo->MPS;

	bit = m_BG[pState->code_num]->BG_getBit(ROM, mmc, &endOfRun);

	if (endOfRun)
	{
		if (bit)
		{
			if (!(currStatus & 0xfe))
			{
				(pContInfo->MPS) ^= 0x01;
			}
			pContInfo->status = pState->nextIfLPS;
		}
		else
		{
			pContInfo->status = pState->nextIfMPS;
		}
	}

	return bit ^ currentMPS;
}

// CM

void sns_rom_sdd1_device::SDD1_CM::CM_prepareDecomp(uint8_t *ROM, uint32_t *mmc, uint32_t first_byte)
{
	int32_t i = 0;
	m_bitplanesInfo = ROM[SSD1_ADD(first_byte)] & 0xc0;
	m_contextBitsInfo = ROM[SSD1_ADD(first_byte)] & 0x30;
	m_bit_number = 0;
	for (i = 0; i < 8; i++)
	{
		m_prevBitplaneBits[i] = 0;
	}
	switch (m_bitplanesInfo)
	{
		case 0x00:
			m_currBitplane = 1;
			break;
		case 0x40:
			m_currBitplane = 7;
			break;
		case 0x80:
			m_currBitplane = 3;
			break;
	}
}

uint8_t sns_rom_sdd1_device::SDD1_CM::CM_getBit(uint8_t *ROM, uint32_t *mmc)
{
	uint8_t currContext;
	uint16_t *context_bits;
	uint8_t bit;

	switch (m_bitplanesInfo)
	{
		case 0x00:
			m_currBitplane ^= 0x01;
			break;
		case 0x40:
			m_currBitplane ^= 0x01;
			if (!(m_bit_number & 0x7f))
				m_currBitplane = ((m_currBitplane + 2) & 0x07);
			break;
		case 0x80:
			m_currBitplane ^= 0x01;
			if (!(m_bit_number & 0x7f))
				m_currBitplane ^= 0x02;
			break;
		case 0xc0:
			m_currBitplane = m_bit_number & 0x07;
			break;
	}

	context_bits = &(m_prevBitplaneBits)[m_currBitplane];

	currContext = (m_currBitplane & 0x01) << 4;
	switch (m_contextBitsInfo)
	{
		case 0x00:
			currContext |= ((*context_bits & 0x01c0) >> 5) | (*context_bits & 0x0001);
			break;
		case 0x10:
			currContext |= ((*context_bits & 0x0180) >> 5) | (*context_bits & 0x0001);
			break;
		case 0x20:
			currContext |= ((*context_bits & 0x00c0) >> 5) | (*context_bits & 0x0001);
			break;
		case 0x30:
			currContext |= ((*context_bits & 0x0180) >> 5) | (*context_bits & 0x0003);
			break;
	}

	bit = m_PEM->PEM_getBit(ROM, mmc, currContext);

	*context_bits <<= 1;
	*context_bits |= bit;

	m_bit_number++;

	return bit;
}

// OL

void sns_rom_sdd1_device::SDD1_OL::OL_prepareDecomp(uint8_t *ROM, uint32_t *mmc, uint32_t first_byte, uint16_t out_len, uint8_t *out_buf)
{
	m_bitplanesInfo = ROM[SSD1_ADD(first_byte)] & 0xc0;
	m_length = out_len;
	m_buffer = out_buf;
}

void sns_rom_sdd1_device::SDD1_OL::OL_launch(uint8_t *ROM, uint32_t *mmc)
{
	uint8_t i;
	uint8_t register1, register2 = 0;

	switch (m_bitplanesInfo)
	{
		case 0x00:
		case 0x40:
		case 0x80:
			i = 1;
			do
			{   // if length == 0, we output 2^16 bytes
				if (!i)
				{
					*(m_buffer++) = register2;
					i = ~i;
				}
				else
				{
					for (register1 = register2 = 0, i = 0x80; i; i >>= 1)
					{
						if (m_CM->CM_getBit(ROM, mmc))
							register1 |= i;

						if (m_CM->CM_getBit(ROM, mmc))
							register2 |= i;
					}
					*(m_buffer++) = register1;
				}
			} while (--(m_length));
			break;
		case 0xc0:
			do
			{
				for (register1 = 0, i = 0x01; i; i <<= 1)
				{
					if (m_CM->CM_getBit(ROM, mmc))
					{
						register1 |= i;
					}
				}
				*(m_buffer++) = register1;
			} while (--(m_length));
			break;
	}
}

// S-DD1

sns_rom_sdd1_device::SDD1_emu::SDD1_emu(running_machine &machine)
		: m_machine(machine)
{
	m_IM = std::make_unique<SDD1_IM>();
	m_GCD = std::make_unique<SDD1_GCD>(m_IM.get());
	m_BG0 = std::make_unique<SDD1_BG>(m_GCD.get(), 0);
	m_BG1 = std::make_unique<SDD1_BG>(m_GCD.get(), 1);
	m_BG2 = std::make_unique<SDD1_BG>(m_GCD.get(), 2);
	m_BG3 = std::make_unique<SDD1_BG>(m_GCD.get(), 3);
	m_BG4 = std::make_unique<SDD1_BG>(m_GCD.get(), 4);
	m_BG5 = std::make_unique<SDD1_BG>(m_GCD.get(), 5);
	m_BG6 = std::make_unique<SDD1_BG>(m_GCD.get(), 6);
	m_BG7 = std::make_unique<SDD1_BG>(m_GCD.get(), 7);
	m_PEM = std::make_unique<SDD1_PEM>(m_BG0.get(), m_BG1.get(), m_BG2.get(), m_BG3.get(),
											m_BG4.get(), m_BG5.get(), m_BG6.get(), m_BG7.get());
	m_CM = std::make_unique<SDD1_CM>(m_PEM.get());
	m_OL = std::make_unique<SDD1_OL>(m_CM.get());
}

void sns_rom_sdd1_device::SDD1_emu::SDD1emu_decompress(uint8_t *ROM, uint32_t *mmc, uint32_t in_buf, uint16_t out_len, uint8_t *out_buf)
{
	m_IM->IM_prepareDecomp(in_buf);
	m_BG0->BG_prepareDecomp();
	m_BG1->BG_prepareDecomp();
	m_BG2->BG_prepareDecomp();
	m_BG3->BG_prepareDecomp();
	m_BG4->BG_prepareDecomp();
	m_BG5->BG_prepareDecomp();
	m_BG6->BG_prepareDecomp();
	m_BG7->BG_prepareDecomp();
	m_PEM->PEM_prepareDecomp();
	m_CM->CM_prepareDecomp(ROM, mmc, in_buf);
	m_OL->OL_prepareDecomp(ROM, mmc, in_buf, out_len, out_buf);

	m_OL->OL_launch(ROM, mmc);
}


//-------------------------------------------------
//  sns_rom_sdd1_device - constructor
//-------------------------------------------------

DEFINE_DEVICE_TYPE(SNS_LOROM_SDD1, sns_rom_sdd1_device, "sns_rom_sdd1", "SNES Cart + S-DD1")


sns_rom_sdd1_device::sns_rom_sdd1_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, type, tag, owner, clock)
	, device_sns_cart_interface(mconfig, *this)
	, m_sdd1_enable(0), m_xfer_enable(0), m_sdd1emu(nullptr)
{
}

sns_rom_sdd1_device::sns_rom_sdd1_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: sns_rom_sdd1_device(mconfig, SNS_LOROM_SDD1, tag, owner, clock)
{
}


void sns_rom_sdd1_device::device_start()
{
	m_sdd1emu = std::make_unique<SDD1_emu>(machine());

	m_buffer.data = std::make_unique<uint8_t[]>(0x10000);
	m_buffer.ready = 0;

	save_item(NAME(m_sdd1_enable));
	save_item(NAME(m_xfer_enable));
	save_item(NAME(m_mmc));

	for (int i = 0; i < 8; i++)
	{
		save_item(NAME(m_dma[i].addr), i);
		save_item(NAME(m_dma[i].size), i);
	}

	save_pointer(NAME(m_buffer.data.get()), 0x10000);
	save_item(NAME(m_buffer.offset));
	save_item(NAME(m_buffer.size));
	save_item(NAME(m_buffer.ready));

	// TODO: save remaining decomp-related items so to fix support if we try to save mid-decompression...
}

void sns_rom_sdd1_device::device_reset()
{
	m_sdd1_enable = 0x00;
	m_xfer_enable = 0x00;

	m_mmc[0] = 0 << 20;
	m_mmc[1] = 1 << 20;
	m_mmc[2] = 2 << 20;
	m_mmc[3] = 3 << 20;

	for(auto & elem : m_dma)
	{
		elem.addr = 0;
		elem.size = 0;
	}
}

/*-------------------------------------------------
 mapper specific handlers
 -------------------------------------------------*/

READ8_MEMBER( sns_rom_sdd1_device::chip_read )
{
	uint16_t addr = offset & 0xffff;

	switch (addr)
	{
		case 0x4804:
			return (m_mmc[0] >> 20) & 7;
		case 0x4805:
			return (m_mmc[1] >> 20) & 7;
		case 0x4806:
			return (m_mmc[2] >> 20) & 7;
		case 0x4807:
			return (m_mmc[3] >> 20) & 7;
	}

//  we should never get here, but...
	return 0;
}


WRITE8_MEMBER( sns_rom_sdd1_device::chip_write )
{
	uint16_t addr = offset & 0xffff;

	if ((addr & 0x4380) == 0x4300)
	{
		uint8_t channel = (addr >> 4) & 7;
		switch(addr & 0xf)
		{
			case 2:
				m_dma[channel].addr = (m_dma[channel].addr & 0xffff00) + (data <<  0);
				break;
			case 3:
				m_dma[channel].addr = (m_dma[channel].addr & 0xff00ff) + (data <<  8);
				break;
			case 4:
				m_dma[channel].addr = (m_dma[channel].addr & 0x00ffff) + (data << 16);
				break;

			case 5:
				m_dma[channel].size = (m_dma[channel].size &   0xff00) + (data <<  0);
				break;
			case 6:
				m_dma[channel].size = (m_dma[channel].size &   0x00ff) + (data <<  8);
				break;
		}
		return;
	}

	switch(addr)
	{
		case 0x4800:
			m_sdd1_enable = data;
			break;
		case 0x4801:
			m_xfer_enable = data;
			break;

		case 0x4804:
			m_mmc[0] = (data & 7) << 20;
			break;
		case 0x4805:
			m_mmc[1] = (data & 7) << 20;
			break;
		case 0x4806:
			m_mmc[2] = (data & 7) << 20;
			break;
		case 0x4807:
			m_mmc[3] = (data & 7) << 20;
			break;
	}

}

uint8_t sns_rom_sdd1_device::read_helper(uint32_t addr)
{
	if (m_sdd1_enable & m_xfer_enable)
	{
		// at least one channel has S-DD1 decompression enabled...
		for (int i = 0; i < 8; i++)
		{
			if (m_sdd1_enable & m_xfer_enable & (1 << i))
			{
				// S-DD1 always uses fixed transfer mode, so address will not change during transfer
				if ((addr + 0xc00000) == m_dma[i].addr)
				{
					uint8_t data;
					if (!m_buffer.ready)
					{
						// first byte read for channel performs full decompression.
						// this really should stream byte-by-byte, but it's not necessary since the size is known
						m_buffer.offset = 0;
						m_buffer.size = m_dma[i].size ? m_dma[i].size : 65536;

						// SDD1_emu calls this function; it needs to access uncompressed data;
						// so temporarily disable decompression mode for decompress() call.
						m_sdd1emu->SDD1emu_decompress(m_rom, m_mmc, addr, m_buffer.size, m_buffer.data.get());

						m_buffer.ready = 1;
					}

					// fetch a decompressed byte; once buffer is depleted, disable channel and invalidate buffer
					data = m_buffer.data[(uint16_t)m_buffer.offset++];
					if (m_buffer.offset >= m_buffer.size)
					{
						m_buffer.ready = 0;
						m_xfer_enable &= ~(1 << i);
					}

					return data;
				}
			}
		}
	}

	return m_rom[m_mmc[(addr >> 20) & 3] + (addr & 0x0fffff)];
}

READ8_MEMBER(sns_rom_sdd1_device::read_l)
{
	if (offset < 0x400000)
		return m_rom[rom_bank_map[offset / 0x10000] * 0x8000 + (offset & 0x7fff)];
	else
		return m_rom[rom_bank_map[(offset - 0x400000) / 0x8000] * 0x8000 + (offset & 0x7fff)];
}

READ8_MEMBER(sns_rom_sdd1_device::read_h)
{
	if (offset >= 0x400000)
		return read_helper(offset - 0x400000);
	else
		return read_l(space, offset);
}


READ8_MEMBER( sns_rom_sdd1_device::read_ram )
{
	return m_nvram[offset & 0x1fff];
}

WRITE8_MEMBER( sns_rom_sdd1_device::write_ram )
{
	m_nvram[offset & 0x1fff] = data;
}
