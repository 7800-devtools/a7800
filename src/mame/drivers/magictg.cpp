// license:BSD-3-Clause
// copyright-holders:Philip Bennett
/***************************************************************************

    Magic the Gathering: Armageddon

    preliminary driver by Phil Bennett


PCB Information (needs tidying:)


TOP Board
.20u    27c4001 stickered   U20
                #1537 V1.0  a 1 was hadwritten over the 0

.u7     stamped     (c) 1997
                ACCLAIM COINOP
                ARMAGEDDON
                SND3 P/N 1605
                9806 D


.u8     stamped     (c) 1997 11/25/97
                ACCLAIM COINOP
                ARMAGEDDON
                1534 SND0
                9752 D

.u13        stamped     (c) 1997 11/25/97
                ACCLAIM COINOP
                ARMAGEDDON
                1536 SND2
                9752 D

.u14        stamped     (c) 1997 11/25/97
                ACCLAIM COINOP
                ARMAGEDDON
                1535 SND1
                9752 D

Analog devices  ADSP 2181
Xilinx      XC5202
dt71256 x2
Analog Devices  AD1866R

Bottom board
.u32    27c801      stickered   4
.u33    27c801      stickered   3
.u34    27c801      stickered   2
.u35    27c801      stickered   1
.u58    AT17C128    stickered   CC3E
.u66    GAL16V8D

Xilinx  XC4005E
Xilinx  XC95108 stickered   ACCLAIM COIN-OP
                CSC2_10.JED
                B710/0B84
                U40 p/N 1611

3dFX    500-0003-03     x2
    BF1684.1
TI  TVP3409
    V53C16258HK40       x24
    V53C511816500K60    x4

U38 and U97 on main board   3DFX
                            500-0004-02
                            BF2733.1 TMU
                            9748 20001
                            TAIWAN 1001

U4 on daughter board        Zoran ZR36050PQC
                            -29.5
                            85 GF7B9726E

U11 on main board           Removed heatsink, Couldn't see anything...


U71 on main board           Galileo
                            GT-64010A-B-0
                            BB8018.1
                            TAIWAN
14.31818 Oscillator by the TI part
50.0000 Oscillator by EPROMS
33.0000 Oscillator by the V53C511816500K60
KM622560LG-7 by Battery

Bottom daughter board
All read as 29F800B
.u9     stamped     (c) 1997
                ACCLAIM COINOP
                ARMAGEDDON S0
                1514 11/25/97
                9803 D


.u10        stamped     (c) 1997
                ACCLAIM COINOP
                ARMAGEDDON S1
                1515 11/25/97
                9803 D

.u11        stamped     (c) 1997
                ACCLAIM COINOP
                ARMAGEDDON S3
                1517 11/25/97
                9803 D

.u12        stamped     (c) 1997
                ACCLAIM COINOP
                ARMAGEDDON S2
                1516 11/25/97
                9803 D

.u20        stamped     (c) 1997
                ACCLAIM COINOP
                ARMAGEDDON K0
                1543 11/25/97
                9752 D

Xilinx  XC4010E
Zoran   ZR36120PQC
Zoran   ZR36016PQC
Xilinx  XC3120A
    DT72811
    DT71256 x2
    DT72271
29.500000 osciallator by ZR36120PQC
Medium size chip with heat sink on it

***************************************************************************/

#include "emu.h"
#include "cpu/adsp2100/adsp2100.h"
#include "cpu/mips/mips3.h"
#include "machine/lpci.h"
#include "sound/dmadac.h"
#include "video/voodoo.h"
#include "screen.h"
#include "speaker.h"


/* TODO: Two 3Dfx Voodoo chipsets are used in SLI configuration */
// #define USE_TWO_3DFX

class magictg_state : public driver_device
{
public:
	magictg_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_mips(*this, "mips"),
		m_adsp(*this, "adsp"),
		m_pci(*this, "pcibus"),
		m_adsp_pram(*this, "adsp_pram"){ }

	required_device<cpu_device>         m_mips;
	required_device<adsp2181_device>    m_adsp;
	required_device<pci_bus_legacy_device>      m_pci;


	/* ASIC */
	struct
	{
		uint32_t src_addr;
		uint32_t dst_addr;
		uint32_t ctrl;
		uint32_t count;
	} m_dma_ch[3];


	/* ADSP-2181 */
	required_shared_ptr<uint32_t> m_adsp_pram;

	struct
	{
		uint16_t bdma_internal_addr;
		uint16_t bdma_external_addr;
		uint16_t bdma_control;
		uint16_t bdma_word_count;
	} m_adsp_regs;


	/* 3Dfx Voodoo */
	voodoo_device*                           m_voodoo[2];

	struct
	{
		/* PCI */
		uint32_t command;
		uint32_t base_addr;

		uint32_t init_enable;
	} m_voodoo_pci_regs[2];


	struct
	{
		/* PCI */
		uint32_t command;
		uint32_t base_addr;

		/* Memory-mapped */
		uint32_t as_regs[19];
	} m_zr36120;


	DECLARE_READ32_MEMBER( zr36120_r );
	DECLARE_WRITE32_MEMBER( zr36120_w );

	DECLARE_READ32_MEMBER( f0_r );
	DECLARE_WRITE32_MEMBER( f0_w );

	DECLARE_READ32_MEMBER( unk_r );
	DECLARE_READ32_MEMBER( unk2_r );

	DECLARE_WRITE32_MEMBER( serial_w );

	DECLARE_READ32_MEMBER( adsp_idma_data_r );
	DECLARE_WRITE32_MEMBER( adsp_idma_data_w );
	DECLARE_WRITE32_MEMBER( adsp_idma_addr_w );

	DECLARE_READ32_MEMBER( adsp_status_r );
	DECLARE_READ16_MEMBER( adsp_control_r );
	DECLARE_WRITE16_MEMBER( adsp_control_w );

	void zr36120_reset();

protected:
	virtual void machine_start() override;
	virtual void machine_reset() override;
	virtual void video_start() override;
public:
	uint32_t screen_update_magictg(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);
};


/*************************************
 *
 *  Machine initialization
 *
 *************************************/

void magictg_state::machine_start()
{
	m_voodoo[0] = (voodoo_device*)machine().device("voodoo_0");
	m_voodoo[1] = (voodoo_device*)machine().device("voodoo_1");
}


void magictg_state::machine_reset()
{
	uint8_t *adsp_boot = (uint8_t*)memregion("adsp")->base();

	zr36120_reset();

	/* Load 32 program words (96 bytes) via BDMA */
	for (int i = 0; i < 32; i ++)
	{
		uint32_t word;

		word = adsp_boot[i*3 + 0] << 16;
		word |= adsp_boot[i*3 + 1] << 8;
		word |= adsp_boot[i*3 + 2];

		m_adsp_pram[i] = word;
	}
}


/*************************************
 *
 *  Video
 *
 *************************************/

void magictg_state::video_start()
{
}

uint32_t magictg_state::screen_update_magictg(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
	return m_voodoo[0]->voodoo_update(bitmap, cliprect) ? 0 : UPDATE_HAS_NOT_CHANGED;
}


/*************************************
 *
 *  3Dfx Voodoo
 *
 *************************************/

static uint32_t pci_dev0_r(device_t *busdevice, device_t *device, int function, int reg, uint32_t mem_mask)
{
	osd_printf_debug("PCI[0] READ: %x\n", reg);
	return 0x00000000; // TODO
}

static void pci_dev0_w(device_t *busdevice, device_t *device, int function, int reg, uint32_t data, uint32_t mem_mask)
{
}


static uint32_t voodoo_0_pci_r(device_t *busdevice, device_t *device, int function, int reg, uint32_t mem_mask)
{
	magictg_state* state = device->machine().driver_data<magictg_state>();
	uint32_t val = 0;

	switch (reg)
	{
		case 0:
			val = 0x0001121a;
			break;
		case 0x10:
			val = state->m_voodoo_pci_regs[0].base_addr;
			break;
		case 0x40:
			val = state->m_voodoo_pci_regs[0].init_enable;
			break;
		default:
			osd_printf_debug("Voodoo[0] PCI R: %x\n", reg);
	}
	return val;
}

static void voodoo_0_pci_w(device_t *busdevice, device_t *device, int function, int reg, uint32_t data, uint32_t mem_mask)
{
	magictg_state* state = device->machine().driver_data<magictg_state>();

	switch (reg)
	{
		case 0x04:
			state->m_voodoo_pci_regs[0].command = data & 0x3;
			break;
		case 0x10:
			if (data == 0xffffffff)
				state->m_voodoo_pci_regs[0].base_addr = 0xff000000;
			else
				state->m_voodoo_pci_regs[0].base_addr = data;
			break;
		case 0x40:
			state->m_voodoo_pci_regs[0].init_enable = data;
			state->m_voodoo[0]->voodoo_set_init_enable(data);
			break;

		default:
			osd_printf_debug("Voodoo [%x]: %x\n", reg, data);
	}
}

#if defined(USE_TWO_3DFX)
static uint32_t voodoo_1_pci_r(device_t *busdevice, device_t *device, int function, int reg, uint32_t mem_mask)
{
	magictg_state* state = space.machine().driver_data<magictg_state>();
	uint32_t val = 0;

	switch (reg)
	{
		case 0:
			val = 0x0001121a;
			break;
		case 0x10:
			val = state->m_voodoo_pci_regs[1].base_addr;
			break;
		case 0x40:
			val = state->m_voodoo_pci_regs[1].init_enable;
			break;
		default:
			osd_printf_debug("Voodoo[1] PCI R: %x\n", reg);
	}
	return val;
}

static void voodoo_1_pci_w(device_t *busdevice, device_t *device, int function, int reg, uint32_t data, uint32_t mem_mask)
{
	magictg_state* state = space.machine().driver_data<magictg_state>();

	switch (reg)
	{
		case 0x04:
			voodoo_pci_regs[1].command = data & 0x3;
			break;
		case 0x10:
			if (data == 0xffffffff)
				state->m_voodoo_pci_regs[1].base_addr = 0xff000000;
			else
				state->m_voodoo_pci_regs[1].base_addr = data;
			break;
		case 0x40:
			state->m_voodoo_pci_regs[1].init_enable = data;
			voodoo_set_init_enable(state->m_voodoo[1], data);
			break;

		default:
			osd_printf_debug("Voodoo [%x]: %x\n", reg, data);
	}
}
#endif


/*************************************
 *
 *  PinkEye (JPEG decoder)
 *
 *************************************/

void magictg_state::zr36120_reset()
{
	/* Reset PCI registers */
	m_zr36120.base_addr = 0;

	/* Reset application-specific registers */
	m_zr36120.as_regs[0x00/4] = (1 << 10) | 0x3ff;
	m_zr36120.as_regs[0x04/4] = (1 << 10) | 0x3ff;
	m_zr36120.as_regs[0x08/4] = (1 << 25) | (2 << 3) | 1;
	m_zr36120.as_regs[0x0c/4] = 0xfffffffc;
	m_zr36120.as_regs[0x10/4] = 0xfffffffc;
	m_zr36120.as_regs[0x14/4] = 0x00000000;
	m_zr36120.as_regs[0x18/4] = (7 << 25) | (0xf0 << 12) | 0x3ff;
	m_zr36120.as_regs[0x1c/4] = 0xfffffffc;
	m_zr36120.as_regs[0x20/4] = 0xfffffffc;
	m_zr36120.as_regs[0x24/4] = 0x000000ff;
	m_zr36120.as_regs[0x28/4] = 0x000000ff;
	m_zr36120.as_regs[0x2c/4] = 0xf0000000;
	m_zr36120.as_regs[0x30/4] = 0xfffffffc;
	m_zr36120.as_regs[0x34/4] = (1 << 29) | (1 << 28) | (3 << 12) | (1 << 8) | (6 << 1);
	m_zr36120.as_regs[0x38/4] = 0x00000000;
	m_zr36120.as_regs[0x3c/4] = 0x00000000;
	m_zr36120.as_regs[0x40/4] = 0x00000000;
	m_zr36120.as_regs[0x44/4] = 0x00000003;
	m_zr36120.as_regs[0x48/4] = 1 << 23;
}

static uint32_t zr36120_pci_r(device_t* busdevice, device_t* device, int function, int reg, uint32_t mem_mask)
{
	magictg_state* state = busdevice->machine().driver_data<magictg_state>();
	uint32_t val = 0;

	switch (reg)
	{
		case 0x00:
			val = 0x612011de;
			break;
		case 0x04:
			val = state->m_zr36120.command;
			break;
		case 0x08:
			val = 0x04000002;
			break;
		case 0x10:
			val = state->m_zr36120.base_addr;
			break;
		default:
			osd_printf_debug("ZR36120 R[%x]\n", reg);
	}
	return val;
}

static void zr36120_pci_w(device_t* busdevice, device_t* device, int function, int reg, uint32_t data, uint32_t mem_mask)
{
	magictg_state* state = busdevice->machine().driver_data<magictg_state>();

	switch (reg)
	{
		case 0x04:
			state->m_zr36120.command = data & 0x6;
			break;
		case 0x10:
			state->m_zr36120.base_addr = data & 0xfffff000;
			break;
		default:
			osd_printf_debug("ZR36120 [%x]: %x\n", reg, data);
	}
}

READ32_MEMBER( magictg_state::zr36120_r )
{
	uint32_t res = 0;

	offset <<= 2;

	if (offset < 0x200)
	{
		switch (offset)
		{
			default:
				res = m_zr36120.as_regs[offset];
		}
	}
	else
	{
		/* Post office */
		res = 0;//mame_rand(space.machine);//m_zr36120.as_regs[0x48/4];
	}
	osd_printf_debug("PINKEYE_R[%x]\n", offset);
	return res;
}

WRITE32_MEMBER( magictg_state::zr36120_w )
{
	offset <<= 2;

	if (offset < 0x200)
	{
		osd_printf_debug("PINKEYE_W[%x] %x\n", offset, data);
		switch (offset)
		{
			case 0x00/4:
				m_zr36120.as_regs[0] = data & 0x400fffff;
				break;
			case 0x04/4:
				m_zr36120.as_regs[1] = data & 0x400003ff;
				break;
			default:
				m_zr36120.as_regs[offset] = data;
		}
	}
	else
	{
		uint32_t guest = (data >> 20) & 3;
		uint32_t g_data = data & 0xff;
		uint32_t g_reg = (data >> 16) & 7;

		/* Direction - 0 for read, 1 for write */
		//  zr36120_guest_write(guest, g_data, g_reg);
		// 2 - ZR36050 JPEG decoder
		// 3 - ZR36016 color-space converter
		osd_printf_debug("GUEST (%.8x): %d  REG: %d  DATA: %x\n", data, guest, g_reg, g_data);
	}
}


/*************************************
 *
 *  System stuff
 *
 *************************************/

READ32_MEMBER( magictg_state::unk_r )
{
	/* Will not boot otherwise */
	return 0x6000;
}

READ32_MEMBER( magictg_state::unk2_r )
{
	return 0xffffffff;
}

WRITE32_MEMBER( magictg_state::serial_w )
{
	if (offset == 0)
	{
		if (mem_mask == 0xff000000)
			printf("%c", data >> 24);
	}
}

WRITE32_MEMBER( magictg_state::f0_w )
{
	int ch;

	offset *= 4;

	data = flipendian_int32(data);
	mem_mask = flipendian_int32(mem_mask);

	ch = ((offset >> 2) & 3) - 1;

	switch (offset)
	{
		case 0x804:
		case 0x808:
		case 0x80c:
			m_dma_ch[ch].count = data;
//          osd_printf_debug("DMA%d COUNT: %.8x\n", ch, data);
			break;

		case 0x814:
		case 0x818:
		case 0x81c:
			m_dma_ch[ch].src_addr = data;
//          osd_printf_debug("DMA%d SRC: %.8x\n", ch, data);
			break;

		case 0x824:
		case 0x828:
		case 0x82c:
			m_dma_ch[ch].dst_addr = data;
//          osd_printf_debug("DMA%d DST: %.8x\n", ch, data);
			break;

		case 0x844:
		case 0x848:
		case 0x84c:
		{
			m_dma_ch[ch].ctrl = data;
//          osd_printf_debug("DMA%d CTRL: %.8x\n", ch, data);

			if (data & 0x1000)
			{
				uint32_t src_addr = m_dma_ch[ch].src_addr;
				uint32_t dst_addr = m_dma_ch[ch].dst_addr;
				//device_t *voodoo = dst_addr > 0xa000000 voodoo0 : voodoo1;

				assert((src_addr & 3) == 0);
				assert((dst_addr & 3) == 0);

				while (m_dma_ch[ch].count > 3)
				{
					uint32_t src_dword = flipendian_int32(space.read_dword(src_addr));
					space.write_dword(dst_addr, src_dword);
					src_addr += 4;
					dst_addr += 4;
					m_dma_ch[ch].count -=4;
				}

				// FIXME!
				if (m_dma_ch[ch].count & 3)
				{
					uint32_t src_dword = flipendian_int32(space.read_dword(src_addr));
					uint32_t dst_dword = space.read_dword(dst_addr);
					uint32_t mask = 0xffffffff >> ((m_dma_ch[ch].count & 3) << 3);

					dst_dword = (dst_dword & ~mask) | (src_dword & mask);
					space.write_dword(dst_addr, dst_dword);
					m_dma_ch[ch].count = 0;
				}
			}

			break;
		}
		case 0xcf8:
		{
			m_pci->write(space, 0, data, mem_mask);
			break;
		}
		case 0xcfc:
		{
			m_pci->write(space, 1, data, mem_mask);
			break;
		}
//      default:
//          osd_printf_debug("W: %.8x: %.8x\n", 0x0f000000 + offset, data);
	}
}

READ32_MEMBER( magictg_state::f0_r )
{
	int ch;
	uint32_t val = 0;
	offset *= 4;

	ch = ((offset >> 2) & 3) - 1;

	switch (offset)
	{
		case 0x804:
		case 0x808:
		case 0x80c:
			val = m_dma_ch[ch].count;
			break;

		case 0x844:
		case 0x848:
		case 0x84c:
			val = 0x00000040; // Status of some sort
			break;

		case 0xcf8:
		{
			val = m_pci->read(space, 0, flipendian_int32(mem_mask));
			break;
		}
		case 0xcfc:
		{
			val = m_pci->read(space, 1, flipendian_int32(mem_mask));
			break;
		}
//      default:
//          osd_printf_debug("R: %.8x\n", 0x0f000000 + offset);
	}

	return flipendian_int32(val);
}


/*************************************
 *
 *  ADSP-2181 internals
 *
 *************************************/

WRITE32_MEMBER( magictg_state::adsp_idma_data_w )
{
	if (ACCESSING_BITS_16_31)
		m_adsp->idma_addr_w(data >> 16);
	else
		m_adsp->idma_addr_w(data & 0xffff);
}

READ32_MEMBER( magictg_state::adsp_idma_data_r )
{
	// TODO: Set /IACK appropriately
	if (ACCESSING_BITS_0_15)
	{
		//osd_printf_debug("RD %.8x %.8x\n", offset, mem_mask);
		return m_adsp->idma_addr_r();
	}
	else
	{
		fatalerror("????\n");
	}
}

WRITE32_MEMBER( magictg_state::adsp_idma_addr_w )
{
	// TODO: Set /IACK appropriately
	if (ACCESSING_BITS_16_31)
	{
		m_adsp->idma_addr_w(data >> 16);
		//osd_printf_debug("WR %.8x %.8x %.8x\n", offset, mem_mask, data >> 16);
	}
	else
		fatalerror("????\n");
}

READ32_MEMBER( magictg_state::adsp_status_r )
{
	// ADSP_IACK = Bit 2
	return (0 << 2) | (space.machine().rand() & 1);
}

READ16_MEMBER( magictg_state::adsp_control_r )
{
	uint16_t res = 0;

	switch (offset)
	{
		case 0x4:
			res = m_adsp_regs.bdma_word_count;
			break;
		case 0x5:
			res = space.machine().rand() & 0xff;
			break;
		default:
			osd_printf_debug("Unhandled register: %x\n", 0x3fe0 + offset);
	}
	return res;
}

WRITE16_MEMBER( magictg_state::adsp_control_w )
{
	switch (offset)
	{
		case 0x1:
			m_adsp_regs.bdma_internal_addr = data & 0x3fff;
			break;
		case 0x2:
			m_adsp_regs.bdma_external_addr = data & 0x3fff;
			break;
		case 0x3:
			m_adsp_regs.bdma_control = data & 0xff0f;
			break;
		case 0x4:
		{
			m_adsp_regs.bdma_word_count = data & 0x3fff;

			if (data > 0)
			{
				uint8_t* adsp_rom = (uint8_t*)memregion("adsp")->base();

				uint32_t page = (m_adsp_regs.bdma_control >> 8) & 0xff;
				uint32_t dir = (m_adsp_regs.bdma_control >> 2) & 1;
				uint32_t type = m_adsp_regs.bdma_control & 3;

				uint32_t src_addr = (page << 14) | m_adsp_regs.bdma_external_addr;

				address_space &addr_space = m_adsp->space((type == 0) ? AS_PROGRAM : AS_DATA);

				if (dir == 0)
				{
					while (m_adsp_regs.bdma_word_count)
					{
						if (type == 0)
						{
							uint32_t src_word =(adsp_rom[src_addr + 0] << 16) |
												(adsp_rom[src_addr + 1] << 8) |
												(adsp_rom[src_addr + 2]);

							addr_space.write_dword(m_adsp_regs.bdma_internal_addr * 4, src_word);

							src_addr += 3;
							m_adsp_regs.bdma_internal_addr ++;
						}
						else if (type == 1)
						{
							uint32_t src_word =(adsp_rom[src_addr + 0] << 8) | adsp_rom[src_addr + 1];

							addr_space.write_dword(m_adsp_regs.bdma_internal_addr * 2, src_word);

							src_addr += 2;
							m_adsp_regs.bdma_internal_addr ++;
						}
						else
						{
							fatalerror("Unsupported BDMA width\n");
						}

						--m_adsp_regs.bdma_word_count;
					}
				}

				/* Update external address count and page */
				m_adsp_regs.bdma_external_addr = src_addr & 0x3fff;
				m_adsp_regs.bdma_control &= ~0xff00;
				m_adsp_regs.bdma_control |= ((src_addr >> 14) & 0xff) << 8;

				if (m_adsp_regs.bdma_control & 8)
					m_adsp->set_input_line(INPUT_LINE_RESET, PULSE_LINE);
			}
			break;
		}
		case 5:
			osd_printf_debug("PFLAGS: %x\n", data);
			break;
		default:
			osd_printf_debug("Unhandled register: %x %x\n", 0x3fe0 + offset, data);
	}
}


/*************************************
 *
 *  Main CPU
 *
 *************************************/

static ADDRESS_MAP_START( magictg_map, AS_PROGRAM, 32, magictg_state )
	AM_RANGE(0x00000000, 0x007fffff) AM_RAM // 8MB RAM
	AM_RANGE(0x00800000, 0x0081003f) AM_RAM // ?
	AM_RANGE(0x0a000000, 0x0affffff) AM_DEVREADWRITE("voodoo_0", voodoo_device, voodoo_r, voodoo_w)
#if defined(USE_TWO_3DFX)
	AM_RANGE(0x0b000000, 0x0bffffff) AM_DEVREADWRITE("voodoo_1", voodoo_device, voodoo_r, voodoo_w)
	AM_RANGE(0x0c000000, 0x0c000fff) AM_READWRITE(zr36120_r, zr36120_w)
#else
	AM_RANGE(0x0b000000, 0x0b000fff) AM_READWRITE(zr36120_r, zr36120_w)
#endif
	AM_RANGE(0x0f000000, 0x0f000fff) AM_READWRITE(f0_r, f0_w) // Split this up?
	AM_RANGE(0x14000100, 0x14000103) AM_READWRITE(adsp_idma_data_r, adsp_idma_data_w)
	AM_RANGE(0x14000104, 0x14000107) AM_WRITE(adsp_idma_addr_w)
	AM_RANGE(0x1b001024, 0x1b001027) AM_READ(adsp_status_r)
	AM_RANGE(0x1b001108, 0x1b00110b) AM_READ(unk_r)
	AM_RANGE(0x1e000000, 0x1e002fff) AM_RAM // NVRAM?
	AM_RANGE(0x1e800000, 0x1e800007) AM_READWRITE(unk2_r, serial_w)
	AM_RANGE(0x1fc00000, 0x1fffffff) AM_ROM AM_REGION("mips", 0)
ADDRESS_MAP_END


/*************************************
 *
 *  Mad Cow (IO/sound)
 *
 *************************************/

static ADDRESS_MAP_START( adsp_program_map, AS_PROGRAM, 32, magictg_state )
	ADDRESS_MAP_UNMAP_HIGH
	AM_RANGE(0x0000, 0x3fff) AM_RAM AM_SHARE("adsp_pram")
ADDRESS_MAP_END

static ADDRESS_MAP_START( adsp_data_map, AS_DATA, 16, magictg_state )
	ADDRESS_MAP_UNMAP_HIGH
//  AM_RANGE(0x0000, 0x03ff) AM_RAMBANK("databank")
	AM_RANGE(0x0400, 0x3fdf) AM_RAM
	AM_RANGE(0x3fe0, 0x3fff) AM_READWRITE(adsp_control_r, adsp_control_w)
ADDRESS_MAP_END

static ADDRESS_MAP_START( adsp_io_map, AS_IO, 16, magictg_state )
	ADDRESS_MAP_UNMAP_HIGH
ADDRESS_MAP_END


/*************************************
 *
 *  Input ports
 *
 *************************************/

static INPUT_PORTS_START( magictg )
	PORT_START("IPT_TEST")
INPUT_PORTS_END


/*************************************
 *
 *  Machine driver
 *
 *************************************/

static MACHINE_CONFIG_START( magictg )
	MCFG_CPU_ADD("mips", R5000BE, 150000000) /* TODO: CPU type and clock are unknown */
	//MCFG_MIPS3_ICACHE_SIZE(16384) /* TODO: Unknown */
	//MCFG_MIPS3_DCACHE_SIZE(16384) /* TODO: Unknown */
	MCFG_CPU_PROGRAM_MAP(magictg_map)

	MCFG_CPU_ADD("adsp", ADSP2181, 16000000)
	MCFG_CPU_PROGRAM_MAP(adsp_program_map)
	MCFG_CPU_DATA_MAP(adsp_data_map)
	MCFG_CPU_IO_MAP(adsp_io_map)

	MCFG_SPEAKER_STANDARD_STEREO("lspeaker", "rspeaker")

	MCFG_SOUND_ADD("dac1", DMADAC, 0)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "rspeaker", 1.0)

	MCFG_SOUND_ADD("dac2", DMADAC, 0)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "lspeaker", 1.0)

	MCFG_PCI_BUS_LEGACY_ADD("pcibus", 0)
	MCFG_PCI_BUS_LEGACY_DEVICE(0, nullptr, pci_dev0_r, pci_dev0_w)
	MCFG_PCI_BUS_LEGACY_DEVICE(7, "voodoo_0", voodoo_0_pci_r, voodoo_0_pci_w)

#if defined(USE_TWO_3DFX)
	MCFG_PCI_BUS_LEGACY_DEVICE(8, "voodoo_1", voodoo_1_pci_r, voodoo_1_pci_w)
#endif
	MCFG_PCI_BUS_LEGACY_DEVICE(9, "zr36120", zr36120_pci_r, zr36120_pci_w)

	MCFG_DEVICE_ADD("voodoo_0", VOODOO_1, STD_VOODOO_1_CLOCK)
	MCFG_VOODOO_FBMEM(2)
	MCFG_VOODOO_TMUMEM(4,0)
	MCFG_VOODOO_SCREEN_TAG("screen")
	MCFG_VOODOO_CPU_TAG("mips")

	MCFG_DEVICE_ADD("voodoo_1", VOODOO_1, STD_VOODOO_1_CLOCK)
	MCFG_VOODOO_FBMEM(2)
	MCFG_VOODOO_TMUMEM(4,0)
	MCFG_VOODOO_SCREEN_TAG("screen")
	MCFG_VOODOO_CPU_TAG("mips")

	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(60)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(0))
	MCFG_SCREEN_SIZE(1024, 1024)
	MCFG_SCREEN_VISIBLE_AREA(0, 511, 16, 447)

	MCFG_SCREEN_UPDATE_DRIVER(magictg_state, screen_update_magictg)
MACHINE_CONFIG_END


/*************************************
 *
 *  ROM definitions
 *
 *************************************/

ROM_START( magictg )
	ROM_REGION32_BE( 0x400000, "mips", 0 )
	ROM_LOAD16_BYTE( "magic.u34", 0x000000, 0x100000, CRC(2e8971e2) SHA1(9bdf433a7c7257389ebdf131317ef26a7d4e1ba2) )
	ROM_LOAD16_BYTE( "magic.u35", 0x000001, 0x100000, CRC(e2202143) SHA1(f07b7da81508cd4594f66e34dabd904a21eb03f0) )
	ROM_LOAD16_BYTE( "magic.u32", 0x200000, 0x100000, CRC(f1d530e3) SHA1(fcc392804cd6b98917a869cc5d3826278b7ba90b) )
	ROM_LOAD16_BYTE( "magic.u33", 0x200001, 0x100000, CRC(b2330cfc) SHA1(559c35426588b349ef31bf8b296b950912f6fcc7) )

	ROM_REGION16_LE( 0x80000, "adsp", 0 )
	ROM_LOAD( "magic.20u", 0x00000, 0x80000, CRC(50968301) SHA1(e9bdd0c942f0c66e18aa8de5a04edb51cdf1fee8) )

	ROM_REGION32_BE( 0x1000000, "adsp_data", 0 )
	ROM_LOAD( "magic.snd0.u8", 0x000000, 0x400000, CRC(3cb81717) SHA1(9d35796381ca57e9782e0338c456e63c31d11266) )
	ROM_LOAD( "magic.snd1.u14",0x400000, 0x400000, CRC(b4ef9977) SHA1(dedc79e5d506bb0d1649a41b9912dcc999e1da72) )
	ROM_LOAD( "magic.snd2.u13",0x800000, 0x400000, CRC(3728f16e) SHA1(6b7da30b100d053e95aa96edf74a0474f1493dfb) )
	ROM_LOAD( "magic.snd3.u7", 0xc00000, 0x400000, CRC(11a1cb63) SHA1(a1048d3cd580747c20eb0b4e816e7e4e0f5c8c2b) )

	ROM_REGION( 0x2000000, "jpeg", 0 )
	ROM_LOAD( "magic_s0.u9",  0x0000000, 0x800000, CRC(a01b5b99) SHA1(e77f2e9b08a97d6118e1e307b38ea79d0177e9b8) )
	ROM_LOAD( "magic_s1.u10", 0x0800000, 0x800000, CRC(d5a1a557) SHA1(2511ee8d08da765a2fa2d42fb504793f9e8b615c) )
	ROM_LOAD( "magic_s2.u12", 0x1000000, 0x800000, CRC(06ed6770) SHA1(884a3e4c97a50fa926546eb6def2c11c5732ba88) )
	ROM_LOAD( "magic_s3.u11", 0x1800000, 0x800000, CRC(71d4c252) SHA1(aeab2542b9d5fb63f4d60b808010a657a895c1d7) )

	ROM_REGION( 0x400000, "key", 0 )
	ROM_LOAD( "magic.k0.u20", 0x000000, 0x400000, CRC(63ab0e9e) SHA1(c4f0b009860ee499496ed7fc1f14ef1e221c1085) )
ROM_END

ROM_START( magictga )
	ROM_REGION32_BE( 0x400000, "mips", 0 )
	ROM_LOAD16_BYTE( "magic.u63", 0x000000, 0x100000, CRC(a10d45f1) SHA1(0ede10f19cf70baf7b43e3f672532b4be1a179f8) )
	ROM_LOAD16_BYTE( "magic.u64", 0x000001, 0x100000, CRC(8fdb6060) SHA1(b638244cad86dc60435a4a9150a5b639f5d61a3f) )
	ROM_LOAD16_BYTE( "magic.u61", 0x200000, 0x100000, CRC(968891d6) SHA1(67ab87039864bb148d20795333ffa7a23e3b84f2) )
	ROM_LOAD16_BYTE( "magic.u62", 0x200001, 0x100000, CRC(690946eb) SHA1(6c9b02367704309f4fde5cbd9d195a45c32c3861) )

	// this set was incomplete, none of these roms were dumped for it, are they the same?
	ROM_REGION32_BE( 0x80000, "adsp", 0 )
	ROM_LOAD( "magic.20u", 0x00000, 0x80000, BAD_DUMP CRC(50968301) SHA1(e9bdd0c942f0c66e18aa8de5a04edb51cdf1fee8) )

	ROM_REGION32_BE( 0x1000000, "adsp_data", 0 )
	ROM_LOAD( "magic.snd0.u8", 0x000000, 0x400000, BAD_DUMP CRC(3cb81717) SHA1(9d35796381ca57e9782e0338c456e63c31d11266) )
	ROM_LOAD( "magic.snd1.u14",0x400000, 0x400000, BAD_DUMP CRC(b4ef9977) SHA1(dedc79e5d506bb0d1649a41b9912dcc999e1da72) )
	ROM_LOAD( "magic.snd2.u13",0x800000, 0x400000, BAD_DUMP CRC(3728f16e) SHA1(6b7da30b100d053e95aa96edf74a0474f1493dfb) )
	ROM_LOAD( "magic.snd3.u7", 0xc00000, 0x400000, BAD_DUMP CRC(11a1cb63) SHA1(a1048d3cd580747c20eb0b4e816e7e4e0f5c8c2b) )

	ROM_REGION( 0x2000000, "jpeg", 0 )
	ROM_LOAD( "magic_s0.u9",  0x0000000, 0x800000, BAD_DUMP CRC(a01b5b99) SHA1(e77f2e9b08a97d6118e1e307b38ea79d0177e9b8) )
	ROM_LOAD( "magic_s1.u10", 0x0800000, 0x800000, BAD_DUMP CRC(d5a1a557) SHA1(2511ee8d08da765a2fa2d42fb504793f9e8b615c) )
	ROM_LOAD( "magic_s2.u12", 0x1000000, 0x800000, BAD_DUMP CRC(06ed6770) SHA1(884a3e4c97a50fa926546eb6def2c11c5732ba88) )
	ROM_LOAD( "magic_s3.u11", 0x1800000, 0x800000, BAD_DUMP CRC(71d4c252) SHA1(aeab2542b9d5fb63f4d60b808010a657a895c1d7) )

	ROM_REGION( 0x400000, "key", 0 )
	ROM_LOAD( "magic.k0.u20", 0x000000, 0x400000, BAD_DUMP CRC(63ab0e9e) SHA1(c4f0b009860ee499496ed7fc1f14ef1e221c1085) )
ROM_END


/*************************************
 *
 *  Game driver(s)
 *
 *************************************/

GAME( 1997, magictg,  0,       magictg, magictg, magictg_state, 0, ROT0, "Acclaim", "Magic the Gathering: Armageddon (set 1)", MACHINE_NOT_WORKING | MACHINE_NO_SOUND )
GAME( 1997, magictga, magictg, magictg, magictg, magictg_state, 0, ROT0, "Acclaim", "Magic the Gathering: Armageddon (set 2)", MACHINE_NOT_WORKING | MACHINE_NO_SOUND )
