// license:BSD-3-Clause
// copyright-holders:Olivier Galibert

#include "emu.h"
#include "naomim2.h"

/********************************************************************************************************

Naomi cartridge type M2/3 mapping
---------------------------------

NAOMI_ROM_OFFSET bit29: ROM size/mapping selection, 0 - 4MB ROM mode, 1 - 8MB ROM mode

bit28: master/slave ROM board select.
  flash-based 171-7885A ROM board JP4 select master (access at 0xxxxxxx) or slave (access at 1xxxxxxx) mode.
  then set to slave it can be stacked with another type M2/3 ROM board.
  this bit selects which one ROM board will be accessed.

note: if ROM is not mounted its area readed as 0xFF

8MB ROM mode:
+---------+----------------------------------------------------------------------------------------------+
|         |                  Cart PCB type                                                               |
| Address +-------------------+----------------------+---------------------------------------------------+
|         |  171-7919A        |  171-8132B           |  NAMCO                                            |
+---------+-------------------+----------------------+---------------------------------------------------+
|00000000 | ROM0  IC22*1(4MB) | IC22(ROM0)*1  (4MB)  | 2F FL0  (8MB)                                     |
|00400000 | FF filled   (4MB) | FF filled     (4MB)  |                                                   |
|00800000 | ROM1  IC1   (8MB) | IC1           (16MB) | 2D FL1  (8MB)                                     |
|01000000 | ROM2  IC2   (8MB) |                      | 2C FL2  (8MB) \ or 4N MA1 (16MB)                  |
|01800000 | ROM3  IC3   (8MB) | IC2           (16MB) | 2B FL3  (8MB) /                                   |
|02000000 | ROM4  IC4   (8MB) |                      | 4M MA2  (16MB)                                    |
|02800000 | ROM5  IC5   (8MB) | IC3           (16MB) |                                                   |
|03000000 | ROM6  IC6   (8MB) |                      | 4L MA3  (16MB)                                    |
|03800000 | ROM7  IC7   (8MB) | IC4           (16MB) |                                                   |
|04000000 | ROM8  IC8   (8MB) |                      | 4K MA4  (16MB)                                    |
|04800000 | ROM9  IC9   (8MB) | IC5           (16MB) |                                                   |
|05000000 | ROM10 IC10  (8MB) |                      | 4J MA5  (16MB)                                    |
|05800000 | ROM11 IC11  (8MB) | IC6           (16MB) |                                                   |
|06000000 | ROM12 IC12S (8MB) |                      | 4H MA6  (16MB)                                    |
|06800000 | ROM13 IC13S (8MB) | IC7           (16MB) |                                                   |
|07000000 | ROM14 IC14S (8MB) |                      | 4F MA7  (16MB)                                    |
|07800000 | ROM15 IC15S (8MB) | IC8           (16MB) |                                                   |
|08000000 | ROM16 IC16S (8MB) |                      | 4E MA8  (16MB)                                    |
|08800000 | ROM17 IC17S (8MB) | IC9           (16MB) |                                                   |
|09000000 | ROM18 IC18S (8MB) |                      | 4D MA9  (16MB)                                    |
|09800000 | ROM19 IC19S (8MB) | IC10          (16MB) |                                                   |
|0A000000 | ROM20 IC20S (8MB) |                      | 4C MA10 (16MB)                                    |
|0A800000 | ROM21 IC21S (8MB) | IC11          (16MB) |                                                   |
|0B000000 | FF filled area    | FF filled area       | 4B MA11 (16MB)                                    |
|0C000000 |                   |                      | 6P MA12 (16MB)                                    |
|0D000000 |                   |                      | 6N MA13 (16MB)                                    |
|0E000000 |                   |                      | 6M MA14 (16MB)                                    |
|0F000000 |                   |                      | 6L MA15 (16MB)                                    |
|10000000 | Slave ROM board   | Slave ROM board      | 6K MA16 (16MB)                                    |
|11000000 | area              | area                 | 6J MA17 (16MB)                                    |
|12000000 |                   |                      | 6H MA18 (16MB)                                    |
|13000000 |                   |                      | 6F MA19 (16MB)                                    |
|14000000 |                   |                      | 6E MA20 (16MB)                                    |
|15000000 |                   |                      | 6D MA21 (16MB)                                    |
|16000000 |                   |                      | 6C MA22 (16MB)                                    |
|17000000 |                   |                      | 6B MA23 (16MB)                                    |
+---------+-------------------+----------------------+---------------------------------------------------+
*1 in the case 2MB IC22 it will be mirrored

4MB ROM mode:
+---------+----------------------------------------------------------------------------------------------+
|         |                  Cart PCB type                                                               |
| Address +-------------------+---------------------+----------------------------------------------------+
|         |  171-7919A        |  171-8132B          |  NAMCO                                             |
+---------+-------------------+---------------------+----------------------------------------------------+
|00000000 | ROM0  IC22  (4MB) | IC22(ROM0)    (4MB) | 2F FL0           (4MB)                             |
|00400000 | ROM1  IC1   (4MB) | IC1           (4MB) | 2D FL1           (4MB)                             |
|00800000 | ROM2  IC2   (4MB) | IC1  2nd half (4MB) | 2C FL2           (4MB) or 4N MA1          (4MB)    |
|00C00000 | ROM3  IC3   (4MB) | IC2           (4MB) | 2B FL3           (4MB) or 4N MA1 2nd half (4MB)    |
|01000000 | ROM4  IC4   (4MB) | IC2  2nd half (4MB) | 4M MA2           (4MB)                             |
|01400000 | ROM5  IC5   (4MB) | IC3           (4MB) | 4M MA2  2nd half (4MB)                             |
|01800000 | ROM6  IC6   (4MB) | IC3  2nd half (4MB) | 4L MA3           (4MB)                             |
|01C00000 | ROM7  IC7   (4MB) | IC4           (4MB) | 4L MA3  2nd half (4MB)                             |
|02000000 | ROM8  IC8   (4MB) | IC4  2nd half (4MB) | 4K MA4           (4MB)                             |
|02400000 | ROM9  IC9   (4MB) | IC5           (4MB) | 4K MA4  2nd half (4MB)                             |
|02800000 | ROM10 IC10  (4MB) | IC5  2nd half (4MB) | 4J MA5           (4MB)                             |
|02C00000 | ROM11 IC11  (4MB) | IC6           (4MB) | 4J MA5  2nd half (4MB)                             |
|03000000 | ROM12 IC12S (4MB) | IC6  2nd half (4MB) | 4H MA6           (4MB)                             |
|03400000 | ROM13 IC13S (4MB) | IC7           (4MB) | 4H MA6  2nd half (4MB)                             |
|03800000 | ROM14 IC14S (4MB) | IC7  2nd half (4MB) | 4F MA7           (4MB)                             |
|03C00000 | ROM15 IC15S (4MB) | IC8           (4MB) | 4F MA7  2nd half (4MB)                             |
|04000000 | ROM16 IC16S (4MB) | IC8  2nd half (4MB) | 4E MA8           (4MB)                             |
|04400000 | ROM17 IC17S (4MB) | IC9           (4MB) | 4E MA8  2nd half (4MB)                             |
|04800000 | ROM18 IC18S (4MB) | IC9  2nd half (4MB) | 4D MA9           (4MB)                             |
|04C00000 | ROM19 IC19S (4MB) | IC10          (4MB) | 4D MA9  2nd half (4MB)                             |
|05000000 | ROM20 IC20S (4MB) | IC10 2nd half (4MB) | 4C MA10          (4MB)                             |
|05400000 | ROM21 IC21S (4MB) | IC11          (4MB) | 4C MA10 2nd half (4MB)                             |
|05800000 | FF filled area    | FF filled area      | 4B MA11          (4MB)                             |
|05C00000 |                   |                     | 4B MA11 2nd half (4MB)                             |
|06000000 |                   |                     | 6P MA12          (4MB)                             |
|06400000 |                   |                     | 6P MA12 2nd half (4MB)                             |
|06800000 |                   |                     | 6N MA13          (4MB)                             |
|06C00000 |                   |                     | 6N MA13 2nd half (4MB)                             |
|07000000 |                   |                     | 6M MA14          (4MB)                             |
|07400000 |                   |                     | 6M MA14 2nd half (4MB)                             |
|07800000 |                   |                     | 6L MA15          (4MB)                             |
|07C00000 |                   |                     | 6L MA15 2nd half (4MB)                             |
+---------+-------------------+---------------------+----------------------------------------------------+
|08000000 | mirror    (128MB) | mirror      (128MB) | mirror         (128MB)                             |
+---------+-------------------+---------------------+----------------------------------------------------+
|10000000 | slave ROM board   | slave ROM board     | FF filled      (256MB) (or MA16-23 in 4MB mode?)   |
+---------+-------------------+---------------------+----------------------------------------------------+

********************************************************************************************************/

DEFINE_DEVICE_TYPE(NAOMI_M2_BOARD, naomi_m2_board, "naomi_m2_board", "Sega NAOMI M2 Board")

naomi_m2_board::naomi_m2_board(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: naomi_board(mconfig, NAOMI_M2_BOARD, tag, owner, clock)
	, m_cryptdevice(*this, "segam2crypt")
	, m_region(*this, DEVICE_SELF)
{
}

void naomi_m2_board::device_start()
{
	naomi_board::device_start();

	ram = std::make_unique<uint8_t[]>(RAM_SIZE);

	save_item(NAME(rom_cur_address));
	save_pointer(NAME(ram.get()), RAM_SIZE);
}

void naomi_m2_board::device_reset()
{
	naomi_board::device_reset();

	memset(ram.get(), 0, RAM_SIZE);

	rom_cur_address = 0;
}

void naomi_m2_board::board_setup_address(uint32_t address, bool is_dma)
{
	rom_cur_address = address;
}

void naomi_m2_board::board_get_buffer(uint8_t *&base, uint32_t &limit)
{
	if(rom_cur_address & 0x40000000) {
		if(rom_cur_address == 0x4001fffe) {
			m_cryptdevice->do_decrypt(base);
			limit = 2;

		} else
			throw emu_fatalerror("NAOMIM2: Unsupported, read from %08x", rom_cur_address);

	} else {
		if (rom_offset & 0x20000000) {
			base = m_region->base() + (rom_cur_address & 0x1fffffff);
			limit = m_region->bytes() - (rom_cur_address & 0x1fffffff);
		} else {
			uint32_t offset4mb = (rom_cur_address & 0x103FFFFF) | ((rom_cur_address & 0x07C00000) << 1);
			base = m_region->base() + offset4mb;
			limit = std::min(m_region->bytes() - offset4mb, 0x00400000 - (offset4mb & 0x003FFFFF));
		}
	}
}

void naomi_m2_board::board_advance(uint32_t size)
{
	rom_cur_address += size;
}

void naomi_m2_board::board_write(offs_t offset, uint16_t data)
{
	if(offset & 0x40000000) {
		if(offset & 0x00020000) {
			offset &= RAM_SIZE-1;
			ram[offset] = data;
			ram[offset+1] = data >> 8;
			return;
		}
		switch(offset & 0x1ffff) {
		case 0x1fff8: m_cryptdevice->set_addr_low(data); return;
		case 0x1fffa: m_cryptdevice->set_addr_high(data);  return;
		case 0x1fffc: m_cryptdevice->set_subkey(data); return;
		}
	}
	logerror("NAOMIM2: unhandled board write %08x, %04x\n", offset, data);
}

uint16_t naomi_m2_board::read_callback(uint32_t addr)
{
	if ((addr & 0xffff0000) == 0x01000000) {
		int base = 2*(addr & 0x7fff);
		return ram[base+1] | (ram[base] << 8);

	}
	else {
		const uint8_t *base = &m_region->as_u8(2*addr);
		return base[1] | (base[0] << 8);
	}
}

MACHINE_CONFIG_MEMBER( naomi_m2_board::device_add_mconfig )
	MCFG_DEVICE_ADD("segam2crypt", SEGA315_5881_CRYPT, 0)
	MCFG_SET_READ_CALLBACK(naomi_m2_board, read_callback)
MACHINE_CONFIG_END
