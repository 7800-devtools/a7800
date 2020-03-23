// license:LGPL-2.1+
// copyright-holders:Angelo Salese
/***************************************************************************

    Fujitsu MB90082 OSD

    preliminary device by Angelo Salese

    TODO:
    - get a real charset ROM;

***************************************************************************/

#include "emu.h"
#include "video/mb90082.h"



//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

// device type definition
DEFINE_DEVICE_TYPE(MB90082, mb90082_device, "mb90082", "Fujitsu MB90082 OSD")

static ADDRESS_MAP_START( mb90082_vram, 0, 16, mb90082_device )
	AM_RANGE(0x0000, 0x023f) AM_RAM // main screen vram
	AM_RANGE(0x0400, 0x063f) AM_RAM // main screen attr
//  AM_RANGE(0x0800, 0x0a3f) AM_RAM // sub screen vram
//  AM_RANGE(0x0c00, 0x0e3f) AM_RAM // sub screen attr
ADDRESS_MAP_END

/* charset is undumped, but apparently a normal ASCII one is enough for the time being (for example "fnt0808.x1" in Sharp X1) */
ROM_START( mb90082 )
	ROM_REGION( 0x2000, "mb90082", ROMREGION_ERASEFF )
	ROM_LOAD("mb90082_char.bin",     0x0000, 0x0800, NO_DUMP )
ROM_END

//-------------------------------------------------
//  rom_region - device-specific ROM region
//-------------------------------------------------

const tiny_rom_entry *mb90082_device::device_rom_region() const
{
	return ROM_NAME( mb90082 );
}

//-------------------------------------------------
//  memory_space_config - return a description of
//  any address spaces owned by this device
//-------------------------------------------------

device_memory_interface::space_config_vector mb90082_device::memory_space_config() const
{
	return space_config_vector {
		std::make_pair(0, &m_space_config)
	};
}

//**************************************************************************
//  INLINE HELPERS
//**************************************************************************

//-------------------------------------------------
//  readbyte - read a byte at the given address
//-------------------------------------------------

inline uint16_t mb90082_device::read_word(offs_t address)
{
	return space().read_word(address << 1);
}

//-------------------------------------------------
//  writebyte - write a byte at the given address
//-------------------------------------------------

inline void mb90082_device::write_word(offs_t address, uint16_t data)
{
	space().write_word(address << 1, data);
}

//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  mb90082_device - constructor
//-------------------------------------------------

mb90082_device::mb90082_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, MB90082, tag, owner, clock)
	, device_memory_interface(mconfig, *this)
	, m_space_config("videoram", ENDIANNESS_LITTLE, 16, 16, 0, nullptr, *ADDRESS_MAP_NAME(mb90082_vram))
{
}


//-------------------------------------------------
//  device_validity_check - perform validity checks
//  on this device
//-------------------------------------------------

void mb90082_device::device_validity_check(validity_checker &valid) const
{
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void mb90082_device::device_start()
{
}


//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void mb90082_device::device_reset()
{
	m_cmd_ff = 0;
}


//**************************************************************************
//  READ/WRITE HANDLERS
//**************************************************************************

WRITE_LINE_MEMBER( mb90082_device::set_cs_line )
{
	m_reset_line = state;

	if(m_reset_line != CLEAR_LINE)
	{
		// ...
	}
}


WRITE8_MEMBER( mb90082_device::write )
{
	uint16_t dat;

	switch(m_cmd_ff)
	{
		case OSD_COMMAND:
			m_cmd = data & 0xf8;
			m_cmd_param = data & 7;
			//printf("cmd %02x\n",data);
			break;
		case OSD_DATA:
			dat = ((m_cmd_param & 7)<<7) | (data & 0x7f);
			switch(m_cmd)
			{
				case 0x80: // Preset VRAM address
					m_osd_addr = dat & 0x1ff;
					m_fil = (dat & 0x200) >> 9;
					break;
				case 0x88: // Attribute select
					m_attr = dat;
					break;
				case 0x90: // Write Character
					int x,y;
					x = (m_osd_addr & 0x01f);
					y = (m_osd_addr & 0x1e0) >> 5;

					if(m_fil)
					{
						int i;
						if(x != 0)
							printf("FIL with %d %d\n",x,y);

						for(i=0;i<24;i++)
						{
							write_word((i+y*24)|0x000,dat);
							write_word((i+y*24)|0x200,m_attr);
						}
					}
					else
					{
						write_word((x+y*24)|0x000,dat);
						write_word((x+y*24)|0x200,m_attr);

						/* handle address increments */
						x = ((x + 1) % 24);
						if(x == 0)
							y = ((y + 1) % 12);
						m_osd_addr = x + (y << 5);
					}
					break;
				case 0xd0: // Set Under Color
					m_uc = dat & 7;
					break;

			}
			break;
	}

	m_cmd_ff ^= 1;
}

uint32_t mb90082_device::screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
	int x,y;
	uint8_t *pcg = memregion("mb90082")->base();
	uint16_t tile,attr;
	uint8_t bg_r,bg_g,bg_b;

	/* TODO: there's probably a way to control the brightness in this */
	bg_b = m_uc & 1 ? 0xdf : 0;
	bg_g = m_uc & 2 ? 0xdf : 0;
	bg_r = m_uc & 4 ? 0xdf : 0;
	bitmap.fill(rgb_t(0xff,bg_r,bg_g,bg_b),cliprect);

	for(y=0;y<12;y++)
	{
		for(x=0;x<24;x++)
		{
			int xi,yi;

			tile = read_word(x+y*24);
			attr = read_word((x+y*24)|0x200);

			/* TODO: charset hook-up is obviously WRONG so following mustn't be trusted at all */
			for(yi=0;yi<16;yi++)
			{
				for(xi=0;xi<16;xi++)
				{
					uint8_t pix;
					uint8_t color = (attr & 0x70) >> 4;
					uint8_t r,g,b;

					pix = (pcg[(tile*8)+(yi >> 1)] >> (7-(xi >> 1))) & 1;

					/* TODO: check this */
					b = (color & 1) ? 0xff : 0;
					g = (color & 2) ? 0xff : 0;
					r = (color & 4) ? 0xff : 0;

					if(tile != 0xff && pix != 0)
						bitmap.pix32(y*16+yi,x*16+xi) = r << 16 | g << 8 | b;
				}
			}
		}
	}

	return 0;
}
