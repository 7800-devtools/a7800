// license:BSD-3-Clause
// copyright-holders:Fabio Priuli
/***********************************************************************************************************


 MegaDrive / Genesis J-Cart (+SEPROM) emulation


 Emulation based on earlier researches by ElBarto


 i2c games mapping table:

 game name                         |   SDA_IN   |  SDA_OUT   |     SCL    |  SIZE_MASK     | PAGE_MASK |
 ----------------------------------|------------|------------|------------|----------------|-----------|
 Micro Machines 2                  | 0x380001-7 | 0x300000-0*| 0x300000-1*| 0x03ff (24C08) |   0x0f    |
 Micro Machines Military           | 0x380001-7 | 0x300000-0*| 0x300000-1*| 0x03ff (24C08) |   0x0f    |
 Micro Machines 96                 | 0x380001-7 | 0x300000-0*| 0x300000-1*| 0x07ff (24C16) |   0x0f    |
 ----------------------------------|------------|------------|------------|----------------|-----------|

 * Notes: check these


 TODO: proper SEPROM emulation, still not worked on (just hooked up the I2C device)

***********************************************************************************************************/



#include "emu.h"
#include "jcart.h"


//-------------------------------------------------
//  md_rom_device - constructor
//-------------------------------------------------

DEFINE_DEVICE_TYPE(MD_JCART,           md_jcart_device,           "md_jcart",           "MD J-Cart games")
DEFINE_DEVICE_TYPE(MD_SEPROM_CODEMAST, md_seprom_codemast_device, "md_seprom_codemast", "MD J-Cart games + SEPROM")
DEFINE_DEVICE_TYPE(MD_SEPROM_MM96,     md_seprom_mm96_device,     "md_seprom_mm96",     "MD Micro Machine 96")

// Sampras, Super Skidmarks?
md_jcart_device::md_jcart_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, type, tag, owner, clock)
	, device_md_cart_interface(mconfig, *this)
	, m_jcart3(*this, "JCART3")
	, m_jcart4(*this, "JCART4")
{
}

md_jcart_device::md_jcart_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: md_jcart_device(mconfig, MD_JCART, tag, owner, clock)
{
}

// Micro Machines 2, Micro Machines Military
md_seprom_codemast_device::md_seprom_codemast_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock)
	: md_jcart_device(mconfig, type, tag, owner, clock)
	, m_i2cmem(*this, "i2cmem"), m_i2c_mem(0), m_i2c_clk(0)
{
}

md_seprom_codemast_device::md_seprom_codemast_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: md_seprom_codemast_device(mconfig, MD_SEPROM_CODEMAST, tag, owner, clock)
{
}

// Micro Machines 96
md_seprom_mm96_device::md_seprom_mm96_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: md_seprom_codemast_device(mconfig, MD_SEPROM_MM96, tag, owner, clock)
{
}


//-------------------------------------------------
//  SERIAL I2C DEVICE
//-------------------------------------------------

// MD_SEPROM_CODEMAST


// MD_SEPROM_MM96
MACHINE_CONFIG_START( md_i2c_24c16a )
	MCFG_24C16A_ADD("i2cmem")
MACHINE_CONFIG_END


//-------------------------------------------------
//  device_add_mconfig - add device configuration
//-------------------------------------------------

MACHINE_CONFIG_MEMBER( md_seprom_codemast_device::device_add_mconfig )
	MCFG_24C08_ADD("i2cmem")
MACHINE_CONFIG_END

MACHINE_CONFIG_MEMBER( md_seprom_mm96_device::device_add_mconfig )
	MCFG_24C16A_ADD("i2cmem")
MACHINE_CONFIG_END


static INPUT_PORTS_START( jcart_ipt )

	PORT_START("JCART3")     /* Joypad 3 on J-Cart (3 button + start) */
	PORT_BIT( 0x0001, IP_ACTIVE_LOW, IPT_JOYSTICK_UP ) PORT_8WAY PORT_PLAYER(3)
	PORT_BIT( 0x0002, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN ) PORT_8WAY PORT_PLAYER(3)
	PORT_BIT( 0x0004, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT ) PORT_8WAY PORT_PLAYER(3)
	PORT_BIT( 0x0008, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT ) PORT_8WAY PORT_PLAYER(3)
	PORT_BIT( 0x0010, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_PLAYER(3) PORT_NAME("P3 B")
	PORT_BIT( 0x0020, IP_ACTIVE_LOW, IPT_BUTTON3 ) PORT_PLAYER(3) PORT_NAME("P3 C")
	PORT_BIT( 0x0040, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_PLAYER(3) PORT_NAME("P3 A")
	PORT_BIT( 0x0080, IP_ACTIVE_LOW, IPT_START ) PORT_PLAYER(3)

	PORT_START("JCART4")     /* Joypad 4 on J-Cart (3 button + start) */
	PORT_BIT( 0x0001, IP_ACTIVE_LOW, IPT_JOYSTICK_UP ) PORT_8WAY PORT_PLAYER(4)
	PORT_BIT( 0x0002, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN ) PORT_8WAY PORT_PLAYER(4)
	PORT_BIT( 0x0004, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT ) PORT_8WAY PORT_PLAYER(4)
	PORT_BIT( 0x0008, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT ) PORT_8WAY PORT_PLAYER(4)
	PORT_BIT( 0x0010, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_PLAYER(4) PORT_NAME("P4 B")
	PORT_BIT( 0x0020, IP_ACTIVE_LOW, IPT_BUTTON3 ) PORT_PLAYER(4) PORT_NAME("P4 C")
	PORT_BIT( 0x0040, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_PLAYER(4) PORT_NAME("P4 A")
	PORT_BIT( 0x0080, IP_ACTIVE_LOW, IPT_START ) PORT_PLAYER(4)

INPUT_PORTS_END

ioport_constructor md_jcart_device::device_input_ports() const
{
	return INPUT_PORTS_NAME( jcart_ipt );
}


void md_jcart_device::device_start()
{
	save_item(NAME(m_jcart_io_data));
}

void md_jcart_device::device_reset()
{
	m_jcart_io_data[0] = 0;
	m_jcart_io_data[1] = 0;
}

void md_seprom_codemast_device::device_start()
{
	save_item(NAME(m_i2c_mem));
	save_item(NAME(m_i2c_clk));
	save_item(NAME(m_jcart_io_data));
}

void md_seprom_codemast_device::device_reset()
{
	m_i2c_mem = 0;
	m_i2c_clk = 0;
	m_jcart_io_data[0] = 0;
	m_jcart_io_data[1] = 0;
}


/*-------------------------------------------------
 mapper specific handlers
 -------------------------------------------------*/

/*-------------------------------------------------
 J-CART ONLY (Pete Sampras Tennis)
 -------------------------------------------------*/

READ16_MEMBER(md_jcart_device::read)
{
	if (offset == 0x38fffe/2)
	{
		uint8_t joy[2];

		if (m_jcart_io_data[0] & 0x40)
		{
			joy[0] = m_jcart3.read_safe(0);
			joy[1] = m_jcart4.read_safe(0);
			return (m_jcart_io_data[0] & 0x40) | joy[0] | (joy[1] << 8);
		}
		else
		{
			joy[0] = ((m_jcart3.read_safe(0) & 0xc0) >> 2) | (m_jcart3.read_safe(0) & 0x03);
			joy[1] = ((m_jcart4.read_safe(0) & 0xc0) >> 2) | (m_jcart4.read_safe(0) & 0x03);
			return (m_jcart_io_data[0] & 0x40) | joy[0] | (joy[1] << 8);
		}
	}
	if (offset < 0x400000/2)
		return m_rom[MD_ADDR(offset)];
	else
		return 0xffff;
}

WRITE16_MEMBER(md_jcart_device::write)
{
	if (offset == 0x38fffe/2)
	{
		m_jcart_io_data[0] = (data & 1) << 6;
		m_jcart_io_data[1] = (data & 1) << 6;
	}
}

/*-------------------------------------------------
 J-CART + SEPROM
 -------------------------------------------------*/

READ16_MEMBER(md_seprom_codemast_device::read)
{
	if (offset == 0x380000/2)
	{
		m_i2c_mem = m_i2cmem->read_sda();
		return (m_i2c_mem & 1) << 7;
	}
	if (offset == 0x38fffe/2)
	{
		uint8_t joy[2];

		if (m_jcart_io_data[0] & 0x40)
		{
			joy[0] = m_jcart3.read_safe(0);
			joy[1] = m_jcart4.read_safe(0);
			return (m_jcart_io_data[0] & 0x40) | joy[0] | (joy[1] << 8);
		}
		else
		{
			joy[0] = ((m_jcart3.read_safe(0) & 0xc0) >> 2) | (m_jcart3.read_safe(0) & 0x03);
			joy[1] = ((m_jcart4.read_safe(0) & 0xc0) >> 2) | (m_jcart4.read_safe(0) & 0x03);
			return (m_jcart_io_data[0] & 0x40) | joy[0] | (joy[1] << 8);
		}
	}
	if (offset < 0x400000/2)
		return m_rom[MD_ADDR(offset)];
	else
		return 0xffff;
}

WRITE16_MEMBER(md_seprom_codemast_device::write)
{
	if (offset == 0x380000/2)
	{
		m_i2c_clk = BIT(data, 9);
		m_i2c_mem = BIT(data, 8);
		m_i2cmem->write_scl(m_i2c_clk);
		m_i2cmem->write_sda(m_i2c_mem);
	}
	if (offset == 0x38fffe/2)
	{
		m_jcart_io_data[0] = (data & 1) << 6;
		m_jcart_io_data[1] = (data & 1) << 6;
	}
}
