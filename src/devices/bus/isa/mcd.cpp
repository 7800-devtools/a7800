// license:BSD-3-Clause
// copyright-holders:Carl

#include "emu.h"
#include "mcd.h"
#include "coreutil.h"

DEVICE_ADDRESS_MAP_START(map, 16, mcd_isa_device)
	AM_RANGE(0x0, 0x1) AM_READWRITE8(data_r, cmd_w, 0x00ff)
	AM_RANGE(0x0, 0x1) AM_READWRITE8(flag_r, reset_w, 0xff00)
ADDRESS_MAP_END

static INPUT_PORTS_START( ide )
INPUT_PORTS_END

//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

DEFINE_DEVICE_TYPE(ISA16_MCD, mcd_isa_device, "mcd_isa", "Mitsumi ISA CD-ROM Adapter")

//-------------------------------------------------
//  input_ports - device-specific input ports
//-------------------------------------------------

ioport_constructor mcd_isa_device::device_input_ports() const
{
	return INPUT_PORTS_NAME( ide );
}

//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  mcd_isa_device - constructor
//-------------------------------------------------

mcd_isa_device::mcd_isa_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	cdrom_image_device(mconfig, ISA16_MCD, tag, owner, clock),
	device_isa16_card_interface( mconfig, *this )
{
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void mcd_isa_device::device_start()
{
	cdrom_image_device::device_start();
	set_isa_device();
	m_isa->set_dma_channel(5, this, false);
	m_isa->install_device(0x0310, 0x0311, *this, &mcd_isa_device::map, 16);
}

//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void mcd_isa_device::device_reset()
{
	m_stat = m_cdrom_handle ? STAT_READY | STAT_CHANGE : 0;
	m_cmdrd_count = 0;
	m_cmdbuf_count = 0;
	m_buf_count = 0;
	m_curtoctrk = 1;
	m_dma = 0;
	m_irq = 0;
	m_conf = 0;
	m_dmalen = 2048;
	m_locked = false;
	m_change = true;
	m_newstat = true;
	m_data = false;
}

bool mcd_isa_device::read_sector(bool first)
{
	if((m_irq & IRQ_DATACOMP) && !first)
		m_isa->irq5_w(ASSERT_LINE);
	if(!m_readcount)
	{
		m_isa->drq5_w(CLEAR_LINE);
		m_data = false;
		return false;
	}
	uint32_t lba = msf_to_lba(m_readmsf);
	cdrom_read_data(m_cdrom_handle, lba - 150, m_buf, m_mode & 0x40 ? CD_TRACK_MODE1_RAW : CD_TRACK_MODE1);
	if(m_mode & 0x40)
	{
		//correct the header
		m_buf[12] = dec_2_bcd((m_readmsf >> 16) & 0xff);
		m_buf[13] = dec_2_bcd((m_readmsf >> 8) & 0xff);
	}
	m_readmsf = lba_to_msf_alt(lba + 1);
	m_buf_count = m_dmalen + 1;
	m_buf_idx = 0;
	m_data = true;
	m_readcount--;
	if(m_dma)
		m_isa->drq5_w(ASSERT_LINE);
	if((m_irq & IRQ_DATAREADY) && first)
		m_isa->irq5_w(ASSERT_LINE);
	return true;
}

READ8_MEMBER(mcd_isa_device::flag_r)
{
	uint8_t ret = 0;
	m_isa->irq5_w(CLEAR_LINE);
	if(!m_buf_count || !m_data || m_dma) // if dma enabled the cpu will never not see that flag as it will be halted
		ret |= FLAG_NODATA;
	if(!m_cmdbuf_count || !m_newstat)
		ret |= FLAG_NOSTAT; // all command results are status
	return ret | FLAG_UNK;
}

READ8_MEMBER(mcd_isa_device::data_r)
{
	if(m_cmdbuf_count)
	{
		m_cmdbuf_count--;
		return m_cmdbuf[m_cmdbuf_idx++];
	}
	else if(m_buf_count)
	{
		uint8_t ret = m_buf_idx < 2352 ? m_buf[m_buf_idx++] : 0;
		m_buf_count--;
		if(!m_buf_count)
			read_sector();
		return ret;
	}
	return m_stat;
}

uint16_t mcd_isa_device::dack16_r(int line)
{
	if(m_buf_count & ~1)
	{
		uint16_t ret = 0;
		if(m_buf_idx < 2351)
		{
			ret = m_buf[m_buf_idx++];
			ret |= (m_buf[m_buf_idx++] << 8);
		}
		m_buf_count -= 2;
		if(!m_buf_count)
			read_sector();
		return ret;
	}
	return 0;
}

WRITE8_MEMBER(mcd_isa_device::reset_w)
{
	reset();
}

WRITE8_MEMBER(mcd_isa_device::cmd_w)
{
	if(m_cmdrd_count)
	{
		m_cmdrd_count--;
		switch(m_cmd)
		{
			case CMD_SET_MODE:
				m_mode = data;
				m_cmdbuf[1] = 0;
				m_cmdbuf_count = 2;
				break;
			case CMD_LOCK:
				m_locked = data & 1 ? true : false;
				m_cmdbuf[1] = 0;
				m_cmdbuf[2] = 0;
				m_cmdbuf_count = 3;
				break;
			case CMD_CONFIG:
				switch(m_cmdrd_count)
				{
					case 1:
						if(m_conf == 1)
						{
							m_dmalen = data << 8;
							break;
						}
						m_conf = data;
						if(m_conf == 1)
							m_cmdrd_count++;
						break;
					case 0:
						switch(m_conf)
						{
							case 0x10:
								m_irq = data;
								break;
							case 0x01:
								m_dmalen |= data;
								break;
							case 0x02:
								m_dma = data;
								break;
						}
						m_cmdbuf[1] = 0;
						m_cmdbuf_count = 2;
						m_conf = 0;
						break;
				}
				break;
			case CMD_READ1X:
			case CMD_READ2X:
				switch(m_cmdrd_count)
				{
					case 5:
						m_readmsf = 0;
					case 4:
					case 3:
						m_readmsf |= bcd_2_dec(data) << ((m_cmdrd_count - 3) * 8);
						break;
					case 2:
						m_readcount = data << 16;
						break;
					case 1:
						m_readcount |= data << 8;
						break;
					case 0:
						m_readcount |= data;
						read_sector(true);
						m_cmdbuf_count = 1;
						m_cmdbuf[0] = STAT_SPIN | STAT_READY;
						break;
				}
				break;
		}
		if(!m_cmdrd_count)
			m_stat = m_cdrom_handle ? (STAT_READY | (m_change ? STAT_CHANGE : 0)) : 0;
		return;
	}
	m_cmd = data;
	m_cmdbuf_idx = 0;
	m_cmdrd_count = 0;
	m_cmdbuf_count = 1;
	m_cmdbuf[0] = m_cdrom_handle ? (STAT_READY | (m_change ? STAT_CHANGE : 0)) : 0;
	m_data = false;
	switch(data)
	{
		case CMD_GET_INFO:
			if(m_cdrom_handle)
			{
				uint32_t first = lba_to_msf(150), last = lba_to_msf(cdrom_get_track_start(m_cdrom_handle, 0xaa));
				m_cmdbuf[1] = 1;
				m_cmdbuf[2] = dec_2_bcd(cdrom_get_last_track(m_cdrom_handle));
				m_cmdbuf[3] = dec_2_bcd((last >> 16) & 0xff);
				m_cmdbuf[4] = dec_2_bcd((last >> 8) & 0xff);
				m_cmdbuf[5] = dec_2_bcd(last & 0xff);
				m_cmdbuf[6] = dec_2_bcd((first >> 16) & 0xff);
				m_cmdbuf[7] = dec_2_bcd((first >> 8) & 0xff);
				m_cmdbuf[8] = dec_2_bcd(first & 0xff);
				m_cmdbuf[9] = 0;
				m_cmdbuf_count = 10;
				m_readcount = 0;
			}
			else
			{
				m_cmdbuf_count = 1;
				m_cmdbuf[0] = STAT_CMD_CHECK;
			}
			break;
		case CMD_GET_Q:
			if(m_cdrom_handle)
			{
				int tracks = cdrom_get_last_track(m_cdrom_handle);
				uint32_t start = lba_to_msf(cdrom_get_track_start(m_cdrom_handle, m_curtoctrk));
				uint32_t end = lba_to_msf(cdrom_get_track_start(m_cdrom_handle, m_curtoctrk < tracks ? m_curtoctrk + 1 : 0xaa));
				m_cmdbuf[1] = (cdrom_get_adr_control(m_cdrom_handle, m_curtoctrk) << 4) & 0xf0;
				m_cmdbuf[2] = 0; // track num except when reading toc
				m_cmdbuf[3] = dec_2_bcd(m_curtoctrk); // index
				m_cmdbuf[4] = dec_2_bcd((start >> 16) & 0xff);
				m_cmdbuf[5] = dec_2_bcd((start >> 8) & 0xff);
				m_cmdbuf[6] = dec_2_bcd(start & 0xff);
				m_cmdbuf[7] = 0;
				m_cmdbuf[8] = dec_2_bcd((end >> 16) & 0xff);
				m_cmdbuf[9] = dec_2_bcd((end >> 8) & 0xff);
				m_cmdbuf[10] = dec_2_bcd(end & 0xff);
				if(m_curtoctrk >= tracks)
					m_curtoctrk = 1;
				m_cmdbuf_count = 11;
				m_readcount = 0;
			}
			else
			{
				m_cmdbuf_count = 1;
				m_cmdbuf[0] = STAT_CMD_CHECK;
			}
			break;
		case CMD_GET_STAT:
			m_change = false;
			break;
		case CMD_SET_MODE:
			m_cmdrd_count = 1;
			break;
		case CMD_STOPCDDA:
		case CMD_STOP:
			m_drvmode = DRV_MODE_STOP;
			m_curtoctrk = 1;
			break;
		case CMD_CONFIG:
			m_cmdrd_count = 2;
			break;
		case CMD_READ1X:
		case CMD_READ2X:
			if(m_cdrom_handle)
			{
				m_readcount = 0;
				m_drvmode = DRV_MODE_READ;
				m_cmdrd_count = 6;
			}
			else
			{
				m_cmdbuf_count = 1;
				m_cmdbuf[0] = STAT_CMD_CHECK;
			}
			break;
		case CMD_GET_VER:
			m_cmdbuf[1] = 1; // ?
			m_cmdbuf[2] = 'D';
			m_cmdbuf[3] = 0;
			m_cmdbuf_count = 4;
			break;
		case CMD_EJECT:
			m_readcount = 0;
			break;
		case CMD_LOCK:
			m_cmdrd_count = 1;
			break;
		default:
			m_cmdbuf[0] = m_stat | STAT_CMD_CHECK;
			break;
	}
}
