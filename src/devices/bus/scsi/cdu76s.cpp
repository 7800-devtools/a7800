// license:BSD-3-Clause
// copyright-holders:smf
#include "emu.h"
#include "cdu76s.h"

void sony_cdu76s_device::ExecCommand()
{
	switch ( command[0] )
	{
		case 0x12: // INQUIRY
			logerror("CDU76S: INQUIRY\n");
			m_phase = SCSI_PHASE_DATAIN;
			m_status_code = SCSI_STATUS_CODE_GOOD;
			m_transfer_length = SCSILengthFromUINT8( &command[ 4 ] );
			break;
	}
}

void sony_cdu76s_device::ReadData( uint8_t *data, int dataLength )
{
	switch ( command[0] )
	{
		case 0x12: // INQUIRY
			memset( data, 0, dataLength );
			data[0] = 0x05; // device is present, device is CD/DVD (MMC-3)
			data[1] = 0x80; // media is removable
			data[2] = 0x05; // device complies with SPC-3 standard
			data[3] = 0x02; // response data format = SPC-3 standard
			// some Konami games freak out if this isn't "Sony", so we'll lie
			// this is the actual drive on my Nagano '98 board
			strcpy((char *)&data[8], "Sony");
			strcpy((char *)&data[16], "CDU-76S");
			strcpy((char *)&data[32], "1.0");
			break;

		default:
			scsicd_device::ReadData( data, dataLength );
			break;
	}
}

// device type definition
DEFINE_DEVICE_TYPE(CDU76S, sony_cdu76s_device, "cdu76s", "Sony CDU-76S")

sony_cdu76s_device::sony_cdu76s_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	scsicd_device(mconfig, CDU76S, tag, owner, clock)
{
}
