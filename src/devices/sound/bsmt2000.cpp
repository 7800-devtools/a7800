// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/***************************************************************************

    bsmt2000.cpp

    BSMT2000 device emulator.

****************************************************************************

    Chip is actually a TMS320C15 DSP with embedded mask rom
    Trivia: BSMT stands for "Brian Schmidt's Mouse Trap"

***************************************************************************/

#include "emu.h"
#include "bsmt2000.h"


// device type definition
DEFINE_DEVICE_TYPE(BSMT2000, bsmt2000_device, "bsmt2000", "BSMT2000")


//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

// program map for the DSP (points to internal ROM)
static ADDRESS_MAP_START(tms_program_map, AS_PROGRAM, 16, bsmt2000_device)
	ADDRESS_MAP_UNMAP_HIGH
	AM_RANGE(0x000, 0xfff) AM_ROM
ADDRESS_MAP_END


// I/O map for the DSP
static ADDRESS_MAP_START(tms_io_map, AS_IO, 16, bsmt2000_device)
	AM_RANGE(0, 0) AM_READWRITE(tms_register_r, tms_rom_addr_w)
	AM_RANGE(1, 1) AM_READWRITE(tms_data_r, tms_rom_bank_w)
	AM_RANGE(2, 2) AM_READ(tms_rom_r)
	AM_RANGE(3, 3) AM_WRITE(tms_left_w)
	AM_RANGE(7, 7) AM_WRITE(tms_right_w)
ADDRESS_MAP_END


// ROM definition for the BSMT2000 program ROM
ROM_START( bsmt2000 )
	ROM_REGION( 0x2000, "bsmt2000", 0 )
	ROM_LOAD16_WORD( "bsmt2000.bin", 0x0000, 0x2000, CRC(c2a265af) SHA1(6ec9eb038fb8eb842c5482aebe1d149daf49f2e6) )
ROM_END




//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  bsmt2000_device - constructor
//-------------------------------------------------

bsmt2000_device::bsmt2000_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, BSMT2000, tag, owner, clock)
	, device_sound_interface(mconfig, *this)
	, device_rom_interface(mconfig, *this, 32)
	, m_ready_callback()
	, m_stream(nullptr)
	, m_cpu(*this, "bsmt2000")
	, m_register_select(0)
	, m_write_data(0)
	, m_rom_address(0)
	, m_rom_bank(0)
	, m_left_data(0)
	, m_right_data(0)
	, m_write_pending(false)
{
}


//-------------------------------------------------
//  static_set_ready_callback - configuration
//  helper to set the ready callback
//-------------------------------------------------

void bsmt2000_device::static_set_ready_callback(device_t &device, ready_callback &&callback)
{
	bsmt2000_device &bsmt = downcast<bsmt2000_device &>(device);
	bsmt.m_ready_callback = std::move(callback);
}


//-------------------------------------------------
//  rom_region - return a pointer to the device's
//  internal ROM region
//-------------------------------------------------

const tiny_rom_entry *bsmt2000_device::device_rom_region() const
{
	return ROM_NAME( bsmt2000 );
}


//-------------------------------------------------
//  device_add_mconfig - add device configuration
//-------------------------------------------------

MACHINE_CONFIG_MEMBER( bsmt2000_device::device_add_mconfig )
	MCFG_CPU_ADD("bsmt2000", TMS32015, DERIVED_CLOCK(1,1))
	MCFG_CPU_PROGRAM_MAP(tms_program_map)
	// data map is internal to the CPU
	MCFG_CPU_IO_MAP(tms_io_map)
	MCFG_TMS32010_BIO_IN_CB(READLINE(bsmt2000_device, tms_write_pending_r))
MACHINE_CONFIG_END


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void bsmt2000_device::device_start()
{
	m_ready_callback.bind_relative_to(*owner());

	// create the stream; BSMT typically runs at 24MHz and writes to a DAC, so
	// in theory we should generate a 24MHz stream, but that's certainly overkill
	// internally at 24MHz the max output sample rate is 32kHz
	// divided by 128 gives us 6x the max output rate which is plenty for oversampling
	m_stream = stream_alloc(0, 2, clock() / 128);

	// register for save states
	save_item(NAME(m_register_select));
	save_item(NAME(m_write_data));
	save_item(NAME(m_rom_address));
	save_item(NAME(m_rom_bank));
	save_item(NAME(m_left_data));
	save_item(NAME(m_right_data));
	save_item(NAME(m_write_pending));
}


//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void bsmt2000_device::device_reset()
{
	synchronize(TIMER_ID_RESET);
}



//-------------------------------------------------
//  device_timer - handle deferred writes and
//  resets as a timer callback
//-------------------------------------------------

void bsmt2000_device::device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr)
{
	switch (id)
	{
		// deferred reset
		case TIMER_ID_RESET:
			m_stream->update();
			m_cpu->reset();
			break;

		// deferred register write
		case TIMER_ID_REG_WRITE:
			m_register_select = param & 0xffff;
			break;

		// deferred data write
		case TIMER_ID_DATA_WRITE:
			m_write_data = param & 0xffff;
			if (m_write_pending) logerror("BSMT2000: Missed data\n");
			m_write_pending = true;
			break;
	}
}


//-------------------------------------------------
//  sound_stream_update - handle update requests
//  for our sound stream
//-------------------------------------------------

void bsmt2000_device::sound_stream_update(sound_stream &stream, stream_sample_t **inputs, stream_sample_t **outputs, int samples)
{
	// just fill with current left/right values
	for (int samp = 0; samp < samples; samp++)
	{
		outputs[0][samp] = m_left_data * 16;
		outputs[1][samp] = m_right_data * 16;
	}
}


//-------------------------------------------------
//  rom_bank_updated - the rom bank has changed
//-------------------------------------------------

void bsmt2000_device::rom_bank_updated()
{
	m_stream->update();
}


//-------------------------------------------------
//  read_status - return the write pending status
//-------------------------------------------------

uint16_t bsmt2000_device::read_status()
{
	return m_write_pending ? 0 : 1;
}


//-------------------------------------------------
//  write_reg - handle writes to the BSMT2000
//  register select interface
//-------------------------------------------------

void bsmt2000_device::write_reg(uint16_t data)
{
	synchronize(TIMER_ID_REG_WRITE, data);
}


//-------------------------------------------------
//  write_data - handle writes to the BSMT2000
//  data port
//-------------------------------------------------

void bsmt2000_device::write_data(uint16_t data)
{
	synchronize(TIMER_ID_DATA_WRITE, data);

	// boost the interleave on a write so that the caller detects the status more accurately
	machine().scheduler().boost_interleave(attotime::from_usec(1), attotime::from_usec(10));
}


//-------------------------------------------------
//  tms_register_r - return the value written to
//  the register select port
//-------------------------------------------------

READ16_MEMBER( bsmt2000_device::tms_register_r )
{
	return m_register_select;
}


//-------------------------------------------------
//  tms_data_r - return the value written to the
//  data port
//-------------------------------------------------

READ16_MEMBER( bsmt2000_device::tms_data_r )
{
	// also implicitly clear the write pending flag
	m_write_pending = false;
	if (!m_ready_callback.isnull())
		m_ready_callback();
	return m_write_data;
}


//-------------------------------------------------
//  tms_rom_r - read a byte from the currently
//  selected ROM bank and address
//-------------------------------------------------

READ16_MEMBER( bsmt2000_device::tms_rom_r )
{
	// underlying logic assumes this is a sign-extended value
	return (int8_t)read_byte((m_rom_bank << 16) + m_rom_address);
}


//-------------------------------------------------
//  tms_rom_addr_w - selects which byte within the
//  current ROM bank to access
//-------------------------------------------------

WRITE16_MEMBER( bsmt2000_device::tms_rom_addr_w )
{
	m_rom_address = data;
}


//-------------------------------------------------
//  tms_rom_bank_w - selects which bank of ROM to
//  access
//-------------------------------------------------

WRITE16_MEMBER( bsmt2000_device::tms_rom_bank_w )
{
	m_rom_bank = data;
}


//-------------------------------------------------
//  tms_left_w - handle writes to the left channel
//  DAC
//-------------------------------------------------

WRITE16_MEMBER( bsmt2000_device::tms_left_w )
{
	m_stream->update();
	m_left_data = data;
}


//-------------------------------------------------
//  tms_right_w - handle writes to the right
//  channel DAC
//-------------------------------------------------

WRITE16_MEMBER( bsmt2000_device::tms_right_w )
{
	m_stream->update();
	m_right_data = data;
}


//-------------------------------------------------
//  tms_write_pending_r - return whether a write
//  is pending; this data is fed into the BIO line
//  on the TMS32015
//-------------------------------------------------

READ_LINE_MEMBER( bsmt2000_device::tms_write_pending_r )
{
	return m_write_pending ? 1 : 0;
}
