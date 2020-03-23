// license:BSD-3-Clause
// copyright-holders:Nathan Woods, Miodrag Milanovic
/*********************************************************************

    bitbngr.c

*********************************************************************/

#include "emu.h"
#include "bitbngr.h"



/***************************************************************************
    IMPLEMENTATION
***************************************************************************/

DEFINE_DEVICE_TYPE(BITBANGER, bitbanger_device, "bitbanger", "Bitbanger")

/*-------------------------------------------------
    ctor
-------------------------------------------------*/

bitbanger_device::bitbanger_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, BITBANGER, tag, owner, clock),
	device_image_interface(mconfig, *this),
	m_interface(nullptr),
	m_is_readonly(false)
{
}



/*-------------------------------------------------
    native_output - outputs data to a file
-------------------------------------------------*/

void bitbanger_device::output(uint8_t data)
{
	if (exists())
	{
		fwrite(&data, 1);
	}
}


/*-------------------------------------------------
    native_input - inputs data from a file
-------------------------------------------------*/

uint32_t bitbanger_device::input(void *buffer, uint32_t length)
{
	if (exists())
	{
		return fread(buffer, length);
	}
	return 0;
}



/*-------------------------------------------------
    device_start
-------------------------------------------------*/

void bitbanger_device::device_start(void)
{
}



/*-------------------------------------------------
    call_load
-------------------------------------------------*/

image_init_result bitbanger_device::call_load(void)
{
	/* we don't need to do anything special */
	return image_init_result::PASS;
}

image_init_result bitbanger_device::call_create(int format_type, util::option_resolution *format_options)
{
	/* we don't need to do anything special */
	return image_init_result::PASS;
}

/*-------------------------------------------------
    call_unload
-------------------------------------------------*/

void bitbanger_device::call_unload(void)
{
}
