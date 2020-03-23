// license:BSD-3-Clause
// copyright-holders:Curt Coder
/*********************************************************************

    formats/atom_tap.c

    Cassette code for Acorn Atom tap files (Kansas City Standard)

*********************************************************************/

/*

    http://beebwiki.jonripley.com/Acorn_cassette_format#Atom

    The Atom supports the System series format (as a 'nameless file') and its own format. The data consists of files separated
    by carrier breaks and the appropriate leaders and trailers, and further subdivided into blocks separated by inter-block gaps.
    An Atom block consists of the following sequence of bytes:

        Four synchronisation bytes (&2A).
        File name (one to thirteen characters).
        One end of file name marker byte (&0D).
        Block flag, one byte.
        Block number, two bytes, high byte first.
        Data block length ? 1, one byte.
        Execution address, two bytes, high byte first.
        Load address, two bytes, high byte first.
        Data, 0 to 256 bytes.
        Checksum, one byte.

    The block flag is set as follows:

        Bit 7 is set if this block is not the last block of a file.
        Bit 6 is set if the block contains data. If clear then the 'data block length' field is invalid.
        Bit 5 is set if this block is not the first block of a file.
        Bits 4 to 0 are undefined. Normally their contents are:
        in the first block, bits 15 to 11 of the end address (=1 + the last address saved in the file);
        in subsequent blocks, bits 6 to 2 of the previous block flag.

    The checksum is a simple sum (modulo 256) of all bytes from the start of the filename to the end of the data.

*/

#include <assert.h>

#include "atom_tap.h"
#include "csw_cas.h"
#include "uef_cas.h"
/***************************************************************************
    PARAMETERS
***************************************************************************/

#define LOG 1

/***************************************************************************
    IMPLEMENTATION
***************************************************************************/

/*-------------------------------------------------
    CassetteModulation atom_tap_modulation
-------------------------------------------------*/

static const struct CassetteModulation atom_tap_modulation =
{
	CASSETTE_MODULATION_SINEWAVE,
	1200.0 - 300, 1200.0, 1200.0 + 300,
	2400.0 - 600, 2400.0, 2400.0 + 600
};

/*-------------------------------------------------
    cassette_image_read_uint8 - read tape data
-------------------------------------------------*/

static uint8_t cassette_image_read_uint8(cassette_image *cassette, uint64_t offset)
{
	uint8_t data;
	cassette_image_read(cassette, &data, offset, 1);
	return data;
}

/*-------------------------------------------------
    atom_tap_identify - identify cassette
-------------------------------------------------*/

static cassette_image::error atom_tap_identify(cassette_image *cassette, struct CassetteOptions *opts)
{
	return cassette_modulation_identify( cassette, &atom_tap_modulation, opts);
}

/*-------------------------------------------------
    atom_tap_load - load cassette
-------------------------------------------------*/

#define MODULATE(_value) \
	for (int i = 0; i < (_value ? 8 : 4); i++) { \
		err = cassette_put_modulated_data_bit(cassette, 0, time_index, _value, &atom_tap_modulation, &time_displacement);\
		if (err != cassette_image::error::SUCCESS) return err;\
		time_index += time_displacement;\
	}

#define BIT(x,n) (((x)>>(n))&1)

static cassette_image::error atom_tap_load(cassette_image *cassette)
{
	cassette_image::error err;
	uint64_t image_size = cassette_image_size(cassette);
	uint64_t image_pos = 0;
	double time_index = 0.0;
	double time_displacement;

	while (image_pos < image_size)
	{
		uint8_t data = cassette_image_read_uint8(cassette, image_pos);

		/* start bit */
		MODULATE(0);

		/* data bits */
		for (int bit = 0; bit < 8; bit++)
		{
			MODULATE(BIT(data, bit));
		}

		/* stop bits */
		MODULATE(1);
		MODULATE(1);

		image_pos++;
	}

	return cassette_image::error::SUCCESS;
}

/*-------------------------------------------------
    CassetteFormat atom_tap_cassette_format
-------------------------------------------------*/

const struct CassetteFormat atom_tap_format =
{
	"tap",
	atom_tap_identify,
	atom_tap_load,
	nullptr
};

CASSETTE_FORMATLIST_START( atom_cassette_formats )
	CASSETTE_FORMAT(atom_tap_format)
	CASSETTE_FORMAT(csw_cassette_format)
	CASSETTE_FORMAT(uef_cassette_format)
CASSETTE_FORMATLIST_END
