// license:BSD-3-Clause
// copyright-holders:Tim Schuerewegen
/**************************************************************************

    sord_cas.c

    Format code for Sord M5 cassette files

**************************************************************************/

#include <string.h>
#include <assert.h>
#include "sord_cas.h"

#define SORDM5_WAVESAMPLES_HEADER  1
#define SORDM5_WAVESAMPLES_TRAILER 1
#define SORDM5_WAVESAMPLES_BLOCK   1

static const uint8_t SORDM5_CAS_HEADER[6] = { 'S', 'O', 'R', 'D', 'M', '5'};

static const struct CassetteModulation sordm5_cas_modulation =
{
	CASSETTE_MODULATION_SINEWAVE,
	1575.0 - 300, 1575.0, 1575.0 + 300,
	3150.0 - 600, 3150.0, 3150.0 + 600
};

static uint8_t cassette_image_read_uint8( cassette_image *cassette, uint64_t offset)
{
	uint8_t data;
	cassette_image_read( cassette, &data, offset, 1);
	return data;
}

static cassette_image::error sordm5_tap_identify( cassette_image *cassette, struct CassetteOptions *opts)
{
	return cassette_modulation_identify( cassette, &sordm5_cas_modulation, opts);
}

static cassette_image::error sordm5_tap_load( cassette_image *cassette)
{
	cassette_image::error err;
	uint64_t image_size;
	double time_index = 0.0;
	double time_displacement;
	uint8_t header[16];
	uint64_t image_pos;
	uint8_t block_type, byte, bit;
	uint32_t block_size, i, j;
	size_t filler_length;
	// init
	image_size = cassette_image_size(cassette);
	image_pos = 0;
	// read/check header
	if (image_size < 16) return cassette_image::error::INVALID_IMAGE;
	cassette_image_read( cassette, &header, image_pos, 16);
	image_pos += 16;
	if (memcmp( header, SORDM5_CAS_HEADER, sizeof(SORDM5_CAS_HEADER)) != 0) return cassette_image::error::INVALID_IMAGE;
	// add silence (header)
	err = cassette_put_sample( cassette, 0, time_index, SORDM5_WAVESAMPLES_HEADER, 0);
	if (err != cassette_image::error::SUCCESS) return err;
	time_index += SORDM5_WAVESAMPLES_HEADER;
	// process blocks
	while (image_pos < image_size)
	{
		// read block type
		block_type = cassette_image_read_uint8( cassette, image_pos + 0);
		if ((block_type != 'H') && (block_type != 'D')) return cassette_image::error::INVALID_IMAGE;
		// read block size
		block_size = cassette_image_read_uint8( cassette, image_pos + 1);
		if (block_size == 0) block_size = 0x100;
		block_size += 3;
		// add silence (header block)
		if (block_type == 'H')
		{
			err = cassette_put_sample( cassette, 0, time_index, SORDM5_WAVESAMPLES_BLOCK, 0);
			if (err != cassette_image::error::SUCCESS) return err;
			time_index += SORDM5_WAVESAMPLES_BLOCK;
		}
		// add sync
		if (block_type == 'H') filler_length = 2.4 * (3150 / 8); else filler_length = 0.15 * (3150 / 8);
		err = cassette_put_modulated_filler(cassette, 0, time_index, 0xFF, filler_length, &sordm5_cas_modulation, &time_displacement);
		if (err != cassette_image::error::SUCCESS) return err;
		time_index += time_displacement;
		// process block
		for (i=0;i<block_size;i++)
		{
			// read byte
			byte = cassette_image_read_uint8( cassette, image_pos + i);
			// calc/check checksum
			#if 0
			if (i == block_size)
			{
				if (byte != (crc & 0xFF)) return cassette_image::error::INVALID_IMAGE;
			}
			if (i > 2) crc += byte;
			#endif
			// process byte
			for (j=0;j<10;j++)
			{
				// get bit
				if (j < 2)
				{
					bit = ((j==0)?1:0);
				}
				else
				{
					bit = (byte >> (j-2)) & 1;
				}
				// add bit
				err = cassette_put_modulated_data_bit( cassette, 0, time_index, bit, &sordm5_cas_modulation, &time_displacement);
				if (err != cassette_image::error::SUCCESS) return err;
				time_index += time_displacement;
			}
		}
		// mark end of block
		err = cassette_put_modulated_data_bit( cassette, 0, time_index, 1, &sordm5_cas_modulation, &time_displacement);
		if (err != cassette_image::error::SUCCESS) return err;
		time_index += time_displacement;
		// next block
		image_pos += block_size;
	}
	// add silence (trailer)
	err = cassette_put_sample( cassette, 0, time_index, SORDM5_WAVESAMPLES_TRAILER, 0);
	if (err != cassette_image::error::SUCCESS) return err;
	time_index += SORDM5_WAVESAMPLES_TRAILER;
	//
	return cassette_image::error::SUCCESS;
}

static const struct CassetteFormat sordm5_cas_format =
{
	"cas",
	sordm5_tap_identify,
	sordm5_tap_load,
	nullptr
};

CASSETTE_FORMATLIST_START(sordm5_cassette_formats)
	CASSETTE_FORMAT(sordm5_cas_format)
CASSETTE_FORMATLIST_END
