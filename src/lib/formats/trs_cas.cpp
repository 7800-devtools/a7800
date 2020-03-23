// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/********************************************************************

Support for TRS80 .cas cassette images

********************************************************************/

#include <assert.h>

#include "formats/trs_cas.h"

#define SILENCE 0
#define SMPLO   -32768
#define SMPHI   32767


static int cas_size;


/*******************************************************************
   Generate one high-low cycle of sample data
********************************************************************/
static inline int trs80l2_cas_cycle(int16_t *buffer, int sample_pos, int silence, int high, int low)
{
	int i = 0;

	if ( buffer )
	{
		while( i < silence )
		{
			buffer[ sample_pos + i ] = SILENCE;
			i++;
		}
		while( i < silence + high)
		{
			buffer[ sample_pos + i ] = SMPHI;
			i++;
		}

		while( i < silence + high + low )
		{
			buffer[ sample_pos + i ] = SMPLO;
			i++;
		}
	}
	return silence + high + low;
}


static int trs80l2_handle_cas(int16_t *buffer, const uint8_t *casdata)
{
	int data_pos, sample_count;

	data_pos = 0;
	sample_count = 0;

	while( data_pos < cas_size )
	{
		uint8_t   data = casdata[data_pos];
		int     i;

		for ( i = 0; i < 8; i++ )
		{
			/* Signal code */
			sample_count += trs80l2_cas_cycle( buffer, sample_count, 33, 6, 6 );

			/* Bit code */
			if ( data & 0x80 )
				sample_count += trs80l2_cas_cycle( buffer, sample_count, 33, 6, 6 );
			else
				sample_count += trs80l2_cas_cycle( buffer, sample_count, 33 + 6 + 6, 0, 0 );

			data <<= 1;
		}

		data_pos++;
	}
	return sample_count;
}


/*******************************************************************
   Generate samples for the tape image
********************************************************************/
static int trs80l2_cas_fill_wave(int16_t *buffer, int sample_count, uint8_t *bytes)
{
	return trs80l2_handle_cas( buffer, bytes );
}


/*******************************************************************
   Calculate the number of samples needed for this tape image
********************************************************************/
static int trs80l2_cas_to_wav_size(const uint8_t *casdata, int caslen)
{
	cas_size = caslen;

	return trs80l2_handle_cas( nullptr, casdata );
}

static const struct CassetteLegacyWaveFiller trs80l2_cas_legacy_fill_wave =
{
	trs80l2_cas_fill_wave,                  /* fill_wave */
	-1,                                     /* chunk_size */
	0,                                      /* chunk_samples */
	trs80l2_cas_to_wav_size,                /* chunk_sample_calc */
	44100,                                  /* sample_frequency */
	0,                                      /* header_samples */
	0                                       /* trailer_samples */
};


static cassette_image::error trs80l2_cas_identify(cassette_image *cassette, struct CassetteOptions *opts)
{
	return cassette_legacy_identify(cassette, opts, &trs80l2_cas_legacy_fill_wave);
}


static cassette_image::error trs80l2_cas_load(cassette_image *cassette)
{
	return cassette_legacy_construct(cassette, &trs80l2_cas_legacy_fill_wave);
}


static const struct CassetteFormat trs80l2_cas_format =
{
	"cas",
	trs80l2_cas_identify,
	trs80l2_cas_load,
	nullptr
};


CASSETTE_FORMATLIST_START(trs80l2_cassette_formats)
	CASSETTE_FORMAT(trs80l2_cas_format)
CASSETTE_FORMATLIST_END
