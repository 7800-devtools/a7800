// license:BSD-3-Clause
// copyright-holders:Olivier Galibert
/*

    Tape support for Orao  TAP format

*/
#include <assert.h>

#include "orao_cas.h"


#define ORAO_WAV_FREQUENCY  44100
#define WAVE_HIGH       -32768
#define WAVE_LOW        0

#define ORAO_WAVE_ONE   24
#define ORAO_WAVE_ZERO  11

#define ORAO_HEADER_SIZE 360

static int16_t    wave_data;
static int      len;

static void orao_output_wave( int16_t **buffer, int length ) {
	if ( buffer == nullptr ) {
		return;
	}

	for( ; length > 0; length-- ) {
		**buffer = wave_data;
		*buffer = *buffer + 1;
	}
}
static int orao_cas_to_wav_size( const uint8_t *casdata, int caslen ) {
	int i,j,size;
	uint8_t b;

	if (casdata == nullptr) return -1;
	if (caslen <= ORAO_HEADER_SIZE) {
		return -1;
	}
	size = 0;
	for (i=ORAO_HEADER_SIZE;i<caslen-ORAO_HEADER_SIZE;i++) {
		for (j=0;j<8;j++) {
			b = (casdata[i] >> j) & 1;
			if (b==0) {
				size += 2*ORAO_WAVE_ZERO;
			} else {
				size += 2*ORAO_WAVE_ONE;
			}
		}
	}
	len = caslen;
	return size;
}

static int orao_cas_fill_wave( int16_t *buffer, int length, uint8_t *bytes ) {
	int i,j,size;
	uint8_t b;
	size = 0;
	if (bytes == nullptr) return -1;
	for (i=ORAO_HEADER_SIZE;i<len-ORAO_HEADER_SIZE;i++) {
		for (j=0;j<8;j++) {
			b = (bytes[i] >> j) & 1;
			if (b==0) {
				wave_data = WAVE_LOW;
				orao_output_wave(&buffer,ORAO_WAVE_ZERO);
				wave_data = WAVE_HIGH;
				orao_output_wave(&buffer,ORAO_WAVE_ZERO);
				size += 2 *ORAO_WAVE_ZERO;
			} else {
				wave_data = WAVE_LOW;
				orao_output_wave(&buffer,ORAO_WAVE_ONE);
				wave_data = WAVE_HIGH;
				orao_output_wave(&buffer,ORAO_WAVE_ONE);
				size += 2 * ORAO_WAVE_ONE;
			}
		}
	}

	return size;
}




static const struct CassetteLegacyWaveFiller orao_legacy_fill_wave = {
	orao_cas_fill_wave,         /* fill_wave */
	-1,                 /* chunk_size */
	0,                  /* chunk_samples */
	orao_cas_to_wav_size,           /* chunk_sample_calc */
	ORAO_WAV_FREQUENCY,         /* sample_frequency */
	0,                  /* header_samples */
	0                   /* trailer_samples */
};



static cassette_image::error orao_cassette_identify( cassette_image *cassette, struct CassetteOptions *opts ) {
	return cassette_legacy_identify( cassette, opts, &orao_legacy_fill_wave );
}



static cassette_image::error orao_cassette_load( cassette_image *cassette ) {
	return cassette_legacy_construct( cassette, &orao_legacy_fill_wave );
}



static const struct CassetteFormat orao_cassette_format = {
	"tap",
	orao_cassette_identify,
	orao_cassette_load,
	nullptr
};


CASSETTE_FORMATLIST_START(orao_cassette_formats)
	CASSETTE_FORMAT(orao_cassette_format)
CASSETTE_FORMATLIST_END
