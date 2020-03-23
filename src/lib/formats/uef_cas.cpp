// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/*

The UEF file format is designed to store accurate images of the common media types associated
with the BBC Micro, Acorn Electron and Atom. Tape storage is compatible with the CUTS/BYTE/Kansas City
Format, and hence the format is also capable of storing software for non-Acorn systems such as the
Altair 8800, PT SOL-20, Ohio Scientific, Compukit UK101, Nascom 1/2/3, Motorola MEK D1 6800 and
SWTPC 6800 kit based computers.

UEF files are chunk based and optionally compressed.

The UEF format supports gzipped images, i'm doing the gunzip step during uef_cas_to_wav_size
because that is when the length of the original file is known. This is needed to determine
the size of memory to alloc for the decoding.

Not nice, but it works...

*/

#include <string.h>
#include <math.h>
#include <assert.h>

#include <zlib.h>
#include "uef_cas.h"


#define UEF_WAV_FREQUENCY   4800
#define WAVE_LOW    -32768
#define WAVE_HIGH   32767
#define WAVE_NULL   0

static const uint8_t UEF_HEADER[10] = { 0x55, 0x45, 0x46, 0x20, 0x46, 0x69, 0x6c, 0x65, 0x21, 0x00 };
static const uint8_t GZ_HEADER[2] = { 0x1f, 0x8b };

/*
    bytes are stored as
    start bit   1 * 0
    data bits   8 * X
    stop bit    1 * 1
*/

/* gzip flag byte */
#define ASCII_FLAG  0x01 /* bit 0 set: file probably ascii text */
#define HEAD_CRC    0x02 /* bit 1 set: header CRC present */
#define EXTRA_FIELD 0x04 /* bit 2 set: extra field present */
#define ORIG_NAME   0x08 /* bit 3 set: original file name present */
#define COMMENT     0x10 /* bit 4 set: file comment present */
#define RESERVED    0xE0 /* bits 5..7: reserved */

static const uint8_t* skip_gz_header( const uint8_t *p ) {
	uint8_t method, flags;
	/* skip initial 1f 8b header */
	p += 2;
	/* get method and flags */
	method = *p; p++;
	flags = *p; p++;
	if ( method != Z_DEFLATED || ( flags & RESERVED ) != 0 ) {
		return nullptr;
	}
	/* Skip time, xflags and OS code */
	p += 6;

	/* Skip the extra field */
	if ( ( flags & EXTRA_FIELD ) != 0 ) {
		int len = ( p[1] << 8 ) | p[0];
		p += 2 + len;
	}
	/* Skip the original file name */
	if ( ( flags & ORIG_NAME ) != 0 ) {
		for ( ; *p; p++ );
	}
	/* Skip the .gz file comment */
	if ( ( flags & COMMENT ) != 0 ) {
		for( ; *p; p++ );
	}
	/* Skip the header crc */
	if ( ( flags & HEAD_CRC ) != 0 ) {
		p += 2;
	}
	return p;
}

static uint8_t *gz_ptr = nullptr;

static float get_uef_float( const uint8_t *Float)
{
		int Mantissa;
		float Result;
		int Exponent;

		/* assume a four byte array named Float exists, where Float[0]
		was the first byte read from the UEF, Float[1] the second, etc */

		/* decode mantissa */
		Mantissa = Float[0] | (Float[1] << 8) | ((Float[2]&0x7f)|0x80) << 16;

		Result = (float)Mantissa;
		Result = (float)ldexp(Result, -23);

		/* decode exponent */
		Exponent = ((Float[2]&0x80) >> 7) | (Float[3]&0x7f) << 1;
		Exponent -= 127;
		Result = (float)ldexp(Result, Exponent);

		/* flip sign if necessary */
		if(Float[3]&0x80)
			Result = -Result;

	/* floating point number is now in 'Result' */
	return Result;
}

static int uef_cas_to_wav_size( const uint8_t *casdata, int caslen ) {
	int pos, size;

	if ( casdata[0] == 0x1f && casdata[1] == 0x8b ) {
		int err;
		z_stream d_stream;
		int inflate_size = ( casdata[ caslen - 1 ] << 24 ) | ( casdata[ caslen - 2 ] << 16 ) | ( casdata[ caslen - 3 ] << 8 ) | casdata[ caslen - 4 ];
		const uint8_t *in_ptr = skip_gz_header( casdata );

		if ( in_ptr == nullptr ) {
			goto cleanup;
		}
		gz_ptr = (uint8_t *)malloc( inflate_size );

		d_stream.zalloc = nullptr;
		d_stream.zfree = nullptr;
		d_stream.opaque = nullptr;
		d_stream.next_in = (unsigned char *)in_ptr;
		d_stream.avail_in = caslen - ( in_ptr - casdata );
		d_stream.next_out = gz_ptr;
		d_stream.avail_out = inflate_size;

		err = inflateInit2( &d_stream, -MAX_WBITS );
		if ( err != Z_OK ) {
			LOG_FORMATS( "inflateInit2 error: %d\n", err );
			goto cleanup;
		}
		err = inflate( &d_stream, Z_NO_FLUSH );
		if ( err != Z_STREAM_END && err != Z_OK ) {
			LOG_FORMATS( "inflate error: %d\n", err );
			goto cleanup;
		}
		err = inflateEnd( &d_stream );
		if ( err != Z_OK ) {
			LOG_FORMATS( "inflateEnd error: %d\n", err );
			goto cleanup;
		}
		caslen = inflate_size;
		casdata = gz_ptr;
	}

	if ( caslen < 18 ) {
		LOG_FORMATS( "uef_cas_to_wav_size: cassette image too small\n" );
		goto cleanup;
	}
	if ( memcmp( casdata, UEF_HEADER, sizeof(UEF_HEADER) ) ) {
		LOG_FORMATS( "uef_cas_to_wav_size: cassette image has incompatible header\n" );
		goto cleanup;
	}

	LOG_FORMATS( "UEF: Determining tape size\n" );
	size = 0;
	pos = sizeof(UEF_HEADER) + 2;
	while( pos < caslen ) {
		int chunk_type = ( casdata[pos+1] << 8 ) | casdata[pos];
		int chunk_length = ( casdata[pos+5] << 24 ) | ( casdata[pos+4] << 16 ) | ( casdata[pos+3] << 8 ) | casdata[pos+2];
		int baud_length;

		pos += 6;
		switch( chunk_type ) {
		case 0x0100:    /* implicit start/stop bit data block */
			size += ( chunk_length * 10 ) * 4;
			break;
		case 0x0101:    /* multiplexed data block */
		case 0x0103:
			LOG_FORMATS( "Unsupported chunk type: %04x\n", chunk_type );
			break;
		case 0x0102:    /* explicit tape data block */
			size += ( ( chunk_length * 10 ) - casdata[pos] ) * 4;
			break;
		case 0x0104:    /* defined tape format data block */
			LOG_FORMATS( "Unsupported chunk type: %04x\n", chunk_type );
			break;
		case 0x0110:    /* carrier tone (previously referred to as 'high tone') */
			baud_length = ( casdata[pos+1] << 8 ) | casdata[pos];
			size += baud_length * 2;
			break;
		case 0x0111:
			LOG_FORMATS( "Unsupported chunk type: %04x\n", chunk_type );
			break;
		case 0x0112:    /* integer gap */
			baud_length = ( casdata[pos+1] << 8 ) | casdata[pos];
			size += baud_length * 2 ;
			break;
		case 0x0116:    /* floating point gap */
			size += get_uef_float(casdata+pos)*UEF_WAV_FREQUENCY;
			break;
		case 0x0113:    /* change of base frequency */
		case 0x0114:    /* security cycles */
		case 0x0115:    /* phase change */
		case 0x0117:    /* data encoding format change */
		case 0x0120:    /* position marker */
		case 0x0130:    /* tape set info */
		case 0x0131:    /* start of tape side */
			LOG_FORMATS( "Unsupported chunk type: %04x\n", chunk_type );
			break;
		}
		pos += chunk_length;
	}
	size = 2 * size;
	return size;

cleanup:
	if ( gz_ptr ) {
		free( gz_ptr );
		gz_ptr = nullptr;
	}
	return -1;
}

static int16_t* uef_cas_fill_bit( int16_t *buffer, int bit ) {
	if ( bit ) {
		*buffer = WAVE_LOW; buffer++;
		*buffer = WAVE_HIGH; buffer++;
		*buffer = WAVE_LOW; buffer++;
		*buffer = WAVE_HIGH; buffer++;
	} else {
		*buffer = WAVE_LOW; buffer++;
		*buffer = WAVE_LOW; buffer++;
		*buffer = WAVE_HIGH; buffer++;
		*buffer = WAVE_HIGH; buffer++;
	}
	return buffer;
}

static int uef_cas_fill_wave( int16_t *buffer, int length, uint8_t *bytes ) {
	int pos;
	int16_t *p = buffer;

	if ( bytes[0] == 0x1f && bytes[1] == 0x8b ) {
		if ( gz_ptr == nullptr ) {
			return 1;
		}
		bytes = gz_ptr;
	}

	pos = sizeof(UEF_HEADER) + 2;
	length = length / 2;
	while( length > 0 ) {
		int chunk_type = ( bytes[pos+1] << 8 ) | bytes[pos];
		int chunk_length = ( bytes[pos+5] << 24 ) | ( bytes[pos+4] << 16 ) | ( bytes[pos+3] << 8 ) | bytes[pos+2];

		int baud_length, i, j;
		uint8_t *c;
		pos += 6;
		switch( chunk_type ) {
		case 0x0100:    /* implicit start/stop bit data block */
			for( j = 0; j < chunk_length; j++ ) {
				uint8_t byte = bytes[pos+j];
				p = uef_cas_fill_bit( p, 0 );
				for( i = 0; i < 8; i++ ) {
					p = uef_cas_fill_bit( p, byte & 1 );
					byte = byte >> 1;
				}
				p = uef_cas_fill_bit( p, 1 );
				length -= ( 10 * 4 );
			}
			break;
		case 0x0101:    /* multiplexed data block */
		case 0x0103:
			LOG_FORMATS( "Unsupported chunk type: %04x\n", chunk_type );
			break;
		case 0x0102:    /* explicit tape data block */
			j = ( chunk_length * 10 ) - bytes[pos];
			c = bytes + pos;
			while( j ) {
				uint8_t byte = *c;
				for( i = 0; i < 8 && i < j; i++ ) {
					p = uef_cas_fill_bit( p, byte & 1 );
					byte = byte >> 1;
					j--;
				}
				c++;
			}
			break;
		case 0x0110:    /* carrier tone (previously referred to as 'high tone') */
			for( baud_length = ( ( bytes[pos+1] << 8 ) | bytes[pos] ) ; baud_length; baud_length-- ) {
				*p = WAVE_LOW; p++;
				*p = WAVE_HIGH; p++;
				length -= 2;
			}
			break;
		case 0x0112:    /* integer gap */
			for( baud_length = ( ( bytes[pos+1] << 8 ) | bytes[pos] ) ; baud_length; baud_length-- ) {
				*p = WAVE_NULL; p++;
				*p = WAVE_NULL; p++;
				length -= 2;
			}
			break;
		case 0x0116:    /* floating point gap */
			for( baud_length = (get_uef_float(bytes+pos)*UEF_WAV_FREQUENCY); baud_length; baud_length-- ) {
				*p = WAVE_NULL; p++;
				length -= 1;
			}
			break;
		}
		pos += chunk_length;
	}
	return p - buffer;
}

static const struct CassetteLegacyWaveFiller uef_legacy_fill_wave = {
	uef_cas_fill_wave,      /* fill_wave */
	-1,                     /* chunk_size */
	0,                      /* chunk_samples */
	uef_cas_to_wav_size,    /* chunk_sample_calc */
	UEF_WAV_FREQUENCY,      /* sample_frequency */
	0,                      /* header_samples */
	0                       /* trailer_samples */
};

static cassette_image::error uef_cassette_identify( cassette_image *cassette, struct CassetteOptions *opts ) {
	uint8_t header[10];

	cassette_image_read(cassette, header, 0, sizeof(header));
	if (memcmp(&header[0], GZ_HEADER, sizeof(GZ_HEADER)) && memcmp(&header[0], UEF_HEADER, sizeof(UEF_HEADER))) {
		return cassette_image::error::INVALID_IMAGE;
	}
	return cassette_legacy_identify( cassette, opts, &uef_legacy_fill_wave );
}

static cassette_image::error uef_cassette_load( cassette_image *cassette ) {
	return cassette_legacy_construct( cassette, &uef_legacy_fill_wave );
}

const struct CassetteFormat uef_cassette_format = {
	"uef",
	uef_cassette_identify,
	uef_cassette_load,
	nullptr
};

CASSETTE_FORMATLIST_START(uef_cassette_formats)
	CASSETTE_FORMAT(uef_cassette_format)
CASSETTE_FORMATLIST_END
