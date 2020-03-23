// license:BSD-3-Clause
// copyright-holders:Curt Coder
/*********************************************************************

    formats/fdi_dsk.h

    Floppy format code for Formatted Disk Image (FDI) v2.0 images

*********************************************************************/

/*

    TODO:

    - everything

*/

#include <assert.h>
#include "imageutl.h"
#include "flopimg.h"

/***************************************************************************
    PARAMETERS
***************************************************************************/

#define LOG 1

#define MAX_TRACKS  180

#define FLAGS_WRITE_PROTECTED   0x01
#define FLAGS_INDEX_SYNC        0x02

/* media type */
enum
{
	TYPE_8_INCH = 0,
	TYPE_5_25_INCH,
	TYPE_3_5_INCH,
	TYPE_3_INCH
};

static const char *const MEDIA_TYPE[] = { "8", "5.25", "3.5", "3" };

/* tracks per inch */
enum
{
	TPI_48 = 0,
	TPI_67,
	TPI_96,
	TPI_100,
	TPI_135,
	TPI_192
};

static const int TRACKS_PER_INCH[] = { 48, 67, 96, 100, 135, 192 };

/* high level track types */
enum
{
	TRACK_BLANK = 0,    /* not supported */
	TRACK_AMIGA_DD,     /* not supported */
	TRACK_AMIGA_HD,     /* not supported */
	TRACK_ST_DD_9,      /* not supported */
	TRACK_ST_DD_10,     /* not supported */
	TRACK_PC_DD_8,      /* not supported */
	TRACK_PC_DD_9,      /* not supported */
	TRACK_PC_HD_15,     /* not supported */
	TRACK_IBM_HD_18,    /* not supported */
	TRACK_IBM_ED_36,    /* not supported */
	TRACK_CBM_1541,     /* not supported */
	TRACK_APPLE_DOS_32, /* not supported */
	TRACK_APPLE_DOS_33, /* not supported */
	TRACK_APPLE_GCR,    /* not supported */
	TRACK_IBM_SD_10     /* not supported */
};

//static const int SECTORS_PER_TRACK[] = { 0, 11, 22, 9, 10, 8, 9, 15, 18, 36, -1, -1, -1, -1, 10 };

/***************************************************************************
    TYPE DEFINITIONS
***************************************************************************/

struct fdidsk_tag
{
	int version;
	int heads;                              /* number of physical heads */
	int tracks;                             /* number of physical tracks */
	uint32_t track_offset[MAX_TRACKS];        /* offset within data for each track */
	uint8_t track_type[MAX_TRACKS];
};

struct fdi_track
{
	uint8_t type;
	uint8_t size;
};

struct fdi_header
{
	uint8_t signature[27];
	uint8_t creator[32];
	uint8_t comment[80];
	uint8_t eof;
	uint8_t version[2];
	uint8_t ltrack[2];
	uint8_t lhead;
	uint8_t type;
	uint8_t rotspeed;
	uint8_t flags;
	uint8_t tpi;
	uint8_t headwidth;
	uint8_t reserved[2];
	struct fdi_track track[MAX_TRACKS];
};

struct fdi_pulse_track_header
{
	uint8_t numpulses[4];
	uint8_t averagesz[3];
	uint8_t minsize[3];
	uint8_t maxsize[3];
	uint8_t indexsize[3];
//  uint8_t averagedt[averagesz];
//  uint8_t mindata[minsize];
//  uint8_t maxdata[maxsize];
//  uint8_t indexdata[indexsize];
};

/***************************************************************************
    INLINE FUNCTIONS
***************************************************************************/

static inline struct fdidsk_tag *get_tag(floppy_image_legacy *floppy)
{
	return (fdidsk_tag *)floppy_tag(floppy);
}

/***************************************************************************
    IMPLEMENTATION
***************************************************************************/

static int fdi_get_heads_per_disk(floppy_image_legacy *floppy)
{
	return get_tag(floppy)->heads;
}

static int fdi_get_tracks_per_disk(floppy_image_legacy *floppy)
{
	return get_tag(floppy)->tracks;
}

static int fdi_get_sectors_per_track(floppy_image_legacy *floppy, int head, int track)
{
	return 0;
}

static floperr_t fdi_get_sector_length(floppy_image_legacy *floppy, int head, int track, int sector, uint32_t *sector_length)
{
	return FLOPPY_ERROR_SUCCESS;
}
/*
static uint32_t fdi_get_sector_offset(floppy_image_legacy* floppy, int head, int track, int sector)
{
    return 0;
}
*/
static floperr_t fdi_get_indexed_sector_info(floppy_image_legacy *floppy, int head, int track, int sector_index, int *cylinder, int *side, int *sector, uint32_t *sector_length, unsigned long *flags)
{
	return FLOPPY_ERROR_UNSUPPORTED;
}

static floperr_t fdi_read_track(floppy_image_legacy *floppy, int head, int track, uint64_t offset, void *buffer, size_t buflen)
{
	return FLOPPY_ERROR_UNSUPPORTED;
}

static floperr_t fdi_read_sector(floppy_image_legacy *floppy, int head, int track, int sector, void *buffer, size_t buflen)
{
	return FLOPPY_ERROR_UNSUPPORTED;
}

static floperr_t fdi_read_indexed_sector(floppy_image_legacy *floppy, int head, int track, int sector, void *buffer, size_t buffer_len)
{
	return FLOPPY_ERROR_UNSUPPORTED;
}

static floperr_t fdi_write_sector(floppy_image_legacy *floppy, int head, int track, int sector, const void *buffer, size_t buflen, int ddam)
{
	return FLOPPY_ERROR_UNSUPPORTED;
}

static floperr_t fdi_write_indexed_sector(floppy_image_legacy *floppy, int head, int track, int sector, const void *buffer, size_t buflen, int ddam)
{
	return FLOPPY_ERROR_UNSUPPORTED;
}

/*-------------------------------------------------
    FLOPPY_IDENTIFY( fdi_dsk_identify )
-------------------------------------------------*/

FLOPPY_IDENTIFY( fdi_dsk_identify )
{
	uint8_t header[25];

	floppy_image_read(floppy, header, 0, 25);

	if (!strncmp((const char *)header, "Formatted Disk Image file", 25))
	{
		*vote = 100;
	}
	else
	{
		*vote = 0;
	}

	return FLOPPY_ERROR_SUCCESS;
}

/*-------------------------------------------------
    FLOPPY_CONSTRUCT( fdi_dsk_construct )
-------------------------------------------------*/

FLOPPY_CONSTRUCT( fdi_dsk_construct )
{
	if (params)
	{
		/* create not supported */
		return FLOPPY_ERROR_UNSUPPORTED;
	}

	struct fdidsk_tag *tag = (struct fdidsk_tag *) floppy_create_tag(floppy, sizeof(struct fdidsk_tag));

	if (!tag) return FLOPPY_ERROR_OUTOFMEMORY;

	/* read header */
	struct fdi_header header;
	floppy_image_read(floppy, &header, 0, sizeof(header));

	tag->version = header.version[0];
	tag->tracks = pick_integer_be(header.ltrack, 0, 2) + 1;
	tag->heads = header.lhead + 1;

	if (LOG)
	{
		LOG_FORMATS("FDI creator: %s\n", header.creator);
		LOG_FORMATS("FDI comment: %s\n", header.comment);
		LOG_FORMATS("FDI version: %u\n", tag->version);
		LOG_FORMATS("FDI tracks: %u\n", tag->tracks);
		LOG_FORMATS("FDI heads: %u\n", tag->heads);
		LOG_FORMATS("FDI media type: %s\"\n", MEDIA_TYPE[header.type]);
		LOG_FORMATS("FDI rotation speed: %u\n", header.rotspeed + 128);
		if (header.flags & FLAGS_WRITE_PROTECTED) LOG_FORMATS("FDI is write protected\n");
		if (header.flags & FLAGS_INDEX_SYNC) LOG_FORMATS("FDI is index synchronized\n");
		LOG_FORMATS("FDI media TPI: %u\n", TRACKS_PER_INCH[header.tpi]);
		LOG_FORMATS("FDI head TPI: %u\n", TRACKS_PER_INCH[header.headwidth]);
	}

	/* find track offsets */
	int offset = sizeof(header);

	for (int track = 0; track < tag->tracks; track++)
	{
		uint8_t type = header.track[track].type;
		int size = header.track[track].size * 256;

		if (LOG) LOG_FORMATS("FDI track %d type %02x size %d offset %d\n", track, type, size, offset);

		tag->track_offset[track] = offset;
		tag->track_type[track] = type;

		offset += size;
	}

	/* set callbacks */
	struct FloppyCallbacks *callbacks = floppy_callbacks(floppy);

	callbacks->read_track = fdi_read_track;
	callbacks->get_heads_per_disk = fdi_get_heads_per_disk;
	callbacks->get_tracks_per_disk = fdi_get_tracks_per_disk;
	callbacks->get_sector_length = fdi_get_sector_length;
	callbacks->read_sector = fdi_read_sector;
	callbacks->read_indexed_sector = fdi_read_indexed_sector;
//  callbacks->write_track = fdi_write_track;
	callbacks->write_sector = fdi_write_sector;
	callbacks->write_indexed_sector = fdi_write_indexed_sector;
	callbacks->get_indexed_sector_info = fdi_get_indexed_sector_info;
	callbacks->get_sectors_per_track = fdi_get_sectors_per_track;

	return FLOPPY_ERROR_SUCCESS;
}
