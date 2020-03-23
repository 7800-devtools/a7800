// license:GPL-2.0+
// copyright-holders:Dirk Best
/***************************************************************************

    APRIDISK

    Disk image format for the ACT Apricot

***************************************************************************/

#include "emu.h"
#include "imageutl.h"
#include "apridisk.h"

apridisk_format::apridisk_format()
{
}

const char *apridisk_format::name() const
{
	return "apridisk";
}

const char *apridisk_format::description() const
{
	return "APRIDISK disk image";
}

const char *apridisk_format::extensions() const
{
	return "dsk";
}

int apridisk_format::identify(io_generic *io, uint32_t form_factor)
{
	uint8_t header[APR_HEADER_SIZE];
	io_generic_read(io, header, 0, APR_HEADER_SIZE);

	const char magic[] = "ACT Apricot disk image\x1a\x04";

	if (memcmp(header, magic, sizeof(magic) - 1) == 0)
		return 100;
	else
		return 0;
}

bool apridisk_format::load(io_generic *io, uint32_t form_factor, floppy_image *image)
{
	desc_pc_sector sectors[80][2][18];
	uint8_t sector_data[MAX_SECTORS * SECTOR_SIZE];
	uint8_t *data_ptr = sector_data;
	int track_count = 0, head_count = 0, sector_count = 0;

	uint64_t file_size = io_generic_size(io);
	uint64_t file_offset = APR_HEADER_SIZE;

	while (file_offset < file_size)
	{
		// read sector header
		uint8_t sector_header[16];
		io_generic_read(io, sector_header, file_offset, 16);

		uint32_t type = pick_integer_le(&sector_header, 0, 4);
		uint16_t compression = pick_integer_le(&sector_header, 4, 2);
		uint16_t header_size = pick_integer_le(&sector_header, 6, 2);
		uint32_t data_size = pick_integer_le(&sector_header, 8, 4);

		file_offset += header_size;

		switch (type)
		{
		case APR_SECTOR:
			uint8_t head = pick_integer_le(&sector_header, 12, 1);
			uint8_t sector = pick_integer_le(&sector_header, 13, 1);
			uint8_t track = (uint8_t) pick_integer_le(&sector_header, 14, 2);

			track_count = std::max(track_count, int(unsigned(track)));
			head_count = std::max(head_count, int(unsigned(head)));
			sector_count = std::max(sector_count, int(unsigned(sector)));

			// build sector info
			sectors[track][head][sector - 1].head = head;
			sectors[track][head][sector - 1].sector = sector;
			sectors[track][head][sector - 1].track = track;
			sectors[track][head][sector - 1].size = SECTOR_SIZE >> 8;
			sectors[track][head][sector - 1].actual_size = SECTOR_SIZE;
			sectors[track][head][sector - 1].deleted = false;
			sectors[track][head][sector - 1].bad_crc = false;

			// read sector data
			switch (compression)
			{
			case APR_COMPRESSED:
				{
					uint8_t comp[3];
					io_generic_read(io, comp, file_offset, 3);
					uint16_t length = pick_integer_le(comp, 0, 2);

					if (length != SECTOR_SIZE)
						fatalerror("apridisk_format: Invalid compression length %04x\n", length);

					memset(data_ptr, comp[2], SECTOR_SIZE);
				}
				break;

			case APR_UNCOMPRESSED:
				io_generic_read(io, data_ptr, file_offset, SECTOR_SIZE);
				break;

			default:
				fatalerror("apridisk_format: Invalid compression %04x\n", compression);
			}

			sectors[track][head][sector - 1].data = data_ptr;
			data_ptr += SECTOR_SIZE;

			break;

		}

		file_offset += data_size;
	}

	// track/head index are zero-based, so increase the final count by 1
	track_count++;
	head_count++;

	int cell_count = (sector_count == 18 ? 200000 : 100000);

	// now build our final track info
	for (int track = 0; track < track_count; track++)
		for (int head = 0; head < head_count; head++)
			build_pc_track_mfm(track, head, image, cell_count, sector_count, sectors[track][head], 84, 80, 50, 22);

	return true;
}

bool apridisk_format::save(io_generic *io, floppy_image *image)
{
	return false;
}

bool apridisk_format::supports_save() const
{
	return false;
}

const floppy_format_type FLOPPY_APRIDISK_FORMAT = &floppy_image_format_creator<apridisk_format>;
