// license:BSD-3-Clause
// copyright-holders:Curt Coder
/*********************************************************************

    formats/victor9k_dsk.h

    Victor 9000 sector disk image format

*********************************************************************/

#ifndef VICTOR9K_DSK_H_
#define VICTOR9K_DSK_H_

#include "flopimg.h"

//#define USE_SCP 1

class victor9k_format : public floppy_image_format_t {
public:
	struct format {
		uint32_t form_factor;      // See floppy_image for possible values
		uint32_t variant;          // See floppy_image for possible values

		uint16_t sector_count;
		uint8_t track_count;
		uint8_t head_count;
		uint16_t sector_base_size;
	};

	victor9k_format();

	virtual const char *name() const override;
	virtual const char *description() const override;
	virtual const char *extensions() const override;

	virtual int identify(io_generic *io, uint32_t form_factor) override;
	virtual bool load(io_generic *io, uint32_t form_factor, floppy_image *image) override;
	virtual bool save(io_generic *io, floppy_image *image) override;
	virtual bool supports_save() const override { return true; }

	static int get_rpm(int head, int track);

protected:
	static const format formats[];

	static const uint32_t cell_size[9];
	static const int sectors_per_track[2][80];
	static const int speed_zone[2][80];
	static const int rpm[9];

	int find_size(io_generic *io, uint32_t form_factor);
	void log_boot_sector(uint8_t *data);
	floppy_image_format_t::desc_e* get_sector_desc(const format &f, int &current_size, int sector_count);
	void build_sector_description(const format &f, uint8_t *sectdata, uint32_t sect_offs, desc_s *sectors, int sector_count) const;
	int get_image_offset(const format &f, int head, int track);
	int compute_track_size(const format &f, int head, int track);
	void extract_sectors(floppy_image *image, const format &f, desc_s *sdesc, int track, int head, int sector_count);
};

extern const floppy_format_type FLOPPY_VICTOR_9000_FORMAT;


FLOPPY_IDENTIFY( victor9k_dsk_identify );

FLOPPY_CONSTRUCT( victor9k_dsk_construct );

#endif
