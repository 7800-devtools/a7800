// license:BSD-3-Clause
// copyright-holders:Curt Coder
/*********************************************************************

    formats/d64_dsk.h

    Commodore 4040/1541/1551 sector disk image format

*********************************************************************/

#ifndef D64_DSK_H_
#define D64_DSK_H_

#include "flopimg.h"

class d64_format : public floppy_image_format_t {
public:
	struct format {
		uint32_t form_factor;      // See floppy_image for possible values
		uint32_t variant;          // See floppy_image for possible values

		uint16_t sector_count;
		uint8_t track_count;
		uint8_t head_count;
		uint16_t sector_base_size;
		uint8_t gap_1;
		uint8_t gap_2;
	};

	d64_format();
	d64_format(const format *formats);

	virtual const char *name() const override;
	virtual const char *description() const override;
	virtual const char *extensions() const override;

	virtual int identify(io_generic *io, uint32_t form_factor) override;
	virtual bool load(io_generic *io, uint32_t form_factor, floppy_image *image) override;
	virtual bool save(io_generic *io, floppy_image *image) override;
	virtual bool supports_save() const override { return true; }

protected:
	enum
	{
		ERROR_00 = 1,
		ERROR_20,       /* header block not found */
		ERROR_21,       /* no sync character */
		ERROR_22,       /* data block not present */
		ERROR_23,       /* checksum error in data block */
		ERROR_24,       /* write verify (on format) UNIMPLEMENTED */
		ERROR_25,       /* write verify error UNIMPLEMENTED */
		ERROR_26,       /* write protect on UNIMPLEMENTED */
		ERROR_27,       /* checksum error in header block */
		ERROR_28,       /* write error UNIMPLEMENTED */
		ERROR_29,       /* disk ID mismatch */
		ERROR_74        /* disk not ready (no device 1) UNIMPLEMENTED */
	};

	const format *formats;

	int find_size(io_generic *io, uint32_t form_factor) const;
	virtual int get_physical_track(const format &f, int head, int track);
	virtual uint32_t get_cell_size(const format &f, int track);
	virtual int get_sectors_per_track(const format &f, int track);
	virtual int get_disk_id_offset(const format &f);
	void get_disk_id(const format &f, io_generic *io, uint8_t &id1, uint8_t &id2);
	virtual int get_image_offset(const format &f, int head, int track);
	int compute_track_size(const format &f, int track);
	virtual int get_gap2(const format &f, int head, int track) { return f.gap_2; }
	virtual floppy_image_format_t::desc_e* get_sector_desc(const format &f, int &current_size, int sector_count, uint8_t id1, uint8_t id2, int gap_2);
	void build_sector_description(const format &f, uint8_t *sectdata, uint32_t sect_offs, uint32_t error_offs, desc_s *sectors, int sector_count) const;
	virtual void fix_end_gap(floppy_image_format_t::desc_e* desc, int remaining_size);
	void extract_sectors(floppy_image *image, const format &f, desc_s *sdesc, int track, int head, int sector_count);

	static const format file_formats[];

	static const uint32_t cell_size[];
	static const int sectors_per_track[];
	static const int speed_zone[];
};

extern const floppy_format_type FLOPPY_D64_FORMAT;



#endif
