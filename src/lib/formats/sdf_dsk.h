// license:BSD-3-Clause
// copyright-holders:Wilbert Pol, tim lindner
/*********************************************************************

    formats/sdf_dsk.h

    SDF disk images. Format created by Darren Atkinson for use with
    his CoCoSDC floppy disk emulator.

*********************************************************************/

#ifndef SDF_DSK_H
#define SDF_DSK_H

#include "flopimg.h"

/**************************************************************************/

class sdf_format : public floppy_image_format_t
{
public:
	sdf_format();

	virtual int identify(io_generic *io, uint32_t form_factor) override;
	virtual bool load(io_generic *io, uint32_t form_factor, floppy_image *image) override;
	virtual bool save(io_generic *io, floppy_image *image) override;

	virtual const char *name() const override;
	virtual const char *description() const override;
	virtual const char *extensions() const override;
	virtual bool supports_save() const override;

protected:
	static constexpr int HEADER_SIZE  = 512;
	static constexpr int TRACK_HEADER_SIZE  = 256;
	static constexpr int TRACK_SIZE  = 6250;
	static constexpr int TRACK_PADDING  = 150;
	static constexpr int TOTAL_TRACK_SIZE  = TRACK_HEADER_SIZE + TRACK_SIZE + TRACK_PADDING;

	static constexpr int SECTOR_SLOT_COUNT  = 31;
};

extern const floppy_format_type FLOPPY_SDF_FORMAT;

#endif /* SDF_DSK_H */
