// license:BSD-3-Clause
// copyright-holders:Olivier Galibert, R. Belmont
/*********************************************************************

    ap2_dsk.h

    Apple II disk images

*********************************************************************/

#ifndef AP2_DISK_H
#define AP2_DISK_H

#include "flopimg.h"


/***************************************************************************

    Constants

***************************************************************************/

#define APPLE2_NIBBLE_SIZE                      416
#define APPLE2_SMALL_NIBBLE_SIZE        374
#define APPLE2_TRACK_COUNT                      35
#define APPLE2_SECTOR_COUNT                     16
#define APPLE2_SECTOR_SIZE                      256



/**************************************************************************/

LEGACY_FLOPPY_OPTIONS_EXTERN(apple2);

// new style code

class a2_16sect_format : public floppy_image_format_t
{
public:
		a2_16sect_format();

		virtual int identify(io_generic *io, uint32_t form_factor) override;
		virtual bool load(io_generic *io, uint32_t form_factor, floppy_image *image) override;
		virtual bool save(io_generic *io, floppy_image *image) override;

		virtual const char *name() const override;
		virtual const char *description() const override;
		virtual const char *extensions() const override;
		virtual bool supports_save() const override;

private:
		static const desc_e mac_gcr[];

		uint8_t gb(const uint8_t *buf, int ts, int &pos, int &wrap);
		void update_chk(const uint8_t *data, int size, uint32_t &chk);

		bool m_prodos_order;
};

extern const floppy_format_type FLOPPY_A216S_FORMAT;

class a2_rwts18_format : public floppy_image_format_t
{
public:
		a2_rwts18_format();

		virtual int identify(io_generic *io, uint32_t form_factor) override;
		virtual bool load(io_generic *io, uint32_t form_factor, floppy_image *image) override;
		virtual bool save(io_generic *io, floppy_image *image) override;

		virtual const char *name() const override;
		virtual const char *description() const override;
		virtual const char *extensions() const override;
		virtual bool supports_save() const override;

private:
		static const desc_e mac_gcr[];

		uint8_t gb(const uint8_t *buf, int ts, int &pos, int &wrap);
		void update_chk(const uint8_t *data, int size, uint32_t &chk);
};

extern const floppy_format_type FLOPPY_RWTS18_FORMAT;


class a2_edd_format : public floppy_image_format_t
{
public:
		a2_edd_format();

		virtual int identify(io_generic *io, uint32_t form_factor) override;
		virtual bool load(io_generic *io, uint32_t form_factor, floppy_image *image) override;
		virtual bool supports_save() const override;

		virtual const char *name() const override;
		virtual const char *description() const override;
		virtual const char *extensions() const override;

private:
		static uint8_t pick(const uint8_t *data, int pos);
};

extern const floppy_format_type FLOPPY_EDD_FORMAT;

#endif /* AP2_DISK_H */
