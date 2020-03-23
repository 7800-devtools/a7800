// license:BSD-3-Clause
// copyright-holders:Nathan Woods
/****************************************************************************

    library.h

    Code relevant to the Imgtool library; analogous to the MESS/MAME driver
    list.

    Unlike MESS and MAME which have static driver lists, Imgtool has a
    concept of a library and this library is built at startup time.
    dynamic for which modules are added to.  This makes "dynamic" modules
    much easier

****************************************************************************/

#ifndef LIBRARY_H
#define LIBRARY_H

#include <time.h>
#include <list>

#include "corestr.h"
#include "opresolv.h"
#include "stream.h"
#include "unicode.h"
#include "charconv.h"
#include "pool.h"

namespace imgtool
{
	class image;
	class partition;
	class directory;
};

enum imgtool_suggestion_viability_t
{
	SUGGESTION_END,
	SUGGESTION_POSSIBLE,
	SUGGESTION_RECOMMENDED
};

union filterinfo
{
	int64_t   i;                                          /* generic integers */
	void *  p;                                          /* generic pointers */
	void *  f;                                          /* generic function pointers */
	const char *s;                                      /* generic strings */

	imgtoolerr_t (*read_file)(imgtool::partition &partition, const char *filename, const char *fork, imgtool::stream &destf);
	imgtoolerr_t (*write_file)(imgtool::partition &partition, const char *filename, const char *fork, imgtool::stream &sourcef, util::option_resolution *opts);
	imgtoolerr_t (*check_stream)(imgtool::stream &stream, imgtool_suggestion_viability_t *viability);
};

typedef void (*filter_getinfoproc)(uint32_t state, union filterinfo *info);

struct imgtool_dirent
{
	char filename[1024];
	char attr[64];
	uint64_t filesize;

	time_t creation_time;
	time_t lastmodified_time;
	time_t lastaccess_time;

	char softlink[1024];
	char comment[256];

	/* flags */
	unsigned int eof : 1;
	unsigned int corrupt : 1;
	unsigned int directory : 1;
	unsigned int hardlink : 1;
};

struct imgtool_chainent
{
	uint8_t level;
	uint64_t block;
};

enum imgtool_forktype_t
{
	FORK_END,
	FORK_DATA,
	FORK_RESOURCE,
	FORK_ALTERNATE
};

struct imgtool_forkent
{
	imgtool_forktype_t type;
	uint64_t size;
	char forkname[64];
};

struct imgtool_transfer_suggestion
{
	imgtool_suggestion_viability_t viability;
	filter_getinfoproc filter;
	const char *fork;
	const char *description;
};

enum
{
	/* --- the following bits of info are returned as 64-bit signed integers --- */
	IMGTOOLATTR_INT_FIRST = 0x00000,
	IMGTOOLATTR_INT_MAC_TYPE,
	IMGTOOLATTR_INT_MAC_CREATOR,
	IMGTOOLATTR_INT_MAC_FINDERFLAGS,
	IMGTOOLATTR_INT_MAC_COORDX,
	IMGTOOLATTR_INT_MAC_COORDY,
	IMGTOOLATTR_INT_MAC_FINDERFOLDER,
	IMGTOOLATTR_INT_MAC_ICONID,
	IMGTOOLATTR_INT_MAC_SCRIPTCODE,
	IMGTOOLATTR_INT_MAC_EXTENDEDFLAGS,
	IMGTOOLATTR_INT_MAC_COMMENTID,
	IMGTOOLATTR_INT_MAC_PUTAWAYDIRECTORY,

	/* --- the following bits of info are returned as pointers to data or functions --- */
	IMGTOOLATTR_PTR_FIRST = 0x10000,

	/* --- the following bits of info are returned as NULL-terminated strings --- */
	IMGTOOLATTR_STR_FIRST = 0x20000,

	/* --- the following bits of info are returned as time_t values --- */
	IMGTOOLATTR_TIME_FIRST = 0x30000,
	IMGTOOLATTR_TIME_CREATED,
	IMGTOOLATTR_TIME_LASTMODIFIED
};

union imgtool_attribute
{
	int64_t   i;
	time_t  t;
};

struct imgtool_iconinfo
{
	unsigned icon16x16_specified : 1;
	unsigned icon32x32_specified : 1;
	uint32_t icon16x16[16][16];
	uint32_t icon32x32[32][32];
};

enum
{
	/* --- the following bits of info are returned as 64-bit signed integers --- */
	IMGTOOLINFO_INT_FIRST = 0x00000,
	IMGTOOLINFO_INT_IMAGE_EXTRA_BYTES,
	IMGTOOLINFO_INT_PARTITION_EXTRA_BYTES,
	IMGTOOLINFO_INT_DIRECTORY_EXTRA_BYTES,
	IMGTOOLINFO_INT_PATH_SEPARATOR,
	IMGTOOLINFO_INT_ALTERNATE_PATH_SEPARATOR,
	IMGTOOLINFO_INT_PREFER_UCASE,
	IMGTOOLINFO_INT_INITIAL_PATH_SEPARATOR,
	IMGTOOLINFO_INT_OPEN_IS_STRICT,
	IMGTOOLINFO_INT_SUPPORTS_CREATION_TIME,
	IMGTOOLINFO_INT_SUPPORTS_LASTMODIFIED_TIME,
	IMGTOOLINFO_INT_TRACKS_ARE_CALLED_CYLINDERS,
	IMGTOOLINFO_INT_WRITING_UNTESTED,
	IMGTOOLINFO_INT_CREATION_UNTESTED,
	IMGTOOLINFO_INT_SUPPORTS_BOOTBLOCK,
	IMGTOOLINFO_INT_BLOCK_SIZE,

	IMGTOOLINFO_INT_CLASS_SPECIFIC = 0x08000,

	/* --- the following bits of info are returned as pointers to data or functions --- */
	IMGTOOLINFO_PTR_FIRST = 0x10000,

	IMGTOOLINFO_PTR_OPEN,
	IMGTOOLINFO_PTR_CREATE,
	IMGTOOLINFO_PTR_CLOSE,
	IMGTOOLINFO_PTR_OPEN_PARTITION,
	IMGTOOLINFO_PTR_CREATE_PARTITION,
	IMGTOOLINFO_PTR_INFO,
	IMGTOOLINFO_PTR_BEGIN_ENUM,
	IMGTOOLINFO_PTR_NEXT_ENUM,
	IMGTOOLINFO_PTR_CLOSE_ENUM,
	IMGTOOLINFO_PTR_FREE_SPACE,
	IMGTOOLINFO_PTR_READ_FILE,
	IMGTOOLINFO_PTR_WRITE_FILE,
	IMGTOOLINFO_PTR_DELETE_FILE,
	IMGTOOLINFO_PTR_LIST_FORKS,
	IMGTOOLINFO_PTR_CREATE_DIR,
	IMGTOOLINFO_PTR_DELETE_DIR,
	IMGTOOLINFO_PTR_LIST_ATTRS,
	IMGTOOLINFO_PTR_GET_ATTRS,
	IMGTOOLINFO_PTR_SET_ATTRS,
	IMGTOOLINFO_PTR_ATTR_NAME,
	IMGTOOLINFO_PTR_GET_ICON_INFO,
	IMGTOOLINFO_PTR_SUGGEST_TRANSFER,
	IMGTOOLINFO_PTR_GET_CHAIN,
	IMGTOOLINFO_PTR_GET_GEOMETRY,
	IMGTOOLINFO_PTR_READ_SECTOR,
	IMGTOOLINFO_PTR_WRITE_SECTOR,
	IMGTOOLINFO_PTR_READ_BLOCK,
	IMGTOOLINFO_PTR_WRITE_BLOCK,
	IMGTOOLINFO_PTR_APPROVE_FILENAME_CHAR,
	IMGTOOLINFO_PTR_CREATEIMAGE_OPTGUIDE,
	IMGTOOLINFO_PTR_WRITEFILE_OPTGUIDE,
	IMGTOOLINFO_PTR_MAKE_CLASS,
	IMGTOOLINFO_PTR_LIST_PARTITIONS,
	IMGTOOLINFO_PTR_CHARCONVERTER,

	IMGTOOLINFO_PTR_CLASS_SPECIFIC = 0x18000,

	/* --- the following bits of info are returned as NULL-terminated strings --- */
	IMGTOOLINFO_STR_FIRST = 0x20000,

	IMGTOOLINFO_STR_NAME,
	IMGTOOLINFO_STR_DESCRIPTION,
	IMGTOOLINFO_STR_FILE,
	IMGTOOLINFO_STR_FILE_EXTENSIONS,
	IMGTOOLINFO_STR_EOLN,
	IMGTOOLINFO_STR_CREATEIMAGE_OPTSPEC,
	IMGTOOLINFO_STR_WRITEFILE_OPTSPEC,

	IMGTOOLINFO_STR_CLASS_SPECIFIC = 0x28000
};



union imgtoolinfo;

struct imgtool_class;
typedef void (*imgtool_get_info)(const imgtool_class *, uint32_t, union imgtoolinfo *);

struct imgtool_class
{
	imgtool_get_info get_info;
	imgtool_get_info derived_get_info;
	void *derived_param;
};



namespace imgtool
{
	class partition_info
	{
	public:
		partition_info(imgtool_get_info get_info, uint64_t base_block, uint64_t block_count)
			: m_base_block(base_block)
			, m_block_count(block_count)
		{
			memset(&m_imgclass, 0, sizeof(m_imgclass));
			m_imgclass.get_info = get_info;
		}

		partition_info(imgtool_class imgclass, uint64_t base_block, uint64_t block_count)
			: m_imgclass(imgclass)
			, m_base_block(base_block)
			, m_block_count(block_count)
		{
		}

		const imgtool_class &imgclass() const { return m_imgclass; }
		uint64_t base_block() const { return m_base_block; }
		uint64_t block_count() const { return m_block_count; }

	private:
		imgtool_class           m_imgclass;
		uint64_t                m_base_block;
		uint64_t                m_block_count;
	};
};


union imgtoolinfo
{
	int64_t   i;                                          /* generic integers */
	void *  p;                                          /* generic pointers */
	void *  f;                                          /* generic function pointers */
	char *  s;                                          /* generic strings */

	imgtoolerr_t    (*open)             (imgtool::image &image, imgtool::stream::ptr &&stream);
	void            (*close)            (imgtool::image &image);
	imgtoolerr_t    (*create)           (imgtool::image &image, imgtool::stream::ptr &&stream, util::option_resolution *opts);
	imgtoolerr_t    (*create_partition) (imgtool::image &image, uint64_t first_block, uint64_t block_count);
	void            (*info)             (imgtool::image &image, std::ostream &stream);
	imgtoolerr_t    (*begin_enum)       (imgtool::directory &enumeration, const char *path);
	imgtoolerr_t    (*next_enum)        (imgtool::directory &enumeration, imgtool_dirent &ent);
	void            (*close_enum)       (imgtool::directory &enumeration);
	imgtoolerr_t    (*open_partition)   (imgtool::partition &partition, uint64_t first_block, uint64_t block_count);
	imgtoolerr_t    (*free_space)       (imgtool::partition &partition, uint64_t *size);
	imgtoolerr_t    (*read_file)        (imgtool::partition &partition, const char *filename, const char *fork, imgtool::stream &destf);
	imgtoolerr_t    (*write_file)       (imgtool::partition &partition, const char *filename, const char *fork, imgtool::stream &sourcef, util::option_resolution *opts);
	imgtoolerr_t    (*delete_file)      (imgtool::partition &partition, const char *filename);
	imgtoolerr_t    (*list_forks)       (imgtool::partition &partition, const char *path, imgtool_forkent *ents, size_t len);
	imgtoolerr_t    (*create_dir)       (imgtool::partition &partition, const char *path);
	imgtoolerr_t    (*delete_dir)       (imgtool::partition &partition, const char *path);
	imgtoolerr_t    (*list_attrs)       (imgtool::partition &partition, const char *path, uint32_t *attrs, size_t len);
	imgtoolerr_t    (*get_attrs)        (imgtool::partition &partition, const char *path, const uint32_t *attrs, imgtool_attribute *values);
	imgtoolerr_t    (*set_attrs)        (imgtool::partition &partition, const char *path, const uint32_t *attrs, const imgtool_attribute *values);
	imgtoolerr_t    (*attr_name)        (uint32_t attribute, const imgtool_attribute *attr, char *buffer, size_t buffer_len);
	imgtoolerr_t    (*get_iconinfo)     (imgtool::partition &partition, const char *path, imgtool_iconinfo *iconinfo);
	imgtoolerr_t    (*suggest_transfer) (imgtool::partition &partition, const char *path, imgtool_transfer_suggestion *suggestions, size_t suggestions_length);
	imgtoolerr_t    (*get_chain)        (imgtool::partition &partition, const char *path, imgtool_chainent *chain, size_t chain_size);
	imgtoolerr_t    (*get_geometry)     (imgtool::image &image, uint32_t *tracks, uint32_t *heads, uint32_t *sectors);
	imgtoolerr_t    (*read_sector)      (imgtool::image &image, uint32_t track, uint32_t head, uint32_t sector, std::vector<uint8_t> &buffer);
	imgtoolerr_t    (*write_sector)     (imgtool::image &image, uint32_t track, uint32_t head, uint32_t sector, const void *buffer, size_t len, int ddam);
	imgtoolerr_t    (*read_block)       (imgtool::image &image, void *buffer, uint64_t block);
	imgtoolerr_t    (*write_block)      (imgtool::image &image, const void *buffer, uint64_t block);
	imgtoolerr_t    (*list_partitions)  (imgtool::image &image, std::vector<imgtool::partition_info> &partitions);
	int             (*approve_filename_char)(char32_t ch);
	int             (*make_class)(int index, imgtool_class *imgclass);

	const util::option_guide *createimage_optguide;
	const util::option_guide *writefile_optguide;
	const imgtool::charconverter *charconverter;
};



static inline int64_t imgtool_get_info_int(const imgtool_class *imgclass, uint32_t state)
{
	union imgtoolinfo info;
	info.i = 0;
	imgclass->get_info(imgclass, state, &info);
	return info.i;
}

static inline void *imgtool_get_info_ptr(const imgtool_class *imgclass, uint32_t state)
{
	union imgtoolinfo info;
	info.p = nullptr;
	imgclass->get_info(imgclass, state, &info);
	return info.p;
}

static inline void *imgtool_get_info_fct(const imgtool_class *imgclass, uint32_t state)
{
	union imgtoolinfo info;
	info.f = nullptr;
	imgclass->get_info(imgclass, state, &info);
	return info.f;
}

static inline char *imgtool_get_info_string(const imgtool_class *imgclass, uint32_t state)
{
	union imgtoolinfo info;
	info.s = nullptr;
	imgclass->get_info(imgclass, state, &info);
	return info.s;
}

/* circular string buffer */
char *imgtool_temp_str(void);



struct imgtool_module
{
	imgtool_class imgclass;

	const char *name;
	const char *description;
	const char *extensions;
	const char *eoln;

	size_t image_extra_bytes;

	/* flags */
	unsigned int initial_path_separator : 1;
	unsigned int open_is_strict : 1;
	unsigned int tracks_are_called_cylinders : 1;   /* used for hard drivers */
	unsigned int writing_untested : 1;              /* used when we support writing, but not in main build */
	unsigned int creation_untested : 1;             /* used when we support creation, but not in main build */

	imgtoolerr_t    (*open)         (imgtool::image &image, imgtool::stream::ptr &&stream);
	void            (*close)        (imgtool::image &image);
	void            (*info)         (imgtool::image &image, std::ostream &stream);
	imgtoolerr_t    (*create)       (imgtool::image &image, imgtool::stream::ptr &&stream, util::option_resolution *opts);
	imgtoolerr_t    (*get_geometry) (imgtool::image &image, uint32_t *track, uint32_t *heads, uint32_t *sectors);
	imgtoolerr_t    (*read_sector)  (imgtool::image &image, uint32_t track, uint32_t head, uint32_t sector, std::vector<uint8_t> &buffer);
	imgtoolerr_t    (*write_sector) (imgtool::image &image, uint32_t track, uint32_t head, uint32_t sector, const void *buffer, size_t len);
	imgtoolerr_t    (*read_block)   (imgtool::image &image, void *buffer, uint64_t block);
	imgtoolerr_t    (*write_block)  (imgtool::image &image, const void *buffer, uint64_t block);
	imgtoolerr_t    (*list_partitions)(imgtool::image &image, std::vector<imgtool::partition_info> &partitions);

	uint32_t block_size;

	const util::option_guide *createimage_optguide;
	const char *createimage_optspec;

	const void *extra;
};

namespace imgtool {

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// imgtool "library" - equivalent to the MAME driver list
class library
{
public:
	typedef std::list<std::unique_ptr<imgtool_module> > modulelist;

	enum class sort_type
	{
		NAME,
		DESCRIPTION
	};

	library();
	~library();

	// adds a module to an imgtool library
	void add(imgtool_get_info get_info);

	// seeks out and removes a module from an imgtool library
	void unlink(const std::string &module_name);

	// sorts an imgtool library
	void sort(sort_type sort);

	// finds a module
	const imgtool_module *findmodule(const std::string &module_name);

	// module iteration
	const modulelist &modules() { return m_modules; }

private:
	object_pool *   m_pool;
	modulelist      m_modules;

	// internal lookup and iteration
	modulelist::iterator find(const std::string &module_name);

	// helpers
	void add_class(const imgtool_class *imgclass);
	int module_compare(const imgtool_module *m1, const imgtool_module *m2, sort_type sort);

	// memory allocators for pooled library memory (these should go away in further C++-ification)
	void *imgtool_library_malloc(size_t mem);
	char *imgtool_library_strdup(const char *s);
	char *imgtool_library_strdup_allow_null(const char *s);
};

} // namespace imgtool

#endif // LIBRARY_H
