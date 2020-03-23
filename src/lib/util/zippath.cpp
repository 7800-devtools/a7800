// license:BSD-3-Clause
// copyright-holders:Nathan Woods
/***************************************************************************

    zippath.c

    File/directory/path operations that work with ZIP files

***************************************************************************/

#include "zippath.h"
#include "unzip.h"
#include "corestr.h"
#include "osdcore.h"

#include <stdlib.h>

#include <cassert>
#include <cctype>
#include <forward_list>
#include <new>


namespace util {
/***************************************************************************
    TYPE DEFINITIONS
***************************************************************************/

/**
 * @class   zippath_directory
 *
 * @brief   A zippath directory.
 */

class zippath_directory
{
public:
	zippath_directory()
		: returned_parent(false)
		, directory()
		, called_zip_first(false)
		, zipfile()
		, returned_dirlist()
	{
	}

	/* common */
	/** @brief  true to returned parent. */
	bool returned_parent;
	/** @brief  The returned entry. */
	osd::directory::entry returned_entry;

	/* specific to normal directories */
	/** @brief  Pathname of the directory. */
	osd::directory::ptr directory;

	/* specific to ZIP directories */
	/** @brief  true to called zip first. */
	bool called_zip_first;
	/** @brief  The zipfile. */
	archive_file::ptr zipfile;
	/** @brief  The zipprefix. */
	std::string zipprefix;
	/** @brief  The returned dirlist. */
	std::forward_list<std::string> returned_dirlist;
};


/***************************************************************************
    FUNCTION PROTOTYPES
***************************************************************************/

static int zippath_find_sub_path(archive_file &zipfile, std::string const &subpath, osd::directory::entry::entry_type &type);
static bool is_zip_file(std::string const &path);
static bool is_zip_file_separator(char c);
static bool is_7z_file(std::string const &path);


/***************************************************************************
    PATH OPERATIONS
***************************************************************************/

/**
 * @fn  int is_path_separator(char c)
 *
 * @brief   ============================================================
 *            is_path_separator
 *          ============================================================.
 *
 * @param   c   The character.
 *
 * @return  An int.
 */

int is_path_separator(char c)
{
	return (c == '/') || (c == '\\');
}

// -------------------------------------------------
//  parse_parent_path - parses out the parent path
// -------------------------------------------------

/**
 * @fn  static void parse_parent_path(const std::string &path, std::string::size_type *beginpos, std::string::size_type *endpos)
 *
 * @brief   Parse parent path.
 *
 * @param   path                Full pathname of the file.
 * @param [in,out]  beginpos    If non-null, the beginpos.
 * @param [in,out]  endpos      If non-null, the endpos.
 */

static void parse_parent_path(const std::string &path, std::string::size_type *beginpos, std::string::size_type *endpos)
{
	std::string::size_type length = path.length();
	std::string::size_type pos;

	// skip over trailing path separators
	pos = length ? (length - 1) : std::string::npos;
	while ((pos > 0) && (pos != std::string::npos) && is_path_separator(path[pos]))
		pos--;

	// return endpos
	if (endpos != nullptr)
		*endpos = pos;

	// now skip until we find a path separator
	while ((pos != std::string::npos) && !is_path_separator(path[pos]))
		pos = (pos > 0) ? pos - 1 : std::string::npos;

	// return beginpos
	if (beginpos != nullptr)
		*beginpos = pos;
}



// -------------------------------------------------
//  zippath_parent - retrieves the parent directory
// -------------------------------------------------

std::string &zippath_parent(std::string &dst, const std::string &path)
{
	std::string::size_type pos;
	parse_parent_path(path, &pos, nullptr);

	if (pos != std::string::npos)
		dst = path.substr(0, pos + 1);
	else
		dst.clear();
	return dst;
}



// -------------------------------------------------
//  zippath_parent - retrieves the parent directory
// -------------------------------------------------

std::string zippath_parent(const std::string &path)
{
	std::string result;
	zippath_parent(result, path);
	return result;
}



// -------------------------------------------------
//  zippath_parent_basename - retrieves the parent
//  directory basename
// -------------------------------------------------

/**
 * @fn  std::string &zippath_parent_basename(std::string &dst, const std::string &path)
 *
 * @brief   Zippath parent basename.
 *
 * @param [in,out]  dst Destination for the.
 * @param   path        Full pathname of the file.
 *
 * @return  A std::string&amp;
 */

std::string &zippath_parent_basename(std::string &dst, const std::string &path)
{
	std::string::size_type beginpos, endpos;
	parse_parent_path(path, &beginpos, &endpos);
	dst.copy((char*)(path.c_str() + beginpos + 1), endpos - beginpos);
	return dst;
}



// -------------------------------------------------
//  zippath_parent_basename - retrieves the parent
//  directory basename
// -------------------------------------------------

std::string zippath_parent_basename(const std::string &path)
{
	std::string result;
	zippath_parent_basename(result, path);
	return result;
}



// -------------------------------------------------
//  zippath_combine - combines two paths
// -------------------------------------------------

/**
 * @fn  std::string &zippath_combine(std::string &dst, const char *path1, const char *path2)
 *
 * @brief   Zippath combine.
 *
 * @param [in,out]  dst Destination for the.
 * @param   path1       The first path.
 * @param   path2       The second path.
 *
 * @return  A std::string&amp;
 */

std::string &zippath_combine(std::string &dst, const std::string &path1, const std::string &path2)
{
	if (path2 == ".")
	{
		dst.assign(path1);
	}
	else if (path2 == "..")
	{
		dst = zippath_parent(path1);
	}
	else if (osd_is_absolute_path(path2))
	{
		dst.assign(path2);
	}
	else if (!path1.empty() && !is_path_separator(*path1.rbegin()))
	{
		dst.assign(path1).append(PATH_SEPARATOR).append(path2);
	}
	else
	{
		dst.assign(path1).append(path2);
	}
	return dst;
}



// -------------------------------------------------
//  zippath_combine - combines two paths
// -------------------------------------------------

std::string zippath_combine(const std::string &path1, const std::string &path2)
{
	std::string result;
	zippath_combine(result, path1, path2);
	return result;
}



/***************************************************************************
    FILE OPERATIONS
***************************************************************************/

// -------------------------------------------------
//  file_error_from_zip_error - translates a
//  osd_file::error to a zip_error
// -------------------------------------------------

/**
 * @fn  static osd_file::error file_error_from_zip_error(archive_file::error ziperr)
 *
 * @brief   File error from zip error.
 *
 * @param   ziperr  The ziperr.
 *
 * @return  A osd_file::error.
 */

static osd_file::error file_error_from_zip_error(archive_file::error ziperr)
{
	osd_file::error filerr;
	switch(ziperr)
	{
	case archive_file::error::NONE:
		filerr = osd_file::error::NONE;
		break;
	case archive_file::error::OUT_OF_MEMORY:
		filerr = osd_file::error::OUT_OF_MEMORY;
		break;
	case archive_file::error::BAD_SIGNATURE:
	case archive_file::error::DECOMPRESS_ERROR:
	case archive_file::error::FILE_TRUNCATED:
	case archive_file::error::FILE_CORRUPT:
	case archive_file::error::UNSUPPORTED:
	case archive_file::error::FILE_ERROR:
		filerr = osd_file::error::INVALID_DATA;
		break;
	case archive_file::error::BUFFER_TOO_SMALL:
	default:
		filerr = osd_file::error::FAILURE;
		break;
	}
	return filerr;
}


// -------------------------------------------------
//  create_core_file_from_zip - creates a core_file
//  from a zip file entry
// -------------------------------------------------

/**
 * @fn  static osd_file::error create_core_file_from_zip(archive_file *zip, util::core_file::ptr &file)
 *
 * @brief   Creates core file from zip.
 *
 * @param [in,out]  zip     If non-null, the zip.
 * @param   header          The header.
 * @param [in,out]  file    [in,out] If non-null, the file.
 *
 * @return  The new core file from zip.
 */

static osd_file::error create_core_file_from_zip(archive_file &zip, util::core_file::ptr &file)
{
	osd_file::error filerr;
	archive_file::error ziperr;
	void *ptr;

	ptr = malloc(zip.current_uncompressed_length());
	if (ptr == nullptr)
	{
		filerr = osd_file::error::OUT_OF_MEMORY;
		goto done;
	}

	ziperr = zip.decompress(ptr, zip.current_uncompressed_length());
	if (ziperr != archive_file::error::NONE)
	{
		filerr = file_error_from_zip_error(ziperr);
		goto done;
	}

	filerr = util::core_file::open_ram_copy(ptr, zip.current_uncompressed_length(), OPEN_FLAG_READ, file);
	if (filerr != osd_file::error::NONE)
		goto done;

done:
	if (ptr != nullptr)
		free(ptr);
	return filerr;
}


// -------------------------------------------------
//  zippath_fopen - opens a zip path file
// -------------------------------------------------

/**
 * @fn  osd_file::error zippath_fopen(const std::string &filename, uint32_t openflags, util::core_file::ptr &file, std::string &revised_path)
 *
 * @brief   Zippath fopen.
 *
 * @param   filename                Filename of the file.
 * @param   openflags               The openflags.
 * @param [in,out]  file            [in,out] If non-null, the file.
 * @param [in,out]  revised_path    Full pathname of the revised file.
 *
 * @return  A osd_file::error.
 */

osd_file::error zippath_fopen(const std::string &filename, uint32_t openflags, util::core_file::ptr &file, std::string &revised_path)
{
	osd_file::error filerr = osd_file::error::NOT_FOUND;
	archive_file::error ziperr;
	archive_file::ptr zip;
	int header;
	osd::directory::entry::entry_type entry_type;
	int len;

	/* first, set up the two types of paths */
	std::string mainpath(filename);
	std::string subpath;
	file = nullptr;

	/* loop through */
	while((file == nullptr) && (mainpath.length() > 0)
		&& ((openflags == OPEN_FLAG_READ) || (subpath.length() == 0)))
	{
		/* is the mainpath a ZIP path? */
		if (is_zip_file(mainpath) || is_7z_file(mainpath))
		{
			/* this file might be a zip file - lets take a look */
			ziperr = is_zip_file(mainpath) ? archive_file::open_zip(mainpath, zip) : archive_file::open_7z(mainpath, zip);
			if (ziperr == archive_file::error::NONE)
			{
				/* it is a zip file - error if we're not opening for reading */
				if (openflags != OPEN_FLAG_READ)
				{
					filerr = osd_file::error::ACCESS_DENIED;
					goto done;
				}

				if (subpath.length() > 0)
					header = zippath_find_sub_path(*zip, subpath, entry_type);
				else
					header = zip->first_file();

				if (header < 0)
				{
					filerr = osd_file::error::NOT_FOUND;
					goto done;
				}

				/* attempt to read the file */
				filerr = create_core_file_from_zip(*zip, file);
				if (filerr != osd_file::error::NONE)
					goto done;

				/* update subpath, if appropriate */
				if (subpath.length() == 0)
					subpath.assign(zip->current_name());

				/* we're done */
				goto done;
			}
		}

		if (subpath.length() == 0)
			filerr = util::core_file::open(filename, openflags, file);
		else
			filerr = osd_file::error::NOT_FOUND;

		/* if we errored, then go up a directory */
		if (filerr != osd_file::error::NONE)
		{
			/* go up a directory */
			auto temp = zippath_parent(mainpath);

			/* append to the sub path */
			if (subpath.length() > 0)
			{
				std::string temp2;
				mainpath = mainpath.substr(temp.length());
				temp2.assign(mainpath).append(PATH_SEPARATOR).append(subpath);
				subpath.assign(temp2);
			}
			else
			{
				mainpath = mainpath.substr(temp.length());
				subpath.assign(mainpath);
			}
			/* get the new main path, truncating path separators */
			len = temp.length();
			while (len > 0 && is_zip_file_separator(temp[len - 1]))
				len--;
			temp = temp.substr(0, len);
			mainpath.assign(temp);
		}
	}

done:
	/* store the revised path */
	revised_path.clear();
	if (filerr == osd_file::error::NONE)
	{
		/* cannonicalize mainpath */
		std::string alloc_fullpath;
		filerr = osd_get_full_path(alloc_fullpath, mainpath);
		if (filerr == osd_file::error::NONE)
		{
			revised_path = alloc_fullpath;
			if (subpath.length() > 0)
				revised_path.append(PATH_SEPARATOR).append(subpath);
		}
	}

	return filerr;
}


/***************************************************************************
    DIRECTORY OPERATIONS
***************************************************************************/

// -------------------------------------------------
//  is_root - tests to see if this path is the root
// -------------------------------------------------

/**
 * @fn  static int is_root(const char *path)
 *
 * @brief   Is root.
 *
 * @param   path    Full pathname of the file.
 *
 * @return  An int.
 */

static int is_root(const char *path)
{
	int i = 0;

	/* skip drive letter */
	if (isalpha(path[i]) && (path[i + 1] == ':'))
		i += 2;

	/* skip path separators */
	while (is_path_separator(path[i]))
		i++;

	return path[i] == '\0';
}



// -------------------------------------------------
//  is_7z_file - tests to see if this file is a
//  7-zip file
// -------------------------------------------------

/**
 * @fn  static int is_7z_file(const char *path)
 *
 * @brief   Is 7z file.
 *
 * @param   path    Full pathname of the file.
 *
 * @return  An int.
 */

static bool is_7z_file(std::string const &path)
{
	auto const s = path.rfind('.');
	return (std::string::npos != s) && !core_stricmp(path.c_str() + s, ".7z");
}


// -------------------------------------------------
//  is_zip_file - tests to see if this file is a
//  ZIP file
// -------------------------------------------------

static bool is_zip_file(std::string const &path)
{
	auto const s = path.rfind('.');
	return (std::string::npos != s) && !core_stricmp(path.c_str() + s, ".zip");
}



// -------------------------------------------------
//  is_zip_file_separator - returns whether this
//  character is a path separator within a ZIP file
// -------------------------------------------------

/**
 * @fn  static int is_zip_file_separator(char c)
 *
 * @brief   Is zip file separator.
 *
 * @param   c   The character.
 *
 * @return  An int.
 */

static bool is_zip_file_separator(char c)
{
	return (c == '/') || (c == '\\');
}



// -------------------------------------------------
//  is_zip_path_separator - returns whether this
//  character is a path separator within a ZIP path
// -------------------------------------------------

/**
 * @fn  static int is_zip_path_separator(char c)
 *
 * @brief   Is zip path separator.
 *
 * @param   c   The character.
 *
 * @return  An int.
 */

static bool is_zip_path_separator(char c)
{
	return is_zip_file_separator(c) || is_path_separator(c);
}



// -------------------------------------------------
//  next_path_char - lexes out the next path
//  character, normalizing separators as '/'
// -------------------------------------------------

/**
 * @fn  static char next_path_char(const char *s, int *pos)
 *
 * @brief   Next path character.
 *
 * @param   s           The const char * to process.
 * @param [in,out]  pos If non-null, the position.
 *
 * @return  A char.
 */

static char next_path_char(std::string const &s, std::string::size_type &pos)
{
	// skip over any initial separators
	if (pos == 0)
	{
		while ((pos < s.length()) && is_zip_file_separator(s[pos]))
			pos++;
	}

	// are we at a path separator?
	if (pos == s.length())
	{
		// return NUL
		return '\0';
	}
	else if (is_zip_file_separator(s[pos]))
	{
		// skip over path separators
		while((pos < s.length()) && is_zip_file_separator(s[pos]))
			pos++;

		// normalize as '/'
		return '/';
	}
	else
	{
		// return character
		return std::tolower(s[pos++]);
	}
}




// -------------------------------------------------
//  zippath_find_sub_path - attempts to identify the
//  type of a sub path in a zip file
// -------------------------------------------------

/**
 * @fn  static const zip_file_header *zippath_find_sub_path(archive_file *zipfile, const char *subpath, osd::directory::entry::entry_type *type)
 *
 * @brief   Zippath find sub path.
 *
 * @param [in,out]  zipfile If non-null, the zipfile.
 * @param   subpath         The subpath.
 * @param [in,out]  type    If non-null, the type.
 *
 * @return  null if it fails, else a zip_file_header*.
 */

static int zippath_find_sub_path(archive_file &zipfile, std::string const &subpath, osd::directory::entry::entry_type &type)
{
	for (int header = zipfile.first_file(); header >= 0; header = zipfile.next_file())
	{
		std::string::size_type i = 0, j = 0;
		char c1, c2;
		do
		{
			c1 = next_path_char(zipfile.current_name(), i);
			c2 = next_path_char(subpath, j);
		}
		while ((c1 == c2) && c1 && c2);

		if (!c2 || ((c2 == '/') && !(c2 = next_path_char(subpath, j))))
		{
			if (!c1)
			{
				type = zipfile.current_is_directory() ? osd::directory::entry::entry_type::DIR : osd::directory::entry::entry_type::FILE;
				return header;
			}
			else if ((c1 == '/') || (i <= 1U))
			{
				type = osd::directory::entry::entry_type::DIR;
				return header;
			}
		}
	}

	type = osd::directory::entry::entry_type::NONE;
	return -1;
}



// -------------------------------------------------
//  zippath_resolve - separates a ZIP path out into
//  true path and ZIP entry components
// -------------------------------------------------

/**
 * @fn  static osd_file::error zippath_resolve(const char *path, osd::directory::entry::entry_type &entry_type, archive_file *&zipfile, std::string &newpath)
 *
 * @brief   Zippath resolve.
 *
 * @param   path                Full pathname of the file.
 * @param [in,out]  entry_type  Type of the entry.
 * @param [in,out]  zipfile     [in,out] If non-null, the zipfile.
 * @param [in,out]  newpath     The newpath.
 *
 * @return  A osd_file::error.
 */

static osd_file::error zippath_resolve(const char *path, osd::directory::entry::entry_type &entry_type, archive_file::ptr &zipfile, std::string &newpath)
{
	newpath.clear();

	// be conservative
	entry_type = osd::directory::entry::entry_type::NONE;
	zipfile.reset();

	std::string apath(path);
	std::string apath_trimmed;
	osd::directory::entry::entry_type current_entry_type;
	bool went_up = false;
	do
	{
		// trim the path of trailing path separators
		auto i = apath.length();
		while ((i > 1) && is_path_separator(apath[i - 1]))
			i--;
		apath.resize(i);
		apath_trimmed = apath;

		// stat the path
		auto current_entry = osd_stat(apath_trimmed);

		// did we find anything?
		if (current_entry)
		{
			// get the entry type and free the stat entry
			current_entry_type = current_entry->type;
		}
		else
		{
			// if we have not found the file or directory, go up
			current_entry_type = osd::directory::entry::entry_type::NONE;
			went_up = true;
			apath = zippath_parent(apath);
		}
	}
	while ((current_entry_type == osd::directory::entry::entry_type::NONE) && !is_root(apath.c_str()));

	// if we did not find anything, then error out
	if (current_entry_type == osd::directory::entry::entry_type::NONE)
		return osd_file::error::NOT_FOUND;

	// is this file a ZIP file?
	if ((current_entry_type == osd::directory::entry::entry_type::FILE) &&
		((is_zip_file(apath_trimmed) && (archive_file::open_zip(apath_trimmed, zipfile) == archive_file::error::NONE)) ||
			(is_7z_file(apath_trimmed) && (archive_file::open_7z(apath_trimmed, zipfile) == archive_file::error::NONE))))
	{
		auto i = strlen(path + apath.length());
		while ((i > 0) && is_zip_path_separator(path[apath.length() + i - 1]))
			i--;
		newpath.assign(path + apath.length(), i);

		// this was a true ZIP path - attempt to identify the type of path
		zippath_find_sub_path(*zipfile, newpath, current_entry_type);
		if (current_entry_type == osd::directory::entry::entry_type::NONE)
			return osd_file::error::NOT_FOUND;
	}
	else
	{
		// this was a normal path
		if (went_up)
			return osd_file::error::NOT_FOUND;

		newpath = path;
	}

	// success!
	entry_type = current_entry_type;
	return osd_file::error::NONE;
}


// -------------------------------------------------
//  zippath_opendir - opens a directory
// -------------------------------------------------

/**
 * @fn  osd_file::error zippath_opendir(const std::string &path, zippath_directory **directory)
 *
 * @brief   Zippath opendir.
 *
 * @param   path                Full pathname of the file.
 * @param [in,out]  directory   If non-null, pathname of the directory.
 *
 * @return  A osd_file::error.
 */

osd_file::error zippath_opendir(const std::string &path, zippath_directory **directory)
{
	osd_file::error err;

	/* allocate a directory */
	zippath_directory *result = nullptr;
	try
	{
		result = new zippath_directory;
	}
	catch (std::bad_alloc &)
	{
		err = osd_file::error::OUT_OF_MEMORY;
		goto done;
	}
	/* resolve the path */
	osd::directory::entry::entry_type entry_type;
	err = zippath_resolve(path.c_str(), entry_type, result->zipfile, result->zipprefix);
	if (err != osd_file::error::NONE)
		goto done;

	/* we have to be a directory */
	if (entry_type != osd::directory::entry::entry_type::DIR)
	{
		err = osd_file::error::NOT_FOUND;
		goto done;
	}

	/* was the result a ZIP? */
	if (!result->zipfile)
	{
		/* a conventional directory */
		result->directory = osd::directory::open(path);
		if (!result->directory)
		{
			err = osd_file::error::FAILURE;
			goto done;
		}

		/* is this path the root? if so, pretend we've already returned the parent */
		if (is_root(path.c_str()))
			result->returned_parent = true;
	}

done:
	if ((directory == nullptr || err != osd_file::error::NONE) && result != nullptr)
	{
		zippath_closedir(result);
		result = nullptr;
	}
	if (directory != nullptr)
		*directory = result;
	return err;
}


// -------------------------------------------------
//  zippath_closedir - closes a directory
// -------------------------------------------------

/**
 * @fn  void zippath_closedir(zippath_directory *directory)
 *
 * @brief   Zippath closedir.
 *
 * @param [in,out]  directory   If non-null, pathname of the directory.
 */

void zippath_closedir(zippath_directory *directory)
{
	if (directory->directory != nullptr)
		directory->directory.reset();

	if (directory->zipfile != nullptr)
		directory->zipfile.reset();

	directory->returned_dirlist.clear();

	delete directory;
}


// -------------------------------------------------
//  get_relative_path - checks to see if a specified
//  header is in the zippath_directory, and if so
//  returns the relative path
// -------------------------------------------------

/**
 * @fn  static const char *get_relative_path(zippath_directory *directory, const zip_file_header *header)
 *
 * @brief   Gets relative path.
 *
 * @param [in,out]  directory   If non-null, pathname of the directory.
 * @param   header              The header.
 *
 * @return  null if it fails, else the relative path.
 */

static const char *get_relative_path(zippath_directory const &directory)
{
	auto len = directory.zipprefix.length();
	char const *prefix = directory.zipprefix.c_str();
	while (is_zip_file_separator(*prefix))
	{
		len--;
		prefix++;
	}

	std::string const &current(directory.zipfile->current_name());
	char const *result = directory.zipfile->current_name().c_str() + len;
	if ((current.length() >= len) &&
		!strncmp(prefix, current.c_str(), len) &&
		(!*prefix || is_zip_file_separator(*result) || is_zip_file_separator(directory.zipprefix.back())))
	{
		while (is_zip_file_separator(*result))
			result++;

		return *result ? result : nullptr;
	}
	else
	{
		return nullptr;
	}
}


// -------------------------------------------------
//  zippath_readdir - reads a directory
// -------------------------------------------------

/**
 * @fn  const osd::directory::entry *zippath_readdir(zippath_directory *directory)
 *
 * @brief   Zippath readdir.
 *
 * @param [in,out]  directory   If non-null, pathname of the directory.
 *
 * @return  null if it fails, else an osd::directory::entry*.
 */

const osd::directory::entry *zippath_readdir(zippath_directory *directory)
{
	const osd::directory::entry *result = nullptr;

	if (!directory->returned_parent)
	{
		/* first thing's first - return parent directory */
		directory->returned_parent = true;
		directory->returned_entry.name = "..";
		directory->returned_entry.type = osd::directory::entry::entry_type::DIR;
		directory->returned_entry.size = 0; // FIXME: what would stat say?
		// FIXME: modified time?
		result = &directory->returned_entry;
	}
	else if (directory->directory)
	{
		/* a normal directory read */
		do
		{
			result = directory->directory->read();
		}
		while (result && (!strcmp(result->name, ".") || !strcmp(result->name, "..")));

		/* special case - is this entry a ZIP file?  if so we need to return it as a "directory" */
		if (result && (is_zip_file(result->name) || is_7z_file(result->name)))
		{
			/* copy; but change the entry type */
			directory->returned_entry = *result;
			directory->returned_entry.type = osd::directory::entry::entry_type::DIR;
			directory->returned_entry.size = 0; // FIXME: what would stat say?
			// FIXME: modified time?
			result = &directory->returned_entry;
		}
	}
	else if (directory->zipfile)
	{
		char const *relpath;
		do
		{
			/* a zip file read */
			int header;
			do
			{
				header = directory->called_zip_first ? directory->zipfile->next_file() : directory->zipfile->first_file();
				directory->called_zip_first = true;
				relpath = nullptr;
			}
			while ((header >= 0) && ((relpath = get_relative_path(*directory)) == nullptr));

			if (relpath)
			{
				/* we've found a ZIP entry; but this may be an entry deep within the target directory */
				char const *separator = relpath;
				while (*separator && !is_zip_file_separator(*separator)) separator++;

				if (*separator || directory->zipfile->current_is_directory())
				{
					/* a nested entry; loop through returned_dirlist to see if we've returned the parent directory */
					auto const len(separator - relpath);
					auto rdent = directory->returned_dirlist.begin();
					while (directory->returned_dirlist.end() != rdent)
					{
						if ((rdent->length() == len) && !core_strnicmp(rdent->c_str(), relpath, len))
							break;
						else
							++rdent;
					}

					if (directory->returned_dirlist.end() == rdent)
					{
						/* we've found a new directory; add this to returned_dirlist */
						directory->returned_dirlist.emplace_front(relpath, separator - relpath);

						/* ...and return it */
						directory->returned_entry.name = directory->returned_dirlist.front().c_str();
						directory->returned_entry.type = osd::directory::entry::entry_type::DIR;
						directory->returned_entry.size = 0; // FIXME: what would stat say?
						// FIXME: modified time?
						result = &directory->returned_entry;
					}
				}
				else
				{
					/* a real file */
					directory->returned_entry.name = relpath;
					directory->returned_entry.type = osd::directory::entry::entry_type::FILE;
					directory->returned_entry.size = directory->zipfile->current_uncompressed_length();
					directory->returned_entry.last_modified = directory->zipfile->current_last_modified();
					result = &directory->returned_entry;
				}
			}
		}
		while (relpath && !result);
	}
	return result;
}



// -------------------------------------------------
//  zippath_is_zip - returns true if this path is
//  a ZIP path or false if not
// -------------------------------------------------

/**
 * @fn  int zippath_is_zip(zippath_directory *directory)
 *
 * @brief   Zippath is zip.
 *
 * @param [in,out]  directory   If non-null, pathname of the directory.
 *
 * @return  An int.
 */

bool zippath_is_zip(zippath_directory *directory)
{
	return directory->zipfile != nullptr;
}

} // namespace util
