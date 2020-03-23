// license:BSD-3-Clause
// copyright-holders:Miodrag Milanovic
/***************************************************************************

    language.cpp

    Multi-language support.

***************************************************************************/

#include "emu.h"
#include "emuopts.h"

static std::unordered_map<std::string, std::string> g_translation;

const char *lang_translate(const char *word)
{
	if (g_translation.find(word) == g_translation.end())
	{
		return word;
	}
	return g_translation[word].c_str();
}

const uint32_t MO_MAGIC = 0x950412de;
const uint32_t MO_MAGIC_REVERSED = 0xde120495;

inline uint32_t endianchange(uint32_t value) {
	uint32_t b0 = (value >> 0) & 0xff;
	uint32_t b1 = (value >> 8) & 0xff;
	uint32_t b2 = (value >> 16) & 0xff;
	uint32_t b3 = (value >> 24) & 0xff;

	return (b0 << 24) | (b1 << 16) | (b2 << 8) | b3;
}

void load_translation(emu_options &m_options)
{
	g_translation.clear();
	emu_file file(m_options.language_path(), OPEN_FLAG_READ);
	auto name = std::string(m_options.language());
	strreplace(name, " ", "_");
	strreplace(name, "(", "");
	strreplace(name, ")", "");
	if (file.open(name.c_str(), PATH_SEPARATOR "strings.mo") == osd_file::error::NONE)
	{
		uint64_t size = file.size();
		uint32_t *buffer = global_alloc_array(uint32_t, size / 4 + 1);
		file.read(buffer, size);
		file.close();

		if (buffer[0] != MO_MAGIC && buffer[0] != MO_MAGIC_REVERSED)
		{
			global_free_array(buffer);
			return;
		}
		if (buffer[0] == MO_MAGIC_REVERSED)
		{
			for (auto i = 0; i < (size / 4) + 1; ++i)
			{
				buffer[i] = endianchange(buffer[i]);
			}
		}

		uint32_t number_of_strings = buffer[2];
		uint32_t original_table_offset = buffer[3] >> 2;
		uint32_t translation_table_offset = buffer[4] >> 2;

		const char *data = reinterpret_cast<const char*>(buffer);

		for (auto i = 1; i < number_of_strings; ++i)
		{
			std::string original = (const char *)data + buffer[original_table_offset + 2 * i + 1];
			std::string translation = (const char *)data + buffer[translation_table_offset + 2 * i + 1];
			g_translation.insert(std::pair<std::string, std::string>(original, translation));
		}
		global_free_array(buffer);
	}
}
