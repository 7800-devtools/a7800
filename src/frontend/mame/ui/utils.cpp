// license:BSD-3-Clause
// copyright-holders:Maurizio Petrarota
/***************************************************************************

    ui/utils.cpp

    Internal UI user interface.

***************************************************************************/

#include "emu.h"
#include "ui/utils.h"

#include "softlist.h"


extern const char UI_VERSION_TAG[];
const char UI_VERSION_TAG[] = "# UI INFO ";

// Years index
uint16_t c_year::actual = 0;
std::vector<std::string> c_year::ui;

// Manufacturers index
uint16_t c_mnfct::actual = 0;
std::vector<std::string> c_mnfct::ui;
std::unordered_map<std::string, int> c_mnfct::uimap;

// Main filters
uint16_t main_filters::actual = 0;
const char *main_filters::text[] = { "All", "Available", "Unavailable", "Working", "Not Working", "Mechanical", "Not Mechanical",
	"Category", "Favorites", "BIOS", "Parents", "Clones", "Manufacturers", "Years", "Support Save",
	"Not Support Save", "CHD", "No CHD", "Vertical", "Horizontal", "Custom" };
size_t main_filters::length = ARRAY_LENGTH(main_filters::text);

// Software filters
uint16_t sw_filters::actual = 0;
const char *sw_filters::text[] = { "All", "Available", "Unavailable", "Parents", "Clones", "Years", "Publishers", "Supported",
	"Partial Supported", "Unsupported", "Region", "Device Type", "Software List", "Custom" };
size_t sw_filters::length = ARRAY_LENGTH(sw_filters::text);

// Globals
uint8_t ui_globals::rpanel = 0;
uint8_t ui_globals::curimage_view = 0;
uint8_t ui_globals::curdats_view = 0;
uint8_t ui_globals::cur_sw_dats_total = 0;
uint8_t ui_globals::curdats_total = 0;
uint8_t ui_globals::cur_sw_dats_view = 0;
bool ui_globals::switch_image = false;
bool ui_globals::default_image = true;
bool ui_globals::reset = false;
bool ui_globals::redraw_icon = false;
int ui_globals::visible_main_lines = 0;
int ui_globals::visible_sw_lines = 0;
uint16_t ui_globals::panels_status = 0;
bool ui_globals::has_icons = false;

// Custom filter
uint16_t custfltr::main = 0;
uint16_t custfltr::numother = 0;
uint16_t custfltr::other[MAX_CUST_FILTER];
uint16_t custfltr::mnfct[MAX_CUST_FILTER];
uint16_t custfltr::year[MAX_CUST_FILTER];
uint16_t custfltr::screen[MAX_CUST_FILTER];

// Custom filter
uint16_t sw_custfltr::main = 0;
uint16_t sw_custfltr::numother = 0;
uint16_t sw_custfltr::other[MAX_CUST_FILTER];
uint16_t sw_custfltr::mnfct[MAX_CUST_FILTER];
uint16_t sw_custfltr::year[MAX_CUST_FILTER];
uint16_t sw_custfltr::region[MAX_CUST_FILTER];
uint16_t sw_custfltr::type[MAX_CUST_FILTER];
uint16_t sw_custfltr::list[MAX_CUST_FILTER];

char* chartrimcarriage(char str[])
{
	char *pstr = strrchr(str, '\n');
	if (pstr)
		str[pstr - str] = '\0';
	pstr = strrchr(str, '\r');
	if (pstr)
		str[pstr - str] = '\0';
	return str;
}

const char* strensure(const char* s)
{
	return s == nullptr ? "" : s;
}

int getprecisionchr(const char* s)
{
	int precision = 1;
	char *dp = const_cast<char *>(strchr(s, '.'));
	if (dp != nullptr)
		precision = strlen(s) - (dp - s) - 1;
	return precision;
}

std::vector<std::string> tokenize(const std::string &text, char sep)
{
	std::vector<std::string> tokens;
	tokens.reserve(64);
	std::size_t start = 0, end = 0;
	while ((end = text.find(sep, start)) != std::string::npos)
	{
		std::string temp = text.substr(start, end - start);
		if (!temp.empty()) tokens.push_back(temp);
		start = end + 1;
	}
	std::string temp = text.substr(start);
	if (!temp.empty()) tokens.push_back(temp);
	return tokens;
}

//-------------------------------------------------
//  search a substring with even partial matching
//-------------------------------------------------

int fuzzy_substring(std::string s_needle, std::string s_haystack)
{
	if (s_needle.empty())
		return s_haystack.size();
	if (s_haystack.empty())
		return s_needle.size();

	strmakelower(s_needle);
	strmakelower(s_haystack);

	if (s_needle == s_haystack)
		return 0;
	if (s_haystack.find(s_needle) != std::string::npos)
		return 0;

	auto *row1 = global_alloc_array_clear<int>(s_haystack.size() + 2);
	auto *row2 = global_alloc_array_clear<int>(s_haystack.size() + 2);

	for (int i = 0; i < s_needle.size(); ++i)
	{
		row2[0] = i + 1;
		for (int j = 0; j < s_haystack.size(); ++j)
		{
			int cost = (s_needle[i] == s_haystack[j]) ? 0 : 1;
			row2[j + 1] = std::min(row1[j + 1] + 1, std::min(row2[j] + 1, row1[j] + cost));
		}

		int *tmp = row1;
		row1 = row2;
		row2 = tmp;
	}

	int *first, *smallest;
	first = smallest = row1;
	int *last = row1 + s_haystack.size();

	while (++first != last)
		if (*first < *smallest)
			smallest = first;

	int rv = *smallest;
	global_free_array(row1);
	global_free_array(row2);

	return rv;
}

//-------------------------------------------------
//  set manufacturers
//-------------------------------------------------

void c_mnfct::set(const char *str)
{
	std::string name = getname(str);
	uimap[name] = 0;
}

std::string c_mnfct::getname(const char *str)
{
	std::string name(str);
	size_t found = name.find("(");

	if (found != std::string::npos)
		return (name.substr(0, found - 1));
	else
		return name;
}

//-------------------------------------------------
//  set years
//-------------------------------------------------

void c_year::set(const char *str)
{
	std::string name(str);
	if (std::find(ui.begin(), ui.end(), name) != ui.end())
		return;

	ui.push_back(name);
}


ui_software_info::ui_software_info(
		software_info const &info,
		software_part const &p,
		game_driver const &d,
		std::string const &li,
		std::string const &is,
		std::string const &de)
	: shortname(info.shortname()), longname(info.longname()), parentname(info.parentname())
	, year(info.year()), publisher(info.publisher())
	, supported(info.supported())
	, part(p.name())
	, driver(&d)
	, listname(li), interface(p.interface()), instance(is)
	, startempty(0)
	, parentlongname()
	, usage()
	, devicetype(de)
	, available(false)
{
	for (feature_list_item const &feature : info.other_info())
	{
		if (feature.name() == "usage")
		{
			usage = feature.value();
			break;
		}
	}
}

// info for starting empty
ui_software_info::ui_software_info(game_driver const &d)
	: shortname(d.name), longname(d.type.fullname()), driver(&d), startempty(1), available(true)
{
}
