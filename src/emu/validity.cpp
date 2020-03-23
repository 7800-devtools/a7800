// license:BSD-3-Clause
// copyright-holders:Aaron Giles, Paul Priest
/***************************************************************************

    validity.cpp

    Validity checks on internal data structures.

***************************************************************************/

#include "emu.h"
#include "validity.h"

#include "emuopts.h"
#include "video/rgbutil.h"

#include <ctype.h>
#include <type_traits>
#include <typeinfo>


//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

//**************************************************************************
//  INLINE FUNCTIONS
//**************************************************************************

//-------------------------------------------------
//  ioport_string_from_index - return an indexed
//  string from the I/O port system
//-------------------------------------------------

inline const char *validity_checker::ioport_string_from_index(u32 index)
{
	return ioport_configurer::string_from_token((const char *)(uintptr_t)index);
}


//-------------------------------------------------
//  get_defstr_index - return the index of the
//  string assuming it is one of the default
//  strings
//-------------------------------------------------

inline int validity_checker::get_defstr_index(const char *string, bool suppress_error)
{
	// check for strings that should be DEF_STR
	auto strindex = m_defstr_map.find(string);
	if (!suppress_error && strindex != m_defstr_map.end() && string != ioport_string_from_index(strindex->second))
		osd_printf_error("Must use DEF_STR( %s )\n", string);
	return (strindex != m_defstr_map.end()) ? strindex->second : 0;
}


//-------------------------------------------------
//  random_u64
//  random_s64
//  random_u32
//  random_s32
//-------------------------------------------------
#undef rand
inline s32 validity_checker::random_i32() { return s32(random_u32()); }
inline u32 validity_checker::random_u32() { return rand() ^ (rand() << 15); }
inline s64 validity_checker::random_i64() { return s64(random_u64()); }
inline u64 validity_checker::random_u64() { return u64(random_u32()) ^ (u64(random_u32()) << 30); }



//-------------------------------------------------
//  validate_tag - ensure that the given tag
//  meets the general requirements
//-------------------------------------------------

void validity_checker::validate_tag(const char *tag)
{
	// some common names that are now deprecated
	if (strcmp(tag, "main") == 0 || strcmp(tag, "audio") == 0 || strcmp(tag, "sound") == 0 || strcmp(tag, "left") == 0 || strcmp(tag, "right") == 0)
		osd_printf_error("Invalid generic tag '%s' used\n", tag);

	// scan for invalid characters
	static const char *validchars = "abcdefghijklmnopqrstuvwxyz0123456789_.:^$";
	for (const char *p = tag; *p != 0; p++)
	{
		// only lower-case permitted
		if (*p != tolower(u8(*p)))
		{
			osd_printf_error("Tag '%s' contains upper-case characters\n", tag);
			break;
		}
		if (*p == ' ')
		{
			osd_printf_error("Tag '%s' contains spaces\n", tag);
			break;
		}
		if (strchr(validchars, *p) == nullptr)
		{
			osd_printf_error("Tag '%s' contains invalid character '%c'\n",  tag, *p);
			break;
		}
	}

	// find the start of the final tag
	const char *begin = strrchr(tag, ':');
	if (begin == nullptr)
		begin = tag;
	else
		begin += 1;

	// 0-length = bad
	if (*begin == 0)
		osd_printf_error("Found 0-length tag\n");

	// too short/too long = bad
	if (strlen(begin) < MIN_TAG_LENGTH)
		osd_printf_error("Tag '%s' is too short (must be at least %d characters)\n", tag, MIN_TAG_LENGTH);
}



//**************************************************************************
//  VALIDATION FUNCTIONS
//**************************************************************************

//-------------------------------------------------
//  validity_checker - constructor
//-------------------------------------------------

validity_checker::validity_checker(emu_options &options)
	: m_drivlist(options)
	, m_errors(0)
	, m_warnings(0)
	, m_print_verbose(options.verbose())
	, m_current_driver(nullptr)
	, m_current_config(nullptr)
	, m_current_device(nullptr)
	, m_current_ioport(nullptr)
	, m_validate_all(false)
{
	// pre-populate the defstr map with all the default strings
	for (int strnum = 1; strnum < INPUT_STRING_COUNT; strnum++)
	{
		const char *string = ioport_string_from_index(strnum);
		if (string != nullptr)
			m_defstr_map.insert(std::make_pair(string, strnum));
	}
}

//-------------------------------------------------
//  validity_checker - destructor
//-------------------------------------------------

validity_checker::~validity_checker()
{
	validate_end();
}

//-------------------------------------------------
//  check_driver - check a single driver
//-------------------------------------------------

void validity_checker::check_driver(const game_driver &driver)
{
	// simply validate the one driver
	validate_begin();
	validate_one(driver);
	validate_end();
}


//-------------------------------------------------
//  check_shared_source - check all drivers that
//  share the same source file as the given driver
//-------------------------------------------------

void validity_checker::check_shared_source(const game_driver &driver)
{
	// initialize
	validate_begin();

	// then iterate over all drivers and check the ones that share the same source file
	m_drivlist.reset();
	while (m_drivlist.next())
		if (strcmp(driver.type.source(), m_drivlist.driver().type.source()) == 0)
			validate_one(m_drivlist.driver());

	// cleanup
	validate_end();
}


//-------------------------------------------------
//  check_all_matching - check all drivers whose
//  names match the given string
//-------------------------------------------------

bool validity_checker::check_all_matching(const char *string)
{
	// start by checking core stuff
	validate_begin();
	validate_core();
	validate_inlines();
	validate_rgb();

	// if we had warnings or errors, output
	if (m_errors > 0 || m_warnings > 0 || !m_verbose_text.empty())
	{
		output_via_delegate(OSD_OUTPUT_CHANNEL_ERROR, "Core: %d errors, %d warnings\n", m_errors, m_warnings);
		if (m_errors > 0)
			output_indented_errors(m_error_text, "Errors");
		if (m_warnings > 0)
			output_indented_errors(m_warning_text, "Warnings");
		if (!m_verbose_text.empty())
			output_indented_errors(m_verbose_text, "Messages");
		output_via_delegate(OSD_OUTPUT_CHANNEL_ERROR, "\n");
	}

	// then iterate over all drivers and check them
	m_drivlist.reset();
	while (m_drivlist.next())
		if (m_drivlist.matches(string, m_drivlist.driver().name))
			validate_one(m_drivlist.driver());

	// validate devices
	if (!string)
		validate_device_types();

	// cleanup
	validate_end();

	return !(m_errors > 0 || m_warnings > 0);
}


//-------------------------------------------------
//  validate_begin - prepare for validation by
//  taking over the output callbacks and resetting
//  our internal state
//-------------------------------------------------

void validity_checker::validate_begin()
{
	// take over error and warning outputs
	osd_output::push(this);

	// reset all our maps
	m_names_map.clear();
	m_descriptions_map.clear();
	m_roms_map.clear();
	m_defstr_map.clear();
	m_region_map.clear();

	// reset internal state
	m_errors = 0;
	m_warnings = 0;
	m_already_checked.clear();
}


//-------------------------------------------------
//  validate_end - restore output callbacks and
//  clean up
//-------------------------------------------------

void validity_checker::validate_end()
{
	// restore the original output callbacks
	osd_output::pop(this);
}


//-------------------------------------------------
//  validate_drivers - master validity checker
//-------------------------------------------------

void validity_checker::validate_one(const game_driver &driver)
{
	// help verbose validation detect configuration-related crashes
	if (m_print_verbose)
		output_via_delegate(OSD_OUTPUT_CHANNEL_ERROR, "Validating driver %s (%s)...\n", driver.name, core_filename_extract_base(driver.type.source()).c_str());

	// set the current driver
	m_current_driver = &driver;
	m_current_config = nullptr;
	m_current_device = nullptr;
	m_current_ioport = nullptr;
	m_region_map.clear();

	// reset error/warning state
	int start_errors = m_errors;
	int start_warnings = m_warnings;
	m_error_text.clear();
	m_warning_text.clear();
	m_verbose_text.clear();

	// wrap in try/except to catch fatalerrors
	try
	{
		machine_config config(driver, m_blank_options);
		m_current_config = &config;
		validate_driver();
		validate_roms(m_current_config->root_device());
		validate_inputs();
		validate_devices();
		m_current_config = nullptr;
	}
	catch (emu_fatalerror &err)
	{
		osd_printf_error("Fatal error %s", err.string());
	}

	// if we had warnings or errors, output
	if (m_errors > start_errors || m_warnings > start_warnings || !m_verbose_text.empty())
	{
		if (!m_print_verbose)
			output_via_delegate(OSD_OUTPUT_CHANNEL_ERROR, "Driver %s (file %s): ", driver.name, core_filename_extract_base(driver.type.source()).c_str());
		output_via_delegate(OSD_OUTPUT_CHANNEL_ERROR, "%d errors, %d warnings\n", m_errors - start_errors, m_warnings - start_warnings);
		if (m_errors > start_errors)
			output_indented_errors(m_error_text, "Errors");
		if (m_warnings > start_warnings)
			output_indented_errors(m_warning_text, "Warnings");
		if (!m_verbose_text.empty())
			output_indented_errors(m_verbose_text, "Messages");
		output_via_delegate(OSD_OUTPUT_CHANNEL_ERROR, "\n");
	}

	// reset the driver/device
	m_current_driver = nullptr;
	m_current_config = nullptr;
	m_current_device = nullptr;
	m_current_ioport = nullptr;
}


//-------------------------------------------------
//  validate_core - validate core internal systems
//-------------------------------------------------

void validity_checker::validate_core()
{
	// basic system checks
	if (~0 != -1) osd_printf_error("Machine must be two's complement\n");

	u8 a = 0xff;
	u8 b = a + 1;
	if (b > a) osd_printf_error("u8 must be 8 bits\n");

	// check size of core integer types
	if (sizeof(s8)  != 1) osd_printf_error("s8 must be 8 bits\n");
	if (sizeof(u8)  != 1) osd_printf_error("u8 must be 8 bits\n");
	if (sizeof(s16) != 2) osd_printf_error("s16 must be 16 bits\n");
	if (sizeof(u16) != 2) osd_printf_error("u16 must be 16 bits\n");
	if (sizeof(s32) != 4) osd_printf_error("s32 must be 32 bits\n");
	if (sizeof(u32) != 4) osd_printf_error("u32 must be 32 bits\n");
	if (sizeof(s64) != 8) osd_printf_error("s64 must be 64 bits\n");
	if (sizeof(u64) != 8) osd_printf_error("u64 must be 64 bits\n");

	// check signed right shift
	s8  a8 = -3;
	s16 a16 = -3;
	s32 a32 = -3;
	s64 a64 = -3;
	if (a8  >> 1 != -2) osd_printf_error("s8 right shift must be arithmetic\n");
	if (a16 >> 1 != -2) osd_printf_error("s16 right shift must be arithmetic\n");
	if (a32 >> 1 != -2) osd_printf_error("s32 right shift must be arithmetic\n");
	if (a64 >> 1 != -2) osd_printf_error("s64 right shift must be arithmetic\n");

	// check pointer size
#ifdef PTR64
	static_assert(sizeof(void *) == 8, "PTR64 flag enabled, but was compiled for 32-bit target\n");
#else
	static_assert(sizeof(void *) == 4, "PTR64 flag not enabled, but was compiled for 64-bit target\n");
#endif

	// TODO: check if this is actually working
	// check endianness definition
	u16 lsbtest = 0;
	*(u8 *)&lsbtest = 0xff;
#ifdef LSB_FIRST
	if (lsbtest == 0xff00) osd_printf_error("LSB_FIRST specified, but running on a big-endian machine\n");
#else
	if (lsbtest == 0x00ff) osd_printf_error("LSB_FIRST not specified, but running on a little-endian machine\n");
#endif
}


//-------------------------------------------------
//  validate_inlines - validate inline function
//  behaviors
//-------------------------------------------------

void validity_checker::validate_inlines()
{
	volatile u64 testu64a = random_u64();
	volatile s64 testi64a = random_i64();
	volatile u32 testu32a = random_u32();
	volatile u32 testu32b = random_u32();
	volatile s32 testi32a = random_i32();
	volatile s32 testi32b = random_i32();
	s32 resulti32, expectedi32;
	u32 resultu32, expectedu32;
	s64 resulti64, expectedi64;
	u64 resultu64, expectedu64;
	s32 remainder, expremainder;
	u32 uremainder, expuremainder, bigu32 = 0xffffffff;

	// use only non-zero, positive numbers
	if (testu64a == 0) testu64a++;
	if (testi64a == 0) testi64a++;
	else if (testi64a < 0) testi64a = -testi64a;
	if (testu32a == 0) testu32a++;
	if (testu32b == 0) testu32b++;
	if (testi32a == 0) testi32a++;
	else if (testi32a < 0) testi32a = -testi32a;
	if (testi32b == 0) testi32b++;
	else if (testi32b < 0) testi32b = -testi32b;

	resulti64 = mul_32x32(testi32a, testi32b);
	expectedi64 = s64(testi32a) * s64(testi32b);
	if (resulti64 != expectedi64)
		osd_printf_error("Error testing mul_32x32 (%08X x %08X) = %08X%08X (expected %08X%08X)\n", testi32a, testi32b, u32(resulti64 >> 32), u32(resulti64), u32(expectedi64 >> 32), u32(expectedi64));

	resultu64 = mulu_32x32(testu32a, testu32b);
	expectedu64 = u64(testu32a) * u64(testu32b);
	if (resultu64 != expectedu64)
		osd_printf_error("Error testing mulu_32x32 (%08X x %08X) = %08X%08X (expected %08X%08X)\n", testu32a, testu32b, u32(resultu64 >> 32), u32(resultu64), u32(expectedu64 >> 32), u32(expectedu64));

	resulti32 = mul_32x32_hi(testi32a, testi32b);
	expectedi32 = (s64(testi32a) * s64(testi32b)) >> 32;
	if (resulti32 != expectedi32)
		osd_printf_error("Error testing mul_32x32_hi (%08X x %08X) = %08X (expected %08X)\n", testi32a, testi32b, resulti32, expectedi32);

	resultu32 = mulu_32x32_hi(testu32a, testu32b);
	expectedu32 = (s64(testu32a) * s64(testu32b)) >> 32;
	if (resultu32 != expectedu32)
		osd_printf_error("Error testing mulu_32x32_hi (%08X x %08X) = %08X (expected %08X)\n", testu32a, testu32b, resultu32, expectedu32);

	resulti32 = mul_32x32_shift(testi32a, testi32b, 7);
	expectedi32 = (s64(testi32a) * s64(testi32b)) >> 7;
	if (resulti32 != expectedi32)
		osd_printf_error("Error testing mul_32x32_shift (%08X x %08X) >> 7 = %08X (expected %08X)\n", testi32a, testi32b, resulti32, expectedi32);

	resultu32 = mulu_32x32_shift(testu32a, testu32b, 7);
	expectedu32 = (s64(testu32a) * s64(testu32b)) >> 7;
	if (resultu32 != expectedu32)
		osd_printf_error("Error testing mulu_32x32_shift (%08X x %08X) >> 7 = %08X (expected %08X)\n", testu32a, testu32b, resultu32, expectedu32);

	while (s64(testi32a) * s64(0x7fffffff) < testi64a)
		testi64a /= 2;
	while (u64(testu32a) * u64(bigu32) < testu64a)
		testu64a /= 2;

	resulti32 = div_64x32(testi64a, testi32a);
	expectedi32 = testi64a / s64(testi32a);
	if (resulti32 != expectedi32)
		osd_printf_error("Error testing div_64x32 (%08X%08X / %08X) = %08X (expected %08X)\n", u32(testi64a >> 32), u32(testi64a), testi32a, resulti32, expectedi32);

	resultu32 = divu_64x32(testu64a, testu32a);
	expectedu32 = testu64a / u64(testu32a);
	if (resultu32 != expectedu32)
		osd_printf_error("Error testing divu_64x32 (%08X%08X / %08X) = %08X (expected %08X)\n", u32(testu64a >> 32), u32(testu64a), testu32a, resultu32, expectedu32);

	resulti32 = div_64x32_rem(testi64a, testi32a, &remainder);
	expectedi32 = testi64a / s64(testi32a);
	expremainder = testi64a % s64(testi32a);
	if (resulti32 != expectedi32 || remainder != expremainder)
		osd_printf_error("Error testing div_64x32_rem (%08X%08X / %08X) = %08X,%08X (expected %08X,%08X)\n", u32(testi64a >> 32), u32(testi64a), testi32a, resulti32, remainder, expectedi32, expremainder);

	resultu32 = divu_64x32_rem(testu64a, testu32a, &uremainder);
	expectedu32 = testu64a / u64(testu32a);
	expuremainder = testu64a % u64(testu32a);
	if (resultu32 != expectedu32 || uremainder != expuremainder)
		osd_printf_error("Error testing divu_64x32_rem (%08X%08X / %08X) = %08X,%08X (expected %08X,%08X)\n", u32(testu64a >> 32), u32(testu64a), testu32a, resultu32, uremainder, expectedu32, expuremainder);

	resulti32 = mod_64x32(testi64a, testi32a);
	expectedi32 = testi64a % s64(testi32a);
	if (resulti32 != expectedi32)
		osd_printf_error("Error testing mod_64x32 (%08X%08X / %08X) = %08X (expected %08X)\n", u32(testi64a >> 32), u32(testi64a), testi32a, resulti32, expectedi32);

	resultu32 = modu_64x32(testu64a, testu32a);
	expectedu32 = testu64a % u64(testu32a);
	if (resultu32 != expectedu32)
		osd_printf_error("Error testing modu_64x32 (%08X%08X / %08X) = %08X (expected %08X)\n", u32(testu64a >> 32), u32(testu64a), testu32a, resultu32, expectedu32);

	while (s64(testi32a) * s64(0x7fffffff) < (s32(testi64a) << 3))
		testi64a /= 2;
	while (u64(testu32a) * u64(0xffffffff) < (u32(testu64a) << 3))
		testu64a /= 2;

	resulti32 = div_32x32_shift(s32(testi64a), testi32a, 3);
	expectedi32 = (s64(s32(testi64a)) << 3) / s64(testi32a);
	if (resulti32 != expectedi32)
		osd_printf_error("Error testing div_32x32_shift (%08X << 3) / %08X = %08X (expected %08X)\n", s32(testi64a), testi32a, resulti32, expectedi32);

	resultu32 = divu_32x32_shift(u32(testu64a), testu32a, 3);
	expectedu32 = (u64(u32(testu64a)) << 3) / u64(testu32a);
	if (resultu32 != expectedu32)
		osd_printf_error("Error testing divu_32x32_shift (%08X << 3) / %08X = %08X (expected %08X)\n", u32(testu64a), testu32a, resultu32, expectedu32);

	if (fabsf(recip_approx(100.0f) - 0.01f) > 0.0001f)
		osd_printf_error("Error testing recip_approx\n");

	testi32a = (testi32a & 0x0000ffff) | 0x400000;
	if (count_leading_zeros(testi32a) != 9)
		osd_printf_error("Error testing count_leading_zeros\n");
	testi32a = (testi32a | 0xffff0000) & ~0x400000;
	if (count_leading_ones(testi32a) != 9)
		osd_printf_error("Error testing count_leading_ones\n");
}


//-------------------------------------------------
//  validate_rgb - validate optimised RGB utility
//  class
//-------------------------------------------------

void validity_checker::validate_rgb()
{
	/*
	    This performs cursory tests of most of the vector-optimised RGB
	    utilities, concentrating on the low-level maths.  It uses random
	    values most of the time for a quick go/no-go indication rather
	    than trying to exercise edge cases.  It doesn't matter too much
	    if the compiler optimises out some of the operations since it's
	    really intended to check for logic bugs in the vector code.  If
	    the compiler can work out that the code produces the expected
	    result, that's good enough.

	    The tests for bitwise logical operations are ordered to minimise
	    the chance of all-zero or all-one patterns producing a
	    misleading good result.

	    The following functions are not tested yet:
	    rgbaint_t()
	    clamp_and_clear(const u32)
	    sign_extend(const u32, const u32)
	    min(const s32)
	    max(const s32)
	    blend(const rgbaint_t&, u8)
	    scale_and_clamp(const rgbaint_t&)
	    scale_imm_and_clamp(const s32)
	    scale2_add_and_clamp(const rgbaint_t&, const rgbaint_t&, const rgbaint_t&)
	    scale_add_and_clamp(const rgbaint_t&, const rgbaint_t&);
	    scale_imm_add_and_clamp(const s32, const rgbaint_t&);
	    static bilinear_filter(u32, u32, u32, u32, u8, u8)
	    bilinear_filter_rgbaint(u32, u32, u32, u32, u8, u8)
	*/

	auto random_i32_nolimit = [this]
	{
		s32 result;
		do { result = random_i32(); } while ((result == std::numeric_limits<s32>::min()) || (result == std::numeric_limits<s32>::max()));
		return result;
	};

	volatile s32 expected_a, expected_r, expected_g, expected_b;
	volatile s32 actual_a, actual_r, actual_g, actual_b;
	volatile s32 imm;
	rgbaint_t rgb, other;
	rgb_t packed;
	auto check_expected = [&] (const char *desc)
	{
		const volatile s32 a = rgb.get_a32();
		const volatile s32 r = rgb.get_r32();
		const volatile s32 g = rgb.get_g32();
		const volatile s32 b = rgb.get_b32();
		if (a != expected_a) osd_printf_error("Error testing %s get_a32() = %d (expected %d)\n", desc, a, expected_a);
		if (r != expected_r) osd_printf_error("Error testing %s get_r32() = %d (expected %d)\n", desc, r, expected_r);
		if (g != expected_g) osd_printf_error("Error testing %s get_g32() = %d (expected %d)\n", desc, g, expected_g);
		if (b != expected_b) osd_printf_error("Error testing %s get_b32() = %d (expected %d)\n", desc, b, expected_b);
	};

	// check set/get
	expected_a = random_i32();
	expected_r = random_i32();
	expected_g = random_i32();
	expected_b = random_i32();
	rgb.set(expected_a, expected_r, expected_g, expected_b);
	check_expected("rgbaint_t::set(a, r, g, b)");

	// check construct/set
	expected_a = random_i32();
	expected_r = random_i32();
	expected_g = random_i32();
	expected_b = random_i32();
	rgb.set(rgbaint_t(expected_a, expected_r, expected_g, expected_b));
	check_expected("rgbaint_t::set(rgbaint_t)");

	// check construct/assign
	expected_a = random_i32();
	expected_r = random_i32();
	expected_g = random_i32();
	expected_b = random_i32();
	rgb = rgbaint_t(expected_a, expected_r, expected_g, expected_b);
	check_expected("rgbaint_t assignment");

	// check piecewise set
	rgb.set_a(expected_a = random_i32());
	check_expected("rgbaint_t::set_a");
	rgb.set_r(expected_r = random_i32());
	check_expected("rgbaint_t::set_r");
	rgb.set_g(expected_g = random_i32());
	check_expected("rgbaint_t::set_g");
	rgb.set_b(expected_b = random_i32());
	check_expected("rgbaint_t::set_b");

	// test merge_alpha
	expected_a = rand();
	rgb.merge_alpha(rgbaint_t(expected_a, rand(), rand(), rand()));
	check_expected("rgbaint_t::merge_alpha");

	// test RGB addition (method)
	expected_a += actual_a = random_i32();
	expected_r += actual_r = random_i32();
	expected_g += actual_g = random_i32();
	expected_b += actual_b = random_i32();
	rgb.add(rgbaint_t(actual_a, actual_r, actual_g, actual_b));
	check_expected("rgbaint_t::add");

	// test RGB addition (operator)
	expected_a += actual_a = random_i32();
	expected_r += actual_r = random_i32();
	expected_g += actual_g = random_i32();
	expected_b += actual_b = random_i32();
	rgb += rgbaint_t(actual_a, actual_r, actual_g, actual_b);
	check_expected("rgbaint_t::operator+=");

	// test offset addition (method)
	imm = random_i32();
	expected_a += imm;
	expected_r += imm;
	expected_g += imm;
	expected_b += imm;
	rgb.add_imm(imm);
	check_expected("rgbaint_t::add_imm");

	// test offset addition (operator)
	imm = random_i32();
	expected_a += imm;
	expected_r += imm;
	expected_g += imm;
	expected_b += imm;
	rgb += imm;
	check_expected("rgbaint_t::operator+=");

	// test immediate RGB addition
	expected_a += actual_a = random_i32();
	expected_r += actual_r = random_i32();
	expected_g += actual_g = random_i32();
	expected_b += actual_b = random_i32();
	rgb.add_imm_rgba(actual_a, actual_r, actual_g, actual_b);
	check_expected("rgbaint_t::add_imm_rgba");

	// test RGB subtraction (method)
	expected_a -= actual_a = random_i32();
	expected_r -= actual_r = random_i32();
	expected_g -= actual_g = random_i32();
	expected_b -= actual_b = random_i32();
	rgb.sub(rgbaint_t(actual_a, actual_r, actual_g, actual_b));
	check_expected("rgbaint_t::sub");

	// test RGB subtraction (operator)
	expected_a -= actual_a = random_i32();
	expected_r -= actual_r = random_i32();
	expected_g -= actual_g = random_i32();
	expected_b -= actual_b = random_i32();
	rgb -= rgbaint_t(actual_a, actual_r, actual_g, actual_b);
	check_expected("rgbaint_t::operator-=");

	// test offset subtraction
	imm = random_i32();
	expected_a -= imm;
	expected_r -= imm;
	expected_g -= imm;
	expected_b -= imm;
	rgb.sub_imm(imm);
	check_expected("rgbaint_t::sub_imm");

	// test immediate RGB subtraction
	expected_a -= actual_a = random_i32();
	expected_r -= actual_r = random_i32();
	expected_g -= actual_g = random_i32();
	expected_b -= actual_b = random_i32();
	rgb.sub_imm_rgba(actual_a, actual_r, actual_g, actual_b);
	check_expected("rgbaint_t::sub_imm_rgba");

	// test reversed RGB subtraction
	expected_a = (actual_a = random_i32()) - expected_a;
	expected_r = (actual_r = random_i32()) - expected_r;
	expected_g = (actual_g = random_i32()) - expected_g;
	expected_b = (actual_b = random_i32()) - expected_b;
	rgb.subr(rgbaint_t(actual_a, actual_r, actual_g, actual_b));
	check_expected("rgbaint_t::subr");

	// test reversed offset subtraction
	imm = random_i32();
	expected_a = imm - expected_a;
	expected_r = imm - expected_r;
	expected_g = imm - expected_g;
	expected_b = imm - expected_b;
	rgb.subr_imm(imm);
	check_expected("rgbaint_t::subr_imm");

	// test reversed immediate RGB subtraction
	expected_a = (actual_a = random_i32()) - expected_a;
	expected_r = (actual_r = random_i32()) - expected_r;
	expected_g = (actual_g = random_i32()) - expected_g;
	expected_b = (actual_b = random_i32()) - expected_b;
	rgb.subr_imm_rgba(actual_a, actual_r, actual_g, actual_b);
	check_expected("rgbaint_t::subr_imm_rgba");

	// test RGB multiplication (method)
	expected_a *= actual_a = random_i32();
	expected_r *= actual_r = random_i32();
	expected_g *= actual_g = random_i32();
	expected_b *= actual_b = random_i32();
	rgb.mul(rgbaint_t(actual_a, actual_r, actual_g, actual_b));
	check_expected("rgbaint_t::mul");

	// test RGB multiplication (operator)
	expected_a *= actual_a = random_i32();
	expected_r *= actual_r = random_i32();
	expected_g *= actual_g = random_i32();
	expected_b *= actual_b = random_i32();
	rgb *= rgbaint_t(actual_a, actual_r, actual_g, actual_b);
	check_expected("rgbaint_t::operator*=");

	// test factor multiplication (method)
	imm = random_i32();
	expected_a *= imm;
	expected_r *= imm;
	expected_g *= imm;
	expected_b *= imm;
	rgb.mul_imm(imm);
	check_expected("rgbaint_t::mul_imm");

	// test factor multiplication (operator)
	imm = random_i32();
	expected_a *= imm;
	expected_r *= imm;
	expected_g *= imm;
	expected_b *= imm;
	rgb *= imm;
	check_expected("rgbaint_t::operator*=");

	// test immediate RGB multiplication
	expected_a *= actual_a = random_i32();
	expected_r *= actual_r = random_i32();
	expected_g *= actual_g = random_i32();
	expected_b *= actual_b = random_i32();
	rgb.mul_imm_rgba(actual_a, actual_r, actual_g, actual_b);
	check_expected("rgbaint_t::mul_imm_rgba");

	// test RGB and not
	expected_a &= ~(actual_a = random_i32());
	expected_r &= ~(actual_r = random_i32());
	expected_g &= ~(actual_g = random_i32());
	expected_b &= ~(actual_b = random_i32());
	rgb.andnot_reg(rgbaint_t(actual_a, actual_r, actual_g, actual_b));
	check_expected("rgbaint_t::andnot_reg");

	// test RGB or
	expected_a |= actual_a = random_i32();
	expected_r |= actual_r = random_i32();
	expected_g |= actual_g = random_i32();
	expected_b |= actual_b = random_i32();
	rgb.or_reg(rgbaint_t(actual_a, actual_r, actual_g, actual_b));
	check_expected("rgbaint_t::or_reg");

	// test RGB and
	expected_a &= actual_a = random_i32();
	expected_r &= actual_r = random_i32();
	expected_g &= actual_g = random_i32();
	expected_b &= actual_b = random_i32();
	rgb.and_reg(rgbaint_t(actual_a, actual_r, actual_g, actual_b));
	check_expected("rgbaint_t::and_reg");

	// test RGB xor
	expected_a ^= actual_a = random_i32();
	expected_r ^= actual_r = random_i32();
	expected_g ^= actual_g = random_i32();
	expected_b ^= actual_b = random_i32();
	rgb.xor_reg(rgbaint_t(actual_a, actual_r, actual_g, actual_b));
	check_expected("rgbaint_t::xor_reg");

	// test uniform or
	imm = random_i32();
	expected_a |= imm;
	expected_r |= imm;
	expected_g |= imm;
	expected_b |= imm;
	rgb.or_imm(imm);
	check_expected("rgbaint_t::or_imm");

	// test uniform and
	imm = random_i32();
	expected_a &= imm;
	expected_r &= imm;
	expected_g &= imm;
	expected_b &= imm;
	rgb.and_imm(imm);
	check_expected("rgbaint_t::and_imm");

	// test uniform xor
	imm = random_i32();
	expected_a ^= imm;
	expected_r ^= imm;
	expected_g ^= imm;
	expected_b ^= imm;
	rgb.xor_imm(imm);
	check_expected("rgbaint_t::xor_imm");

	// test immediate RGB or
	expected_a |= actual_a = random_i32();
	expected_r |= actual_r = random_i32();
	expected_g |= actual_g = random_i32();
	expected_b |= actual_b = random_i32();
	rgb.or_imm_rgba(actual_a, actual_r, actual_g, actual_b);
	check_expected("rgbaint_t::or_imm_rgba");

	// test immediate RGB and
	expected_a &= actual_a = random_i32();
	expected_r &= actual_r = random_i32();
	expected_g &= actual_g = random_i32();
	expected_b &= actual_b = random_i32();
	rgb.and_imm_rgba(actual_a, actual_r, actual_g, actual_b);
	check_expected("rgbaint_t::and_imm_rgba");

	// test immediate RGB xor
	expected_a ^= actual_a = random_i32();
	expected_r ^= actual_r = random_i32();
	expected_g ^= actual_g = random_i32();
	expected_b ^= actual_b = random_i32();
	rgb.xor_imm_rgba(actual_a, actual_r, actual_g, actual_b);
	check_expected("rgbaint_t::xor_imm_rgba");

	// test 8-bit get
	expected_a = s32(u32(expected_a) & 0x00ff);
	expected_r = s32(u32(expected_r) & 0x00ff);
	expected_g = s32(u32(expected_g) & 0x00ff);
	expected_b = s32(u32(expected_b) & 0x00ff);
	actual_a = s32(u32(rgb.get_a()));
	actual_r = s32(u32(rgb.get_r()));
	actual_g = s32(u32(rgb.get_g()));
	actual_b = s32(u32(rgb.get_b()));
	if (actual_a != expected_a) osd_printf_error("Error testing rgbaint_t::get_a() = %d (expected %d)\n", actual_a, expected_a);
	if (actual_r != expected_r) osd_printf_error("Error testing rgbaint_t::get_r() = %d (expected %d)\n", actual_r, expected_r);
	if (actual_g != expected_g) osd_printf_error("Error testing rgbaint_t::get_g() = %d (expected %d)\n", actual_g, expected_g);
	if (actual_b != expected_b) osd_printf_error("Error testing rgbaint_t::get_b() = %d (expected %d)\n", actual_b, expected_b);

	// test set from packed RGBA
	imm = random_i32();
	expected_a = s32((u32(imm) >> 24) & 0x00ff);
	expected_r = s32((u32(imm) >> 16) & 0x00ff);
	expected_g = s32((u32(imm) >> 8) & 0x00ff);
	expected_b = s32((u32(imm) >> 0) & 0x00ff);
	rgb.set(u32(imm));
	check_expected("rgbaint_t::set(u32)");

	// while we have a value loaded that we know doesn't exceed 8-bit range, check the non-clamping convert-to-rgba
	packed = rgb.to_rgba();
	if (u32(imm) != u32(packed))
		osd_printf_error("Error testing rgbaint_t::to_rgba() = %08x (expected %08x)\n", u32(packed), u32(imm));

	// test construct from packed RGBA and assign
	imm = random_i32();
	expected_a = s32((u32(imm) >> 24) & 0x00ff);
	expected_r = s32((u32(imm) >> 16) & 0x00ff);
	expected_g = s32((u32(imm) >> 8) & 0x00ff);
	expected_b = s32((u32(imm) >> 0) & 0x00ff);
	rgb = rgbaint_t(u32(imm));
	check_expected("rgbaint_t(u32)");

	// while we have a value loaded that we know doesn't exceed 8-bit range, check the non-clamping convert-to-rgba
	packed = rgb.to_rgba();
	if (u32(imm) != u32(packed))
		osd_printf_error("Error testing rgbaint_t::to_rgba() = %08x (expected %08x)\n", u32(packed), u32(imm));

	// test set with rgb_t
	packed = random_u32();
	expected_a = s32(u32(packed.a()));
	expected_r = s32(u32(packed.r()));
	expected_g = s32(u32(packed.g()));
	expected_b = s32(u32(packed.b()));
	rgb.set(packed);
	check_expected("rgbaint_t::set(rgba_t)");

	// test construct with rgb_t
	packed = random_u32();
	expected_a = s32(u32(packed.a()));
	expected_r = s32(u32(packed.r()));
	expected_g = s32(u32(packed.g()));
	expected_b = s32(u32(packed.b()));
	rgb = rgbaint_t(packed);
	check_expected("rgbaint_t::set(rgba_t)");

	// test clamping convert-to-rgba with hand-crafted values to catch edge cases
	rgb.set(std::numeric_limits<s32>::min(), -1, 0, 1);
	packed = rgb.to_rgba_clamp();
	if (u32(0x00000001) != u32(packed))
		osd_printf_error("Error testing rgbaint_t::to_rgba_clamp() = %08x (expected 0x00000001)\n", u32(packed));
	rgb.set(254, 255, 256, std::numeric_limits<s32>::max());
	packed = rgb.to_rgba_clamp();
	if (u32(0xfeffffff) != u32(packed))
		osd_printf_error("Error testing rgbaint_t::to_rgba_clamp() = %08x (expected 0xfeffffff)\n", u32(packed));
	rgb.set(std::numeric_limits<s32>::max(), std::numeric_limits<s32>::min(), 256, -1);
	packed = rgb.to_rgba_clamp();
	if (u32(0xff00ff00) != u32(packed))
		osd_printf_error("Error testing rgbaint_t::to_rgba_clamp() = %08x (expected 0xff00ff00)\n", u32(packed));
	rgb.set(0, 255, 1, 254);
	packed = rgb.to_rgba_clamp();
	if (u32(0x00ff01fe) != u32(packed))
		osd_printf_error("Error testing rgbaint_t::to_rgba_clamp() = %08x (expected 0x00ff01fe)\n", u32(packed));

	// test in-place clamping with hand-crafted values to catch edge cases
	expected_a = 0;
	expected_r = 0;
	expected_g = 0;
	expected_b = 1;
	rgb.set(std::numeric_limits<s32>::min(), -1, 0, 1);
	rgb.clamp_to_uint8();
	check_expected("rgbaint_t::clamp_to_uint8");
	expected_a = 254;
	expected_r = 255;
	expected_g = 255;
	expected_b = 255;
	rgb.set(254, 255, 256, std::numeric_limits<s32>::max());
	rgb.clamp_to_uint8();
	check_expected("rgbaint_t::clamp_to_uint8");
	expected_a = 255;
	expected_r = 0;
	expected_g = 255;
	expected_b = 0;
	rgb.set(std::numeric_limits<s32>::max(), std::numeric_limits<s32>::min(), 256, -1);
	rgb.clamp_to_uint8();
	check_expected("rgbaint_t::clamp_to_uint8");
	expected_a = 0;
	expected_r = 255;
	expected_g = 1;
	expected_b = 254;
	rgb.set(0, 255, 1, 254);
	rgb.clamp_to_uint8();
	check_expected("rgbaint_t::clamp_to_uint8");

	// test shift left
	expected_a = (actual_a = random_i32()) << 19;
	expected_r = (actual_r = random_i32()) << 3;
	expected_g = (actual_g = random_i32()) << 21;
	expected_b = (actual_b = random_i32()) << 6;
	rgb.set(actual_a, actual_r, actual_g, actual_b);
	rgb.shl(rgbaint_t(19, 3, 21, 6));
	check_expected("rgbaint_t::shl");

	// test shift left immediate
	expected_a = (actual_a = random_i32()) << 7;
	expected_r = (actual_r = random_i32()) << 7;
	expected_g = (actual_g = random_i32()) << 7;
	expected_b = (actual_b = random_i32()) << 7;
	rgb.set(actual_a, actual_r, actual_g, actual_b);
	rgb.shl_imm(7);
	check_expected("rgbaint_t::shl_imm");

	// test logical shift right
	expected_a = s32(u32(actual_a = random_i32()) >> 8);
	expected_r = s32(u32(actual_r = random_i32()) >> 18);
	expected_g = s32(u32(actual_g = random_i32()) >> 26);
	expected_b = s32(u32(actual_b = random_i32()) >> 4);
	rgb.set(actual_a, actual_r, actual_g, actual_b);
	rgb.shr(rgbaint_t(8, 18, 26, 4));
	check_expected("rgbaint_t::shr");

	// test logical shift right with opposite signs
	expected_a = s32(u32(actual_a = -actual_a) >> 21);
	expected_r = s32(u32(actual_r = -actual_r) >> 13);
	expected_g = s32(u32(actual_g = -actual_g) >> 11);
	expected_b = s32(u32(actual_b = -actual_b) >> 17);
	rgb.set(actual_a, actual_r, actual_g, actual_b);
	rgb.shr(rgbaint_t(21, 13, 11, 17));
	check_expected("rgbaint_t::shr");

	// test logical shift right immediate
	expected_a = s32(u32(actual_a = random_i32()) >> 5);
	expected_r = s32(u32(actual_r = random_i32()) >> 5);
	expected_g = s32(u32(actual_g = random_i32()) >> 5);
	expected_b = s32(u32(actual_b = random_i32()) >> 5);
	rgb.set(actual_a, actual_r, actual_g, actual_b);
	rgb.shr_imm(5);
	check_expected("rgbaint_t::shr_imm");

	// test logical shift right immediate with opposite signs
	expected_a = s32(u32(actual_a = -actual_a) >> 15);
	expected_r = s32(u32(actual_r = -actual_r) >> 15);
	expected_g = s32(u32(actual_g = -actual_g) >> 15);
	expected_b = s32(u32(actual_b = -actual_b) >> 15);
	rgb.set(actual_a, actual_r, actual_g, actual_b);
	rgb.shr_imm(15);
	check_expected("rgbaint_t::shr_imm");

	// test arithmetic shift right
	expected_a = (actual_a = random_i32()) >> 16;
	expected_r = (actual_r = random_i32()) >> 20;
	expected_g = (actual_g = random_i32()) >> 14;
	expected_b = (actual_b = random_i32()) >> 2;
	rgb.set(actual_a, actual_r, actual_g, actual_b);
	rgb.sra(rgbaint_t(16, 20, 14, 2));
	check_expected("rgbaint_t::sra");

	// test arithmetic shift right with opposite signs
	expected_a = (actual_a = -actual_a) >> 1;
	expected_r = (actual_r = -actual_r) >> 29;
	expected_g = (actual_g = -actual_g) >> 10;
	expected_b = (actual_b = -actual_b) >> 22;
	rgb.set(actual_a, actual_r, actual_g, actual_b);
	rgb.sra(rgbaint_t(1, 29, 10, 22));
	check_expected("rgbaint_t::sra");

	// test arithmetic shift right immediate (method)
	expected_a = (actual_a = random_i32()) >> 12;
	expected_r = (actual_r = random_i32()) >> 12;
	expected_g = (actual_g = random_i32()) >> 12;
	expected_b = (actual_b = random_i32()) >> 12;
	rgb.set(actual_a, actual_r, actual_g, actual_b);
	rgb.sra_imm(12);
	check_expected("rgbaint_t::sra_imm");

	// test arithmetic shift right immediate with opposite signs (method)
	expected_a = (actual_a = -actual_a) >> 9;
	expected_r = (actual_r = -actual_r) >> 9;
	expected_g = (actual_g = -actual_g) >> 9;
	expected_b = (actual_b = -actual_b) >> 9;
	rgb.set(actual_a, actual_r, actual_g, actual_b);
	rgb.sra_imm(9);
	check_expected("rgbaint_t::sra_imm");

	// test arithmetic shift right immediate (operator)
	expected_a = (actual_a = random_i32()) >> 7;
	expected_r = (actual_r = random_i32()) >> 7;
	expected_g = (actual_g = random_i32()) >> 7;
	expected_b = (actual_b = random_i32()) >> 7;
	rgb.set(actual_a, actual_r, actual_g, actual_b);
	rgb >>= 7;
	check_expected("rgbaint_t::operator>>=");

	// test arithmetic shift right immediate with opposite signs (operator)
	expected_a = (actual_a = -actual_a) >> 11;
	expected_r = (actual_r = -actual_r) >> 11;
	expected_g = (actual_g = -actual_g) >> 11;
	expected_b = (actual_b = -actual_b) >> 11;
	rgb.set(actual_a, actual_r, actual_g, actual_b);
	rgb >>= 11;
	check_expected("rgbaint_t::operator>>=");

	// test RGB equality comparison
	actual_a = random_i32_nolimit();
	actual_r = random_i32_nolimit();
	actual_g = random_i32_nolimit();
	actual_b = random_i32_nolimit();
	expected_a = ~s32(0);
	expected_r = 0;
	expected_g = 0;
	expected_b = 0;
	rgb.set(actual_a, actual_r, actual_g, actual_b);
	rgb.cmpeq(rgbaint_t(actual_a, actual_r - 1, actual_g + 1, std::numeric_limits<s32>::min()));
	check_expected("rgbaint_t::cmpeq");
	expected_a = 0;
	expected_r = ~s32(0);
	expected_g = 0;
	expected_b = 0;
	rgb.set(actual_a, actual_r, actual_g, actual_b);
	rgb.cmpeq(rgbaint_t(std::numeric_limits<s32>::max(), actual_r, actual_g - 1, actual_b + 1));
	check_expected("rgbaint_t::cmpeq");

	// test immediate equality comparison
	actual_a = random_i32_nolimit();
	actual_r = random_i32_nolimit();
	actual_g = random_i32_nolimit();
	actual_b = random_i32_nolimit();
	expected_a = ~s32(0);
	expected_r = (actual_r == actual_a) ? ~s32(0) : 0;
	expected_g = (actual_g == actual_a) ? ~s32(0) : 0;
	expected_b = (actual_b == actual_a) ? ~s32(0) : 0;
	rgb.set(actual_a, actual_r, actual_g, actual_b);
	rgb.cmpeq_imm(actual_a);
	check_expected("rgbaint_t::cmpeq_imm");
	expected_a = (actual_a == actual_r) ? ~s32(0) : 0;
	expected_r = ~s32(0);
	expected_g = (actual_g == actual_r) ? ~s32(0) : 0;
	expected_b = (actual_b == actual_r) ? ~s32(0) : 0;
	rgb.set(actual_a, actual_r, actual_g, actual_b);
	rgb.cmpeq_imm(actual_r);
	check_expected("rgbaint_t::cmpeq_imm");
	expected_a = (actual_a == actual_g) ? ~s32(0) : 0;
	expected_r = (actual_r == actual_g) ? ~s32(0) : 0;
	expected_g = ~s32(0);
	expected_b = (actual_b == actual_g) ? ~s32(0) : 0;
	rgb.set(actual_a, actual_r, actual_g, actual_b);
	rgb.cmpeq_imm(actual_g);
	check_expected("rgbaint_t::cmpeq_imm");
	expected_a = (actual_a == actual_b) ? ~s32(0) : 0;
	expected_r = (actual_r == actual_b) ? ~s32(0) : 0;
	expected_g = (actual_g == actual_b) ? ~s32(0) : 0;
	expected_b = ~s32(0);
	rgb.set(actual_a, actual_r, actual_g, actual_b);
	rgb.cmpeq_imm(actual_b);
	check_expected("rgbaint_t::cmpeq_imm");
	expected_a = 0;
	expected_r = 0;
	expected_g = 0;
	expected_b = 0;
	rgb.set(actual_a, actual_r, actual_g, actual_b);
	rgb.cmpeq_imm(std::numeric_limits<s32>::min());
	check_expected("rgbaint_t::cmpeq_imm");
	expected_a = !actual_a ? ~s32(0) : 0;
	expected_r = !actual_r ? ~s32(0) : 0;
	expected_g = !actual_g ? ~s32(0) : 0;
	expected_b = !actual_b ? ~s32(0) : 0;
	rgb.set(actual_a, actual_r, actual_g, actual_b);
	rgb.cmpeq_imm(0);
	check_expected("rgbaint_t::cmpeq_imm");
	expected_a = 0;
	expected_r = 0;
	expected_g = 0;
	expected_b = 0;
	rgb.set(actual_a, actual_r, actual_g, actual_b);
	rgb.cmpeq_imm(std::numeric_limits<s32>::max());
	check_expected("rgbaint_t::cmpeq_imm");

	// test immediate RGB equality comparison
	actual_a = random_i32_nolimit();
	actual_r = random_i32_nolimit();
	actual_g = random_i32_nolimit();
	actual_b = random_i32_nolimit();
	expected_a = 0;
	expected_r = 0;
	expected_g = ~s32(0);
	expected_b = 0;
	rgb.set(actual_a, actual_r, actual_g, actual_b);
	rgb.cmpeq_imm_rgba(std::numeric_limits<s32>::min(), std::numeric_limits<s32>::max(), actual_g, actual_b - 1);
	check_expected("rgbaint_t::cmpeq_imm_rgba");
	expected_a = 0;
	expected_r = 0;
	expected_g = 0;
	expected_b = ~s32(0);
	rgb.set(actual_a, actual_r, actual_g, actual_b);
	rgb.cmpeq_imm_rgba(actual_a + 1, std::numeric_limits<s32>::min(), std::numeric_limits<s32>::max(), actual_b);
	check_expected("rgbaint_t::cmpeq_imm_rgba");

	// test RGB greater than comparison
	actual_a = random_i32_nolimit();
	actual_r = random_i32_nolimit();
	actual_g = random_i32_nolimit();
	actual_b = random_i32_nolimit();
	expected_a = 0;
	expected_r = ~s32(0);
	expected_g = 0;
	expected_b = ~s32(0);
	rgb.set(actual_a, actual_r, actual_g, actual_b);
	rgb.cmpgt(rgbaint_t(actual_a, actual_r - 1, actual_g + 1, std::numeric_limits<s32>::min()));
	check_expected("rgbaint_t::cmpgt");
	expected_a = 0;
	expected_r = 0;
	expected_g = ~s32(0);
	expected_b = 0;
	rgb.set(actual_a, actual_r, actual_g, actual_b);
	rgb.cmpgt(rgbaint_t(std::numeric_limits<s32>::max(), actual_r, actual_g - 1, actual_b + 1));
	check_expected("rgbaint_t::cmpgt");

	// test immediate greater than comparison
	actual_a = random_i32_nolimit();
	actual_r = random_i32_nolimit();
	actual_g = random_i32_nolimit();
	actual_b = random_i32_nolimit();
	expected_a = 0;
	expected_r = (actual_r > actual_a) ? ~s32(0) : 0;
	expected_g = (actual_g > actual_a) ? ~s32(0) : 0;
	expected_b = (actual_b > actual_a) ? ~s32(0) : 0;
	rgb.set(actual_a, actual_r, actual_g, actual_b);
	rgb.cmpgt_imm(actual_a);
	check_expected("rgbaint_t::cmpgt_imm");
	expected_a = (actual_a > actual_r) ? ~s32(0) : 0;
	expected_r = 0;
	expected_g = (actual_g > actual_r) ? ~s32(0) : 0;
	expected_b = (actual_b > actual_r) ? ~s32(0) : 0;
	rgb.set(actual_a, actual_r, actual_g, actual_b);
	rgb.cmpgt_imm(actual_r);
	check_expected("rgbaint_t::cmpgt_imm");
	expected_a = (actual_a > actual_g) ? ~s32(0) : 0;
	expected_r = (actual_r > actual_g) ? ~s32(0) : 0;
	expected_g =0;
	expected_b = (actual_b > actual_g) ? ~s32(0) : 0;
	rgb.set(actual_a, actual_r, actual_g, actual_b);
	rgb.cmpgt_imm(actual_g);
	check_expected("rgbaint_t::cmpgt_imm");
	expected_a = (actual_a > actual_b) ? ~s32(0) : 0;
	expected_r = (actual_r > actual_b) ? ~s32(0) : 0;
	expected_g = (actual_g > actual_b) ? ~s32(0) : 0;
	expected_b = 0;
	rgb.set(actual_a, actual_r, actual_g, actual_b);
	rgb.cmpgt_imm(actual_b);
	check_expected("rgbaint_t::cmpgt_imm");
	expected_a = ~s32(0);
	expected_r = ~s32(0);
	expected_g = ~s32(0);
	expected_b = ~s32(0);
	rgb.set(actual_a, actual_r, actual_g, actual_b);
	rgb.cmpgt_imm(std::numeric_limits<s32>::min());
	check_expected("rgbaint_t::cmpgt_imm");
	expected_a = (actual_a > 0) ? ~s32(0) : 0;
	expected_r = (actual_r > 0) ? ~s32(0) : 0;
	expected_g = (actual_g > 0) ? ~s32(0) : 0;
	expected_b = (actual_b > 0) ? ~s32(0) : 0;
	rgb.set(actual_a, actual_r, actual_g, actual_b);
	rgb.cmpgt_imm(0);
	check_expected("rgbaint_t::cmpgt_imm");
	expected_a = 0;
	expected_r = 0;
	expected_g = 0;
	expected_b = 0;
	rgb.set(actual_a, actual_r, actual_g, actual_b);
	rgb.cmpgt_imm(std::numeric_limits<s32>::max());
	check_expected("rgbaint_t::cmpgt_imm");

	// test immediate RGB greater than comparison
	actual_a = random_i32_nolimit();
	actual_r = random_i32_nolimit();
	actual_g = random_i32_nolimit();
	actual_b = random_i32_nolimit();
	expected_a = ~s32(0);
	expected_r = 0;
	expected_g = 0;
	expected_b = ~s32(0);
	rgb.set(actual_a, actual_r, actual_g, actual_b);
	rgb.cmpgt_imm_rgba(std::numeric_limits<s32>::min(), std::numeric_limits<s32>::max(), actual_g, actual_b - 1);
	check_expected("rgbaint_t::cmpgt_imm_rgba");
	expected_a = 0;
	expected_r = ~s32(0);
	expected_g = 0;
	expected_b = 0;
	rgb.set(actual_a, actual_r, actual_g, actual_b);
	rgb.cmpgt_imm_rgba(actual_a + 1, std::numeric_limits<s32>::min(), std::numeric_limits<s32>::max(), actual_b);
	check_expected("rgbaint_t::cmpgt_imm_rgba");

	// test RGB less than comparison
	actual_a = random_i32_nolimit();
	actual_r = random_i32_nolimit();
	actual_g = random_i32_nolimit();
	actual_b = random_i32_nolimit();
	expected_a = 0;
	expected_r = 0;
	expected_g = ~s32(0);
	expected_b = 0;
	rgb.set(actual_a, actual_r, actual_g, actual_b);
	rgb.cmplt(rgbaint_t(actual_a, actual_r - 1, actual_g + 1, std::numeric_limits<s32>::min()));
	check_expected("rgbaint_t::cmplt");
	expected_a = ~s32(0);
	expected_r = 0;
	expected_g = 0;
	expected_b = ~s32(0);
	rgb.set(actual_a, actual_r, actual_g, actual_b);
	rgb.cmplt(rgbaint_t(std::numeric_limits<s32>::max(), actual_r, actual_g - 1, actual_b + 1));
	check_expected("rgbaint_t::cmplt");

	// test immediate less than comparison
	actual_a = random_i32_nolimit();
	actual_r = random_i32_nolimit();
	actual_g = random_i32_nolimit();
	actual_b = random_i32_nolimit();
	expected_a = 0;
	expected_r = (actual_r < actual_a) ? ~s32(0) : 0;
	expected_g = (actual_g < actual_a) ? ~s32(0) : 0;
	expected_b = (actual_b < actual_a) ? ~s32(0) : 0;
	rgb.set(actual_a, actual_r, actual_g, actual_b);
	rgb.cmplt_imm(actual_a);
	check_expected("rgbaint_t::cmplt_imm");
	expected_a = (actual_a < actual_r) ? ~s32(0) : 0;
	expected_r = 0;
	expected_g = (actual_g < actual_r) ? ~s32(0) : 0;
	expected_b = (actual_b < actual_r) ? ~s32(0) : 0;
	rgb.set(actual_a, actual_r, actual_g, actual_b);
	rgb.cmplt_imm(actual_r);
	check_expected("rgbaint_t::cmplt_imm");
	expected_a = (actual_a < actual_g) ? ~s32(0) : 0;
	expected_r = (actual_r < actual_g) ? ~s32(0) : 0;
	expected_g =0;
	expected_b = (actual_b < actual_g) ? ~s32(0) : 0;
	rgb.set(actual_a, actual_r, actual_g, actual_b);
	rgb.cmplt_imm(actual_g);
	check_expected("rgbaint_t::cmplt_imm");
	expected_a = (actual_a < actual_b) ? ~s32(0) : 0;
	expected_r = (actual_r < actual_b) ? ~s32(0) : 0;
	expected_g = (actual_g < actual_b) ? ~s32(0) : 0;
	expected_b = 0;
	rgb.set(actual_a, actual_r, actual_g, actual_b);
	rgb.cmplt_imm(actual_b);
	check_expected("rgbaint_t::cmplt_imm");
	expected_a = 0;
	expected_r = 0;
	expected_g = 0;
	expected_b = 0;
	rgb.set(actual_a, actual_r, actual_g, actual_b);
	rgb.cmplt_imm(std::numeric_limits<s32>::min());
	check_expected("rgbaint_t::cmplt_imm");
	expected_a = (actual_a < 0) ? ~s32(0) : 0;
	expected_r = (actual_r < 0) ? ~s32(0) : 0;
	expected_g = (actual_g < 0) ? ~s32(0) : 0;
	expected_b = (actual_b < 0) ? ~s32(0) : 0;
	rgb.set(actual_a, actual_r, actual_g, actual_b);
	rgb.cmplt_imm(0);
	check_expected("rgbaint_t::cmplt_imm");
	expected_a = ~s32(0);
	expected_r = ~s32(0);
	expected_g = ~s32(0);
	expected_b = ~s32(0);
	rgb.set(actual_a, actual_r, actual_g, actual_b);
	rgb.cmplt_imm(std::numeric_limits<s32>::max());
	check_expected("rgbaint_t::cmplt_imm");

	// test immediate RGB less than comparison
	actual_a = random_i32_nolimit();
	actual_r = random_i32_nolimit();
	actual_g = random_i32_nolimit();
	actual_b = random_i32_nolimit();
	expected_a = 0;
	expected_r = ~s32(0);
	expected_g = 0;
	expected_b = 0;
	rgb.set(actual_a, actual_r, actual_g, actual_b);
	rgb.cmplt_imm_rgba(std::numeric_limits<s32>::min(), std::numeric_limits<s32>::max(), actual_g, actual_b - 1);
	check_expected("rgbaint_t::cmplt_imm_rgba");
	expected_a = ~s32(0);
	expected_r = 0;
	expected_g = ~s32(0);
	expected_b = 0;
	rgb.set(actual_a, actual_r, actual_g, actual_b);
	rgb.cmplt_imm_rgba(actual_a + 1, std::numeric_limits<s32>::min(), std::numeric_limits<s32>::max(), actual_b);
	check_expected("rgbaint_t::cmplt_imm_rgba");
}


//-------------------------------------------------
//  validate_driver - validate basic driver
//  information
//-------------------------------------------------

void validity_checker::validate_driver()
{
	// check for duplicate names
	if (!m_names_map.insert(std::make_pair(m_current_driver->name, m_current_driver)).second)
	{
		const game_driver *match = m_names_map.find(m_current_driver->name)->second;
		osd_printf_error("Driver name is a duplicate of %s(%s)\n", core_filename_extract_base(match->type.source()).c_str(), match->name);
	}

	// check for duplicate descriptions
	if (!m_descriptions_map.insert(std::make_pair(m_current_driver->type.fullname(), m_current_driver)).second)
	{
		const game_driver *match = m_descriptions_map.find(m_current_driver->type.fullname())->second;
		osd_printf_error("Driver description is a duplicate of %s(%s)\n", core_filename_extract_base(match->type.source()).c_str(), match->name);
	}

	// determine if we are a clone
	bool is_clone = (strcmp(m_current_driver->parent, "0") != 0);
	int clone_of = m_drivlist.clone(*m_current_driver);
	if (clone_of != -1 && (m_drivlist.driver(clone_of).flags & MACHINE_IS_BIOS_ROOT))
		is_clone = false;

	// if we have at least 100 drivers, validate the clone
	// (100 is arbitrary, but tries to avoid tiny.mak dependencies)
	if (driver_list::total() > 100 && clone_of == -1 && is_clone)
		osd_printf_error("Driver is a clone of nonexistent driver %s\n", m_current_driver->parent);

	// look for recursive cloning
	if (clone_of != -1 && &m_drivlist.driver(clone_of) == m_current_driver)
		osd_printf_error("Driver is a clone of itself\n");

	// look for clones that are too deep
	if (clone_of != -1 && (clone_of = m_drivlist.non_bios_clone(clone_of)) != -1)
		osd_printf_error("Driver is a clone of a clone\n");

	// make sure the driver name is not too long
	if (!is_clone && strlen(m_current_driver->name) > 16)
		osd_printf_error("Parent driver name must be 16 characters or less\n");
	if (is_clone && strlen(m_current_driver->name) > 16)
		osd_printf_error("Clone driver name must be 16 characters or less\n");

	// make sure the driver name doesn't contain invalid characters
	for (const char *s = m_current_driver->name; *s != 0; s++)
		if (((*s < '0') || (*s > '9')) && ((*s < 'a') || (*s > 'z')) && (*s != '_'))
		{
			osd_printf_error("Driver name contains invalid characters\n");
			break;
		}

	// make sure the year is only digits, '?' or '+'
	for (const char *s = m_current_driver->year; *s != 0; s++)
		if (!isdigit(u8(*s)) && *s != '?' && *s != '+')
		{
			osd_printf_error("Driver has an invalid year '%s'\n", m_current_driver->year);
			break;
		}

	// normalize driver->compatible_with
	const char *compatible_with = m_current_driver->compatible_with;
	if (compatible_with != nullptr && strcmp(compatible_with, "0") == 0)
		compatible_with = nullptr;

	// check for this driver being compatible with a non-existant driver
	if (compatible_with != nullptr && m_drivlist.find(m_current_driver->compatible_with) == -1)
		osd_printf_error("Driver is listed as compatible with nonexistent driver %s\n", m_current_driver->compatible_with);

	// check for clone_of and compatible_with being specified at the same time
	if (m_drivlist.clone(*m_current_driver) != -1 && compatible_with != nullptr)
		osd_printf_error("Driver cannot be both a clone and listed as compatible with another system\n");

	// find any recursive dependencies on the current driver
	for (int other_drv = m_drivlist.compatible_with(*m_current_driver); other_drv != -1; other_drv = m_drivlist.compatible_with(other_drv))
		if (m_current_driver == &m_drivlist.driver(other_drv))
		{
			osd_printf_error("Driver is recursively compatible with itself\n");
			break;
		}

	// make sure sound-less drivers are flagged
	sound_interface_iterator iter(m_current_config->root_device());
	if ((m_current_driver->flags & MACHINE_IS_BIOS_ROOT) == 0 && !iter.first() && (m_current_driver->flags & (MACHINE_NO_SOUND | MACHINE_NO_SOUND_HW)) == 0)
		osd_printf_error("Driver is missing MACHINE_NO_SOUND flag\n");

	// catch invalid flag combinations
	if ((m_current_driver->flags & MACHINE_WRONG_COLORS) && (m_current_driver->flags & MACHINE_IMPERFECT_COLORS))
		osd_printf_error("Driver cannot have colours that are both completely wrong and imperfect\n");
	if ((m_current_driver->flags & MACHINE_NO_SOUND_HW) && (m_current_driver->flags & (MACHINE_NO_SOUND | MACHINE_IMPERFECT_SOUND)))
		osd_printf_error("Machine without sound hardware cannot have unemulated sound\n");
	if ((m_current_driver->flags & MACHINE_NO_SOUND) && (m_current_driver->flags & MACHINE_IMPERFECT_SOUND))
		osd_printf_error("Driver cannot have sound emulation that's both imperfect and not present\n");
}


//-------------------------------------------------
//  validate_roms - validate ROM definitions
//-------------------------------------------------

void validity_checker::validate_roms(device_t &root)
{
	// iterate, starting with the driver's ROMs and continuing with device ROMs
	for (device_t &device : device_iterator(root))
	{
		// track the current device
		m_current_device = &device;

		// scan the ROM entries for this device
		const char *last_region_name = "???";
		const char *last_name = "???";
		u32 current_length = 0;
		int items_since_region = 1;
		int last_bios = 0;
		int total_files = 0;
		for (const rom_entry *romp = rom_first_region(device); romp != nullptr && !ROMENTRY_ISEND(romp); romp++)
		{
			// if this is a region, make sure it's valid, and record the length
			if (ROMENTRY_ISREGION(romp))
			{
				// if we haven't seen any items since the last region, print a warning
				if (items_since_region == 0)
					osd_printf_warning("Empty ROM region '%s' (warning)\n", last_region_name);

				// reset our region tracking states
				const char *basetag = ROMREGION_GETTAG(romp);
				items_since_region = (ROMREGION_ISERASE(romp) || ROMREGION_ISDISKDATA(romp)) ? 1 : 0;
				last_region_name = basetag;

				// check for a valid tag
				if (basetag == nullptr)
				{
					osd_printf_error("ROM_REGION tag with nullptr name\n");
					continue;
				}

				// validate the base tag
				validate_tag(basetag);

				// generate the full tag
				std::string fulltag = rom_region_name(device, romp);

				// attempt to add it to the map, reporting duplicates as errors
				current_length = ROMREGION_GETLENGTH(romp);
				if (!m_region_map.insert(std::make_pair(fulltag, current_length)).second)
					osd_printf_error("Multiple ROM_REGIONs with the same tag '%s' defined\n", fulltag.c_str());
			}

			// If this is a system bios, make sure it is using the next available bios number
			else if (ROMENTRY_ISSYSTEM_BIOS(romp))
			{
				int bios_flags = ROM_GETBIOSFLAGS(romp);
				if (bios_flags != last_bios + 1)
					osd_printf_error("Non-sequential bios %s (specified as %d, expected to be %d)\n", ROM_GETNAME(romp), bios_flags, last_bios + 1);
				last_bios = bios_flags;
			}

			// if this is a file, make sure it is properly formatted
			else if (ROMENTRY_ISFILE(romp))
			{
				// track the last filename we found
				last_name = ROM_GETNAME(romp);
				total_files++;

				// make sure the hash is valid
				util::hash_collection hashes;
				if (!hashes.from_internal_string(ROM_GETHASHDATA(romp)))
					osd_printf_error("ROM '%s' has an invalid hash string '%s'\n", last_name, ROM_GETHASHDATA(romp));
			}

			// for any non-region ending entries, make sure they don't extend past the end
			if (!ROMENTRY_ISREGIONEND(romp) && current_length > 0)
			{
				items_since_region++;
				if (ROM_GETOFFSET(romp) + ROM_GETLENGTH(romp) > current_length)
					osd_printf_error("ROM '%s' extends past the defined memory region\n", last_name);
			}
		}

		// final check for empty regions
		if (items_since_region == 0)
			osd_printf_warning("Empty ROM region '%s' (warning)\n", last_region_name);


		// reset the current device
		m_current_device = nullptr;
	}
}


//-------------------------------------------------
//  validate_analog_input_field - validate an
//  analog input field
//-------------------------------------------------

void validity_checker::validate_analog_input_field(ioport_field &field)
{
	// analog ports must have a valid sensitivity
	if (field.sensitivity() == 0)
		osd_printf_error("Analog port with zero sensitivity\n");

	// check that the default falls in the bitmask range
	if (field.defvalue() & ~field.mask())
		osd_printf_error("Analog port with a default value (%X) out of the bitmask range (%X)\n", field.defvalue(), field.mask());

	// tests for positional devices
	if (field.type() == IPT_POSITIONAL || field.type() == IPT_POSITIONAL_V)
	{
		int shift;
		for (shift = 0; shift <= 31 && (~field.mask() & (1 << shift)) != 0; shift++) { }

		// convert the positional max value to be in the bitmask for testing
		//s32 analog_max = field.maxval();
		//analog_max = (analog_max - 1) << shift;

		// positional port size must fit in bits used
		if ((field.mask() >> shift) + 1 < field.maxval())
			osd_printf_error("Analog port with a positional port size bigger then the mask size\n");
	}

	// tests for absolute devices
	else if (field.type() > IPT_ANALOG_ABSOLUTE_FIRST && field.type() < IPT_ANALOG_ABSOLUTE_LAST)
	{
		// adjust for signed values
		s32 default_value = field.defvalue();
		s32 analog_min = field.minval();
		s32 analog_max = field.maxval();
		if (analog_min > analog_max)
		{
			analog_min = -analog_min;
			if (default_value > analog_max)
				default_value = -default_value;
		}

		// check that the default falls in the MINMAX range
		if (default_value < analog_min || default_value > analog_max)
			osd_printf_error("Analog port with a default value (%X) out of PORT_MINMAX range (%X-%X)\n", field.defvalue(), field.minval(), field.maxval());

		// check that the MINMAX falls in the bitmask range
		// we use the unadjusted min for testing
		if (field.minval() & ~field.mask() || analog_max & ~field.mask())
			osd_printf_error("Analog port with a PORT_MINMAX (%X-%X) value out of the bitmask range (%X)\n", field.minval(), field.maxval(), field.mask());

		// absolute analog ports do not use PORT_RESET
		if (field.analog_reset())
			osd_printf_error("Absolute analog port using PORT_RESET\n");

		// absolute analog ports do not use PORT_WRAPS
		if (field.analog_wraps())
			osd_printf_error("Absolute analog port using PORT_WRAPS\n");
	}

	// tests for non IPT_POSITIONAL relative devices
	else
	{
		// relative devices do not use PORT_MINMAX
		if (field.minval() != 0 || field.maxval() != field.mask())
			osd_printf_error("Relative port using PORT_MINMAX\n");

		// relative devices do not use a default value
		// the counter is at 0 on power up
		if (field.defvalue() != 0)
			osd_printf_error("Relative port using non-0 default value\n");

		// relative analog ports do not use PORT_WRAPS
		if (field.analog_wraps())
			osd_printf_error("Absolute analog port using PORT_WRAPS\n");
	}
}


//-------------------------------------------------
//  validate_dip_settings - validate a DIP switch
//  setting
//-------------------------------------------------

void validity_checker::validate_dip_settings(ioport_field &field)
{
	const char *demo_sounds = ioport_string_from_index(INPUT_STRING_Demo_Sounds);
	const char *flipscreen = ioport_string_from_index(INPUT_STRING_Flip_Screen);
	u8 coin_list[__input_string_coinage_end + 1 - __input_string_coinage_start] = { 0 };
	bool coin_error = false;

	// iterate through the settings
	for (ioport_setting &setting : field.settings())
	{
		// note any coinage strings
		int strindex = get_defstr_index(setting.name());
		if (strindex >= __input_string_coinage_start && strindex <= __input_string_coinage_end)
			coin_list[strindex - __input_string_coinage_start] = 1;

		// make sure demo sounds default to on
		if (field.name() == demo_sounds && strindex == INPUT_STRING_On && field.defvalue() != setting.value())
			osd_printf_error("Demo Sounds must default to On\n");

		// check for bad demo sounds options
		if (field.name() == demo_sounds && (strindex == INPUT_STRING_Yes || strindex == INPUT_STRING_No))
			osd_printf_error("Demo Sounds option must be Off/On, not %s\n", setting.name());

		// check for bad flip screen options
		if (field.name() == flipscreen && (strindex == INPUT_STRING_Yes || strindex == INPUT_STRING_No))
			osd_printf_error("Flip Screen option must be Off/On, not %s\n", setting.name());

		// if we have a neighbor, compare ourselves to him
		if (setting.next() != nullptr)
		{
			// check for inverted off/on dispswitch order
			int next_strindex = get_defstr_index(setting.next()->name(), true);
			if (strindex == INPUT_STRING_On && next_strindex == INPUT_STRING_Off)
				osd_printf_error("%s option must have Off/On options in the order: Off, On\n", field.name());

			// check for inverted yes/no dispswitch order
			else if (strindex == INPUT_STRING_Yes && next_strindex == INPUT_STRING_No)
				osd_printf_error("%s option must have Yes/No options in the order: No, Yes\n", field.name());

			// check for inverted upright/cocktail dispswitch order
			else if (strindex == INPUT_STRING_Cocktail && next_strindex == INPUT_STRING_Upright)
				osd_printf_error("%s option must have Upright/Cocktail options in the order: Upright, Cocktail\n", field.name());

			// check for proper coin ordering
			else if (strindex >= __input_string_coinage_start && strindex <= __input_string_coinage_end && next_strindex >= __input_string_coinage_start && next_strindex <= __input_string_coinage_end &&
						strindex >= next_strindex && setting.condition() == setting.next()->condition())
			{
				osd_printf_error("%s option has unsorted coinage %s > %s\n", field.name(), setting.name(), setting.next()->name());
				coin_error = true;
			}
		}
	}

	// if we have a coin error, demonstrate the correct way
	if (coin_error)
	{
		output_via_delegate(OSD_OUTPUT_CHANNEL_ERROR, "   Note proper coin sort order should be:\n");
		for (int entry = 0; entry < ARRAY_LENGTH(coin_list); entry++)
			if (coin_list[entry])
				output_via_delegate(OSD_OUTPUT_CHANNEL_ERROR, "      %s\n", ioport_string_from_index(__input_string_coinage_start + entry));
	}
}


//-------------------------------------------------
//  validate_condition - validate a condition
//  stored within an ioport field or setting
//-------------------------------------------------

void validity_checker::validate_condition(ioport_condition &condition, device_t &device, std::unordered_set<std::string> &port_map)
{
	// resolve the tag
	// then find a matching port
	if (port_map.find(device.subtag(condition.tag())) == port_map.end())
		osd_printf_error("Condition referencing non-existent ioport tag '%s'\n", condition.tag());
}


//-------------------------------------------------
//  validate_inputs - validate input configuration
//-------------------------------------------------

void validity_checker::validate_inputs()
{
	std::unordered_set<std::string> port_map;

	// iterate over devices
	for (device_t &device : device_iterator(m_current_config->root_device()))
	{
		// see if this device has ports; if not continue
		if (device.input_ports() == nullptr)
			continue;

		// track the current device
		m_current_device = &device;

		// allocate the input ports
		ioport_list portlist;
		std::string errorbuf;
		portlist.append(device, errorbuf);

		// report any errors during construction
		if (!errorbuf.empty())
			osd_printf_error("I/O port error during construction:\n%s\n", errorbuf.c_str());

		// do a first pass over ports to add their names and find duplicates
		for (auto &port : portlist)
			if (!port_map.insert(port.second->tag()).second)
				osd_printf_error("Multiple I/O ports with the same tag '%s' defined\n", port.second->tag());

		// iterate over ports
		for (auto &port : portlist)
		{
			m_current_ioport = port.second->tag();

			// iterate through the fields on this port
			for (ioport_field &field : port.second->fields())
			{
				// verify analog inputs
				if (field.is_analog())
					validate_analog_input_field(field);

				// look for invalid (0) types which should be mapped to IPT_OTHER
				if (field.type() == IPT_INVALID)
					osd_printf_error("Field has an invalid type (0); use IPT_OTHER instead\n");

				// verify dip switches
				if (field.type() == IPT_DIPSWITCH)
				{
					// dip switch fields must have a specific name
					if (field.specific_name() == nullptr)
						osd_printf_error("DIP switch has no specific name\n");

					// verify the settings list
					validate_dip_settings(field);
				}

				// verify config settings
				if (field.type() == IPT_CONFIG)
				{
					// config fields must have a specific name
					if (field.specific_name() == nullptr)
						osd_printf_error("Config switch has no specific name\n");
				}

				// verify names
				const char *name = field.specific_name();
				if (name != nullptr)
				{
					// check for empty string
					if (name[0] == 0)
						osd_printf_error("Field name is an empty string\n");

					// check for trailing spaces
					if (name[0] != 0 && name[strlen(name) - 1] == ' ')
						osd_printf_error("Field '%s' has trailing spaces\n", name);

					// check for invalid UTF-8
					if (!utf8_is_valid_string(name))
						osd_printf_error("Field '%s' has invalid characters\n", name);

					// look up the string and print an error if default strings are not used
					/*strindex =get_defstr_index(defstr_map, name, driver, &error);*/
				}

				// verify conditions on the field
				if (!field.condition().none())
					validate_condition(field.condition(), device, port_map);

				// verify conditions on the settings
				for (ioport_setting &setting : field.settings())
					if (!setting.condition().none())
						validate_condition(setting.condition(), device, port_map);
			}

			// done with this port
			m_current_ioport = nullptr;
		}

		// done with this device
		m_current_device = nullptr;
	}
}


//-------------------------------------------------
//  validate_devices - run per-device validity
//  checks
//-------------------------------------------------

void validity_checker::validate_devices()
{
	std::unordered_set<std::string> device_map;

	for (device_t &device : device_iterator(m_current_config->root_device()))
	{
		// track the current device
		m_current_device = &device;

		// validate auto-finders
		device.findit(true);

		// validate the device tag
		validate_tag(device.basetag());

		// look for duplicates
		if (!device_map.insert(device.tag()).second)
			osd_printf_error("Multiple devices with the same tag defined\n");

		// check for device-specific validity check
		device.validity_check(*this);

		// done with this device
		m_current_device = nullptr;

		// if it's a slot, iterate over possible cards (don't recurse, or you'll stack infinite tee connectors)
		device_slot_interface *const slot = dynamic_cast<device_slot_interface *>(&device);
		if (slot != nullptr && !slot->fixed())
		{
			for (auto &option : slot->option_list())
			{
				// the default option is already instantiated here, so don't try adding it again
				if (slot->default_option() != nullptr && option.first == slot->default_option())
					continue;

				device_t *const card = m_current_config->device_add(&slot->device(), option.second->name(), option.second->devtype(), option.second->clock());

				const char *const def_bios = option.second->default_bios();
				if (def_bios)
					device_t::static_set_default_bios_tag(*card, def_bios);
				machine_config_constructor const additions = option.second->machine_config();
				if (additions)
					(*additions)(*m_current_config, card, card);

				for (device_slot_interface &subslot : slot_interface_iterator(*card))
				{
					if (subslot.fixed())
					{
						device_slot_option const *const suboption = subslot.option(subslot.default_option());
						if (suboption)
						{
							device_t *const sub_card = m_current_config->device_add(&subslot.device(), suboption->name(), suboption->devtype(), suboption->clock());
							const char *const sub_bios = suboption->default_bios();
							if (sub_bios)
								device_t::static_set_default_bios_tag(*sub_card, sub_bios);
							machine_config_constructor const sub_additions = suboption->machine_config();
							if (sub_additions)
								(*sub_additions)(*m_current_config, sub_card, sub_card);
						}
					}
				}

				for (device_t &card_dev : device_iterator(*card))
					card_dev.config_complete();
				validate_roms(*card);

				for (device_t &card_dev : device_iterator(*card))
				{
					m_current_device = &card_dev;
					card_dev.findit(true);
					card_dev.validity_check(*this);
					m_current_device = nullptr;
				}

				m_current_config->device_remove(&slot->device(), option.second->name());
			}
		}
	}
}


//-------------------------------------------------
//  validate_devices_types - check validity of
//  registered device types
//-------------------------------------------------

void validity_checker::validate_device_types()
{
	// reset error/warning state
	int start_errors = m_errors;
	int start_warnings = m_warnings;
	m_error_text.clear();
	m_warning_text.clear();
	m_verbose_text.clear();

	std::unordered_map<std::string, std::add_pointer_t<device_type> > device_name_map, device_shortname_map;
	machine_config config(GAME_NAME(___empty), m_drivlist.options());
	for (device_type type : registered_device_types)
	{
		device_t *const dev = config.device_add(&config.root_device(), "_tmp", type, 0);

		char const *name((dev->shortname() && *dev->shortname()) ? dev->shortname() : type.type().name());
		std::string const description((dev->source() && *dev->source()) ? util::string_format("%s(%s)", core_filename_extract_base(dev->source()).c_str(), name) : name);

		// ensure shortname exists
		if (!dev->shortname() || !*dev->shortname())
		{
			osd_printf_error("Device %s does not have short name defined\n", description.c_str());
		}
		else
		{
			// make sure the device name is not too long
			if (strlen(dev->shortname()) > 32)
				osd_printf_error("Device short name must be 32 characters or less\n");

			// check for invalid characters in shortname
			for (char const *s = dev->shortname(); *s; ++s)
			{
				if (((*s < '0') || (*s > '9')) && ((*s < 'a') || (*s > 'z')) && (*s != '_'))
				{
					osd_printf_error("Device %s short name contains invalid characters\n", description.c_str());
					break;
				}
			}

			// check for name conflicts
			auto const drvname(m_names_map.find(dev->shortname()));
			auto const devname(device_shortname_map.emplace(dev->shortname(), &type));
			if (m_names_map.end() != drvname)
			{
				game_driver const &dup(*drvname->second);
				osd_printf_error("Device %s short name is a duplicate of %s(%s)\n", description.c_str(), core_filename_extract_base(dup.type.source()).c_str(), dup.name);
			}
			else if (!devname.second)
			{
				device_t *const dup = config.device_add(&config.root_device(), "_dup", *devname.first->second, 0);
				osd_printf_error("Device %s short name is a duplicate of %s(%s)\n", description.c_str(), core_filename_extract_base(dup->source()).c_str(), dup->shortname());
				config.device_remove(&config.root_device(), "_dup");
			}
		}

		// ensure name exists
		if (!dev->name() || !*dev->name())
		{
			osd_printf_error("Device %s does not have name defined\n", description.c_str());
		}
		else
		{
			// check for description conflicts
			auto const drvdesc(m_descriptions_map.find(dev->name()));
			auto const devdesc(device_name_map.emplace(dev->name(), &type));
			if (m_names_map.end() != drvdesc)
			{
				game_driver const &dup(*drvdesc->second);
				osd_printf_error("Device %s name '%s' is a duplicate of %s(%s)\n", description.c_str(), dev->name(), core_filename_extract_base(dup.type.source()).c_str(), dup.name);
			}
			else if (!devdesc.second)
			{
				device_t *const dup = config.device_add(&config.root_device(), "_dup", *devdesc.first->second, 0);
				osd_printf_error("Device %s name '%s' is a duplicate of %s(%s)\n", description.c_str(), dev->name(), core_filename_extract_base(dup->source()).c_str(), dup->shortname());
				config.device_remove(&config.root_device(), "_dup");
			}
		}

		// ensure source exists
		if (!dev->source() || !*dev->source())
			osd_printf_error("Device %s does not have source defined\n", description.c_str());

		// check that reported type matches supplied type
		if (dev->type().type() != type.type())
			osd_printf_error("Device %s reports type '%s' (created with '%s')\n", description.c_str(), dev->type().type().name(), type.type().name());

		config.device_remove(&config.root_device(), "_tmp");
	}

	// if we had warnings or errors, output
	if (m_errors > start_errors || m_warnings > start_warnings || !m_verbose_text.empty())
	{
		output_via_delegate(OSD_OUTPUT_CHANNEL_ERROR, "%d errors, %d warnings\n", m_errors - start_errors, m_warnings - start_warnings);
		if (m_errors > start_errors)
			output_indented_errors(m_error_text, "Errors");
		if (m_warnings > start_warnings)
			output_indented_errors(m_warning_text, "Warnings");
		if (!m_verbose_text.empty())
			output_indented_errors(m_verbose_text, "Messages");
		output_via_delegate(OSD_OUTPUT_CHANNEL_ERROR, "\n");
	}
}


//-------------------------------------------------
//  build_output_prefix - create a prefix
//  indicating the current source file, driver,
//  and device
//-------------------------------------------------

void validity_checker::build_output_prefix(std::string &str)
{
	// start empty
	str.clear();

	// if we have a current (non-root) device, indicate that
	if (m_current_device != nullptr && m_current_device->owner() != nullptr)
		str.append(m_current_device->name()).append(" device '").append(m_current_device->tag() + 1).append("': ");

	// if we have a current port, indicate that as well
	if (m_current_ioport != nullptr)
		str.append("ioport '").append(m_current_ioport).append("': ");
}


//-------------------------------------------------
//  error_output - error message output override
//-------------------------------------------------

void validity_checker::output_callback(osd_output_channel channel, const char *msg, va_list args)
{
	std::string output;
	switch (channel)
	{
	case OSD_OUTPUT_CHANNEL_ERROR:
		// count the error
		m_errors++;

		// output the source(driver) device 'tag'
		build_output_prefix(output);

		// generate the string
		strcatvprintf(output, msg, args);
		m_error_text.append(output);
		break;

	case OSD_OUTPUT_CHANNEL_WARNING:
		// count the error
		m_warnings++;

		// output the source(driver) device 'tag'
		build_output_prefix(output);

		// generate the string and output to the original target
		strcatvprintf(output, msg, args);
		m_warning_text.append(output);
		break;

	case OSD_OUTPUT_CHANNEL_VERBOSE:
		// if we're not verbose, skip it
		if (!m_print_verbose) break;

		// output the source(driver) device 'tag'
		build_output_prefix(output);

		// generate the string and output to the original target
		strcatvprintf(output, msg, args);
		m_verbose_text.append(output);
		break;

	default:
		chain_output(channel, msg, args);
		break;
	}
}

//-------------------------------------------------
//  output_via_delegate - helper to output a
//  message via a varargs string, so the argptr
//  can be forwarded onto the given delegate
//-------------------------------------------------

void validity_checker::output_via_delegate(osd_output_channel channel, const char *format, ...)
{
	va_list argptr;

	// call through to the delegate with the proper parameters
	va_start(argptr, format);
	chain_output(channel, format, argptr);
	va_end(argptr);
}

//-------------------------------------------------
//  output_indented_errors - helper to output error
//  and warning messages with header and indents
//-------------------------------------------------
void validity_checker::output_indented_errors(std::string &text, const char *header)
{
	// remove trailing newline
	if (text[text.size()-1] == '\n')
		text.erase(text.size()-1, 1);
	strreplace(text, "\n", "\n   ");
	output_via_delegate(OSD_OUTPUT_CHANNEL_ERROR, "%s:\n   %s\n", header, text.c_str());
}
