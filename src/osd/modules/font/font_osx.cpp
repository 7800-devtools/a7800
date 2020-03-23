// license:BSD-3-Clause
// copyright-holders:Olivier Galibert, R. Belmont, Vas Crabb
/*
 * font_osx.c
 *
 */

#include "font_module.h"
#include "modules/osdmodule.h"

#ifdef SDLMAME_MACOSX

#include "corealloc.h"
#include "fileio.h"

#include <ApplicationServices/ApplicationServices.h>
#include <CoreFoundation/CoreFoundation.h>

//-------------------------------------------------
//  font_open - attempt to "open" a handle to the
//  font with the given name
//-------------------------------------------------

class osd_font_osx : public osd_font
{
public:
	osd_font_osx() : m_font(nullptr) { }
	osd_font_osx(osd_font_osx &&obj) : m_font(obj.m_font) { obj.m_font = nullptr; }
	virtual ~osd_font_osx() { close(); }

	virtual bool open(std::string const &font_path, std::string const &name, int &height);
	virtual void close();
	virtual bool get_bitmap(char32_t chnum, bitmap_argb32 &bitmap, std::int32_t &width, std::int32_t &xoffs, std::int32_t &yoffs);

	osd_font_osx &operator=(osd_font_osx &&obj)
	{
		using std::swap;
		swap(m_font, obj.m_font);
		return *this;
	}

private:
	osd_font_osx(osd_font_osx const &) = delete;
	osd_font_osx &operator=(osd_font_osx const &) = delete;

	static constexpr CGFloat POINT_SIZE = 144.0;
	static constexpr CGFloat EXTRA_HEIGHT = 1.0;
	static constexpr CGFloat EXTRA_WIDTH = 1.15;

	CTFontRef m_font;
};

bool osd_font_osx::open(std::string const &font_path, std::string const &name, int &height)
{
	osd_printf_verbose("FONT NAME %s\n", name.c_str());
#if 0
	if (name != "default")
	{
		name = "LucidaGrande";
	}
#endif

	CFStringRef const font_name = CFStringCreateWithCString(nullptr, name.c_str(), kCFStringEncodingUTF8);
	if (font_name && (kCFNotFound != CFStringFind(font_name, CFSTR(".BDF"), kCFCompareCaseInsensitive | kCFCompareBackwards | kCFCompareAnchored | kCFCompareNonliteral).location))
	{
		// handle bdf fonts in the core
		CFRelease(font_name);
		return false;
	}
	CTFontRef ct_font = nullptr;
	if (font_name)
	{
		CTFontDescriptorRef const font_descriptor = CTFontDescriptorCreateWithNameAndSize(font_name, 0.0);
		if (font_descriptor)
		{
			ct_font = CTFontCreateWithFontDescriptor(font_descriptor, POINT_SIZE, &CGAffineTransformIdentity);
			CFRelease(font_descriptor);
		}
		CFRelease(font_name);
	}

	if (!ct_font)
	{
		osd_printf_verbose("Couldn't find/open font %s, using MAME default\n", name.c_str());
		return false;
	}

	CFStringRef const real_name = CTFontCopyPostScriptName(ct_font);
	char real_name_c_string[255];
	CFStringGetCString(real_name, real_name_c_string, 255, kCFStringEncodingUTF8);
	osd_printf_verbose("Matching font: %s\n", real_name_c_string);
	CFRelease(real_name);

	CGFloat line_height = 0.0;
	line_height += CTFontGetAscent(ct_font);
	line_height += CTFontGetDescent(ct_font);
	line_height += CTFontGetLeading(ct_font);
	height = ceilf(line_height * EXTRA_HEIGHT);

	close();
	m_font = ct_font;
	return true;
}

//-------------------------------------------------
//  font_close - release resources associated with
//  a given OSD font
//-------------------------------------------------

void osd_font_osx::close()
{
	if (m_font != nullptr)
		CFRelease(m_font);
	m_font = nullptr;
}

//-------------------------------------------------
//  font_get_bitmap - allocate and populate a
//  BITMAP_FORMAT_ARGB32 bitmap containing the
//  pixel values rgb_t(0xff,0xff,0xff,0xff)
//  or rgb_t(0x00,0xff,0xff,0xff) for each
//  pixel of a black & white font
//-------------------------------------------------

bool osd_font_osx::get_bitmap(char32_t chnum, bitmap_argb32 &bitmap, std::int32_t &width, std::int32_t &xoffs, std::int32_t &yoffs)
{
	UniChar uni_char;
	CGGlyph glyph;
	CTFontRef ct_font = m_font;
	const CFIndex count = 1;
	CGRect bounding_rect, success_rect;
	CGContextRef context_ref;

	if( chnum == ' ' )
	{
		uni_char = 'n';
		CTFontGetGlyphsForCharacters( ct_font, &uni_char, &glyph, count );
		success_rect = CTFontGetBoundingRectsForGlyphs( ct_font, kCTFontDefaultOrientation, &glyph, &bounding_rect, count );
		uni_char = chnum;
		CTFontGetGlyphsForCharacters( ct_font, &uni_char, &glyph, count );
	}
	else
	{
		uni_char = chnum;
		CTFontGetGlyphsForCharacters( ct_font, &uni_char, &glyph, count );
		success_rect = CTFontGetBoundingRectsForGlyphs( ct_font, kCTFontDefaultOrientation, &glyph, &bounding_rect, count );
	}

	if( CGRectEqualToRect( success_rect, CGRectNull ) == false )
	{
		size_t bitmap_width;
		size_t bitmap_height;

		bitmap_width = ceilf(bounding_rect.size.width * EXTRA_WIDTH);
		bitmap_width = bitmap_width == 0 ? 1 : bitmap_width;

		bitmap_height = ceilf( (CTFontGetAscent(ct_font) + CTFontGetDescent(ct_font) + CTFontGetLeading(ct_font)) * EXTRA_HEIGHT);

		xoffs = yoffs = 0;
		width = bitmap_width;

		size_t bits_per_component;
		CGColorSpaceRef color_space;
		CGBitmapInfo bitmap_info = kCGBitmapByteOrder32Host | kCGImageAlphaPremultipliedFirst;

		color_space = CGColorSpaceCreateDeviceRGB();
		bits_per_component = 8;

		bitmap.allocate(bitmap_width, bitmap_height);

		context_ref = CGBitmapContextCreate( bitmap.raw_pixptr(0), bitmap_width, bitmap_height, bits_per_component, bitmap.rowpixels()*4, color_space, bitmap_info );

		if( context_ref != nullptr )
		{
			CGFontRef font_ref;
			font_ref = CTFontCopyGraphicsFont( ct_font, nullptr );
			CGContextSetTextPosition(context_ref, -bounding_rect.origin.x*EXTRA_WIDTH, CTFontGetDescent(ct_font)+CTFontGetLeading(ct_font) );
			CGContextSetRGBFillColor(context_ref, 1.0, 1.0, 1.0, 1.0);
			CGContextSetFont( context_ref, font_ref );
			CGContextSetFontSize( context_ref, POINT_SIZE );
			CGContextShowGlyphs( context_ref, &glyph, count );
			CGFontRelease( font_ref );
			CGContextRelease( context_ref );
		}

		CGColorSpaceRelease( color_space );
	}

	return bitmap.valid();
}


class font_osx : public osd_module, public font_module
{
public:
	font_osx() : osd_module(OSD_FONT_PROVIDER, "osx"), font_module() { }

	virtual int init(const osd_options &options) override { return 0; }
	virtual osd_font::ptr font_alloc() override { return std::make_unique<osd_font_osx>(); }
	virtual bool get_font_families(std::string const &font_path, std::vector<std::pair<std::string, std::string> > &result) override;

private:
	static CFComparisonResult sort_callback(CTFontDescriptorRef first, CTFontDescriptorRef second, void *refCon)
	{
		CFStringRef left = (CFStringRef)CTFontDescriptorCopyLocalizedAttribute(first, kCTFontDisplayNameAttribute, nullptr);
		if (!left) left = (CFStringRef)CTFontDescriptorCopyAttribute(first, kCTFontNameAttribute);
		CFStringRef right = (CFStringRef)CTFontDescriptorCopyLocalizedAttribute(second, kCTFontDisplayNameAttribute, nullptr);
		if (!right) right = (CFStringRef)CTFontDescriptorCopyAttribute(second, kCTFontNameAttribute);

		CFComparisonResult result;
		if (left && right) result = CFStringCompareWithOptions(left, right, CFRangeMake(0, CFStringGetLength(left)), kCFCompareCaseInsensitive | kCFCompareLocalized | kCFCompareNonliteral);
		else if (!left) result = kCFCompareLessThan;
		else if (!right) result = kCFCompareGreaterThan;
		else result = kCFCompareEqualTo;

		if (left) CFRelease(left);
		if (right) CFRelease(right);
		return result;
	}
};

bool font_osx::get_font_families(std::string const &font_path, std::vector<std::pair<std::string, std::string> > &result)
{
	CFStringRef keys[] = { kCTFontCollectionRemoveDuplicatesOption };
	std::uintptr_t values[ARRAY_LENGTH(keys)] = { 1 };
	CFDictionaryRef const options = CFDictionaryCreate(kCFAllocatorDefault, (void const **)keys, (void const **)values, ARRAY_LENGTH(keys), &kCFTypeDictionaryKeyCallBacks, nullptr);
	CTFontCollectionRef const collection = CTFontCollectionCreateFromAvailableFonts(nullptr);
	CFRelease(options);
	if (!collection) return false;

	CFArrayRef const descriptors = CTFontCollectionCreateMatchingFontDescriptorsSortedWithCallback(collection, &sort_callback, nullptr);
	CFRelease(collection);
	if (!descriptors) return false;

	result.clear();
	CFIndex const count = CFArrayGetCount(descriptors);
	result.reserve(count);
	for (CFIndex i = 0; i != count; i++)
	{
		CTFontDescriptorRef const font = (CTFontDescriptorRef)CFArrayGetValueAtIndex(descriptors, i);
		CFStringRef const name = (CFStringRef)CTFontDescriptorCopyAttribute(font, kCTFontNameAttribute);
		CFStringRef const display = (CFStringRef)CTFontDescriptorCopyLocalizedAttribute(font, kCTFontDisplayNameAttribute, nullptr);

		if (name && display)
		{
			char const *utf;
			std::vector<char> buf;

			utf = CFStringGetCStringPtr(name, kCFStringEncodingUTF8);
			if (!utf)
			{
				buf.resize(CFStringGetMaximumSizeForEncoding(std::max(CFStringGetLength(name), CFStringGetLength(display)), kCFStringEncodingUTF8));
				CFStringGetCString(name, &buf[0], buf.size(), kCFStringEncodingUTF8);
			}
			std::string utf8name(utf ? utf : &buf[0]);

			utf = CFStringGetCStringPtr(display, kCFStringEncodingUTF8);
			if (!utf)
			{
				buf.resize(CFStringGetMaximumSizeForEncoding(CFStringGetLength(display), kCFStringEncodingUTF8));
				CFStringGetCString(display, &buf[0], buf.size(), kCFStringEncodingUTF8);
			}
			std::string utf8display(utf ? utf : &buf[0]);

			result.emplace_back(std::move(utf8name), std::move(utf8display));
		}

		if (name) CFRelease(name);
		if (display) CFRelease(display);
	}

	return true;
}

#else /* SDLMAME_MACOSX */

MODULE_NOT_SUPPORTED(font_osx, OSD_FONT_PROVIDER, "osx")

#endif

MODULE_DEFINITION(FONT_OSX, font_osx)
