// license:BSD-3-Clause
// copyright-holders:Nicola Salmoria, Aaron Giles, Alex W. Jackson
/***************************************************************************

    digfx.c

    Device graphics interfaces.

***************************************************************************/

#include "emu.h"
#include "validity.h"


//**************************************************************************
//  DEVICE GFX INTERFACE
//**************************************************************************

//-------------------------------------------------
//  device_gfx_interface - constructor
//-------------------------------------------------

device_gfx_interface::device_gfx_interface(const machine_config &mconfig, device_t &device,
										const gfx_decode_entry *gfxinfo, const char *palette_tag)
	: device_interface(device, "gfx"),
	m_palette(nullptr),
	m_gfxdecodeinfo(gfxinfo),
	m_palette_tag(palette_tag),
	m_palette_is_sibling(palette_tag == nullptr),
	m_palette_is_disabled(false),
	m_decoded(false)
{
}

//-------------------------------------------------
//  ~device_gfx_interface - destructor
//-------------------------------------------------

device_gfx_interface::~device_gfx_interface()
{
}


//-------------------------------------------------
//  static_set_info: configuration helper to set
//  the gfxdecode info used by the device
//-------------------------------------------------

void device_gfx_interface::static_set_info(device_t &device, const gfx_decode_entry *gfxinfo)
{
	device_gfx_interface *gfx;
	if (!device.interface(gfx))
		throw emu_fatalerror("MCFG_GFX_INFO called on device '%s' with no gfx interface\n", device.tag());

	gfx->m_gfxdecodeinfo = gfxinfo;
}


//-------------------------------------------------
//  static_set_palette: configuration helper to
//  set the palette used by the device
//-------------------------------------------------

void device_gfx_interface::static_set_palette(device_t &device, const char *tag)
{
	device_gfx_interface *gfx;
	if (!device.interface(gfx))
		throw emu_fatalerror("MCFG_GFX_PALETTE called on device '%s' with no gfx interface\n", device.tag());

	gfx->m_palette_tag = tag;
	gfx->m_palette_is_sibling = true;
}


//-------------------------------------------------
//  set_palette_disable: configuration helper to
//  disable the use of a palette by the device
//-------------------------------------------------

void device_gfx_interface::set_palette_disable(bool disable)
{
	m_palette_is_disabled = disable;
}


//-------------------------------------------------
//  interface_pre_start - make sure all our input
//  devices are started
//-------------------------------------------------

void device_gfx_interface::interface_pre_start()
{
	if (!m_palette_is_disabled)
	{
		if (m_palette_tag == nullptr)
			fatalerror("No palette specified for device\n");

		// find our palette device, either as a sibling device or subdevice
		device_t *paldev;
		if (m_palette_is_sibling)
			paldev = device().owner()->subdevice(m_palette_tag);
		else
			paldev = device().subdevice(m_palette_tag);

		if (paldev == nullptr)
			fatalerror("Device '%s' specifies nonexistent %sdevice '%s' as palette\n",
					   device().tag(),
					   (m_palette_is_sibling ? "sibling " : "sub"),
					   m_palette_tag);
		if (!paldev->interface(m_palette))
			fatalerror("Device '%s' specifies %sdevice '%s' as palette, but it has no palette interface\n",
					   device().tag(),
					   (m_palette_is_sibling ? "sibling " : "sub"),
					   m_palette_tag);
	}

	// if palette device isn't started, wait for it
	// if (!m_palette->device().started())
	//  throw device_missing_dependencies();
}


//-------------------------------------------------
//  interface_post_start - decode gfx, if we
//  haven't done so already
//-------------------------------------------------

void device_gfx_interface::interface_post_start()
{
	if (!m_decoded)
		decode_gfx(m_gfxdecodeinfo);
}


//-------------------------------------------------
//  decode_gfx - parse gfx decode info and
//  create gfx elements
//-------------------------------------------------

void device_gfx_interface::decode_gfx(const gfx_decode_entry *gfxdecodeinfo)
{
	// skip if nothing to do
	if (gfxdecodeinfo == nullptr)
		return;

	// local variables to hold mutable copies of gfx layout data
	gfx_layout glcopy;
	std::vector<u32> extxoffs(0);
	std::vector<u32> extyoffs(0);

	// loop over all elements
	for (u8 curgfx = 0; curgfx < MAX_GFX_ELEMENTS && gfxdecodeinfo[curgfx].gfxlayout != nullptr; curgfx++)
	{
		const gfx_decode_entry &gfx = gfxdecodeinfo[curgfx];

		// extract the scale factors and xormask
		u32 xscale = GFXENTRY_GETXSCALE(gfx.flags);
		u32 yscale = GFXENTRY_GETYSCALE(gfx.flags);
		u32 xormask = GFXENTRY_ISREVERSE(gfx.flags) ? 7 : 0;

		// resolve the region
		u32          region_length;
		const u8     *region_base;
		u8           region_width;
		endianness_t region_endianness;

		if (gfx.memory_region != nullptr)
		{
			device_t &basedevice = (GFXENTRY_ISDEVICE(gfx.flags)) ? device() : *device().owner();
			if (GFXENTRY_ISRAM(gfx.flags))
			{
				memory_share *share = basedevice.memshare(gfx.memory_region);
				assert(share != nullptr);
				region_length = 8 * share->bytes();
				region_base = reinterpret_cast<u8 *>(share->ptr());
				region_width = share->bytewidth();
				region_endianness = share->endianness();
			}
			else
			{
				memory_region *region = basedevice.memregion(gfx.memory_region);
				assert(region != nullptr);
				region_length = 8 * region->bytes();
				region_base = region->base();
				region_width = region->bytewidth();
				region_endianness = region->endianness();
			}
		}
		else
		{
			region_length = 0;
			region_base = nullptr;
			region_width = 1;
			region_endianness = ENDIANNESS_NATIVE;
		}

		if (region_endianness != ENDIANNESS_NATIVE)
		{
			switch (region_width)
			{
				case 2:
					xormask |= 0x08;
					break;
				case 4:
					xormask |= 0x18;
					break;
				case 8:
					xormask |= 0x38;
					break;
			}
		}

		// copy the layout into our temporary variable
		memcpy(&glcopy, gfx.gfxlayout, sizeof(gfx_layout));

		// if the character count is a region fraction, compute the effective total
		if (IS_FRAC(glcopy.total))
		{
			assert(region_length != 0);
			glcopy.total = region_length / glcopy.charincrement * FRAC_NUM(glcopy.total) / FRAC_DEN(glcopy.total);
		}

		// for non-raw graphics, decode the X and Y offsets
		if (glcopy.planeoffset[0] != GFX_RAW)
		{
			// copy the X and Y offsets into our temporary arrays
			extxoffs.resize(glcopy.width * xscale);
			extyoffs.resize(glcopy.height * yscale);
			memcpy(&extxoffs[0], (glcopy.extxoffs != nullptr) ? glcopy.extxoffs : glcopy.xoffset, glcopy.width * sizeof(u32));
			memcpy(&extyoffs[0], (glcopy.extyoffs != nullptr) ? glcopy.extyoffs : glcopy.yoffset, glcopy.height * sizeof(u32));

			// always use the extended offsets here
			glcopy.extxoffs = &extxoffs[0];
			glcopy.extyoffs = &extyoffs[0];

			// expand X and Y by the scale factors
			if (xscale > 1)
			{
				glcopy.width *= xscale;
				for (int j = glcopy.width - 1; j >= 0; j--)
					extxoffs[j] = extxoffs[j / xscale];
			}
			if (yscale > 1)
			{
				glcopy.height *= yscale;
				for (int j = glcopy.height - 1; j >= 0; j--)
					extyoffs[j] = extyoffs[j / yscale];
			}

			// loop over all the planes, converting fractions
			for (int j = 0; j < glcopy.planes; j++)
			{
				u32 value1 = glcopy.planeoffset[j];
				if (IS_FRAC(value1))
				{
					assert(region_length != 0);
					glcopy.planeoffset[j] = FRAC_OFFSET(value1) + region_length * FRAC_NUM(value1) / FRAC_DEN(value1);
				}
			}

			// loop over all the X/Y offsets, converting fractions
			for (int j = 0; j < glcopy.width; j++)
			{
				u32 value2 = extxoffs[j];
				if (IS_FRAC(value2))
				{
					assert(region_length != 0);
					extxoffs[j] = FRAC_OFFSET(value2) + region_length * FRAC_NUM(value2) / FRAC_DEN(value2);
				}
			}

			for (int j = 0; j < glcopy.height; j++)
			{
				u32 value3 = extyoffs[j];
				if (IS_FRAC(value3))
				{
					assert(region_length != 0);
					extyoffs[j] = FRAC_OFFSET(value3) + region_length * FRAC_NUM(value3) / FRAC_DEN(value3);
				}
			}
		}

		// otherwise, just use the line modulo
		else
		{
			int base = gfx.start;
			int end = region_length/8;
			int linemod = glcopy.yoffset[0];
			while (glcopy.total > 0)
			{
				int elementbase = base + (glcopy.total - 1) * glcopy.charincrement / 8;
				int lastpixelbase = elementbase + glcopy.height * linemod / 8 - 1;
				if (lastpixelbase < end)
					break;
				glcopy.total--;
			}
		}

		// allocate the graphics
		m_gfx[curgfx] = std::make_unique<gfx_element>(m_palette, glcopy, (region_base != nullptr) ? region_base + gfx.start : nullptr, xormask, gfx.total_color_codes, gfx.color_codes_start);
	}

	m_decoded = true;
}


//-------------------------------------------------
//  interface_validity_check - validate graphics
//  decoding configuration
//-------------------------------------------------

void device_gfx_interface::interface_validity_check(validity_checker &valid) const
{
	if (!m_palette_is_disabled)
	{
		// validate palette tag
		if (m_palette_tag == nullptr)
			osd_printf_error("No palette specified for device '%s'\n", device().tag());
		else
		{
			device_t *paldev;
			if (m_palette_is_sibling)
				paldev = device().owner()->subdevice(m_palette_tag);
			else
				paldev = device().subdevice(m_palette_tag);
			if (paldev == nullptr)
				osd_printf_error("Nonexistent %sdevice '%s' specified as palette\n",
								 (m_palette_is_sibling ? "sibling " : "sub"),
								 m_palette_tag);
			else
				{
					device_palette_interface *palintf;
					if (!paldev->interface(palintf))
						osd_printf_error("%sdevice '%s' specified as palette, but it has no palette interface\n",
										 (m_palette_is_sibling ? "Sibling " : "Sub"),
										 m_palette_tag);
				}
		}
	}

	if (!m_gfxdecodeinfo)
		return;

	// validate graphics decoding entries
	for (int gfxnum = 0; gfxnum < MAX_GFX_ELEMENTS && m_gfxdecodeinfo[gfxnum].gfxlayout != nullptr; gfxnum++)
	{
		const gfx_decode_entry &gfx = m_gfxdecodeinfo[gfxnum];
		const gfx_layout &layout = *gfx.gfxlayout;

		// currently we are unable to validate RAM-based entries
		const char *region = gfx.memory_region;
		if (region != nullptr && GFXENTRY_ISROM(gfx.flags))
		{
			// resolve the region
			std::string gfxregion;
			if (GFXENTRY_ISDEVICE(gfx.flags))
				gfxregion = device().subtag(region);
			else
				gfxregion = device().owner()->subtag(region);

			u32 region_length = valid.region_length(gfxregion.c_str());
			if (region_length == 0)
				osd_printf_error("gfx[%d] references nonexistent region '%s'\n", gfxnum, gfxregion.c_str());

			// if we have a valid region, and we're not using auto-sizing, check the decode against the region length
			else if (!IS_FRAC(layout.total))
			{
				// determine which plane is at the largest offset
				int start = 0;
				for (int plane = 0; plane < layout.planes; plane++)
					if (layout.planeoffset[plane] > start)
						start = layout.planeoffset[plane];
				start &= ~(layout.charincrement - 1);

				// determine the total length based on this info
				int len = layout.total * layout.charincrement;

				// do we have enough space in the region to cover the whole decode?
				int avail = region_length - (gfx.start & ~(layout.charincrement / 8 - 1));

				// if not, this is an error
				if ((start + len) / 8 > avail)
					osd_printf_error("gfx[%d] extends past allocated memory of region '%s'\n", gfxnum, region);
			}
		}

		int xscale = GFXENTRY_GETXSCALE(gfx.flags);
		int yscale = GFXENTRY_GETYSCALE(gfx.flags);

		// verify raw decode, which can only be full-region and have no scaling
		if (layout.planeoffset[0] == GFX_RAW)
		{
			if (layout.total != RGN_FRAC(1,1))
				osd_printf_error("gfx[%d] RAW layouts can only be RGN_FRAC(1,1)\n", gfxnum);
			if (xscale != 1 || yscale != 1)
				osd_printf_error("gfx[%d] RAW layouts do not support xscale/yscale\n", gfxnum);
		}

		// verify traditional decode doesn't have too many planes,
		// and has extended offset arrays if its width and/or height demand them
		else
		{
			if (layout.planes > MAX_GFX_PLANES)
				osd_printf_error("gfx[%d] planes > %d\n", gfxnum, MAX_GFX_PLANES);
			if (layout.width > MAX_GFX_SIZE && layout.extxoffs == nullptr)
				osd_printf_error("gfx[%d] width > %d but missing extended xoffset info\n", gfxnum, MAX_GFX_SIZE);
			if (layout.height > MAX_GFX_SIZE && layout.extyoffs == nullptr)
				osd_printf_error("gfx[%d] height > %d but missing extended yoffset info\n", gfxnum, MAX_GFX_SIZE);
		}
	}
}
