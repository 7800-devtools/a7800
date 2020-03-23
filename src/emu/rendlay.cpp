// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/***************************************************************************

    rendlay.c

    Core rendering layout parser and manager.

****************************************************************************

    Overview of objects:

        layout_file -- A layout_file comprises a list of elements and a
            list of views. The elements are reusable items that the views
            reference.

        layout_view -- A layout_view describes a single view within a
            layout_file. The view is described using arbitrary coordinates
            that are scaled to fit within the render target. Pixels within
            a view are assumed to be square.

        view_item -- Each view has four lists of view_items, one for each
            "layer." Each view item is specified using floating point
            coordinates in arbitrary units, and is assumed to have square
            pixels. Each view item can control its orientation independently.
            Each item can also have an optional name, and can be set at
            runtime into different "states", which control how the embedded
            elements are displayed.

        layout_element -- A layout_element is a description of a piece of
            visible artwork. Most view_items (except for those in the screen
            layer) have exactly one layout_element which describes the
            contents of the item. Elements are separate from items because
            they can be re-used multiple times within a layout. Even though
            an element can contain a number of components, they are treated
            as if they were a single bitmap.

        element_component -- Each layout_element contains one or more
            components. Each component can describe either an image or
            a rectangle/disk primitive. Each component also has a "state"
            associated with it, which controls whether or not the component
            is visible (if the owning item has the same state, it is
            visible).

***************************************************************************/

#include <ctype.h>

#include "emu.h"
#include "emuopts.h"
#include "render.h"
#include "rendfont.h"
#include "rendlay.h"
#include "rendutil.h"
#include "xmlfile.h"



/***************************************************************************
    STANDARD LAYOUTS
***************************************************************************/

// screenless layouts
#include "noscreens.lh"

// single screen layouts
#include "horizont.lh"
#include "vertical.lh"

// dual screen layouts
#include "dualhsxs.lh"
#include "dualhovu.lh"
#include "dualhuov.lh"

// triple screen layouts
#include "triphsxs.lh"

// quad screen layouts
#include "quadhsxs.lh"

// LCD screen layouts
#include "lcd.lh"
#include "lcd_rot.lh"

// SVG screen layouts
#include "svg.lh"


//**************************************************************************
//  CONSTANTS
//**************************************************************************

const int LAYOUT_VERSION = 2;

enum
{
	LINE_CAP_NONE = 0,
	LINE_CAP_START = 1,
	LINE_CAP_END = 2
};



//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

render_screen_list render_target::s_empty_screen_list;



//**************************************************************************
//  INLINE HELPERS
//**************************************************************************

//-------------------------------------------------
//  gcd - compute the greatest common divisor (GCD)
//  of two integers using the Euclidean algorithm
//-------------------------------------------------

inline int gcd(int a, int b)
{
	while (b != 0)
	{
		int t = b;
		b = a % b;
		a = t;
	}
	return a;
}


//-------------------------------------------------
//  reduce_fraction - reduce a fraction by
//  dividing out common factors
//-------------------------------------------------

inline void reduce_fraction(int &num, int &den)
{
	// search the greatest common divisor
	int div = gcd(num, den);

	// reduce the fraction if a common divisor has been found
	if (div > 1)
	{
		num /= div;
		den /= div;
	}
}



//**************************************************************************
//  SHARED PARSING HELPERS
//**************************************************************************

//-------------------------------------------------
//  get_variable_value - compute the value of
//  a variable in an XML attribute
//-------------------------------------------------

static int get_variable_value(running_machine &machine, const char *string, char **outputptr)
{
	char temp[100];

	// screen 0 parameters
	int scrnum = 0;
	for (const screen_device &device : screen_device_iterator(machine.root_device()))
	{
		// native X aspect factor
		sprintf(temp, "~scr%dnativexaspect~", scrnum);
		if (!strncmp(string, temp, strlen(temp)))
		{
			int num = device.visible_area().width();
			int den = device.visible_area().height();
			reduce_fraction(num, den);
			*outputptr += sprintf(*outputptr, "%d", num);
			return strlen(temp);
		}

		// native Y aspect factor
		sprintf(temp, "~scr%dnativeyaspect~", scrnum);
		if (!strncmp(string, temp, strlen(temp)))
		{
			int num = device.visible_area().width();
			int den = device.visible_area().height();
			reduce_fraction(num, den);
			*outputptr += sprintf(*outputptr, "%d", den);
			return strlen(temp);
		}

		// native width
		sprintf(temp, "~scr%dwidth~", scrnum);
		if (!strncmp(string, temp, strlen(temp)))
		{
			*outputptr += sprintf(*outputptr, "%d", device.visible_area().width());
			return strlen(temp);
		}

		// native height
		sprintf(temp, "~scr%dheight~", scrnum);
		if (!strncmp(string, temp, strlen(temp)))
		{
			*outputptr += sprintf(*outputptr, "%d", device.visible_area().height());
			return strlen(temp);
		}

		// keep count
		scrnum++;
	}

	// default: copy the first character and continue
	**outputptr = *string;
	*outputptr += 1;
	return 1;
}


//-------------------------------------------------
//  xml_get_attribute_string_with_subst - analog
//  to xml_get_attribute_string but with variable
//  substitution
//-------------------------------------------------

static const char *xml_get_attribute_string_with_subst(running_machine &machine, util::xml::data_node const &node, const char *attribute, const char *defvalue)
{
	const char *str = node.get_attribute_string(attribute, nullptr);
	static char buffer[1000];

	// if nothing, just return the default
	if (str == nullptr)
		return defvalue;

	// if no tildes, don't worry
	if (strchr(str, '~') == nullptr)
		return str;

	// make a copy of the string, doing substitutions along the way
	const char *s;
	char *d;
	for (s = str, d = buffer; *s != 0; )
	{
		// if not a variable, just copy
		if (*s != '~')
			*d++ = *s++;

		// extract the variable
		else
			s += get_variable_value(machine, s, &d);
	}
	*d = 0;
	return buffer;
}


//-------------------------------------------------
//  xml_get_attribute_int_with_subst - analog
//  to xml_get_attribute_int but with variable
//  substitution
//-------------------------------------------------

static int xml_get_attribute_int_with_subst(running_machine &machine, util::xml::data_node const &node, const char *attribute, int defvalue)
{
	const char *string = xml_get_attribute_string_with_subst(machine, node, attribute, nullptr);
	int value;
	unsigned int uvalue;

	if (string == nullptr)
		return defvalue;
	if (string[0] == '$')
		return (sscanf(&string[1], "%X", &uvalue) == 1) ? uvalue : defvalue;
	if (string[0] == '0' && string[1] == 'x')
		return (sscanf(&string[2], "%X", &uvalue) == 1) ? uvalue : defvalue;
	if (string[0] == '#')
		return (sscanf(&string[1], "%d", &value) == 1) ? value : defvalue;
	return (sscanf(&string[0], "%d", &value) == 1) ? value : defvalue;
}


//-------------------------------------------------
//  xml_get_attribute_float_with_subst - analog
//  to xml_get_attribute_float but with variable
//  substitution
//-------------------------------------------------

static float xml_get_attribute_float_with_subst(running_machine &machine, util::xml::data_node const &node, const char *attribute, float defvalue)
{
	const char *string = xml_get_attribute_string_with_subst(machine, node, attribute, nullptr);
	float value;

	if (string == nullptr || sscanf(string, "%f", &value) != 1)
		return defvalue;
	return value;
}


//-------------------------------------------------
//  parse_bounds - parse a bounds XML node
//-------------------------------------------------

void parse_bounds(running_machine &machine, util::xml::data_node const *boundsnode, render_bounds &bounds)
{
	// skip if nothing
	if (boundsnode == nullptr)
	{
		bounds.x0 = bounds.y0 = 0.0f;
		bounds.x1 = bounds.y1 = 1.0f;
		return;
	}

	// parse out the data
	if (boundsnode->has_attribute("left"))
	{
		// left/right/top/bottom format
		bounds.x0 = xml_get_attribute_float_with_subst(machine, *boundsnode, "left", 0.0f);
		bounds.x1 = xml_get_attribute_float_with_subst(machine, *boundsnode, "right", 1.0f);
		bounds.y0 = xml_get_attribute_float_with_subst(machine, *boundsnode, "top", 0.0f);
		bounds.y1 = xml_get_attribute_float_with_subst(machine, *boundsnode, "bottom", 1.0f);
	}
	else if (boundsnode->has_attribute("x"))
	{
		// x/y/width/height format
		bounds.x0 = xml_get_attribute_float_with_subst(machine, *boundsnode, "x", 0.0f);
		bounds.x1 = bounds.x0 + xml_get_attribute_float_with_subst(machine, *boundsnode, "width", 1.0f);
		bounds.y0 = xml_get_attribute_float_with_subst(machine, *boundsnode, "y", 0.0f);
		bounds.y1 = bounds.y0 + xml_get_attribute_float_with_subst(machine, *boundsnode, "height", 1.0f);
	}
	else
		throw emu_fatalerror("Illegal bounds value in XML");

	// check for errors
	if (bounds.x0 > bounds.x1 || bounds.y0 > bounds.y1)
		throw emu_fatalerror("Illegal bounds value in XML: (%f-%f)-(%f-%f)",
				(double) bounds.x0, (double) bounds.x1, (double) bounds.y0, (double) bounds.y1);
}


//-------------------------------------------------
//  parse_color - parse a color XML node
//-------------------------------------------------

void parse_color(running_machine &machine, util::xml::data_node const *colornode, render_color &color)
{
	// skip if nothing
	if (colornode == nullptr)
	{
		color.r = color.g = color.b = color.a = 1.0f;
		return;
	}

	// parse out the data
	color.r = xml_get_attribute_float_with_subst(machine, *colornode, "red", 1.0);
	color.g = xml_get_attribute_float_with_subst(machine, *colornode, "green", 1.0);
	color.b = xml_get_attribute_float_with_subst(machine, *colornode, "blue", 1.0);
	color.a = xml_get_attribute_float_with_subst(machine, *colornode, "alpha", 1.0);

	// check for errors
	if (color.r < 0.0f || color.r > 1.0f || color.g < 0.0f || color.g > 1.0f ||
		color.b < 0.0f || color.b > 1.0f || color.a < 0.0f || color.a > 1.0f)
		throw emu_fatalerror("Illegal ARGB color value in XML: %f,%f,%f,%f",
				(double) color.r, (double) color.g, (double) color.b, (double) color.a);
}


//-------------------------------------------------
//  parse_orientation - parse an orientation XML
//  node
//-------------------------------------------------

static void parse_orientation(running_machine &machine, util::xml::data_node const *orientnode, int &orientation)
{
	// skip if nothing
	if (orientnode == nullptr)
	{
		orientation = ROT0;
		return;
	}

	// parse out the data
	int rotate = xml_get_attribute_int_with_subst(machine, *orientnode, "rotate", 0);
	switch (rotate)
	{
		case 0:     orientation = ROT0;     break;
		case 90:    orientation = ROT90;    break;
		case 180:   orientation = ROT180;   break;
		case 270:   orientation = ROT270;   break;
		default:    throw emu_fatalerror("Invalid rotation in XML orientation node: %d", rotate);
	}
	if (strcmp("yes", xml_get_attribute_string_with_subst(machine, *orientnode, "swapxy", "no")) == 0)
		orientation ^= ORIENTATION_SWAP_XY;
	if (strcmp("yes", xml_get_attribute_string_with_subst(machine, *orientnode, "flipx", "no")) == 0)
		orientation ^= ORIENTATION_FLIP_X;
	if (strcmp("yes", xml_get_attribute_string_with_subst(machine, *orientnode, "flipy", "no")) == 0)
		orientation ^= ORIENTATION_FLIP_Y;
}



//**************************************************************************
//  LAYOUT ELEMENT
//**************************************************************************

layout_element::make_component_map const layout_element::s_make_component{
	{ "image",         &make_component<image_component>         },
	{ "text",          &make_component<text_component>          },
	{ "dotmatrix",     &make_dotmatrix_component<8>             },
	{ "dotmatrix5dot", &make_dotmatrix_component<5>             },
	{ "dotmatrixdot",  &make_dotmatrix_component<1>             },
	{ "simplecounter", &make_component<simplecounter_component> },
	{ "reel",          &make_component<reel_component>          },
	{ "led7seg",       &make_component<led7seg_component>       },
	{ "led8seg_gts1",  &make_component<led8seg_gts1_component>  },
	{ "led14seg",      &make_component<led14seg_component>      },
	{ "led14segsc",    &make_component<led14segsc_component>    },
	{ "led16seg",      &make_component<led16seg_component>      },
	{ "led16segsc",    &make_component<led16segsc_component>    },
	{ "rect",          &make_component<rect_component>          },
	{ "disk",          &make_component<disk_component>          }
};

//-------------------------------------------------
//  layout_element - constructor
//-------------------------------------------------

layout_element::layout_element(running_machine &machine, util::xml::data_node const &elemnode, const char *dirname)
	: m_next(nullptr),
		m_machine(machine),
		m_defstate(0),
		m_maxstate(0)
{
	// extract the name
	const char *name = xml_get_attribute_string_with_subst(machine, elemnode, "name", nullptr);
	if (name == nullptr)
		throw emu_fatalerror("All layout elements must have a name!\n");
	m_name = name;

	// get the default state
	m_defstate = xml_get_attribute_int_with_subst(machine, elemnode, "defstate", -1);

	// parse components in order
	bool first = true;
	render_bounds bounds = { 0 };
	for (util::xml::data_node const *compnode = elemnode.get_first_child(); compnode; compnode = compnode->get_next_sibling())
	{
		make_component_map::const_iterator const make_func(s_make_component.find(compnode->get_name()));
		if (make_func == s_make_component.end())
			throw emu_fatalerror("Unknown element component: %s", compnode->get_name());

		// insert the new component into the list
		component const &newcomp(**m_complist.emplace(m_complist.end(), make_func->second(machine, *compnode, dirname)));

		// accumulate bounds
		if (first)
			bounds = newcomp.bounds();
		else
			union_render_bounds(&bounds, &newcomp.bounds());
		first = false;

		// determine the maximum state
		m_maxstate = std::max(m_maxstate, newcomp.maxstate());
	}

	if (!m_complist.empty())
	{
		// determine the scale/offset for normalization
		float xoffs = bounds.x0;
		float yoffs = bounds.y0;
		float xscale = 1.0f / (bounds.x1 - bounds.x0);
		float yscale = 1.0f / (bounds.y1 - bounds.y0);

		// normalize all the component bounds
		for (component::ptr const &curcomp : m_complist)
			curcomp->normalize_bounds(xoffs, yoffs, xscale, yscale);
	}

	// allocate an array of element textures for the states
	m_elemtex.resize(m_maxstate + 1);
}


//-------------------------------------------------
//  ~layout_element - destructor
//-------------------------------------------------

layout_element::~layout_element()
{
}


//-------------------------------------------------
//  state_texture - return a pointer to a
//  render_texture for the given state, allocating
//  one if needed
//-------------------------------------------------

render_texture *layout_element::state_texture(int state)
{
	assert(state <= m_maxstate);
	if (m_elemtex[state].m_texture == nullptr)
	{
		m_elemtex[state].m_element = this;
		m_elemtex[state].m_state = state;
		m_elemtex[state].m_texture = machine().render().texture_alloc(element_scale, &m_elemtex[state]);
	}
	return m_elemtex[state].m_texture;
}


//-------------------------------------------------
//  element_scale - scale an element by rendering
//  all the components at the appropriate
//  resolution
//-------------------------------------------------

void layout_element::element_scale(bitmap_argb32 &dest, bitmap_argb32 &source, const rectangle &sbounds, void *param)
{
	texture *elemtex = (texture *)param;

	// iterate over components that are part of the current state
	for (auto &curcomp : elemtex->m_element->m_complist)
		if (curcomp->state() == -1 || curcomp->state() == elemtex->m_state)
		{
			// get the local scaled bounds
			rectangle bounds;
			bounds.min_x = render_round_nearest(curcomp->bounds().x0 * dest.width());
			bounds.min_y = render_round_nearest(curcomp->bounds().y0 * dest.height());
			bounds.max_x = render_round_nearest(curcomp->bounds().x1 * dest.width());
			bounds.max_y = render_round_nearest(curcomp->bounds().y1 * dest.height());
			bounds &= dest.cliprect();

			// based on the component type, add to the texture
			curcomp->draw(elemtex->m_element->machine(), dest, bounds, elemtex->m_state);
		}
}


//-------------------------------------------------
//  make_component - create component of given type
//-------------------------------------------------

template <typename T>
layout_element::component::ptr layout_element::make_component(running_machine &machine, util::xml::data_node const &compnode, const char *dirname)
{
	return std::make_unique<T>(machine, compnode, dirname);
}


//-------------------------------------------------
//  make_component - create dotmatrix component
//  with given vertical resolution
//-------------------------------------------------

template <int D>
layout_element::component::ptr layout_element::make_dotmatrix_component(running_machine &machine, util::xml::data_node const &compnode, const char *dirname)
{
	return std::make_unique<dotmatrix_component>(D, machine, compnode, dirname);
}



//**************************************************************************
//  LAYOUT ELEMENT TEXTURE
//**************************************************************************

//-------------------------------------------------
//  texture - constructors
//-------------------------------------------------

layout_element::texture::texture()
	: m_element(nullptr)
	, m_texture(nullptr)
	, m_state(0)
{
}


layout_element::texture::texture(texture &&that) : texture()
{
	operator=(std::move(that));
}


//-------------------------------------------------
//  ~texture - destructor
//-------------------------------------------------

layout_element::texture::~texture()
{
	if (m_element != nullptr)
		m_element->machine().render().texture_free(m_texture);
}


//-------------------------------------------------
//  opearator= - move assignment
//-------------------------------------------------

layout_element::texture &layout_element::texture::operator=(texture &&that)
{
	using std::swap;
	swap(m_element, that.m_element);
	swap(m_texture, that.m_texture);
	swap(m_state, that.m_state);
	return *this;
}



//**************************************************************************
//  LAYOUT ELEMENT COMPONENT
//**************************************************************************

//-------------------------------------------------
//  component - constructor
//-------------------------------------------------

layout_element::component::component(running_machine &machine, util::xml::data_node const &compnode, const char *dirname)
	: m_state(0)
{
	// fetch common data
	m_state = xml_get_attribute_int_with_subst(machine, compnode, "state", -1);
	parse_bounds(machine, compnode.get_child("bounds"), m_bounds);
	parse_color(machine, compnode.get_child("color"), m_color);
}


//-------------------------------------------------
//  normalize_bounds - normalize component bounds
//-------------------------------------------------

void layout_element::component::normalize_bounds(float xoffs, float yoffs, float xscale, float yscale)
{
	m_bounds.x0 = (m_bounds.x0 - xoffs) * xscale;
	m_bounds.x1 = (m_bounds.x1 - xoffs) * xscale;
	m_bounds.y0 = (m_bounds.y0 - yoffs) * yscale;
	m_bounds.y1 = (m_bounds.y1 - yoffs) * yscale;
}


//-------------------------------------------------
//  image_component - constructor
//-------------------------------------------------

layout_element::image_component::image_component(running_machine &machine, util::xml::data_node const &compnode, const char *dirname)
	: component(machine, compnode, dirname)
	, m_hasalpha(false)
{
	if (dirname != nullptr)
		m_dirname = dirname;
	m_imagefile = xml_get_attribute_string_with_subst(machine, compnode, "file", "");
	m_alphafile = xml_get_attribute_string_with_subst(machine, compnode, "alphafile", "");
	m_file = std::make_unique<emu_file>(machine.options().art_path(), OPEN_FLAG_READ);
}


//-------------------------------------------------
//  draw - draw a component
//-------------------------------------------------

void layout_element::image_component::draw(running_machine &machine, bitmap_argb32 &dest, const rectangle &bounds, int state)
{
	if (!m_bitmap.valid())
		load_bitmap();

	bitmap_argb32 destsub(dest, bounds);
	render_resample_argb_bitmap_hq(destsub, m_bitmap, color());
}


//-------------------------------------------------
//  load_bitmap - load a PNG/JPG file with artwork for
//  a component
//-------------------------------------------------

void layout_element::image_component::load_bitmap()
{
	// load the basic bitmap
	assert(m_file != nullptr);
	m_hasalpha = render_load_png(m_bitmap, *m_file, m_dirname.c_str(), m_imagefile.c_str());

	// load the alpha bitmap if specified
	if (m_bitmap.valid() && !m_alphafile.empty())
		render_load_png(m_bitmap, *m_file, m_dirname.c_str(), m_alphafile.c_str(), true);

	// PNG failed, let's try JPG
	if (!m_bitmap.valid())
		render_load_jpeg(m_bitmap, *m_file, m_dirname.c_str(), m_imagefile.c_str());

	// if we can't load the bitmap, allocate a dummy one and report an error
	if (!m_bitmap.valid())
	{
		// draw some stripes in the bitmap
		m_bitmap.allocate(100, 100);
		m_bitmap.fill(0);
		for (int step = 0; step < 100; step += 25)
			for (int line = 0; line < 100; line++)
				m_bitmap.pix32((step + line) % 100, line % 100) = rgb_t(0xff,0xff,0xff,0xff);

		// log an error
		if (m_alphafile.empty())
			osd_printf_warning("Unable to load component bitmap '%s'\n", m_imagefile.c_str());
		else
			osd_printf_warning("Unable to load component bitmap '%s'/'%s'\n", m_imagefile.c_str(), m_alphafile.c_str());
	}
}


//-------------------------------------------------
//  text_component - constructor
//-------------------------------------------------

layout_element::text_component::text_component(running_machine &machine, util::xml::data_node const &compnode, const char *dirname)
	: component(machine, compnode, dirname)
{
	m_string = xml_get_attribute_string_with_subst(machine, compnode, "string", "");
	m_textalign = xml_get_attribute_int_with_subst(machine, compnode, "align", 0);
}


//-------------------------------------------------
//  draw - draw a component
//-------------------------------------------------

void layout_element::text_component::draw(running_machine &machine, bitmap_argb32 &dest, const rectangle &bounds, int state)
{
	render_font *font = machine.render().font_alloc("default");
	draw_text(*font, dest, bounds, m_string.c_str(), m_textalign);
	machine.render().font_free(font);
}


//-------------------------------------------------
//  dotmatrix_component - constructor
//-------------------------------------------------

layout_element::dotmatrix_component::dotmatrix_component(int dots, running_machine &machine, util::xml::data_node const &compnode, const char *dirname)
	: component(machine, compnode, dirname),
		m_dots(dots)
{
}


//-------------------------------------------------
//  draw - draw a component
//-------------------------------------------------

void layout_element::dotmatrix_component::draw(running_machine &machine, bitmap_argb32 &dest, const rectangle &bounds, int state)
{
	const rgb_t onpen = rgb_t(0xff, 0xff, 0xff, 0xff);
	const rgb_t offpen = rgb_t(0xff, 0x20, 0x20, 0x20);

	// sizes for computation
	int bmheight = 300;
	int dotwidth = 250;

	// allocate a temporary bitmap for drawing
	bitmap_argb32 tempbitmap(dotwidth*m_dots, bmheight);
	tempbitmap.fill(rgb_t(0xff, 0x00, 0x00, 0x00));

	for (int i = 0; i < m_dots; i++)
		draw_segment_decimal(tempbitmap, ((dotwidth/2 )+ (i * dotwidth)), bmheight/2, dotwidth, (state & (1 << i))?onpen:offpen);

	// resample to the target size
	render_resample_argb_bitmap_hq(dest, tempbitmap, color());
}


//-------------------------------------------------
//  simplecounter_component - constructor
//-------------------------------------------------

layout_element::simplecounter_component::simplecounter_component(running_machine &machine, util::xml::data_node const &compnode, const char *dirname)
	: component(machine, compnode, dirname)
{
	m_digits = xml_get_attribute_int_with_subst(machine, compnode, "digits", 2);
	m_textalign = xml_get_attribute_int_with_subst(machine, compnode, "align", 0);
	m_maxstate = xml_get_attribute_int_with_subst(machine, compnode, "maxstate", 999);
}


//-------------------------------------------------
//  draw - draw a component
//-------------------------------------------------

void layout_element::simplecounter_component::draw(running_machine &machine, bitmap_argb32 &dest, const rectangle &bounds, int state)
{
	render_font *font = machine.render().font_alloc("default");
	std::string temp = string_format("%0*d", m_digits, state);
	draw_text(*font, dest, bounds, temp.c_str(), m_textalign);
	machine.render().font_free(font);
}


//-------------------------------------------------
//  reel_component - constructor
//-------------------------------------------------

layout_element::reel_component::reel_component(running_machine &machine, util::xml::data_node const &compnode, const char *dirname)
	: component(machine, compnode, dirname)
{
	for (auto & elem : m_hasalpha)
		elem = false;

	std::string symbollist = xml_get_attribute_string_with_subst(machine, compnode, "symbollist", "0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15");

	// split out position names from string and figure out our number of symbols
	int location;
	m_numstops = 0;
	location=symbollist.find(",");
	while (location!=-1)
	{
		m_stopnames[m_numstops] = symbollist;
		m_stopnames[m_numstops] = m_stopnames[m_numstops].substr(0, location);
		symbollist = symbollist.substr(location+1, symbollist.length()-(location-1));
		m_numstops++;
		location=symbollist.find(",");
	}
	m_stopnames[m_numstops++] = symbollist;

	// careful, dirname is nullptr if we're coming from internal layout, and our string assignment doesn't like that
	if (dirname != nullptr)
		m_dirname = dirname;

	for (int i=0;i<m_numstops;i++)
	{
		location=m_stopnames[i].find(":");
		if (location!=-1)
		{
			m_imagefile[i] = m_stopnames[i];
			m_stopnames[i] = m_stopnames[i].substr(0, location);
			m_imagefile[i] = m_imagefile[i].substr(location+1, m_imagefile[i].length()-(location-1));

			//m_alphafile[i] =
			m_file[i] = std::make_unique<emu_file>(machine.options().art_path(), OPEN_FLAG_READ);
		}
		else
		{
			//m_imagefile[i] = 0;
			//m_alphafile[i] = 0;
			m_file[i].reset();
		}
	}

	m_stateoffset = xml_get_attribute_int_with_subst(machine, compnode, "stateoffset", 0);
	m_numsymbolsvisible = xml_get_attribute_int_with_subst(machine, compnode, "numsymbolsvisible", 3);
	m_reelreversed = xml_get_attribute_int_with_subst(machine, compnode, "reelreversed", 0);
	m_beltreel = xml_get_attribute_int_with_subst(machine, compnode, "beltreel", 0);
}


//-------------------------------------------------
//  draw - draw a component
//-------------------------------------------------

/* state is a normalized value between 0 and 65536 so that we don't need to worry about how many motor steps here or in the .lay, only the number of symbols */
void layout_element::reel_component::draw(running_machine &machine, bitmap_argb32 &dest, const rectangle &bounds, int state)
{
	if (m_beltreel)
	{
		draw_beltreel(machine,dest,bounds,state);
		return;
	}

	const int max_state_used = 0x10000;

	// shift the reels a bit based on this param, allows fine tuning
	int use_state = (state + m_stateoffset) % max_state_used;

	// compute premultiplied colors
	u32 r = color().r * 255.0f;
	u32 g = color().g * 255.0f;
	u32 b = color().b * 255.0f;
	u32 a = color().a * 255.0f;

	// get the width of the string
	render_font *font = machine.render().font_alloc("default");
	float aspect = 1.0f;
	s32 width;

	int curry = 0;
	int num_shown = m_numsymbolsvisible;

	int ourheight = bounds.height();

	for (int fruit = 0;fruit<m_numstops;fruit++)
	{
		int basey;

		if (m_reelreversed==1)
		{
			basey = bounds.min_y + ((use_state)*(ourheight/num_shown)/(max_state_used/m_numstops)) + curry;
		}
		else
		{
			basey = bounds.min_y - ((use_state)*(ourheight/num_shown)/(max_state_used/m_numstops)) + curry;
		}

		// wrap around...
		if (basey < bounds.min_y)
			basey += ((max_state_used)*(ourheight/num_shown)/(max_state_used/m_numstops));
		if (basey > bounds.max_y)
			basey -= ((max_state_used)*(ourheight/num_shown)/(max_state_used/m_numstops));

		int endpos = basey+ourheight/num_shown;

		// only render the symbol / text if it's atually in view because the code is SLOW
		if ((endpos >= bounds.min_y) && (basey <= bounds.max_y))
		{
			while (1)
			{
				width = font->string_width(ourheight / num_shown, aspect, m_stopnames[fruit].c_str());
				if (width < bounds.width())
					break;
				aspect *= 0.9f;
			}

			s32 curx;
			curx = bounds.min_x + (bounds.width() - width) / 2;

			if (m_file[fruit])
				if (!m_bitmap[fruit].valid())
					load_reel_bitmap(fruit);

			if (m_file[fruit]) // render gfx
			{
				bitmap_argb32 tempbitmap2(dest.width(), ourheight/num_shown);

				if (m_bitmap[fruit].valid())
				{
					render_resample_argb_bitmap_hq(tempbitmap2, m_bitmap[fruit], color());

					for (int y = 0; y < ourheight/num_shown; y++)
					{
						int effy = basey + y;

						if (effy >= bounds.min_y && effy <= bounds.max_y)
						{
							u32 *src = &tempbitmap2.pix32(y);
							u32 *d = &dest.pix32(effy);
							for (int x = 0; x < dest.width(); x++)
							{
								int effx = x;
								if (effx >= bounds.min_x && effx <= bounds.max_x)
								{
									u32 spix = rgb_t(src[x]).a();
									if (spix != 0)
									{
										d[effx] = src[x];
									}
								}
							}
						}
					}
				}
			}
			else // render text (fallback)
			{
				// allocate a temporary bitmap
				bitmap_argb32 tempbitmap(dest.width(), dest.height());

				const char *origs = m_stopnames[fruit].c_str();
				const char *ends = origs + strlen(origs);
				const char *s = origs;
				char32_t schar;

				// loop over characters
				while (*s != 0)
				{
					int scharcount = uchar_from_utf8(&schar, s, ends - s);

					if (scharcount == -1)
						break;

					// get the font bitmap
					rectangle chbounds;
					font->get_scaled_bitmap_and_bounds(tempbitmap, ourheight/num_shown, aspect, schar, chbounds);

					// copy the data into the target
					for (int y = 0; y < chbounds.height(); y++)
					{
						int effy = basey + y;

						if (effy >= bounds.min_y && effy <= bounds.max_y)
						{
							u32 *src = &tempbitmap.pix32(y);
							u32 *d = &dest.pix32(effy);
							for (int x = 0; x < chbounds.width(); x++)
							{
								int effx = curx + x + chbounds.min_x;
								if (effx >= bounds.min_x && effx <= bounds.max_x)
								{
									u32 spix = rgb_t(src[x]).a();
									if (spix != 0)
									{
										rgb_t dpix = d[effx];
										u32 ta = (a * (spix + 1)) >> 8;
										u32 tr = (r * ta + dpix.r() * (0x100 - ta)) >> 8;
										u32 tg = (g * ta + dpix.g() * (0x100 - ta)) >> 8;
										u32 tb = (b * ta + dpix.b() * (0x100 - ta)) >> 8;
										d[effx] = rgb_t(tr, tg, tb);
									}
								}
							}
						}
					}

					// advance in the X direction
					curx += font->char_width(ourheight/num_shown, aspect, schar);
					s += scharcount;
				}
			}
		}

		curry += ourheight/num_shown;
	}

	// free the temporary bitmap and font
	machine.render().font_free(font);
}


void layout_element::reel_component::draw_beltreel(running_machine &machine, bitmap_argb32 &dest, const rectangle &bounds, int state)
{
	const int max_state_used = 0x10000;

	// shift the reels a bit based on this param, allows fine tuning
	int use_state = (state + m_stateoffset) % max_state_used;

	// compute premultiplied colors
	u32 r = color().r * 255.0f;
	u32 g = color().g * 255.0f;
	u32 b = color().b * 255.0f;
	u32 a = color().a * 255.0f;

	// get the width of the string
	render_font *font = machine.render().font_alloc("default");
	float aspect = 1.0f;
	s32 width;
	int currx = 0;
	int num_shown = m_numsymbolsvisible;

	int ourwidth = bounds.width();

	for (int fruit = 0;fruit<m_numstops;fruit++)
	{
		int basex;
		if (m_reelreversed==1)
		{
			basex = bounds.min_x + ((use_state)*(ourwidth/num_shown)/(max_state_used/m_numstops)) + currx;
		}
		else
		{
			basex = bounds.min_x - ((use_state)*(ourwidth/num_shown)/(max_state_used/m_numstops)) + currx;
		}

		// wrap around...
		if (basex < bounds.min_x)
			basex += ((max_state_used)*(ourwidth/num_shown)/(max_state_used/m_numstops));
		if (basex > bounds.max_x)
			basex -= ((max_state_used)*(ourwidth/num_shown)/(max_state_used/m_numstops));

		int endpos = basex+(ourwidth/num_shown);

		// only render the symbol / text if it's atually in view because the code is SLOW
		if ((endpos >= bounds.min_x) && (basex <= bounds.max_x))
		{
			while (1)
			{
				width = font->string_width(dest.height(), aspect, m_stopnames[fruit].c_str());
				if (width < bounds.width())
					break;
				aspect *= 0.9f;
			}

			s32 curx;
			curx = bounds.min_x;

			if (m_file[fruit])
				if (!m_bitmap[fruit].valid())
					load_reel_bitmap(fruit);

			if (m_file[fruit]) // render gfx
			{
				bitmap_argb32 tempbitmap2(ourwidth/num_shown, dest.height());

				if (m_bitmap[fruit].valid())
				{
					render_resample_argb_bitmap_hq(tempbitmap2, m_bitmap[fruit], color());

					for (int y = 0; y < dest.height(); y++)
					{
						int effy = y;

						if (effy >= bounds.min_y && effy <= bounds.max_y)
						{
							u32 *src = &tempbitmap2.pix32(y);
							u32 *d = &dest.pix32(effy);
							for (int x = 0; x < ourwidth/num_shown; x++)
							{
								int effx = basex + x;
								if (effx >= bounds.min_x && effx <= bounds.max_x)
								{
									u32 spix = rgb_t(src[x]).a();
									if (spix != 0)
									{
										d[effx] = src[x];
									}
								}
							}
						}

					}
				}
			}
			else // render text (fallback)
			{
				// allocate a temporary bitmap
				bitmap_argb32 tempbitmap(dest.width(), dest.height());


				const char *origs =m_stopnames[fruit].c_str();
				const char *ends = origs + strlen(origs);
				const char *s = origs;
				char32_t schar;

				// loop over characters
				while (*s != 0)
				{
					int scharcount = uchar_from_utf8(&schar, s, ends - s);

					if (scharcount == -1)
						break;

					// get the font bitmap
					rectangle chbounds;
					font->get_scaled_bitmap_and_bounds(tempbitmap, dest.height(), aspect, schar, chbounds);

					// copy the data into the target
					for (int y = 0; y < chbounds.height(); y++)
					{
						int effy = y;

						if (effy >= bounds.min_y && effy <= bounds.max_y)
						{
							u32 *src = &tempbitmap.pix32(y);
							u32 *d = &dest.pix32(effy);
							for (int x = 0; x < chbounds.width(); x++)
							{
								int effx = basex + curx + x;
								if (effx >= bounds.min_x && effx <= bounds.max_x)
								{
									u32 spix = rgb_t(src[x]).a();
									if (spix != 0)
									{
										rgb_t dpix = d[effx];
										u32 ta = (a * (spix + 1)) >> 8;
										u32 tr = (r * ta + dpix.r() * (0x100 - ta)) >> 8;
										u32 tg = (g * ta + dpix.g() * (0x100 - ta)) >> 8;
										u32 tb = (b * ta + dpix.b() * (0x100 - ta)) >> 8;
										d[effx] = rgb_t(tr, tg, tb);
									}
								}
							}
						}
					}

					// advance in the X direction
					curx += font->char_width(dest.height(), aspect, schar);
					s += scharcount;
				}
			}
		}

		currx += ourwidth/num_shown;
	}

	// free the temporary bitmap and font
	machine.render().font_free(font);
}


void layout_element::reel_component::load_reel_bitmap(int number)
{
	// load the basic bitmap
	assert(m_file != nullptr);
	/*m_hasalpha[number] = */ render_load_png(m_bitmap[number], *m_file[number], m_dirname.c_str(), m_imagefile[number].c_str());

	// load the alpha bitmap if specified
	//if (m_bitmap[number].valid() && m_alphafile[number])
	//  render_load_png(m_bitmap[number], *m_file[number], m_dirname, m_alphafile[number], true);

	// if we can't load the bitmap just use text rendering
	if (!m_bitmap[number].valid())
	{
		// fallback to text rendering
		m_file[number].reset();
	}

}


//-------------------------------------------------
//  led7seg_component - constructor
//-------------------------------------------------

layout_element::led7seg_component::led7seg_component(running_machine &machine, util::xml::data_node const &compnode, const char *dirname)
	: component(machine, compnode, dirname)
{
}


//-------------------------------------------------
//  draw - draw a component
//-------------------------------------------------

void layout_element::led7seg_component::draw(running_machine &machine, bitmap_argb32 &dest, const rectangle &bounds, int state)
{
	const rgb_t onpen = rgb_t(0xff,0xff,0xff,0xff);
	const rgb_t offpen = rgb_t(0xff,0x20,0x20,0x20);

	// sizes for computation
	int bmwidth = 250;
	int bmheight = 400;
	int segwidth = 40;
	int skewwidth = 40;

	// allocate a temporary bitmap for drawing
	bitmap_argb32 tempbitmap(bmwidth + skewwidth, bmheight);
	tempbitmap.fill(rgb_t(0xff,0x00,0x00,0x00));

	// top bar
	draw_segment_horizontal(tempbitmap, 0 + 2*segwidth/3, bmwidth - 2*segwidth/3, 0 + segwidth/2, segwidth, (state & (1 << 0)) ? onpen : offpen);

	// top-right bar
	draw_segment_vertical(tempbitmap, 0 + 2*segwidth/3, bmheight/2 - segwidth/3, bmwidth - segwidth/2, segwidth, (state & (1 << 1)) ? onpen : offpen);

	// bottom-right bar
	draw_segment_vertical(tempbitmap, bmheight/2 + segwidth/3, bmheight - 2*segwidth/3, bmwidth - segwidth/2, segwidth, (state & (1 << 2)) ? onpen : offpen);

	// bottom bar
	draw_segment_horizontal(tempbitmap, 0 + 2*segwidth/3, bmwidth - 2*segwidth/3, bmheight - segwidth/2, segwidth, (state & (1 << 3)) ? onpen : offpen);

	// bottom-left bar
	draw_segment_vertical(tempbitmap, bmheight/2 + segwidth/3, bmheight - 2*segwidth/3, 0 + segwidth/2, segwidth, (state & (1 << 4)) ? onpen : offpen);

	// top-left bar
	draw_segment_vertical(tempbitmap, 0 + 2*segwidth/3, bmheight/2 - segwidth/3, 0 + segwidth/2, segwidth, (state & (1 << 5)) ? onpen : offpen);

	// middle bar
	draw_segment_horizontal(tempbitmap, 0 + 2*segwidth/3, bmwidth - 2*segwidth/3, bmheight/2, segwidth, (state & (1 << 6)) ? onpen : offpen);

	// apply skew
	apply_skew(tempbitmap, 40);

	// decimal point
	draw_segment_decimal(tempbitmap, bmwidth + segwidth/2, bmheight - segwidth/2, segwidth, (state & (1 << 7)) ? onpen : offpen);

	// resample to the target size
	render_resample_argb_bitmap_hq(dest, tempbitmap, color());
}


//-------------------------------------------------
//  led8seg_gts1_component - constructor
//-------------------------------------------------

layout_element::led8seg_gts1_component::led8seg_gts1_component(running_machine &machine, util::xml::data_node const &compnode, const char *dirname)
	: component(machine, compnode, dirname)
{
}


//-------------------------------------------------
//  draw - draw a component
//-------------------------------------------------

void layout_element::led8seg_gts1_component::draw(running_machine &machine, bitmap_argb32 &dest, const rectangle &bounds, int state)
{
	const rgb_t onpen = rgb_t(0xff,0xff,0xff,0xff);
	const rgb_t offpen = rgb_t(0xff,0x20,0x20,0x20);
	const rgb_t backpen = rgb_t(0xff,0x00,0x00,0x00);

	// sizes for computation
	int bmwidth = 250;
	int bmheight = 400;
	int segwidth = 40;
	int skewwidth = 40;

	// allocate a temporary bitmap for drawing
	bitmap_argb32 tempbitmap(bmwidth + skewwidth, bmheight);
	tempbitmap.fill(backpen);

	// top bar
	draw_segment_horizontal(tempbitmap, 0 + 2*segwidth/3, bmwidth - 2*segwidth/3, 0 + segwidth/2, segwidth, (state & (1 << 0)) ? onpen : offpen);

	// top-right bar
	draw_segment_vertical(tempbitmap, 0 + 2*segwidth/3, bmheight/2 - segwidth/3, bmwidth - segwidth/2, segwidth, (state & (1 << 1)) ? onpen : offpen);

	// bottom-right bar
	draw_segment_vertical(tempbitmap, bmheight/2 + segwidth/3, bmheight - 2*segwidth/3, bmwidth - segwidth/2, segwidth, (state & (1 << 2)) ? onpen : offpen);

	// bottom bar
	draw_segment_horizontal(tempbitmap, 0 + 2*segwidth/3, bmwidth - 2*segwidth/3, bmheight - segwidth/2, segwidth, (state & (1 << 3)) ? onpen : offpen);

	// bottom-left bar
	draw_segment_vertical(tempbitmap, bmheight/2 + segwidth/3, bmheight - 2*segwidth/3, 0 + segwidth/2, segwidth, (state & (1 << 4)) ? onpen : offpen);

	// top-left bar
	draw_segment_vertical(tempbitmap, 0 + 2*segwidth/3, bmheight/2 - segwidth/3, 0 + segwidth/2, segwidth, (state & (1 << 5)) ? onpen : offpen);

	// horizontal bars
	draw_segment_horizontal(tempbitmap, 0 + 2*segwidth/3, 2*bmwidth/3 - 2*segwidth/3, bmheight/2, segwidth, (state & (1 << 6)) ? onpen : offpen);
	draw_segment_horizontal(tempbitmap, 0 + 2*segwidth/3 + bmwidth/2, bmwidth - 2*segwidth/3, bmheight/2, segwidth, (state & (1 << 6)) ? onpen : offpen);

	// vertical bars
	draw_segment_vertical(tempbitmap, 0 + segwidth/3 - 8, bmheight/2 - segwidth/3 + 2, 2*bmwidth/3 - segwidth/2 - 4, segwidth + 8, backpen);
	draw_segment_vertical(tempbitmap, 0 + segwidth/3, bmheight/2 - segwidth/3, 2*bmwidth/3 - segwidth/2 - 4, segwidth, (state & (1 << 7)) ? onpen : offpen);

	draw_segment_vertical(tempbitmap, bmheight/2 + segwidth/3 - 2, bmheight - segwidth/3 + 8, 2*bmwidth/3 - segwidth/2 - 4, segwidth + 8, backpen);
	draw_segment_vertical(tempbitmap, bmheight/2 + segwidth/3, bmheight - segwidth/3, 2*bmwidth/3 - segwidth/2 - 4, segwidth, (state & (1 << 7)) ? onpen : offpen);

	// apply skew
	apply_skew(tempbitmap, 40);

	// resample to the target size
	render_resample_argb_bitmap_hq(dest, tempbitmap, color());
}


//-------------------------------------------------
//  led14seg_component - constructor
//-------------------------------------------------

layout_element::led14seg_component::led14seg_component(running_machine &machine, util::xml::data_node const &compnode, const char *dirname)
	: component(machine, compnode, dirname)
{
}


//-------------------------------------------------
//  draw - draw a component
//-------------------------------------------------

void layout_element::led14seg_component::draw(running_machine &machine, bitmap_argb32 &dest, const rectangle &bounds, int state)
{
	const rgb_t onpen = rgb_t(0xff, 0xff, 0xff, 0xff);
	const rgb_t offpen = rgb_t(0xff, 0x20, 0x20, 0x20);

	// sizes for computation
	int bmwidth = 250;
	int bmheight = 400;
	int segwidth = 40;
	int skewwidth = 40;

	// allocate a temporary bitmap for drawing
	bitmap_argb32 tempbitmap(bmwidth + skewwidth, bmheight);
	tempbitmap.fill(rgb_t(0xff, 0x00, 0x00, 0x00));

	// top bar
	draw_segment_horizontal(tempbitmap,
		0 + 2*segwidth/3, bmwidth - 2*segwidth/3, 0 + segwidth/2,
		segwidth, (state & (1 << 0)) ? onpen : offpen);

	// right-top bar
	draw_segment_vertical(tempbitmap,
		0 + 2*segwidth/3, bmheight/2 - segwidth/3, bmwidth - segwidth/2,
		segwidth, (state & (1 << 1)) ? onpen : offpen);

	// right-bottom bar
	draw_segment_vertical(tempbitmap,
		bmheight/2 + segwidth/3, bmheight - 2*segwidth/3, bmwidth - segwidth/2,
		segwidth, (state & (1 << 2)) ? onpen : offpen);

	// bottom bar
	draw_segment_horizontal(tempbitmap,
		0 + 2*segwidth/3, bmwidth - 2*segwidth/3, bmheight - segwidth/2,
		segwidth, (state & (1 << 3)) ? onpen : offpen);

	// left-bottom bar
	draw_segment_vertical(tempbitmap,
		bmheight/2 + segwidth/3, bmheight - 2*segwidth/3, 0 + segwidth/2,
		segwidth, (state & (1 << 4)) ? onpen : offpen);

	// left-top bar
	draw_segment_vertical(tempbitmap,
		0 + 2*segwidth/3, bmheight/2 - segwidth/3, 0 + segwidth/2,
		segwidth, (state & (1 << 5)) ? onpen : offpen);

	// horizontal-middle-left bar
	draw_segment_horizontal_caps(tempbitmap,
		0 + 2*segwidth/3, bmwidth/2 - segwidth/10, bmheight/2,
		segwidth, LINE_CAP_START, (state & (1 << 6)) ? onpen : offpen);

	// horizontal-middle-right bar
	draw_segment_horizontal_caps(tempbitmap,
		0 + bmwidth/2 + segwidth/10, bmwidth - 2*segwidth/3, bmheight/2,
		segwidth, LINE_CAP_END, (state & (1 << 7)) ? onpen : offpen);

	// vertical-middle-top bar
	draw_segment_vertical_caps(tempbitmap,
		0 + segwidth + segwidth/3, bmheight/2 - segwidth/2 - segwidth/3, bmwidth/2,
		segwidth, LINE_CAP_NONE, (state & (1 << 8)) ? onpen : offpen);

	// vertical-middle-bottom bar
	draw_segment_vertical_caps(tempbitmap,
		bmheight/2 + segwidth/2 + segwidth/3, bmheight - segwidth - segwidth/3, bmwidth/2,
		segwidth, LINE_CAP_NONE, (state & (1 << 9)) ? onpen : offpen);

	// diagonal-left-bottom bar
	draw_segment_diagonal_1(tempbitmap,
		0 + segwidth + segwidth/5, bmwidth/2 - segwidth/2 - segwidth/5,
		bmheight/2 + segwidth/2 + segwidth/3, bmheight - segwidth - segwidth/3,
		segwidth, (state & (1 << 10)) ? onpen : offpen);

	// diagonal-left-top bar
	draw_segment_diagonal_2(tempbitmap,
		0 + segwidth + segwidth/5, bmwidth/2 - segwidth/2 - segwidth/5,
		0 + segwidth + segwidth/3, bmheight/2 - segwidth/2 - segwidth/3,
		segwidth, (state & (1 << 11)) ? onpen : offpen);

	// diagonal-right-top bar
	draw_segment_diagonal_1(tempbitmap,
		bmwidth/2 + segwidth/2 + segwidth/5, bmwidth - segwidth - segwidth/5,
		0 + segwidth + segwidth/3, bmheight/2 - segwidth/2 - segwidth/3,
		segwidth, (state & (1 << 12)) ? onpen : offpen);

	// diagonal-right-bottom bar
	draw_segment_diagonal_2(tempbitmap,
		bmwidth/2 + segwidth/2 + segwidth/5, bmwidth - segwidth - segwidth/5,
		bmheight/2 + segwidth/2 + segwidth/3, bmheight - segwidth - segwidth/3,
		segwidth, (state & (1 << 13)) ? onpen : offpen);

	// apply skew
	apply_skew(tempbitmap, 40);

	// resample to the target size
	render_resample_argb_bitmap_hq(dest, tempbitmap, color());
}


//-------------------------------------------------
//  led14segsc_component - constructor
//-------------------------------------------------

layout_element::led14segsc_component::led14segsc_component(running_machine &machine, util::xml::data_node const &compnode, const char *dirname)
	: component(machine, compnode, dirname)
{
}


//-------------------------------------------------
//  draw - draw a component
//-------------------------------------------------

void layout_element::led14segsc_component::draw(running_machine &machine, bitmap_argb32 &dest, const rectangle &bounds, int state)
{
	const rgb_t onpen = rgb_t(0xff, 0xff, 0xff, 0xff);
	const rgb_t offpen = rgb_t(0xff, 0x20, 0x20, 0x20);

	// sizes for computation
	int bmwidth = 250;
	int bmheight = 400;
	int segwidth = 40;
	int skewwidth = 40;

	// allocate a temporary bitmap for drawing, adding some extra space for the tail
	bitmap_argb32 tempbitmap(bmwidth + skewwidth, bmheight + segwidth);
	tempbitmap.fill(rgb_t(0xff, 0x00, 0x00, 0x00));

	// top bar
	draw_segment_horizontal(tempbitmap,
		0 + 2*segwidth/3, bmwidth - 2*segwidth/3, 0 + segwidth/2,
		segwidth, (state & (1 << 0)) ? onpen : offpen);

	// right-top bar
	draw_segment_vertical(tempbitmap,
		0 + 2*segwidth/3, bmheight/2 - segwidth/3, bmwidth - segwidth/2,
		segwidth, (state & (1 << 1)) ? onpen : offpen);

	// right-bottom bar
	draw_segment_vertical(tempbitmap,
		bmheight/2 + segwidth/3, bmheight - 2*segwidth/3, bmwidth - segwidth/2,
		segwidth, (state & (1 << 2)) ? onpen : offpen);

	// bottom bar
	draw_segment_horizontal(tempbitmap,
		0 + 2*segwidth/3, bmwidth - 2*segwidth/3, bmheight - segwidth/2,
		segwidth, (state & (1 << 3)) ? onpen : offpen);

	// left-bottom bar
	draw_segment_vertical(tempbitmap,
		bmheight/2 + segwidth/3, bmheight - 2*segwidth/3, 0 + segwidth/2,
		segwidth, (state & (1 << 4)) ? onpen : offpen);

	// left-top bar
	draw_segment_vertical(tempbitmap,
		0 + 2*segwidth/3, bmheight/2 - segwidth/3, 0 + segwidth/2,
		segwidth, (state & (1 << 5)) ? onpen : offpen);

	// horizontal-middle-left bar
	draw_segment_horizontal_caps(tempbitmap,
		0 + 2*segwidth/3, bmwidth/2 - segwidth/10, bmheight/2,
		segwidth, LINE_CAP_START, (state & (1 << 6)) ? onpen : offpen);

	// horizontal-middle-right bar
	draw_segment_horizontal_caps(tempbitmap,
		0 + bmwidth/2 + segwidth/10, bmwidth - 2*segwidth/3, bmheight/2,
		segwidth, LINE_CAP_END, (state & (1 << 7)) ? onpen : offpen);

	// vertical-middle-top bar
	draw_segment_vertical_caps(tempbitmap,
		0 + segwidth + segwidth/3, bmheight/2 - segwidth/2 - segwidth/3, bmwidth/2,
		segwidth, LINE_CAP_NONE, (state & (1 << 8)) ? onpen : offpen);

	// vertical-middle-bottom bar
	draw_segment_vertical_caps(tempbitmap,
		bmheight/2 + segwidth/2 + segwidth/3, bmheight - segwidth - segwidth/3, bmwidth/2,
		segwidth, LINE_CAP_NONE, (state & (1 << 9)) ? onpen : offpen);

	// diagonal-left-bottom bar
	draw_segment_diagonal_1(tempbitmap,
		0 + segwidth + segwidth/5, bmwidth/2 - segwidth/2 - segwidth/5,
		bmheight/2 + segwidth/2 + segwidth/3, bmheight - segwidth - segwidth/3,
		segwidth, (state & (1 << 10)) ? onpen : offpen);

	// diagonal-left-top bar
	draw_segment_diagonal_2(tempbitmap,
		0 + segwidth + segwidth/5, bmwidth/2 - segwidth/2 - segwidth/5,
		0 + segwidth + segwidth/3, bmheight/2 - segwidth/2 - segwidth/3,
		segwidth, (state & (1 << 11)) ? onpen : offpen);

	// diagonal-right-top bar
	draw_segment_diagonal_1(tempbitmap,
		bmwidth/2 + segwidth/2 + segwidth/5, bmwidth - segwidth - segwidth/5,
		0 + segwidth + segwidth/3, bmheight/2 - segwidth/2 - segwidth/3,
		segwidth, (state & (1 << 12)) ? onpen : offpen);

	// diagonal-right-bottom bar
	draw_segment_diagonal_2(tempbitmap,
		bmwidth/2 + segwidth/2 + segwidth/5, bmwidth - segwidth - segwidth/5,
		bmheight/2 + segwidth/2 + segwidth/3, bmheight - segwidth - segwidth/3,
		segwidth, (state & (1 << 13)) ? onpen : offpen);

	// apply skew
	apply_skew(tempbitmap, 40);

	// comma tail
	draw_segment_diagonal_1(tempbitmap,
		bmwidth - (segwidth/2), bmwidth + segwidth,
		bmheight - (segwidth), bmheight + segwidth*1.5,
		segwidth/2, (state & (1 << 15)) ? onpen : offpen);

	// decimal point
	draw_segment_decimal(tempbitmap, bmwidth + segwidth/2, bmheight - segwidth/2, segwidth, (state & (1 << 14)) ? onpen : offpen);

	// resample to the target size
	render_resample_argb_bitmap_hq(dest, tempbitmap, color());
}


//-------------------------------------------------
//  led16seg_component - constructor
//-------------------------------------------------

layout_element::led16seg_component::led16seg_component(running_machine &machine, util::xml::data_node const &compnode, const char *dirname)
	: component(machine, compnode, dirname)
{
}


//-------------------------------------------------
//  draw - draw a component
//-------------------------------------------------

void layout_element::led16seg_component::draw(running_machine &machine, bitmap_argb32 &dest, const rectangle &bounds, int state)
{
	const rgb_t onpen = rgb_t(0xff, 0xff, 0xff, 0xff);
	const rgb_t offpen = rgb_t(0xff, 0x20, 0x20, 0x20);

	// sizes for computation
	int bmwidth = 250;
	int bmheight = 400;
	int segwidth = 40;
	int skewwidth = 40;

	// allocate a temporary bitmap for drawing
	bitmap_argb32 tempbitmap(bmwidth + skewwidth, bmheight);
	tempbitmap.fill(rgb_t(0xff, 0x00, 0x00, 0x00));

	// top-left bar
	draw_segment_horizontal_caps(tempbitmap,
		0 + 2*segwidth/3, bmwidth/2 - segwidth/10, 0 + segwidth/2,
		segwidth, LINE_CAP_START, (state & (1 << 0)) ? onpen : offpen);

	// top-right bar
	draw_segment_horizontal_caps(tempbitmap,
		0 + bmwidth/2 + segwidth/10, bmwidth - 2*segwidth/3, 0 + segwidth/2,
		segwidth, LINE_CAP_END, (state & (1 << 1)) ? onpen : offpen);

	// right-top bar
	draw_segment_vertical(tempbitmap,
		0 + 2*segwidth/3, bmheight/2 - segwidth/3, bmwidth - segwidth/2,
		segwidth, (state & (1 << 2)) ? onpen : offpen);

	// right-bottom bar
	draw_segment_vertical(tempbitmap,
		bmheight/2 + segwidth/3, bmheight - 2*segwidth/3, bmwidth - segwidth/2,
		segwidth, (state & (1 << 3)) ? onpen : offpen);

	// bottom-right bar
	draw_segment_horizontal_caps(tempbitmap,
		0 + bmwidth/2 + segwidth/10, bmwidth - 2*segwidth/3, bmheight - segwidth/2,
		segwidth, LINE_CAP_END, (state & (1 << 4)) ? onpen : offpen);

	// bottom-left bar
	draw_segment_horizontal_caps(tempbitmap,
		0 + 2*segwidth/3, bmwidth/2 - segwidth/10, bmheight - segwidth/2,
		segwidth, LINE_CAP_START, (state & (1 << 5)) ? onpen : offpen);

	// left-bottom bar
	draw_segment_vertical(tempbitmap,
		bmheight/2 + segwidth/3, bmheight - 2*segwidth/3, 0 + segwidth/2,
		segwidth, (state & (1 << 6)) ? onpen : offpen);

	// left-top bar
	draw_segment_vertical(tempbitmap,
		0 + 2*segwidth/3, bmheight/2 - segwidth/3, 0 + segwidth/2,
		segwidth, (state & (1 << 7)) ? onpen : offpen);

	// horizontal-middle-left bar
	draw_segment_horizontal_caps(tempbitmap,
		0 + 2*segwidth/3, bmwidth/2 - segwidth/10, bmheight/2,
		segwidth, LINE_CAP_START, (state & (1 << 8)) ? onpen : offpen);

	// horizontal-middle-right bar
	draw_segment_horizontal_caps(tempbitmap,
		0 + bmwidth/2 + segwidth/10, bmwidth - 2*segwidth/3, bmheight/2,
		segwidth, LINE_CAP_END, (state & (1 << 9)) ? onpen : offpen);

	// vertical-middle-top bar
	draw_segment_vertical_caps(tempbitmap,
		0 + segwidth + segwidth/3, bmheight/2 - segwidth/2 - segwidth/3, bmwidth/2,
		segwidth, LINE_CAP_NONE, (state & (1 << 10)) ? onpen : offpen);

	// vertical-middle-bottom bar
	draw_segment_vertical_caps(tempbitmap,
		bmheight/2 + segwidth/2 + segwidth/3, bmheight - segwidth - segwidth/3, bmwidth/2,
		segwidth, LINE_CAP_NONE, (state & (1 << 11)) ? onpen : offpen);

	// diagonal-left-bottom bar
	draw_segment_diagonal_1(tempbitmap,
		0 + segwidth + segwidth/5, bmwidth/2 - segwidth/2 - segwidth/5,
		bmheight/2 + segwidth/2 + segwidth/3, bmheight - segwidth - segwidth/3,
		segwidth, (state & (1 << 12)) ? onpen : offpen);

	// diagonal-left-top bar
	draw_segment_diagonal_2(tempbitmap,
		0 + segwidth + segwidth/5, bmwidth/2 - segwidth/2 - segwidth/5,
		0 + segwidth + segwidth/3, bmheight/2 - segwidth/2 - segwidth/3,
		segwidth, (state & (1 << 13)) ? onpen : offpen);

	// diagonal-right-top bar
	draw_segment_diagonal_1(tempbitmap,
		bmwidth/2 + segwidth/2 + segwidth/5, bmwidth - segwidth - segwidth/5,
		0 + segwidth + segwidth/3, bmheight/2 - segwidth/2 - segwidth/3,
		segwidth, (state & (1 << 14)) ? onpen : offpen);

	// diagonal-right-bottom bar
	draw_segment_diagonal_2(tempbitmap,
		bmwidth/2 + segwidth/2 + segwidth/5, bmwidth - segwidth - segwidth/5,
		bmheight/2 + segwidth/2 + segwidth/3, bmheight - segwidth - segwidth/3,
		segwidth, (state & (1 << 15)) ? onpen : offpen);

	// apply skew
	apply_skew(tempbitmap, 40);

	// resample to the target size
	render_resample_argb_bitmap_hq(dest, tempbitmap, color());
}


//-------------------------------------------------
//  led16segsc_component - constructor
//-------------------------------------------------

layout_element::led16segsc_component::led16segsc_component(running_machine &machine, util::xml::data_node const &compnode, const char *dirname)
	: component(machine, compnode, dirname)
{
}


//-------------------------------------------------
//  draw - draw a component
//-------------------------------------------------

void layout_element::led16segsc_component::draw(running_machine &machine, bitmap_argb32 &dest, const rectangle &bounds, int state)
{
	const rgb_t onpen = rgb_t(0xff, 0xff, 0xff, 0xff);
	const rgb_t offpen = rgb_t(0xff, 0x20, 0x20, 0x20);

	// sizes for computation
	int bmwidth = 250;
	int bmheight = 400;
	int segwidth = 40;
	int skewwidth = 40;

	// allocate a temporary bitmap for drawing
	bitmap_argb32 tempbitmap(bmwidth + skewwidth, bmheight + segwidth);
	tempbitmap.fill(rgb_t(0xff, 0x00, 0x00, 0x00));

	// top-left bar
	draw_segment_horizontal_caps(tempbitmap,
		0 + 2*segwidth/3, bmwidth/2 - segwidth/10, 0 + segwidth/2,
		segwidth, LINE_CAP_START, (state & (1 << 0)) ? onpen : offpen);

	// top-right bar
	draw_segment_horizontal_caps(tempbitmap,
		0 + bmwidth/2 + segwidth/10, bmwidth - 2*segwidth/3, 0 + segwidth/2,
		segwidth, LINE_CAP_END, (state & (1 << 1)) ? onpen : offpen);

	// right-top bar
	draw_segment_vertical(tempbitmap,
		0 + 2*segwidth/3, bmheight/2 - segwidth/3, bmwidth - segwidth/2,
		segwidth, (state & (1 << 2)) ? onpen : offpen);

	// right-bottom bar
	draw_segment_vertical(tempbitmap,
		bmheight/2 + segwidth/3, bmheight - 2*segwidth/3, bmwidth - segwidth/2,
		segwidth, (state & (1 << 3)) ? onpen : offpen);

	// bottom-right bar
	draw_segment_horizontal_caps(tempbitmap,
		0 + bmwidth/2 + segwidth/10, bmwidth - 2*segwidth/3, bmheight - segwidth/2,
		segwidth, LINE_CAP_END, (state & (1 << 4)) ? onpen : offpen);

	// bottom-left bar
	draw_segment_horizontal_caps(tempbitmap,
		0 + 2*segwidth/3, bmwidth/2 - segwidth/10, bmheight - segwidth/2,
		segwidth, LINE_CAP_START, (state & (1 << 5)) ? onpen : offpen);

	// left-bottom bar
	draw_segment_vertical(tempbitmap,
		bmheight/2 + segwidth/3, bmheight - 2*segwidth/3, 0 + segwidth/2,
		segwidth, (state & (1 << 6)) ? onpen : offpen);

	// left-top bar
	draw_segment_vertical(tempbitmap,
		0 + 2*segwidth/3, bmheight/2 - segwidth/3, 0 + segwidth/2,
		segwidth, (state & (1 << 7)) ? onpen : offpen);

	// horizontal-middle-left bar
	draw_segment_horizontal_caps(tempbitmap,
		0 + 2*segwidth/3, bmwidth/2 - segwidth/10, bmheight/2,
		segwidth, LINE_CAP_START, (state & (1 << 8)) ? onpen : offpen);

	// horizontal-middle-right bar
	draw_segment_horizontal_caps(tempbitmap,
		0 + bmwidth/2 + segwidth/10, bmwidth - 2*segwidth/3, bmheight/2,
		segwidth, LINE_CAP_END, (state & (1 << 9)) ? onpen : offpen);

	// vertical-middle-top bar
	draw_segment_vertical_caps(tempbitmap,
		0 + segwidth + segwidth/3, bmheight/2 - segwidth/2 - segwidth/3, bmwidth/2,
		segwidth, LINE_CAP_NONE, (state & (1 << 10)) ? onpen : offpen);

	// vertical-middle-bottom bar
	draw_segment_vertical_caps(tempbitmap,
		bmheight/2 + segwidth/2 + segwidth/3, bmheight - segwidth - segwidth/3, bmwidth/2,
		segwidth, LINE_CAP_NONE, (state & (1 << 11)) ? onpen : offpen);

	// diagonal-left-bottom bar
	draw_segment_diagonal_1(tempbitmap,
		0 + segwidth + segwidth/5, bmwidth/2 - segwidth/2 - segwidth/5,
		bmheight/2 + segwidth/2 + segwidth/3, bmheight - segwidth - segwidth/3,
		segwidth, (state & (1 << 12)) ? onpen : offpen);

	// diagonal-left-top bar
	draw_segment_diagonal_2(tempbitmap,
		0 + segwidth + segwidth/5, bmwidth/2 - segwidth/2 - segwidth/5,
		0 + segwidth + segwidth/3, bmheight/2 - segwidth/2 - segwidth/3,
		segwidth, (state & (1 << 13)) ? onpen : offpen);

	// diagonal-right-top bar
	draw_segment_diagonal_1(tempbitmap,
		bmwidth/2 + segwidth/2 + segwidth/5, bmwidth - segwidth - segwidth/5,
		0 + segwidth + segwidth/3, bmheight/2 - segwidth/2 - segwidth/3,
		segwidth, (state & (1 << 14)) ? onpen : offpen);

	// diagonal-right-bottom bar
	draw_segment_diagonal_2(tempbitmap,
		bmwidth/2 + segwidth/2 + segwidth/5, bmwidth - segwidth - segwidth/5,
		bmheight/2 + segwidth/2 + segwidth/3, bmheight - segwidth - segwidth/3,
		segwidth, (state & (1 << 15)) ? onpen : offpen);

	// comma tail
	draw_segment_diagonal_1(tempbitmap,
		bmwidth - (segwidth/2), bmwidth + segwidth,
		bmheight - (segwidth), bmheight + segwidth*1.5,
		segwidth/2, (state & (1 << 17)) ? onpen : offpen);

	// decimal point (draw last for priority)
	draw_segment_decimal(tempbitmap, bmwidth + segwidth/2, bmheight - segwidth/2, segwidth, (state & (1 << 16)) ? onpen : offpen);

	// apply skew
	apply_skew(tempbitmap, 40);

	// resample to the target size
	render_resample_argb_bitmap_hq(dest, tempbitmap, color());
}


//-------------------------------------------------
//  rect_component - constructor
//-------------------------------------------------

layout_element::rect_component::rect_component(running_machine &machine, util::xml::data_node const &compnode, const char *dirname)
	: component(machine, compnode, dirname)
{
}

//-------------------------------------------------
//  draw - draw a component
//-------------------------------------------------

void layout_element::rect_component::draw(running_machine &machine, bitmap_argb32 &dest, const rectangle &bounds, int state)
{
	// compute premultiplied colors
	u32 r = color().r * color().a * 255.0f;
	u32 g = color().g * color().a * 255.0f;
	u32 b = color().b * color().a * 255.0f;
	u32 inva = (1.0f - color().a) * 255.0f;

	// iterate over X and Y
	for (u32 y = bounds.min_y; y <= bounds.max_y; y++)
	{
		for (u32 x = bounds.min_x; x <= bounds.max_x; x++)
		{
			u32 finalr = r;
			u32 finalg = g;
			u32 finalb = b;

			// if we're translucent, add in the destination pixel contribution
			if (inva > 0)
			{
				rgb_t dpix = dest.pix32(y, x);
				finalr += (dpix.r() * inva) >> 8;
				finalg += (dpix.g() * inva) >> 8;
				finalb += (dpix.b() * inva) >> 8;
			}

			// store the target pixel, dividing the RGBA values by the overall scale factor
			dest.pix32(y, x) = rgb_t(finalr, finalg, finalb);
		}
	}
}


//-------------------------------------------------
//  disk_component - constructor
//-------------------------------------------------

layout_element::disk_component::disk_component(running_machine &machine, util::xml::data_node const &compnode, const char *dirname)
	: component(machine, compnode, dirname)
{
}


//-------------------------------------------------
//  draw - draw a component
//-------------------------------------------------

void layout_element::disk_component::draw(running_machine &machine, bitmap_argb32 &dest, const rectangle &bounds, int state)
{
	// compute premultiplied colors
	u32 r = color().r * color().a * 255.0f;
	u32 g = color().g * color().a * 255.0f;
	u32 b = color().b * color().a * 255.0f;
	u32 inva = (1.0f - color().a) * 255.0f;

	// find the center
	float xcenter = float(bounds.xcenter());
	float ycenter = float(bounds.ycenter());
	float xradius = float(bounds.width()) * 0.5f;
	float yradius = float(bounds.height()) * 0.5f;
	float ooyradius2 = 1.0f / (yradius * yradius);

	// iterate over y
	for (u32 y = bounds.min_y; y <= bounds.max_y; y++)
	{
		float ycoord = ycenter - ((float)y + 0.5f);
		float xval = xradius * sqrtf(1.0f - (ycoord * ycoord) * ooyradius2);

		// compute left/right coordinates
		s32 left = s32(xcenter - xval + 0.5f);
		s32 right = s32(xcenter + xval + 0.5f);

		// draw this scanline
		for (u32 x = left; x < right; x++)
		{
			u32 finalr = r;
			u32 finalg = g;
			u32 finalb = b;

			// if we're translucent, add in the destination pixel contribution
			if (inva > 0)
			{
				rgb_t dpix = dest.pix32(y, x);
				finalr += (dpix.r() * inva) >> 8;
				finalg += (dpix.g() * inva) >> 8;
				finalb += (dpix.b() * inva) >> 8;
			}

			// store the target pixel, dividing the RGBA values by the overall scale factor
			dest.pix32(y, x) = rgb_t(finalr, finalg, finalb);
		}
	}
}


//-------------------------------------------------
//  draw_text - draw text in the specified color
//-------------------------------------------------

void layout_element::component::draw_text(render_font &font, bitmap_argb32 &dest, const rectangle &bounds, const char *str, int align)
{
	// compute premultiplied colors
	u32 r = color().r * 255.0f;
	u32 g = color().g * 255.0f;
	u32 b = color().b * 255.0f;
	u32 a = color().a * 255.0f;

	// get the width of the string
	float aspect = 1.0f;
	s32 width;

	while (1)
	{
		width = font.string_width(bounds.height(), aspect, str);
		if (width < bounds.width())
			break;
		aspect *= 0.9f;
	}

	// get alignment
	s32 curx;
	switch (align)
	{
		// left
		case 1:
			curx = bounds.min_x;
			break;

		// right
		case 2:
			curx = bounds.max_x - width;
			break;

		// default to center
		default:
			curx = bounds.min_x + (bounds.width() - width) / 2;
			break;
	}

	// allocate a temporary bitmap
	bitmap_argb32 tempbitmap(dest.width(), dest.height());

	// loop over characters
	const char *origs = str;
	const char *ends = origs + strlen(origs);
	const char *s = origs;
	char32_t schar;

	// loop over characters
	while (*s != 0)
	{
		int scharcount = uchar_from_utf8(&schar, s, ends - s);

		if (scharcount == -1)
			break;

		// get the font bitmap
		rectangle chbounds;
		font.get_scaled_bitmap_and_bounds(tempbitmap, bounds.height(), aspect, schar, chbounds);

		// copy the data into the target
		for (int y = 0; y < chbounds.height(); y++)
		{
			int effy = bounds.min_y + y;
			if (effy >= bounds.min_y && effy <= bounds.max_y)
			{
				u32 *src = &tempbitmap.pix32(y);
				u32 *d = &dest.pix32(effy);
				for (int x = 0; x < chbounds.width(); x++)
				{
					int effx = curx + x + chbounds.min_x;
					if (effx >= bounds.min_x && effx <= bounds.max_x)
					{
						u32 spix = rgb_t(src[x]).a();
						if (spix != 0)
						{
							rgb_t dpix = d[effx];
							u32 ta = (a * (spix + 1)) >> 8;
							u32 tr = (r * ta + dpix.r() * (0x100 - ta)) >> 8;
							u32 tg = (g * ta + dpix.g() * (0x100 - ta)) >> 8;
							u32 tb = (b * ta + dpix.b() * (0x100 - ta)) >> 8;
							d[effx] = rgb_t(tr, tg, tb);
						}
					}
				}
			}
		}

		// advance in the X direction
		curx += font.char_width(bounds.height(), aspect, schar);
		s += scharcount;
	}
}


//-------------------------------------------------
//  draw_segment_horizontal_caps - draw a
//  horizontal LED segment with definable end
//  and start points
//-------------------------------------------------

void layout_element::component::draw_segment_horizontal_caps(bitmap_argb32 &dest, int minx, int maxx, int midy, int width, int caps, rgb_t color)
{
	// loop over the width of the segment
	for (int y = 0; y < width / 2; y++)
	{
		u32 *d0 = &dest.pix32(midy - y);
		u32 *d1 = &dest.pix32(midy + y);
		int ty = (y < width / 8) ? width / 8 : y;

		// loop over the length of the segment
		for (int x = minx + ((caps & LINE_CAP_START) ? ty : 0); x < maxx - ((caps & LINE_CAP_END) ? ty : 0); x++)
			d0[x] = d1[x] = color;
	}
}


//-------------------------------------------------
//  draw_segment_horizontal - draw a horizontal
//  LED segment
//-------------------------------------------------

void layout_element::component::draw_segment_horizontal(bitmap_argb32 &dest, int minx, int maxx, int midy, int width, rgb_t color)
{
	draw_segment_horizontal_caps(dest, minx, maxx, midy, width, LINE_CAP_START | LINE_CAP_END, color);
}


//-------------------------------------------------
//  draw_segment_vertical_caps - draw a
//  vertical LED segment with definable end
//  and start points
//-------------------------------------------------

void layout_element::component::draw_segment_vertical_caps(bitmap_argb32 &dest, int miny, int maxy, int midx, int width, int caps, rgb_t color)
{
	// loop over the width of the segment
	for (int x = 0; x < width / 2; x++)
	{
		u32 *d0 = &dest.pix32(0, midx - x);
		u32 *d1 = &dest.pix32(0, midx + x);
		int tx = (x < width / 8) ? width / 8 : x;

		// loop over the length of the segment
		for (int y = miny + ((caps & LINE_CAP_START) ? tx : 0); y < maxy - ((caps & LINE_CAP_END) ? tx : 0); y++)
			d0[y * dest.rowpixels()] = d1[y * dest.rowpixels()] = color;
	}
}


//-------------------------------------------------
//  draw_segment_vertical - draw a vertical
//  LED segment
//-------------------------------------------------

void layout_element::component::draw_segment_vertical(bitmap_argb32 &dest, int miny, int maxy, int midx, int width, rgb_t color)
{
	draw_segment_vertical_caps(dest, miny, maxy, midx, width, LINE_CAP_START | LINE_CAP_END, color);
}


//-------------------------------------------------
//  draw_segment_diagonal_1 - draw a diagonal
//  LED segment that looks like a backslash
//-------------------------------------------------

void layout_element::component::draw_segment_diagonal_1(bitmap_argb32 &dest, int minx, int maxx, int miny, int maxy, int width, rgb_t color)
{
	// compute parameters
	width *= 1.5;
	float ratio = (maxy - miny - width) / (float)(maxx - minx);

	// draw line
	for (int x = minx; x < maxx; x++)
		if (x >= 0 && x < dest.width())
		{
			u32 *d = &dest.pix32(0, x);
			int step = (x - minx) * ratio;

			for (int y = maxy - width - step; y < maxy - step; y++)
				if (y >= 0 && y < dest.height())
					d[y * dest.rowpixels()] = color;
		}
}


//-------------------------------------------------
//  draw_segment_diagonal_2 - draw a diagonal
//  LED segment that looks like a forward slash
//-------------------------------------------------

void layout_element::component::draw_segment_diagonal_2(bitmap_argb32 &dest, int minx, int maxx, int miny, int maxy, int width, rgb_t color)
{
	// compute parameters
	width *= 1.5;
	float ratio = (maxy - miny - width) / (float)(maxx - minx);

	// draw line
	for (int x = minx; x < maxx; x++)
		if (x >= 0 && x < dest.width())
		{
			u32 *d = &dest.pix32(0, x);
			int step = (x - minx) * ratio;

			for (int y = miny + step; y < miny + step + width; y++)
				if (y >= 0 && y < dest.height())
					d[y * dest.rowpixels()] = color;
		}
}


//-------------------------------------------------
//  draw_segment_decimal - draw a decimal point
//-------------------------------------------------

void layout_element::component::draw_segment_decimal(bitmap_argb32 &dest, int midx, int midy, int width, rgb_t color)
{
	// compute parameters
	width /= 2;
	float ooradius2 = 1.0f / (float)(width * width);

	// iterate over y
	for (u32 y = 0; y <= width; y++)
	{
		u32 *d0 = &dest.pix32(midy - y);
		u32 *d1 = &dest.pix32(midy + y);
		float xval = width * sqrt(1.0f - (float)(y * y) * ooradius2);
		s32 left, right;

		// compute left/right coordinates
		left = midx - s32(xval + 0.5f);
		right = midx + s32(xval + 0.5f);

		// draw this scanline
		for (u32 x = left; x < right; x++)
			d0[x] = d1[x] = color;
	}
}


//-------------------------------------------------
//  draw_segment_comma - draw a comma tail
//-------------------------------------------------

void layout_element::component::draw_segment_comma(bitmap_argb32 &dest, int minx, int maxx, int miny, int maxy, int width, rgb_t color)
{
	// compute parameters
	width *= 1.5;
	float ratio = (maxy - miny - width) / (float)(maxx - minx);

	// draw line
	for (int x = minx; x < maxx; x++)
	{
		u32 *d = &dest.pix32(0, x);
		int step = (x - minx) * ratio;

		for (int y = maxy; y < maxy  - width - step; y--)
			d[y * dest.rowpixels()] = color;
	}
}


//-------------------------------------------------
//  apply_skew - apply skew to a bitmap
//-------------------------------------------------

void layout_element::component::apply_skew(bitmap_argb32 &dest, int skewwidth)
{
	for (int y = 0; y < dest.height(); y++)
	{
		u32 *destrow = &dest.pix32(y);
		int offs = skewwidth * (dest.height() - y) / dest.height();
		for (int x = dest.width() - skewwidth - 1; x >= 0; x--)
			destrow[x + offs] = destrow[x];
		for (int x = 0; x < offs; x++)
			destrow[x] = 0;
	}
}



//**************************************************************************
//  LAYOUT VIEW
//**************************************************************************

const simple_list<layout_view::item> layout_view::s_null_list;

//-------------------------------------------------
//  layout_view - constructor
//-------------------------------------------------

layout_view::layout_view(running_machine &machine, util::xml::data_node const &viewnode, simple_list<layout_element> &elemlist)
	: m_next(nullptr)
	, m_aspect(1.0f)
	, m_scraspect(1.0f)
{
	// allocate a copy of the name
	m_name = xml_get_attribute_string_with_subst(machine, viewnode, "name", "");

	// if we have a bounds item, load it
	util::xml::data_node const *const boundsnode = viewnode.get_child("bounds");
	m_expbounds.x0 = m_expbounds.y0 = m_expbounds.x1 = m_expbounds.y1 = 0;
	if (boundsnode != nullptr)
		parse_bounds(machine, boundsnode, m_expbounds);

	// load backdrop items
	for (util::xml::data_node const *itemnode = viewnode.get_child("backdrop"); itemnode != nullptr; itemnode = itemnode->get_next_sibling("backdrop"))
		m_backdrop_list.append(*global_alloc(item(machine, *itemnode, elemlist)));

	// load screen items
	for (util::xml::data_node const *itemnode = viewnode.get_child("screen"); itemnode != nullptr; itemnode = itemnode->get_next_sibling("screen"))
		m_screen_list.append(*global_alloc(item(machine, *itemnode, elemlist)));

	// load overlay items
	for (util::xml::data_node const *itemnode = viewnode.get_child("overlay"); itemnode != nullptr; itemnode = itemnode->get_next_sibling("overlay"))
		m_overlay_list.append(*global_alloc(item(machine, *itemnode, elemlist)));

	// load bezel items
	for (util::xml::data_node const *itemnode = viewnode.get_child("bezel"); itemnode != nullptr; itemnode = itemnode->get_next_sibling("bezel"))
		m_bezel_list.append(*global_alloc(item(machine, *itemnode, elemlist)));

	// load cpanel items
	for (util::xml::data_node const *itemnode = viewnode.get_child("cpanel"); itemnode != nullptr; itemnode = itemnode->get_next_sibling("cpanel"))
		m_cpanel_list.append(*global_alloc(item(machine, *itemnode, elemlist)));

	// load marquee items
	for (util::xml::data_node const *itemnode = viewnode.get_child("marquee"); itemnode != nullptr; itemnode = itemnode->get_next_sibling("marquee"))
		m_marquee_list.append(*global_alloc(item(machine, *itemnode, elemlist)));

	// recompute the data for the view based on a default layer config
	recompute(render_layer_config());
}


//-------------------------------------------------
//  layout_view - destructor
//-------------------------------------------------

layout_view::~layout_view()
{
}


//-------------------------------------------------
//  items - return the appropriate list
//-------------------------------------------------

const simple_list<layout_view::item> &layout_view::items(item_layer layer) const
{
	switch (layer)
	{
		case ITEM_LAYER_BACKDROP:   return m_backdrop_list;
		case ITEM_LAYER_SCREEN:     return m_screen_list;
		case ITEM_LAYER_OVERLAY:    return m_overlay_list;
		case ITEM_LAYER_BEZEL:      return m_bezel_list;
		case ITEM_LAYER_CPANEL:     return m_cpanel_list;
		case ITEM_LAYER_MARQUEE:    return m_marquee_list;
		default:                    return s_null_list;
	}
}


//-------------------------------------------------
//  recompute - recompute the bounds and aspect
//  ratio of a view and all of its contained items
//-------------------------------------------------

void layout_view::recompute(render_layer_config layerconfig)
{
	// reset the bounds
	m_bounds.x0 = m_bounds.y0 = m_bounds.x1 = m_bounds.y1 = 0.0f;
	m_scrbounds.x0 = m_scrbounds.y0 = m_scrbounds.x1 = m_scrbounds.y1 = 0.0f;
	m_screens.reset();

	// loop over all layers
	bool first = true;
	bool scrfirst = true;
	for (item_layer layer = ITEM_LAYER_FIRST; layer < ITEM_LAYER_MAX; ++layer)
	{
		// determine if this layer should be visible
		switch (layer)
		{
			case ITEM_LAYER_BACKDROP:   m_layenabled[layer] = layerconfig.backdrops_enabled();  break;
			case ITEM_LAYER_OVERLAY:    m_layenabled[layer] = layerconfig.overlays_enabled();   break;
			case ITEM_LAYER_BEZEL:      m_layenabled[layer] = layerconfig.bezels_enabled();     break;
			case ITEM_LAYER_CPANEL:     m_layenabled[layer] = layerconfig.cpanels_enabled();    break;
			case ITEM_LAYER_MARQUEE:    m_layenabled[layer] = layerconfig.marquees_enabled();   break;
			default:                    m_layenabled[layer] = true;                             break;
		}

		// only do it if requested
		if (m_layenabled[layer])
			for (item &curitem : items(layer))
			{
				// accumulate bounds
				if (first)
					m_bounds = curitem.m_rawbounds;
				else
					union_render_bounds(&m_bounds, &curitem.m_rawbounds);
				first = false;

				// accumulate screen bounds
				if (curitem.m_screen != nullptr)
				{
					if (scrfirst)
						m_scrbounds = curitem.m_rawbounds;
					else
						union_render_bounds(&m_scrbounds, &curitem.m_rawbounds);
					scrfirst = false;

					// accumulate the screens in use while we're scanning
					m_screens.add(*curitem.m_screen);
				}
			}
	}

	// if we have an explicit bounds, override it
	if (m_expbounds.x1 > m_expbounds.x0)
		m_bounds = m_expbounds;

	// if we're handling things normally, the target bounds are (0,0)-(1,1)
	render_bounds target_bounds;
	if (!layerconfig.zoom_to_screen() || m_screens.count() == 0)
	{
		// compute the aspect ratio of the view
		m_aspect = (m_bounds.x1 - m_bounds.x0) / (m_bounds.y1 - m_bounds.y0);

		target_bounds.x0 = target_bounds.y0 = 0.0f;
		target_bounds.x1 = target_bounds.y1 = 1.0f;
	}

	// if we're cropping, we want the screen area to fill (0,0)-(1,1)
	else
	{
		// compute the aspect ratio of the screen
		m_scraspect = (m_scrbounds.x1 - m_scrbounds.x0) / (m_scrbounds.y1 - m_scrbounds.y0);

		float targwidth = (m_bounds.x1 - m_bounds.x0) / (m_scrbounds.x1 - m_scrbounds.x0);
		float targheight = (m_bounds.y1 - m_bounds.y0) / (m_scrbounds.y1 - m_scrbounds.y0);
		target_bounds.x0 = (m_bounds.x0 - m_scrbounds.x0) / (m_bounds.x1 - m_bounds.x0) * targwidth;
		target_bounds.y0 = (m_bounds.y0 - m_scrbounds.y0) / (m_bounds.y1 - m_bounds.y0) * targheight;
		target_bounds.x1 = target_bounds.x0 + targwidth;
		target_bounds.y1 = target_bounds.y0 + targheight;
	}

	// determine the scale/offset for normalization
	float xoffs = m_bounds.x0;
	float yoffs = m_bounds.y0;
	float xscale = (target_bounds.x1 - target_bounds.x0) / (m_bounds.x1 - m_bounds.x0);
	float yscale = (target_bounds.y1 - target_bounds.y0) / (m_bounds.y1 - m_bounds.y0);

	// normalize all the item bounds
	for (item_layer layer = ITEM_LAYER_FIRST; layer < ITEM_LAYER_MAX; ++layer)
		for (item &curitem : items(layer))
		{
			curitem.m_bounds.x0 = target_bounds.x0 + (curitem.m_rawbounds.x0 - xoffs) * xscale;
			curitem.m_bounds.x1 = target_bounds.x0 + (curitem.m_rawbounds.x1 - xoffs) * xscale;
			curitem.m_bounds.y0 = target_bounds.y0 + (curitem.m_rawbounds.y0 - yoffs) * yscale;
			curitem.m_bounds.y1 = target_bounds.y0 + (curitem.m_rawbounds.y1 - yoffs) * yscale;
		}
}


//-----------------------------
//  resolve_tags - resolve tags
//-----------------------------

void layout_view::resolve_tags()
{
	for (item_layer layer = ITEM_LAYER_FIRST; layer < ITEM_LAYER_MAX; ++layer)
	{
		for (item &curitem : items(layer))
		{
			curitem.resolve_tags();
		}
	}
}



//**************************************************************************
//  LAYOUT VIEW ITEM
//**************************************************************************

//-------------------------------------------------
//  item - constructor
//-------------------------------------------------

layout_view::item::item(running_machine &machine, util::xml::data_node const &itemnode, simple_list<layout_element> &elemlist)
	: m_next(nullptr)
	, m_element(nullptr)
	, m_input_port(nullptr)
	, m_input_mask(0)
	, m_screen(nullptr)
	, m_orientation(ROT0)
{
	// allocate a copy of the output name
	m_output_name = xml_get_attribute_string_with_subst(machine, itemnode, "name", "");

	// allocate a copy of the input tag
	m_input_tag = xml_get_attribute_string_with_subst(machine, itemnode, "inputtag", "");

	// find the associated element
	const char *name = xml_get_attribute_string_with_subst(machine, itemnode, "element", nullptr);
	if (name != nullptr)
	{
		// search the list of elements for a match
		for (layout_element &elem : elemlist)
			if (strcmp(name, elem.name()) == 0)
			{
				m_element = &elem;
				break;
			}

		// error if not found
		if (m_element == nullptr)
			throw emu_fatalerror("Unable to find layout element %s", name);
	}

	// fetch common data
	int index = xml_get_attribute_int_with_subst(machine, itemnode, "index", -1);
	if (index != -1)
		m_screen = screen_device_iterator(machine.root_device()).byindex(index);
	m_input_mask = xml_get_attribute_int_with_subst(machine, itemnode, "inputmask", 0);
	if (m_output_name[0] != 0 && m_element != nullptr)
		machine.output().set_value(m_output_name.c_str(), m_element->default_state());
	parse_bounds(machine, itemnode.get_child("bounds"), m_rawbounds);
	parse_color(machine, itemnode.get_child("color"), m_color);
	parse_orientation(machine, itemnode.get_child("orientation"), m_orientation);

	// sanity checks
	if (strcmp(itemnode.get_name(), "screen") == 0)
	{
		if (m_screen == nullptr)
			throw emu_fatalerror("Layout references invalid screen index %d", index);
	}
	else
	{
		if (m_element == nullptr)
			throw emu_fatalerror("Layout item of type %s require an element tag", itemnode.get_name());
	}

	if (has_input())
	{
		m_input_port = m_element->machine().root_device().ioport(m_input_tag.c_str());
	}
}


//-------------------------------------------------
//  item - destructor
//-------------------------------------------------

layout_view::item::~item()
{
}


//-------------------------------------------------
//  screen_container - retrieve screen container
//-------------------------------------------------


render_container *layout_view::item::screen_container(running_machine &machine) const
{
	return (m_screen != nullptr) ? &m_screen->container() : nullptr;
}


//-------------------------------------------------
//  state - fetch state based on configured source
//-------------------------------------------------

int layout_view::item::state() const
{
	int state = 0;

	assert(m_element != nullptr);

	// if configured to an output, fetch the output value
	if (m_output_name[0] != 0)
		state = m_element->machine().output().get_value(m_output_name.c_str());

	// if configured to an input, fetch the input value
	else if (m_input_tag[0] != 0)
	{
		if (m_input_port != nullptr)
		{
			ioport_field *field = m_input_port->field(m_input_mask);
			if (field != nullptr)
				state = ((m_input_port->read() ^ field->defvalue()) & m_input_mask) ? 1 : 0;
		}
	}
	return state;
}


//---------------------------------------------
//  resolve_tags - resolve tags, if any are set
//---------------------------------------------


void layout_view::item::resolve_tags()
{
	if (has_input())
	{
		m_input_port = m_element->machine().root_device().ioport(m_input_tag.c_str());
	}
}



//**************************************************************************
//  LAYOUT FILE
//**************************************************************************

//-------------------------------------------------
//  layout_file - constructor
//-------------------------------------------------

layout_file::layout_file(running_machine &machine, util::xml::data_node const &rootnode, const char *dirname)
	: m_next(nullptr)
{
	// find the layout node
	util::xml::data_node const *const mamelayoutnode = rootnode.get_child("mamelayout");
	if (mamelayoutnode == nullptr)
		throw emu_fatalerror("Invalid XML file: missing mamelayout node");

	// validate the config data version
	int const version = mamelayoutnode->get_attribute_int("version", 0);
	if (version != LAYOUT_VERSION)
		throw emu_fatalerror("Invalid XML file: unsupported version");

	// parse all the elements
	for (util::xml::data_node const *elemnode = mamelayoutnode->get_child("element"); elemnode != nullptr; elemnode = elemnode->get_next_sibling("element"))
		m_elemlist.append(*global_alloc(layout_element(machine, *elemnode, dirname)));

	// parse all the views
	for (util::xml::data_node const *viewnode = mamelayoutnode->get_child("view"); viewnode != nullptr; viewnode = viewnode->get_next_sibling("view"))
	{
		// the trouble with allowing emu_fatalerror to propagate here is that it wreaks havoc with screenless systems that use a terminal by default
		// e.g. intlc44 and intlc440 have a terminal on the tty port by default and have a view with the front panel above the terminal screen
		// however, they have a second view with just the front panel which is very useful if you're using e.g. -tty null_modem with a socket
		// if the emu_fatalerror is allowed to propagate, the entire layout is dropped so you can't select the useful view
		try
		{
			m_viewlist.append(*global_alloc(layout_view(machine, *viewnode, m_elemlist)));
		}
		catch (emu_fatalerror const &)
		{
			// the exception will print its own message before we get here
		}
	}
}


//-------------------------------------------------
//  ~layout_file - destructor
//-------------------------------------------------

layout_file::~layout_file()
{
}
