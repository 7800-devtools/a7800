// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/***************************************************************************

    osdepend.c

    OS-dependent code interface.

***************************************************************************/


#include "emu.h"
#include "osdepend.h"
#include "modules/lib/osdobj_common.h"

const options_entry osd_options::s_option_entries[] =
{
	{ nullptr,                               nullptr,           OPTION_HEADER,    "OSD KEYBOARD MAPPING OPTIONS" },
#ifdef SDLMAME_MACOSX
	{ OSDOPTION_UIMODEKEY,                   "DEL",             OPTION_STRING,    "Key to toggle keyboard mode" },
#else
	{ OSDOPTION_UIMODEKEY,                   "SCRLOCK",         OPTION_STRING,    "Key to toggle keyboard mode" },
#endif  // SDLMAME_MACOSX

	{ nullptr,                                nullptr,          OPTION_HEADER,    "OSD FONT OPTIONS" },
	{ OSD_FONT_PROVIDER,                      OSDOPTVAL_AUTO,   OPTION_STRING,    "provider for ui font: " },

	{ nullptr,                                nullptr,          OPTION_HEADER,    "OSD OUTPUT OPTIONS" },
	{ OSD_OUTPUT_PROVIDER,                    OSDOPTVAL_AUTO,   OPTION_STRING,    "provider for output: " },

	{ nullptr,                                nullptr,          OPTION_HEADER,    "OSD INPUT OPTIONS" },
	{ OSD_KEYBOARDINPUT_PROVIDER,             OSDOPTVAL_AUTO,   OPTION_STRING,    "provider for keyboard input: " },
	{ OSD_MOUSEINPUT_PROVIDER,                OSDOPTVAL_AUTO,   OPTION_STRING,    "provider for mouse input: " },
	{ OSD_LIGHTGUNINPUT_PROVIDER,             OSDOPTVAL_AUTO,   OPTION_STRING,    "provider for lightgun input: " },
	{ OSD_JOYSTICKINPUT_PROVIDER,             OSDOPTVAL_AUTO,   OPTION_STRING,    "provider for joystick input: " },

	{ nullptr,                                nullptr,          OPTION_HEADER,    "OSD CLI OPTIONS" },
	{ OSDCOMMAND_LIST_MIDI_DEVICES ";mlist",  "0",              OPTION_COMMAND,   "list available MIDI I/O devices" },
	{ OSDCOMMAND_LIST_NETWORK_ADAPTERS ";nlist", "0",           OPTION_COMMAND,   "list available network adapters" },

	{ nullptr,                                nullptr,          OPTION_HEADER,    "OSD DEBUGGING OPTIONS" },
	{ OSDOPTION_DEBUGGER,                     OSDOPTVAL_AUTO,   OPTION_STRING,    "debugger used: " },
	{ OSDOPTION_DEBUGGER_FONT ";dfont",       OSDOPTVAL_AUTO,   OPTION_STRING,    "specifies the font to use for debugging" },
	{ OSDOPTION_DEBUGGER_FONT_SIZE ";dfontsize", "0",           OPTION_FLOAT,     "specifies the font size to use for debugging" },
	{ OSDOPTION_WATCHDOG ";wdog",             "0",              OPTION_INTEGER,   "force the program to terminate if no updates within specified number of seconds" },

	{ nullptr,                                nullptr,          OPTION_HEADER,    "OSD PERFORMANCE OPTIONS" },
	{ OSDOPTION_NUMPROCESSORS ";np",          OSDOPTVAL_AUTO,   OPTION_STRING,    "number of processors; this overrides the number the system reports" },
	{ OSDOPTION_BENCH,                        "0",              OPTION_INTEGER,   "benchmark for the given number of emulated seconds; implies -video none -sound none -nothrottle" },

	{ nullptr,                                nullptr,          OPTION_HEADER,    "OSD VIDEO OPTIONS" },
// OS X can be trusted to have working hardware OpenGL, so default to it on for the best user experience
	//{ OSDOPTION_VIDEO,                        OSDOPTVAL_AUTO,   OPTION_STRING,    "video output method: " },
	{ OSDOPTION_VIDEO,                        "bgfx",           OPTION_STRING,    "video output method: " },
	{ OSDOPTION_NUMSCREENS "(1-4)",           "1",              OPTION_INTEGER,   "number of screens to create; usually, you want just one" },
	//{ OSDOPTION_WINDOW ";w",                  "0",              OPTION_BOOLEAN,   "enable window mode; otherwise, full screen mode is assumed" },
	{ OSDOPTION_WINDOW ";w",                  "1",              OPTION_BOOLEAN,   "enable window mode; otherwise, full screen mode is assumed" },
	{ OSDOPTION_MAXIMIZE ";max",              "1",              OPTION_BOOLEAN,   "default to maximized windows; otherwise, windows will be minimized" },
	{ OSDOPTION_WAITVSYNC ";vs",              "0",              OPTION_BOOLEAN,   "enable waiting for the start of VBLANK before flipping screens; reduces tearing effects" },
	{ OSDOPTION_SYNCREFRESH ";srf",           "0",              OPTION_BOOLEAN,   "enable using the start of VBLANK for throttling instead of the game time" },
	{ OSD_MONITOR_PROVIDER,                   OSDOPTVAL_AUTO,   OPTION_STRING,    "monitor discovery method" },

	// per-window options
	{ nullptr,                                nullptr,          OPTION_HEADER,    "OSD PER-WINDOW VIDEO OPTIONS" },
	{ OSDOPTION_SCREEN,                       OSDOPTVAL_AUTO,   OPTION_STRING,    "explicit name of the first screen; 'auto' here will try to make a best guess" },
	{ OSDOPTION_ASPECT ";screen_aspect",      OSDOPTVAL_AUTO,   OPTION_STRING,    "aspect ratio for all screens; 'auto' here will try to make a best guess" },
	{ OSDOPTION_RESOLUTION ";r",              OSDOPTVAL_AUTO,   OPTION_STRING,    "preferred resolution for all screens; format is <width>x<height>[@<refreshrate>] or 'auto'" },
	{ OSDOPTION_VIEW,                         OSDOPTVAL_AUTO,   OPTION_STRING,    "preferred view for all screens" },

	{ OSDOPTION_SCREEN "0",                   OSDOPTVAL_AUTO,   OPTION_STRING,    "explicit name of the first screen; 'auto' here will try to make a best guess" },
	{ OSDOPTION_ASPECT "0",                   OSDOPTVAL_AUTO,   OPTION_STRING,    "aspect ratio of the first screen; 'auto' here will try to make a best guess" },
	{ OSDOPTION_RESOLUTION "0;r0",            OSDOPTVAL_AUTO,   OPTION_STRING,    "preferred resolution of the first screen; format is <width>x<height>[@<refreshrate>] or 'auto'" },
	{ OSDOPTION_VIEW "0",                     OSDOPTVAL_AUTO,   OPTION_STRING,    "preferred view for the first screen" },

	{ OSDOPTION_SCREEN "1",                   OSDOPTVAL_AUTO,   OPTION_STRING,    "explicit name of the second screen; 'auto' here will try to make a best guess" },
	{ OSDOPTION_ASPECT "1",                   OSDOPTVAL_AUTO,   OPTION_STRING,    "aspect ratio of the second screen; 'auto' here will try to make a best guess" },
	{ OSDOPTION_RESOLUTION "1;r1",            OSDOPTVAL_AUTO,   OPTION_STRING,    "preferred resolution of the second screen; format is <width>x<height>[@<refreshrate>] or 'auto'" },
	{ OSDOPTION_VIEW "1",                     OSDOPTVAL_AUTO,   OPTION_STRING,    "preferred view for the second screen" },

	{ OSDOPTION_SCREEN "2",                   OSDOPTVAL_AUTO,   OPTION_STRING,    "explicit name of the third screen; 'auto' here will try to make a best guess" },
	{ OSDOPTION_ASPECT "2",                   OSDOPTVAL_AUTO,   OPTION_STRING,    "aspect ratio of the third screen; 'auto' here will try to make a best guess" },
	{ OSDOPTION_RESOLUTION "2;r2",            OSDOPTVAL_AUTO,   OPTION_STRING,    "preferred resolution of the third screen; format is <width>x<height>[@<refreshrate>] or 'auto'" },
	{ OSDOPTION_VIEW "2",                     OSDOPTVAL_AUTO,   OPTION_STRING,    "preferred view for the third screen" },

	{ OSDOPTION_SCREEN "3",                   OSDOPTVAL_AUTO,   OPTION_STRING,    "explicit name of the fourth screen; 'auto' here will try to make a best guess" },
	{ OSDOPTION_ASPECT "3",                   OSDOPTVAL_AUTO,   OPTION_STRING,    "aspect ratio of the fourth screen; 'auto' here will try to make a best guess" },
	{ OSDOPTION_RESOLUTION "3;r3",            OSDOPTVAL_AUTO,   OPTION_STRING,    "preferred resolution of the fourth screen; format is <width>x<height>[@<refreshrate>] or 'auto'" },
	{ OSDOPTION_VIEW "3",                     OSDOPTVAL_AUTO,   OPTION_STRING,    "preferred view for the fourth screen" },

	// full screen options
	{ nullptr,                                nullptr,          OPTION_HEADER,    "OSD FULL SCREEN OPTIONS" },
	{ OSDOPTION_SWITCHRES,                    "0",              OPTION_BOOLEAN,   "enable resolution switching" },

	{ nullptr,                                nullptr,          OPTION_HEADER,    "OSD ACCELERATED VIDEO OPTIONS" },
	//{ OSDOPTION_FILTER ";glfilter;flt",       "1",              OPTION_BOOLEAN,   "enable bilinear filtering on screen output" },
	{ OSDOPTION_FILTER ";glfilter;flt",       "0",              OPTION_BOOLEAN,   "enable bilinear filtering on screen output" },
	{ OSDOPTION_PRESCALE,                     "1",              OPTION_INTEGER,   "scale screen rendering by this amount in software" },

#if USE_OPENGL
	{ nullptr,                                nullptr,          OPTION_HEADER,    "OpenGL-SPECIFIC OPTIONS" },
	{ OSDOPTION_GL_FORCEPOW2TEXTURE,          "0",              OPTION_BOOLEAN,   "force power of two textures  (default no)" },
	{ OSDOPTION_GL_NOTEXTURERECT,             "0",              OPTION_BOOLEAN,   "don't use OpenGL GL_ARB_texture_rectangle (default on)" },
	{ OSDOPTION_GL_VBO,                       "1",              OPTION_BOOLEAN,   "enable OpenGL VBO,  if available (default on)" },
	{ OSDOPTION_GL_PBO,                       "1",              OPTION_BOOLEAN,   "enable OpenGL PBO,  if available (default on)" },
	{ OSDOPTION_GL_GLSL,                      "0",              OPTION_BOOLEAN,   "enable OpenGL GLSL, if available (default off)" },
	{ OSDOPTION_GLSL_FILTER,                  "1",              OPTION_STRING,    "enable OpenGL GLSL filtering instead of FF filtering 0-plain, 1-bilinear (default)" },
	{ OSDOPTION_SHADER_MAME "0",              OSDOPTVAL_NONE,   OPTION_STRING,    "custom OpenGL GLSL shader set mame bitmap 0" },
	{ OSDOPTION_SHADER_MAME "1",              OSDOPTVAL_NONE,   OPTION_STRING,    "custom OpenGL GLSL shader set mame bitmap 1" },
	{ OSDOPTION_SHADER_MAME "2",              OSDOPTVAL_NONE,   OPTION_STRING,    "custom OpenGL GLSL shader set mame bitmap 2" },
	{ OSDOPTION_SHADER_MAME "3",              OSDOPTVAL_NONE,   OPTION_STRING,    "custom OpenGL GLSL shader set mame bitmap 3" },
	{ OSDOPTION_SHADER_MAME "4",              OSDOPTVAL_NONE,   OPTION_STRING,    "custom OpenGL GLSL shader set mame bitmap 4" },
	{ OSDOPTION_SHADER_MAME "5",              OSDOPTVAL_NONE,   OPTION_STRING,    "custom OpenGL GLSL shader set mame bitmap 5" },
	{ OSDOPTION_SHADER_MAME "6",              OSDOPTVAL_NONE,   OPTION_STRING,    "custom OpenGL GLSL shader set mame bitmap 6" },
	{ OSDOPTION_SHADER_MAME "7",              OSDOPTVAL_NONE,   OPTION_STRING,    "custom OpenGL GLSL shader set mame bitmap 7" },
	{ OSDOPTION_SHADER_MAME "8",              OSDOPTVAL_NONE,   OPTION_STRING,    "custom OpenGL GLSL shader set mame bitmap 8" },
	{ OSDOPTION_SHADER_MAME "9",              OSDOPTVAL_NONE,   OPTION_STRING,    "custom OpenGL GLSL shader set mame bitmap 9" },
	{ OSDOPTION_SHADER_SCREEN "0",            OSDOPTVAL_NONE,   OPTION_STRING,    "custom OpenGL GLSL shader screen bitmap 0" },
	{ OSDOPTION_SHADER_SCREEN "1",            OSDOPTVAL_NONE,   OPTION_STRING,    "custom OpenGL GLSL shader screen bitmap 1" },
	{ OSDOPTION_SHADER_SCREEN "2",            OSDOPTVAL_NONE,   OPTION_STRING,    "custom OpenGL GLSL shader screen bitmap 2" },
	{ OSDOPTION_SHADER_SCREEN "3",            OSDOPTVAL_NONE,   OPTION_STRING,    "custom OpenGL GLSL shader screen bitmap 3" },
	{ OSDOPTION_SHADER_SCREEN "4",            OSDOPTVAL_NONE,   OPTION_STRING,    "custom OpenGL GLSL shader screen bitmap 4" },
	{ OSDOPTION_SHADER_SCREEN "5",            OSDOPTVAL_NONE,   OPTION_STRING,    "custom OpenGL GLSL shader screen bitmap 5" },
	{ OSDOPTION_SHADER_SCREEN "6",            OSDOPTVAL_NONE,   OPTION_STRING,    "custom OpenGL GLSL shader screen bitmap 6" },
	{ OSDOPTION_SHADER_SCREEN "7",            OSDOPTVAL_NONE,   OPTION_STRING,    "custom OpenGL GLSL shader screen bitmap 7" },
	{ OSDOPTION_SHADER_SCREEN "8",            OSDOPTVAL_NONE,   OPTION_STRING,    "custom OpenGL GLSL shader screen bitmap 8" },
	{ OSDOPTION_SHADER_SCREEN "9",            OSDOPTVAL_NONE,   OPTION_STRING,    "custom OpenGL GLSL shader screen bitmap 9" },
#endif

	{ nullptr,                                nullptr,          OPTION_HEADER,    "OSD SOUND OPTIONS" },
	{ OSDOPTION_SOUND,                        OSDOPTVAL_AUTO,   OPTION_STRING,    "sound output method: " },
	{ OSDOPTION_AUDIO_LATENCY "(1-5)",        "2",              OPTION_INTEGER,   "set audio latency (increase to reduce glitches, decrease for responsiveness)" },

#ifndef NO_USE_PORTAUDIO
	{ nullptr,                                nullptr,          OPTION_HEADER,    "PORTAUDIO OPTIONS" },
	{ OSDOPTION_PA_API,                       OSDOPTVAL_NONE,   OPTION_STRING,    "PortAudio API" },
	{ OSDOPTION_PA_DEVICE,                    OSDOPTVAL_NONE,   OPTION_STRING,    "PortAudio device" },
	{ OSDOPTION_PA_LATENCY "(0-0.25)",        "0",              OPTION_FLOAT,     "suggested latency in seconds, 0 for default" },
#endif

#ifdef SDLMAME_MACOSX
	{ nullptr,                                nullptr,          OPTION_HEADER,    "CoreAudio-SPECIFIC OPTIONS" },
	{ OSDOPTION_AUDIO_OUTPUT,                 OSDOPTVAL_AUTO,   OPTION_STRING,    "Audio output device" },
	{ OSDOPTION_AUDIO_EFFECT "0",             OSDOPTVAL_NONE,   OPTION_STRING,    "AudioUnit effect 0" },
	{ OSDOPTION_AUDIO_EFFECT "1",             OSDOPTVAL_NONE,   OPTION_STRING,    "AudioUnit effect 1" },
	{ OSDOPTION_AUDIO_EFFECT "2",             OSDOPTVAL_NONE,   OPTION_STRING,    "AudioUnit effect 2" },
	{ OSDOPTION_AUDIO_EFFECT "3",             OSDOPTVAL_NONE,   OPTION_STRING,    "AudioUnit effect 3" },
	{ OSDOPTION_AUDIO_EFFECT "4",             OSDOPTVAL_NONE,   OPTION_STRING,    "AudioUnit effect 4" },
	{ OSDOPTION_AUDIO_EFFECT "5",             OSDOPTVAL_NONE,   OPTION_STRING,    "AudioUnit effect 5" },
	{ OSDOPTION_AUDIO_EFFECT "6",             OSDOPTVAL_NONE,   OPTION_STRING,    "AudioUnit effect 6" },
	{ OSDOPTION_AUDIO_EFFECT "7",             OSDOPTVAL_NONE,   OPTION_STRING,    "AudioUnit effect 7" },
	{ OSDOPTION_AUDIO_EFFECT "8",             OSDOPTVAL_NONE,   OPTION_STRING,    "AudioUnit effect 8" },
	{ OSDOPTION_AUDIO_EFFECT "9",             OSDOPTVAL_NONE,   OPTION_STRING,    "AudioUnit effect 9" },
#endif

	{ nullptr,                                nullptr,           OPTION_HEADER, "BGFX POST-PROCESSING OPTIONS" },
	{ OSDOPTION_BGFX_PATH,                    "bgfx",            OPTION_STRING, "path to BGFX-related files" },
	{ OSDOPTION_BGFX_BACKEND,                 "auto",            OPTION_STRING, "BGFX backend to use (d3d9, d3d11, metal, opengl, gles)" },
	{ OSDOPTION_BGFX_DEBUG,                   "0",               OPTION_BOOLEAN, "enable BGFX debugging statistics" },
	//{ OSDOPTION_BGFX_SCREEN_CHAINS,           "default",         OPTION_STRING, "comma-delimited list of screen chain JSON names, colon-delimited per-window" },
	//{ OSDOPTION_BGFX_SCREEN_CHAINS,           "hlsl",         OPTION_STRING, "comma-delimited list of screen chain JSON names, colon-delimited per-window" },
	{ OSDOPTION_BGFX_SCREEN_CHAINS,           "none",         OPTION_STRING, "comma-delimited list of screen chain JSON names, colon-delimited per-window" },
	{ OSDOPTION_BGFX_SHADOW_MASK,             "slot-mask.png",   OPTION_STRING, "shadow mask texture name" },
	{ OSDOPTION_BGFX_AVI_NAME,                OSDOPTVAL_AUTO,    OPTION_STRING, "filename for BGFX output logging" },

		// End of list
	{ nullptr }
};

osd_options::osd_options()
: emu_options()
{
	add_entries(osd_options::s_option_entries);
}

// Window list
std::list<std::shared_ptr<osd_window>> osd_common_t::s_window_list;

//-------------------------------------------------
//  osd_interface - constructor
//-------------------------------------------------

osd_common_t::osd_common_t(osd_options &options)
	: osd_output(), m_machine(nullptr),
		m_options(options),
		m_print_verbose(false),
		m_font_module(nullptr),
		m_sound(nullptr),
		m_debugger(nullptr),
		m_midi(nullptr),
		m_keyboard_input(nullptr),
		m_mouse_input(nullptr),
		m_lightgun_input(nullptr),
		m_joystick_input(nullptr),
		m_output(nullptr),
		m_monitor_module(nullptr),
		m_watchdog(nullptr)
{
	osd_output::push(this);
}

//-------------------------------------------------
//  osd_interface - destructor
//-------------------------------------------------

osd_common_t::~osd_common_t()
{
	for(unsigned int i= 0; i < m_video_names.size(); ++i)
		free(const_cast<char*>(m_video_names[i]));
	//m_video_options,reset();
	osd_output::pop(this);
}

#define REGISTER_MODULE(_O, _X ) { extern const module_type _X; _O . register_module( _X ); }

void osd_common_t::register_options()
{
	REGISTER_MODULE(m_mod_man, FONT_OSX);
	REGISTER_MODULE(m_mod_man, FONT_WINDOWS);
	REGISTER_MODULE(m_mod_man, FONT_DWRITE);
	REGISTER_MODULE(m_mod_man, FONT_SDL);
	REGISTER_MODULE(m_mod_man, FONT_NONE);

	REGISTER_MODULE(m_mod_man, SOUND_XAUDIO2);
	REGISTER_MODULE(m_mod_man, SOUND_DSOUND);
	REGISTER_MODULE(m_mod_man, SOUND_COREAUDIO);
	REGISTER_MODULE(m_mod_man, SOUND_JS);
	REGISTER_MODULE(m_mod_man, SOUND_SDL);
#ifndef NO_USE_PORTAUDIO
	REGISTER_MODULE(m_mod_man, SOUND_PORTAUDIO);
#endif
	REGISTER_MODULE(m_mod_man, SOUND_NONE);

	REGISTER_MODULE(m_mod_man, MONITOR_SDL);
	REGISTER_MODULE(m_mod_man, MONITOR_WIN32);
	REGISTER_MODULE(m_mod_man, MONITOR_DXGI);

#ifdef SDLMAME_MACOSX
	REGISTER_MODULE(m_mod_man, DEBUG_OSX);
#endif
#ifndef OSD_MINI
	REGISTER_MODULE(m_mod_man, DEBUG_WINDOWS);
	REGISTER_MODULE(m_mod_man, DEBUG_QT);
	REGISTER_MODULE(m_mod_man, DEBUG_IMGUI);
	REGISTER_MODULE(m_mod_man, DEBUG_NONE);
#endif

	REGISTER_MODULE(m_mod_man, NETDEV_TAPTUN);
	REGISTER_MODULE(m_mod_man, NETDEV_PCAP);
	REGISTER_MODULE(m_mod_man, NETDEV_NONE);

#ifndef NO_USE_MIDI
	REGISTER_MODULE(m_mod_man, MIDI_PM);
#endif
	REGISTER_MODULE(m_mod_man, MIDI_NONE);

	REGISTER_MODULE(m_mod_man, KEYBOARDINPUT_SDL);
	REGISTER_MODULE(m_mod_man, KEYBOARDINPUT_RAWINPUT);
	REGISTER_MODULE(m_mod_man, KEYBOARDINPUT_DINPUT);
	REGISTER_MODULE(m_mod_man, KEYBOARDINPUT_WIN32);
	REGISTER_MODULE(m_mod_man, KEYBOARDINPUT_UWP);
	REGISTER_MODULE(m_mod_man, KEYBOARD_NONE);

	REGISTER_MODULE(m_mod_man, MOUSEINPUT_SDL);
	REGISTER_MODULE(m_mod_man, MOUSEINPUT_RAWINPUT);
	REGISTER_MODULE(m_mod_man, MOUSEINPUT_DINPUT);
	REGISTER_MODULE(m_mod_man, MOUSEINPUT_WIN32);
	REGISTER_MODULE(m_mod_man, MOUSE_NONE);

	REGISTER_MODULE(m_mod_man, LIGHTGUN_X11);
	REGISTER_MODULE(m_mod_man, LIGHTGUNINPUT_RAWINPUT);
	REGISTER_MODULE(m_mod_man, LIGHTGUNINPUT_WIN32);
	REGISTER_MODULE(m_mod_man, LIGHTGUN_NONE);

	REGISTER_MODULE(m_mod_man, JOYSTICKINPUT_SDL);
	REGISTER_MODULE(m_mod_man, JOYSTICKINPUT_WINHYBRID);
	REGISTER_MODULE(m_mod_man, JOYSTICKINPUT_DINPUT);
	REGISTER_MODULE(m_mod_man, JOYSTICKINPUT_XINPUT);
	REGISTER_MODULE(m_mod_man, JOYSTICKINPUT_UWP);
	REGISTER_MODULE(m_mod_man, JOYSTICK_NONE);

	REGISTER_MODULE(m_mod_man, OUTPUT_NONE);
	REGISTER_MODULE(m_mod_man, OUTPUT_CONSOLE);
	REGISTER_MODULE(m_mod_man, OUTPUT_NETWORK);
	REGISTER_MODULE(m_mod_man, OUTPUT_WIN32);


	// after initialization we know which modules are supported

	const char *names[20];
	int num;
	std::vector<const char *> dnames;

	m_mod_man.get_module_names(OSD_MONITOR_PROVIDER, 20, &num, names);
	for (int i = 0; i < num; i++)
		dnames.push_back(names[i]);
	update_option(OSD_MONITOR_PROVIDER, dnames);

	m_mod_man.get_module_names(OSD_FONT_PROVIDER, 20, &num, names);
	dnames.clear();
	for (int i = 0; i < num; i++)
		dnames.push_back(names[i]);
	update_option(OSD_FONT_PROVIDER, dnames);

	m_mod_man.get_module_names(OSD_KEYBOARDINPUT_PROVIDER, 20, &num, names);
	dnames.clear();
	for (int i = 0; i < num; i++)
		dnames.push_back(names[i]);
	update_option(OSD_KEYBOARDINPUT_PROVIDER, dnames);

	m_mod_man.get_module_names(OSD_MOUSEINPUT_PROVIDER, 20, &num, names);
	dnames.clear();
	for (int i = 0; i < num; i++)
		dnames.push_back(names[i]);
	update_option(OSD_MOUSEINPUT_PROVIDER, dnames);

	m_mod_man.get_module_names(OSD_LIGHTGUNINPUT_PROVIDER, 20, &num, names);
	dnames.clear();
	for (int i = 0; i < num; i++)
		dnames.push_back(names[i]);
	update_option(OSD_LIGHTGUNINPUT_PROVIDER, dnames);

	m_mod_man.get_module_names(OSD_JOYSTICKINPUT_PROVIDER, 20, &num, names);
	dnames.clear();
	for (int i = 0; i < num; i++)
		dnames.push_back(names[i]);
	update_option(OSD_JOYSTICKINPUT_PROVIDER, dnames);

	m_mod_man.get_module_names(OSD_SOUND_PROVIDER, 20, &num, names);
	dnames.clear();
	for (int i = 0; i < num; i++)
		dnames.push_back(names[i]);
	update_option(OSD_SOUND_PROVIDER, dnames);

#if 0
	// Register midi options and update options
	m_mod_man.get_module_names(OSD_MIDI_PROVIDER, 20, &num, names);
	dnames.clear();
	for (int i = 0; i < num; i++)
		dnames.push_back(names[i]);
	update_option(OSD_MIDI_PROVIDER, dnames);
#endif

	// Register debugger options and update options
	m_mod_man.get_module_names(OSD_DEBUG_PROVIDER, 20, &num, names);
	dnames.clear();
	for (int i = 0; i < num; i++)
		dnames.push_back(names[i]);
	update_option(OSD_DEBUG_PROVIDER, dnames);

	m_mod_man.get_module_names(OSD_OUTPUT_PROVIDER, 20, &num, names);
	dnames.clear();
	for (int i = 0; i < num; i++)
		dnames.push_back(names[i]);
	update_option(OSD_OUTPUT_PROVIDER, dnames);

	// Register video options and update options
	video_options_add("none", nullptr);
	video_register();
	update_option(OSDOPTION_VIDEO, m_video_names);
}

void osd_common_t::update_option(const char * key, std::vector<const char *> &values) const
{
	std::string current_value(m_options.description(key));
	std::string new_option_value("");
	for (unsigned int index = 0; index < values.size(); index++)
	{
		std::string t(values[index]);
		if (new_option_value.length() > 0)
		{
			if( index != (values.size()-1))
				new_option_value.append(", ");
			else
				new_option_value.append(" or ");
		}
		new_option_value.append(t);
	}
	// TODO: core_strdup() is leaked
	m_options.set_description(key, core_strdup(current_value.append(new_option_value).c_str()));
}


//-------------------------------------------------
//  output_callback  - callback for osd_printf_...
//-------------------------------------------------
void osd_common_t::output_callback(osd_output_channel channel, const char *msg, va_list args)
{
	switch (channel)
	{
		case OSD_OUTPUT_CHANNEL_ERROR:
		case OSD_OUTPUT_CHANNEL_WARNING:
			vfprintf(stderr, msg, args);
			break;
		case OSD_OUTPUT_CHANNEL_INFO:
		case OSD_OUTPUT_CHANNEL_LOG:
			vfprintf(stdout, msg, args);
			break;
		case OSD_OUTPUT_CHANNEL_VERBOSE:
			if (verbose()) vfprintf(stdout, msg, args);
			break;
		case OSD_OUTPUT_CHANNEL_DEBUG:
#ifdef MAME_DEBUG
			vfprintf(stdout, msg, args);
#endif
			break;
		default:
			break;
	}
}

//-------------------------------------------------
//  init - initialize the OSD system.
//-------------------------------------------------

void osd_common_t::init(running_machine &machine)
{
	//
	// This function is responsible for initializing the OSD-specific
	// video and input functionality, and registering that functionality
	// with the MAME core.
	//
	// In terms of video, this function is expected to create one or more
	// render_targets that will be used by the MAME core to provide graphics
	// data to the system. Although it is possible to do this later, the
	// assumption in the MAME core is that the user interface will be
	// visible starting at init() time, so you will have some work to
	// do to avoid these assumptions.
	//
	// In terms of input, this function is expected to enumerate all input
	// devices available and describe them to the MAME core by adding
	// input devices and their attached items (buttons/axes) via the input
	// system.
	//
	// Beyond these core responsibilities, init() should also initialize
	// any other OSD systems that require information about the current
	// running_machine.
	//
	// This callback is also the last opportunity to adjust the options
	// before they are consumed by the rest of the core.
	//
	// Future work/changes:
	//
	// Audio initialization may eventually move into here as well,
	// instead of relying on independent callbacks from each system.
	//

	m_machine = &machine;

	osd_options &options = downcast<osd_options &>(machine.options());
	// extract the verbose printing option
	if (options.verbose())
		set_verbose(true);

	// ensure we get called on the way out
	machine.add_notifier(MACHINE_NOTIFY_EXIT, machine_notify_delegate(&osd_common_t::osd_exit, this));


	/* now setup watchdog */
	int watchdog_timeout = options.watchdog();

	if (watchdog_timeout != 0)
	{
		m_watchdog = std::make_unique<osd_watchdog>();
		m_watchdog->setTimeout(watchdog_timeout);
	}
}


//-------------------------------------------------
//  update - periodic system update
//-------------------------------------------------

void osd_common_t::update(bool skip_redraw)
{
	//
	// This method is called periodically to flush video updates to the
	// screen, and also to allow the OSD a chance to update other systems
	// on a regular basis. In general this will be called at the frame
	// rate of the system being run; however, it may be called at more
	// irregular intervals in some circumstances (e.g., multi-screen games
	// or games with asynchronous updates).
	//
	if (m_watchdog != nullptr)
		m_watchdog->reset();

	update_slider_list();

}


//-------------------------------------------------
//  init_debugger - perform debugger-specific
//  initialization
//-------------------------------------------------

void osd_common_t::init_debugger()
{
	//
	// Unlike init() above, this method is only called if the debugger
	// is active. This gives any OSD debugger interface a chance to
	// create all of its structures.
	//
	m_debugger->init_debugger(machine());
}


//-------------------------------------------------
//  wait_for_debugger - wait for a debugger
//  command to be processed
//-------------------------------------------------

void osd_common_t::wait_for_debugger(device_t &device, bool firststop)
{
	//
	// When implementing an OSD-driver debugger, this method should be
	// overridden to wait for input, process it, and return. It will be
	// called repeatedly until a command is issued that resumes
	// execution.
	//
	m_debugger->wait_for_debugger(device, firststop);
}

void osd_common_t::debugger_update()
{
	if (m_debugger) m_debugger->debugger_update();
}


//-------------------------------------------------
//  update_audio_stream - update the stereo audio
//  stream
//-------------------------------------------------

void osd_common_t::update_audio_stream(const int16_t *buffer, int samples_this_frame)
{
	//
	// This method is called whenever the system has new audio data to stream.
	// It provides an array of stereo samples in L-R order which should be
	// output at the configured sample_rate.
	//
	m_sound->update_audio_stream(m_machine->video().throttled(), buffer,samples_this_frame);
}


//-------------------------------------------------
//  set_mastervolume - set the system volume
//-------------------------------------------------

void osd_common_t::set_mastervolume(int attenuation)
{
	//
	// Attenuation is the attenuation in dB (a negative number).
	// To convert from dB to a linear volume scale do the following:
	//    volume = MAX_VOLUME;
	//    while (attenuation++ < 0)
	//       volume /= 1.122018454;      //  = (10 ^ (1/20)) = 1dB
	//
	if (m_sound != nullptr)
		m_sound->set_mastervolume(attenuation);
}


//-------------------------------------------------
//  customize_input_type_list - provide OSD
//  additions/modifications to the input list
//-------------------------------------------------

void osd_common_t::customize_input_type_list(simple_list<input_type_entry> &typelist)
{
	//
	// inptport.c defines some general purpose defaults for key and joystick bindings.
	// They may be further adjusted by the OS dependent code to better match the
	// available keyboard, e.g. one could map pause to the Pause key instead of P, or
	// snapshot to PrtScr instead of F12. Of course the user can further change the
	// settings to anything he/she likes.
	//
	// This function is called on startup, before reading the configuration from disk.
	// Scan the list, and change the keys/joysticks you want.
	//
}


//-------------------------------------------------
//  get_slider_list - allocate and populate a
//  list of OS-dependent slider values.
//-------------------------------------------------

std::vector<ui::menu_item> osd_common_t::get_slider_list()
{
	return m_sliders;
}


//-------------------------------------------------
//  add_audio_to_recording - append audio samples
//  to an AVI recording if one is active
//-------------------------------------------------

void osd_common_t::add_audio_to_recording(const int16_t *buffer, int samples_this_frame)
{
	// Do nothing
}


//-------------------------------------------------
//  execute_command - execute a command not yet
//  handled by the core
//-------------------------------------------------

bool osd_common_t::execute_command(const char *command)
{
	if (strcmp(command, OSDCOMMAND_LIST_NETWORK_ADAPTERS) == 0)
	{
		osd_module *om = select_module_options(options(), OSD_NETDEV_PROVIDER);

		if (om->probe())
		{
			om->init(options());
			osd_list_network_adapters();
			om->exit();
		}

		return true;
	}
	else if (strcmp(command, OSDCOMMAND_LIST_MIDI_DEVICES) == 0)
	{
		osd_module *om = select_module_options(options(), OSD_MIDI_PROVIDER);
		midi_module *pm = select_module_options<midi_module *>(options(), OSD_MIDI_PROVIDER);

		if (om->probe())
		{
			om->init(options());
			pm->list_midi_devices();
			om->exit();
		}
		return true;
	}

	return false;

}

static void output_notifier_callback(const char *outname, int32_t value, void *param)
{
	static_cast<osd_common_t*>(param)->notify(outname, value);
}

void osd_common_t::init_subsystems()
{
	// monitors have to be initialized before video init
	m_monitor_module = select_module_options<monitor_module *>(options(), OSD_MONITOR_PROVIDER);
	assert(m_monitor_module != nullptr);
	m_monitor_module->init(options());

	if (!video_init())
	{
		video_exit();
		osd_printf_error("video_init: Initialization failed!\n\n\n");
		fflush(stderr);
		fflush(stdout);
		exit(-1);
	}

	m_keyboard_input = select_module_options<input_module *>(options(), OSD_KEYBOARDINPUT_PROVIDER);
	m_mouse_input = select_module_options<input_module *>(options(), OSD_MOUSEINPUT_PROVIDER);
	m_lightgun_input = select_module_options<input_module *>(options(), OSD_LIGHTGUNINPUT_PROVIDER);
	m_joystick_input = select_module_options<input_module *>(options(), OSD_JOYSTICKINPUT_PROVIDER);

	m_font_module = select_module_options<font_module *>(options(), OSD_FONT_PROVIDER);
	m_sound = select_module_options<sound_module *>(options(), OSD_SOUND_PROVIDER);
	m_sound->m_sample_rate = options().sample_rate();
	m_sound->m_audio_latency = options().audio_latency();

	m_debugger = select_module_options<debug_module *>(options(), OSD_DEBUG_PROVIDER);

	select_module_options<netdev_module *>(options(), OSD_NETDEV_PROVIDER);

	m_midi = select_module_options<midi_module *>(options(), OSD_MIDI_PROVIDER);

	m_output = select_module_options<output_module *>(options(), OSD_OUTPUT_PROVIDER);
	m_output->set_machine(&machine());
	machine().output().set_notifier(nullptr, output_notifier_callback, this);

	m_mod_man.init(options());

	input_init();
	// we need pause callbacks
	machine().add_notifier(MACHINE_NOTIFY_PAUSE, machine_notify_delegate(&osd_common_t::input_pause, this));
	machine().add_notifier(MACHINE_NOTIFY_RESUME, machine_notify_delegate(&osd_common_t::input_resume, this));
}

bool osd_common_t::video_init()
{
	return true;
}

bool osd_common_t::window_init()
{
	return true;
}

bool osd_common_t::no_sound()
{
	return (strcmp(options().sound(),"none")==0) ? true : false;
}

void osd_common_t::video_register()
{
}

bool osd_common_t::input_init()
{
	m_keyboard_input->input_init(machine());
	m_mouse_input->input_init(machine());
	m_lightgun_input->input_init(machine());
	m_joystick_input->input_init(machine());
	return true;
}

void osd_common_t::input_pause()
{
	m_keyboard_input->pause();
	m_mouse_input->pause();
	m_lightgun_input->pause();
	m_joystick_input->pause();
}

void osd_common_t::input_resume()
{
	m_keyboard_input->resume();
	m_mouse_input->resume();
	m_lightgun_input->resume();
	m_joystick_input->resume();
}

void osd_common_t::exit_subsystems()
{
	video_exit();
}

void osd_common_t::video_exit()
{
}

void osd_common_t::window_exit()
{
}

void osd_common_t::osd_exit()
{
	m_mod_man.exit();

	exit_subsystems();
}

void osd_common_t::video_options_add(const char *name, void *type)
{
	//m_video_options.add(name, type, false);
	m_video_names.push_back(core_strdup(name));
}
