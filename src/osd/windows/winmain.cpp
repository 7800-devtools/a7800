// license:BSD-3-Clause
// copyright-holders:Aaron Giles
//============================================================
//
//  winmain.c - Win32 main program
//
//============================================================

// only for oslog callback
#include <functional>

// standard windows headers
#include <windows.h>
#include <commctrl.h>
#include <mmsystem.h>
#include <tchar.h>
#include <io.h>

// standard C headers
#include <ctype.h>
#include <stdarg.h>

// MAME headers
#include "emu.h"
#include "emuopts.h"
#include "strconv.h"

// MAMEOS headers
#include "winmain.h"
#include "window.h"
#include "winutf8.h"
#include "winutil.h"
#include "winfile.h"
#include "modules/diagnostics/diagnostics_module.h"
#include "modules/monitor/monitor_common.h"

#if !WINAPI_FAMILY_PARTITION(WINAPI_PARTITION_DESKTOP)
#include <wrl/client.h>
using namespace Windows::Storage;
using namespace Platform;
using namespace Windows::ApplicationModel;
using namespace Windows::ApplicationModel::Core;
using namespace Windows::UI::Popups;
#endif

#define DEBUG_SLOW_LOCKS    0

//**************************************************************************
//  MACROS
//**************************************************************************

#ifdef UNICODE
#define UNICODE_POSTFIX "W"
#else
#define UNICODE_POSTFIX "A"
#endif

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

#if WINAPI_FAMILY_PARTITION(WINAPI_PARTITION_DESKTOP)

//============================================================
//  winui_output_error
//============================================================

class winui_output_error : public osd_output
{
public:
	virtual void output_callback(osd_output_channel channel, const char *msg, va_list args) override
	{
		if (channel == OSD_OUTPUT_CHANNEL_ERROR)
		{
			char buffer[1024];

			// if we are in fullscreen mode, go to windowed mode
			if ((video_config.windowed == 0) && !osd_common_t::s_window_list.empty())
				winwindow_toggle_full_screen();

			vsnprintf(buffer, ARRAY_LENGTH(buffer), msg, args);
			win_message_box_utf8(!osd_common_t::s_window_list.empty() ? std::static_pointer_cast<win_window_info>(osd_common_t::s_window_list.front())->platform_window() : nullptr, buffer, emulator_info::get_appname(), MB_OK);
		}
		else
			chain_output(channel, msg, args);
	}
};

#else

//============================================================
//  winuniversal_output_error
//============================================================

class winuniversal_output_error : public osd_output
{
public:
	virtual void output_callback(osd_output_channel channel, const char *msg, va_list args) override
	{
		char buffer[2048];
		if (channel == OSD_OUTPUT_CHANNEL_ERROR)
		{
			vsnprintf(buffer, ARRAY_LENGTH(buffer), msg, args);

			std::wstring wcbuffer(osd::text::to_wstring(buffer));
			std::wstring wcappname(osd::text::to_wstring(emulator_info::get_appname()));

			auto dlg = ref new MessageDialog(ref new Platform::String(wcbuffer.data()), ref new Platform::String(wcbuffer.data()));
			dlg->ShowAsync();
		}
		else if (channel == OSD_OUTPUT_CHANNEL_VERBOSE)
		{
			vsnprintf(buffer, ARRAY_LENGTH(buffer), msg, args);
			std::wstring wcbuffer = osd::text::to_wstring(buffer);
			OutputDebugString(wcbuffer.c_str());

			// Chain to next anyway
			chain_output(channel, msg, args);
		}
		else
			chain_output(channel, msg, args);
	}
};

#endif


//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

// this line prevents globbing on the command line
int _CRT_glob = 0;

//**************************************************************************
//  LOCAL VARIABLES
//**************************************************************************

#if WINAPI_FAMILY_PARTITION(WINAPI_PARTITION_DESKTOP)
static int timeresult = !TIMERR_NOERROR;
static TIMECAPS timecaps;
#endif

static running_machine *g_current_machine;


//**************************************************************************
//  FUNCTION PROTOTYPES
//**************************************************************************

static BOOL WINAPI control_handler(DWORD type);
static int is_double_click_start(int argc);


//**************************************************************************
//  OPTIONS
//**************************************************************************

// struct definitions
const options_entry windows_options::s_option_entries[] =
{
	// performance options
	{ nullptr,                                        nullptr,    OPTION_HEADER,     "WINDOWS PERFORMANCE OPTIONS" },
	{ WINOPTION_PRIORITY "(-15-1)",                   "0",        OPTION_INTEGER,    "thread priority for the main game thread; range from -15 to 1" },
	{ WINOPTION_PROFILE,                              "0",        OPTION_INTEGER,    "enables profiling, specifying the stack depth to track" },

	// video options
	{ nullptr,                                        nullptr,    OPTION_HEADER,     "WINDOWS VIDEO OPTIONS" },
	{ WINOPTION_MENU,                                 "0",        OPTION_BOOLEAN,    "enables menu bar if available by UI implementation" },

	// post-processing options
	{ nullptr,                                                  nullptr,             OPTION_HEADER,     "DIRECT3D POST-PROCESSING OPTIONS" },
	{ WINOPTION_HLSLPATH,                                       "hlsl",              OPTION_STRING,     "path to hlsl files" },
	{ WINOPTION_HLSL_ENABLE";hlsl",                             "0",                 OPTION_BOOLEAN,    "enables HLSL post-processing (PS3.0 required)" },
	{ WINOPTION_HLSL_OVERSAMPLING,                              "0",                 OPTION_BOOLEAN,    "enables HLSL oversampling" },
	{ WINOPTION_HLSL_WRITE,                                     OSDOPTVAL_AUTO,      OPTION_STRING,     "enables HLSL AVI writing (huge disk bandwidth suggested)" },
	{ WINOPTION_HLSL_SNAP_WIDTH,                                "2048",              OPTION_STRING,     "HLSL upscaled-snapshot width" },
	{ WINOPTION_HLSL_SNAP_HEIGHT,                               "1536",              OPTION_STRING,     "HLSL upscaled-snapshot height" },
	{ WINOPTION_SHADOW_MASK_TILE_MODE,                          "0",                 OPTION_INTEGER,    "shadow mask tile mode (0 for screen based, 1 for source based)" },
	{ WINOPTION_SHADOW_MASK_ALPHA";fs_shadwa(0.0-1.0)",         "0.0",               OPTION_FLOAT,      "shadow mask alpha-blend value (1.0 is fully blended, 0.0 is no mask)" },
	{ WINOPTION_SHADOW_MASK_TEXTURE";fs_shadwt(0.0-1.0)",       "shadow-mask.png",   OPTION_STRING,     "shadow mask texture name" },
	{ WINOPTION_SHADOW_MASK_COUNT_X";fs_shadww",                "6",                 OPTION_INTEGER,    "shadow mask tile width, in screen dimensions" },
	{ WINOPTION_SHADOW_MASK_COUNT_Y";fs_shadwh",                "4",                 OPTION_INTEGER,    "shadow mask tile height, in screen dimensions" },
	{ WINOPTION_SHADOW_MASK_USIZE";fs_shadwu(0.0-1.0)",         "0.1875",            OPTION_FLOAT,      "shadow mask texture width, in U/V dimensions" },
	{ WINOPTION_SHADOW_MASK_VSIZE";fs_shadwv(0.0-1.0)",         "0.25",              OPTION_FLOAT,      "shadow mask texture height, in U/V dimensions" },
	{ WINOPTION_SHADOW_MASK_UOFFSET";fs_shadwou(-1.0-1.0)",     "0.0",               OPTION_FLOAT,      "shadow mask texture offset, in U direction" },
	{ WINOPTION_SHADOW_MASK_VOFFSET";fs_shadwov(-1.0-1.0)",     "0.0",               OPTION_FLOAT,      "shadow mask texture offset, in V direction" },
	{ WINOPTION_DISTORTION";fs_dist(-1.0-1.0)",                 "0.0",               OPTION_FLOAT,      "screen distortion amount" },
	{ WINOPTION_CUBIC_DISTORTION";fs_cubedist(-1.0-1.0)",       "0.0",               OPTION_FLOAT,      "screen cubic distortion amount" },
	{ WINOPTION_DISTORT_CORNER";fs_distc(0.0-1.0)",             "0.0",               OPTION_FLOAT,      "screen distort corner amount" },
	{ WINOPTION_ROUND_CORNER";fs_rndc(0.0-1.0)",                "0.0",               OPTION_FLOAT,      "screen round corner amount" },
	{ WINOPTION_SMOOTH_BORDER";fs_smob(0.0-1.0)",               "0.0",               OPTION_FLOAT,      "screen smooth border amount" },
	{ WINOPTION_REFLECTION";fs_ref(0.0-1.0)",                   "0.0",               OPTION_FLOAT,      "screen reflection amount" },
	{ WINOPTION_VIGNETTING";fs_vig(0.0-1.0)",                   "0.0",               OPTION_FLOAT,      "image vignetting amount" },
	/* Beam-related values below this line*/
	{ WINOPTION_SCANLINE_AMOUNT";fs_scanam(0.0-4.0)",           "0.0",               OPTION_FLOAT,      "overall alpha scaling value for scanlines" },
	{ WINOPTION_SCANLINE_SCALE";fs_scansc(0.0-4.0)",            "1.0",               OPTION_FLOAT,      "overall height scaling value for scanlines" },
	{ WINOPTION_SCANLINE_HEIGHT";fs_scanh(0.0-4.0)",            "1.0",               OPTION_FLOAT,      "individual height scaling value for scanlines" },
	{ WINOPTION_SCANLINE_VARIATION";fs_scanv(0.0-4.0)",         "1.0",               OPTION_FLOAT,      "individual height varying value for scanlines" },
	{ WINOPTION_SCANLINE_BRIGHT_SCALE";fs_scanbs(0.0-2.0)",     "1.0",               OPTION_FLOAT,      "overall brightness scaling value for scanlines (multiplicative)" },
	{ WINOPTION_SCANLINE_BRIGHT_OFFSET";fs_scanbo(0.0-1.0)",    "0.0",               OPTION_FLOAT,      "overall brightness offset value for scanlines (additive)" },
	{ WINOPTION_SCANLINE_JITTER";fs_scanjt(0.0-4.0)",           "0.0",               OPTION_FLOAT,      "overall interlace jitter scaling value for scanlines" },
	{ WINOPTION_HUM_BAR_ALPHA";fs_humba(0.0-1.0)",              "0.0",               OPTION_FLOAT,      "overall alpha scaling value for hum bar" },
	{ WINOPTION_DEFOCUS";fs_focus",                             "0.0,0.0",           OPTION_STRING,     "overall defocus value in screen-relative coords" },
	{ WINOPTION_CONVERGE_X";fs_convx",                          "0.0,0.0,0.0",       OPTION_STRING,     "convergence in screen-relative X direction" },
	{ WINOPTION_CONVERGE_Y";fs_convy",                          "0.0,0.0,0.0",       OPTION_STRING,     "convergence in screen-relative Y direction" },
	{ WINOPTION_RADIAL_CONVERGE_X";fs_rconvx",                  "0.0,0.0,0.0",       OPTION_STRING,     "radial convergence in screen-relative X direction" },
	{ WINOPTION_RADIAL_CONVERGE_Y";fs_rconvy",                  "0.0,0.0,0.0",       OPTION_STRING,     "radial convergence in screen-relative Y direction" },
	/* RGB colorspace convolution below this line */
	{ WINOPTION_RED_RATIO";fs_redratio",                        "1.0,0.0,0.0",       OPTION_STRING,     "red output signal generated by input signal" },
	{ WINOPTION_GRN_RATIO";fs_grnratio",                        "0.0,1.0,0.0",       OPTION_STRING,     "green output signal generated by input signal" },
	{ WINOPTION_BLU_RATIO";fs_bluratio",                        "0.0,0.0,1.0",       OPTION_STRING,     "blue output signal generated by input signal" },
	{ WINOPTION_SATURATION";fs_sat(0.0-4.0)",                   "1.0",               OPTION_FLOAT,      "saturation scaling value" },
	{ WINOPTION_OFFSET";fs_offset",                             "0.0,0.0,0.0",       OPTION_STRING,     "signal offset value (additive)" },
	{ WINOPTION_SCALE";fs_scale",                               "1.0,1.0,1.0",       OPTION_STRING,     "signal scaling value (multiplicative)" },
	{ WINOPTION_POWER";fs_power",                               "1.0,1.0,1.0",       OPTION_STRING,     "signal power value (exponential)" },
	{ WINOPTION_FLOOR";fs_floor",                               "0.0,0.0,0.0",       OPTION_STRING,     "signal floor level" },
	{ WINOPTION_PHOSPHOR";fs_phosphor",                         "0.0,0.0,0.0",       OPTION_STRING,     "phosphorescence decay rate (0.0 is instant, 1.0 is forever)" },
	/* NTSC simulation below this line */
	{ nullptr,                                                  nullptr,             OPTION_HEADER,     "NTSC POST-PROCESSING OPTIONS" },
	{ WINOPTION_YIQ_ENABLE";yiq",                               "0",                 OPTION_BOOLEAN,    "enables YIQ-space HLSL post-processing" },
	{ WINOPTION_YIQ_JITTER";yiqj",                              "0.0",               OPTION_FLOAT,      "Jitter for the NTSC signal processing" },
	{ WINOPTION_YIQ_CCVALUE";yiqcc",                            "3.57954545",        OPTION_FLOAT,      "Color Carrier frequency for NTSC signal processing" },
	{ WINOPTION_YIQ_AVALUE";yiqa",                              "0.5",               OPTION_FLOAT,      "A value for NTSC signal processing" },
	{ WINOPTION_YIQ_BVALUE";yiqb",                              "0.5",               OPTION_FLOAT,      "B value for NTSC signal processing" },
	{ WINOPTION_YIQ_OVALUE";yiqo",                              "0.0",               OPTION_FLOAT,      "Outgoing Color Carrier phase offset for NTSC signal processing" },
	{ WINOPTION_YIQ_PVALUE";yiqp",                              "1.0",               OPTION_FLOAT,      "Incoming Pixel Clock scaling value for NTSC signal processing" },
	{ WINOPTION_YIQ_NVALUE";yiqn",                              "1.0",               OPTION_FLOAT,      "Y filter notch width for NTSC signal processing" },
	{ WINOPTION_YIQ_YVALUE";yiqy",                              "6.0",               OPTION_FLOAT,      "Y filter cutoff frequency for NTSC signal processing" },
	{ WINOPTION_YIQ_IVALUE";yiqi",                              "1.2",               OPTION_FLOAT,      "I filter cutoff frequency for NTSC signal processing" },
	{ WINOPTION_YIQ_QVALUE";yiqq",                              "0.6",               OPTION_FLOAT,      "Q filter cutoff frequency for NTSC signal processing" },
	{ WINOPTION_YIQ_SCAN_TIME";yiqsc",                          "52.6",              OPTION_FLOAT,      "Horizontal scanline duration for NTSC signal processing (in usec)" },
	{ WINOPTION_YIQ_PHASE_COUNT";yiqpc",                        "2",                 OPTION_INTEGER,    "Phase Count value for NTSC signal processing" },
	/* Vector simulation below this line */
	{ nullptr,                                                  nullptr,             OPTION_HEADER,     "VECTOR POST-PROCESSING OPTIONS" },
	{ WINOPTION_VECTOR_BEAM_SMOOTH";vecsmooth",                 "0.0",               OPTION_FLOAT,      "The vector beam smoothness" },
	{ WINOPTION_VECTOR_LENGTH_SCALE";vecscale",                 "0.5",               OPTION_FLOAT,      "The maximum vector attenuation" },
	{ WINOPTION_VECTOR_LENGTH_RATIO";vecratio",                 "0.5",               OPTION_FLOAT,      "The minimum vector length (vector length to screen size ratio) that is affected by the attenuation" },
	/* Bloom below this line */
	{ nullptr,                                                  nullptr,             OPTION_HEADER,     "BLOOM POST-PROCESSING OPTIONS" },
	{ WINOPTION_BLOOM_BLEND_MODE,                               "0",                 OPTION_INTEGER,    "bloom blend mode (0 for brighten, 1 for darken)" },
	{ WINOPTION_BLOOM_SCALE,                                    "0.0",               OPTION_FLOAT,      "Intensity factor for bloom" },
	{ WINOPTION_BLOOM_OVERDRIVE,                                "1.0,1.0,1.0",       OPTION_STRING,     "Overdrive factor for bloom" },
	{ WINOPTION_BLOOM_LEVEL0_WEIGHT,                            "1.0",               OPTION_FLOAT,      "Bloom level 0 weight (full-size target)" },
	{ WINOPTION_BLOOM_LEVEL1_WEIGHT,                            "0.64",              OPTION_FLOAT,      "Bloom level 1 weight (1/4 smaller that level 0 target)" },
	{ WINOPTION_BLOOM_LEVEL2_WEIGHT,                            "0.32",              OPTION_FLOAT,      "Bloom level 2 weight (1/4 smaller that level 1 target)" },
	{ WINOPTION_BLOOM_LEVEL3_WEIGHT,                            "0.16",              OPTION_FLOAT,      "Bloom level 3 weight (1/4 smaller that level 2 target)" },
	{ WINOPTION_BLOOM_LEVEL4_WEIGHT,                            "0.08",              OPTION_FLOAT,      "Bloom level 4 weight (1/4 smaller that level 3 target)" },
	{ WINOPTION_BLOOM_LEVEL5_WEIGHT,                            "0.06",              OPTION_FLOAT,      "Bloom level 5 weight (1/4 smaller that level 4 target)" },
	{ WINOPTION_BLOOM_LEVEL6_WEIGHT,                            "0.04",              OPTION_FLOAT,      "Bloom level 6 weight (1/4 smaller that level 5 target)" },
	{ WINOPTION_BLOOM_LEVEL7_WEIGHT,                            "0.02",              OPTION_FLOAT,      "Bloom level 7 weight (1/4 smaller that level 6 target)" },
	{ WINOPTION_BLOOM_LEVEL8_WEIGHT,                            "0.01",              OPTION_FLOAT,      "Bloom level 8 weight (1/4 smaller that level 7 target)" },

	// full screen options
	{ nullptr,                                        nullptr,    OPTION_HEADER,     "FULL SCREEN OPTIONS" },
	{ WINOPTION_TRIPLEBUFFER ";tb",                   "0",        OPTION_BOOLEAN,    "enables triple buffering" },
	{ WINOPTION_FULLSCREENBRIGHTNESS ";fsb(0.1-2.0)", "1.0",      OPTION_FLOAT,      "brightness value in full screen mode" },
	{ WINOPTION_FULLSCREENCONTRAST ";fsc(0.1-2.0)",   "1.0",      OPTION_FLOAT,      "contrast value in full screen mode" },
	{ WINOPTION_FULLSCREENGAMMA ";fsg(0.1-3.0)",      "1.0",      OPTION_FLOAT,      "gamma value in full screen mode" },

	// input options
	{ nullptr,                                        nullptr,    OPTION_HEADER,     "INPUT DEVICE OPTIONS" },
	{ WINOPTION_GLOBAL_INPUTS,                        "0",        OPTION_BOOLEAN,    "enables global inputs" },
	{ WINOPTION_DUAL_LIGHTGUN ";dual",                "0",        OPTION_BOOLEAN,    "enables dual lightgun input" },

	{ nullptr }
};

//**************************************************************************
//  MAIN ENTRY POINT
//**************************************************************************

#if WINAPI_FAMILY_PARTITION(WINAPI_PARTITION_DESKTOP)

//============================================================
//  utf8_main
//============================================================

int main(std::vector<std::string> &args)
{
	// use small output buffers on non-TTYs (i.e. pipes)
	if (!isatty(fileno(stdout)))
		setvbuf(stdout, (char *) nullptr, _IOFBF, 64);
	if (!isatty(fileno(stderr)))
		setvbuf(stderr, (char *) nullptr, _IOFBF, 64);

	// initialize common controls
	InitCommonControls();

	// set a handler to catch ctrl-c
	SetConsoleCtrlHandler(control_handler, TRUE);

	// Initialize crash diagnostics
	diagnostics_module::get_instance()->init_crash_diagnostics();

	// parse config and cmdline options
	DWORD result;
	{
		windows_options options;
		windows_osd_interface osd(options);
		// if we're a GUI app, out errors to message boxes
		// Initialize this after the osd interface so that we are first in the
		// output order
		winui_output_error winerror;
		if (win_is_gui_application() || is_double_click_start(args.size()))
		{
			// if we are a GUI app, output errors to message boxes
			osd_output::push(&winerror);
			// make sure any console window that opened on our behalf is nuked
			FreeConsole();
		}
		osd.register_options();
		result = emulator_info::start_frontend(options, osd, args);
		osd_output::pop(&winerror);
	}

	return result;
}

//============================================================
//  control_handler
//============================================================

static BOOL WINAPI control_handler(DWORD type)
{
	// indicate to the user that we detected something
	switch (type)
	{
	case CTRL_C_EVENT:          fprintf(stderr, "Caught Ctrl+C");                   break;
	case CTRL_BREAK_EVENT:      fprintf(stderr, "Caught Ctrl+break");               break;
	case CTRL_CLOSE_EVENT:      fprintf(stderr, "Caught console close");            break;
	case CTRL_LOGOFF_EVENT:     fprintf(stderr, "Caught logoff");                   break;
	case CTRL_SHUTDOWN_EVENT:   fprintf(stderr, "Caught shutdown");                 break;
	default:                    fprintf(stderr, "Caught unexpected console event"); break;
	}

	// if we don't have a machine yet, or if we are handling ctrl+c/ctrl+break,
	// just terminate hard, without throwing or handling any atexit stuff
	if (g_current_machine == nullptr || type == CTRL_C_EVENT || type == CTRL_BREAK_EVENT)
	{
		fprintf(stderr, ", exiting\n");
		TerminateProcess(GetCurrentProcess(), EMU_ERR_FATALERROR);
	}

	// all other situations attempt to do a clean exit
	else
	{
		fprintf(stderr, ", exit requested\n");
		g_current_machine->schedule_exit();
	}

	// in all cases we handled it
	return TRUE;
}

#else

// The main function is only used to initialize our IFrameworkView class.
[Platform::MTAThread]
int main(Platform::Array<Platform::String^>^ args)
{
	auto direct3DApplicationSource = ref new MameViewSource();
	CoreApplication::Run(direct3DApplicationSource);
	return 0;
}

MameMainApp::MameMainApp()
{
	// Turn off application view scaling so XBOX gets full screen
	Windows::UI::ViewManagement::ApplicationViewScaling::TrySetDisableLayoutScaling(true);
}

void MameMainApp::Initialize(Windows::ApplicationModel::Core::CoreApplicationView^ applicationView)
{
	// Register event handlers for app lifecycle.
}

// Called when the CoreWindow object is created (or re-created).
void MameMainApp::SetWindow(Windows::UI::Core::CoreWindow^ window)
{
	// Attach event handlers on the window for input, etc.
}

// Initializes scene resources, or loads a previously saved app state.
void MameMainApp::Load(Platform::String^ entryPoint)
{
}

void MameMainApp::Run()
{
	// use small output buffers on non-TTYs (i.e. pipes)
	if (!isatty(fileno(stdout)))
		setvbuf(stdout, (char *) nullptr, _IOFBF, 64);
	if (!isatty(fileno(stderr)))
		setvbuf(stderr, (char *) nullptr, _IOFBF, 64);

	// parse config and cmdline options
	m_options = std::make_unique<windows_options>();
	m_osd = std::make_unique<windows_osd_interface>(*m_options.get());

	// Since we're a GUI app, out errors to message boxes
	// Initialize this after the osd interface so that we are first in the
	// output order
	winuniversal_output_error winerror;
	osd_output::push(&winerror);

	m_osd->register_options();

	// To satisfy the latter things, pass in the module path name
	char exe_path[MAX_PATH];
	GetModuleFileNameA(nullptr, exe_path, MAX_PATH);
	char* args[3] = { exe_path, (char*)"-verbose", (char*)"-mouse" };

	DWORD result = emulator_info::start_frontend(*m_options.get(), *m_osd.get(), ARRAY_LENGTH(args), args);
	osd_output::pop(&winerror);
}

// Required for IFrameworkView.
void MameMainApp::Uninitialize()
{
	// Terminate events do not cause Uninitialize to be called. It will be called if your IFrameworkView
	// class is torn down while the app is in the foreground.
}

IFrameworkView^ MameViewSource::CreateView()
{
	return ref new MameMainApp();
}

#endif


//============================================================
//  windows_options
//============================================================

windows_options::windows_options()
: osd_options()
{
	add_entries(s_option_entries);
#if !WINAPI_FAMILY_PARTITION(WINAPI_PARTITION_DESKTOP)
	String^ path = ApplicationData::Current->LocalFolder->Path + L"\\";
	set_default_value(OPTION_INIPATH, (osd::text::from_wstring((LPCWSTR)path->Data()) + ";" + ini_path()).c_str());
	set_default_value(OPTION_CFG_DIRECTORY, (osd::text::from_wstring((LPCWSTR)path->Data()) +  cfg_directory()).c_str());
	set_default_value(OPTION_NVRAM_DIRECTORY, (osd::text::from_wstring((LPCWSTR)path->Data()) + nvram_directory()).c_str());
	set_default_value(OPTION_INPUT_DIRECTORY, (osd::text::from_wstring((LPCWSTR)path->Data()) + input_directory()).c_str());
	set_default_value(OPTION_STATE_DIRECTORY, (osd::text::from_wstring((LPCWSTR)path->Data()) + state_directory()).c_str());
	set_default_value(OPTION_SNAPSHOT_DIRECTORY, (osd::text::from_wstring((LPCWSTR)path->Data()) + snapshot_directory()).c_str());
	set_default_value(OPTION_DIFF_DIRECTORY, (osd::text::from_wstring((LPCWSTR)path->Data()) + diff_directory()).c_str());
	set_default_value(OPTION_COMMENT_DIRECTORY, (osd::text::from_wstring((LPCWSTR)path->Data()) + comment_directory()).c_str());

	set_default_value(OPTION_HOMEPATH, osd::text::from_wstring((LPCWSTR)path->Data()).c_str());
	set_default_value(OPTION_MEDIAPATH, (osd::text::from_wstring((LPCWSTR)path->Data()) + media_path()).c_str());
#endif
}


//============================================================
//  output_oslog
//============================================================

void windows_osd_interface::output_oslog(const char *buffer)
{
	if (IsDebuggerPresent())
		win_output_debug_string_utf8(buffer);
}


//============================================================
//  constructor
//============================================================

windows_osd_interface::windows_osd_interface(windows_options &options)
	: osd_common_t(options)
	, m_options(options)
{
}


//============================================================
//  destructor
//============================================================

windows_osd_interface::~windows_osd_interface()
{
}


//============================================================
//  video_register
//============================================================

void windows_osd_interface::video_register()
{
	video_options_add("gdi", nullptr);
	video_options_add("d3d", nullptr);
#if USE_OPENGL
	video_options_add("opengl", nullptr);
#endif
	video_options_add("bgfx", nullptr);
	//video_options_add("auto", nullptr); // making d3d video default one
}

//============================================================
//  init
//============================================================

void windows_osd_interface::init(running_machine &machine)
{
	// call our parent
	osd_common_t::init(machine);

	const char *stemp;
	windows_options &options = downcast<windows_options &>(machine.options());

	// determine if we are benchmarking, and adjust options appropriately
	int bench = options.bench();
	if (bench > 0)
	{
		options.set_value(OPTION_THROTTLE, false, OPTION_PRIORITY_MAXIMUM);
		options.set_value(OSDOPTION_SOUND, "none", OPTION_PRIORITY_MAXIMUM);
		options.set_value(OSDOPTION_VIDEO, "none", OPTION_PRIORITY_MAXIMUM);
		options.set_value(OPTION_SECONDS_TO_RUN, bench, OPTION_PRIORITY_MAXIMUM);
	}

	// determine if we are profiling, and adjust options appropriately
	int profile = options.profile();
	if (profile > 0)
	{
		options.set_value(OPTION_THROTTLE, false, OPTION_PRIORITY_MAXIMUM);
		options.set_value(OSDOPTION_NUMPROCESSORS, 1, OPTION_PRIORITY_MAXIMUM);
	}

	// thread priority
	if (!(machine.debug_flags & DEBUG_FLAG_OSD_ENABLED))
		SetThreadPriority(GetCurrentThread(), options.priority());

	// get number of processors
	stemp = options.numprocessors();

	osd_num_processors = 0;

	if (strcmp(stemp, "auto") != 0)
	{
		osd_num_processors = atoi(stemp);
		if (osd_num_processors < 1)
		{
			osd_printf_warning("Warning: numprocessors < 1 doesn't make much sense. Assuming auto ...\n");
			osd_num_processors = 0;
		}
	}

	// initialize the subsystems
	osd_common_t::init_subsystems();

	// notify listeners of screen configuration
	for (auto info : osd_common_t::s_window_list)
	{
		machine.output().set_value(string_format("Orientation(%s)", info->monitor()->devicename()).c_str(), std::static_pointer_cast<win_window_info>(info)->m_targetorient);
	}

	// hook up the debugger log
	if (options.oslog())
	{
		using namespace std::placeholders;
		machine.add_logerror_callback(std::bind(&windows_osd_interface::output_oslog, this, _1));
	}

#if WINAPI_FAMILY_PARTITION(WINAPI_PARTITION_DESKTOP)
	// crank up the multimedia timer resolution to its max
	// this gives the system much finer timeslices
	timeresult = timeGetDevCaps(&timecaps, sizeof(timecaps));
	if (timeresult == TIMERR_NOERROR)
		timeBeginPeriod(timecaps.wPeriodMin);
#endif

	// create and start the profiler
	if (profile > 0)
	{
		diagnostics_module::get_instance()->start_profiler(1000, profile - 1);
	}

	// initialize sockets
	win_init_sockets();

	// note the existence of a machine
	g_current_machine = &machine;
}


//============================================================
//  osd_exit
//============================================================

void windows_osd_interface::osd_exit()
{
	// no longer have a machine
	g_current_machine = nullptr;

	// cleanup sockets
	win_cleanup_sockets();

	osd_common_t::osd_exit();

	// stop the profiler
	diagnostics_module::get_instance()->stop_profiler();
	diagnostics_module::get_instance()->print_profiler_results();

#if WINAPI_FAMILY_PARTITION(WINAPI_PARTITION_DESKTOP)
	// restore the timer resolution
	if (timeresult == TIMERR_NOERROR)
		timeEndPeriod(timecaps.wPeriodMin);
#endif

	// one last pass at events
	winwindow_process_events(machine(), false, false);
}


#if WINAPI_FAMILY_PARTITION(WINAPI_PARTITION_DESKTOP)

//============================================================
//  check_for_double_click_start
//============================================================

static int is_double_click_start(int argc)
{
	STARTUPINFO startup_info = { sizeof(STARTUPINFO) };

	// determine our startup information
	GetStartupInfo(&startup_info);

	// try to determine if MAME was simply double-clicked
	return (argc <= 1 && startup_info.dwFlags && !(startup_info.dwFlags & STARTF_USESTDHANDLES));
}

#endif // WINAPI_FAMILY_PARTITION(WINAPI_PARTITION_DESKTOP)

