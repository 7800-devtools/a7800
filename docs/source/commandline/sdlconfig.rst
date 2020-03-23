SDL-Specific Commandline Options
================================


This section contains configuration options that are specific to any build supported by SDL (including Windows where compiled as SDL instead of native).


In addition to the keys described in config.txt, the following additional keys are defined for SDL-specific versions of MAME:



Debugging options
-----------------

**-[no]oslog**

	Outputs the error.log data to the stderr TTY channel (usually the command line window MAME was started in). This can be used at	the same time as *-log* to output the log data to both targets as well. Default is OFF (*-nooslog*).

**-watchdog** *<duration>* / **-wdog** *<duration>*

	Enables an internal watchdog timer that will automatically kill the MAME process if more than *<duration>* seconds passes without a frame update. Keep in mind that some games sit for a while during load time without updating the screen, so *<duration>* should be long enough to cover that. 10-30 seconds on a modern system should be plenty in general. By default there is no watchdog.



Performance options
-------------------

**-numprocessors** *<auto|value>* / **-np** *<auto|value>*

	Specify the number of processors to use for work queues. Specifying	"*auto*" will use the value reported by the system or environment variable **OSDPROCESSORS**. To avoid abuse, this value is internally limited to 4 times the number of processors reported by the system. The default is "*auto*".

**-sdlvideofps**

	Enable output of benchmark data on the SDL video subsystem, including your system's video driver, X server (if applicable), and OpenGL stack in **-video opengl** mode.

**-bench** *[n]*

	Benchmark for [n] number of emulated seconds; implies the command string: **-str [n] -video none -sound none -nothrottle**. Default is OFF (*-nobench*)



Video options
-------------

**-video** *<soft|opengl|none>*

	Specifies which video subsystem to use for drawing.  The default for Mac OS X is '*opengl*' because OS X is guaranteed to have a compliant OpenGL stack.  The default on all other systems is '*soft*'.

**-numscreens** *<count>*

	Tells MAME how many output windows to create. For most games, a single output window is all you need, but some games originally used multiple screens. Each screen (up to 4) has its own independent settings for physical monitor, aspect ratio, resolution, and view, which can be set using the options below. The default is 1. 

**-[no]window** / **-[no]w**

	Run MAME in either a window or full screen. The default is OFF (*-nowindow*).

**-[no]maximize** / **-[no]max**

	Controls initial window size in windowed mode. If it is set on, the window will initially be set to the maximum supported size when you start MAME. If it is turned off, the window will start out at the smallest supported size. This option only has an effect when the **-window** option is used. The default is ON (*-maximize*).

**-[no]keepaspect** / **-[no]ka**

	Enables aspect ratio enforcement. When this option is on, the game's proper aspect ratio (generally 4:3 or 3:4) is enforced, so you get the game looking like it should. When running in a window with this option on, you can only resize the window to the proper aspect ratio, unless you are holding down the CONTROL key. By turning the option off, the aspect ratio is allowed to float. In full screen mode, this means that all games will stretch to the full screen size (even vertical games). In window mode, it means that you can freely resize the window without any constraints. The default is ON (*-keepaspect*).

**-[no]unevenstretch**

	Allow non-integer stretch factors allowing for great window sizing flexability.  The default is ON. (*-unevenstretch*)

**-[no]centerh**

	Center horizontally within the view area. Default is ON (*-centerh*).

**-[no]centerv**

	Center vertically within the view area. Default is ON (*-centerv*).

**-[no]waitvsync**

	Waits for the refresh period on your computer's monitor to finish before starting to draw video to your screen. If this option is off, MAME will just draw to the screen at any old time, even in the middle of a refresh cycle. This can cause "tearing" artifacts, where the top portion of the screen is out of sync with the bottom portion. Tearing is not noticeable on all games, and some people hate it more than others. However, if you turn this option on, you will waste more of your CPU cycles waiting for the proper time to draw, so you will see a performance hit. You should only need to turn this on in windowed mode. In full screen mode, it is only needed if **-triplebuffer** does not remove the tearing, in which case you should use **-notriplebuffer -waitvsync.** Note that support for this option depends entirely on your operating system and video drivers; in general it will not work in windowed mode so **-video opengl** and fullscreen give the greatest chance of success. The default is OFF (*-nowaitvsync*).

**-[no]syncrefresh**

	Enables speed throttling only to the refresh of your monitor. This means that the game's actual refresh rate is ignored; however, the sound code still attempts to keep up with the game's original refresh rate, so you may encounter sound problems. This option is intended mainly for those who have tweaked their video card's settings to provide carefully matched refresh rate options. The default is OFF (*-nosyncrefresh*).


Video soft-specific options
---------------------------

**-scalemode**

	Scale mode: none, async, yv12, yuy2, yv12x2, yuy2x2 (**-video soft** only). Default is '*none*'.



Video OpenGL-specific options
-----------------------------

**-[no]filter** / **-[no]flt**

	Enable bilinear filtering on the game screen graphics. When disabled, point filtering is applied, which is crisper but leads to scaling artifacts. If you don't like the filtered look, you are probably better off increasing the **-prescale** value rather than turning off filtering altogether. The default is ON (*-filter*).

**-prescale** *<amount>*

	Controls the size of the screen images when they are passed off to the graphics system for scaling. At the minimum setting of 1, the screen is rendered at its original resolution before being scaled. At higher settings, the screen is expanded by a factor of *<amount>* before being scaled. This produces a less blurry image at the expense of some speed and also increases the effective resolution of non-screen elements such as artwork and fonts. The default is *1*.

Video OpenGL debugging options
------------------------------
	
These 4 options are for compatibility in **-video opengl**.  If you report rendering artifacts you may be asked to try messing with them by the devs, but normally they should be left at their defaults which results in the best possible video performance.	

**-[no]gl_forcepow2texture**

	Always use only power-of-2 sized textures (default *off*)
	
**-[no]gl_notexturerect**

	Don't use OpenGL GL_ARB_texture_rectangle (default *on*)

**-[no]gl_vbo**

    Enable OpenGL VBO,  if available (default *on*)

**-[no]gl_pbo**

    Enable OpenGL PBO,  if available (default *on*)


Video OpenGL GLSL options
-------------------------
	

**-gl_glsl**

	Enable OpenGL GLSL, if available (default *off*)

**-gl_glsl_filter**

	Enable OpenGL GLSL filtering instead of FF filtering -- *0-plain, 1-bilinear* (default is *1*)

**-glsl_shader_mame0**

	Custom OpenGL GLSL shader set MAME bitmap 0  [todo: better details on usage at some point. See http://forums.bannister.org/ubbthreads.php?ubb=showflat&Number=100988#Post100988 ]

**-glsl_shader_mame1**

	Custom OpenGL GLSL shader set MAME bitmap 1

**-glsl_shader_mame2**

	Custom OpenGL GLSL shader set MAME bitmap 2

**-glsl_shader_mame3**

	Custom OpenGL GLSL shader set MAME bitmap 3

**-glsl_shader_mame4**

	Custom OpenGL GLSL shader set MAME bitmap 4

**-glsl_shader_mame5**

	Custom OpenGL GLSL shader set MAME bitmap 5

**-glsl_shader_mame6**

	Custom OpenGL GLSL shader set MAME bitmap 6

**-glsl_shader_mame7**

	Custom OpenGL GLSL shader set MAME bitmap 7

**-glsl_shader_mame8**

	Custom OpenGL GLSL shader set MAME bitmap 8

**-glsl_shader_mame9**

	Custom OpenGL GLSL shader set MAME bitmap 9


**-glsl_shader_screen0**

	Custom OpenGL GLSL shader screen bitmap 0

**-glsl_shader_screen1**

	Custom OpenGL GLSL shader screen bitmap 1

**-glsl_shader_screen2**

	Custom OpenGL GLSL shader screen bitmap 2

**-glsl_shader_screen3**

	Custom OpenGL GLSL shader screen bitmap 3

**-glsl_shader_screen4**

	Custom OpenGL GLSL shader screen bitmap 4

**-glsl_shader_screen5**

	Custom OpenGL GLSL shader screen bitmap 5

**-glsl_shader_screen6**

	Custom OpenGL GLSL shader screen bitmap 6

**-glsl_shader_screen7**

	Custom OpenGL GLSL shader screen bitmap 7

**-glsl_shader_screen8**

	Custom OpenGL GLSL shader screen bitmap 8

**-glsl_shader_screen9**

	Custom OpenGL GLSL shader screen bitmap 9

**-gl_glsl_vid_attr**

	Enable OpenGL GLSL handling of brightness and contrast. Better RGB game performance.  Default is *on*.



Per-window options
------------------

NOTE:  **Multiple Screens may fail to work correctly on some Macintosh machines as of right now.**

| **-screen** *<display>*
| **-screen0** *<display>*
| **-screen1** *<display>*
| **-screen2** *<display>*
| **-screen3** *<display>*
|
|	Specifies which physical monitor on your system you wish to have each window use by default. In order to use multiple windows, you must have increased the value of the **-numscreens** option. The name of each display in your system can be determined by running MAME with the -verbose option. The display names are typically in the format of a number from 1 to the number of connected monitors. The default value for these options is '*auto*', which means that the first window is placed on the first display, the second window on the second display, etc.
|
|	The **-screen0**, **-screen1**, **-screen2**, **-screen3** parameters apply to the specific window. The **-screen** parameter applies to all windows. The window-specific options override values from the all window option. 
|
|


| **-aspect** *<width:height>* / **-screen_aspect** *<num:den>*
| **-aspect0** *<width:height>*
| **-aspect1** *<width:height>*
| **-aspect2** *<width:height>*
| **-aspect3** *<width:height>*
|
|
|	Specifies the physical aspect ratio of the physical monitor for each window. In order to use multiple windows, you must have increased the value of the **-numscreens** option. The physical aspect ratio can be determined by measuring the width and height of the visible screen image and specifying them separated by a colon. The default value for these options is '*auto*', which means that MAME assumes the aspect ratio is proportional to the number of pixels in the desktop video mode for each monitor.
|
|	The **-aspect0**, **-aspect1**, **-aspect2**, **-aspect3** parameters apply to the specific window. The **-aspect** parameter applies to all windows. The window-specific options override values from the all window option.
|
|


| **-resolution** *<widthxheight[@refresh]>* / **-r** *<widthxheight[@refresh]>*
| **-resolution0** *<widthxheight[@refresh]>* / **-r0** *<widthxheight[@refresh]>*
| **-resolution1** *<widthxheight[@refresh]>* / **-r1** *<widthxheight[@refresh]>*
| **-resolution2** *<widthxheight[@refresh]>* / **-r2** *<widthxheight[@refresh]>*
| **-resolution3** *<widthxheight[@refresh]>* / **-r3** *<widthxheight[@refresh]>*
|
|	Specifies an exact resolution to run in. In full screen mode, MAME will try to use the specific resolution you request. The width and height are required; the refresh rate is optional. If omitted or set to 0, MAME will determine the mode automatically. For example, **-resolution 640x480** will force 640x480 resolution, but MAME is free to choose the refresh rate. Similarly, **-resolution 0x0@60** will force a 60Hz refresh rate, but allows MAME to choose the resolution. The string "*auto*" is also supported, and is equivalent to *0x0@0*. In window mode, this resolution is used as a maximum size for the window. This option requires the **-switchres** option. The default value for these options is '*auto*'.
|
|	The **-resolution0**, **-resolution1**, **-resolution2**, **-resolution3** parameters apply to the specific window. The -resolution parameter applies to all windows. The window-specific options override values from the all window option.
|
|


| **-view** *<viewname>*
| **-view0** *<viewname>*
| **-view1** *<viewname>*
| **-view2** *<viewname>*
| **-view3** *<viewname>*
|
|	Specifies the initial view setting for each window. The *<viewname>* does not need to be a perfect match; rather, it will select the first view whose name matches all the characters specified by *<viewname>*. For example, **-view native** will match the "*Native (15:14)*" view even though it is not a perfect match. The value '*auto*' is also supported, and requests that MAME perform a default selection. The default value for these options is '*auto*'.
|
|	The **-view0**, **-view1**, **-view2**, **-view3** parameters apply to the specific window. The **-view** parameter applies to all windows. The window-specific options override values from the all window option.
|
|



Full screen options
-------------------

**-[no]switchres**

	Enables resolution switching. This option is required for the **-resolution\*** options to switch resolutions in full screen mode. On modern video cards, there is little reason to switch resolutions unless you are trying to achieve the "exact" pixel resolutions of the original games, which requires significant tweaking. This option is also useful on LCD displays, since they run with a fixed resolution and switching resolutions on them is just silly. The default is OFF (*-noswitchres*).


Sound options
-------------

**-sound** *<sdl|none>*

	Specifies which sound subsystem to use. '*none*' disables sound altogether. The default is *sdl*.

**-audio_latency** *<value>*

	This controls the amount of latency built into the audio streaming. By default MAME tries to keep the audio buffer between 1/5 and 2/5 full. On some systems, this is pushing it too close to the edge, and you get poor sound sometimes. The latency parameter controls the lower threshold. The default is *1* (meaning lower=1/5 and upper=2/5). Set it to 2 (**-audio_latency 2**) to keep the sound buffer between 2/5 and 3/5 full. If you crank it up to 4, you can *definitely* notice audio lag.


SDL Keyboard Mapping
--------------------

**-keymap**

	Enable keymap. Default is OFF (*-nokeymap*)

**-keymap_file** *<file>*
	
	Keymap Filename. Default is '*keymap.dat*'.

**-uimodekey** *<key>*
	
	Key to toggle keyboard mode. Default is '*SCRLOCK*'



SDL Joystick Mapping
--------------------

**-joy_idx1** *<name>*

Name of joystick mapped to joystick #1, default is *auto*.

**-joy_idx2** *<name>*

Name of joystick mapped to joystick #2, default is *auto*.

**-joy_idx3** *<name>*

Name of joystick mapped to joystick #3, default is *auto*.

**-joy_idx4** *<name>*

Name of joystick mapped to joystick #4, default is *auto*.

**-joy_idx5** *<name>*

Name of joystick mapped to joystick #5, default is *auto*.

**-joy_idx6** *<name>*

Name of joystick mapped to joystick #6, default is *auto*.

**-joy_idx7** *<name>*

Name of joystick mapped to joystick #7, default is *auto*.

**-joy_idx8** *<name>*

Name of joystick mapped to joystick #8, default is *auto*.

**-sixaxis**

	Use special handling for PS3 Sixaxis controllers. Default is OFF (*-nosixaxis*)



SDL Lowlevel driver options
---------------------------

**-videodriver** *<driver>*

	SDL video driver to use ('x11', 'directfb', ... or '*auto*' for SDL default)

**-audiodriver** *<driver>*

	SDL audio driver to use ('alsa', 'arts', ... or '*auto*' for SDL default)

**-gl_lib** *<driver>*

	Alternative **libGL.so** to use; '*auto*' for system default


