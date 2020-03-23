Windows-Specific Commandline Options
====================================

This section contains configuration options that are specific to the native (non-SDL) Windows version of MAME.



Debugging options
-----------------

**-[no]oslog**

	Outputs the error.log data to the Windows debugger. This can be used at	the same time as -log to output the log data to both targets as well. Default is OFF (*-nooslog*).

**-watchdog** *<duration>* / **-wdog** *<duration>*

	Enables an internal watchdog timer that will automatically kill the MAME process if more than *<duration>* seconds passes without a frame update. Keep in mind that some games sit for a while during load time without updating the screen, so *<duration>* should be long enough to cover that. 10-30 seconds on a modern system should be plenty in general. By default there is no watchdog.

**-debugger_font** *<fontname>* / **-dfont** *<fontname>*

	Specifies the name of the font to use for debugger windows. By default,	the font is Lucida Console.

**-debugger_font_size** *<points>* / **-dfontsize** *<points>*

	Specifies the size of the font to use for debugger windows, in points. By default, this is set to 9pt.



Performance options
-------------------

**-priority** *<priority>*

	Sets the thread priority for the MAME threads. By default the priority is left alone to guarantee proper cooperation with other applications. The valid range is -15 to 1, with 1 being the highest priority. The default is *0* (*NORMAL priority*).

**-numprocessors** *<auto|value>* / **-np** *<auto|value>*

	Specify the number of processors to use for work queues. Specifying "*auto*" will use the value reported by the system or environment variable **OSDPROCESSORS**. To avoid abuse, this value is internally limited to 4 times the number of processors reported by the system. The default is "*auto*".

**-profile** *[n]*

        Enables profiling, specifying the stack depth of *[n]* to track.

**-bench** *[n]*

        Benchmark for *[n]* number of emulated seconds; implies the command string:

        **-str [n] -video none -sound none -nothrottle**



Video options
-------------

**-video** *<bgfx|gdi|d3d|none>*

	Specifies which video subsystem to use for drawing. Using '*bgfx*' specifies the new hardware accelerated renderer. By specifying '*gdi*' here, you tell MAME to render video using older standard Windows graphics drawing calls. This is the slowest but most compatible option on older versions of Windows. Specifying '*d3d*' tells MAME to use Direct3D for rendering. This produces the highest quality output and enables all rendering options. It is recommended if you have a semi-recent (2002+) video card or onboard Intel video of the HD3000 line or better. The final option 'none' displays no windows and does no drawing. This is primarily present for doing CPU benchmarks without the overhead of the video system. The default is *d3d*.

**-numscreens** *<count>*

	Tells MAME how many output windows to create. For most games, a single output window is all you need, but some games originally used multiple screens (*e.g. Darius, PlayChoice-10*). Each screen (up to 4) has its own independent settings for physical monitor, aspect ratio, resolution, and view, which can be set using the options below. The default is *1*.

**-[no]window** / **-[no]w**

	Run MAME in either a window or full screen. The default is OFF (*-nowindow*).

**-[no]maximize** / **-[no]max**

	Controls initial window size in windowed mode. If it is set on, the window will initially be set to the maximum supported size when you start MAME. If it is turned off, the window will start out at the smallest supported size. This option only has an effect when the -window option is used. The default is ON (*-maximize*).

**-[no]keepaspect** / **-[no]ka**

	Enables aspect ratio enforcement. When this option is on, the game's proper aspect ratio (generally 4:3 or 3:4) is enforced, so you get the game looking like it should. When running in a window with this option on, you can only resize the window to the proper aspect ratio, unless you are holding down the CONTROL key. By turning the option off, the aspect ratio is allowed to float. In full screen mode, this means that all games will stretch to the full screen size (even vertical games). In window mode, it means that you can freely resize the window without any constraints. The default is ON (*-keepaspect*).

	The MAME team heavily suggests you leave this at default. Stretching games beyond their original aspect ratio will mangle the appearance of the game in ways that no filtering or HLSL can repair.

**-prescale** *<amount>*

	Controls the size of the screen images when they are passed off to the graphics system for scaling. At the minimum setting of 1, the screen is rendered at its original resolution before being scaled. At higher settings, the screen is expanded by a factor of *<amount>* before being scaled. With **-video d3d**, this produces a less blurry image at the expense of some speed. The default is *1*. 

**-[no]waitvsync**

	Waits for the refresh period on your computer's monitor to finish before starting to draw video to your screen. If this option is off, MAME will just draw to the screen at any old time, even in the middle of a refresh cycle. This can cause "tearing" artifacts, where the top portion of the screen is out of sync with the bottom portion. Tearing is not noticeable on all games, and some people hate it more than others. However, if you turn this option on, you will waste more of your CPU cycles waiting for the proper time to draw, so you will see a performance hit. You should only need to turn this on in windowed mode. In full screen mode, it is only needed if **-triplebuffer** does not remove the tearing, in which case you should use **-notriplebuffer -waitvsync**. Note that this option does not work with **-video gdi** mode. The default is OFF (*-nowaitvsync*).

**-[no]syncrefresh**

	Enables speed throttling only to the refresh of your monitor. This means that the game's actual refresh rate is ignored; however, the sound code still attempts to keep up with the game's original refresh rate, so you may encounter sound problems. This option is intended mainly for those who have tweaked their video card's settings to provide carefully matched refresh rate options. Note that this option does not work with -video gdi mode. The default is OFF (*-nosyncrefresh*).


Direct3D-specific options
-------------------------

**-[no]filter** / **-[no]d3dfilter** / **-[no]flt**

	Enable bilinear filtering on the game screen graphics. When disabled, point filtering is applied, which is crisper but leads to scaling artifacts. If you don't like the filtered look, you are probably better off increasing the *-prescale* value rather than turning off filtering altogether. The default is ON (*-filter*).



Per-window options
------------------

| **-screen** *<display>*
| **-screen0** *<display>*
| **-screen1** *<display>*
| **-screen2** *<display>*
| **-screen3** *<display>*
|
|	Specifies which physical monitor on your system you wish to have each window use by default. In order to use multiple windows, you must have increased the value of the **-numscreens** option. The name of each display in your system can be determined by running MAME with the -verbose option. The display names are typically in the format of: *\\\\.\\DISPLAYn*, where 'n' is a number from 1 to the number of connected monitors. The default value for these options is '*auto*', which means that the first window is placed on the first display, the second window on the second display, etc.
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
|	Specifies an exact resolution to run in. In full screen mode, MAME will try to use the specific resolution you request. The width and height are required; the refresh rate is optional. If omitted or set to 0, MAME will determine the mode automatically. For example, **-resolution 640x480** will force 640x480 resolution, but MAME is free to choose the refresh rate. Similarly, **-resolution 0x0@60** will force a 60Hz refresh rate, but allows MAME to choose the resolution. The string "*auto*" is also supported, and is equivalent to *0x0@0*. In window mode, this resolution is used as a maximum size for the window. This option requires the **-switchres** option as well in order to actually enable resolution switching with **-video d3d**. The default value for these options is '*auto*'.
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

**-[no]triplebuffer** / **-[no]tb**

	Enables or disables "triple buffering". Normally, MAME just draws directly to the screen, without any fancy buffering. But with this option enabled, MAME creates three buffers to draw to, and cycles between them in order. It attempts to keep things flowing such that one buffer is currently displayed, the second buffer is waiting to be displayed, and the third buffer is being drawn to. **-triplebuffer** will override **-waitvsync**, if the buffer is successfully created. This option does not work with **-video gdi**. The default is OFF (*-notriplebuffer*).

**-[no]switchres**

	Enables resolution switching. This option is required for the **-resolution\*** options to switch resolutions in full screen mode. On modern video cards, there is little reason to switch resolutions unless you are trying to achieve the "exact" pixel resolutions of the original games, which requires significant tweaking. This option is also useful on LCD displays, since they run with a fixed resolution and switching resolutions on them is just silly. This option does not work with **-video gdi**. The default is OFF (*-noswitchres*).

**-full_screen_brightness** *<value>* / **-fsb** *<value>*

	Controls the brightness, or black level, of the entire display. The standard value is 1.0. Selecting lower values (down to 0.1) will produce a darkened display, while selecting higher values (up to 2.0) will give a brighter display. Note that not all video cards have hardware to support this option. This option does not work with **-video gdi**. The default is *1.0*.

**-full_screen_contrast** *<value>* / **-fsc** *<value>*

	Controls the contrast, or white level, of the entire display. The standard value is 1.0. Selecting lower values (down to 0.1) will produce a dimmer display, while selecting higher values (up to 2.0) will give a more saturated display. Note that not all video cards have hardware to support this option. This option does not work with **-video gdi**. The default is *1.0*.

**-full_screen_gamma** *<value>* / **-fsg** *<value>*

	Controls the gamma, which produces a potentially nonlinear black to white ramp, for the entire display. The standard value is 1.0, which gives a linear ramp from black to white. Selecting lower values (down to 0.1) will increase the nonlinearity toward black, while selecting higher values (up to 3.0) will push the nonlinearity toward white. Note that not all video cards have hardware to support this option. This option does not work with **-video gdi**. The default is *1.0.*



Sound options
-------------

**-sound** *<dsound|sdl|none>*

	Specifies which sound subsystem to use. '*none*' disables sound altogether. The default is *dsound*.


**-audio_latency** *<value>*

	This controls the amount of latency built into the audio streaming. By default MAME tries to keep the DirectSound audio buffer between 1/5 and 2/5 full. On some systems, this is pushing it too close to the edge, and you get poor sound sometimes. The latency parameter controls the lower threshold. The default is *1* (meaning lower=1/5 and upper=2/5). Set it to 2 (**-audio_latency 2**) to keep the sound buffer between 2/5 and 3/5 full. If you crank it up to 4, you can *definitely* notice audio lag.



Input device options
--------------------

**-[no]dual_lightgun** / **-[no]dual**

	Controls whether or not MAME attempts to track two lightguns connected at the same time. This option requires -lightgun. This option is a hack for supporting certain older dual lightgun setups. If you have multiple lightguns connected, you will probably just need to enable -mouse and configure each lightgun independently. The default is *OFF* (*-nodual_lightgun*).