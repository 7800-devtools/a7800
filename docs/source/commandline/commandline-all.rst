.. _universal-command-line:

Universal Commandline Options
=============================


This section contains configuration options that are applicable to *all* MAME sub-builds (both SDL and Windows native).

Core commands
-------------

**-help** / **-h** / **-?**

	Displays current MAME version and copyright notice.

**-validate** / **-valid**

	Performs internal validation on every driver in the system. Run this
	before submitting changes to ensure that you haven't violated any of
	the core system rules.



Configuration commands
----------------------

**-createconfig** / **-cc**

	Creates the default mame.ini file. All the configuration options
	(not commands) described below can be permanently changed by editing
	this configuration file.

**-showconfig** / **-sc**

	Displays the current configuration settings. If you route this to a
	file, you can use it as an INI file. For example, the command:

		**mame -showconfig >mame.ini**

	is equivalent to **-createconfig**.

**-showusage** / **-su**

	Displays a summary of all the command line options. For options that
	are not mentioned here, the short summary given by "mame -showusage"
	is usually sufficient.



Frontend commands
-----------------

Note: By default, all the '**-list**' commands below write info to the screen. If you wish to write the info to a textfile instead, add this to the end of your command:

  > filename

...where 'filename' is the textfile's path and name (e.g., c:\\mame\\list.txt).


**-listxml** / **-lx** [*<gamename|wildcard>*]

	List comprehensive details for all of the supported games. The output is quite long, so it is usually better to redirect this into a file. The output is in XML format. By default all games are listed; however, you can limit this list by specifying a driver name or wildcard after the -listxml command.

**-listfull** / **-ll** [*<gamename|wildcard>*]

	Displays a list of game driver names and descriptions. By default all games are listed; however, you can limit this list by specifying a driver name or wildcard after the **-listfull** command.

**-listsource** / **-ls** [<*gamename|wildcard>*]

	Displays a list of drivers and the names of the source files their game drivers live in. Useful for finding which driver a game runs on in order to fix bugs. By default all games are listed; however, you	can limit this list by specifying a driver name or wildcard after the **-listsource** command.

**-listclones** / **-lc** [<*gamename|wildcard*>]

	Displays a list of clones. By default all clones are listed; however, you can limit this list by specifying a driver name or wildcard after the **-listsource** command.

**-listbrothers** / **-lb** [<*gamename|wildcard*>]

	Displays a list of '*brothers*', or rather, other sets which are located in the same sourcefile as the gamename searched for.

**-listcrc** [<*gamename|wildcard*>]

	Displays a full list of CRCs of all ROM images referenced by all drivers within MAME.

**-listroms** [<*gamename|wildcard*>]

	Displays a list of ROM images referenced by the specified game.


**-listsamples** [<*gamename|wildcard*>]

	Displays a list of samples referenced by the specified game.

**-verifyroms** [<*gamename|wildcard*>]

	Checks for invalid or missing ROM images. By default all drivers that have valid ZIP files or directories in the rompath are verified; however, you can limit this list by specifying a driver name or wildcard after the **-verifyroms** command.

**-verifysamples** [<*gamename|wildcard*>]

	Checks for invalid or missing samples. By default all drivers that have valid ZIP files or directories in the samplepath are verified;	however, you can limit this list by specifying a driver name or wildcard after the **-verifyroms** command.

**-romident** [*path\\to\\romstocheck.zip*]

	Attempts to identify ROM files, if they are known to MAME, in the specified .zip file or directory. This command can be used to try and identify ROM sets taken from unknown boards. On exit, the errorlevel is returned as one of the following:

		* 0: means all files were identified
		* 7: means all files were identified except for 1 or more "non-ROM" files
		* 8: means some files were identified
		* 9: means no files were identified

**-listdevices** / **-ld** [<*gamename|wildcard*>]

        Displays a list of all devices known to be hooked up to a game.  The ":" is considered the game itself with the devices list being attached to give the user a better understanding of what the emulation is using.

**-listslots** [<*gamename|wildcard*>]

        Show available slots and options for each slot (if available).  Primarily used for MAME to allow control over internal plug-in cards, much like PCs needing video, sound and other expansion cards.
		
        The slot name (e.g. **ctrl1**) can be used from the command line (**-ctrl1** in this case) 

**-listmedia** / **-lm** [<*gamename|wildcard*>]

        List available media that the chosen game or system allows to be used.  This includes media types (cartridge, cassette, diskette and more) as well as common file extentions which are supported.

**-listsoftware** [<*gamename|wildcard*>]

        Posts to screen all software lists which can be used by the entered gamename or system.  Note that this is simply a copy/paste of the .XML file which reside in the HASH folder which are allowed to be used.

**-verifysoftware** [<*gamename|wildcard*>]

	Checks for invalid or missing ROM images in your software lists. By default all drivers that have valid ZIP files or directories in the rompath are verified; however, you can limit this list by specifying a specific driver name or wildcard after the -verifysoftware command.

**-getsoftlist** [<*gamename|wildcard*>]

        Posts to screen a specific software list which matches with the gamename provided.

**-verifysoftlist** [softwarelistname]

	Checks a specified software list for missing ROM images if files exist for issued softwarelistname. By default, all drivers that have valid ZIP files or directories in the rompath are verified; however, you can limit this list by specifying a specific softwarelistname (without .XML) after the -verifysoftlist command.


.. _osd-commandline-options:

OSD related options
-------------------

**-uimodekey** [*keystring*]

	Key used to toggle emulated keyboard on and off.  Default setting is *SCRLOCK*.

**\-uifontprovider**

	Chooses provider for UI font:  win, none or auto.  Default setting is *AUTO*.

**\-menu**

	Enables menu bar at the top of the MAME window, if available by UI implementation.  Default is *OFF*

**\-keyboardprovider**

	Chooses how MAME will get keyboard input.
	
	On Windows, you can choose from: auto, rawinput, dinput, win32, or none
	On SDL, you can choose from: auto, sdl, none
	
	The default is *auto*. On Windows, auto will try rawinput with fallback to dinput. On SDL, auto will default to sdl.
	
**\-mouseprovider**

	Chooses how MAME will get mouse input.

	On Windows, you can choose from: auto, rawinput, dinput, win32, or none
	On SDL, you can choose from: auto, sdl, none
	
	The default is *auto*. On Windows, auto will try rawinput with fallback to dinput. On SDL, auto will default to sdl.

**\-lightgunprovider**

	Chooses how MAME will get light gun input.

	On Windows, you can choose from: auto, rawinput, win32, or none
	On SDL, you can choose from: auto, x11 or none

	The default is *auto*. On Windows, auto will try rawinput with fallback to win32, or none if it doesn't find any. On SDL/Linux, auto will default to x11, or none if it doesn't find any. On other SDL, auto will default to none.

**\-joystickprovider**

	Chooses how MAME will get joystick input.

	On Windows, you can choose from: auto, winhybrid, dinput, xinput, or none
	On SDL, you can choose from: auto, sdl, none
	
	The default is *auto*. On Windows, auto will default to dinput.
	
	Note that Microsoft X-Box 360 and X-Box One controllers will be happiest with *winhybrid* or *xinput*. The *winhybrid* option supports a mix of DirectInput and XInput controllers at the same time. On SDL, auto will default to sdl.



OSD CLI options
---------------

**\-listmidi**

    Create a list of available MIDI I/O devices for use with emulation.

**\-listnetwork**

	Create a list of available Network Adapters for use with emulation.



OSD output options
------------------

**\-output**

	Chooses how MAME will handle processing of output notifiers.
	
	you can choose from: auto, none, console or network
	
	Note that network port is fixed at 8000.



Configuration options
---------------------

**-[no]readconfig** / **-[no]rc**

	Enables or disables the reading of the config files. When enabled (which is the default), MAME reads the following config files in order:

		- mame.ini
		- <mymame>.ini   (i.e. if MAME was renamed mame060.exe, MAME parses mame060.ini here)
		- debug.ini      (if the debugger is enabled)
		- <driver>.ini   (based on the source filename of the driver)
		- vertical.ini   (for games with vertical monitor orientation)
		- horizont.ini   (for games with horizontal monitor orientation)
		- arcade.ini     (for games in source added with GAME() macro)
		- console.ini    (for games in source added with CONS() macro)
		- computer.ini   (for games in source added with COMP() macro)
		- othersys.ini   (for games in source added with SYST() macro)
		- vector.ini     (for vector games only)
		- <parent>.ini   (for clones only, may be called recursively)
		- <gamename>.ini

        (See :ref:`advanced-multi-CFG` for further details)

	The settings in the later INIs override those in the earlier INIs.
	So, for example, if you wanted to disable overlay effects in the vector games, you can create a vector.ini with the "effect none" line in it, and it will override whatever effect value you have in your mame.ini. The default is ON (*-readconfig*).



Core search path options
------------------------

**-rompath** / **-rp** *<path>*

	Specifies a list of paths within which to find ROM or hard disk images. Multiple paths can be specified by separating them with semicolons. The default is 'roms' (that is, a directory "roms" in the same directory as the MAME executable).

**-hashpath** *<path>*

	Specifies a list of paths within which to find Software List HASH files. Multiple paths can be specified by separating them with semicolons. The default is 'hash' (that is, a directory "roms" in the same directory as the MAME executable).

**-samplepath** / **-sp** *<path>*

	Specifies a list of paths within which to find sample files. Multiple paths can be specified by separating them with semicolons. The default is 'samples' (that is, a directory "samples" in the same directory as the MAME executable).

**-artpath** *<path>* / **-artwork_directory** *<path>*

	Specifies a list of paths within which to find artwork files. Multiple paths can be specified by separating them with semicolons. The default is 'artwork' (that is, a directory "artwork" in the same directory as the MAME executable).

**-ctrlrpath** / **-ctrlr_directory** *<path>*

	Specifies a list of paths within which to find controller-specific configuration files. Multiple paths can be specified by separating them with semicolons. The default is 'ctrlr' (that is, a directory "ctrlr" in the same directory as the MAME executable).

**-inipath** *<path>*

	Specifies a list of paths within which to find .INI files. Multiple paths can be specified by separating them with semicolons. The default is '.;ini' (that is, search in the current directory first, and then in the directory "ini" in the same directory as the MAME executable).

**-fontpath** *<path>*

	Specifies a list of paths within which to find .BDF font files. Multiple paths can be specified by separating them with semicolons. The default is '.' (that is, search in the same directory as the MAME executable).

**-cheatpath** *<path>*

    Specifies a list of paths within which to find .XML cheat files. Multiple paths can be specified by separating them with semicolons. The default is 'cheat' (that is, a folder called 'cheat' located in the same directory as the as the MAME executable).

**-crosshairpath** *<path>*

	Specifies a list of paths within which to find crosshair files. Multiple paths can be specified by separating them with semicolons. The default is 'crsshair' (that is, a directory "crsshair" in the same directory as the MAME executable).  If the Crosshair is set to default in the menu, MAME will look for gamename\\cross#.png and then cross#.png in the specified crsshairpath, where # is the player number.  Failing that, MAME will use built-in default crosshairs.

**-pluginspath** *<path>*

	Specifies a list of paths within which to find Lua plugins for MAME.

**-languagepath** *<path>*

	Specifies a list of paths within which to find language files for localized UI text.



Core Output Directory Options
-----------------------------

**-cfg_directory** *<path>*

	Specifies a single directory where configuration files are stored. Configuration files store user configurable settings that are read at startup and written when MAME exits. The default is 'cfg' (that is, a directory "cfg" in the same directory as the MAME executable). If this directory does not exist, it will be automatically created.

**-nvram_directory** *<path>*

	Specifies a single directory where NVRAM files are stored. NVRAM files store the contents of EEPROM and non-volatile RAM (NVRAM) for games which used this type of hardware. This data is read at startup and written when MAME exits. The default is 'nvram' (that is, a directory "nvram" in the same directory as the MAME executable). If this directory does not exist, it will be automatically created.

**-input_directory** *<path>*

	Specifies a single directory where input recording files are stored. Input recordings are created via the -record option and played back via the -playback option. The default is 'inp' (that is, a directory	"inp" in the same directory as the MAME executable). If this directory does not exist, it will be automatically created.

**-state_directory** *<path>*

	Specifies a single directory where save state files are stored. Save state files are read and written either upon user request, or when using the -autosave option. The default is 'sta' (that is, a directory "sta" in the same directory as the MAME executable). If this directory does not exist, it will be  automatically created.

**-snapshot_directory** *<path>*

	Specifies a single directory where screen snapshots are stored, when requested by the user. The default is 'snap' (that is, a directory "snap" in the same directory as the MAME executable). If this directory does not exist, it will be automatically created.

**-diff_directory** *<path>*

	Specifies a single directory where hard drive differencing files are stored. Hard drive differencing files store any data that is written back to a hard disk image, in order to preserve the original image. The differencing files are created at startup when a game with a hard disk image. The default is 'diff' (that is, a directory "diff" in the same directory as the MAME executable). If this directory does not exist, it will be automatically created.

**-comment_directory** *<path>*

	Specifies a single directory where debugger comment files are stored. Debugger comment files are written by the debugger when comments are added to the disassembly for a game. The default is 'comments' (that is, a directory "comments" in the same directory as the MAME executable). If this directory does not exist, it will be automatically created.



Core state/playback options
---------------------------

**-state** *<slot>*

	Immediately after starting the specified game, will cause the save state in the specified <slot> to be loaded.

**-[no]autosave**

	When enabled, automatically creates a save state file when exiting MAME and automatically attempts to reload it when later starting MAME with the same game. This only works for games that have explicitly enabled save state support in their driver. The default is OFF (-noautosave).

**-playback** / **-pb** *<filename>*

	Specifies a file from which to play back a series of game inputs. Thisfeature does not work reliably for all games, but can be used to watch a previously recorded game session from start to finish. In order to make things consistent, you should only record and playback with all configuration (.cfg), NVRAM (.nv), and memory card files deleted. The default is NULL (no playback).

**-exit_after_playback**

	Tells MAME to exit after finishing playback of the input file.

**-record** / **-rec** *<filename>*

	Specifies a file to record all input from a game session. This can be used to record a game session for later playback. This feature does not work reliably for all games, but can be used to watch a previously recorded game session from start to finish. In order to make things consistent, you should only record and playback with all configuration (.cfg), NVRAM (.nv), and memory card files deleted. The default is NULL (no recording).

**-record_timecode**

	Tells MAME to create a timecode file. It contains a line with elapsed times on each press of timecode shortcut key (*default is F12*). This option works only when recording mode is enabled (**-record** option). The file is saved in the *inp* folder. By default, no timecode file is saved.

**-mngwrite** *<filename>*

	Writes each video frame to the given <filename> in MNG format, producing an animation of the game session. Note that -mngwrite only writes video frames; it does not save any audio data. Use -wavwrite for that, and reassemble the audio/video using offline tools. The default is NULL (no recording).

**-aviwrite** *<filename>*

	Stream video and sound data to the given <filename> in AVI format, producing an animation of the game session complete with sound. The default is NULL (no recording).

**-wavwrite** *<filename>*

	Writes the final mixer output to the given <filename> in WAV format, producing an audio recording of the game session. The default is NULL (no recording).

**-snapname** *<name>*

	Describes how MAME should name files for snapshots. <name> is a string that provides a template that is used to generate a filename. 
	
	Three simple substitutions are provided: the / character represents the path separator on any target platform (even Windows); the string %g represents the driver name of the current game; and the string %i represents an incrementing index. If %i is omitted, then each snapshot taken will overwrite the previous one; otherwise, MAME will find the next empty value for %i and use that for a filename.
	
	The default is %g/%i, which creates a separate folder for each game, and names the snapshots under it starting with 0000 and increasing from there.  
	
	In addition to the above, for drivers using different media, like carts or floppy disks, you can also use the %d_[media] indicator.  Replace [media] with the media switch you want to use. 
	
	A few examples: if you use 'mame robby -snapname foo/%g%i' snapshots will be saved as 'snaps\\foo\\robby0000.png' , 'snaps\\foo\\robby0001.png' and so on; if you use 'mame nes -cart robby -snapname %g/%d_cart' snapshots will be saved as 'snaps\\nes\\robby.png' ; if you use 'mame c64 -flop1 robby -snapname %g/%d_flop1/%i' snapshots will be saved as 'snaps\\c64\\robby\\0000.png'.

**-snapsize** *<width>x<height>*

	Hard-codes the size for snapshots and movie recording. By default, MAME will create snapshots at the game's current resolution in raw pixels, and will create movies at the game's starting resolution in raw pixels. If you specify this option, then MAME will create both snapshots and movies at the size specified, and will bilinear filter the result. Note that this size does not automatically rotate if the game is vertically oriented. The default is '*auto*'.

**-snapview** *<viewname>*

	Specifies the view to use when rendering snapshots and movies. By default, both use a special 'internal' view, which renders a separate snapshot per screen or renders movies only of the first screen. By specifying this option, you can override this default behavior and select a single view that will apply to all snapshots and movies. Note that <viewname> does not need to be a perfect match; rather, it will select the first view whose name matches all the characters specified by <viewname>.
	
	For example, **-snapview native** will match the "Native (15:14)" view even though it is not a perfect match. <viewname> can also be 'auto', which selects the first view with all screens present. The default value is '*internal*'.

**-[no]snapbilinear**

	Specify if the snapshot or movie should have bilinear filtering	applied.  Shutting this off can make a difference in some performance while recording video to a file.  The default is ON (*-snapbilinear*).

**-statename** *<name>*

	Describes how MAME should store save state files, relative to the state_directory path. <name> is a string that provides a template that is used to generate a relative path.
	
	Two simple substitutions are provided: the / character represents the path separator on any target platform (even Windows); the string %g represents the driver name of the current game.
	
	The default is %g, which creates a separate folder for each game.  
	
	In addition to the above, for drivers using different media, like carts or floppy disks, you can also use the %d_[media] indicator.  Replace [media] with the media switch you want to use. 
	
	A few examples: if you use 'mame robby -statename foo/%g' save states will be stored inside 'sta\\foo\\robby\\' ; if you use 'mame nes -cart robby -statename %g/%d_cart' save states will be stored inside 'sta\\nes\\robby\\' ; if you use 'mame c64 -flop1 robby -statename %g/%d_flop1' save states will be stored inside 'sta\\c64\\robby\\'.

**-[no]burnin**

	Tracks brightness of the screen during play and at the end of emulation generates a PNG that can be used to simulate burn-in effects on other games. The resulting PNG is created such that the least used-areas of the screen are fully white (since burned-in areas are darker, all other areas of the screen must be lightened a touch). 

	The intention is that this PNG can be loaded via an artwork file with a low alpha (e.g, 0.1-0.2 seems to work well) and blended over the entire screen. The PNG files are saved in the snap directory under the gamename/burnin-<screen.name>.png. The default is OFF (*-noburnin*).



Core performance options
------------------------

**-[no]autoframeskip** / **-[no]afs**

	Automatically determines the frameskip level while you're playing the game, adjusting it constantly in a frantic attempt to keep the game running at full speed. Turning this on overrides the value you have set for -frameskip below. The default is OFF (*-noautoframeskip*).

**-frameskip** / **-fs** *<level>*

	Specifies the frameskip value. This is the number of frames out of every 12 to drop when running. For example, if you say -frameskip 2, then MAME will display 10 out of every 12 frames. By skipping those frames, you may be able to get full speed in a game that requires more horsepower than your computer has. The default value is **-frameskip 0**, which skips no frames.

**-seconds_to_run** / **-str** *<seconds>*

	This option can be used for benchmarking and automated testing. It tells MAME to stop execution after a fixed number of seconds. By combining this with a fixed set of other command line options, you can set up a consistent environment for benchmarking MAME performance. In addition, upon exit, the **-str** option will write a screenshot called *final.png* to the game's snapshot directory.

**-[no]throttle**

	Configures the default thottling setting. When throttling is on, MAME attempts to keep the game running at the game's intended speed. When throttling is off, MAME runs the game as fast as it can. Note that the fastest speed is more often than not limited by your graphics card, especially for older games. The default is ON (*-throttle*).

**-[no]sleep**

	Allows MAME to give time back to the system when running with -throttle. This allows other programs to have some CPU time, assuming that the game isn't taxing 100% of your CPU resources. This option can potentially cause hiccups in performance if other demanding programs are running. The default is ON (*-sleep*).

**-speed** *<factor>*

	Changes the way MAME throttles gameplay such that the game runs at some multiplier of the original speed. A <factor> of 1.0 means to run the game at its normal speed. A <factor> of 0.5 means run at half speed, and a <factor> of 2.0 means run at 2x speed. Note that changing this value affects sound playback as well, which will scale in pitch accordingly. The internal resolution of the fraction is two decimalplaces, so a value of 1.002 is the same as 1.0. The default is 1.0.

**-[no]refreshspeed** / **-[no]rs**

	Allows MAME to dynamically adjust the gameplay speed such that it does not exceed the slowest refresh rate for any targeted monitors in your system. Thus, if you have a 60Hz monitor and run a game that is actually designed to run at 60.6Hz, MAME will dynamically change the speed down to 99% in order to prevent sound hiccups or other undesirable side effects of running at a slower refresh rate. The default is OFF (*-norefreshspeed*).



Core rotation options
---------------------

| **-[no]rotate**
|
|	Rotate the game to match its normal state (horizontal/vertical). This ensures that both vertically and horizontally oriented games show up correctly without the need to rotate your monitor. If you want to keep the game displaying 'raw' on the screen the way it would have in the arcade, turn this option OFF. The default is ON (*-rotate*).
|
|


| **-[no]ror**
| **-[no]rol**
| 
|
|	Rotate the game screen to the right (clockwise) or left (counter-clockwise) relative to either its normal state (if **-rotate** is specified) or its native state (if **-norotate** is specified). The default for both of these options is OFF (*-noror -norol*).
|
|


| **-[no]autoror**
| **-[no]autorol**
| 
|
|	These options are designed for use with pivoting screens that only pivot in a single direction. If your screen only pivots clockwise, use -autorol to ensure that the game will fill the screen either horizontally or vertically in one of the directions you can handle. If your screen only pivots counter-clockwise, use **-autoror**.
|
|


| **-[no]flipx**
| **-[no]flipy**
| 
|
|	Flip (mirror) the game screen either horizontally (-flipx) or vertically (-flipy). The flips are applied after the -rotate and -ror/-rol options are applied. The default for both of these options is OFF (*-noflipx -noflipy*).
|
|


Core artwork options
--------------------

**-[no]artwork_crop** / **-[no]artcrop**

	Enable cropping of artwork to the game screen area only. This works best with -video gdi or -video d3d, and means that vertically oriented games running full screen can display their artwork to the left and right sides of the screen. This option can also be controlled via the Video Options menu in the user interface. The default is OFF (*-noartwork_crop*).

**-[no]use_backdrops** / **-[no]backdrop**

	Enables/disables the display of backdrops. The default is ON (*-use_backdrops*).

**-[no]use_overlays** / **-[no]overlay**

	Enables/disables the display of overlays. The default is ON (*-use_overlays*).

**-[no]use_bezels** / **-[no]bezels**

	Enables/disables the display of bezels. The default is ON (*-use_bezels*).

**-[no]use_cpanels** / **-[no]cpanels**

	Enables/disables the display of control panels. The default is ON (*-use_cpanels*).

**-[no]use_marquees** / **-[no]marquees**

	Enables/disables the display of marquees. The default is ON (*-use_marquees*).



Core screen options
-------------------

**-brightness** *<value>*

	Controls the default brightness, or black level, of the game screens. This option does not affect the artwork or other parts of the display. Using the MAME UI, you can individually set the brightness for each game screen; this option controls the initial value for all visible game screens. The standard value is 1.0. Selecting lower values (down to 0.1) will produce a darkened display, while selecting higher values (up to 2.0) will give a brighter display. The default is *1.0*.

**-contrast** *<value>*

	Controls the contrast, or white level, of the game screens. This option does not affect the artwork or other parts of the display. Using the MAME UI, you can individually set the contrast for each game screen; this option controls the initial value for all visible game screens. The standard value is 1.0. Selecting lower values (down to 0.1) will produce a dimmer display, while selecting higher values (up to 2.0) will give a more saturated display. The default is *1.0*.

**-gamma** *<value>*

	Controls the gamma, which produces a potentially nonlinear black to white ramp, for the game screens. This option does not affect the artwork or other parts of the display. Using the MAME UI, you can individually set the gamma for each game screen; this option controls the initial value for all visible game screens. The standard value is 1.0, which gives a linear ramp from black to white. Selecting lower 	values (down to 0.1) will increase the nonlinearity toward black, while selecting higher values (up to 3.0) will push the nonlinearity toward white. The default is *1.0*.

**-pause_brightness** *<value>*

	This controls the brightness level when MAME is paused. The default value is *0.65*.

**-effect** *<filename>*

	Specifies a single PNG file that is used as an overlay over any game screens in the video display. This PNG file is assumed to live in the root of one of the artpath directories. The pattern in the PNG file is repeated both horizontally and vertically to cover the entire game screen areas (but not any external artwork), and is rendered at the target resolution of the game image. For -video gdi and -video d3d modes, this means that one pixel in the PNG will map to one pixel on your output display. The RGB values of each pixel in the PNG are multiplied against the RGB values of the target screen. The default is '*none*', meaning no effect.



Core vector options
-------------------

**-[no]antialias** / **-[no]aa**

	Enables antialiased line rendering for vector games. The default is ON (*-antialias*).

**-beam** *<width>*

	Sets the width of the vectors. This is a scaling factor against the standard vector width. A value of 1.0 will keep the default vector line width. Smaller values will reduce the width, and larger values will increase the width. The default is *1.0*.

**-flicker** *<value>*

	Simulates a vector "flicker" effect, similar to a vector monitor that needs adjustment. This option requires a float argument in the range of 0.00 - 100.00 (0=none, 100=maximum). The default is *0*.



Core sound options
------------------

**-samplerate** *<value>* / **-sr** *<value>*

	Sets the audio sample rate. Smaller values (e.g. 11025) cause lower audio quality but faster emulation speed. Higher values (e.g. 48000) cause higher audio quality but slower emulation speed. The default is *48000*.

**-[no]samples**

	Use samples if available. The default is ON (*-samples*).

**-volume** / **-vol** *<value>*

	Sets the startup volume. It can later be changed with the user interface (see Keys section). The volume is an attenuation in dB: e.g., "**-volume -12**" will start with -12dB attenuation. The default is *0*.



Core input options
------------------

**-[no]coin_lockout** / **-[no]coinlock**

	Enables simulation of the "coin lockout" feature that is implemented on a number of game PCBs. It was up to the operator whether or not the coin lockout outputs were actually connected to the coin mechanisms. If this feature is enabled, then attempts to enter a coin while the lockout is active will fail and will display a popup message in the user interface (In debug mode). If this feature is disabled, the coin lockout signal will be ignored. The default is ON (*-coin_lockout*).

**-ctrlr** *<controller>*

	Enables support for special controllers. Configuration files are loaded from the ctrlrpath. They are in the same format as the .cfg files that are saved, but only control configuration data is read from the file. The default is NULL (no controller file).

**-[no]mouse**

	Controls whether or not MAME makes use of mouse controllers. When this is enabled, you will likely be unable to use your mouse for other purposes until you exit or pause the game. The default is OFF (*-nomouse*).

**-[no]joystick** / **-[no]joy**

	Controls whether or not MAME makes use of joystick/gamepad controllers. When this is enabled, MAME will ask DirectInput about which controllers are connected. The default is OFF (*-nojoystick*).

**-[no]lightgun** / **-[no]gun**

	Controls whether or not MAME makes use of lightgun controllers. Note that most lightguns map to the mouse, so using -lightgun and -mouse together may produce strange results. The default is OFF (*-nolightgun*).

**-[no]multikeyboard** / **-[no]multikey**

	Determines whether MAME differentiates between multiple keyboards. Some systems may report more than one keyboard; by default, the data from all of these keyboards is combined so that it looks like a single keyboard. Turning this option on will enable MAME to report keypresses	on different keyboards independently. The default is OFF (*-nomultikeyboard*).

**-[no]multimouse**

	Determines whether MAME differentiates between multiple mice. Some systems may report more than one mouse device; by default, the data from all of these mice is combined so that it looks like a single mouse. Turning this option on will enable MAME to report mouse movement and button presses on different mice independently. The default is OFF (*-nomultimouse*).

**-[no]steadykey** / **-[no]steady**

	Some games require two or more buttons to be pressed at exactly the same time to make special moves. Due to limitations in the keyboard hardware, it can be difficult or even impossible to accomplish that using the standard keyboard handling. This option selects a different handling that makes it easier to register simultaneous button presses, but has the disadvantage of making controls less responsive. The default is OFF (*-nosteadykey*)

**-[no]ui_active**

        Enable user interface on top of emulated keyboard (if present).  The default is OFF (*-noui_active*)

**-[no]offscreen_reload** / **-[no]reload**

	Controls whether or not MAME treats a second button input from a lightgun as a reload signal. In this case, MAME will report the gun's position as (0,MAX) with the trigger held, which is equivalent to an	offscreen reload. This is only needed for games that required you to shoot offscreen to reload, and then only if your gun does not support off screen reloads. The default is OFF (*-nooffscreen_reload*).

**-joystick_map** *<map>* / **-joymap** *<map>*

	Controls how joystick values map to digital joystick controls. MAME accepts all joystick input from the system as analog data. For true analog joysticks, this needs to be mapped down to the usual 4-way or 8-way digital joystick values. To do this, MAME divides the analog range into a 9x9 grid. It then takes the joystick axis position (for X and Y axes only), maps it to this grid, and then looks up a translation from a joystick map. This parameter allows you to specify the map. The default is 'auto', which means that a standard 8-way, 4-way, or 4-way diagonal map is selected automatically based on the input port configuration of the current game.

	Maps are defined as a string of numbers and characters. Since the grid is 9x9, there are a total of 81 characters necessary to define a	complete map. Below is an example map for an 8-way joystick:

		+-------------+---------------------------------------------------------+
		| | 777888999 |                                                         |
		| | 777888999 | | Note that the numeric digits correspond to the keys   |
		| | 777888999 | | on a numeric keypad. So '7' maps to up+left, '4' maps |
		| | 444555666 | | to left, '5' maps to neutral, etc. In addition to the |
		| | 444555666 | | numeric values, you can specify the character 's',    |
		| | 444555666 | | which means "sticky". In this case, the value of the  |
		| | 111222333 | | map is the same as it was the last time a non-sticky  |
		| | 111222333 | | value was read.                                       |
		| | 111222333 |                                                         |
		+-------------+---------------------------------------------------------+

	To specify the map for this parameter, you can specify a string of rows separated by a '.' (which indicates the end of a row), like so:

 +-------------------------------------------------------------------------------------------+
 | 777888999.777888999.777888999.444555666.444555666.444555666.111222333.111222333.111222333 |
 +-------------------------------------------------------------------------------------------+
 
	However, this can be reduced using several shorthands supported by the <map> parameter. If information about a row is missing, then it is assumed that any missing data in columns 5-9 are left/right symmetric with data in columns 0-4; and any missing data in colums 0-4 is assumed to be copies of the previous data. The same logic applies to missing rows, except that up/down symmetry is assumed.

	By using these shorthands, the 81 character map can be simply specified by this 11 character string: 7778...4445

	Looking at the first row, 7778 is only 4 characters long. The 5th entry can't use symmetry, so it is assumed to be equal to the previous character '8'. The 6th character is left/right symmetric with the 4th character, giving an '8'. The 7th character is left/right symmetric with the 3rd character, giving a '9' (which is '7' with left/right flipped). Eventually this gives the full 777888999 string of the row.

	The second and third rows are missing, so they are assumed to be identical to the first row. The fourth row decodes similarly to the first row, producing 444555666. The fifth row is missing so it is assumed to be the same as the fourth.

	The remaining three rows are also missing, so they are assumed to be the up/down mirrors of the first three rows, giving three final rows of 111222333.

**-joystick_deadzone** *<value>* / **-joy_deadzone** *<value>* / **-jdz** *<value>*

	If you play with an analog joystick, the center can drift a little. joystick_deadzone tells how far along an axis you must move before the axis starts to change. This option expects a float in the range of 0.0 to 1.0. Where 0 is the center of the joystick and 1 is the outer limit. The default is *0.3*.

**-joystick_saturation** *<value>* / **joy_saturation** *<value>* / **-jsat** *<value>*

	If you play with an analog joystick, the ends can drift a little, and may not match in the +/- directions. joystick_saturation tells how far along an axis movement change will be accepted before it reaches the maximum range. This option expects a float in the range of 0.0 to 1.0, where 0 is the center of the joystick and 1 is the outer limit. The default is *0.85*.

**\-natural**

        Allows user to specify whether or not to use a natural keyboard or not. This allows you to start your game or system in a 'native' mode, depending on your region, allowing compatability for non-"QWERTY" style keyboards. The default is OFF (*-nonatural*)

**-joystick_contradictory**

        Enable contradictory direction digital joystick input at the same time such as **Left and Right** or **Up and Down** at the same time.  The default is OFF (*-nojoystick_contradictory*)

**-coin_impulse** *[n]*

        Set coin impulse time based on n (n<0 disable impulse, n==0 obey driver, 0<n set time n).  Default is *0*.



Core input automatic enable options
-----------------------------------

**\-paddle_device**       enable (none|keyboard|mouse|lightgun|joystick) if a paddle control is present

**\-adstick_device**      enable (none|keyboard|mouse|lightgun|joystick) if an analog joystick control is present

**\-pedal_device**        enable (none|keyboard|mouse|lightgun|joystick) if a pedal control is present

**\-dial_device**         enable (none|keyboard|mouse|lightgun|joystick) if a dial control is present

**\-trackball_device**    enable (none|keyboard|mouse|lightgun|joystick) if a trackball control is present

**\-lightgun_device**     enable (none|keyboard|mouse|lightgun|joystick) if a lightgun control is present

**\-positional_device**   enable (none|keyboard|mouse|lightgun|joystick) if a positional control is present

**\-mouse_device**        enable (none|keyboard|mouse|lightgun|joystick) if a mouse control is present

	Each of these options controls autoenabling the mouse, joystick, or lightgun depending on the presence of a particular class of analog control for a particular game. For example, if you specify the option -paddle mouse, then any game that has a paddle control will automatically enable mouse controls just as if you had explicitly specified -mouse. Note that these controls override the values of -[no]mouse, -[no]joystick, etc.



Debugging options
-----------------

**-[no]verbose** / **-[no]v**

	Displays internal diagnostic information. This information is very useful for debugging problems with your configuration. IMPORTANT: when reporting bugs, please run with **mame -verbose** and include the resulting information. The default is OFF (*-noverbose*).

**-[no]oslog**

	Output error.log data to the system debugger. The default is OFF (*-nooslog*).

**-[no]log**

	Creates a file called error.log which contains all of the internal log messages generated by the MAME core and game drivers. The default is OFF (*-nolog*).

**-[no]debug**

	Activates the integrated debugger. By default, the debugger is entered by pressing the tilde (~) key during emulation. It is also entered immediately at startup. The default is OFF (*-nodebug*).

**-debugscript** *<filename>*

	Specifies a file that contains a list of debugger commands to execute immediately upon startup. The default is NULL (*no commands*).

**-[no]update_in_pause**

	Enables updating of the main screen bitmap while the game is paused. This means that the VIDEO_UPDATE callback will be called repeatedly during pause, which can be useful for debugging. The default is OFF (*-noupdate_in_pause*).


Core communication options
--------------------------

**-comm_localhost** *<string>*

	Local address to bind to.  This can be a traditional xxx.xxx.xxx.xxx address or a string containing a resolvable hostname.  The default is value is "*0.0.0.0*"

**-comm_localport** *<string>*

	Local port to bind to.  This can be any traditional communications port as an unsigned 16-bit integer (0-65535).  The default value is "*15122*".

**-comm_remotehost** *<string>*

	Remote address to connect to.  This can be a traditional xxx.xxx.xxx.xxx address or a string containing a resolvable hostname.  The default is value is "*0.0.0.0*"

**-comm_remoteport** *<string>*

	Remote port to connect to.  This can be any traditional communications port as an unsigned 16-bit integer (0-65535).  The default value is "*15122*".



Core misc options
-----------------

**-[no]drc**
	Enable DRC cpu core if available.  The default is ON (*-drc*).

**\-drc_use_c**

	Force DRC use the C code backend.  The default is OFF (*-nodrc_use_c*).

**\-drc_log_uml**

	Write DRC UML disassembly log.  The default is OFF (*-nodrc_log_uml*).

**\-drc_log_native**

	write DRC native disassembly log.  The default is OFF (*-nodrc_log_native*).

**-bios** *<biosname>*

	Specifies the specific BIOS to use with the current game, for game systems that make use of a BIOS. The **-listxml** output will list all of the possible BIOS names for a game. The default is '*default*'.

**-[no]cheat** / **-[no]c**

	Activates the cheat menu with autofire options and other tricks from the cheat database, if present. The default is OFF (*-nocheat*).

**-[no]skip_gameinfo**

	Forces MAME to skip displaying the game info screen. The default is OFF (*-noskip_gameinfo*).

**-uifont** *<fontname>*

	Specifies the name of a font file to use for the UI font. If this font cannot be found or cannot be loaded, the system will fall back to its built-in UI font. On some platforms 'fontname' can be a system font name (TTF) instead of a (BDF) font file. The default is '*default*' (use the OSD-determined default font).

**-ramsize** *[n]*

	Allows you to change the default RAM size (if supported by driver).

**\-confirm_quit**

	Display a Confirm Quit dialong to screen on exit, requiring one extra step to exit MAME.  The default is OFF (*-noconfirm_quit*).

**\-ui_mouse**

	Displays a mouse cursor when using the built-in UI for MAME.  The default is (*-noui_mouse*).

**-autoboot_command** *"<command>"*

	Command string to execute after machine boot (in quotes " ").  To issue a quote to the emulation, use """ in the string.  Using **\\n** will issue a create a new line, issuing what was typed prior as a command. 

	Example:  -autoboot_command "load """$""",8,1\\n"

**-autoboot_delay** *[n]*

    Timer delay (in seconds) to trigger command execution on autoboot.

**-autoboot_script** / **-script** *[filename.lua]*

    File containing scripting to execute after machine boot.

**-language** *<language>*

	Specify a localization language found in the *languagepath* tree.
